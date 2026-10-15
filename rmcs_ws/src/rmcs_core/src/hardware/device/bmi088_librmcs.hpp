#pragma once

#include "filter/ekf_imu.hpp"
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <numbers>
#include <rclcpp/logging.hpp>
#include <utility>

namespace rmcs_core::hardware::device {

class LibrmcsBmi088 {
public:
    explicit LibrmcsBmi088(
        double process_noise1 = 10.0f, double process_noise2 = 0.001f,
        double measurement_noise = 1000000.0f, double q0 = 1, double q1 = 0, double q2 = 0,
        double q3 = 0)
        : imu_filter_(process_noise1, process_noise2, measurement_noise)
        , q0_(q0)
        , q1_(q1)
        , q2_(q2)
        , q3_(q3) {
        last_update_time_ = std::chrono::steady_clock::now();
    };

    void set_coordinate_mapping(
        std::function<std::tuple<double, double, double>(double, double, double)>
            mapping_function) {
        coordinate_mapping_function_ = std::move(mapping_function);
    }

    // Extra gyro bias applied *after* the coordinate mapping, i.e. in the body
    // frame and in rad/s.  The integer imu_bias_* parameters can only subtract
    // whole LSB (1 LSB = 0.0613 °/s = 3.7 °/min), which is far too coarse: the
    // 2026-10-13 measurements left a residual z bias of 0.147 °/s, and that
    // residual makes the yaw loop rotate the physical gimbal by 8.8 °/min.
    // Calibrate it by reading the mean of /debug/top_board/imu_gyro_* while the
    // board is standing still.
    void set_gyroscope_fine_bias(double x, double y, double z) {
        gyro_fine_bias_x_ = x;
        gyro_fine_bias_y_ = y;
        gyro_fine_bias_z_ = z;
    }

    // The gyro bias is only observable through the accelerometer, so the EKF needs
    // seconds to learn it; averaging the first frames while the robot stands still
    // settles it up front.  Pass 0 to disable the calibration.
    void set_startup_calibration_samples(std::size_t samples) {
        startup_calibration_samples_ = samples;
    }

    void store_accelerometer_status(int16_t x, int16_t y, int16_t z) {
        accelerometer_data_.store({x, y, z}, std::memory_order::relaxed);
    }

    void store_gyroscope_status(int16_t x, int16_t y, int16_t z) {
        gyroscope_data_.store({x, y, z}, std::memory_order::relaxed);
    }

    void update_status() {
        auto acc = accelerometer_data_.load(std::memory_order::relaxed);
        auto gyro = gyroscope_data_.load(std::memory_order::relaxed);

        auto solve_acc = [](int16_t value) { return value / 32767.0 * 6.0; };
        auto solve_gyro = [](int16_t value) {
            return value / 32767.0 * 2000.0 / 180.0 * std::numbers::pi;
        };

        gx_ = solve_gyro(gyro.x), gy_ = solve_gyro(gyro.y), gz_ = solve_gyro(gyro.z);
        ax_ = solve_acc(acc.x), ay_ = solve_acc(acc.y), az_ = solve_acc(acc.z);

        if (coordinate_mapping_function_) {
            std::tie(gx_, gy_, gz_) = coordinate_mapping_function_(gx_, gy_, gz_);
            std::tie(ax_, ay_, az_) = coordinate_mapping_function_(ax_, ay_, az_);
        }

        gx_ -= gyro_fine_bias_x_;
        gy_ -= gyro_fine_bias_y_;
        gz_ -= gyro_fine_bias_z_;

        update_startup_calibration();
        ekf_ahrs_update_imu(ax_, ay_, az_, gx_, gy_, gz_);
    }

    double ax() const { return ax_; }
    double ay() const { return ay_; }
    double az() const { return az_; }

    double gx() const { return gx_; }
    double gy() const { return gy_; }
    double gz() const { return gz_; }

    double& q0() { return q0_; }
    double& q1() { return q1_; }
    double& q2() { return q2_; }
    double& q3() { return q3_; }

private:
    void update_startup_calibration() {
        if (calibration_done_) [[likely]]
            return;
        if (startup_calibration_samples_ == 0) {
            calibration_done_ = true;
            return;
        }

        const double gyro[3] = {gx_, gy_, gz_};
        const double accel[3] = {ax_, ay_, az_};
        for (int i = 0; i < 3; ++i) {
            calibration_sum_[i] += gyro[i];
            calibration_square_sum_[i] += gyro[i] * gyro[i];
            calibration_accel_sum_[i] += accel[i];
        }
        if (++calibration_count_ < startup_calibration_samples_)
            return;

        const double count = static_cast<double>(calibration_count_);
        double mean[3], deviation[3];
        bool still = true;
        for (int i = 0; i < 3; ++i) {
            mean[i] = calibration_sum_[i] / count;
            const double variance =
                std::max(calibration_square_sum_[i] / count - mean[i] * mean[i], 0.0);
            deviation[i] = std::sqrt(variance);
            still = still && deviation[i] < kMaxCalibrationDeviation
                 && std::abs(mean[i]) < kMaxCalibrationMean;
        }
        const double mean_accel[3] = {
            calibration_accel_sum_[0] / count, calibration_accel_sum_[1] / count,
            calibration_accel_sum_[2] / count};
        const double accel_norm = std::sqrt(
            mean_accel[0] * mean_accel[0] + mean_accel[1] * mean_accel[1]
            + mean_accel[2] * mean_accel[2]);
        still =
            still && accel_norm > kMinCalibrationAccelNorm && accel_norm < kMaxCalibrationAccelNorm;

        if (!still && ++calibration_attempts_ < kMaxCalibrationAttempts) {
            // The robot was moving during the window, try another one.
            calibration_count_ = 0;
            calibration_sum_[0] = calibration_sum_[1] = calibration_sum_[2] = 0.0;
            calibration_square_sum_[0] = calibration_square_sum_[1] = calibration_square_sum_[2] =
                0.0;
            calibration_accel_sum_[0] = calibration_accel_sum_[1] = calibration_accel_sum_[2] = 0.0;
            return;
        }

        calibration_done_ = true;
        if (!still) {
            RCLCPP_WARN(
                rclcpp::get_logger("HW_Diag"),
                "Gyro startup calibration skipped (robot still moving), keeping the bias at 0.");
            return;
        }

        // z is not an EKF state (update() forces it back to 0 every tick), so only
        // x/y can be seeded; imu_bias_fine_z covers the z residual instead.
        const double roll = std::atan2(mean_accel[1], mean_accel[2]);
        const double pitch = std::atan2(-mean_accel[0], std::hypot(mean_accel[1], mean_accel[2]));
        imu_filter_.set_initial_gyro_bias(mean[0], mean[1], 0.0);
        imu_filter_.set_initial_attitude(roll, pitch);
        RCLCPP_INFO(
            rclcpp::get_logger("HW_Diag"),
            "Gyro startup calibration: bias (rad/s) x %.5f y %.5f z %.5f, attitude roll %.2f "
            "pitch %.2f deg, over %.3f s%s",
            mean[0], mean[1], mean[2], roll * 180.0 / std::numbers::pi,
            pitch * 180.0 / std::numbers::pi,
            static_cast<double>(startup_calibration_samples_) / 1000.0,
            calibration_attempts_ > 0 ? " (after retries)" : "");
    }

    static constexpr double kMaxCalibrationDeviation = 0.03; // rad/s
    static constexpr double kMaxCalibrationMean = 0.02;      // rad/s
    static constexpr double kMinCalibrationAccelNorm = 0.9;  // g
    static constexpr double kMaxCalibrationAccelNorm = 1.1;  // g
    static constexpr std::size_t kMaxCalibrationAttempts = 10;

    void ekf_ahrs_update_imu(double ax, double ay, double az, double gx, double gy, double gz) {
        auto now = std::chrono::steady_clock::now();
        float dt = static_cast<float>(std::chrono::duration_cast<std::chrono::microseconds>(
                                          now - last_update_time_)
                                          .count())
                 / 1000000.0f;
        imu_filter_.update(gx, gy, gz, ax, ay, az, dt);
        last_update_time_ = now;

        const auto& state = imu_filter_.get_state();
        q0_ = state.q[0];
        q1_ = state.q[1];
        q2_ = state.q[2];
        q3_ = state.q[3];
    }

    filter::EkfImu imu_filter_;
    std::chrono::steady_clock::time_point last_update_time_;

    struct alignas(8) ImuData {
        int16_t x, y, z;
    };
    std::atomic<ImuData> accelerometer_data_, gyroscope_data_;
    static_assert(std::atomic<ImuData>::is_always_lock_free);

    double ax_ = 0.0, ay_ = 0.0, az_ = 0.0, gx_ = 0.0, gy_ = 0.0, gz_ = 0.0;
    double gyro_fine_bias_x_ = 0.0, gyro_fine_bias_y_ = 0.0, gyro_fine_bias_z_ = 0.0;

    std::size_t startup_calibration_samples_ = 1000;
    std::size_t calibration_count_ = 0;
    std::size_t calibration_attempts_ = 0;
    bool calibration_done_ = false;
    double calibration_sum_[3] = {0.0, 0.0, 0.0};
    double calibration_square_sum_[3] = {0.0, 0.0, 0.0};
    double calibration_accel_sum_[3] = {0.0, 0.0, 0.0};

    std::function<std::tuple<double, double, double>(double, double, double)>
        coordinate_mapping_function_;

    // Quaternion of sensor frame relative to auxiliary frame
    double q0_, q1_, q2_, q3_;
};

} // namespace rmcs_core::hardware::device
