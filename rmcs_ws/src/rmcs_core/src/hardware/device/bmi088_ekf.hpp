#pragma once

#include <eigen3/Eigen/Geometry>
#include <rclcpp/logging.hpp>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <numbers>
#include <optional>
#include <utility>

#include <rmcs_msgs/board_clock.hpp>
#include <rmcs_msgs/imu_snapshot.hpp>

#include "filter/imu_ekf.hpp"

namespace rmcs_core::hardware::device {

// Time constant of the first order low pass filter that is applied to the
// accelerator before it is fed to the attitude filter.  A short low pass keeps
// flying bullets and chassis vibration out of the gravity direction estimate.
inline constexpr double accel_low_pass_time_constant = 0.01; // s

class Bmi088Ekf {
public:
    struct Config {
        Eigen::Matrix3d body_to_sensor = Eigen::Matrix3d::Identity();
    };

    using TimePoint = rmcs_msgs::BoardClock::time_point;
    using Snapshot = rmcs_msgs::ImuSnapshot;

    explicit Bmi088Ekf(Config config)
        : config_(std::move(config)) {}

    Bmi088Ekf()
        : Bmi088Ekf(Config{}) {}

    void push_accelerometer_sample(int16_t x, int16_t y, int16_t z, TimePoint sample_time) {
        const Eigen::Vector3d raw_accel_g =
            config_.body_to_sensor.transpose() * convert_accelerometer(x, y, z);
        const Eigen::Vector3d accel_g = low_pass_accelerometer(raw_accel_g, sample_time);

        if (!calibration_done_) {
            calibration_accel_sum_ += raw_accel_g;
            ++calibration_accel_count_;
        }

        if (!initialized_) {
            if (ekf_.reset_from_accel(accel_g)) {
                apply_calibrated_bias();
                ekf_state_time_ = sample_time;
                latest_snapshot_ =
                    Snapshot{ekf_.quaternion(), Eigen::Vector3d::Zero(), ekf_state_time_};
                const std::scoped_lock lock{mutex_};
                initialized_ = true;
            }
            return;
        }

        if (sample_time < ekf_state_time_)
            return;
        if (pending_accel_sample_ && sample_time < pending_accel_sample_->sample_time)
            return;
        pending_accel_sample_ = AccelSample{accel_g, sample_time};
    }

    std::optional<Snapshot>
        try_update_with_gyroscope_sample(int16_t x, int16_t y, int16_t z, TimePoint sample_time) {
        const Eigen::Vector3d gyro_rad_per_sec =
            config_.body_to_sensor.transpose() * convert_gyroscope(x, y, z);

        accumulate_calibration_gyro(gyro_rad_per_sec);

        if (!initialized_)
            return std::nullopt;
        if (sample_time < ekf_state_time_)
            return std::nullopt;

        if (is_gyro_saturated(gyro_rad_per_sec))
            ekf_.inflate_attitude_uncertainty_to_initial();

        while (pending_accel_sample_) {
            const AccelSample accel_sample = *pending_accel_sample_;
            if (accel_sample.sample_time < ekf_state_time_
                || accel_sample.sample_time > sample_time)
                break;

            const double accel_dt =
                std::chrono::duration<double>{accel_sample.sample_time - ekf_state_time_}.count();
            if (!ekf_.predict(gyro_rad_per_sec, accel_dt))
                break;
            ekf_state_time_ = accel_sample.sample_time;

            const std::optional<filter::ImuEkf::AccelCorrection> correction =
                ekf_.prepare_correction(accel_sample.accel_g);
            if (!correction || correction->chi_square() >= kChiSquareTestThreshold)
                break;
            if (!ekf_.correct(*correction))
                break;

            pending_accel_sample_.reset();
            break;
        }

        // librmcs can deliver a burst of backlogged samples after a reconnect.
        // Integrating such a burst would let the filter drift away for as long
        // as the backlog lasts, so drop the frame instead and resynchronize.
        // TODO: replace this with a proper timebase reset once the burst can be
        // detected at the board layer.
        const double frame_dt =
            std::chrono::duration<double>{sample_time - ekf_state_time_}.count();
        if (frame_dt > kMaxIntegrationStepSeconds) {
            ekf_state_time_ = sample_time;
            return std::nullopt;
        }

        if (!ekf_.predict(gyro_rad_per_sec, frame_dt))
            return std::nullopt;
        ekf_state_time_ = sample_time;

        Snapshot snapshot =
            Snapshot{ekf_.quaternion(), bias_corrected_gyro(gyro_rad_per_sec), ekf_state_time_};
        {
            const std::scoped_lock lock{mutex_};
            latest_snapshot_ = snapshot;
        }
        return snapshot;
    }

    [[nodiscard]] bool initialized() const noexcept {
        const std::scoped_lock lock{mutex_};
        return initialized_;
    }

    [[nodiscard]] std::optional<Snapshot> snapshot() const noexcept {
        const std::scoped_lock lock{mutex_};
        if (!initialized_)
            return std::nullopt;
        return latest_snapshot_;
    }

private:
    struct AccelSample {
        Eigen::Vector3d accel_g;
        TimePoint sample_time;
    };

    static constexpr std::size_t kCalibrationSamples = 1000;
    static constexpr std::size_t kMaxCalibrationAttempts = 10;

    // 1 LSB of the BMI088 gyroscope is 0.0613 °/s, so the integer imu_bias_*
    // parameters can only cancel whole LSBs and always leave a residual that is
    // large enough to visibly rotate the yaw loop.  Estimate the fractional
    // part on startup instead, while the robot is guaranteed to be standing
    // still, and keep the z residual (the attitude filter cannot observe it).
    static constexpr double kMaxCalibrationDeviation = 0.03; // rad/s
    static constexpr double kMaxCalibrationMean = 0.02;      // rad/s
    static constexpr double kMinCalibrationAccelNorm = 0.9;  // g
    static constexpr double kMaxCalibrationAccelNorm = 1.1;  // g

    static constexpr double kChiSquareTestThreshold = 3.0;
    static constexpr double kMaxIntegrationStepSeconds = 1.0 / 1000.0;

    [[nodiscard]] Eigen::Vector3d
        low_pass_accelerometer(const Eigen::Vector3d& accel_g, TimePoint sample_time) {
        if (!last_accel_time_.has_value() || sample_time < *last_accel_time_) {
            accel_low_pass_state_ = accel_g;
            last_accel_time_ = sample_time;
            return accel_low_pass_state_;
        }

        const double dt = std::chrono::duration<double>{sample_time - *last_accel_time_}.count();
        last_accel_time_ = sample_time;
        if (!(dt > 0.0) || !std::isfinite(dt))
            return accel_low_pass_state_;

        const double alpha = dt / (dt + accel_low_pass_time_constant);
        accel_low_pass_state_ += alpha * (accel_g - accel_low_pass_state_);
        return accel_low_pass_state_;
    }

    void accumulate_calibration_gyro(const Eigen::Vector3d& gyro_rad_per_sec) {
        if (calibration_done_)
            return;

        calibration_gyro_sum_ += gyro_rad_per_sec;
        calibration_gyro_square_sum_ += gyro_rad_per_sec.cwiseProduct(gyro_rad_per_sec);
        ++calibration_gyro_count_;

        if (calibration_gyro_count_ >= kCalibrationSamples)
            finish_calibration();
    }

    void finish_calibration() {
        const double inverse_count = 1.0 / static_cast<double>(calibration_gyro_count_);
        const Eigen::Vector3d gyro_mean = calibration_gyro_sum_ * inverse_count;
        const Eigen::Vector3d gyro_variance =
            (calibration_gyro_square_sum_ * inverse_count - gyro_mean.cwiseProduct(gyro_mean))
                .cwiseMax(0.0);
        const Eigen::Vector3d gyro_deviation = gyro_variance.cwiseSqrt();

        const bool has_accel = calibration_accel_count_ > 0;
        Eigen::Vector3d accel_mean = Eigen::Vector3d::Zero();
        if (has_accel)
            accel_mean = calibration_accel_sum_ / static_cast<double>(calibration_accel_count_);
        const double accel_norm = accel_mean.norm();

        const bool still = has_accel && (gyro_deviation.array() < kMaxCalibrationDeviation).all()
                        && (gyro_mean.cwiseAbs().array() < kMaxCalibrationMean).all()
                        && accel_norm > kMinCalibrationAccelNorm
                        && accel_norm < kMaxCalibrationAccelNorm;

        if (!still && ++calibration_attempt_ < kMaxCalibrationAttempts) {
            reset_calibration_accumulators();
            return;
        }

        calibration_done_ = true;
        if (!still) {
            RCLCPP_WARN(
                rclcpp::get_logger("HW_Diag"),
                "Gyro startup calibration skipped (robot still moving), keeping the bias at 0.");
            return;
        }

        calibrated_gyro_bias_ = gyro_mean;
        if (initialized_ && ekf_.reset_from_accel(accel_mean, 0.0)) {
            ekf_.set_initial_gyro_bias(gyro_mean(0), gyro_mean(1));
            gyro_bias_z_ = gyro_mean(2);
        }

        RCLCPP_INFO(
            rclcpp::get_logger("HW_Diag"),
            "Gyro startup calibration: bias (%.5f, %.5f, %.5f) rad/s after %zu attempt(s).",
            gyro_mean(0), gyro_mean(1), gyro_mean(2), calibration_attempt_ + 1);
    }

    void reset_calibration_accumulators() {
        calibration_gyro_sum_.setZero();
        calibration_gyro_square_sum_.setZero();
        calibration_accel_sum_.setZero();
        calibration_gyro_count_ = 0;
        calibration_accel_count_ = 0;
    }

    void apply_calibrated_bias() {
        if (!calibrated_gyro_bias_.has_value())
            return;

        const Eigen::Vector3d& bias = *calibrated_gyro_bias_;
        ekf_.set_initial_gyro_bias(bias(0), bias(1));
        gyro_bias_z_ = bias(2);
    }

    [[nodiscard]] Eigen::Vector3d
        bias_corrected_gyro(const Eigen::Vector3d& gyro_rad_per_sec) const {
        const Eigen::Vector2d ekf_bias = ekf_.gyro_bias();
        return {
            gyro_rad_per_sec.x() - ekf_bias.x(), gyro_rad_per_sec.y() - ekf_bias.y(),
            gyro_rad_per_sec.z() - gyro_bias_z_};
    }

    [[nodiscard]] static Eigen::Vector3d convert_accelerometer(int16_t x, int16_t y, int16_t z) {
        return Eigen::Vector3d{
                   static_cast<double>(x), static_cast<double>(y), static_cast<double>(z)}
             / 32767.0 * 6.0;
    }

    [[nodiscard]] static Eigen::Vector3d convert_gyroscope(int16_t x, int16_t y, int16_t z) {
        return Eigen::Vector3d{
                   static_cast<double>(x), static_cast<double>(y), static_cast<double>(z)}
             / 32767.0 * 2000.0 / 180.0 * std::numbers::pi;
    }

    [[nodiscard]] static bool is_gyro_saturated(const Eigen::Vector3d& gyro_rad_per_sec) {
        constexpr double kGyroscopeSaturationThreshold = 0.98 * 2000.0 / 180.0 * std::numbers::pi;
        return gyro_rad_per_sec.cwiseAbs().maxCoeff() >= kGyroscopeSaturationThreshold;
    }

    const Config config_;

    mutable std::mutex mutex_;
    bool initialized_ = false;
    filter::ImuEkf ekf_;
    TimePoint ekf_state_time_;
    std::optional<AccelSample> pending_accel_sample_;
    Snapshot latest_snapshot_;

    bool calibration_done_ = false;
    std::size_t calibration_attempt_ = 0;
    std::size_t calibration_gyro_count_ = 0;
    std::size_t calibration_accel_count_ = 0;
    Eigen::Vector3d calibration_gyro_sum_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d calibration_gyro_square_sum_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d calibration_accel_sum_ = Eigen::Vector3d::Zero();
    std::optional<Eigen::Vector3d> calibrated_gyro_bias_;
    double gyro_bias_z_ = 0.0;

    Eigen::Vector3d accel_low_pass_state_ = Eigen::Vector3d::Zero();
    std::optional<TimePoint> last_accel_time_;
};

} // namespace rmcs_core::hardware::device
