#pragma once

#include <eigen3/Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <utility>

namespace rmcs_core::filter {

class ImuEkf {
public:
    using Vec3 = Eigen::Vector3d;
    using Vec4 = Eigen::Vector4d;
    using Vec6 = Eigen::Matrix<double, 6, 1>;
    using Mat3 = Eigen::Matrix3d;
    using Mat4 = Eigen::Matrix4d;
    using Mat6 = Eigen::Matrix<double, 6, 6>;
    using Mat36 = Eigen::Matrix<double, 3, 6>;
    using Mat43 = Eigen::Matrix<double, 4, 3>;
    using Mat63 = Eigen::Matrix<double, 6, 3>;

    struct Config {
        Mat3 process_noise = 1.22e-3 * Mat3::Identity();
        double bias_process_noise = 1.0e-3;
        Mat3 measurement_noise = 50.0 * Mat3::Identity();
        Mat3 gate_noise = 0.02 * Mat3::Identity();

        Mat4 initial_covariance = 0.15 * Mat4::Identity();
        double initial_bias_variance = 1.0e-3;
    };

    class AccelCorrection {
    public:
        [[nodiscard]] double chi_square() const noexcept { return chi_square_; }

    private:
        friend class ImuEkf;

        AccelCorrection(
            const std::uint64_t revision, Vec3 innovation, Mat36 jacobian_h_x,
            Eigen::LDLT<Mat3> innovation_covariance_ldlt, const double chi_square)
            : revision_(revision)
            , innovation_(std::move(innovation))
            , jacobian_h_x_(std::move(jacobian_h_x))
            , innovation_covariance_ldlt_(std::move(innovation_covariance_ldlt))
            , chi_square_(chi_square) {}

        std::uint64_t revision_ = 0;
        Vec3 innovation_ = Vec3::Zero();
        Mat36 jacobian_h_x_ = Mat36::Zero();
        Eigen::LDLT<Mat3> innovation_covariance_ldlt_;
        double chi_square_ = std::numeric_limits<double>::quiet_NaN();
    };

    ImuEkf()
        : ImuEkf(Config{}) {}

    explicit ImuEkf(Config config)
        : config_(std::move(config)) {
        reset();
    }

    [[nodiscard]] const Config& config() const noexcept { return config_; }

    void set_config(const Config& config) {
        config_ = config;
        reset();
    }

    void reset() {
        state_ = identity_state();
        covariance_ = projected_initial_covariance(state_.head<4>());
        ++revision_;
    }

    [[nodiscard]] bool reset_from_accel(const Vec3& accel_g, const double initial_yaw = 0.0) {
        if (!accel_g.allFinite() || !std::isfinite(initial_yaw)) {
            return false;
        }

        const std::optional<Vec3> normalized_accel = normalize_vector(accel_g);
        if (!normalized_accel.has_value()) {
            return false;
        }

        const double gx = (*normalized_accel)(0);
        const double gy = (*normalized_accel)(1);
        const double gz = std::clamp((*normalized_accel)(2), -1.0, 1.0);

        double q0_val, q1_val, q2_val, q3_val;
        if (gz >= 0.0) {
            const double s = std::sqrt(0.5 * (1.0 + gz));
            const double inv = 0.5 / s;
            q0_val = s;
            q1_val = gy * inv;
            q2_val = -gx * inv;
            q3_val = 0.0;
        } else {
            const double s = std::sqrt(0.5 * (1.0 - gz));
            const double inv = 0.5 / s;
            q0_val = gy * inv;
            q1_val = s;
            q2_val = 0.0;
            q3_val = gx * inv;
        }

        const double cy = std::cos(initial_yaw * 0.5);
        const double sy = std::sin(initial_yaw * 0.5);

        Vec4 quaternion;
        quaternion(0) = cy * q0_val - sy * q3_val;
        quaternion(1) = cy * q1_val - sy * q2_val;
        quaternion(2) = cy * q2_val + sy * q1_val;
        quaternion(3) = cy * q3_val + sy * q0_val;

        quaternion = normalize_quaternion(quaternion);
        state_ = identity_state();
        state_.head<4>() = quaternion;
        covariance_ = projected_initial_covariance(quaternion);
        ++revision_;
        return true;
    }

    void inflate_attitude_uncertainty_to_initial() {
        Mat6 covariance_floor = Mat6::Zero();
        covariance_floor.topLeftCorner<4, 4>() =
            projected_covariance(state_.head<4>(), config_.initial_covariance);
        covariance_floor.bottomRightCorner<2, 2>() = covariance_.bottomRightCorner<2, 2>();
        covariance_ = covariance_with_psd_floor(covariance_, covariance_floor);
        ++revision_;
    }

    [[nodiscard]] bool predict(const Vec3& gyro_rad_per_sec, const double dt_seconds) {
        if (!gyro_rad_per_sec.allFinite()) {
            return false;
        }

        if (!std::isfinite(dt_seconds) || dt_seconds < 0.0) {
            return false;
        }

        const Vec4 quaternion_prior = state_.head<4>();
        const Vec3 bias_corrected_gyro = gyro_rad_per_sec - Vec3{state_(4), state_(5), 0.0};

        const PropagationResult propagation =
            propagate_quaternion(quaternion_prior, bias_corrected_gyro, dt_seconds);
        if (!propagation.quaternion.allFinite() || !propagation.orthogonalization.allFinite()) {
            return false;
        }
        const Mat4& orthogonalization = propagation.orthogonalization;

        const Mat6 jacobian_f_x =
            state_jacobian(quaternion_prior, bias_corrected_gyro, dt_seconds, orthogonalization);
        const Mat43 jacobian_f_w = process_noise_jacobian(quaternion_prior, orthogonalization);

        Mat6 process_covariance = Mat6::Zero();
        process_covariance.topLeftCorner<4, 4>() =
            jacobian_f_w * config_.process_noise * jacobian_f_w.transpose();
        process_covariance.bottomRightCorner<2, 2>() =
            config_.bias_process_noise * Eigen::Matrix2d::Identity();

        Mat6 prior_cov = symmetrized(
            jacobian_f_x * covariance_ * jacobian_f_x.transpose()
            + dt_seconds * process_covariance);
        if (!is_positive_semidefinite(prior_cov)) {
            return false;
        }

        state_.head<4>() = propagation.quaternion;
        covariance_ = prior_cov;
        ++revision_;
        return true;
    }

    [[nodiscard]] std::optional<AccelCorrection> prepare_correction(const Vec3& accel_g) const {
        if (!accel_g.allFinite())
            return std::nullopt;

        const std::optional<Vec3> normalized_accel = normalize_vector(accel_g);
        if (!normalized_accel.has_value())
            return std::nullopt;

        const Vec3 predicted_accel = measurement_model(state_.head<4>());
        const Vec3 gate_innovation = accel_g - predicted_accel;
        const Vec3 update_innovation = *normalized_accel - predicted_accel;
        const Mat36 jacobian_h_x = measurement_jacobian(state_.head<4>());
        const Mat3 projected_covariance = jacobian_h_x * covariance_ * jacobian_h_x.transpose();
        const Mat3 gate_covariance = symmetrized(projected_covariance + config_.gate_noise);

        if (!is_positive_definite(gate_covariance)) {
            return std::nullopt;
        }

        const auto gate_ldt = gate_covariance.ldlt();
        if (gate_ldt.info() != Eigen::Success || !gate_ldt.isPositive()) {
            return std::nullopt;
        }
        const Vec3 innovation_weighted = gate_ldt.solve(gate_innovation);
        if (!innovation_weighted.allFinite()) {
            return std::nullopt;
        }
        const double chi_square = gate_innovation.dot(innovation_weighted);
        if (!std::isfinite(chi_square)) {
            return std::nullopt;
        }

        const Mat3 update_covariance =
            symmetrized(projected_covariance + config_.measurement_noise);
        if (!is_positive_definite(update_covariance)) {
            return std::nullopt;
        }

        const auto update_ldt = update_covariance.ldlt();
        if (update_ldt.info() != Eigen::Success || !update_ldt.isPositive()) {
            return std::nullopt;
        }

        return AccelCorrection(revision_, update_innovation, jacobian_h_x, update_ldt, chi_square);
    }

    bool correct(const AccelCorrection& update) {
        if (update.revision_ != revision_) {
            return false;
        }
        const Vec6 state_prior = state_;
        const Mat36 k_transpose =
            update.innovation_covariance_ldlt_.solve(update.jacobian_h_x_ * covariance_);
        if (!k_transpose.allFinite()) {
            return false;
        }
        const Mat63 kalman_gain = k_transpose.transpose();

        // The accelerometer observes only the two tilt degrees of freedom; the
        // rotation about gravity (yaw) is unobservable.  Zeroing the body-frame
        // q3 term only equals "no yaw correction" while the body is level, so
        // remove the component of the attitude correction along the predicted
        // gravity direction instead.
        Vec6 state_update = kalman_gain * update.innovation_;
        const Vec3 gravity_direction = measurement_model(state_prior.head<4>());
        if (gravity_direction.squaredNorm() > kEpsilon) {
            Vec3 rotation_update = state_update.segment<3>(1);
            rotation_update -= rotation_update.dot(gravity_direction) * gravity_direction;
            state_update.segment<3>(1) = rotation_update;
        }

        const Vec6 state_raw = state_prior + state_update;
        if (!state_raw.allFinite() || state_raw.head<4>().squaredNorm() <= kEpsilon) {
            return false;
        }

        const Mat6 matrix_tmp = Mat6::Identity() - kalman_gain * update.jacobian_h_x_;
        Mat6 corrected_covariance =
            matrix_tmp * covariance_ * matrix_tmp.transpose()
            + kalman_gain * config_.measurement_noise * kalman_gain.transpose();

        Mat6 normalization = Mat6::Identity();
        normalization.topLeftCorner<4, 4>() = orthogonalization_jacobian(state_raw.head<4>());
        corrected_covariance =
            symmetrized(normalization * corrected_covariance * normalization.transpose());

        if (!is_positive_semidefinite(corrected_covariance)) {
            return false;
        }
        state_ = state_raw;
        state_.head<4>() = normalize_quaternion(state_raw.head<4>());
        covariance_ = corrected_covariance;
        ++revision_;
        return true;
    }

    bool correct(const Vec3& accel_mps2) {
        const std::optional<AccelCorrection> update = prepare_correction(accel_mps2);
        return update.has_value() && correct(*update);
    }

    [[nodiscard]] Eigen::Quaterniond quaternion() const noexcept {
        return {state_[0], state_[1], state_[2], state_[3]};
    }

    void set_initial_gyro_bias(const double bias_x, const double bias_y) noexcept {
        state_(4) = bias_x;
        state_(5) = bias_y;
    }

    [[nodiscard]] Eigen::Vector2d gyro_bias() const noexcept { return {state_(4), state_(5)}; }

private:
    struct PropagationResult {
        Vec4 quaternion = identity_quaternion();
        Mat4 orthogonalization = Mat4::Identity();
    };

    [[nodiscard]] static Vec4 identity_quaternion() {
        Vec4 q;
        q << 1.0, 0.0, 0.0, 0.0;
        return q;
    }

    [[nodiscard]] static Vec6 identity_state() {
        Vec6 state = Vec6::Zero();
        state(0) = 1.0;
        return state;
    }

    [[nodiscard]] static std::optional<Vec3> normalize_vector(const Vec3& vector) {
        const double squared_norm = vector.squaredNorm();
        if (!std::isfinite(squared_norm) || squared_norm <= kEpsilon) {
            return std::nullopt;
        }

        return vector / std::sqrt(squared_norm);
    }

    [[nodiscard]] static Vec4 normalize_quaternion(const Vec4& quaternion) {
        const double squared_norm = quaternion.squaredNorm();
        if (!std::isfinite(squared_norm) || squared_norm <= kEpsilon) {
            return identity_quaternion();
        }

        return quaternion / std::sqrt(squared_norm);
    }

    template <typename Derived>
    [[nodiscard]] static typename Derived::PlainObject
        symmetrized(const Eigen::MatrixBase<Derived>& matrix) {
        return (0.5 * (matrix + matrix.transpose())).eval();
    }

    [[nodiscard]] static bool is_positive_definite(const Mat3& matrix) {
        if (!matrix.allFinite()) {
            return false;
        }

        const Eigen::SelfAdjointEigenSolver<Mat3> solver(symmetrized(matrix));
        if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite()) {
            return false;
        }

        const double max_abs_eigenvalue = solver.eigenvalues().cwiseAbs().maxCoeff();
        const double tolerance = kEpsilon * std::max(1.0, max_abs_eigenvalue);
        return solver.eigenvalues().minCoeff() > tolerance;
    }

    template <typename Derived>
    [[nodiscard]] static bool is_positive_semidefinite(const Eigen::MatrixBase<Derived>& matrix) {
        if (!matrix.allFinite()) {
            return false;
        }

        using PlainMatrix = typename Derived::PlainObject;
        const Eigen::SelfAdjointEigenSolver<PlainMatrix> solver(symmetrized(matrix));
        if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite()) {
            return false;
        }

        const double max_abs_eigenvalue = solver.eigenvalues().cwiseAbs().maxCoeff();
        const double tolerance = kEpsilon * std::max(1.0, max_abs_eigenvalue);
        return solver.eigenvalues().minCoeff() >= -tolerance;
    }

    template <typename Derived>
    [[nodiscard]] static typename Derived::PlainObject
        positive_semidefinite_part(const Eigen::MatrixBase<Derived>& matrix) {
        using PlainMatrix = typename Derived::PlainObject;
        const Eigen::SelfAdjointEigenSolver<PlainMatrix> solver(symmetrized(matrix));
        if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite()) {
            return PlainMatrix::Zero();
        }

        return symmetrized(
            solver.eigenvectors() * solver.eigenvalues().cwiseMax(0.0).asDiagonal()
            * solver.eigenvectors().transpose());
    }

    [[nodiscard]] static Mat6
        covariance_with_psd_floor(const Mat6& covariance, const Mat6& covariance_floor) {
        Mat6 floor = symmetrized(covariance_floor);
        Mat6 result = floor + positive_semidefinite_part(symmetrized(covariance) - floor);
        result = symmetrized(result);
        if (is_positive_semidefinite(result)) {
            return result;
        }

        return floor;
    }

    [[nodiscard]] static Mat4
        projected_covariance(const Vec4& quaternion, const Mat4& raw_covariance) {
        const Mat4 j_norm = orthogonalization_jacobian(quaternion);
        Mat4 covariance = symmetrized(j_norm * raw_covariance * j_norm.transpose());
        if (is_positive_semidefinite(covariance)) {
            return covariance;
        }

        return symmetrized(j_norm * Mat4::Identity() * j_norm.transpose());
    }

    [[nodiscard]] Mat6 projected_initial_covariance(const Vec4& quaternion) const {
        Mat6 covariance = Mat6::Zero();
        covariance.topLeftCorner<4, 4>() =
            projected_covariance(quaternion, config_.initial_covariance);
        covariance.bottomRightCorner<2, 2>() =
            config_.initial_bias_variance * Eigen::Matrix2d::Identity();
        return covariance;
    }

    [[nodiscard]] static Mat4 omega_matrix(const Vec3& gyro_rad_per_sec) {
        const double gx = gyro_rad_per_sec(0);
        const double gy = gyro_rad_per_sec(1);
        const double gz = gyro_rad_per_sec(2);
        Mat4 matrix;
        // clang-format off
        matrix << 0, -gx, -gy, -gz,
                  gx,  0,  gz, -gy,
                  gy, -gz,  0,  gx,
                  gz,  gy, -gx,  0;
        // clang-format on
        return matrix;
    }

    [[nodiscard]] static Mat4 orthogonalization_jacobian(const Vec4& quaternion) {
        const double norm = quaternion.norm();
        const double inv_norm = (std::isfinite(norm) && norm > kEpsilon) ? 1.0 / norm : 1.0;
        return inv_norm
             * (Mat4::Identity() - inv_norm * inv_norm * quaternion * quaternion.transpose());
    }

    [[nodiscard]] static PropagationResult
        propagate_quaternion(const Vec4& state, const Vec3& gyro_rad_per_sec, const double dt) {
        const Mat4 omega = omega_matrix(gyro_rad_per_sec);
        const Vec4 quaternion_tmp = state + 0.5 * dt * omega * state;

        PropagationResult result;
        result.orthogonalization = orthogonalization_jacobian(quaternion_tmp);
        result.quaternion = normalize_quaternion(quaternion_tmp);
        return result;
    }

    [[nodiscard]] static Mat6 state_jacobian(
        const Vec4& quaternion, const Vec3& gyro_rad_per_sec, const double dt,
        const Mat4& orthogonalization) {
        Mat6 result = Mat6::Identity();
        result.topLeftCorner<4, 4>() =
            orthogonalization * (Mat4::Identity() + 0.5 * dt * omega_matrix(gyro_rad_per_sec));

        // Derivative of the propagated quaternion with respect to the gyroscope
        // bias, left-multiplied by the normalization Jacobian.
        Eigen::Matrix<double, 4, 2> bias_jacobian;
        // clang-format off
        bias_jacobian <<  quaternion(1),  quaternion(2),
                         -quaternion(0),  quaternion(3),
                         -quaternion(3), -quaternion(0),
                          quaternion(2), -quaternion(1);
        // clang-format on
        result.topRightCorner<4, 2>() = orthogonalization * (0.5 * dt * bias_jacobian);
        return result;
    }

    [[nodiscard]] static Mat43
        process_noise_jacobian(const Vec4& state, const Mat4& orthogonalization) {
        Mat43 matrix_q;
        // clang-format off
        matrix_q << -state(1), -state(2), -state(3),
                     state(0), -state(3),  state(2),
                     state(3),  state(0), -state(1),
                    -state(2),  state(1),  state(0);
        // clang-format on
        return orthogonalization * (0.5 * matrix_q);
    }

    [[nodiscard]] static Vec3 measurement_model(const Vec4& state) {
        Vec3 result;
        result(0) = 2.0 * (state(1) * state(3) - state(0) * state(2));
        result(1) = 2.0 * (state(2) * state(3) + state(0) * state(1));
        result(2) =
            state(0) * state(0) - state(1) * state(1) - state(2) * state(2) + state(3) * state(3);
        return result;
    }

    [[nodiscard]] static Mat36 measurement_jacobian(const Vec4& state) {
        Mat36 result = Mat36::Zero();
        // clang-format off
        result.topLeftCorner<3, 4>() <<
            -2.0 * state(2),  2.0 * state(3), -2.0 * state(0),  2.0 * state(1),
             2.0 * state(1),  2.0 * state(0),  2.0 * state(3),  2.0 * state(2),
             2.0 * state(0), -2.0 * state(1), -2.0 * state(2),  2.0 * state(3);
        // clang-format on
        return result;
    }

    static constexpr double kEpsilon = 1e-12;

    Config config_;
    Vec6 state_ = identity_state();
    Mat6 covariance_ = Mat6::Identity();
    std::uint64_t revision_ = 0;
};

} // namespace rmcs_core::filter
