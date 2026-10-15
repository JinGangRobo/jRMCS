#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/chassis_mode.hpp>

#include "filter/angle_warper.hpp"
#include "filter/kalman_filter.hpp"

namespace rmcs_core::controller::gimbal {

class DualYawSolver
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DualYawSolver()
        : rclcpp::Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , bottom_yaw_velocity_estimator_(0.001, 0.0001, 0.5, 0.01) {

        // 自旋时给 bottom yaw 的提前量（rad），再乘以自旋方向；符号可用 yaml 调。
        spin_angle_bias_ = get_parameter_or("spin_angle_bias", 0.6);
        // top yaw 负责的误差额度（rad）：先由它吃这么多，超出部分才交给 bottom。
        // 越大 → top 越主动、bottom 越少参与（顶级要求 top 比 bottom 快就是靠这个）。
        top_yaw_limit_ = get_parameter_or("top_yaw_limit", std::numbers::pi / 4.0);

        register_input("/gimbal/yaw/control_angle_error", control_angle_error_);
        register_input("/gimbal/yaw/control_angle_shift", control_angle_shift_, false);
        register_input("/gimbal/top_yaw/angle", top_yaw_angle_);
        register_input("/gimbal/yaw/control_angle_velocity", control_angle_velocity_, false);

        register_input("/gimbal/yaw/velocity_imu", gimbal_yaw_velocity_imu_);

        register_input("/chassis/control_mode", control_mode_);
        register_input("/chassis/is_spinning_forward", is_spinning_forward_);

        register_output(
            "/gimbal/bottom_yaw/estimated_velocity", estimated_bottom_yaw_velocity_, 0.0);

        register_output("/gimbal/top_yaw/target_angle_error", top_yaw_target_error_, 0.0);
        register_output("/gimbal/bottom_yaw/target_angle_error", bottom_yaw_target_error_, 0.0);
        register_output("/gimbal/top_yaw/target_angle_velocity", top_yaw_target_velocity_, 0.0);
        register_output(
            "/gimbal/bottom_yaw/target_angle_velocity", bottom_yaw_target_velocity_, 0.0);

        register_output(
            "/gimbal/bottom_yaw/control_angle_shift", bottom_yaw_control_angle_shift_, 0.0);

        status_component_ =
            create_partner_component<DualYawStatus>(get_component_name() + "_status");
    }

    void before_updating() override {
        if (!control_angle_shift_.ready()) {
            RCLCPP_INFO(
                get_logger(), "Failed to fetch \"/gimbal/yaw/control_angle_shift\", set to NaN.");
            control_angle_shift_.bind_directly(nan_);
        }
        if (!control_angle_velocity_.ready()) {
            RCLCPP_INFO(
                get_logger(), "Failed to fetch \"/gimbal/yaw/control_angle_velocity\", set to 0.");
            control_angle_velocity_.bind_directly(0.0);
        }
    }

    void update() override {
        *estimated_bottom_yaw_velocity_ =
            bottom_yaw_velocity_estimator_.update(-*top_yaw_angle_, *gimbal_yaw_velocity_imu_);

        if (std::isnan(*control_angle_error_)) {
            *top_yaw_target_error_ = nan_;
            *bottom_yaw_target_error_ = nan_;
            *top_yaw_target_velocity_ = nan_;
            *bottom_yaw_target_velocity_ = nan_;

        } else {
            double e_total = filter::wrap_to_pi(*control_angle_error_);
            // 双 yaw 协同：bottom 承担持续转动，top 既帮忙又负责把自身收回中位。
            // e_bot 是"含 top 当前偏角的剩余量"，两个环都朝它收敛 → top 回中、bottom 吃误差。
            double e_bot = filter::wrap_to_pi(e_total + *top_yaw_angle_);
            double e_bot_baised = filter::wrap_to_pi(
                e_bot
                + (rmcs_msgs::is_spining(*control_mode_) ? spin_angle_bias_ : 0.0)
                      * (*is_spinning_forward_ ? -1.0 : 1.0));

            // top 的偏角额度（原为硬编码 45°）：超过就让 bottom 接手，避免顶到走线极限。
            double top_target = std::clamp(e_bot, -top_yaw_limit_, top_yaw_limit_);
            *top_yaw_target_error_ = filter::wrap_to_pi(top_target - *top_yaw_angle_);
            *bottom_yaw_target_error_ = e_bot_baised;

            // top 的目标转速不再减 estimated_bottom_yaw_velocity（见
            // BottomYawVelocityEstimator 的 /4 说明）：那个估计值量纲不对，喂给 top 会让它乱冲。
            *bottom_yaw_target_velocity_ = *control_angle_velocity_;
            *top_yaw_target_velocity_ = *control_angle_velocity_;
        }

        if (std::isnan(*control_angle_shift_)) {
            *bottom_yaw_control_angle_shift_ = nan_;
        } else {
            *bottom_yaw_control_angle_shift_ = *control_angle_shift_;
        }
    }

private:
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    InputInterface<double> control_angle_error_, control_angle_shift_;
    double spin_angle_bias_ = 0.6;
    double top_yaw_limit_ = std::numbers::pi / 4.0;
    InputInterface<double> top_yaw_angle_;
    InputInterface<double> control_angle_velocity_;
    InputInterface<double> gimbal_yaw_velocity_imu_;
    InputInterface<rmcs_msgs::ChassisMode> control_mode_;
    InputInterface<bool> is_spinning_forward_;

    OutputInterface<double> estimated_bottom_yaw_velocity_;
    OutputInterface<double> top_yaw_target_error_, bottom_yaw_target_error_;
    OutputInterface<double> top_yaw_target_velocity_, bottom_yaw_target_velocity_;
    OutputInterface<double> bottom_yaw_control_angle_shift_;

    class DualYawStatus : public rmcs_executor::Component {
    public:
        explicit DualYawStatus() {
            register_input("/gimbal/top_yaw/angle", top_yaw_angle_);
            register_input("/gimbal/top_yaw/velocity", top_yaw_velocity_);
            register_input("/gimbal/bottom_yaw/angle", bottom_yaw_angle_);
            register_input("/gimbal/bottom_yaw/velocity", bottom_yaw_velocity_);

            register_output("/gimbal/yaw/angle", yaw_angle_, 0.0);
            register_output("/gimbal/yaw/velocity", yaw_velocity_, 0.0);
        }

        void update() override {
            *yaw_angle_ = filter::wrap_to_2pi(*top_yaw_angle_ + *bottom_yaw_angle_);

            *yaw_velocity_ = *top_yaw_velocity_ + *bottom_yaw_velocity_;
        }

    private:
        InputInterface<double> top_yaw_angle_, top_yaw_velocity_;
        InputInterface<double> bottom_yaw_angle_, bottom_yaw_velocity_;
        OutputInterface<double> yaw_angle_, yaw_velocity_;
    };

    std::shared_ptr<DualYawStatus> status_component_;

    class BottomYawVelocityEstimator {
    public:
        // 状态量：[电机角度, 云台绝对角速度] (State=2)
        // 观测量：[电机测量角度] (Measure=1)
        // 控制量：[圆盘IMU gz] (Control=1)
        using EKF = filter::ExtendedKalmanFilter<2, 1, 1>;

        using State = Eigen::Vector2d;
        using Measure = Eigen::Matrix<double, 1, 1>;
        using Control = Eigen::Matrix<double, 1, 1>;

        BottomYawVelocityEstimator(
            double dt, double process_noise_angle, double process_noise_velocity,
            double measurement_noise)
            : dt_(dt) {

            // 1. 过程模型 f(x, u)：u(0) 就是 gz
            // 下一时刻电机角度 = 当前电机角度 + (云台绝对速度 - 底盘圆盘速度) * dt
            auto process_model = [this](const State& x, const Control& u) -> State {
                State next_state;
                next_state(0) =
                    x(0) + (x(1) - u(0)) * dt_; // θ_m(k+1) = θ_m(k) + (ω_g(k) - gz) * dt
                next_state(1) = x(1);           // ω_g(k+1) = ω_g(k)
                return next_state;
            };

            // 2. 测量模型 h(x)：我们只测得了电机的相对角度
            auto measurement_model = [](const State& x) -> Measure {
                Measure z;
                z(0) = x(0);
                return z;
            };

            // 3. 过程雅可比矩阵 F = ∂f/∂x
            auto process_jacobian = [this](const State& x, const Control& u) -> Eigen::Matrix2d {
                (void)x;
                (void)u;

                Eigen::Matrix2d F;
                F << 1, dt_, // ∂θ_{m,k+1}/∂θ_{m,k} = 1, ∂θ_{m,k+1}/∂ω_{g,k} = dt
                    0, 1;    // ∂ω_{g,k+1}/∂θ_{m,k} = 0, ∂ω_{g,k+1}/∂ω_{g,k} = 1
                return F;
            };

            // 4. 测量雅可比矩阵 H = ∂h/∂x
            auto measurement_jacobian = [](const State& x) -> Eigen::Matrix<double, 1, 2> {
                (void)x;

                Eigen::Matrix<double, 1, 2> H;
                H << 1, 0; // 仅对位置项有偏导
                return H;
            };

            // 5. 过程噪声 Q 与 测量噪声 R
            Eigen::Matrix2d Q;
            Q << process_noise_angle, 0, 0, process_noise_velocity;

            Eigen::Matrix<double, 1, 1> R;
            R(0, 0) = measurement_noise;

            ekf_ = std::make_unique<EKF>(
                process_model, measurement_model, process_jacobian, measurement_jacobian, Q, R);
        }

        static double normalize_angle(double angle) { // TODO: May cause problem, replace it.
            while (angle > std::numbers::pi)
                angle -= 2 * std::numbers::pi;
            while (angle < -std::numbers::pi)
                angle += 2 * std::numbers::pi;
            return angle;
        }

        static double angle_difference(double target, double current) {
            return normalize_angle(target - current);
        }

        /**
         * 更新状态并获取云台绝对速度评估
         * @param encoder_angle 编码器测出的0~2pi角度
         * @param disk_gz       圆盘独立IMU的测出的Z轴角速度
         * @return 预测出的云台平滑绝对角速度 (rad/s)
         */
        double update(double encoder_angle, double disk_gz) {
            // ---- 连续性角度处理 (Wrap-around 处理) ----
            double current_angle = ekf_->posterior_estimate()(0);
            double angle_diff = angle_difference(encoder_angle, current_angle);
            double unwrapped_angle = current_angle + angle_diff;

            // 构造测量和控制向量
            Measure measurement;
            measurement(0) = unwrapped_angle;

            Control control;
            control(0) = disk_gz;

            // EKF预测与更新
            State estimated_state = ekf_->update(measurement, control);

            // 状态 1 = (底盘角速度 + bottom 电机角速度)，单位 rad/s。实测几何比值
            // 0.240，说明这里多除了一个 4，但**不能**简单地去掉：估计器是带滞后的
            // EKF，放大成真值会直接把 bottom SMC 的 de 项增益放大 4 倍（2026-10-14
            // 实测：去掉后上电即抖、左右都抖）。要用真值必须重做估计器，再单独整定
            // bottom 回路，不要和别的改动混在一起。
            return estimated_state(1) / 4.0; // TODO: 统一量纲（见上面的说明）
        }

    private:
        std::unique_ptr<EKF> ekf_;
        double dt_;
    };

    BottomYawVelocityEstimator bottom_yaw_velocity_estimator_;
};

} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::gimbal::DualYawSolver, rmcs_executor::Component)
