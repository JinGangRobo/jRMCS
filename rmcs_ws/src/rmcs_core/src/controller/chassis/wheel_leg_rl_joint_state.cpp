// WheelLegRlJointState: 把 RMCS 电机角/速映射到仿真系，并解算并联腿的虚拟膝，
// 以 RL 观测/控制需要的关节接口暴露出去。
//
// 这是状态的唯一来源（含电机<->仿真系标定 sign/zero），不依赖 rl_bridge，因此
// 不会与"消费动作"的控制器形成依赖环。
//
// 输入: /chassis/{left_front_hip,left_back_hip,left_wheel,right_front_hip,right_back_hip,right_wheel}/{angle,velocity}
// 输出: /wheel_leg/{left_hip_joint,left_knee_joint,left_wheel,right_hip_joint,right_knee_joint,right_wheel}/{angle,velocity}
//       /wheel_leg/rl/motor/{left_back_hip,right_back_hip}/{angle,velocity}

#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <string>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

#include "controller/chassis/wheel_leg_linkage.hpp"

namespace rmcs_core::controller::chassis {

class WheelLegRlJointState
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    WheelLegRlJointState()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        l1_ = get_parameter_or<double>("l1", 0.175);
        l2_ = get_parameter_or<double>("l2", 0.208);
        linkage_offset_ = get_parameter_or<double>("linkage_offset", 1.6614);
        map_eps_ = get_parameter_or<double>("map_eps", 1e-6);
        default_left_knee_ = get_parameter_or<double>("default_left_knee", -0.65);
        default_right_knee_ = get_parameter_or<double>("default_right_knee", 0.65);
        swap_sides_ = get_parameter_or<bool>("swap_sides", false);

        constexpr std::array<const char*, 6> kSignKeys{
            "sign_lf0", "sign_l20", "sign_lw", "sign_rf0", "sign_r20", "sign_rw"};
        constexpr std::array<const char*, 6> kZeroKeys{
            "zero_lf0", "zero_l20", "zero_lw", "zero_rf0", "zero_r20", "zero_rw"};
        for (std::size_t i = 0; i < 6; ++i) {
            sign_[i] = get_parameter_or<double>(kSignKeys[i], 1.0);
            zero_[i] = get_parameter_or<double>(kZeroKeys[i], 0.0);
        }

        // 电机输入: [lf0, l20, lw, rf0, r20, rw]
        constexpr std::array<const char*, 6> kMotorNames{
            "left_front_hip", "left_back_hip", "left_wheel", "right_front_hip", "right_back_hip",
            "right_wheel"};
        for (std::size_t i = 0; i < 6; ++i) {
            register_input(std::string{"/chassis/"} + kMotorNames[i] + "/angle", motor_angle_[i]);
            register_input(
                std::string{"/chassis/"} + kMotorNames[i] + "/velocity", motor_velocity_[i]);
        }

        constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
        // 虚拟关节输出: [lf0, lf1, lw, rf0, rf1, rw]
        constexpr std::array<const char*, 6> kJointNames{
            "left_hip_joint", "left_knee_joint", "left_wheel", "right_hip_joint", "right_knee_joint",
            "right_wheel"};
        for (std::size_t i = 0; i < 6; ++i) {
            register_output(
                std::string{"/wheel_leg/"} + kJointNames[i] + "/angle", joint_angle_[i], kNan);
            register_output(
                std::string{"/wheel_leg/"} + kJointNames[i] + "/velocity", joint_velocity_[i], kNan);
        }
        // 后髋电机(仿真系), 供力矩控制器 force_map 用
        register_output("/wheel_leg/rl/motor/left_back_hip/angle", l20_angle_, kNan);
        register_output("/wheel_leg/rl/motor/left_back_hip/velocity", l20_velocity_, kNan);
        register_output("/wheel_leg/rl/motor/right_back_hip/angle", r20_angle_, kNan);
        register_output("/wheel_leg/rl/motor/right_back_hip/velocity", r20_velocity_, kNan);
    }

    void update() override {
        for (std::size_t i = 0; i < 6; ++i)
            if (!motor_angle_[i].ready() || !motor_velocity_[i].ready()
                || !std::isfinite(*motor_angle_[i]) || !std::isfinite(*motor_velocity_[i]))
                return;

        // RMCS 电机序: [lf0,l20,lw,rf0,r20,rw]
        //   = [left_front_hip, left_back_hip, left_wheel, right_front_hip, right_back_hip, right_wheel]
        double rmcs[6], rmcs_vel[6];
        for (std::size_t i = 0; i < 6; ++i) {
            rmcs[i] = sign_[i] * *motor_angle_[i] + zero_[i];
            rmcs_vel[i] = sign_[i] * *motor_velocity_[i];
        }
        // 模型序 [lf0,l20,lw,rf0,r20,rw]。RMCS 左=模型右时，模型 k 取 RMCS (k+3)%6。
        double model[6], model_vel[6];
        for (std::size_t k = 0; k < 6; ++k) {
            const std::size_t r = swap_sides_ ? (k + 3) % 6 : k;
            model[k] = rmcs[r];
            model_vel[k] = rmcs_vel[r];
        }
        // 电机角只按模 2π 有意义：把整圈偏移(如 44 rad)折到 [-π,π]。
        // 连杆几何/force_map 对 phi1/phi4 都是 2π 周期，折圈不影响结果(虚拟膝再按支路展开)。
        constexpr double kTwoPi = 2.0 * std::numbers::pi;
        for (const std::size_t k : {std::size_t{0}, std::size_t{1}, std::size_t{3}, std::size_t{4}})
            model[k] = std::remainder(model[k], kTwoPi);

        const double lf0 = model[0], l20 = model[1];
        const double rf0 = model[3], r20 = model[4];

        // phi3 用 atan2(值域 (-π,π])，跨 ±π 会跳 2π。把虚拟膝解到名义值附近的支路
        // (工作区间远小于 π)，消除跳变；微分也用同一支路展开。
        const auto wrap_near = [](double value, double reference) {
            return reference + std::remainder(value - reference, kTwoPi);
        };
        const auto solve = [&](bool left, double phi1, double phi4) {
            return left ? linkage::solve_phi3_left(phi1, phi4, l1_, l2_)
                        : linkage::solve_phi3_right(phi1, phi4, l1_, l2_);
        };
        const double phi1_left = linkage_offset_ + l20;
        const double phi1_right = linkage_offset_ - r20;
        const double lf1 = wrap_near(solve(true, phi1_left, lf0), default_left_knee_);
        const double rf1 = wrap_near(solve(false, phi1_right, -rf0), default_right_knee_);
        const auto derivative = [&](bool left, double phi1, double phi4, double center) {
            const std::array<double, 4> f{
                wrap_near(solve(left, phi1 + map_eps_, phi4), center),
                wrap_near(solve(left, phi1 - map_eps_, phi4), center),
                wrap_near(solve(left, phi1, phi4 + map_eps_), center),
                wrap_near(solve(left, phi1, phi4 - map_eps_), center)};
            return std::array<double, 2>{
                (f[0] - f[1]) / (2.0 * map_eps_), (f[2] - f[3]) / (2.0 * map_eps_)};
        };
        const auto dl = derivative(true, phi1_left, lf0, lf1);
        const auto dr = derivative(false, phi1_right, -rf0, rf1);
        const double lf1_vel = dl[0] * model_vel[1] + dl[1] * model_vel[0];
        // 右侧几何用取反角(offset-r20, -rf0)，速度链同样取反(与 sim2sim 的 rf20_vel/rf0_vel 一致)。
        const double rf1_vel = -(dr[0] * model_vel[4] + dr[1] * model_vel[3]);

        const std::array<double, 6> q{lf0, lf1, model[2], rf0, rf1, model[5]};
        const std::array<double, 6> qd{
            model_vel[0], lf1_vel, model_vel[2], model_vel[3], rf1_vel, model_vel[5]};
        for (std::size_t i = 0; i < 6; ++i) {
            *joint_angle_[i] = q[i];
            *joint_velocity_[i] = qd[i];
        }
        *l20_angle_ = l20;
        *l20_velocity_ = model_vel[1];
        *r20_angle_ = r20;
        *r20_velocity_ = model_vel[4];
    }

private:
    double l1_ = 0.175, l2_ = 0.208, linkage_offset_ = 1.6614, map_eps_ = 1e-6;
    double default_left_knee_ = -0.65, default_right_knee_ = 0.65;
    bool swap_sides_ = false;
    std::array<double, 6> sign_{1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    std::array<double, 6> zero_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    std::array<InputInterface<double>, 6> motor_angle_, motor_velocity_;
    std::array<OutputInterface<double>, 6> joint_angle_, joint_velocity_;
    OutputInterface<double> l20_angle_, l20_velocity_, r20_angle_, r20_velocity_;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::WheelLegRlJointState, rmcs_executor::Component)
