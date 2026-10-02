// WheelLegRlController: 并联(闭链)轮腿的 RL 力矩级控制器（只消费动作、输出力矩）。
//
// 复刻 wheel_leg_real2sim / infantry_binglian 的 compute_control：
//   虚拟关节 PD(位置/速度动作) -> 虚拟力矩 -> 连杆 jacobian -> 电机力矩
//   -> force_map 域内气弹簧补偿 -> 反变换回电机力矩 -> 6 电机 control_torque。
//
// 关节/电机状态由 WheelLegRlJointState 提供（仿真系），本组件只做控制，
// 因此不反向依赖关节状态，避免与 rl_bridge 形成环。
//
// 输入: /wheel_leg/{left_hip_joint,left_knee_joint,left_wheel,right_hip_joint,right_knee_joint,right_wheel}/{angle,velocity}
//       /wheel_leg/rl/motor/{left_back_hip,right_back_hip}/{angle,velocity}   (仿真系电机角)
//       /wheel_leg/rl/action/{left_hip,left_knee,left_wheel,right_hip,right_knee,right_wheel}
//       /wheel_leg/rl/valid, /wheel_leg/rl/healthy
// 输出: /chassis/<motor>/control_torque  (6 电机)
//       [lf0, l20, lw, rf0, r20, rw]

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

#include "controller/chassis/wheel_leg_linkage.hpp"

namespace rmcs_core::controller::chassis {

class WheelLegRlController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    WheelLegRlController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        l1_ = get_parameter_or<double>("l1", 0.175);
        l2_ = get_parameter_or<double>("l2", 0.208);
        linkage_offset_ = get_parameter_or<double>("linkage_offset", 1.6614);
        map_eps_ = get_parameter_or<double>("map_eps", 1e-6);
        gas_spring_force_ = get_parameter_or<double>("gas_spring_force", 300.0 * 1.23);
        torque_map_ = get_parameter_or<std::string>("torque_map", "analytic");

        action_scale_pos_ = get_parameter_or<double>("action_scale_pos", 0.5);
        action_scale_vel_ = get_parameter_or<double>("action_scale_vel", 10.0);
        p_gain_ = get_parameter_or<double>("p_gain", 15.0);
        d_gain_ = get_parameter_or<double>("d_gain", 1.0);
        wheel_d_gain_ = get_parameter_or<double>("wheel_d_gain", 0.1);
        default_left_hip_ = get_parameter_or<double>("default_left_hip", -0.23);
        default_left_knee_ = get_parameter_or<double>("default_left_knee", -0.65);
        default_right_hip_ = get_parameter_or<double>("default_right_hip", 0.23);
        default_right_knee_ = get_parameter_or<double>("default_right_knee", 0.65);
        torque_limit_ = get_parameter_or<double>("torque_limit", 30.0);
        wheel_torque_limit_ = get_parameter_or<double>("wheel_torque_limit", 5.0);

        // 与 wheel_leg_rl_joint_state 的 sign_* 必须一致（sim->接口的力矩同号映射）。
        constexpr std::array<const char*, 6> kSignKeys{
            "sign_lf0", "sign_l20", "sign_lw", "sign_rf0", "sign_r20", "sign_rw"};
        for (std::size_t i = 0; i < 6; ++i)
            sign_[i] = get_parameter_or<double>(kSignKeys[i], 1.0);
        swap_sides_ = get_parameter_or<bool>("swap_sides", false);

        // 虚拟关节状态(仿真系): [lf0, lf1, lw, rf0, rf1, rw]
        constexpr std::array<const char*, 6> kJointNames{
            "left_hip_joint", "left_knee_joint", "left_wheel", "right_hip_joint", "right_knee_joint",
            "right_wheel"};
        for (std::size_t i = 0; i < 6; ++i) {
            register_input(std::string{"/wheel_leg/"} + kJointNames[i] + "/angle", joint_angle_[i]);
            register_input(
                std::string{"/wheel_leg/"} + kJointNames[i] + "/velocity", joint_velocity_[i]);
        }
        // 后髋电机(仿真系), force_map 需要
        register_input("/wheel_leg/rl/motor/left_back_hip/angle", l20_angle_);
        register_input("/wheel_leg/rl/motor/right_back_hip/angle", r20_angle_);

        // RL 动作: index 顺序 = [lf0, lf1, lw, rf0, rf1, rw]
        constexpr std::array<const char*, 6> kActionNames{
            "left_hip", "left_knee", "left_wheel", "right_hip", "right_knee", "right_wheel"};
        for (std::size_t i = 0; i < 6; ++i)
            register_input(std::string{"/wheel_leg/rl/action/"} + kActionNames[i], action_[i], false);
        register_input("/wheel_leg/rl/valid", valid_, false);
        register_input("/wheel_leg/rl/healthy", healthy_, false);

        // 电机力矩输出: 注册顺序 [lf0, l20, lw, rf0, r20, rw]
        constexpr std::array<const char*, 6> kMotorNames{
            "left_front_hip", "left_back_hip", "left_wheel", "right_front_hip", "right_back_hip",
            "right_wheel"};
        for (std::size_t i = 0; i < 6; ++i)
            register_output(
                std::string{"/chassis/"} + kMotorNames[i] + "/control_torque", motor_torque_[i],
                0.0);
    }

    void update() override {
        if (!state_ready() || !policy_active()) {
            for (auto& torque : motor_torque_)
                *torque = 0.0;
            return;
        }
        compute_control();
    }

private:
    bool state_ready() const {
        for (std::size_t i = 0; i < 6; ++i)
            if (!joint_angle_[i].ready() || !joint_velocity_[i].ready()
                || !std::isfinite(*joint_angle_[i]) || !std::isfinite(*joint_velocity_[i]))
                return false;
        return l20_angle_.ready() && r20_angle_.ready() && std::isfinite(*l20_angle_)
            && std::isfinite(*r20_angle_);
    }

    bool policy_active() const {
        return valid_.ready() && healthy_.ready() && *valid_ > 0.5 && *healthy_ > 0.5
            && std::all_of(action_.begin(), action_.end(), [](const auto& input) {
                   return input.ready() && std::isfinite(*input);
               });
    }

    void compute_control() {
        const double q[6] = {
            *joint_angle_[0], *joint_angle_[1], *joint_angle_[2],
            *joint_angle_[3], *joint_angle_[4], *joint_angle_[5]};
        const double qd[6] = {
            *joint_velocity_[0], *joint_velocity_[1], *joint_velocity_[2],
            *joint_velocity_[3], *joint_velocity_[4], *joint_velocity_[5]};

        const std::array<double, 6> p_gain{p_gain_, p_gain_, 0.0, p_gain_, p_gain_, 0.0};
        const std::array<double, 6> d_gain{
            d_gain_, d_gain_, wheel_d_gain_, d_gain_, d_gain_, wheel_d_gain_};
        const std::array<double, 6> default_pos{
            default_left_hip_, default_left_knee_, 0.0, default_right_hip_, default_right_knee_,
            0.0};

        std::array<double, 6> pos_ref{}, vel_ref{}, tau_virtual{};
        for (std::size_t i = 0; i < 6; ++i)
            pos_ref[i] = *action_[i] * action_scale_pos_;
        pos_ref[2] = 0.0;
        pos_ref[5] = 0.0;
        for (std::size_t i = 0; i < 6; ++i)
            vel_ref[i] = *action_[i] * action_scale_vel_;
        vel_ref[0] = vel_ref[1] = vel_ref[3] = vel_ref[4] = 0.0;
        for (std::size_t i = 0; i < 6; ++i)
            tau_virtual[i] =
                p_gain[i] * (pos_ref[i] + default_pos[i] - q[i]) + d_gain[i] * (vel_ref[i] - qd[i]);

        const double lf0 = q[0], rf0 = q[3];
        const double l20 = *l20_angle_, r20 = *r20_angle_;
        const double tau_lf0 = tau_virtual[0], tau_lf1 = tau_virtual[1];
        const double tau_rf0 = tau_virtual[3], tau_rf1 = tau_virtual[4];

        double tau_lf0_act = 0.0, tau_lf20_act = 0.0;
        double tau_rf0_act = 0.0, tau_rf20_act = 0.0;
        if (torque_map_ == "numeric") {
            const auto jl = linkage::solve_phi_piandao(
                linkage::solve_phi3_left, linkage_offset_ + l20, lf0, l1_, l2_, map_eps_);
            const auto jr = linkage::solve_phi_piandao(
                linkage::solve_phi3_right, linkage_offset_ - r20, -rf0, l1_, l2_, map_eps_);
            tau_lf0_act = tau_lf0 + tau_lf1 * jl.d_phi4;
            tau_lf20_act = tau_lf1 * jl.d_phi1;
            tau_rf0_act = tau_rf0 - tau_rf1 * jr.d_phi4;
            tau_rf20_act = -tau_rf1 * jr.d_phi1;
        } else {
            const auto jl = linkage::solve_jacobian_analytic(linkage_offset_ + l20, lf0, l1_, l2_);
            const auto jr = linkage::solve_jacobian_analytic(linkage_offset_ - r20, -rf0, l1_, l2_);
            tau_lf0_act = tau_lf0 + tau_lf1 * jl.d_phi4;
            tau_lf20_act = tau_lf1 * jl.d_phi1;
            tau_rf0_act = tau_rf0 + tau_rf1 * jr.d_phi4;
            tau_rf20_act = tau_rf1 * jr.d_phi1;
        }

        // force_map 域内气弹簧补偿
        const auto ml = linkage::force_map(linkage_offset_ + l20, lf0, l1_, l2_);
        double fl0 = ml.i00 * tau_lf20_act + ml.i01 * tau_lf0_act;
        double fl1 = ml.i10 * tau_lf20_act + ml.i11 * tau_lf0_act;
        fl0 -= gas_spring_force_ * ml.l0;
        tau_lf20_act = ml.j00 * fl0 + ml.j01 * fl1;
        tau_lf0_act = ml.j10 * fl0 + ml.j11 * fl1;

        const auto mr = linkage::force_map(linkage_offset_ - r20, -rf0, l1_, l2_);
        double fr0 = mr.i00 * tau_rf20_act + mr.i01 * tau_rf0_act;
        double fr1 = mr.i10 * tau_rf20_act + mr.i11 * tau_rf0_act;
        fr0 += gas_spring_force_ * mr.l0;
        tau_rf20_act = mr.j00 * fr0 + mr.j01 * fr1;
        tau_rf0_act = mr.j10 * fr0 + mr.j11 * fr1;

        // 按电机注册顺序 [lf0, l20, lw, rf0, r20, rw] 对齐输出
        const std::array<double, 6> tau_motor{
            clamp(tau_lf0_act, torque_limit_), clamp(tau_lf20_act, torque_limit_),
            clamp(tau_virtual[2], wheel_torque_limit_), clamp(tau_rf0_act, torque_limit_),
            clamp(tau_rf20_act, torque_limit_), clamp(tau_virtual[5], wheel_torque_limit_)};
        // tau_motor 是模型序；motor_torque_ 是 RMCS 序。RMCS 左=模型右时需按 (k+3)%6 对调。
        for (std::size_t k = 0; k < 6; ++k) {
            const std::size_t r = swap_sides_ ? (k + 3) % 6 : k;
            *motor_torque_[r] = sign_[r] * tau_motor[k];
        }
    }

    static double clamp(double value, double limit) {
        return std::clamp(value, -limit, limit);
    }

    double l1_ = 0.175, l2_ = 0.208, linkage_offset_ = 1.6614, map_eps_ = 1e-6;
    double gas_spring_force_ = 300.0 * 1.23;
    std::string torque_map_ = "analytic";
    double action_scale_pos_ = 0.5, action_scale_vel_ = 10.0;
    double p_gain_ = 15.0, d_gain_ = 1.0, wheel_d_gain_ = 0.1;
    double default_left_hip_ = -0.23, default_left_knee_ = -0.65;
    double default_right_hip_ = 0.23, default_right_knee_ = 0.65;
    double torque_limit_ = 30.0, wheel_torque_limit_ = 5.0;
    std::array<double, 6> sign_{1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    bool swap_sides_ = false;

    std::array<InputInterface<double>, 6> joint_angle_, joint_velocity_, action_;
    std::array<OutputInterface<double>, 6> motor_torque_;
    InputInterface<double> l20_angle_, r20_angle_;
    InputInterface<double> valid_, healthy_;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::WheelLegRlController, rmcs_executor::Component)
