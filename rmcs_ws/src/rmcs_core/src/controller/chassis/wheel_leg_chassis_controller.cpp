#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <numbers>

#include <eigen3/Eigen/Dense>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/chassis_mode.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/switch.hpp>

#include "controller/pid/pid_calculator.hpp"
#include "controller/chassis/wheel_leg_control_state.hpp"

namespace rmcs_core::controller::chassis {

// Chassis command source of the wheel-leg. It decodes the remote control into the chassis command
// interfaces and selects the fixed-pose or RL mode. It does not read joint feedback, solve the
// closed chain, or run the policy; those belong to hardware, WheelLegRlConsumer, and rmcs_rl.
class WheelLegChassisController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    explicit WheelLegChassisController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/rotary_knob", rotary_knob_);
        register_input("/remote/keyboard", keyboard_);

        register_input("/wheel_leg/imu/quaternion", chassis_imu_quaternion_, false);
        // 云台相对底盘的 yaw 电机角；用于"以云台系控制底盘"。
        register_input("/gimbal/yaw/angle", gimbal_yaw_angle_, false);

        register_output(
            "/chassis/control_velocity", chassis_control_velocity_,
            rmcs_description::BaseLink::DirectionVector{0.0, 0.0, 0.0});
        register_output("/chassis/control_height", chassis_control_height_, 0.0);
        register_output("/chassis/control_state", chassis_control_state_, 0);
        register_output("/chassis/reset_count", reset_count_output_, std::size_t{0});
        register_output("/wheel_leg/rl/enable", rl_enable_, false);
        register_output("/wheel_leg/joint_enable", joint_enable_, false);
        register_output("/chassis/control_mode", mode_, rmcs_msgs::ChassisMode::AUTO);
        register_output("/chassis/task_mode/stand", task_mode_[0], 1.0);
        register_output("/chassis/task_mode/move", task_mode_[1], 0.0);
        register_output("/chassis/task_mode/up", task_mode_[2], 0.0);
        register_output("/chassis/task_mode/down", task_mode_[3], 0.0);
        register_output("/chassis/task_mode/jump", task_mode_[4], 0.0);

        vx_max_ = get_parameter_or<double>("vx_max", 2.5);
        yaw_rate_max_ = get_parameter_or<double>("yaw_rate_max", 3.0);
        deadzone_ = get_parameter_or<double>("deadzone", 0.08);

        command_height_min_ = get_parameter_or<double>("command_height_min", 0.20);
        command_height_max_ = get_parameter_or<double>("command_height_max", 0.42);
        default_command_height_ = get_parameter_or<double>("default_command_height", 0.22);
        height_range_ = get_parameter_or<double>("height_range", 0.20);
        height_step_ = get_parameter_or<double>("height_step", 0.01);
        // 带回中旋钮的高度变化率 (m/s)
        height_rate_ = get_parameter_or<double>("height_rate", 0.2);

        angular_z_invert_ = get_parameter_or<bool>("angular_z_invert", false);
        height_invert_ = get_parameter_or<bool>("height_invert", false);
        heading_kp_ = get_parameter_or<double>("heading_kp", 3.0);

        // 航向参考系: false=场地系(重置时锁定世界朝向)；true=云台系(摇杆按云台朝向解释)。
        // 云台是对世界稳定的，所以云台系下底盘转向对齐时误差会收敛(不会自旋)。
        use_gimbal_frame_ = get_parameter_or<bool>("use_gimbal_frame", false);
        gimbal_yaw_sign_ = get_parameter_or<double>("gimbal_yaw_sign", 1.0);
        // 底盘前向持续跟随云台前向（摇杆居中时也继续对齐）。
        gimbal_follow_ = get_parameter_or<bool>("gimbal_follow", true);

        // 偏航 PID：输入=航向误差，输出=偏航率。yaw_ki/yaw_kd 默认 0 ⇒ 退化为纯 P（同原来）。
        yaw_pid_ = pid::PidCalculator{
            get_parameter_or<double>("yaw_kp", heading_kp_),
            get_parameter_or<double>("yaw_ki", 0.0),
            get_parameter_or<double>("yaw_kd", 0.0)};
        yaw_pid_.output_max = yaw_rate_max_;
        yaw_pid_.output_min = -yaw_rate_max_;
        // 航向固定偏移(rad)：抵消摇杆/云台零点造成的固定角偏差（如推前却偏 90°）。
        heading_offset_ = get_parameter_or<double>("heading_offset", 0.0);
        // 云台"正方向"相对 yaw 电机零点的固定角(rad)：云台正方向与底盘相反 => 设 pi。
        gimbal_forward_offset_ = get_parameter_or<double>("gimbal_forward_offset", 0.0);
        // 目标在 ±90° 之外时改为倒车（而不是掉头）。
        allow_reverse_ = get_parameter_or<bool>("allow_reverse", true);

        height_ = default_command_height_;
        last_height_time_ = std::chrono::steady_clock::now();
        stop_controls_(WheelLegControlState::kInit);
    }

    void before_updating() override {
        if (!chassis_imu_quaternion_.ready()) {
            chassis_imu_quaternion_.make_and_bind_directly(Eigen::Quaterniond::Identity());
            RCLCPP_WARN(
                get_logger(),
                "Failed to fetch \"/wheel_leg/imu/quaternion\". Set to identity; direction "
                "alignment will be disabled.");
        }
    }

    void update() override {
        const auto switch_right = *switch_right_;
        const auto switch_left = *switch_left_;
        const auto keyboard = *keyboard_;
        const auto selected_state = wheel_leg_control_state(switch_left, switch_right);

        // Power-on hold: stay at kInit(0) until the operator first moves a switch.
        if (!switch_activity_seen_
            && (switch_left != last_switch_left_ || switch_right != last_switch_right_))
            switch_activity_seen_ = true;

        if (switch_left != last_switch_left_ || switch_right != last_switch_right_)
            RCLCPP_INFO(
                get_logger(), "[wheel_leg mode] switches left=%d right=%d state=%d enable=%d",
                static_cast<int>(switch_left), static_cast<int>(switch_right),
                static_cast<int>(selected_state),
                static_cast<int>(switch_activity_seen_
                                 && selected_state != WheelLegControlState::kDisabled));

        *joint_enable_ =
            switch_activity_seen_ && selected_state != WheelLegControlState::kDisabled;

        do {
            if (!switch_activity_seen_) {
                stop_controls_(WheelLegControlState::kInit);
                break;
            }

            if (selected_state != WheelLegControlState::kDisabled)
                reset_active_ = false;

            if (selected_state == WheelLegControlState::kDisabled) {
                reset_all_controls_();
                break;
            }

            auto mode = *mode_;
            if (selected_state == WheelLegControlState::kRl) {
                if (!last_keyboard_.c && keyboard.c) {
                    if (mode != rmcs_msgs::ChassisMode::SPIN_FAST) {
                        mode = rmcs_msgs::ChassisMode::SPIN_FAST;
                        spinning_forward_ = !spinning_forward_;
                    } else {
                        mode = rmcs_msgs::ChassisMode::AUTO;
                    }
                } else if (!last_keyboard_.x && keyboard.x) {
                    mode = mode != rmcs_msgs::ChassisMode::LAUNCH_RAMP
                             ? rmcs_msgs::ChassisMode::LAUNCH_RAMP
                             : rmcs_msgs::ChassisMode::AUTO;
                } else if (!last_keyboard_.z && keyboard.z) {
                    mode = mode != rmcs_msgs::ChassisMode::STEP_DOWN
                             ? rmcs_msgs::ChassisMode::STEP_DOWN
                             : rmcs_msgs::ChassisMode::AUTO;
                }

                *mode_ = mode;
            }

            update_remote_control_(selected_state);
        } while (false);

        last_switch_left_ = switch_left;
        last_switch_right_ = switch_right;
        last_keyboard_ = keyboard;
    }

private:
    void reset_all_controls_() {
        if (!reset_active_) {
            *reset_count_output_ += 1;
            reset_active_ = true;
            spinning_forward_ = true;
            // Capture the current chassis facing as the "gimbal forward" for this session.
            reference_yaw_ = chassis_yaw_();
        }
        stop_controls_(WheelLegControlState::kDisabled);
    }

    void stop_controls_(WheelLegControlState state) {
        hold_chassis_commands_();
        yaw_pid_.reset();
        *chassis_control_state_ = static_cast<int>(state);
        *rl_enable_ = false;
        *joint_enable_ = false;
    }

    void hold_chassis_commands_() {
        chassis_control_velocity_->vector << 0.0, 0.0, 0.0;
        *chassis_control_height_ = default_command_height_;
        *mode_ = rmcs_msgs::ChassisMode::AUTO;
        *task_mode_[0] = 1.0;
        for (std::size_t i = 1; i < task_mode_.size(); ++i)
            *task_mode_[i] = 0.0;
        height_ = default_command_height_;
        height_offset_ = 0.0;
    }

    void update_remote_control_(WheelLegControlState state) {
        *chassis_control_state_ = static_cast<int>(state);
        *rl_enable_ = state == WheelLegControlState::kRl;
        if (state != WheelLegControlState::kRl) {
            hold_chassis_commands_();
            return;
        }
        update_velocity_control_();
        update_height_control_();
    }

    void update_velocity_control_() {
        const Eigen::Vector2d command = read_translational_command_();
        const double vx = update_translational_velocity_control_(command);
        const double yaw_rate = update_angular_velocity_control_(command);

        chassis_control_velocity_->vector.x() = std::clamp(vx, -vx_max_, vx_max_);
        chassis_control_velocity_->vector.y() = 0.0;
        chassis_control_velocity_->vector.z() = std::clamp(yaw_rate, -yaw_rate_max_, yaw_rate_max_);
        const bool moving = command.norm() > 1e-6;
        *task_mode_[0] = moving ? 0.0 : 1.0;
        *task_mode_[1] = moving ? 1.0 : 0.0;
        *task_mode_[2] = 0.0;
        *task_mode_[3] = 0.0;
        *task_mode_[4] = 0.0;
    }

    Eigen::Vector2d read_translational_command_() const {
        // joystick_right = (forward, left)  [dr16/vt13: x=前后, y=左右]，与标准控制器一致。
        // 之前写成 {y, x} 把两轴对调了，导致推前被当成 90°。
        Eigen::Vector2d command{joystick_right_->x(), joystick_right_->y()};
        if (std::abs(command.x()) < deadzone_)
            command.x() = 0.0;
        if (std::abs(command.y()) < deadzone_)
            command.y() = 0.0;

        const double magnitude = command.norm();
        if (magnitude > 1.0)
            command /= magnitude;
        return command;
    }

    double update_translational_velocity_control_(const Eigen::Vector2d& command) const {
        const double command_magnitude = command.norm();
        if (command_magnitude <= 1e-6)
            return 0.0;

        // A two-wheeled base cannot strafe: drive along the (possibly reversed) heading.
        const auto solution = heading_solution_(command);
        return solution.direction * command_magnitude * vx_max_ * std::cos(solution.error);
    }

    double update_angular_velocity_control_(const Eigen::Vector2d& command) {
        using rmcs_msgs::ChassisMode;

        switch (*mode_) {
        case ChassisMode::SPIN_SLOW:
            return 0.3 * (spinning_forward_ ? yaw_rate_max_ : -yaw_rate_max_);
        case ChassisMode::SPIN_FAST:
            return 0.6 * (spinning_forward_ ? yaw_rate_max_ : -yaw_rate_max_);
        default: break;
        }

        // 航向误差（摇杆居中且开启云台跟随时也可能非 0）
        const auto solution = heading_solution_(command);
        const double yaw_rate = yaw_pid_.update(solution.error);
        return angular_z_invert_ ? -yaw_rate : yaw_rate;
    }

    // Chassis yaw derived from the body x axis projected onto the horizontal plane.
    double chassis_yaw_() const {
        if (!chassis_imu_quaternion_.ready())
            return 0.0;
        const Eigen::Vector3d forward = *chassis_imu_quaternion_ * Eigen::Vector3d::UnitX();
        return std::atan2(forward.y(), forward.x());
    }

    // 摇杆方向相对参考系应转过的角度（正=逆时针，喂给偏航控制）。
    //   场地系: 相对重置瞬间锁定的世界朝向；
    //   云台系: 相对云台当前世界朝向。云台世界朝向 = 底盘世界朝向 + 云台电机角，
    //           两者相减时底盘世界朝向抵消 => 只剩云台电机角。
    double heading_error_(const Eigen::Vector2d& command) const {
        const bool gimbal_mode = use_gimbal_frame_ && gimbal_yaw_angle_.ready();
        const bool idle = command.norm() <= 1e-6;
        // 摇杆居中时：场地系 / 未开"云台跟随" => 不转；云台跟随开启 => 仍把前向对齐云台前向。
        if (idle && !(gimbal_mode && gimbal_follow_))
            return 0.0;
        // command = (forward, left) => 航向角(逆时针为正) = atan2(left, forward)
        const double stick_angle = idle ? 0.0 : std::atan2(command.y(), command.x());
        const double reference = gimbal_mode
                                   ? -(gimbal_yaw_sign_ * *gimbal_yaw_angle_ + gimbal_forward_offset_)
                                   : (chassis_yaw_() - reference_yaw_);
        return normalize_angle_(stick_angle - reference + heading_offset_);
    }

    // 把航向误差折成"前进或倒车"二选一：
    //   目标在 ±90° 之外时不掉头，而是折回 ±90° 并令 direction=-1（倒退，屁股朝目标）。
    struct HeadingSolution {
        double error;
        double direction;
    };
    HeadingSolution heading_solution_(const Eigen::Vector2d& command) const {
        double error = heading_error_(command);
        if (allow_reverse_ && std::abs(error) > std::numbers::pi / 2.0) {
            error += error > 0.0 ? -std::numbers::pi : std::numbers::pi;
            return {error, -1.0};
        }
        return {error, 1.0};
    }

    static double normalize_angle_(double angle) {
        angle = std::fmod(angle, 2.0 * std::numbers::pi);
        if (angle > std::numbers::pi)
            angle -= 2.0 * std::numbers::pi;
        else if (angle < -std::numbers::pi)
            angle += 2.0 * std::numbers::pi;
        return angle;
    }

    void update_height_control_() {
        const auto& rotary_knob = *rotary_knob_;
        const auto& keyboard = *keyboard_;

        const auto now = std::chrono::steady_clock::now();
        const double dt = std::clamp(
            std::chrono::duration<double>(now - last_height_time_).count(), 0.0, 0.1);
        last_height_time_ = now;

        if (!last_keyboard_.q && keyboard.q)
            height_offset_ -= height_step_;
        if (!last_keyboard_.e && keyboard.e)
            height_offset_ += height_step_;

        // 带回中的旋钮：偏转量 -> 高度变化率(m/s)；松开(回中)即停，保持当前高度。
        if (std::abs(rotary_knob) > 0.1)
            height_offset_ += height_rate_ * rotary_knob * dt;

        height_ = default_command_height_ + height_offset_;
        height_ = std::clamp(height_, command_height_min_, command_height_max_);
        *chassis_control_height_ = height_;
    }

    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<double> rotary_knob_;
    InputInterface<rmcs_msgs::Keyboard> keyboard_;
    InputInterface<Eigen::Quaterniond> chassis_imu_quaternion_;
    InputInterface<double> gimbal_yaw_angle_;

    OutputInterface<rmcs_description::BaseLink::DirectionVector> chassis_control_velocity_;
    OutputInterface<double> chassis_control_height_;
    OutputInterface<int> chassis_control_state_;
    OutputInterface<std::size_t> reset_count_output_;
    OutputInterface<bool> rl_enable_;
    OutputInterface<bool> joint_enable_;
    std::array<OutputInterface<double>, 5> task_mode_;

    OutputInterface<rmcs_msgs::ChassisMode> mode_;

    rmcs_msgs::Switch last_switch_left_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Keyboard last_keyboard_ = rmcs_msgs::Keyboard::zero();

    double vx_max_ = 2.5;
    double yaw_rate_max_ = 3.0;
    double deadzone_ = 0.08;
    double command_height_min_ = 0.20;
    double command_height_max_ = 0.42;
    double default_command_height_ = 0.22;
    double height_range_ = 0.20;
    double height_step_ = 0.05;
    double height_rate_ = 0.2;
    std::chrono::steady_clock::time_point last_height_time_{};
    bool angular_z_invert_ = false;
    bool height_invert_ = false;
    double heading_kp_ = 3.0;
    bool use_gimbal_frame_ = false;
    double gimbal_yaw_sign_ = 1.0;
    double heading_offset_ = 0.0;
    double gimbal_forward_offset_ = 0.0;
    bool allow_reverse_ = true;
    bool gimbal_follow_ = true;
    pid::PidCalculator yaw_pid_;

    bool spinning_forward_ = true;
    bool switch_activity_seen_ = false;
    bool reset_active_ = false;
    double reference_yaw_ = 0.0;
    double height_ = 0.0;
    double height_offset_ = 0.0;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::WheelLegChassisController, rmcs_executor::Component)
