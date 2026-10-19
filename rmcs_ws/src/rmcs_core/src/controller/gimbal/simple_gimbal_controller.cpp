#include <algorithm>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/switch.hpp>

#include "controller/gimbal/two_axis_gimbal_solver.hpp"

namespace rmcs_core::controller::gimbal {

using namespace rmcs_description;

class SimpleGimbalController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    SimpleGimbalController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , two_axis_gimbal_solver{
              *this,
              get_parameter("upper_limit").as_double(),
              get_parameter("lower_limit").as_double(),
          } {
        get_parameter("shift_control_clamp", shift_control_clamp_);
        get_parameter("joystick_left_bias_y", joystick_left_bias_y_);
        get_parameter("joystick_left_bias_x", joystick_left_bias_x_);
        get_parameter("depond_motors", depond_motors_);

        for (const auto& motor_ : depond_motors_) {
            auto motor_alive_input = std::make_unique<InputInterface<bool>>();
            register_input(motor_ + "/alive", *motor_alive_input);
            depond_motor_alive_inputs_[motor_] = std::move(motor_alive_input);
        }

        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/mouse/velocity", mouse_velocity_);
        register_input("/remote/mouse", mouse_);

        register_input("/auto_aim/should_control", auto_aim_should_control_, false);
        register_input("/auto_aim/control_direction", auto_aim_control_direction_, false);

        register_output("/gimbal/yaw/control_angle_error", yaw_angle_error_, nan_);
        register_output("/gimbal/pitch/control_angle_error", pitch_angle_error_, nan_);

        two_axis_gimbal_solver.enable_yaw_limit(
            *this, get_parameter("yaw_upper_limit").as_double(),
            get_parameter("yaw_lower_limit").as_double());
    }

    void update() override {
        auto angle_error = calculate_angle_error();
        *yaw_angle_error_ = angle_error.yaw_angle_error;
        *pitch_angle_error_ = angle_error.pitch_angle_error;
    }

    double value_abs_clamp(double value) {
        return std::copysign(std::min(std::abs(value), shift_control_clamp_), value);
    }

    TwoAxisGimbalSolver::AngleError calculate_angle_error() {
        auto switch_right = *switch_right_;
        auto switch_left = *switch_left_;

        using namespace rmcs_msgs;
        if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
            || (switch_left == Switch::DOWN && switch_right == Switch::DOWN))
            return two_axis_gimbal_solver.update(TwoAxisGimbalSolver::SetDisabled());

        // Disable gimbal control while a depended motor is offline, otherwise a
        // stale feedback would drive the gimbal with an unbounded error.
        for (const auto& alive_input : depond_motor_alive_inputs_) {
            if (!**alive_input.second) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 2000, "%s is not alive, disabling gimbal control.",
                    alive_input.first.c_str());
                return two_axis_gimbal_solver.update(TwoAxisGimbalSolver::SetDisabled());
            }
        }

        const auto auto_aim_requested = mouse_->right || switch_right == Switch::UP;
        const auto should_control = auto_aim_should_control_.ready() && *auto_aim_should_control_;
        const auto valid_control =
            auto_aim_control_direction_.ready() && auto_aim_control_direction_->allFinite();

        if (auto_aim_requested && should_control && valid_control) {
            return two_axis_gimbal_solver.update(
                TwoAxisGimbalSolver::SetControlDirection(
                    OdomImu::DirectionVector(*auto_aim_control_direction_)));
        }

        if (!two_axis_gimbal_solver.enabled())
            return two_axis_gimbal_solver.update(TwoAxisGimbalSolver::SetToLevel());

        constexpr double joystick_sensitivity = 0.004;
        constexpr double mouse_sensitivity = 0.5;

        double yaw_shift = value_abs_clamp(
            joystick_sensitivity * (joystick_left_->y() - joystick_left_bias_y_)
            + mouse_sensitivity * mouse_velocity_->y());
        double pitch_shift = value_abs_clamp(
            -joystick_sensitivity * (joystick_left_->x() - joystick_left_bias_x_)
            + mouse_sensitivity * mouse_velocity_->x());

        return two_axis_gimbal_solver.update(
            TwoAxisGimbalSolver::SetControlShift(yaw_shift, pitch_shift));
    }

private:
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<Eigen::Vector2d> mouse_velocity_;
    InputInterface<rmcs_msgs::Mouse> mouse_;

    InputInterface<bool> auto_aim_should_control_;
    InputInterface<Eigen::Vector3d> auto_aim_control_direction_;

    TwoAxisGimbalSolver two_axis_gimbal_solver;

    OutputInterface<double> yaw_angle_error_, pitch_angle_error_;

    double joystick_left_bias_y_ = 0.0;
    double joystick_left_bias_x_ = 0.0;
    double shift_control_clamp_ = 0.01;
    std::vector<std::string> depond_motors_ = {};
    std::map<std::string, std::unique_ptr<InputInterface<bool>>> depond_motor_alive_inputs_ = {};
};

} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::gimbal::SimpleGimbalController, rmcs_executor::Component)
