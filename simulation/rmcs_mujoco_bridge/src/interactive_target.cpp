#include <algorithm>
#include <array>
#include <cmath>
#include <mutex>
#include <stdexcept>
#include <string>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "mujoco_model.hpp"

namespace rmcs_mujoco_bridge {

class InteractiveTarget final
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    InteractiveTarget()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , max_speed_(get_parameter_or("max_speed", 0.5)) {
        if (!std::isfinite(max_speed_) || max_speed_ <= 0.0)
            throw std::invalid_argument("max_speed must be positive and finite");

        register_input("/predefined/update_rate", update_rate_);
        register_input("/arm/feedback_valid", feedback_valid_);
        register_input("/mujoco/native_target_position", native_target_, false);
        register_input("/mujoco/native_gripper_target", native_gripper_target_, false);
        register_output("/arm/enable_flag", enable_, false);
        register_output("/arm/gripper/target_theta", gripper_target_theta_, 0.0);
        for (std::size_t i = 0; i < kJointCount; ++i) {
            const std::string prefix = "/arm/joint_" + std::to_string(i + 1);
            register_input(prefix + "/theta", position_[i]);
            register_input(prefix + "/lower_limit", lower_limit_[i]);
            register_input(prefix + "/upper_limit", upper_limit_[i]);
            register_output(prefix + "/target_theta", target_theta_[i], 0.0);
            register_output(prefix + "/target_velocity", target_velocity_[i], 0.0);
        }

        subscription_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "/mujoco/target_position", rclcpp::QoS(1),
            [this](const std_msgs::msg::Float64MultiArray& message) {
                const auto& q = message.data;
                if (q.size() != kJointCount || !std::all_of(q.begin(), q.end(), [](double angle) {
                        return std::isfinite(angle);
                    }))
                    return;
                std::lock_guard lock(command_mutex_);
                std::copy(q.begin(), q.end(), pending_.begin());
                pending_valid_ = true;
            });
    }

    void update() override {
        if (!*feedback_valid_) {
            *enable_ = false;
            return;
        }
        if (!*enable_) {
            for (std::size_t i = 0; i < kJointCount; ++i) {
                requested_[i] = *target_theta_[i] = *position_[i];
                *target_velocity_[i] = 0.0;
            }
            *enable_ = true;
        }

        if (native_target_.ready()) {
            SetRequested(*native_target_);
        } else {
            std::unique_lock lock(command_mutex_, std::try_to_lock);
            if (lock && pending_valid_) {
                SetRequested(pending_);
                pending_valid_ = false;
            }
        }
        if (native_gripper_target_.ready())
            *gripper_target_theta_ = *native_gripper_target_;

        const double rate = *update_rate_;
        const double max_step = max_speed_ / rate;
        for (std::size_t i = 0; i < kJointCount; ++i) {
            const double step = std::clamp(requested_[i] - *target_theta_[i], -max_step, max_step);
            *target_theta_[i] += step;
            *target_velocity_[i] = step * rate;
        }
    }

private:
    void SetRequested(const std::array<double, kJointCount>& target) {
        for (std::size_t i = 0; i < kJointCount; ++i) {
            if (target[i] != requested_[i]) {
                requested_[i] = std::clamp(
                    target[i], *lower_limit_[i] + kTargetMargin, *upper_limit_[i] - kTargetMargin);
            }
        }
    }

    const double max_speed_;
    std::mutex command_mutex_;
    std::array<double, kJointCount> pending_{}, requested_{};
    bool pending_valid_ = false;
    InputInterface<double> update_rate_;
    InputInterface<bool> feedback_valid_;
    InputInterface<std::array<double, kJointCount>> native_target_;
    InputInterface<double> native_gripper_target_;
    OutputInterface<bool> enable_;
    OutputInterface<double> gripper_target_theta_;
    std::array<InputInterface<double>, kJointCount> position_, lower_limit_, upper_limit_;
    std::array<OutputInterface<double>, kJointCount> target_theta_, target_velocity_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
};

} // namespace rmcs_mujoco_bridge

PLUGINLIB_EXPORT_CLASS(rmcs_mujoco_bridge::InteractiveTarget, rmcs_executor::Component)
