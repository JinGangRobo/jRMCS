#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/utilities.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_mujoco_bridge {

class PositionTest final
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    PositionTest()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , duration_(get_parameter("duration").as_double()) {
        const auto target = get_parameter("target_position").as_double_array();
        if (target.size() != kJointCount || !std::isfinite(duration_) || duration_ <= 0.0
            || !std::all_of(
                target.begin(), target.end(), [](double q) { return std::isfinite(q); }))
            throw std::invalid_argument(
                "Position test requires a positive duration and seven finite angles");
        std::copy(target.begin(), target.end(), target_.begin());

        register_input("/arm/feedback_valid", feedback_valid_);
        register_input("/arm/simulation/time", time_);
        register_output("/arm/enable_flag", enable_, false);
        for (std::size_t i = 0; i < kJointCount; ++i) {
            const std::string prefix = "/arm/joint_" + std::to_string(i + 1);
            register_input(prefix + "/theta", position_[i]);
            register_input(prefix + "/lower_limit", lower_limit_[i]);
            register_input(prefix + "/upper_limit", upper_limit_[i]);
            register_input(prefix + "/torque", torque_[i]);
            register_input(prefix + "/torque_limit", torque_limit_[i]);
            register_output(prefix + "/target_theta", target_position_[i], target_[i]);
            register_output(prefix + "/target_velocity", target_velocity_[i], 0.0);
        }
    }

    void update() override {
        if (!*enable_) {
            if (!*feedback_valid_)
                return;
            for (std::size_t i = 0; i < kJointCount; ++i) {
                if (target_[i] <= *lower_limit_[i] || target_[i] >= *upper_limit_[i])
                    throw std::invalid_argument(
                        "Simulation target must be inside the joint limits");
                direction_[i] = target_[i] > *position_[i] ? 1.0
                              : target_[i] < *position_[i] ? -1.0
                                                           : 0.0;
            }
            *enable_ = true;
            return;
        }

        if (!*feedback_valid_) {
            *enable_ = false;
            throw std::runtime_error("Position test lost valid joint feedback");
        }

        ++samples_;
        for (std::size_t i = 0; i < kJointCount; ++i) {
            const double error = *position_[i] - target_[i];
            const double torque = std::abs(*torque_[i]);
            overshoot_[i] = std::max(overshoot_[i], direction_[i] * error);
            peak_torque_[i] = std::max(peak_torque_[i], torque);
            if (torque >= *torque_limit_[i])
                ++saturated_samples_[i];
        }
        if (*time_ < duration_)
            return;

        *enable_ = false;
        for (std::size_t i = 0; i < kJointCount; ++i) {
            const double saturated = 100.0 * saturated_samples_[i] / samples_;
            RCLCPP_INFO(
                get_logger(),
                "J%zu final_error=%+.6f rad overshoot=%.6f rad peak_torque=%.4f N*m "
                "saturated=%.2f%%",
                i + 1, *position_[i] - target_[i], overshoot_[i], peak_torque_[i], saturated);
        }
        RCLCPP_INFO(get_logger(), "Position test completed: %.3f s / %zu steps", *time_, samples_);
        rclcpp::shutdown();
    }

private:
    static constexpr std::size_t kJointCount = 7;
    using JointValues = std::array<double, kJointCount>;

    const double duration_;
    JointValues target_, direction_{}, overshoot_{}, peak_torque_{};
    std::array<std::size_t, kJointCount> saturated_samples_{};
    std::size_t samples_ = 0;
    InputInterface<bool> feedback_valid_;
    InputInterface<double> time_;
    OutputInterface<bool> enable_;
    std::array<InputInterface<double>, kJointCount> position_, lower_limit_, upper_limit_;
    std::array<InputInterface<double>, kJointCount> torque_, torque_limit_;
    std::array<OutputInterface<double>, kJointCount> target_position_, target_velocity_;
};

} // namespace rmcs_mujoco_bridge

PLUGINLIB_EXPORT_CLASS(rmcs_mujoco_bridge::PositionTest, rmcs_executor::Component)
