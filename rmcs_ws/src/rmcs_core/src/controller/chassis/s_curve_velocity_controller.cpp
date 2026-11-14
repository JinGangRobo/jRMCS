#include <algorithm>
#include <cmath>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::chassis {

namespace {

// Single-axis S-curve velocity planner.
//
// This is the velocity-reference shaper used on one steering wheel:
//
//   velocity target -> acceleration/jerk limited velocity reference
//
// Behaviour:
//   * The velocity error is converted to a desired acceleration through
//     `time_constant`.
//   * The acceleration is clamped to `max_acceleration`; below `zero_velocity`
//     it is clamped to `zero_acceleration`, so the wheel crosses zero slowly.
//   * The acceleration itself is rate-limited by `max_jerk`.
//   * If the next step would overshoot the target, the reference snaps to the
//     target and the acceleration is cleared.
class SCurveVelocityPlanner {
public:
    struct Config {
        double max_acceleration = 80.0;  // rad/s^2
        double max_jerk = 800.0;         // rad/s^3
        double zero_velocity = 0.5;      // rad/s
        double zero_acceleration = 10.0; // rad/s^2
        double time_constant = 0.03;     // s
    };

    SCurveVelocityPlanner() = default;

    explicit SCurveVelocityPlanner(const Config& config)
        : config_(config) {}

    // Reconfigure the limits without resetting the current velocity state.
    void configure(const Config& config) { config_ = config; }

    void reset(double velocity = 0.0) {
        velocity_ = velocity;
        acceleration_ = 0.0;
    }

    double update(double target, double dt) {
        if (!(dt > 0.0) || !(config_.time_constant > 0.0))
            return velocity_;

        const double acceleration_limit = (std::abs(velocity_) < config_.zero_velocity)
                                            ? config_.zero_acceleration
                                            : config_.max_acceleration;

        double desired_acceleration = (target - velocity_) / config_.time_constant;
        desired_acceleration =
            std::clamp(desired_acceleration, -acceleration_limit, acceleration_limit);

        acceleration_ += std::clamp(
            desired_acceleration - acceleration_, -config_.max_jerk * dt, config_.max_jerk * dt);

        const double next_velocity = velocity_ + acceleration_ * dt;
        if ((target - velocity_) * (target - next_velocity) <= 0.0) {
            velocity_ = target;
            acceleration_ = 0.0;
            return velocity_;
        }

        velocity_ = next_velocity;
        return velocity_;
    }

    double velocity() const { return velocity_; }

private:
    Config config_{};
    double velocity_ = 0.0;
    double acceleration_ = 0.0;
};

} // namespace

// ROS component wrapper around SCurveVelocityPlanner.
//
// It shapes a single velocity target; use one instance per steering wheel (or
// remap `target_velocity` / `planned_velocity` for each axis).
class SCurveVelocityController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    SCurveVelocityController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , planner_(load_config())
        , dt_(get_parameter_or("dt", 0.001)) {

        register_input(get_parameter("target_velocity").as_string(), target_velocity_);
        register_output(get_parameter("planned_velocity").as_string(), planned_velocity_, 0.0);
    }

    void update() override {
        const double target = *target_velocity_;
        if (!std::isfinite(target) || !(dt_ > 0.0)) {
            planner_.reset();
            *planned_velocity_ = 0.0;
            return;
        }

        *planned_velocity_ = planner_.update(target, dt_);
    }

private:
    SCurveVelocityPlanner::Config load_config() {
        SCurveVelocityPlanner::Config config;
        config.max_acceleration = get_parameter_or("max_acceleration", 80.0);
        config.max_jerk = get_parameter_or("max_jerk", 800.0);
        config.zero_velocity = get_parameter_or("zero_velocity", 0.5);
        config.zero_acceleration = get_parameter_or("zero_acceleration", 10.0);
        config.time_constant = get_parameter_or("time_constant", 0.03);
        return config;
    }

    InputInterface<double> target_velocity_;
    OutputInterface<double> planned_velocity_;
    SCurveVelocityPlanner planner_;
    double dt_ = 0.001;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::SCurveVelocityController, rmcs_executor::Component)
