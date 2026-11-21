#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rmcs_executor/component.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "simulate_link.hpp"

namespace rmcs_mujoco_bridge {

class TrajectoryTarget final
    : public rmcs_executor::Component
    , public rclcpp::Node {
    using Action = control_msgs::action::FollowJointTrajectory;
    using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;
    using Result = Action::Result;
    using GripperAction = control_msgs::action::GripperCommand;
    using GripperHandle = rclcpp_action::ServerGoalHandle<GripperAction>;
    static constexpr std::size_t kAxes = std::tuple_size_v<JointValues>;
    static constexpr double kOpen = 0.032;  // m
    static constexpr double kGear = -100.0; // rad/m

public:
    TrajectoryTarget()
        : Node{get_component_name()} {
        register_input("/arm/feedback_valid", feedback_valid_);
        register_input("/arm/simulation/time", simulation_time_);
        register_input("/arm/gripper/motor/angle", gripper_angle_);
        register_input("/arm/gripper/motor/velocity", gripper_velocity_);
        register_input("/arm/gripper/motor/torque", gripper_torque_);
        register_input("/arm/gripper/motor/max_torque", gripper_max_torque_);
        register_output("/arm/enable_flag", enable_, false);
        register_output("/arm/gripper/target_theta", gripper_target_, 0.0);
        for (std::size_t i = 0; i < kAxes; ++i) {
            const auto number = std::to_string(i + 1);
            const auto prefix = "/arm/joint_" + number;
            names_[i] = "joint" + number;
            register_input(prefix + "/theta", position_[i]);
            register_input(prefix + "/velocity", velocity_[i]);
            register_input(prefix + "/lower_limit", lower_[i]);
            register_input(prefix + "/upper_limit", upper_[i]);
            register_output(prefix + "/target_theta", target_[i], 0.0);
            register_output(prefix + "/target_velocity", target_velocity_[i], 0.0);
        }
        gripper_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        server_ = rclcpp_action::create_server<Action>(
            this, "/JGarm_controller/follow_joint_trajectory",
            [this](const auto&, const auto& goal) { return PrepareGoal(*goal); },
            [this](const auto&) {
                std::lock_guard lock(mutex_);
                if (!trajectory_ || outcome_)
                    return rclcpp_action::CancelResponse::REJECT;
                cancel_requested_ = true;
                return rclcpp_action::CancelResponse::ACCEPT;
            },
            [this](auto goal) { handle_ = std::move(goal); });
        gripper_server_ = rclcpp_action::create_server<GripperAction>(
            this, "/Gripper_controller/gripper_cmd",
            [this](const auto&, const auto& goal) { return PrepareGripper(goal->command); },
            [this](const auto&) {
                std::lock_guard lock(mutex_);
                if (!grip_ || grip_->outcome)
                    return rclcpp_action::CancelResponse::REJECT;
                grip_->cancel = true;
                return rclcpp_action::CancelResponse::ACCEPT;
            },
            [this](auto goal) { gripper_handle_ = std::move(goal); });
        timer_ = create_wall_timer(std::chrono::milliseconds(20), [this] { Publish(); });
    }

    void update() override {
        if (!*feedback_valid_) {
            *enable_ = false;
            interrupted_ = true;
        }
        std::unique_lock lock(mutex_, std::try_to_lock);
        if (!lock)
            return;
        const double time = *simulation_time_;
        snapshot_.valid = *feedback_valid_;
        if (interrupted_ || time < snapshot_.time) {
            ++snapshot_.epoch;
            if (trajectory_ && !outcome_)
                outcome_ = {Result::INVALID_GOAL, "MuJoCo feedback lost or simulation reset"};
            if (grip_ && !grip_->outcome)
                grip_->outcome = GripOutcome::Lost;
            *enable_ = false;
            interrupted_ = false;
        }
        snapshot_.time = time;
        if (!snapshot_.valid)
            return;

        for (std::size_t i = 0; i < kAxes; ++i) {
            snapshot_.position[i] = *position_[i];
            snapshot_.velocity[i] = *velocity_[i];
            snapshot_.lower[i] = *lower_[i];
            snapshot_.upper[i] = *upper_[i];
        }
        const double jaw = kOpen + *gripper_angle_ / kGear;
        // Treat small soft-limit overshoot like the bridge does for the arm joints.
        constexpr double kJawLimitTolerance = 1e-5; // m
        snapshot_.jaw_position = (jaw >= -kJawLimitTolerance && jaw <= kOpen + kJawLimitTolerance)
                                   ? std::clamp(jaw, 0.0, kOpen)
                                   : jaw;
        snapshot_.jaw_velocity = *gripper_velocity_ / kGear;
        snapshot_.jaw_effort = std::abs(*gripper_torque_ * kGear);
        snapshot_.jaw_effort_limit = std::abs(*gripper_max_torque_ * kGear);
        if (!*enable_) {
            WriteTarget(snapshot_.position, {});
            *gripper_target_ = *gripper_angle_;
            *enable_ = true;
        }
        if (grip_ && !grip_->outcome)
            AdvanceGripper(*grip_);
        if (!trajectory_ || outcome_)
            return;
        if (cancel_requested_) {
            Finish(Result::SUCCESSFUL, "Trajectory canceled", true);
            return;
        }
        Advance(*trajectory_);
    }

private:
    struct Point {
        double time = 0.0;
        JointValues position{}, velocity{};
        bool has_velocity = false;
    };

    struct Snapshot : Point {
        bool valid = false;
        std::uint64_t epoch = 0;
        JointValues lower{}, upper{};
        double jaw_position = 0.0, jaw_velocity = 0.0, jaw_effort = 0.0, jaw_effort_limit = 0.0;
    };

    struct Tolerance {
        JointValues position, velocity;
    };

    struct Trajectory {
        std::vector<Point> points;
        Tolerance path, goal;
        double grace;
        std::optional<double> start;
    };

    struct Outcome {
        int code;
        const char* message;
        bool canceled = false;
    };

    enum class GripOutcome { Reached, Canceled, Stalled, Lost };

    struct GripGoal {
        double position, start;
        bool cancel = false;
        std::optional<GripOutcome> outcome;
    };

    rclcpp_action::GoalResponse PrepareGripper(const control_msgs::msg::GripperCommand& command) {
        using Response = rclcpp_action::GoalResponse;
        std::lock_guard lock(mutex_);
        // The simulated actuator uses the solver's configured effort limit.
        if (!snapshot_.valid || grip_ || !std::isfinite(command.position) || command.position < 0.0
            || command.position > kOpen || !std::isfinite(command.max_effort)
            || command.max_effort < 0.0
            || (command.max_effort > 0.0 && command.max_effort < snapshot_.jaw_effort_limit))
            return Response::REJECT;
        grip_ = GripGoal{command.position, snapshot_.time, false, {}};
        return Response::ACCEPT_AND_EXECUTE;
    }

    void AdvanceGripper(GripGoal& goal) {
        *gripper_target_ = (goal.position - kOpen) * kGear;
        if (goal.cancel)
            goal.outcome = GripOutcome::Canceled;
        else if (
            std::abs(goal.position - snapshot_.jaw_position) <= 0.0003
            && std::abs(snapshot_.jaw_velocity) <= 0.002)
            goal.outcome = GripOutcome::Reached;
        else if (snapshot_.time - goal.start > 3.0)
            goal.outcome = GripOutcome::Stalled;
        if (goal.outcome && goal.outcome != GripOutcome::Reached)
            *gripper_target_ = *gripper_angle_;
    }

    static double Seconds(const builtin_interfaces::msg::Duration& duration) {
        return duration.nanosec < 1'000'000'000 ? duration.sec + 1e-9 * duration.nanosec : -1.0;
    }

    bool ApplyTolerances(
        const std::vector<control_msgs::msg::JointTolerance>& values, Tolerance& tolerance) const {
        std::array<bool, kAxes> seen{};
        const auto valid = [](double x) { return std::isfinite(x) && (x >= 0.0 || x == -1.0); };
        for (const auto& value : values) {
            const auto name = std::find(names_.begin(), names_.end(), value.name);
            if (name == names_.end() || !valid(value.position) || !valid(value.velocity)
                || (value.acceleration != 0.0 && value.acceleration != -1.0))
                return false;
            const auto i = static_cast<std::size_t>(name - names_.begin());
            if (seen[i])
                return false;
            seen[i] = true;
            if (value.position != 0.0)
                tolerance.position[i] = value.position;
            if (value.velocity != 0.0)
                tolerance.velocity[i] = value.velocity;
        }
        return true;
    }

    rclcpp_action::GoalResponse PrepareGoal(const Action::Goal& goal) {
        using Response = rclcpp_action::GoalResponse;
        const auto& joint_goal = goal.trajectory;
        const auto& stamp = joint_goal.header.stamp;
        if (joint_goal.joint_names.size() != kAxes || joint_goal.points.empty() || stamp.sec != 0
            || stamp.nanosec != 0 || !goal.multi_dof_trajectory.points.empty()
            || !goal.component_path_tolerance.empty() || !goal.component_goal_tolerance.empty())
            return Response::REJECT;

        Snapshot seed;
        {
            std::lock_guard lock(mutex_);
            if (trajectory_ || !snapshot_.valid)
                return Response::REJECT;
            seed = snapshot_;
        }
        Trajectory plan{};
        plan.grace = Seconds(goal.goal_time_tolerance);
        if (plan.grace < 0.0)
            return Response::REJECT;
        if (plan.grace == 0.0)
            plan.grace = 1.0;
        plan.path.position.fill(0.12);
        plan.path.velocity.fill(-1.0);
        plan.goal.position.fill(0.01);
        plan.goal.velocity.fill(0.05);
        if (!ApplyTolerances(goal.path_tolerance, plan.path)
            || !ApplyTolerances(goal.goal_tolerance, plan.goal))
            return Response::REJECT;

        std::array<std::size_t, kAxes> indices;
        for (std::size_t i = 0; i < kAxes; ++i) {
            const auto found =
                std::find(joint_goal.joint_names.begin(), joint_goal.joint_names.end(), names_[i]);
            if (found == joint_goal.joint_names.end())
                return Response::REJECT;
            indices[i] = static_cast<std::size_t>(found - joint_goal.joint_names.begin());
        }
        double previous = -1.0;
        plan.points.reserve(joint_goal.points.size());
        for (const auto& source : joint_goal.points) {
            const double time = Seconds(source.time_from_start);
            if (time < 0.0 || time <= previous || source.positions.size() != kAxes
                || !source.effort.empty()
                || (!source.velocities.empty() && source.velocities.size() != kAxes))
                return Response::REJECT;
            auto& point = plan.points.emplace_back();
            point.time = previous = time;
            point.has_velocity = !source.velocities.empty();
            for (std::size_t i = 0; i < kAxes; ++i) {
                const double q = source.positions[indices[i]];
                const double dq = point.has_velocity ? source.velocities[indices[i]] : 0.0;
                if (!std::isfinite(q) || q < seed.lower[i] || q > seed.upper[i]
                    || !std::isfinite(dq) || (time == 0.0 && std::abs(q - seed.position[i]) > 0.01))
                    return Response::REJECT;
                point.position[i] = q;
                point.velocity[i] = dq;
            }
        }
        std::lock_guard lock(mutex_);
        if (!snapshot_.valid || snapshot_.epoch != seed.epoch)
            return Response::REJECT;
        trajectory_ = std::move(plan);
        cancel_requested_ = false;
        return Response::ACCEPT_AND_EXECUTE;
    }

    void Advance(Trajectory& plan) {
        if (!plan.start) {
            plan.start = snapshot_.time;
            start_ = {0.0, snapshot_.position, snapshot_.velocity, true};
            if (plan.points.front().time == 0.0 && !Track(plan.points.front(), StartTolerance())) {
                Finish(Result::INVALID_GOAL, "First waypoint differs from measured state");
                return;
            }
        }
        const double elapsed = snapshot_.time - *plan.start;
        Point sample = plan.points.back();
        const bool finished = elapsed >= sample.time;
        if (finished) {
            sample.velocity.fill(0.0);
        } else {
            const auto next = std::upper_bound(
                plan.points.begin(), plan.points.end(), elapsed,
                [](double time, const Point& point) { return time < point.time; });
            const auto& first = next == plan.points.begin() ? start_ : *(next - 1);
            sample = Interpolate(first, *next, elapsed);
        }
        for (std::size_t i = 0; i < kAxes; ++i) {
            if (!std::isfinite(sample.position[i]) || !std::isfinite(sample.velocity[i])
                || sample.position[i] < snapshot_.lower[i] - 1e-9
                || sample.position[i] > snapshot_.upper[i] + 1e-9) {
                Finish(Result::INVALID_GOAL, "Interpolated reference exceeds joint limits");
                return;
            }
        }
        WriteTarget(sample.position, sample.velocity);
        if (finished && Track(sample, plan.goal))
            outcome_ = {Result::SUCCESSFUL, ""};
        else if (finished && elapsed > sample.time + plan.grace)
            Finish(Result::GOAL_TOLERANCE_VIOLATED, "Final joint feedback outside goal tolerances");
        else if (!finished && !Track(sample, plan.path))
            Finish(Result::PATH_TOLERANCE_VIOLATED, "Joint feedback outside path tolerances");
    }

    static Tolerance StartTolerance() {
        Tolerance tolerance;
        tolerance.position.fill(0.01);
        tolerance.velocity.fill(-1.0);
        return tolerance;
    }

    static Point Interpolate(const Point& a, const Point& b, double time) {
        Point sample;
        const double duration = b.time - a.time;
        const double fraction = (time - a.time) / duration;
        for (std::size_t i = 0; i < kAxes; ++i) {
            const double delta = b.position[i] - a.position[i];
            if (a.has_velocity && b.has_velocity) {
                // Hermite spline preserves the waypoint angles and angular velocities.
                const double v0 = a.velocity[i] * duration;
                const double v1 = b.velocity[i] * duration;
                const double c = 3.0 * delta - 2.0 * v0 - v1;
                const double d = v0 + v1 - 2.0 * delta;
                sample.position[i] =
                    a.position[i] + fraction * (v0 + fraction * (c + fraction * d));
                sample.velocity[i] = (v0 + fraction * (2.0 * c + 3.0 * fraction * d)) / duration;
            } else {
                sample.position[i] = std::lerp(a.position[i], b.position[i], fraction);
                sample.velocity[i] = delta / duration;
            }
        }
        return sample;
    }

    bool Track(const Point& point, const Tolerance& tolerance) const {
        for (std::size_t i = 0; i < kAxes; ++i) {
            if ((tolerance.position[i] >= 0.0
                 && std::abs(point.position[i] - snapshot_.position[i]) > tolerance.position[i])
                || (tolerance.velocity[i] >= 0.0
                    && std::abs(point.velocity[i] - snapshot_.velocity[i]) > tolerance.velocity[i]))
                return false;
        }
        return true;
    }

    void WriteTarget(const JointValues& position, const JointValues& velocity) {
        desired_.position = position;
        desired_.velocity = velocity;
        for (std::size_t i = 0; i < kAxes; ++i) {
            *target_[i] = position[i];
            *target_velocity_[i] = velocity[i];
        }
    }

    void Finish(int code, const char* message, bool canceled = false) {
        WriteTarget(snapshot_.position, {});
        outcome_ = {code, message, canceled};
    }

    void Publish() {
        Snapshot actual;
        Point desired;
        std::optional<Outcome> outcome;
        std::optional<GripOutcome> grip_outcome;
        {
            std::lock_guard lock(mutex_);
            actual = snapshot_;
            desired = desired_;
            if (handle_ && outcome_) {
                outcome = outcome_;
                outcome_.reset();
                trajectory_.reset();
            }
            if (gripper_handle_ && grip_ && grip_->outcome) {
                grip_outcome = grip_->outcome;
                grip_.reset();
            }
        }

        if (actual.valid) {
            sensor_msgs::msg::JointState jaws;
            jaws.header.stamp = now();
            jaws.name = {"joint8", "joint9"};
            jaws.position = {-actual.jaw_position, actual.jaw_position};
            jaws.velocity = {-actual.jaw_velocity, actual.jaw_velocity};
            gripper_pub_->publish(jaws);
        }
        PublishGripper(actual, grip_outcome);
        if (!handle_)
            return;
        if (outcome) {
            auto result = std::make_shared<Result>();
            result->error_code = outcome->code;
            result->error_string = outcome->message;
            if (outcome->canceled)
                handle_->canceled(result);
            else if (result->error_code == Result::SUCCESSFUL)
                handle_->succeed(result);
            else
                handle_->abort(result);
            handle_.reset();
            return;
        }
        if (!actual.valid)
            return;
        auto feedback = std::make_shared<Action::Feedback>();
        feedback->header.stamp = now();
        feedback->joint_names.assign(names_.begin(), names_.end());
        feedback->actual.positions.assign(actual.position.begin(), actual.position.end());
        feedback->actual.velocities.assign(actual.velocity.begin(), actual.velocity.end());
        feedback->desired.positions.assign(desired.position.begin(), desired.position.end());
        feedback->desired.velocities.assign(desired.velocity.begin(), desired.velocity.end());
        for (std::size_t i = 0; i < kAxes; ++i) {
            feedback->error.positions.push_back(desired.position[i] - actual.position[i]);
            feedback->error.velocities.push_back(desired.velocity[i] - actual.velocity[i]);
        }
        handle_->publish_feedback(feedback);
    }

    void PublishGripper(const Snapshot& actual, std::optional<GripOutcome> outcome) {
        if (!gripper_handle_)
            return;
        if (!outcome) {
            if (actual.valid) {
                auto feedback = std::make_shared<GripperAction::Feedback>();
                feedback->position = actual.jaw_position;
                feedback->effort = actual.jaw_effort;
                gripper_handle_->publish_feedback(feedback);
            }
            return;
        }
        auto result = std::make_shared<GripperAction::Result>();
        result->position = actual.jaw_position;
        result->effort = actual.jaw_effort;
        result->reached_goal = outcome == GripOutcome::Reached;
        result->stalled = outcome == GripOutcome::Stalled;
        if (result->reached_goal)
            gripper_handle_->succeed(result);
        else if (outcome == GripOutcome::Canceled)
            gripper_handle_->canceled(result);
        else
            gripper_handle_->abort(result);
        gripper_handle_.reset();
    }

    std::array<std::string, kAxes> names_;
    std::array<InputInterface<double>, kAxes> position_, velocity_, lower_, upper_;
    std::array<OutputInterface<double>, kAxes> target_, target_velocity_;
    InputInterface<bool> feedback_valid_;
    InputInterface<double> simulation_time_, gripper_angle_, gripper_velocity_;
    InputInterface<double> gripper_torque_, gripper_max_torque_;
    OutputInterface<bool> enable_;
    OutputInterface<double> gripper_target_;
    bool interrupted_ = false;

    std::mutex mutex_;
    Snapshot snapshot_;
    Point start_, desired_;
    std::optional<Trajectory> trajectory_;
    std::optional<Outcome> outcome_;
    std::optional<GripGoal> grip_;
    bool cancel_requested_ = false;

    std::shared_ptr<GoalHandle> handle_;
    std::shared_ptr<GripperHandle> gripper_handle_;
    rclcpp_action::Server<Action>::SharedPtr server_;
    rclcpp_action::Server<GripperAction>::SharedPtr gripper_server_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr gripper_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace rmcs_mujoco_bridge

PLUGINLIB_EXPORT_CLASS(rmcs_mujoco_bridge::TrajectoryTarget, rmcs_executor::Component)
