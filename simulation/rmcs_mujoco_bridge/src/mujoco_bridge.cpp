#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <mujoco/mujoco.h>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <std_msgs/msg/string.hpp>
#include <tinyxml2.h>

#include "mujoco_model.hpp"
#include "simulate_link.hpp"

namespace rmcs_mujoco_bridge {

class MuJoCoBridge final
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    MuJoCoBridge()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)} {
        const auto initial = get_parameter("initial_position").as_double_array();
        const auto limits = get_parameter("torque_limits").as_double_array();
        if (initial.size() != kJointCount || limits.size() != kJointCount)
            throw std::invalid_argument(
                "initial_position and torque_limits must have seven values");

        const std::string description = LoadDescription(limits);
        model_ = LoadModel(description);
        data_ = MakeData(*model_);
        if (!get_parameter_or("gravity", false))
            std::fill_n(model_->opt.gravity, 3, 0.0);
        // 首轮只测关节跟踪，隔离原始网格重叠产生的接触力。
        model_->opt.disableflags |= mjDSBL_CONTACT;

        for (std::size_t i = 0; i < kJointCount; ++i) {
            const std::string number = std::to_string(i + 1);
            const int joint = mj_name2id(model_.get(), mjOBJ_JOINT, ("joint" + number).c_str());
            if (joint < 0 || model_->jnt_type[joint] != mjJNT_HINGE || !model_->jnt_limited[joint])
                throw std::runtime_error("Expected a bounded revolute joint: joint" + number);
            if (!std::isfinite(initial[i]) || initial[i] < model_->jnt_range[2 * joint]
                || initial[i] > model_->jnt_range[2 * joint + 1])
                throw std::invalid_argument(
                    "Initial angle must be inside the limits of joint" + number);

            auto& axis = axes_[i];
            axis.position_address = model_->jnt_qposadr[joint];
            axis.velocity_address = model_->jnt_dofadr[joint];
            axis.lower = model_->jnt_range[2 * joint];
            axis.upper = model_->jnt_range[2 * joint + 1];
            axis.torque_limit = limits[i];
            data_->qpos[axis.position_address] = initial[i];

            const std::string prefix = "/arm/joint_" + number + "/motor";
            register_output(prefix + "/angle", axis.angle, initial[i]);
            register_output(prefix + "/velocity", axis.velocity, 0.0);
            register_output(prefix + "/torque", axis.torque, 0.0);
            register_output(prefix + "/max_torque", axis.max_torque, limits[i]);
        }
        mj_forward(model_.get(), data_.get());
        if (get_parameter_or("external_simulate", false)) {
            simulate_link_.emplace(kRmcsPort, kSimulatePort);
            register_output("/mujoco/native_target_position", native_target_, JointValues{});
            register_output("/mujoco/native_gripper_target", native_gripper_target_, 0.0);
            register_output("/arm/gripper/motor/angle", gripper_angle_, 0.0);
            register_output("/arm/gripper/motor/velocity", gripper_velocity_, 0.0);
            register_output("/arm/gripper/motor/torque", gripper_torque_, 0.0);
            register_output(
                "/arm/gripper/motor/max_torque", gripper_max_torque_, kGripperTorqueLimit);
        }
        register_output("/arm/simulation/time", simulation_time_, 0.0);
        register_output("/arm/simulation/gravity", simulation_gravity_, std::array<double, 3>{});
        command_ = create_partner_component<Command>(get_component_name() + "_command", *this);

        description_pub_ = create_publisher<std_msgs::msg::String>(
            "/robot_description", rclcpp::QoS(1).transient_local().reliable());
        std_msgs::msg::String message;
        message.data = description;
        description_pub_->publish(message);
    }

    void update() override {
        if (simulate_link_) {
            ReadNativeState();
            return;
        }
        for (auto& axis : axes_) {
            *axis.angle = FeedbackAngle(axis, data_->qpos[axis.position_address]);
            *axis.velocity = data_->qvel[axis.velocity_address];
            *axis.torque = data_->qfrc_applied[axis.velocity_address];
        }
        *simulation_time_ = data_->time;
        std::copy_n(model_->opt.gravity, simulation_gravity_->size(), simulation_gravity_->begin());
    }

private:
    using Clock = std::chrono::steady_clock;
    static constexpr double kGripperTorqueLimit = 0.05; // N·m，仅用于仿真。

    struct Axis {
        int position_address;
        int velocity_address;
        double lower;
        double upper;
        double torque_limit;
        OutputInterface<double> angle, velocity, torque, max_torque;
    };

    static double FeedbackAngle(const Axis& axis, double angle) {
        // 只吸收 MuJoCo 软限位的微小越界，较大越界仍由 ArmConfig 判为无效。
        constexpr double kLimitTolerance = 1e-5;
        if (angle < axis.lower && axis.lower - angle <= kLimitTolerance)
            return axis.lower;
        if (angle > axis.upper && angle - axis.upper <= kLimitTolerance)
            return axis.upper;
        return angle;
    }

    void ReadNativeState() {
        SimulateState sample;
        if (simulate_link_->ReceiveLatest(sample)) {
            const auto finite = [](const auto& values) {
                return std::all_of(
                    values.begin(), values.end(), [](double x) { return std::isfinite(x); });
            };
            if (std::isfinite(sample.time) && finite(sample.position) && finite(sample.velocity)
                && finite(sample.torque) && finite(sample.target) && finite(sample.gravity)
                && std::isfinite(sample.gripper_angle) && std::isfinite(sample.gripper_velocity)
                && std::isfinite(sample.gripper_torque) && std::isfinite(sample.gripper_target)) {
                last_native_state_ = sample;
                last_native_feedback_ = Clock::now();
            }
        }
        const bool fresh = Clock::now() - last_native_feedback_ < std::chrono::milliseconds(100);
        for (std::size_t i = 0; i < kJointCount; ++i) {
            auto& axis = axes_[i];
            *axis.angle = fresh ? FeedbackAngle(axis, last_native_state_.position[i])
                                : std::numeric_limits<double>::quiet_NaN();
            *axis.velocity = fresh ? last_native_state_.velocity[i] : 0.0;
            *axis.torque = fresh ? last_native_state_.torque[i] : 0.0;
        }
        *native_target_ = last_native_state_.target;
        *simulation_time_ = last_native_state_.time;
        *simulation_gravity_ = last_native_state_.gravity;
        *gripper_angle_ =
            fresh ? last_native_state_.gripper_angle : std::numeric_limits<double>::quiet_NaN();
        *gripper_velocity_ = fresh ? last_native_state_.gripper_velocity : 0.0;
        *gripper_torque_ = fresh ? last_native_state_.gripper_torque : 0.0;
        *native_gripper_target_ = last_native_state_.gripper_target;
    }

    void SendNativeTorque(double gripper_torque) const {
        SimulateTorque torque;
        for (std::size_t i = 0; i < kJointCount; ++i)
            torque.arm[i] = data_->qfrc_applied[axes_[i].velocity_address];
        torque.gripper = std::isfinite(gripper_torque)
                           ? std::clamp(gripper_torque, -kGripperTorqueLimit, kGripperTorqueLimit)
                           : 0.0;
        simulate_link_->Send(torque);
    }

    class Command final : public rmcs_executor::Component {
    public:
        explicit Command(MuJoCoBridge& bridge)
            : bridge_(bridge) {
            register_input("/arm/enable_flag", enable_);
            register_input("/arm/feedback_valid", feedback_valid_);
            register_input("/predefined/update_rate", update_rate_);
            for (std::size_t i = 0; i < kJointCount; ++i)
                register_input(
                    "/arm/joint_" + std::to_string(i + 1) + "/motor/control_torque", torque_[i]);
            if (bridge_.simulate_link_)
                register_input("/arm/gripper/motor/control_torque", gripper_torque_, false);
        }

        void update() override {
            if (!*enable_ || !*feedback_valid_) {
                for (const auto& axis : bridge_.axes_)
                    bridge_.data_->qfrc_applied[axis.velocity_address] = 0.0;
                first_group_ = false;
                if (bridge_.simulate_link_)
                    bridge_.SendNativeTorque(0.0);
                return;
            }

            // 沿用实机的发送分组，各轴在两次更新间保持上一拍力矩。
            const std::size_t begin = first_group_ ? 0 : 4;
            const std::size_t end = first_group_ ? 4 : kJointCount;
            for (std::size_t i = begin; i < end; ++i) {
                const auto& axis = bridge_.axes_[i];
                const double torque = *torque_[i];
                bridge_.data_->qfrc_applied[axis.velocity_address] =
                    std::isfinite(torque)
                        ? std::clamp(torque, -axis.torque_limit, axis.torque_limit)
                        : 0.0;
            }
            first_group_ = !first_group_;
            if (bridge_.simulate_link_) {
                bridge_.SendNativeTorque(gripper_torque_.ready() ? *gripper_torque_ : 0.0);
                return;
            }
            bridge_.model_->opt.timestep = 1.0 / *update_rate_;
            const double previous_time = bridge_.data_->time;
            mj_step(bridge_.model_.get(), bridge_.data_.get());
            if (!(bridge_.data_->time > previous_time)
                || std::any_of(
                    bridge_.data_->warning, bridge_.data_->warning + mjNWARNING,
                    [](const mjWarningStat& warning) { return warning.number != 0; }))
                throw std::runtime_error(
                    "MuJoCo reported a physics warning or reset during stepping");
        }

    private:
        MuJoCoBridge& bridge_;
        InputInterface<bool> enable_, feedback_valid_;
        InputInterface<double> update_rate_;
        std::array<InputInterface<double>, kJointCount> torque_;
        InputInterface<double> gripper_torque_;
        bool first_group_ = false;
    };

    static std::string LoadDescription(const std::vector<double>& limits) {
        const auto share =
            std::filesystem::path{ament_index_cpp::get_package_share_directory("arm_description")};
        tinyxml2::XMLDocument xml;
        if (xml.LoadFile((share / "urdf/JGarm.urdf").c_str()) != tinyxml2::XML_SUCCESS)
            throw std::runtime_error(std::string{"Cannot read JGarm.urdf: "} + xml.ErrorStr());
        auto* robot = xml.FirstChildElement("robot");
        if (!robot)
            throw std::runtime_error("JGarm.urdf has no robot element");

        auto find_joint = [robot](std::string_view name) {
            for (auto* joint = robot->FirstChildElement("joint"); joint;
                 joint = joint->NextSiblingElement("joint")) {
                if (const char* value = joint->Attribute("name"); value && name == value)
                    return joint;
            }
            throw std::runtime_error("Joint missing from JGarm.urdf: " + std::string{name});
        };
        for (std::size_t i = 0; i < kJointCount; ++i) {
            auto* limit = find_joint("joint" + std::to_string(i + 1))->FirstChildElement("limit");
            if (!limit || !std::isfinite(limits[i]) || limits[i] <= 0.0)
                throw std::invalid_argument("Seven positive simulation torque limits are required");
            limit->SetAttribute("effort", limits[i]);
        }
        constexpr std::string_view kPackageUri = "package://arm_description/";
        for (auto* link = robot->FirstChildElement("link"); link;
             link = link->NextSiblingElement("link")) {
            for (auto* shape = link->FirstChildElement(); shape;
                 shape = shape->NextSiblingElement()) {
                auto* geometry = shape->FirstChildElement("geometry");
                auto* mesh = geometry ? geometry->FirstChildElement("mesh") : nullptr;
                const char* filename = mesh ? mesh->Attribute("filename") : nullptr;
                if (filename && std::string_view{filename}.starts_with(kPackageUri))
                    mesh->SetAttribute(
                        "filename", (share / (filename + kPackageUri.size())).c_str());
            }
        }
        auto* mujoco = robot->FirstChildElement("mujoco");
        if (!mujoco)
            mujoco = robot->InsertNewChildElement("mujoco");
        auto* compiler = mujoco->FirstChildElement("compiler");
        if (!compiler)
            compiler = mujoco->InsertNewChildElement("compiler");
        compiler->SetAttribute("strippath", false);

        tinyxml2::XMLPrinter printer;
        xml.Print(&printer);
        return printer.CStr();
    }

    ModelPtr model_{nullptr, mj_deleteModel};
    DataPtr data_{nullptr, mj_deleteData};
    std::array<Axis, kJointCount> axes_{};
    OutputInterface<double> simulation_time_;
    OutputInterface<std::array<double, 3>> simulation_gravity_;
    OutputInterface<JointValues> native_target_;
    OutputInterface<double> native_gripper_target_;
    OutputInterface<double> gripper_angle_, gripper_velocity_, gripper_torque_, gripper_max_torque_;
    std::optional<SimulateLink> simulate_link_;
    SimulateState last_native_state_{};
    Clock::time_point last_native_feedback_{};
    std::shared_ptr<Command> command_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr description_pub_;
};

} // namespace rmcs_mujoco_bridge

PLUGINLIB_EXPORT_CLASS(rmcs_mujoco_bridge::MuJoCoBridge, rmcs_executor::Component)
