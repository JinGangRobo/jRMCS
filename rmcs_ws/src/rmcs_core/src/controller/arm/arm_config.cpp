#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <eigen3/Eigen/Dense>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <urdf/model.h>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::arm {

class ArmConfig
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    explicit ArmConfig()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , link{
              Link(*this, "/arm/link_1"), Link(*this, "/arm/link_2"),
              Link(*this, "/arm/link_3"), Link(*this, "/arm/link_4"),
              Link(*this, "/arm/link_5"), Link(*this, "/arm/link_6"),
              Link(*this, "/arm/link_7"),
          }
        , joint{
              Joint(*this, "/arm/joint_1"), Joint(*this, "/arm/joint_2"),
              Joint(*this, "/arm/joint_3"), Joint(*this, "/arm/joint_4"),
              Joint(*this, "/arm/joint_5"), Joint(*this, "/arm/joint_6"),
              Joint(*this, "/arm/joint_7"),
          } {
        for (std::size_t i = 0; i < kJointCount; ++i) {
            joint_names_[i] = "joint" + std::to_string(i + 1);
        }

        arm_urdf_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/robot_description", rclcpp::QoS(1).transient_local().reliable(),
            [this](const std_msgs::msg::String::ConstSharedPtr& msg) { load_urdf(msg); });
        joint_states_pub_ =
            this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

        for (std::size_t i = 0; i < kJointCount; ++i) {
            register_input(
                "/arm/joint_" + std::to_string(i + 1) + "/motor/angle", joint_motor_angle_[i]);
            register_input(
                "/arm/joint_" + std::to_string(i + 1) + "/motor/velocity",
                joint_motor_velocity_[i]);
            register_input(
                "/arm/joint_" + std::to_string(i + 1) + "/motor/torque", joint_motor_torque_[i]);
        }
        register_output("urdf_loaded", is_load_, false);
    }

    void update() override {
        std::lock_guard<std::mutex> lock(data_mutex_);

        for (std::size_t i = 0; i < kJointCount; ++i) {
            joint[i].update(
                *joint_motor_angle_[i], *joint_motor_velocity_[i], *joint_motor_torque_[i]);
        }

        if (!*is_load_)
            return;

        if (++joint_states_divider_ < kJointStatesDivider)
            return;
        joint_states_divider_ = 0;

        sensor_msgs::msg::JointState msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "base_link";
        msg.name.resize(kJointCount);
        msg.position.resize(kJointCount);
        msg.velocity.resize(kJointCount);

        for (std::size_t i = 0; i < kJointCount; ++i) {
            msg.name[i] = joint_names_[i];
            msg.position[i] = *joint_motor_angle_[i];
            msg.velocity[i] = *joint_motor_velocity_[i];
        }
        joint_states_pub_->publish(msg);
    }

private:
    static constexpr std::size_t kJointCount = 7;
    static constexpr std::size_t kJointStatesDivider = 9;

    static std::string link_candidate(std::size_t index, std::size_t candidate) {
        if (candidate == 0)
            return "link_" + std::to_string(index + 1);
        return "Link" + std::to_string(index + 1);
    }

    static std::string joint_candidate(std::size_t index, std::size_t candidate) {
        if (candidate == 0)
            return "joint_" + std::to_string(index + 1);
        return "joint" + std::to_string(index + 1);
    }

    static urdf::LinkConstSharedPtr find_link(const urdf::Model& model, std::size_t index) {
        for (std::size_t candidate = 0; candidate < 2; ++candidate) {
            auto link = model.getLink(link_candidate(index, candidate));
            if (link)
                return link;
        }
        return nullptr;
    }

    static urdf::JointConstSharedPtr find_joint(const urdf::Model& model, std::size_t index) {
        for (std::size_t candidate = 0; candidate < 2; ++candidate) {
            auto joint = model.getJoint(joint_candidate(index, candidate));
            if (joint)
                return joint;
        }
        return nullptr;
    }

    // Kept for compatibility with the original arm interface. The MDH-based
    // arm_solver should consume the MDH table directly instead of these values.
    void modify_link_length(const urdf::Model& model) {
        const auto joint2 = find_joint(model, 1);
        const auto joint3 = find_joint(model, 2);
        const auto joint4 = find_joint(model, 3);
        const auto joint5 = find_joint(model, 4);
        const auto joint6 = find_joint(model, 5);
        const auto joint7 = find_joint(model, 6);
        if (!joint2 || !joint3 || !joint4 || !joint5 || !joint6 || !joint7)
            return;

        link[0].load_length(joint2->parent_to_joint_origin_transform.position.z);
        link[1].load_length(joint3->parent_to_joint_origin_transform.position.y);
        link[2].load_length(joint4->parent_to_joint_origin_transform.position.x);
        link[3].load_length(
            joint4->parent_to_joint_origin_transform.position.y
            + joint5->parent_to_joint_origin_transform.position.z);
        link[4].load_length(0.0);
        link[5].load_length(joint6->parent_to_joint_origin_transform.position.y);
        link[6].load_length(joint7->parent_to_joint_origin_transform.position.y);
    }

    class Link {
    public:
        explicit Link(rmcs_executor::Component& status_component, const std::string& name_prefix) {
            status_component.register_output(
                name_prefix + "/com", com_,
                Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN()));
            status_component.register_output(name_prefix + "/length", length_, NAN);
            status_component.register_output(
                name_prefix + "/inertia", inertia_,
                Eigen::Matrix3d::Constant(std::numeric_limits<double>::quiet_NaN()));
            status_component.register_output(name_prefix + "/mass", mass_, NAN);
        }

        double get_mass() const { return *mass_; }
        double get_length() const { return *length_; }
        Eigen::Vector3d get_com() const { return *com_; }
        Eigen::Matrix3d get_inertia() const { return *inertia_; }

        void load_parameter(
            const double& mass, const Eigen::Vector3d& com, const Eigen::Matrix3d& inertia) {
            *mass_ = mass;
            *com_ = com;
            *inertia_ = inertia;
        }

        void load_length(const double& length) { *length_ = length; }

    private:
        OutputInterface<double> length_;
        OutputInterface<double> mass_;
        OutputInterface<Eigen::Vector3d> com_;
        OutputInterface<Eigen::Matrix3d> inertia_;
    };

    class Joint {
    public:
        explicit Joint(rmcs_executor::Component& status_component, const std::string& name_prefix) {
            status_component.register_output(name_prefix + "/lower_limit", lower_limit_, NAN);
            status_component.register_output(name_prefix + "/upper_limit", upper_limit_, NAN);
            status_component.register_output(name_prefix + "/velocity_limit", velocity_limit_, NAN);
            status_component.register_output(name_prefix + "/torque_limit", torque_limit_, NAN);
            status_component.register_output(name_prefix + "/theta", theta_, NAN);
            status_component.register_output(name_prefix + "/velocity", velocity_, NAN);
            status_component.register_output(name_prefix + "/torque", torque_, NAN);
            status_component.register_output(
                name_prefix + "/position", position_,
                Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN()));
            status_component.register_output(name_prefix + "/friction", friction_, NAN);
        }

        double get_lower_limit() const { return *lower_limit_; }
        double get_upper_limit() const { return *upper_limit_; }
        double get_velocity_limit() const { return *velocity_limit_; }
        double get_torque_limit() const { return *torque_limit_; }
        double get_angle() const { return *theta_; }

        void load_parameter(
            const double& upper, const double& lower, const double& velocity, const double& torque,
            const Eigen::Vector3d& position, const double& friction) {
            *lower_limit_ = lower;
            *upper_limit_ = upper;
            *velocity_limit_ = velocity;
            *torque_limit_ = torque;
            *position_ = position;
            *friction_ = friction;
        }

        void update(const double& angle, const double& velocity, const double& torque) {
            *theta_ = angle;
            *velocity_ = velocity;
            *torque_ = torque;
        }

    private:
        OutputInterface<double> lower_limit_;
        OutputInterface<double> upper_limit_;
        OutputInterface<double> velocity_limit_;
        OutputInterface<double> torque_limit_;

        OutputInterface<double> theta_;
        OutputInterface<double> velocity_;
        OutputInterface<double> torque_;
        OutputInterface<double> friction_;

        OutputInterface<Eigen::Vector3d> position_;
    };

    void load_urdf(const std_msgs::msg::String::ConstSharedPtr& msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (*is_load_)
            return;

        urdf::Model model;
        if (!model.initString(msg->data)) {
            RCLCPP_ERROR(get_logger(), "URDF parse failed");
            return;
        }

        for (std::size_t i = 0; i < kJointCount; ++i) {
            const auto link_urdf = find_link(model, i);
            if (!link_urdf) {
                RCLCPP_ERROR(get_logger(), "URDF missing link for joint %zu", i + 1);
                return;
            }
            if (!link_urdf->inertial) {
                RCLCPP_ERROR(
                    get_logger(), "URDF link has no inertial: %s", link_urdf->name.c_str());
                return;
            }

            const auto& inertial = link_urdf->inertial;
            const auto& center = inertial->origin.position;
            const Eigen::Vector3d com(center.x, center.y, center.z);

            Eigen::Matrix3d inertia;
            inertia << inertial->ixx, inertial->ixy, inertial->ixz, inertial->ixy, inertial->iyy,
                inertial->iyz, inertial->ixz, inertial->iyz, inertial->izz;
            link[i].load_parameter(inertial->mass, com, inertia);

            const auto joint_urdf = find_joint(model, i);
            if (!joint_urdf) {
                RCLCPP_ERROR(get_logger(), "URDF missing joint %zu", i + 1);
                return;
            }
            if (!joint_urdf->limits) {
                RCLCPP_ERROR(
                    get_logger(), "URDF joint has no limits: %s", joint_urdf->name.c_str());
                return;
            }

            const auto& origin = joint_urdf->parent_to_joint_origin_transform.position;
            const Eigen::Vector3d position(origin.x, origin.y, origin.z);
            const double friction = joint_urdf->dynamics ? joint_urdf->dynamics->friction : 0.0;
            joint[i].load_parameter(
                joint_urdf->limits->upper, joint_urdf->limits->lower, joint_urdf->limits->velocity,
                joint_urdf->limits->effort, position, friction);
            joint_names_[i] = joint_urdf->name;
        }

        modify_link_length(model);
        *is_load_ = true;
        arm_urdf_sub_.reset();
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr arm_urdf_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;

    std::array<Link, kJointCount> link;
    std::array<Joint, kJointCount> joint;
    std::array<std::string, kJointCount> joint_names_;

    OutputInterface<bool> is_load_;
    std::mutex data_mutex_;
    std::size_t joint_states_divider_ = 0;

    std::array<InputInterface<double>, kJointCount> joint_motor_angle_;
    std::array<InputInterface<double>, kJointCount> joint_motor_velocity_;
    std::array<InputInterface<double>, kJointCount> joint_motor_torque_;
};

} // namespace rmcs_core::controller::arm

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::arm::ArmConfig, rmcs_executor::Component)
