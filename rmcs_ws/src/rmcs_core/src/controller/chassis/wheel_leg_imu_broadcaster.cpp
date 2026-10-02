// WheelLegImuBroadcaster: 把 IMU 相关量广播成 ROS 话题，便于在 Foxglove 里对照观察。
//   /wheel_leg/imu/data                 (sensor_msgs/Imu)       <- 原始 q_WB + 角速度
//   /wheel_leg/rl/imu/angular_velocity  (geometry_msgs/Vector3) <- RL 角速度（Body）
//   /wheel_leg/rl/imu/projected_gravity (geometry_msgs/Vector3) <- RL 世界重力投影（Body）
// 只做广播，不参与控制；不需要任何参数。

#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace rmcs_core::controller::chassis {

class WheelLegImuBroadcaster
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    WheelLegImuBroadcaster()
        : Node{get_component_name()} {
        register_input("/wheel_leg/imu/quaternion", orientation_);
        register_input("/wheel_leg/imu/angular_velocity", angular_velocity_);
        register_input("/wheel_leg/rl/imu/angular_velocity", rl_angular_velocity_);
        register_input("/wheel_leg/rl/imu/projected_gravity", rl_projected_gravity_);

        imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(
            "/wheel_leg/imu/data", rclcpp::QoS{5}.reliable());
        rl_angular_velocity_publisher_ = create_publisher<geometry_msgs::msg::Vector3>(
            "/wheel_leg/rl/imu/angular_velocity", rclcpp::QoS{5}.reliable());
        rl_projected_gravity_publisher_ = create_publisher<geometry_msgs::msg::Vector3>(
            "/wheel_leg/rl/imu/projected_gravity", rclcpp::QoS{5}.reliable());
    }

    void update() override {
        if (!orientation_.ready() || !angular_velocity_.ready())
            return;

        const auto& q = *orientation_;
        const auto& w = *angular_velocity_;

        sensor_msgs::msg::Imu imu;
        imu.header.stamp = get_clock()->now();
        imu.header.frame_id = "odom_imu";
        imu.orientation.x = q.x();
        imu.orientation.y = q.y();
        imu.orientation.z = q.z();
        imu.orientation.w = q.w();
        imu.angular_velocity.x = w.x();
        imu.angular_velocity.y = w.y();
        imu.angular_velocity.z = w.z();
        imu.linear_acceleration_covariance[0] = -1.0; // 该组件不提供线加速度
        imu_publisher_->publish(imu);

        if (rl_angular_velocity_.ready()) {
            geometry_msgs::msg::Vector3 msg;
            msg.x = rl_angular_velocity_->x();
            msg.y = rl_angular_velocity_->y();
            msg.z = rl_angular_velocity_->z();
            rl_angular_velocity_publisher_->publish(msg);
        }
        if (rl_projected_gravity_.ready()) {
            geometry_msgs::msg::Vector3 msg;
            msg.x = rl_projected_gravity_->x();
            msg.y = rl_projected_gravity_->y();
            msg.z = rl_projected_gravity_->z();
            rl_projected_gravity_publisher_->publish(msg);
        }
    }

private:
    InputInterface<Eigen::Quaterniond> orientation_;
    InputInterface<Eigen::Vector3d> angular_velocity_;
    InputInterface<Eigen::Vector3d> rl_angular_velocity_;
    InputInterface<Eigen::Vector3d> rl_projected_gravity_;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr rl_angular_velocity_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr rl_projected_gravity_publisher_;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::WheelLegImuBroadcaster, rmcs_executor::Component)
