#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <tuple>

#include <eigen3/Eigen/Geometry>
#include <fast_tf/rcl.hpp>
#include <librmcs/board/c_board.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/serial_interface.hpp>
#include <rmcs_utility/ring_buffer.hpp>
#include <std_msgs/msg/int32.hpp>

#include "filter/low_pass_filter.hpp"
#include "hardware/device/bmi088.hpp"
#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dm_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/j_supercap.hpp"

namespace rmcs_core::hardware {

class Hero
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    Hero()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , command_component_(
              create_partner_component<HeroCommand>(get_component_name() + "_command", *this)) {
        using namespace rmcs_description;

        register_output("/tf", tf_);
        tf_->set_transform<PitchLink, CameraLink>(Eigen::Translation3d{0.17, 0.0, 0.05});
        tf_->set_transform<PitchLink, CameraLink>(
            Eigen::AngleAxisd{0.10472, Eigen::Vector3d::UnitY()});
        tf_->set_transform<PitchLink, MuzzleLink>(Eigen::Translation3d{0.0, 0.0, 0.0});

        gimbal_calibrate_subscription_ = create_subscription<std_msgs::msg::Int32>(
            "/gimbal/calibrate", rclcpp::QoS{0}, [this](std_msgs::msg::Int32::UniquePtr&& msg) {
                gimbal_calibrate_subscription_callback(std::move(msg));
            });

        top_board_ = std::make_unique<TopBoard>(
            *this, *command_component_, get_parameter("board_serial_top_board").as_string());
        bottom_board_ = std::make_unique<BottomBoard>(
            *this, *command_component_, get_parameter("board_serial_bottom_board").as_string());
    }

    ~Hero() override = default;

    void update() override {
        top_board_->update();
        bottom_board_->update();
    }

    void command_update() {
        top_board_->command_update();
        bottom_board_->command_update();
    }

private:
    void gimbal_calibrate_subscription_callback(std_msgs::msg::Int32::UniquePtr) {
        RCLCPP_INFO(
            get_logger(), "[gimbal calibration] New yaw offset: %d",
            bottom_board_->gimbal_yaw_motor_.calibrate_zero_point());
        RCLCPP_INFO(
            get_logger(), "[gimbal calibration] New pitch offset: %d",
            top_board_->gimbal_pitch_motor_.calibrate_zero_point());
    }

    class HeroCommand : public rmcs_executor::Component {
    public:
        explicit HeroCommand(Hero& hero)
            : hero_(hero) {}

        void update() override { hero_.command_update(); }

        Hero& hero_;
    };
    std::shared_ptr<HeroCommand> command_component_;

    class TopBoard final : public librmcs::board::CBoard::Callback {
    public:
        friend class Hero;
        explicit TopBoard(Hero& hero, HeroCommand& hero_command, std::string_view board_serial = {})
            : tf_(hero.tf_)
            , imu_(10.0f, 0.001f, 1000000.0f)
            , dr16_{}
            , imu_bias_x(static_cast<int16_t>(hero.get_parameter("imu_bias_x").as_int()))
            , imu_bias_y(static_cast<int16_t>(hero.get_parameter("imu_bias_y").as_int()))
            , imu_bias_z(static_cast<int16_t>(hero.get_parameter("imu_bias_z").as_int()))
            , gimbal_pitch_motor_(
                  hero, hero_command, "/gimbal/pitch",
                  device::DmMotor::Config{device::DmMotor::Type::kJ4310}
                      .set_encoder_zero_point(
                          static_cast<int>(hero.get_parameter("pitch_motor_zero_point").as_int()))
                      .set_reversed())
            , gimbal_friction_wheels_(
                  {hero, hero_command, "/gimbal/first_friction",
                   device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 1}
                       .set_reduction_ratio(1.)
                       .set_reversed()},
                  {hero, hero_command, "/gimbal/second_friction",
                   device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 2}.set_reduction_ratio(
                       1.)},
                  {hero, hero_command, "/gimbal/third_friction",
                   device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 3}
                       .set_reduction_ratio(1.)
                       .set_reversed()}) {

            imu_.set_coordinate_mapping([](double x, double y, double z) {
                // The rotation angle must be an exact multiple of 90 degrees, otherwise use a
                // matrix. See the upstream hero implementation for the derivation.
                return std::make_tuple(-y, x, z);
            });

            hero.register_output("/gimbal/yaw/velocity_imu", gimbal_yaw_velocity_imu_);
            hero.register_output("/gimbal/pitch/velocity_imu", gimbal_pitch_velocity_imu_);

            hero.register_output("/debug/pitch/raw_angle", debug_pitch_raw_angle_);
            hero.register_output("/debug/pitch/temp", debug_pitch_temp);

            board_ = std::make_unique<librmcs::board::CBoard>(*this, board_serial);
        }

        ~TopBoard() final = default;

        void update() {
            imu_.update_status();
            Eigen::Quaterniond gimbal_imu_pose{imu_.q0(), imu_.q1(), imu_.q2(), imu_.q3()};

            tf_->set_transform<rmcs_description::PitchLink, rmcs_description::OdomImu>(
                gimbal_imu_pose.conjugate());
            tf_->set_transform<rmcs_description::BaseLink, rmcs_description::RawImu>(
                gimbal_imu_pose);
            fast_tf::rcl::broadcast_all(*tf_);

            dr16_.update_status();

            *gimbal_yaw_velocity_imu_ = imu_gz_velocity_filter_.update(imu_.gz());
            *gimbal_pitch_velocity_imu_ = imu_gy_velocity_filter_.update(imu_.gy());

            *debug_pitch_raw_angle_ = gimbal_pitch_motor_.last_raw_angle();
            gimbal_pitch_motor_.update_status();
            tf_->set_state<rmcs_description::YawLink, rmcs_description::PitchLink>(
                gimbal_pitch_motor_.angle());

            fast_tf::rcl::broadcast_all(*tf_);

            for (auto& motor : gimbal_friction_wheels_)
                motor.update_status();
        }

        void command_update() {
            auto friction_packet = device::CanPacket8{
                gimbal_friction_wheels_[0].generate_command(),
                gimbal_friction_wheels_[1].generate_command(),
                gimbal_friction_wheels_[2].generate_command(),
                device::CanPacket8::PaddingQuarter{}};
            auto pitch_packet = gimbal_pitch_motor_.generate_torque_command();

            board_->start_transmit()
                .can_transmit(
                    Spec::kCans.kCan1, {.can_id = 0x200, .can_data = friction_packet.as_bytes()})
                .can_transmit(
                    Spec::kCans.kCan2, {.can_id = 0x9, .can_data = pitch_packet.as_bytes()});
        }

    private:
        void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
                return;

            const auto& can_id = data.can_id;
            if (can == Spec::kCans.kCan1) {
                for (auto& motor : gimbal_friction_wheels_)
                    if (motor.match_then_store_status(can_id, data.can_data))
                        break;
            } else if (can == Spec::kCans.kCan2) {
                if (can_id == 0x219) {
                    gimbal_pitch_motor_.store_status(data.can_data);
                }
            }
        }

        void uart_receive_callback(const Spec::Uart& uart, const View::Uart& data) override {
            if (uart == Spec::kUarts.kDbus) {
                dr16_.store_status(data.uart_data.data(), data.uart_data.size());
            }
        }

        void accelerometer_receive_callback(const View::ImuAccelerometer& data) override {
            imu_.store_accelerometer_status(data.x, data.y, data.z);
        }

        void gyroscope_receive_callback(const View::ImuGyroscope& data) override {
            imu_.store_gyroscope_status(
                data.x - imu_bias_x, data.y - imu_bias_y, data.z - imu_bias_z);
        }

        OutputInterface<rmcs_description::Tf>& tf_;

        device::Bmi088 imu_;
        device::Dr16 dr16_;

        OutputInterface<double> gimbal_yaw_velocity_imu_;
        OutputInterface<double> gimbal_pitch_velocity_imu_;
        OutputInterface<double> debug_pitch_raw_angle_;
        OutputInterface<double> debug_pitch_temp;

        int16_t imu_bias_x = 0, imu_bias_y = 0, imu_bias_z = 0;

        filter::LowPassFilter<> imu_gy_velocity_filter_{4.0f, 1000.0f};
        filter::LowPassFilter<> imu_gz_velocity_filter_{8.0f, 1000.0f};

        device::DmMotor gimbal_pitch_motor_;

        device::DjiMotor gimbal_friction_wheels_[3];

        std::unique_ptr<librmcs::board::CBoard> board_;
    };

    class BottomBoard final : public librmcs::board::CBoard::Callback {
    public:
        friend class Hero;
        explicit BottomBoard(
            Hero& hero, HeroCommand& hero_command, std::string_view board_serial = {})
            : imu_(10.0f, 0.001f, 1000000.0f)
            , tf_(hero.tf_)
            , chassis_wheel_motors_(
                  {hero, hero_command, "/chassis/left_front_wheel",
                   device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 1}},
                  {hero, hero_command, "/chassis/left_back_wheel",
                   device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 2}},
                  {hero, hero_command, "/chassis/right_back_wheel",
                   device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 3}},
                  {hero, hero_command, "/chassis/right_front_wheel",
                   device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 4}})
            , supercap_(hero, 28.5)
            , gimbal_yaw_motor_(
                  hero, hero_command, "/gimbal/yaw",
                  device::DmMotor::Config{device::DmMotor::Type::kJ4310}.set_encoder_zero_point(
                      static_cast<int>(hero.get_parameter("yaw_motor_zero_point").as_int())))
            , gimbal_bullet_feeder_(
                  hero, hero_command, "/gimbal/bullet_feeder",
                  device::DmMotor::Config{device::DmMotor::Type::kJ4310}
                      .enable_multi_turn_angle()) {

            imu_.set_coordinate_mapping([](double x, double y, double z) {
                // The rotation angle must be an exact multiple of 90 degrees, otherwise use a
                // matrix. See the upstream hero implementation for the derivation.
                return std::make_tuple(x, z, y);
            });

            hero.register_output("/referee/serial", referee_serial_);
            referee_serial_->read = [this](std::byte* buffer, size_t size) {
                return referee_ring_buffer_receive_.pop_front_n(
                    [&buffer](std::byte byte) noexcept { *buffer++ = byte; }, size);
            };
            referee_serial_->write = [this](const std::byte* buffer, size_t size) {
                board_->start_transmit().uart_transmit(
                    Spec::kUarts.kUart1, {.uart_data = std::span<const std::byte>{buffer, size}});
                return size;
            };

            hero.register_output("/chassis/yaw/velocity_imu", chassis_yaw_velocity_imu_, 0);
            hero.register_output("/debug/yaw/raw_angle", debug_yaw_raw_angle_);

            board_ = std::make_unique<librmcs::board::CBoard>(*this, board_serial);
        }

        ~BottomBoard() final = default;

        void update() {
            imu_.update_status();
            gimbal_yaw_motor_.update_status();
            *chassis_yaw_velocity_imu_ = imu_gz_velocity_filter_.update(imu_.gz());

            tf_->set_state<rmcs_description::GimbalCenterLink, rmcs_description::YawLink>(
                gimbal_yaw_motor_.angle());

            for (auto& motor : chassis_wheel_motors_)
                motor.update_status();
            gimbal_bullet_feeder_.update_status();

            supercap_.update_status();

            *debug_yaw_raw_angle_ = gimbal_yaw_motor_.last_raw_angle();
        }

        void command_update() {
            auto wheels_packet = device::CanPacket8{
                chassis_wheel_motors_[0].generate_command(),
                chassis_wheel_motors_[1].generate_command(),
                chassis_wheel_motors_[2].generate_command(),
                chassis_wheel_motors_[3].generate_command()};
            auto feeder_packet = gimbal_bullet_feeder_.generate_torque_command();
            auto yaw_packet = gimbal_yaw_motor_.generate_torque_command();

            board_->start_transmit()
                .can_transmit(
                    Spec::kCans.kCan1, {.can_id = 0x200, .can_data = wheels_packet.as_bytes()})
                .can_transmit(
                    Spec::kCans.kCan2, {.can_id = 0x4, .can_data = feeder_packet.as_bytes()})
                .can_transmit(
                    Spec::kCans.kCan2, {.can_id = 0x2, .can_data = yaw_packet.as_bytes()});
        }

    private:
        void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
                return;
            if (can != Spec::kCans.kCan2) [[unlikely]]
                return;

            const auto& can_id = data.can_id;
            if (can_id == 0x214) {
                gimbal_bullet_feeder_.store_status(data.can_data);
            } else if (can_id == 0x212) {
                gimbal_yaw_motor_.store_status(data.can_data);
            } else if (can_id == 0x20c) {
                supercap_.store_status(data.can_data);
            }
        }

        void uart_receive_callback(const Spec::Uart& uart, const View::Uart& data) override {
            if (uart == Spec::kUarts.kUart1) {
                const auto* uart_data = data.uart_data.data();
                referee_ring_buffer_receive_.emplace_back_n(
                    [&uart_data](std::byte* storage) noexcept { *storage = *uart_data++; },
                    data.uart_data.size());
            }
        }

        void accelerometer_receive_callback(const View::ImuAccelerometer& data) override {
            imu_.store_accelerometer_status(data.x, data.y, data.z);
        }

        void gyroscope_receive_callback(const View::ImuGyroscope& data) override {
            imu_.store_gyroscope_status(data.x, data.y, data.z);
        }

        device::Bmi088 imu_;
        OutputInterface<rmcs_description::Tf>& tf_;

        OutputInterface<double> chassis_yaw_velocity_imu_;
        OutputInterface<double> debug_yaw_raw_angle_;

        filter::LowPassFilter<> imu_gz_velocity_filter_{60.0f, 1000.0f};

        device::DjiMotor chassis_wheel_motors_[4];
        device::Jsupercap supercap_;

        device::DmMotor gimbal_yaw_motor_;
        device::DmMotor gimbal_bullet_feeder_;

        rmcs_utility::RingBuffer<std::byte> referee_ring_buffer_receive_{256};
        OutputInterface<rmcs_msgs::SerialInterface> referee_serial_;

        std::unique_ptr<librmcs::board::CBoard> board_;
    };

    OutputInterface<rmcs_description::Tf> tf_;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr gimbal_calibrate_subscription_;

    std::unique_ptr<TopBoard> top_board_;
    std::unique_ptr<BottomBoard> bottom_board_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::Hero, rmcs_executor::Component)
