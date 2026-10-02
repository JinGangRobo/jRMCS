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
#include "hardware/device/bmi088_ekf.hpp"
#include "hardware/device/board_clock_lifter.hpp"
#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dm_motor.hpp"
#include "hardware/device/lk_motor.hpp"

#include "hardware/device/dr16.hpp"
#include "hardware/device/j_supercap.hpp"
#include "hardware/device/remote_control.hpp"

namespace rmcs_core::hardware {

class WheelLegInfantry
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    WheelLegInfantry()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , command_component_(
              create_partner_component<WheelLegInfantryCommand>(
                  get_component_name() + "_command", *this)) {
        using namespace rmcs_description;

        register_output("/tf", tf_);
        // 整车底盘(浮动基座)IMU: 由下板 IMU 提供, 供 RL 观测 / 底盘控制器使用。
        register_output(
            "/wheel_leg/imu/quaternion", imu_quaternion_, Eigen::Quaterniond::Identity());
        register_output(
            "/wheel_leg/imu/angular_velocity", imu_angular_velocity_, Eigen::Vector3d::Zero());
        tf_->set_transform<PitchLink, CameraLink>(Eigen::Translation3d{-0.052, 0.0, 0.084});
        tf_->set_transform<PitchLink, MuzzleLink>(Eigen::Translation3d{0.0, 0.0, 0.0});

        gimbal_calibrate_subscription_ = create_subscription<std_msgs::msg::Int32>(
            "/gimbal/calibrate", rclcpp::QoS{0}, [this](std_msgs::msg::Int32::UniquePtr&& msg) {
                gimbal_calibrate_subscription_callback(std::move(msg));
            });
        remote_control_ = std::make_unique<device::RemoteControl>(*this);
        top_board_ = std::make_unique<TopBoard>(
            *this, *command_component_, get_parameter("board_serial_top_board").as_string());
        bottom_board_ = std::make_unique<BottomBoard>(
            *this, *command_component_, get_parameter("board_serial_bottom_board").as_string());
    }

    ~WheelLegInfantry() override = default;

    void update() override {

        top_board_->update();
        bottom_board_->update();
        remote_control_->update();
    }

    void command_update() {
        const bool even = ((cmd_tick_++ & 1u) == 0u);
        top_board_->command_update();
        bottom_board_->command_update(even);
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

    class WheelLegInfantryCommand : public rmcs_executor::Component {
    public:
        explicit WheelLegInfantryCommand(WheelLegInfantry& wheeleg_infantry)
            : wheeleg_infantry_(wheeleg_infantry) {}

        void update() override { wheeleg_infantry_.command_update(); }

        WheelLegInfantry& wheeleg_infantry_;
    };
    std::shared_ptr<WheelLegInfantryCommand> command_component_;

    class TopBoard final : public librmcs::board::CBoard::Callback {
    public:
        friend class WheelLegInfantry;
        explicit TopBoard(
            WheelLegInfantry& wheeleg_infantry, WheelLegInfantryCommand& wheeleg_infantry_command,
            std::string_view board_serial = {})
            : wheeleg_infantry_(wheeleg_infantry)
            , tf_(wheeleg_infantry.tf_)
            , dr16_{}
            , imu_bias_x(
                  static_cast<int16_t>(wheeleg_infantry.get_parameter("imu_bias_x").as_int()))
            , imu_bias_y(
                  static_cast<int16_t>(wheeleg_infantry.get_parameter("imu_bias_y").as_int()))
            , imu_bias_z(
                  static_cast<int16_t>(wheeleg_infantry.get_parameter("imu_bias_z").as_int()))
            , gimbal_pitch_motor_(wheeleg_infantry, wheeleg_infantry_command, "/gimbal/pitch")
            , gimbal_left_friction_(
                  wheeleg_infantry, wheeleg_infantry_command, "/gimbal/left_friction")
            , gimbal_right_friction_(
                  wheeleg_infantry, wheeleg_infantry_command, "/gimbal/right_friction") {

            gimbal_pitch_motor_.configure(
                device::DmMotor::Config{device::DmMotor::Type::kJ4310}.set_encoder_zero_point(
                    static_cast<int>(
                        wheeleg_infantry.get_parameter("pitch_motor_zero_point").as_int())));

            gimbal_left_friction_.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 1}.set_reduction_ratio(
                    1.));
            gimbal_right_friction_.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 2}
                    .set_reduction_ratio(1.)
                    .set_reversed());

            wheeleg_infantry.register_output("/gimbal/yaw/velocity_imu", gimbal_yaw_velocity_imu_);
            wheeleg_infantry.register_output(
                "/gimbal/pitch/velocity_imu", gimbal_pitch_velocity_imu_);
            wheeleg_infantry.register_output("/debug/pitch/raw_angle", debug_pitch_raw_angle_);

            board_ = std::make_unique<librmcs::board::CBoard>(*this, board_serial);
            wheeleg_infantry.remote_control_->register_dr16(&dr16_);
        }

        ~TopBoard() final = default;

        void update() {
            const auto snapshot = imu_.snapshot();

            if (snapshot) {
                tf_->set_transform<rmcs_description::PitchLink, rmcs_description::OdomImu>(
                    snapshot->orientation.conjugate());
                // tf_->set_transform<rmcs_description::BaseLink, rmcs_description::RawImu>(
                //     snapshot->orientation);
            }
            fast_tf::rcl::broadcast_all(*tf_);

            dr16_.update_status();

            if (snapshot) {
                // 上板 IMU 装在云台上: 只供云台 yaw/pitch 稳定, 不作为整车底盘姿态。
                *gimbal_yaw_velocity_imu_ = imu_gz_velocity_filter_.update(snapshot->gyro_body.z());
                *gimbal_pitch_velocity_imu_ =
                    imu_gy_velocity_filter_.update(snapshot->gyro_body.y());
            }

            *debug_pitch_raw_angle_ = gimbal_pitch_motor_.last_raw_angle();

            gimbal_pitch_motor_.update_status();
            tf_->set_state<rmcs_description::YawLink, rmcs_description::PitchLink>(
                gimbal_pitch_motor_.angle());
            fast_tf::rcl::broadcast_all(*tf_);

            gimbal_left_friction_.update_status();
            gimbal_right_friction_.update_status();
        }

        void command_update() {
            auto friction_packet = device::CanPacket8{
                gimbal_left_friction_.generate_command(), gimbal_right_friction_.generate_command(),
                device::CanPacket8::PaddingQuarter{}, device::CanPacket8::PaddingQuarter{}};
            // auto pitch_packet = device::CanPacket8{
            //     gimbal_pitch_motor_.generate_command(), device::CanPacket8::PaddingQuarter{},
            //     device::CanPacket8::PaddingQuarter{}, device::CanPacket8::PaddingQuarter{}};

            board_->start_transmit()
                .can_transmit(
                    Spec::kCans.kCan1, {.can_id = 0x200, .can_data = friction_packet.as_bytes()})
                .can_transmit(
                    Spec::kCans.kCan2,
                    {.can_id = 0x06,
                     .can_data = gimbal_pitch_motor_.generate_torque_command().as_bytes()});
        }

    private:
        void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
                return;
            // if (can != Spec::kCans.kCan1) [[unlikely]]
            //     return;

            const auto& can_id = data.can_id;
            if (can == Spec::kCans.kCan1) {
                if (can_id == 0x201) {
                    gimbal_left_friction_.store_status(data.can_data);
                } else if (can_id == 0x202) {
                    gimbal_right_friction_.store_status(data.can_data);
                }
            } else if (can == Spec::kCans.kCan2) {
                if (can_id == 0x216) {
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
            const auto timestamp = board_clock_lifter_.advance_timebase(data.timestamp_quarter_us);
            imu_.push_accelerometer_sample(data.x, data.y, data.z, timestamp);
        }

        void gyroscope_receive_callback(const View::ImuGyroscope& data) override {
            const auto timestamp = board_clock_lifter_.lift_timestamp(data.timestamp_quarter_us);
            if (!timestamp.has_value())
                return;
            imu_.try_update_with_gyroscope_sample(
                data.x - imu_bias_x, data.y - imu_bias_y, data.z - imu_bias_z, *timestamp);
        }

        WheelLegInfantry& wheeleg_infantry_;
        OutputInterface<rmcs_description::Tf>& tf_;

        device::Bmi088Ekf imu_{device::Bmi088Ekf::Config{}};
        device::BoardClockLifter board_clock_lifter_;
        device::Dr16 dr16_;

        int16_t imu_bias_x = 0, imu_bias_y = 0, imu_bias_z = 0;

        OutputInterface<double> gimbal_yaw_velocity_imu_;
        OutputInterface<double> gimbal_pitch_velocity_imu_;
        OutputInterface<double> debug_pitch_raw_angle_;

        device::DmMotor gimbal_pitch_motor_;
        device::DjiMotor gimbal_left_friction_;
        device::DjiMotor gimbal_right_friction_;

        filter::LowPassFilter<> imu_gy_velocity_filter_{4.0f, 1000.0f};
        filter::LowPassFilter<> imu_gz_velocity_filter_{8.0f, 1000.0f};

        std::unique_ptr<librmcs::board::CBoard> board_;
    };

    class BottomBoard final : public librmcs::board::CBoard::Callback {
    public:
        friend class WheelLegInfantry;
        explicit BottomBoard(
            WheelLegInfantry& wheeleg_infantry, WheelLegInfantryCommand& wheeleg_infantry_command,
            std::string_view board_serial = {})
            : wheeleg_infantry_(wheeleg_infantry)
            , tf_(wheeleg_infantry.tf_)
            , gimbal_yaw_motor_(wheeleg_infantry, wheeleg_infantry_command, "/gimbal/yaw")
            , gimbal_bullet_feeder_(
                  wheeleg_infantry, wheeleg_infantry_command, "/gimbal/bullet_feeder")
            , chassis_wheel_motors_(
                  {wheeleg_infantry, wheeleg_infantry_command, "/chassis/left_wheel",
                   device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 1}
                       .set_reduction_ratio(268.0 / 17.0)
                       .set_reversed()
                       .enable_multi_turn_angle()},
                  {wheeleg_infantry, wheeleg_infantry_command, "/chassis/right_wheel",
                   device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 2}
                       .set_reduction_ratio(268.0 / 17.0)
                       .set_reversed()
                       .enable_multi_turn_angle()})
            , left_front_hip_motors_(
                  wheeleg_infantry, wheeleg_infantry_command, "/chassis/left_front_hip",
                  device::DmMotor::Config{device::DmMotor::Type::kJ4310}
                      .enable_multi_turn_angle()
                      .set_encoder_zero_point(
                          static_cast<int>(
                              wheeleg_infantry.get_parameter("left_front_hip_motors_zero_point")
                                  .as_int())))
            , left_back_hip_motors_(
                  wheeleg_infantry, wheeleg_infantry_command, "/chassis/left_back_hip",
                  device::DmMotor::Config{device::DmMotor::Type::kJ4310}
                      .enable_multi_turn_angle()
                      .set_encoder_zero_point(
                          static_cast<int>(
                              wheeleg_infantry.get_parameter("left_back_hip_motors_zero_point")
                                  .as_int())))
            , right_front_hip_motors_(
                  wheeleg_infantry, wheeleg_infantry_command, "/chassis/right_front_hip",
                  device::DmMotor::Config{device::DmMotor::Type::kJ4310}
                      .enable_multi_turn_angle()
                      .set_encoder_zero_point(
                          static_cast<int>(
                              wheeleg_infantry.get_parameter("right_front_hip_motors_zero_point")
                                  .as_int())))
            , right_back_hip_motors_(
                  wheeleg_infantry, wheeleg_infantry_command, "/chassis/right_back_hip",
                  device::DmMotor::Config{device::DmMotor::Type::kJ4310}
                      .enable_multi_turn_angle()
                      .set_encoder_zero_point(
                          static_cast<int>(
                              wheeleg_infantry.get_parameter("right_back_hip_motors_zero_point")
                                  .as_int())))
            , supercap_(wheeleg_infantry, 28.5) {

            gimbal_yaw_motor_.configure(
                device::DmMotor::Config{device::DmMotor::Type::kJ4310}.set_encoder_zero_point(
                    static_cast<int>(
                        wheeleg_infantry.get_parameter("yaw_motor_zero_point").as_int())));

            gimbal_bullet_feeder_.configure(
                device::LkMotor::Config{device::LkMotor::Type::kMG4005Ei10}
                    .enable_multi_turn_angle()
                    .set_reversed());

            wheeleg_infantry.register_output("/referee/serial", referee_serial_);
            referee_serial_->read = [this](std::byte* buffer, size_t size) {
                return referee_ring_buffer_receive_.pop_front_n(
                    [&buffer](std::byte byte) noexcept { *buffer++ = byte; }, size);
            };
            referee_serial_->write = [this](const std::byte* buffer, size_t size) {
                board_->start_transmit().uart_transmit(
                    Spec::kUarts.kUart1, {.uart_data = std::span<const std::byte>{buffer, size}});
                return size;
            };

            wheeleg_infantry.register_output(
                "/chassis/yaw/velocity_imu", chassis_yaw_velocity_imu_, 0);
            wheeleg_infantry.register_output("/debug/yaw/raw_angle", debug_yaw_raw_angle_);

            board_ = std::make_unique<librmcs::board::CBoard>(*this, board_serial);
        }

        ~BottomBoard() final = default;

        void update() {
            if (const auto snapshot = imu_.snapshot()) {
                *chassis_yaw_velocity_imu_ =
                    imu_gz_velocity_filter_.update(snapshot->gyro_body.z());
                // 下板 IMU 装在底盘上: 作为整车底盘(浮动基座)姿态, 供 RL 观测使用。
                *wheeleg_infantry_.imu_quaternion_ = snapshot->orientation;
                *wheeleg_infantry_.imu_angular_velocity_ = snapshot->gyro_body;
            }
            *debug_yaw_raw_angle_ = gimbal_yaw_motor_.last_raw_angle();

            gimbal_yaw_motor_.update_status();
            left_front_hip_motors_.update_status();
            left_back_hip_motors_.update_status();
            right_front_hip_motors_.update_status();
            right_back_hip_motors_.update_status();

            tf_->set_state<rmcs_description::GimbalCenterLink, rmcs_description::YawLink>(
                gimbal_yaw_motor_.angle());
            fast_tf::rcl::broadcast_all(*tf_);

            gimbal_bullet_feeder_.update_status();

            for (auto& motor : chassis_wheel_motors_)
                motor.update_status();

            supercap_.update_status();
        }

        void command_update(bool even) {
            auto wheels_packet = device::CanPacket8{
                chassis_wheel_motors_[0].generate_command(),
                chassis_wheel_motors_[1].generate_command(), device::CanPacket8::PaddingQuarter{},
                device::CanPacket8::PaddingQuarter{}};
            // auto yaw_feeder_packet = device::CanPacket8{
            //     device::CanPacket8::PaddingQuarter{}, gimbal_yaw_motor_.generate_command(),
            //     gimbal_bullet_feeder_.generate_command(), device::CanPacket8::PaddingQuarter{}};

            auto builder = board_->start_transmit();
            if (even) {
                builder.can_transmit(
                    Spec::kCans.kCan1,
                    {.can_id = 0x01,
                     .can_data = left_front_hip_motors_.generate_torque_command().as_bytes()});
                builder.can_transmit(
                    Spec::kCans.kCan1,
                    {.can_id = 0x02,
                     .can_data = left_back_hip_motors_.generate_torque_command().as_bytes()});
            } else {
                builder.can_transmit(
                    Spec::kCans.kCan1,
                    {.can_id = 0x03,
                     .can_data = right_front_hip_motors_.generate_torque_command().as_bytes()});
                builder.can_transmit(
                    Spec::kCans.kCan1,
                    {.can_id = 0x04,
                     .can_data = right_back_hip_motors_.generate_torque_command().as_bytes()});
                builder.can_transmit(
                    Spec::kCans.kCan1,
                    {.can_id = 0x05,
                     .can_data = gimbal_yaw_motor_.generate_torque_command().as_bytes()});
            }
            builder.can_transmit(
                Spec::kCans.kCan2, {.can_id = 0x200, .can_data = wheels_packet.as_bytes()});
            builder.can_transmit(
                Spec::kCans.kCan2,
                {.can_id = 0x141, .can_data = gimbal_bullet_feeder_.generate_command().as_bytes()});
        }

    private:
        void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
                return;

            const auto& can_id = data.can_id;

            // if (can != Spec::kCans.kCan1) [[unlikely]]
            //     return;
            if (can == Spec::kCans.kCan1) {
                if (can_id == 0x211) {
                    left_front_hip_motors_.store_status(data.can_data);
                } else if (can_id == 0x212) {
                    left_back_hip_motors_.store_status(data.can_data);
                } else if (can_id == 0x213) {
                    right_front_hip_motors_.store_status(data.can_data);
                } else if (can_id == 0x214) {
                    right_back_hip_motors_.store_status(data.can_data);
                } else if (can_id == 0x215) {
                    gimbal_yaw_motor_.store_status(data.can_data);
                }
            }
            if (can == Spec::kCans.kCan2) {
                if (can_id == 0x201) {
                    chassis_wheel_motors_[0].store_status(data.can_data);
                } else if (can_id == 0x202) {
                    chassis_wheel_motors_[1].store_status(data.can_data);
                } else if (can_id == 0x141) {
                    gimbal_bullet_feeder_.store_status(data.can_data);
                } else if (can_id == 0x20c) {
                    supercap_.store_status(data.can_data);
                }
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
            const auto timestamp = board_clock_lifter_.advance_timebase(data.timestamp_quarter_us);
            imu_.push_accelerometer_sample(data.x, data.y, data.z, timestamp);
        }

        void gyroscope_receive_callback(const View::ImuGyroscope& data) override {
            const auto timestamp = board_clock_lifter_.lift_timestamp(data.timestamp_quarter_us);
            if (!timestamp.has_value())
                return;
            imu_.try_update_with_gyroscope_sample(data.x, data.y, data.z, *timestamp);
        }

        device::Bmi088Ekf imu_{device::Bmi088Ekf::Config{}};
        device::BoardClockLifter board_clock_lifter_;
        WheelLegInfantry& wheeleg_infantry_;
        OutputInterface<rmcs_description::Tf>& tf_;

        filter::LowPassFilter<> imu_gz_velocity_filter_{60.0f, 1000.0f};

        OutputInterface<double> chassis_yaw_velocity_imu_;
        OutputInterface<double> debug_yaw_raw_angle_;

        device::DmMotor gimbal_yaw_motor_;
        device::LkMotor gimbal_bullet_feeder_;

        device::DjiMotor chassis_wheel_motors_[2];
        device::DmMotor left_front_hip_motors_;
        device::DmMotor left_back_hip_motors_;
        device::DmMotor right_front_hip_motors_;
        device::DmMotor right_back_hip_motors_;
        device::Jsupercap supercap_;

        rmcs_utility::RingBuffer<std::byte> referee_ring_buffer_receive_{256};
        OutputInterface<rmcs_msgs::SerialInterface> referee_serial_;

        std::unique_ptr<librmcs::board::CBoard> board_;
    };

    OutputInterface<rmcs_description::Tf> tf_;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr gimbal_calibrate_subscription_;

    OutputInterface<Eigen::Quaterniond> imu_quaternion_;
    OutputInterface<Eigen::Vector3d> imu_angular_velocity_;

    std::unique_ptr<TopBoard> top_board_;
    std::unique_ptr<BottomBoard> bottom_board_;
    uint32_t cmd_tick_ = 0;

    std::unique_ptr<device::RemoteControl> remote_control_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::WheelLegInfantry, rmcs_executor::Component)
