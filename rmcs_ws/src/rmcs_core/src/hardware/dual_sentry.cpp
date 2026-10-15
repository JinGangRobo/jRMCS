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
#include "hardware/device/dr16.hpp"
#include "hardware/device/gy614.hpp"
#include "hardware/device/j_supercap.hpp"

namespace rmcs_core::hardware {

class DualSentry
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DualSentry()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , command_component_(
              create_partner_component<DualSentryCommand>(
                  get_component_name() + "_command", *this)) {
        using namespace rmcs_description;

        register_output("/tf", tf_);
        tf_->set_transform<PitchLink, CameraLink>(Eigen::Translation3d{0.084, 0.0, 0.048});
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

    ~DualSentry() override = default;

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
            bottom_board_->gimbal_bottom_yaw_motor_.calibrate_zero_point());
        RCLCPP_INFO(
            get_logger(), "[gimbal calibration] New yaw offset: %d",
            top_board_->gimbal_top_yaw_motor_.calibrate_zero_point());
        RCLCPP_INFO(
            get_logger(), "[gimbal calibration] New pitch offset: %d",
            top_board_->gimbal_pitch_motor_.calibrate_zero_point());
    }

    class DualSentryCommand : public rmcs_executor::Component {
    public:
        explicit DualSentryCommand(DualSentry& dual_sentry)
            : dual_sentry_(dual_sentry) {}

        void update() override { dual_sentry_.command_update(); }

        DualSentry& dual_sentry_;
    };
    std::shared_ptr<DualSentryCommand> command_component_;

    class TopBoard final : public librmcs::board::CBoard::Callback {
    public:
        friend class DualSentry;
        explicit TopBoard(
            DualSentry& dual_sentry, DualSentryCommand& dual_sentry_command,
            std::string_view board_serial = {})
            : tf_(dual_sentry.tf_)
            , gy614_(dual_sentry, "/friction_wheels/temperature")
            , dr16_{}
            , imu_bias_x(static_cast<int16_t>(dual_sentry.get_parameter("imu_bias_x").as_int()))
            , imu_bias_y(static_cast<int16_t>(dual_sentry.get_parameter("imu_bias_y").as_int()))
            , imu_bias_z(static_cast<int16_t>(dual_sentry.get_parameter("imu_bias_z").as_int()))
            , gimbal_top_yaw_motor_(dual_sentry, dual_sentry_command, "/gimbal/top_yaw")
            , gimbal_pitch_motor_(dual_sentry, dual_sentry_command, "/gimbal/pitch")
            , gimbal_left_friction_(dual_sentry, dual_sentry_command, "/gimbal/left_friction")
            , gimbal_right_friction_(dual_sentry, dual_sentry_command, "/gimbal/right_friction")
            , gimbal_bullet_feeder_(dual_sentry, dual_sentry_command, "/gimbal/bullet_feeder") {

            gimbal_top_yaw_motor_.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kGM6020Voltage, 2}
                    .set_encoder_zero_point(
                        static_cast<int>(
                            dual_sentry.get_parameter("top_yaw_motor_zero_point").as_int())));

            gimbal_pitch_motor_.configure(
                device::DmMotor::Config{device::DmMotor::Type::kJ4310}
                    .set_encoder_zero_point(
                        static_cast<int>(
                            dual_sentry.get_parameter("pitch_motor_zero_point").as_int()))
                    .set_reversed());

            gimbal_left_friction_.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 2}
                    .set_reduction_ratio(1.)
                    .set_reversed());
            gimbal_right_friction_.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 1}.set_reduction_ratio(
                    1.));

            gimbal_bullet_feeder_.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 8}
                    .enable_multi_turn_angle()
                    .set_reversed()
                    .set_reduction_ratio(19 * 2));

            dual_sentry.register_output("/gimbal/yaw/velocity_imu", gimbal_yaw_velocity_imu_);
            dual_sentry.register_output("/gimbal/pitch/velocity_imu", gimbal_pitch_velocity_imu_);

            dual_sentry.register_output("/debug/pitch/raw_angle", debug_pitch_raw_angle_);
            dual_sentry.register_output("/debug/top_yaw/raw_angle", debug_top_yaw_raw_angle_);

            board_ = std::make_unique<librmcs::board::CBoard>(*this, board_serial);
        }

        ~TopBoard() final = default;

        void update() {
            const auto snapshot = imu_.snapshot();

            gimbal_top_yaw_motor_.update_status();
            gimbal_pitch_motor_.update_status();

            if (snapshot) {
                tf_->set_transform<rmcs_description::PitchLink, rmcs_description::OdomImu>(
                    snapshot->orientation.conjugate());
            }
            tf_->set_state<rmcs_description::YawLink, rmcs_description::PitchLink>(
                gimbal_pitch_motor_.angle());

            fast_tf::rcl::broadcast_all(*tf_);
            if (snapshot) {
                tf_->set_transform<rmcs_description::BaseLink, rmcs_description::RawImu>(
                    snapshot->orientation);
            }

            gy614_.update_status();
            dr16_.update_status();

            if (snapshot) {
                *gimbal_yaw_velocity_imu_ = imu_gz_velocity_filter_.update(snapshot->gyro_body.z());
                *gimbal_pitch_velocity_imu_ =
                    imu_gy_velocity_filter_.update(snapshot->gyro_body.y());
            }

            *debug_pitch_raw_angle_ = gimbal_pitch_motor_.last_raw_angle();
            *debug_top_yaw_raw_angle_ = gimbal_top_yaw_motor_.last_raw_angle();

            gimbal_left_friction_.update_status();
            gimbal_right_friction_.update_status();
            gimbal_bullet_feeder_.update_status();
        }

        void command_update() {
            auto top_yaw_packet = device::CanPacket8{
                device::CanPacket8::PaddingQuarter{}, gimbal_top_yaw_motor_.generate_command(),
                device::CanPacket8::PaddingQuarter{}, gimbal_bullet_feeder_.generate_command()};
            auto friction_packet = device::CanPacket8{
                gimbal_right_friction_.generate_command(), gimbal_left_friction_.generate_command(),
                device::CanPacket8::PaddingQuarter{}, device::CanPacket8::PaddingQuarter{}};
            auto pitch_packet = gimbal_pitch_motor_.generate_torque_command();

            board_->start_transmit()
                .can_transmit(
                    Spec::kCans.kCan1, {.can_id = 0x1FF, .can_data = top_yaw_packet.as_bytes()})
                .can_transmit(
                    Spec::kCans.kCan1, {.can_id = 0x200, .can_data = friction_packet.as_bytes()})
                .can_transmit(
                    Spec::kCans.kCan2, {.can_id = 0x4, .can_data = pitch_packet.as_bytes()});
        }

    private:
        void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
                return;

            const auto& can_id = data.can_id;
            if (can == Spec::kCans.kCan1) {
                if (can_id == gimbal_top_yaw_motor_.recv_id()) {
                    gimbal_top_yaw_motor_.store_status(data.can_data);
                } else if (can_id == gimbal_left_friction_.recv_id()) {
                    gimbal_left_friction_.store_status(data.can_data);
                } else if (can_id == gimbal_right_friction_.recv_id()) {
                    gimbal_right_friction_.store_status(data.can_data);
                } else if (can_id == gimbal_bullet_feeder_.recv_id()) {
                    gimbal_bullet_feeder_.store_status(data.can_data);
                }
            } else if (can == Spec::kCans.kCan2) {
                if (can_id == 0x214) {
                    gimbal_pitch_motor_.store_status(data.can_data);
                }
            }
        }

        void uart_receive_callback(const Spec::Uart& uart, const View::Uart& data) override {
            if (uart == Spec::kUarts.kDbus) {
                dr16_.store_status(data.uart_data.data(), data.uart_data.size());
            } else if (uart == Spec::kUarts.kUart2) {
                gy614_.store_status(data.uart_data.data(), data.uart_data.size());
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

        OutputInterface<rmcs_description::Tf>& tf_;

        device::Bmi088Ekf imu_{device::Bmi088Ekf::Config{}};
        device::BoardClockLifter board_clock_lifter_;
        device::Gy614 gy614_;
        device::Dr16 dr16_;

        OutputInterface<double> gimbal_yaw_velocity_imu_;
        OutputInterface<double> gimbal_pitch_velocity_imu_;
        OutputInterface<double> debug_pitch_raw_angle_;
        OutputInterface<double> debug_top_yaw_raw_angle_;

        int16_t imu_bias_x = 0, imu_bias_y = 0, imu_bias_z = 0;

        filter::LowPassFilter<> imu_gy_velocity_filter_{3.0f, 1000.0f};
        filter::LowPassFilter<> imu_gz_velocity_filter_{8.0f, 1000.0f};

        device::DjiMotor gimbal_top_yaw_motor_;
        device::DmMotor gimbal_pitch_motor_;

        device::DjiMotor gimbal_left_friction_;
        device::DjiMotor gimbal_right_friction_;

        device::DjiMotor gimbal_bullet_feeder_;

        std::unique_ptr<librmcs::board::CBoard> board_;
    };

    class BottomBoard final : public librmcs::board::CBoard::Callback {
    public:
        friend class DualSentry;
        explicit BottomBoard(
            DualSentry& dual_sentry, DualSentryCommand& dual_sentry_command,
            std::string_view board_serial = {})
            : tf_(dual_sentry.tf_)
            , gimbal_bottom_yaw_motor_(dual_sentry, dual_sentry_command, "/gimbal/bottom_yaw")
            , chassis_wheel_motors_(
                  {dual_sentry, dual_sentry_command, "/chassis/left_front_wheel",
                   device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 1}},
                  {dual_sentry, dual_sentry_command, "/chassis/right_front_wheel",
                   device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 2}},
                  {dual_sentry, dual_sentry_command, "/chassis/right_back_wheel",
                   device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 3}},
                  {dual_sentry, dual_sentry_command, "/chassis/left_back_wheel",
                   device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 4}})
            , supercap_(dual_sentry, 28.5) {

            gimbal_bottom_yaw_motor_.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kGM6020Voltage, 1}
                    .set_encoder_zero_point(
                        static_cast<int>(
                            dual_sentry.get_parameter("bottom_yaw_motor_zero_point").as_int())));

            dual_sentry.register_output("/referee/serial", referee_serial_);
            referee_serial_->read = [this](std::byte* buffer, size_t size) {
                return referee_ring_buffer_receive_.pop_front_n(
                    [&buffer](std::byte byte) noexcept { *buffer++ = byte; }, size);
            };
            referee_serial_->write = [this](const std::byte* buffer, size_t size) {
                board_->start_transmit().uart_transmit(
                    Spec::kUarts.kUart1, {.uart_data = std::span<const std::byte>{buffer, size}});
                return size;
            };

            dual_sentry.register_output("/chassis/yaw/velocity_imu", chassis_yaw_velocity_imu_, 0);

            board_ = std::make_unique<librmcs::board::CBoard>(*this, board_serial);
        }

        ~BottomBoard() final = default;

        void update() {
            if (const auto snapshot = imu_.snapshot())
                *chassis_yaw_velocity_imu_ = snapshot->gyro_body.z();

            gimbal_bottom_yaw_motor_.update_status();
            tf_->set_state<rmcs_description::GimbalCenterLink, rmcs_description::YawLink>(
                gimbal_bottom_yaw_motor_.angle());
            fast_tf::rcl::broadcast_all(*tf_);

            for (auto& motor : chassis_wheel_motors_)
                motor.update_status();

            supercap_.update_status();
        }

        void command_update() {
            auto wheels_packet = device::CanPacket8{
                chassis_wheel_motors_[0].generate_command(),
                chassis_wheel_motors_[1].generate_command(),
                chassis_wheel_motors_[2].generate_command(),
                chassis_wheel_motors_[3].generate_command()};
            auto bottom_yaw_packet = device::CanPacket8{
                gimbal_bottom_yaw_motor_.generate_command(), device::CanPacket8::PaddingQuarter{},
                device::CanPacket8::PaddingQuarter{}, device::CanPacket8::PaddingQuarter{}};

            board_->start_transmit()
                .can_transmit(
                    Spec::kCans.kCan1, {.can_id = 0x200, .can_data = wheels_packet.as_bytes()})
                .can_transmit(
                    Spec::kCans.kCan1, {.can_id = 0x1FF, .can_data = bottom_yaw_packet.as_bytes()});
        }

    private:
        void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
                return;
            if (can != Spec::kCans.kCan1) [[unlikely]]
                return;

            const auto& can_id = data.can_id;
            if (can_id == gimbal_bottom_yaw_motor_.recv_id()) {
                gimbal_bottom_yaw_motor_.store_status(data.can_data);
            } else if (can_id == 0x20c) {
                supercap_.store_status(data.can_data);
            } else {
                for (auto& motor : chassis_wheel_motors_)
                    if (motor.match_then_store_status(can_id, data.can_data))
                        break;
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
        OutputInterface<rmcs_description::Tf>& tf_;

        OutputInterface<double> chassis_yaw_velocity_imu_;

        device::DjiMotor gimbal_bottom_yaw_motor_;
        device::DjiMotor chassis_wheel_motors_[4];
        device::Jsupercap supercap_;

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

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::DualSentry, rmcs_executor::Component)
