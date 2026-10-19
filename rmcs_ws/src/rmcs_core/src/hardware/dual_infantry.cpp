#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <numbers>
#include <span>
#include <string>
#include <string_view>

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
#include <rmcs_msgs/board_clock.hpp>
#include <rmcs_msgs/imu_snapshot.hpp>
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
#include "hardware/device/remote_control.hpp"

namespace rmcs_core::hardware {

class DualInfantry
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DualInfantry()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , command_component_(create_partner_component<DualInfantryCommand>(
              get_component_name() + "_command", *this)) {
        using namespace rmcs_description;

        register_output("/tf", tf_);
        tf_->set_transform<PitchLink, CameraLink>(Eigen::Translation3d{0.084, 0.0, 0.048});
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

    ~DualInfantry() override = default;

    void update() override {
        top_board_->update();
        bottom_board_->update();
        remote_control_->update();

        if (tf_broadcast_counter_++ % kTfBroadcastDivider == 0)
            fast_tf::rcl::broadcast_all(*tf_);
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

    class DualInfantryCommand : public rmcs_executor::Component {
    public:
        explicit DualInfantryCommand(DualInfantry& dual_infantry)
            : dual_infantry_(dual_infantry) {}

        void update() override { dual_infantry_.command_update(); }

        DualInfantry& dual_infantry_;
    };
    std::shared_ptr<DualInfantryCommand> command_component_;

    class TopBoard final : public librmcs::board::CBoard::Callback {
    public:
        friend class DualInfantry;
        explicit TopBoard(
            DualInfantry& dual_infantry, DualInfantryCommand& dual_infantry_command,
            std::string_view board_serial = {})
            : tf_(dual_infantry.tf_)
            , gy614_(dual_infantry, "/friction_wheels/temperature")
            , dr16_{}
            , imu_bias_x(static_cast<int16_t>(dual_infantry.get_parameter("imu_bias_x").as_int()))
            , imu_bias_y(static_cast<int16_t>(dual_infantry.get_parameter("imu_bias_y").as_int()))
            , imu_bias_z(static_cast<int16_t>(dual_infantry.get_parameter("imu_bias_z").as_int()))
            , gimbal_top_yaw_motor_(dual_infantry, dual_infantry_command, "/gimbal/top_yaw")
            , gimbal_pitch_motor_(dual_infantry, dual_infantry_command, "/gimbal/pitch")
            , gimbal_left_friction_(dual_infantry, dual_infantry_command, "/gimbal/left_friction")
            , gimbal_right_friction_(dual_infantry, dual_infantry_command, "/gimbal/right_friction")
            , gimbal_bullet_feeder_(dual_infantry, dual_infantry_command, "/gimbal/bullet_feeder") {

            // TODO(hardware-migration): v2 used the GM6020 current-loop frame (raw_current_max
            // 16384). Confirm the physical yaw motor control mode on the real robot.
            gimbal_top_yaw_motor_.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kGM6020, 2}
                    .set_encoder_zero_point(static_cast<int>(
                        dual_infantry.get_parameter("top_yaw_motor_zero_point").as_int())));

            gimbal_pitch_motor_.configure(
                device::DmMotor::Config{device::DmMotor::Type::kJ4310}
                    .set_reversed()
                    .set_encoder_zero_point(static_cast<int>(
                        dual_infantry.get_parameter("pitch_motor_zero_point").as_int())));

            gimbal_left_friction_.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 2}
                    .set_reduction_ratio(1.)
                    .set_reversed());
            gimbal_right_friction_.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 1}.set_reduction_ratio(
                    1.));

            gimbal_bullet_feeder_.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 3}
                    .enable_multi_turn_angle()
                    .set_reversed()
                    .set_reduction_ratio(19 * 2));

            dual_infantry.register_output("/gimbal/yaw/velocity_imu", gimbal_yaw_velocity_imu_);
            dual_infantry.register_output("/gimbal/pitch/velocity_imu", gimbal_pitch_velocity_imu_);

            dual_infantry.register_output("/debug/pitch/raw_angle", debug_pitch_raw_angle_);
            dual_infantry.register_output("/debug/top_board/imu_gyro_x", debug_gyro_x_, 0.0);
            dual_infantry.register_output("/debug/top_board/imu_gyro_y", debug_gyro_y_, 0.0);
            dual_infantry.register_output("/debug/top_board/imu_gyro_z", debug_gyro_z_, 0.0);
            dual_infantry.register_output("/debug/top_board/imu_roll", debug_imu_roll_, 0.0);
            dual_infantry.register_output("/debug/top_board/imu_pitch", debug_imu_pitch_, 0.0);
            dual_infantry.register_output("/debug/top_board/imu_yaw", debug_imu_yaw_, 0.0);

            dual_infantry.remote_control_->register_dr16(&dr16_);

            // Auto-aim synchronization: the attitude filter publishes a timestamped IMU snapshot
            // that the auto-aim capturer pairs with camera frames.
            dual_infantry.register_output("/gimbal/auto_aim/imu_snapshot", imu_snapshot_output_);

            board_ = std::make_unique<librmcs::board::CBoard>(*this, board_serial);

            // TODO(hardware-migration): the camera exposure signal must be wired to a C-board GPIO
            // channel for hardware-triggered capture. The channel is not yet confirmed on the real
            // robot, so hardware sync stays disabled unless `camera_sync_gpio_channel` is set.
            const int camera_sync_gpio_channel =
                static_cast<int>(dual_infantry.get_parameter_or("camera_sync_gpio_channel", -1));
            if (camera_sync_gpio_channel >= 0) {
                dual_infantry.register_output(
                    "/gimbal/auto_aim/exposure_signal", camera_signal_output_);
                camera_signal_enabled_ = true;
                board_->start_transmit().gpio_digital_read(
                    Spec::kGpios[static_cast<std::size_t>(camera_sync_gpio_channel)],
                    {
                        .period_ms = 0,
                        .asap = false,
                        .rising_edge = false,
                        .falling_edge = true,
                        .capture_timestamp = true,
                        .pull = librmcs::data::GpioPull::kUp,
                    });
            }
        }

        ~TopBoard() final = default;

        void update() {
            gimbal_top_yaw_motor_.update_status();
            gimbal_pitch_motor_.update_status();

            if (const auto imu_snapshot = imu_.snapshot()) {
                const Eigen::Quaterniond gimbal_imu_pose = imu_snapshot->orientation;
                const Eigen::Vector3d& gyro_body = imu_snapshot->gyro_body;

                const Eigen::Matrix3d rotation = gimbal_imu_pose.toRotationMatrix();
                *debug_imu_roll_ = std::atan2(rotation(2, 1), rotation(2, 2));
                *debug_imu_pitch_ = std::asin(std::clamp(-rotation(2, 0), -1.0, 1.0));
                *debug_imu_yaw_ = std::atan2(rotation(1, 0), rotation(0, 0));

                tf_->set_transform<rmcs_description::PitchLink, rmcs_description::OdomImu>(
                    gimbal_imu_pose.conjugate());
                tf_->set_transform<rmcs_description::BaseLink, rmcs_description::RawImu>(
                    gimbal_imu_pose);

                *gimbal_yaw_velocity_imu_ = imu_gz_velocity_filter_.update(gyro_body.z());
                *gimbal_pitch_velocity_imu_ = imu_gy_velocity_filter_.update(gyro_body.y());

                *debug_gyro_x_ = gyro_body.x();
                *debug_gyro_y_ = gyro_body.y();
                *debug_gyro_z_ = gyro_body.z();
            }

            tf_->set_state<rmcs_description::YawLink, rmcs_description::PitchLink>(
                gimbal_pitch_motor_.angle());

            gy614_.update_status();
            dr16_.update_status();

            *debug_pitch_raw_angle_ = gimbal_pitch_motor_.last_raw_angle();

            gimbal_left_friction_.update_status();
            gimbal_right_friction_.update_status();
            gimbal_bullet_feeder_.update_status();
        }

        void command_update() {
            auto top_yaw_packet = device::CanPacket8{
                device::CanPacket8::PaddingQuarter{}, gimbal_top_yaw_motor_.generate_command(),
                device::CanPacket8::PaddingQuarter{}, device::CanPacket8::PaddingQuarter{}};
            auto friction_packet = device::CanPacket8{
                gimbal_right_friction_.generate_command(), gimbal_left_friction_.generate_command(),
                gimbal_bullet_feeder_.generate_command(), device::CanPacket8::PaddingQuarter{}};
            auto pitch_packet = gimbal_pitch_motor_.generate_torque_command();

            board_->start_transmit()
                .can_transmit(
                    Spec::kCans.kCan1, {.can_id = 0x1FF, .can_data = top_yaw_packet.as_bytes()})
                .can_transmit(
                    Spec::kCans.kCan1, {.can_id = 0x200, .can_data = friction_packet.as_bytes()})
                // TODO(hardware-migration): v2 drove the pitch motor on CAN2 (tx id 0x3 / rx id
                // 0x213). Verify the physical bus wiring on the real robot.
                .can_transmit(
                    Spec::kCans.kCan2, {.can_id = 0x3, .can_data = pitch_packet.as_bytes()});
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
                // TODO(hardware-migration): v2 received the DM pitch feedback on CAN2 (0x213).
                if (can_id == 0x213) {
                    gimbal_pitch_motor_.store_status(data.can_data);
                }
            }
        }

        void uart_receive_callback(const Spec::Uart& uart, const View::Uart& data) override {
            if (uart == Spec::kUarts.kDbus)
                dr16_.store_status(data.uart_data.data(), data.uart_data.size());
            else if (uart == Spec::kUarts.kUart2)
                gy614_.store_status(data.uart_data.data(), data.uart_data.size());
        }

        void accelerometer_receive_callback(const View::ImuAccelerometer& data) override {
            const auto timestamp = board_clock_lifter_.advance_timebase(data.timestamp_quarter_us);
            imu_.push_accelerometer_sample(data.x, data.y, data.z, timestamp);
        }

        void gyroscope_receive_callback(const View::ImuGyroscope& data) override {
            const auto timestamp = board_clock_lifter_.lift_timestamp(data.timestamp_quarter_us);
            if (!timestamp.has_value())
                return;
            if (const auto snapshot = imu_.try_update_with_gyroscope_sample(
                    data.x - imu_bias_x, data.y - imu_bias_y, data.z - imu_bias_z, *timestamp))
                imu_snapshot_output_.emit(*snapshot);
        }

        void gpio_digital_read_result_callback(
            const Spec::Gpio& /*gpio*/, const View::GpioDigital& data) override {
            if (!camera_signal_enabled_ || !data.timestamp_quarter_us)
                return;
            if (const auto timestamp =
                    board_clock_lifter_.lift_timestamp(*data.timestamp_quarter_us))
                camera_signal_output_.emit(*timestamp);
        }

        OutputInterface<rmcs_description::Tf>& tf_;

        // TODO(hardware-migration): v2 mapped the top-board IMU as an identity (x, y, z).
        // Re-verify the physical sensor orientation on the real robot.
        device::Bmi088Ekf imu_{};
        device::BoardClockLifter board_clock_lifter_;
        device::Gy614 gy614_;
        device::Dr16 dr16_;

        OutputInterface<double> gimbal_yaw_velocity_imu_;
        OutputInterface<double> gimbal_pitch_velocity_imu_;
        OutputInterface<double> debug_pitch_raw_angle_;
        OutputInterface<double> debug_gyro_x_, debug_gyro_y_, debug_gyro_z_;
        OutputInterface<double> debug_imu_roll_, debug_imu_pitch_, debug_imu_yaw_;

        // Auto-aim synchronization events consumed by AutoAimCapturerComponent.
        EventOutputInterface<rmcs_msgs::ImuSnapshot> imu_snapshot_output_;
        EventOutputInterface<rmcs_msgs::BoardClock::time_point> camera_signal_output_;
        bool camera_signal_enabled_ = false;

        filter::LowPassFilter<> imu_gy_velocity_filter_{10.0f, 1000.0f};
        filter::LowPassFilter<> imu_gz_velocity_filter_{30.0f, 1000.0f};

        int16_t imu_bias_x = 0, imu_bias_y = 0, imu_bias_z = 0;

        device::DjiMotor gimbal_top_yaw_motor_;
        device::DmMotor gimbal_pitch_motor_;

        device::DjiMotor gimbal_left_friction_;
        device::DjiMotor gimbal_right_friction_;

        device::DjiMotor gimbal_bullet_feeder_;

        std::unique_ptr<librmcs::board::CBoard> board_;
    };

    class BottomBoard final : public librmcs::board::CBoard::Callback {
    public:
        friend class DualInfantry;
        explicit BottomBoard(
            DualInfantry& dual_infantry, DualInfantryCommand& dual_infantry_command,
            std::string_view board_serial = {})
            : tf_(dual_infantry.tf_)
            , gimbal_bottom_yaw_motor_(dual_infantry, dual_infantry_command, "/gimbal/bottom_yaw")
            , chassis_wheel_motors_(
                  {dual_infantry, dual_infantry_command, "/chassis/left_front_wheel",
                   device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 1}},
                  {dual_infantry, dual_infantry_command, "/chassis/right_front_wheel",
                   device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 2}},
                  {dual_infantry, dual_infantry_command, "/chassis/right_back_wheel",
                   device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 3}},
                  {dual_infantry, dual_infantry_command, "/chassis/left_back_wheel",
                   device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 4}})
            , supercap_(dual_infantry, 28.5) {

            // TODO(hardware-migration): see the top-board note; v2 used the GM6020 current-loop
            // frame.
            gimbal_bottom_yaw_motor_.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kGM6020, 1}
                    .set_encoder_zero_point(static_cast<int>(
                        dual_infantry.get_parameter("bottom_yaw_motor_zero_point").as_int())));

            dual_infantry.register_output(
                "/chassis/yaw/velocity_imu", chassis_yaw_velocity_imu_, 0.0);

            dual_infantry.register_output("/debug/bottom_board/imu_gyro_x", debug_gyro_x_, 0.0);
            dual_infantry.register_output("/debug/bottom_board/imu_gyro_y", debug_gyro_y_, 0.0);
            dual_infantry.register_output("/debug/bottom_board/imu_gyro_z", debug_gyro_z_, 0.0);
            dual_infantry.register_output(
                "/debug/bottom_board/imu_roll", debug_imu_roll_, 0.0);
            dual_infantry.register_output(
                "/debug/bottom_board/imu_pitch", debug_imu_pitch_, 0.0);
            dual_infantry.register_output("/debug/bottom_board/imu_yaw", debug_imu_yaw_, 0.0);

            dual_infantry.register_output("/referee/serial", referee_serial_);
            referee_serial_->read = [this](std::byte* buffer, size_t size) {
                return referee_ring_buffer_receive_.pop_front_n(
                    [&buffer](std::byte byte) noexcept { *buffer++ = byte; }, size);
            };
            referee_serial_->write = [this](const std::byte* buffer, size_t size) {
                board_->start_transmit().uart_transmit(
                    Spec::kUarts.kUart1, {.uart_data = std::span<const std::byte>{buffer, size}});
                return size;
            };

            board_ = std::make_unique<librmcs::board::CBoard>(*this, board_serial);
        }

        ~BottomBoard() final = default;

        void update() {
            if (const auto imu_snapshot = imu_.snapshot()) {
                const Eigen::Quaterniond chassis_imu_pose = imu_snapshot->orientation;
                const Eigen::Vector3d& gyro_body = imu_snapshot->gyro_body;

                *chassis_yaw_velocity_imu_ = imu_gz_velocity_filter_.update(gyro_body.z());

                *debug_gyro_x_ = gyro_body.x();
                *debug_gyro_y_ = gyro_body.y();
                *debug_gyro_z_ = gyro_body.z();

                const Eigen::Matrix3d chassis_rotation = chassis_imu_pose.toRotationMatrix();
                *debug_imu_roll_ = std::atan2(chassis_rotation(2, 1), chassis_rotation(2, 2));
                *debug_imu_pitch_ = std::asin(std::clamp(-chassis_rotation(2, 0), -1.0, 1.0));
                *debug_imu_yaw_ = std::atan2(chassis_rotation(1, 0), chassis_rotation(0, 0));
            }

            gimbal_bottom_yaw_motor_.update_status();
            tf_->set_state<rmcs_description::GimbalCenterLink, rmcs_description::YawLink>(
                gimbal_bottom_yaw_motor_.angle());

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

        device::Bmi088Ekf imu_{};
        device::BoardClockLifter board_clock_lifter_;
        OutputInterface<rmcs_description::Tf>& tf_;

        OutputInterface<double> chassis_yaw_velocity_imu_;
        OutputInterface<double> debug_gyro_x_, debug_gyro_y_, debug_gyro_z_;
        OutputInterface<double> debug_imu_roll_, debug_imu_pitch_, debug_imu_yaw_;

        device::DjiMotor gimbal_bottom_yaw_motor_;
        device::DjiMotor chassis_wheel_motors_[4];
        device::Jsupercap supercap_;

        filter::LowPassFilter<> imu_gz_velocity_filter_{4.0f, 1000.0f};

        rmcs_utility::RingBuffer<std::byte> referee_ring_buffer_receive_{256};
        OutputInterface<rmcs_msgs::SerialInterface> referee_serial_;

        std::unique_ptr<librmcs::board::CBoard> board_;
    };

    OutputInterface<rmcs_description::Tf> tf_;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr gimbal_calibrate_subscription_;

    std::unique_ptr<device::RemoteControl> remote_control_;

    std::unique_ptr<TopBoard> top_board_;
    std::unique_ptr<BottomBoard> bottom_board_;

    static constexpr int kTfBroadcastDivider = 10; // 1 kHz control loop -> 100 Hz /tf
    int tf_broadcast_counter_ = 0;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::DualInfantry, rmcs_executor::Component)
