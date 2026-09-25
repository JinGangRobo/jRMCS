#pragma once

#include <algorithm>
#include <atomic>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <span>
#include <string>
#include <utility>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_utility/tick_timer.hpp>

#include "filter/low_pass_filter.hpp"
#include "hardware/device/can_packet.hpp"

namespace rmcs_core::hardware::device {

class DmMotor {
public:
    enum class Type : uint8_t { kJ4310 };

    enum class ErrorMsg : uint8_t {
        kDisable = 0,
        kEnable = 1,
        kUnderVoltage = 0,
        kOverVoltage = 8,
        kOverCurrent = 11,
        kMosOverTemperature = 12,
        kRotorOverTemperature = 13,
        kCommunicationError = 0xd,
        kOverload = 15,
    };

    struct Config {
        explicit Config(Type motor_type)
            : motor_type(motor_type) {}

        Config& set_encoder_zero_point(int value) { return encoder_zero_point = value, *this; }
        Config& set_reduction_ratio(double value) { return reduction_ratio = value, *this; }
        Config& set_reversed() { return reversed = true, *this; }
        Config& enable_multi_turn_angle() { return multi_turn_angle_enabled = true, *this; }

        Type motor_type;
        int encoder_zero_point = 0;
        double reduction_ratio = 1.0;
        bool reversed = false;
        bool multi_turn_angle_enabled = false;
    };

    DmMotor(
        rmcs_executor::Component& status_component, rmcs_executor::Component& command_component,
        const std::string& name_prefix)
        : name_prefix_(name_prefix) {
        status_component.register_output(name_prefix + "/angle", angle_output_, 0.0);
        status_component.register_output(name_prefix + "/raw_angle", raw_angle_output_, 0.0);
        status_component.register_output(name_prefix + "/velocity", velocity_output_, 0.0);
        status_component.register_output(
            name_prefix + "/velocity_filtered", velocity_filtered_output_, 0.0);
        status_component.register_output(name_prefix + "/torque", torque_output_, 0.0);
        status_component.register_output(name_prefix + "/max_torque", max_torque_output_, 0.0);
        status_component.register_output(name_prefix + "/temperature", temperature_output_, 0.0);
        status_component.register_output(name_prefix + "/alive", alive_output_, false);

        command_component.register_input(name_prefix + "/control_torque", control_torque_, false);
        command_component.register_input(
            name_prefix + "/control_velocity", control_velocity_, false);

        alive_watchdog_.reset(50);
    }

    DmMotor(
        rmcs_executor::Component& status_component, rmcs_executor::Component& command_component,
        const std::string& name_prefix, const Config& config)
        : DmMotor(status_component, command_component, name_prefix) {
        configure(config);
    }

    DmMotor(const DmMotor&) = delete;
    DmMotor& operator=(const DmMotor&) = delete;
    DmMotor(DmMotor&&) = delete;
    DmMotor& operator=(DmMotor&&) = delete;

    ~DmMotor() = default;

    void configure(const Config& config) {
        motor_type_ = config.motor_type;

        encoder_zero_point_ = config.encoder_zero_point % kRawAngleMax;
        if (encoder_zero_point_ < 0)
            encoder_zero_point_ += kRawAngleMax;

        reversed_ = config.reversed;
        multi_turn_angle_enabled_ = config.multi_turn_angle_enabled;

        angle_multi_turn_ = 0;
        last_raw_angle_ = 0;

        switch (config.motor_type) {
        case Type::kJ4310:
            // Note: maximum torque is taken from the manufacturer's documentation and is used as a
            // reference only; the MIT command clamps to the protocol range regardless.
            max_torque_ = 10.0;
            break;
        default: std::unreachable();
        }

        *max_torque_output_ = max_torque();
    }

    void store_status(std::span<const std::byte> can_data) {
        if (can_data.size() != 8) [[unlikely]]
            return;

        can_packet_.store(CanPacket8{can_data}, std::memory_order::relaxed);

        *alive_output_ = true;
        alive_watchdog_.reset(50);
    }

    void update_status() {
        const auto feedback =
            std::bit_cast<DmMotorFeedback>(can_packet_.load(std::memory_order::relaxed));

        last_error_msg_ = static_cast<ErrorMsg>((feedback.id_err >> 4) & 0x0F);

        // Temperature unit: celsius
        temperature_ = static_cast<double>(feedback.temp_mos);
        mos_temperature_ = static_cast<double>(feedback.temp_rotor);

        // Angle unit: rad
        const auto raw_angle = static_cast<uint16_t>(
            (static_cast<uint16_t>(feedback.pos_high) << 8) | feedback.pos_low);
        int calibrated_raw_angle = static_cast<int>(raw_angle) - encoder_zero_point_;
        if (calibrated_raw_angle < 0)
            calibrated_raw_angle += kRawAngleMax;
        if (reversed_)
            calibrated_raw_angle = kRawAngleMax - calibrated_raw_angle;

        if (!multi_turn_angle_enabled_) {
            // The J4310 reports a 16-bit raw angle whose single-turn resolution covers 14 bits.
            angle_ =
                static_cast<double>(calibrated_raw_angle & 0x1FFF) / 8192.0 * 2 * std::numbers::pi;
        } else {
            auto diff = (calibrated_raw_angle - angle_multi_turn_) % kRawAngleMax;
            if (diff <= -kRawAngleMax / 2)
                diff += kRawAngleMax;
            else if (diff > kRawAngleMax / 2)
                diff -= kRawAngleMax;
            angle_multi_turn_ += diff;
            angle_ = static_cast<double>(angle_multi_turn_) / 8192.0 * 2 * std::numbers::pi;
        }
        last_raw_angle_ = static_cast<int>(raw_angle);

        const auto velocity_raw = static_cast<uint16_t>(
            (static_cast<uint16_t>(feedback.vel_high) << 4)
            | ((feedback.vel_low_torque_high >> 4) & 0x0F));
        const auto torque_raw = static_cast<uint16_t>(
            (static_cast<uint16_t>(feedback.vel_low_torque_high & 0x0F) << 8)
            | feedback.torque_low);

        // Velocity unit: rad/s
        velocity_ = (static_cast<double>(velocity_raw) - 2048.0) / 4096.0 * 60.0;

        // Torque unit: N*m
        torque_ = (static_cast<double>(torque_raw) - 2048.0) / 4096.0 * 20.0;

        if (alive_watchdog_.tick()) {
            *alive_output_ = false;
            RCLCPP_WARN(
                rclcpp::get_logger("HW_Diag"), "Dm motor %s offline!", name_prefix_.c_str());
        }

        *angle_output_ = angle();
        *raw_angle_output_ = static_cast<double>(last_raw_angle());
        *velocity_output_ = velocity();
        *velocity_filtered_output_ = velocity_filter_.update(velocity());
        *torque_output_ = torque();
        *temperature_output_ = temperature();
    }

    int calibrate_zero_point() {
        angle_multi_turn_ = 0;
        encoder_zero_point_ = last_raw_angle_;
        return encoder_zero_point_;
    }

    CanPacket8 generate_torque_command(double control_torque) const {
        if (std::isnan(control_torque))
            control_torque = 0.0;
        if (last_error_msg_ == ErrorMsg::kDisable)
            return generate_error_command(0xfc);
        if (last_error_msg_ == ErrorMsg::kCommunicationError)
            return generate_error_command(0xfb);

        return to_mit_control_command(
            0.0f, 0.0f, 0.0f, 0.0f,
            (reversed_ ? -1.0f : 1.0f) * static_cast<float>(control_torque));
    }

    CanPacket8 generate_torque_command() const { return generate_torque_command(control_torque()); }

    CanPacket8 generate_velocity_command(double control_velocity) const {
        if (std::isnan(control_velocity))
            control_velocity = 0.0;
        if (last_error_msg_ == ErrorMsg::kDisable)
            return generate_error_command(0xfc);
        if (last_error_msg_ == ErrorMsg::kCommunicationError)
            return generate_error_command(0xfb);

        return to_mit_control_command(
            0.0f, (reversed_ ? -1.0f : 1.0f) * static_cast<float>(control_velocity), 0.0f, 0.1f,
            0.0f);
    }

    CanPacket8 generate_velocity_command() const {
        return generate_velocity_command(control_velocity());
    }

    double control_torque() const {
        if (control_torque_.ready()) [[likely]]
            return *control_torque_;
        else
            return 0.0;
    }

    double control_velocity() const {
        if (control_velocity_.ready()) [[likely]]
            return *control_velocity_;
        else
            return 0.0;
    }

    double angle() const { return angle_; }
    double velocity() const { return velocity_; }
    double torque() const { return torque_; }
    double max_torque() const { return max_torque_; }
    double temperature() const { return temperature_; }
    double mos_temperature() const { return mos_temperature_; }
    int last_raw_angle() const { return last_raw_angle_; }
    ErrorMsg last_error_msg() const { return last_error_msg_; }

private:
    static uint16_t float_to_uint(float value, float min, float max, int bits) {
        const float span = max - min;
        if (span <= 0.0f)
            return 0;
        const float normalized = std::clamp((value - min) / span, 0.0f, 1.0f);
        return static_cast<uint16_t>(normalized * static_cast<float>((1 << bits) - 1));
    }

    static CanPacket8
        to_mit_control_command(float position, float velocity, float kp, float kd, float torque) {
        const uint16_t position_raw = float_to_uint(position, kPositionMin, kPositionMax, 16);
        const uint16_t velocity_raw = float_to_uint(velocity, kVelocityMin, kVelocityMax, 12);
        const uint16_t kp_raw = float_to_uint(kp, kKpMin, kKpMax, 12);
        const uint16_t kd_raw = float_to_uint(kd, kKdMin, kKdMax, 12);
        const uint16_t torque_raw = float_to_uint(torque, kTorqueMin, kTorqueMax, 12);

        const struct [[gnu::packed]] {
            uint8_t position_high;
            uint8_t position_low;
            uint8_t velocity_high;
            uint8_t velocity_low_kp_high;
            uint8_t kp_low;
            uint8_t kd_high;
            uint8_t kd_low_torque_high;
            uint8_t torque_low;
        } command alignas(CanPacket8){
            .position_high = static_cast<uint8_t>(position_raw >> 8),
            .position_low = static_cast<uint8_t>(position_raw),
            .velocity_high = static_cast<uint8_t>(velocity_raw >> 4),
            .velocity_low_kp_high =
                static_cast<uint8_t>(((velocity_raw & 0x0F) << 4) | (kp_raw >> 8)),
            .kp_low = static_cast<uint8_t>(kp_raw),
            .kd_high = static_cast<uint8_t>(kd_raw >> 4),
            .kd_low_torque_high = static_cast<uint8_t>(((kd_raw & 0x0F) << 4) | (torque_raw >> 8)),
            .torque_low = static_cast<uint8_t>(torque_raw),
        };

        return std::bit_cast<CanPacket8>(command);
    }

    static CanPacket8 generate_error_command(uint8_t code) {
        const struct [[gnu::packed]] {
            uint8_t filler[7];
            uint8_t command;
        } packet alignas(CanPacket8){
            .filler = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
            .command = code,
        };

        return std::bit_cast<CanPacket8>(packet);
    }

    struct alignas(CanPacket8) DmMotorFeedback {
        uint8_t id_err;              // D[0]: ID | ERR << 4
        uint8_t pos_high;            // D[1]: POS[15:8]
        uint8_t pos_low;             // D[2]: POS[7:0]
        uint8_t vel_high;            // D[3]: VEL[11:4]
        uint8_t vel_low_torque_high; // D[4]: VEL[3:0] | T[11:8]
        uint8_t torque_low;          // D[5]: T[7:0]
        uint8_t temp_mos;            // D[6]: T_MOS
        uint8_t temp_rotor;          // D[7]: T_Rotor
    };

    static constexpr int kRawAngleMax = 65536;

    static constexpr float kPositionMin = -25.1327f;
    static constexpr float kPositionMax = 25.1327f;
    static constexpr float kVelocityMin = -30.0f;
    static constexpr float kVelocityMax = 30.0f;
    static constexpr float kKpMin = 0.0f;
    static constexpr float kKpMax = 500.0f;
    static constexpr float kKdMin = 0.0f;
    static constexpr float kKdMax = 5.0f;
    static constexpr float kTorqueMin = -10.0f;
    static constexpr float kTorqueMax = 10.0f;

    Type motor_type_ = Type::kJ4310;

    int encoder_zero_point_ = 0;
    int last_raw_angle_ = 0;

    bool reversed_ = false;
    bool multi_turn_angle_enabled_ = false;
    int64_t angle_multi_turn_ = 0;

    double angle_ = 0.0;
    double velocity_ = 0.0;
    double torque_ = 0.0;
    double max_torque_ = 0.0;
    double temperature_ = 0.0;
    double mos_temperature_ = 0.0;
    ErrorMsg last_error_msg_ = ErrorMsg::kDisable;

    std::string name_prefix_;
    std::atomic<CanPacket8> can_packet_;
    rmcs_utility::TickTimer alive_watchdog_;
    filter::LowPassFilter<> velocity_filter_{4, 1000};

    rmcs_executor::Component::OutputInterface<double> angle_output_;
    rmcs_executor::Component::OutputInterface<double> raw_angle_output_;
    rmcs_executor::Component::OutputInterface<double> velocity_output_;
    rmcs_executor::Component::OutputInterface<double> velocity_filtered_output_;
    rmcs_executor::Component::OutputInterface<double> torque_output_;
    rmcs_executor::Component::OutputInterface<double> max_torque_output_;
    rmcs_executor::Component::OutputInterface<double> temperature_output_;
    rmcs_executor::Component::OutputInterface<bool> alive_output_;

    rmcs_executor::Component::InputInterface<double> control_torque_;
    rmcs_executor::Component::InputInterface<double> control_velocity_;
};

} // namespace rmcs_core::hardware::device
