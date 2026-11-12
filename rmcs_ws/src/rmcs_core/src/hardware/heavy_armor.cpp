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
#include "hardware/device/dm_motor.hpp"
#include "hardware/device/dr16.hpp"

namespace rmcs_core::hardware {
class HeavyArmor
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    HeavyArmor()
        : Node{
            get_component_name(),
            rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , command_component_(
            create_partner_component<HeavyArmorCommand>(
                get_component_name() + "_command", *this)) {
        using namespace rmcs_description;
        register_output("/tf", tf_);           
        
        arm_board_ = std::make_unique<ArmBoard>(
            *this, *command_component_, get_parameter("board_serial_arm_board").as_string());
    }

    ~HeavyArmor() final = default;

    void update() override {
        arm_board_->update();
    }

    void command_update() {
        arm_board_->command_update();
    }
    
private:
    class HeavyArmorCommand : public rmcs_executor::Component {
    public:
        explicit HeavyArmorCommand(HeavyArmor& heavy_armor)
            : heavy_armor_(heavy_armor) {}

        void update() override { heavy_armor_.command_update(); }

        HeavyArmor& heavy_armor_;    
    };
    std::shared_ptr<HeavyArmorCommand> command_component_;

    class ArmBoard final : public librmcs::board::CBoard::Callback {
    public:
        friend class HeavyArmor;
        explicit ArmBoard(
            HeavyArmor& heavy_armor, HeavyArmorCommand& heavy_armor_command,
            std::string_view board_serial = {})
            :tf_(heavy_armor.tf_)
            ,joint(
                {heavy_armor, heavy_armor_command, "/arm/joint_1/motor"}, 
                {heavy_armor, heavy_armor_command, "/arm/joint_2/motor"},
                {heavy_armor, heavy_armor_command, "/arm/joint_3/motor"},
                {heavy_armor, heavy_armor_command, "/arm/joint_4/motor"},
                {heavy_armor, heavy_armor_command, "/arm/joint_5/motor"},
                {heavy_armor, heavy_armor_command, "/arm/joint_6/motor"},
                {heavy_armor, heavy_armor_command, "/arm/joint_7/motor"})
            , gripper(heavy_armor, heavy_armor_command, "/arm/gripper/motor"){
            using namespace device;
            joint[0].configure(
                device::DmMotor::Config{device::DmMotor::Type::kJ8009p}
                .set_encoder_zero_point(static_cast<int>(heavy_armor.get_parameter("joint_1_motor_zero_point").as_int())));
            joint[1].configure(
                device::DmMotor::Config{device::DmMotor::Type::kJ8009p}
                .set_encoder_zero_point(static_cast<int>(heavy_armor.get_parameter("joint_2_motor_zero_point").as_int())));
            joint[2].configure(
                device::DmMotor::Config{device::DmMotor::Type::kJ4340}
                .set_encoder_zero_point(static_cast<int>(heavy_armor.get_parameter("joint_3_motor_zero_point").as_int())));
            joint[3].configure(
                device::DmMotor::Config{device::DmMotor::Type::kJ4340}
                .set_encoder_zero_point(static_cast<int>(heavy_armor.get_parameter("joint_4_motor_zero_point").as_int())));     
            joint[4].configure(
                device::DmMotor::Config{device::DmMotor::Type::kJ8009p}
                .set_encoder_zero_point(static_cast<int>(heavy_armor.get_parameter("joint_5_motor_zero_point").as_int())));
            joint[5].configure(
                device::DmMotor::Config{device::DmMotor::Type::kJ8009p}
                .set_encoder_zero_point(static_cast<int>(heavy_armor.get_parameter("joint_6_motor_zero_point").as_int())));
            joint[6].configure(
                device::DmMotor::Config{device::DmMotor::Type::kJ8009p}
                .set_encoder_zero_point(static_cast<int>(heavy_armor.get_parameter("joint_7_motor_zero_point").as_int())));
            gripper.configure(
                device::DmMotor::Config{device::DmMotor::Type::kJ4340}
                .set_encoder_zero_point(static_cast<int>(heavy_armor.get_parameter("gripper_motor_zero_point").as_int())));
            board_ = std::make_unique<librmcs::board::CBoard>(*this, board_serial);
        }

        ~ArmBoard() final = default;

        void update() {
            joint[0].update_status();
            joint[1].update_status();
            joint[2].update_status();
            joint[3].update_status();
            joint[4].update_status();
            joint[5].update_status();
            joint[6].update_status();
            gripper.update_status();
        }

        void command_update() {
            auto builder = board_->start_transmit();
            if (command_phase_) {
                builder.can_transmit(
                    Spec::kCans.kCan1,
                    {.can_id = 0x01,
                     .can_data = joint[0].generate_torque_command().as_bytes()});
                builder.can_transmit(
                    Spec::kCans.kCan1,
                    {.can_id = 0x02,
                     .can_data = joint[1].generate_torque_command().as_bytes()});
                builder.can_transmit(
                    Spec::kCans.kCan1,
                    {.can_id = 0x03,
                     .can_data = joint[2].generate_torque_command().as_bytes()});
                builder.can_transmit(
                    Spec::kCans.kCan1,
                    {.can_id = 0x04,
                     .can_data = joint[3].generate_torque_command().as_bytes()});
            } else {
                builder.can_transmit(
                    Spec::kCans.kCan1,
                    {.can_id = 0x05,
                     .can_data = joint[4].generate_torque_command().as_bytes()});
                builder.can_transmit(
                    Spec::kCans.kCan1,
                    {.can_id = 0x06,
                     .can_data = joint[5].generate_torque_command().as_bytes()});
                builder.can_transmit(
                    Spec::kCans.kCan1,
                    {.can_id = 0x07,
                     .can_data = joint[6].generate_torque_command().as_bytes()});
                builder.can_transmit(
                    Spec::kCans.kCan1,
                    {.can_id = 0x08,
                     .can_data = gripper.generate_torque_command().as_bytes()});
            }
            command_phase_ = !command_phase_;
        }
    private:
        void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
                return;

            const auto& can_id = data.can_id;

            if(can == Spec::kCans.kCan1){
                if(can_id == 0x211){
                    joint[0].store_status(data.can_data);
                }else if(can_id == 0x212){
                    joint[1].store_status(data.can_data);
                }else if(can_id == 0x213){
                    joint[2].store_status(data.can_data);
                }else if(can_id == 0x214){
                    joint[3].store_status(data.can_data);
                }else if(can_id == 0x215){
                    joint[4].store_status(data.can_data);
                }else if(can_id == 0x216){
                    joint[5].store_status(data.can_data);
                }else if(can_id == 0x217){
                    joint[6].store_status(data.can_data);
                }else if(can_id == 0x218){
                    gripper.store_status(data.can_data);
                }
            }
        }
        OutputInterface<rmcs_description::Tf>& tf_;
        device::DmMotor joint[7];
        device::DmMotor gripper;
        bool command_phase_ = false;
        std::unique_ptr<librmcs::board::CBoard> board_;
    };

    OutputInterface<rmcs_description::Tf> tf_;
    std::unique_ptr<ArmBoard> arm_board_;
};


}


