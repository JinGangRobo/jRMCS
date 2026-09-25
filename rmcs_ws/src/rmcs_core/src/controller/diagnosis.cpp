#include <algorithm>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::diagnosis {

enum class DiagnosisMsg : uint8_t { // larger value means higher priority
    ALL_READY = 0,
    CTR_MOTOR_OFFLINE = 1,
    CTR_SUPERCAP_OFFLINE = 2,
};

class Diagnosis
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    Diagnosis()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {

        get_parameter("motors", motors_interface_name_);
        get_parameter("enable_motor_check", enable_motor_check_);
        get_parameter("enable_supercap_check", enable_supercap_check_);

        for (const auto& motor : motors_interface_name_) {
            auto motor_alive_input = std::make_unique<InputInterface<bool>>();
            register_input(motor + "/alive", *motor_alive_input);
            motor_alive_inputs_.push_back(std::move(motor_alive_input));
        }
        register_input("/chassis/supercap/enabled", supercap_alive_input_, false);

        register_output(
            "/diagnosis/status", status_output_, static_cast<uint8_t>(DiagnosisMsg::ALL_READY));
    }

    void update() override {
        errors_.clear();

        // Check motor status
        if (enable_motor_check_)
            for (auto& motor_alive_input : motor_alive_inputs_) {
                if (!motor_alive_input->ready() || !**motor_alive_input) {
                    errors_.push_back(DiagnosisMsg::CTR_MOTOR_OFFLINE);
                }
            }

        // Check supercap status
        if (enable_supercap_check_)
            if (!supercap_alive_input_.ready() || !*supercap_alive_input_) {
                errors_.push_back(DiagnosisMsg::CTR_SUPERCAP_OFFLINE);
            }

        const auto highest =
            std::max_element(errors_.begin(), errors_.end(), [](DiagnosisMsg a, DiagnosisMsg b) {
                return static_cast<uint8_t>(a) < static_cast<uint8_t>(b);
            });
        *status_output_ = errors_.empty() ? static_cast<uint8_t>(DiagnosisMsg::ALL_READY)
                                          : static_cast<uint8_t>(*highest);
    }

private:
    std::vector<std::string> motors_interface_name_ = {};
    bool enable_motor_check_ = false;
    bool enable_supercap_check_ = false;

    std::vector<std::unique_ptr<InputInterface<bool>>> motor_alive_inputs_;
    InputInterface<bool> supercap_alive_input_;

    std::vector<DiagnosisMsg> errors_ = {};

    OutputInterface<uint8_t> status_output_;
};

} // namespace rmcs_core::controller::diagnosis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::diagnosis::Diagnosis, rmcs_executor::Component)
