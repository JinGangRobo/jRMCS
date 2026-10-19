#include <cmath>
#include <cstdint>
#include <numbers>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/chassis_mode.hpp>
#include <rmcs_msgs/game_stage.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/shoot_mode.hpp>

#include "referee/app/ui/shape/shape.hpp"
#include "referee/app/ui/widget/status_bar.hpp"
#include "referee/app/ui/widget/status_ring.hpp"

// Ported from reATRM (v2) referee/app/ui/mini_infantry.cpp.
//
// Differences from the v2 widget set, forced by interfaces that no longer exist
// in the v3 referee stack:
//   * The hurt-direction indicator is dropped: v3 `referee::Status` no longer
//     publishes `/referee/robot/hurt`.
//   * The operation-assist highlight (`/chassis/operate_mode` /
//     rmcs_msgs::OperateMode) is dropped: operation assist is not migrated.
//   * The center-area status readout (`/referee/game/center_area`) is dropped.
//   * The v3 `StatusRing` API is used instead of the v2 one (`set_power_safe` /
//     `update_bullet_speed` / `update_power` are gone).
namespace rmcs_core::referee::app::ui {
using namespace std::chrono_literals;

class MiniInfantry
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    MiniInfantry()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , status_ring_(100, 220, 600, 30, /*supercap=*/true, /*battery=*/false)
        , status_bar_(100, 99)
        , time_reminder_(Shape::Color::PINK, 50, 5, x_center + 150, y_center + 65, 0, false) {

        for (int i = 0; i < 4; ++i) {
            chassis_indicator_[i].set_x(x_center);
            chassis_indicator_[i].set_y(y_center);
            chassis_indicator_[i].set_r(84);
            chassis_indicator_[i].set_width(8);
            chassis_indicator_[i].set_color(Shape::Color::WHITE);
        }

        register_input("/chassis/supercap/enabled", supercap_enabled_);
        register_input("/chassis/supercap/energy_percentage", supercap_energy_percentage_);
        register_input("/chassis/power", chassis_power_);

        register_input("/gimbal/left_friction/control_velocity", first_friction_control_velocity_);
        register_input("/gimbal/left_friction/velocity", first_friction_velocity_);

        register_input("/referee/shooter/bullet_allowance", robot_bullet_allowance_);
        register_input("/referee/shooter/heat_limit", shooter_heat_limit_);
        register_input("/referee/chassis/power_limit", referee_chassis_power_limit_);

        register_input("/chassis/control_mode", chassis_mode_);
        register_input("/chassis/angle", chassis_angle_);
        register_input("/chassis/control_angle", chassis_control_angle_);

        register_input("/gimbal/shooter/mode", shoot_mode_);
        register_input("/gimbal/shooter/heat", shooter_heat_);
        register_input(
            "/gimbal/control_bullet_allowance/limited_by_heat", bullet_allowance_limit_by_heat_);

        register_input("/remote/mouse", mouse_);

        register_input("/referee/game/stage", game_stage_);
    }

    void update() override {
        update_normal_ui();

        set_normal_ui_visible(true);
    }

private:
    void set_normal_ui_visible(bool value) {
        status_ring_.set_visible(value);
        status_bar_.set_visible(value);

        for (int i = 0; i < 4; ++i)
            chassis_indicator_[i].set_visible(value);
    }

    void update_normal_ui() {
        update_chassis_direction_indicator();

        status_ring_.update_friction_wheel_speed(
            *first_friction_velocity_,
            *first_friction_control_velocity_ > 0 && *robot_bullet_allowance_ > 0);
        status_ring_.update_supercap(
            *supercap_energy_percentage_, supercap_enabled_.ready() && *supercap_enabled_);
        status_ring_.update_static_parts(
            {mouse_->right == 1, *shoot_mode_ == rmcs_msgs::ShootMode::PRECISE});

        status_bar_.set_cool_limit(*shooter_heat_limit_);
        status_bar_.update_power_part(*chassis_power_, *referee_chassis_power_limit_, false);
        status_bar_.update_dynamic_part(
            *shooter_heat_limit_ - *shooter_heat_, *robot_bullet_allowance_, 0,
            *bullet_allowance_limit_by_heat_);
    }

    void update_chassis_direction_indicator() {
        auto chassis_mode = *chassis_mode_;

        if (!(chassis_mode_.ready() && chassis_angle_.ready()))
            return;

        auto to_referee_angle = [](double angle) {
            return static_cast<int>(
                std::round((2 * std::numbers::pi - angle) / std::numbers::pi * 180));
        };
        auto unify_angle = [](int angle) constexpr noexcept {
            angle %= 360;
            return (angle < 0) ? (angle + 360) : angle;
        };

        for (int i = 0; i < 4; ++i) {
            switch (chassis_mode) {
            case rmcs_msgs::ChassisMode::SPIN_SLOW:
            case rmcs_msgs::ChassisMode::SPIN_FAST:
                chassis_indicator_[i].set_color(Shape::Color::GREEN);
                break;
            case rmcs_msgs::ChassisMode::STEP_DOWN:
            case rmcs_msgs::ChassisMode::LAUNCH_RAMP:
                chassis_indicator_[i].set_color(i == 0 ? Shape::Color::YELLOW : Shape::Color::CYAN);
                break;
            default:
                chassis_indicator_[i].set_color(
                    i == 0 ? Shape::Color::YELLOW : Shape::Color::WHITE);
                break;
            }

            chassis_indicator_[i].set_angle(
                unify_angle(to_referee_angle(*chassis_angle_) + i * 90), 20);
        }
    }

    static constexpr uint16_t screen_width = 1920, screen_height = 1080;
    static constexpr uint16_t x_center = screen_width / 2, y_center = screen_height / 2;

    InputInterface<rmcs_msgs::ChassisMode> chassis_mode_;
    InputInterface<double> chassis_angle_, chassis_control_angle_;

    InputInterface<bool> supercap_enabled_;
    InputInterface<double> supercap_energy_percentage_;
    InputInterface<double> chassis_power_;

    InputInterface<double> referee_chassis_power_limit_;

    InputInterface<uint16_t> robot_bullet_allowance_;
    InputInterface<int64_t> shooter_heat_;
    InputInterface<int64_t> shooter_heat_limit_;
    InputInterface<int64_t> bullet_allowance_limit_by_heat_;

    InputInterface<double> first_friction_control_velocity_;
    InputInterface<double> first_friction_velocity_;

    InputInterface<rmcs_msgs::Mouse> mouse_;
    InputInterface<rmcs_msgs::GameStage> game_stage_;
    InputInterface<rmcs_msgs::ShootMode> shoot_mode_;

    StatusRing status_ring_;
    StatusBar status_bar_;

    Arc chassis_indicator_[4];

    Integer time_reminder_;
};

} // namespace rmcs_core::referee::app::ui

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::referee::app::ui::MiniInfantry, rmcs_executor::Component)
