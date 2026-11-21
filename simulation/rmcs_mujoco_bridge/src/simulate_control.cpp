#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <optional>

#include <mujoco/mujoco.h>

#include "simulate_link.hpp"

namespace rmcs_mujoco_bridge {
namespace {

// 控制滑条对应零增益 actuator。只有 RMCS 回传的力矩会驱动模型。
void Control(const mjModel* model, mjData* data) {
    using Clock = std::chrono::steady_clock;
    constexpr int kAxes = 7;
    if (model->nq != kAxes + 2 || model->nv != kAxes + 2 || model->nu != kAxes + 1
        || mj_name2id(model, mjOBJ_ACTUATOR, "rmcs_J1") != 0
        || mj_name2id(model, mjOBJ_ACTUATOR, "rmcs_gripper") != kAxes)
        return;

    static std::optional<SimulateLink> link;
    static bool initialized = false;
    if (!initialized) {
        initialized = true;
        try {
            link.emplace(kSimulatePort, kRmcsPort);
        } catch (const std::exception& e) {
            std::fprintf(stderr, "jRMCS: %s\n", e.what());
        }
    }
    if (!link)
        return;

    static Clock::time_point last_command{};
    static SimulateTorque torque{};
    if (link->ReceiveLatest(torque))
        last_command = Clock::now();
    if (Clock::now() - last_command > std::chrono::milliseconds(100))
        torque = {};

    SimulateState state{};
    state.time = data->time;
    if (!(model->opt.disableflags & mjDSBL_GRAVITY))
        std::copy_n(model->opt.gravity, state.gravity.size(), state.gravity.begin());
    for (int i = 0; i < kAxes; ++i) {
        data->qfrc_applied[i] = std::isfinite(torque.arm[i]) ? torque.arm[i] : 0.0;
        state.position[i] = data->qpos[i];
        state.velocity[i] = data->qvel[i];
        state.torque[i] = data->qfrc_applied[i];
        state.target[i] = data->ctrl[i];
    }

    const int slide = model->actuator_trnid[2 * kAxes];
    const int position = model->jnt_qposadr[slide];
    const int velocity = model->jnt_dofadr[slide];
    const double transmission = model->actuator_gear[6 * kAxes];
    const double open = model->qpos0[position];
    const double closed = model->jnt_range[2 * slide];
    state.gripper_torque = std::isfinite(torque.gripper) ? torque.gripper : 0.0;
    // theta = transmission * (q - q0), so virtual work gives F = transmission * torque.
    data->qfrc_applied[velocity] = transmission * state.gripper_torque;
    state.gripper_angle = transmission * (data->qpos[position] - open);
    state.gripper_velocity = transmission * data->qvel[velocity];
    state.gripper_target = transmission * (closed - open) * std::clamp(data->ctrl[kAxes], 0.0, 1.0);
    link->Send(state);
}

} // namespace
} // namespace rmcs_mujoco_bridge

mjPLUGIN_LIB_INIT(rmcs_simulate_control) { mjcb_control = rmcs_mujoco_bridge::Control; }
