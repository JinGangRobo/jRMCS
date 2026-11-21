#pragma once

#include <algorithm>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>

#include <mujoco/mujoco.h>

namespace rmcs_mujoco_bridge {

inline constexpr std::size_t kJointCount = 7;
inline constexpr double kTargetMargin = 0.05; // rad, between slider endpoints and joint limits.

using ModelPtr = std::unique_ptr<mjModel, decltype(&mj_deleteModel)>;
using DataPtr = std::unique_ptr<mjData, decltype(&mj_deleteData)>;

inline ModelPtr LoadModel(const std::string& description) {
    char error[1024]{};
    const std::unique_ptr<mjSpec, decltype(&mj_deleteSpec)> spec{
        mj_parseXMLString(description.c_str(), nullptr, error, sizeof(error)), mj_deleteSpec};
    if (!spec)
        throw std::runtime_error(std::string{"MuJoCo failed to parse JGarm: "} + error);

    for (int i = 8; i <= 9; ++i) {
        const auto number = std::to_string(i);
        const auto joint_name = "joint" + number;
        auto* element = mjs_findElement(spec.get(), mjOBJ_JOINT, joint_name.c_str());
        auto* joint = element ? mjs_asJoint(element) : nullptr;
        auto* body = mjs_findBody(spec.get(), ("Link" + number).c_str());
        if (!joint || !body || joint->type != mjJNT_SLIDE)
            throw std::runtime_error("Expected a gripper slider: " + joint_name);

        // Gripper_open uses joint8's lower limit and joint9's upper limit.
        // Shift the body reference with qpos0 so the URDF kinematics stay equivalent.
        const double open = joint->range[i - 8];
        mjtNum travel[3], shift[3];
        mju_scl3(travel, joint->axis, open - joint->ref);
        mju_rotVecQuat(shift, travel, body->quat);
        mju_addTo3(body->pos, shift);
        joint->ref = open;

        auto* hold = mjs_addEquality(spec.get(), nullptr);
        hold->type = mjEQ_JOINT;
        hold->objtype = mjOBJ_JOINT;
        mjs_setString(hold->name1, joint_name.c_str());
        std::fill_n(hold->data, 5, 0.0); // Hold the slide at qpos0 relative to its reference.
        hold->solref[0] = 0.005;
    }

    ModelPtr model{mj_compile(spec.get(), nullptr), mj_deleteModel};
    if (!model)
        throw std::runtime_error(
            std::string{"MuJoCo failed to compile JGarm: "} + mjs_getError(spec.get()));
    if (model->nq != kJointCount + 2 || model->nv != kJointCount + 2)
        throw std::runtime_error("Expected seven arm joints and two gripper slides");
    return model;
}

inline DataPtr MakeData(const mjModel& model) {
    DataPtr data{mj_makeData(&model), mj_deleteData};
    if (!data)
        throw std::runtime_error("MuJoCo could not allocate simulation data");
    return data;
}

} // namespace rmcs_mujoco_bridge
