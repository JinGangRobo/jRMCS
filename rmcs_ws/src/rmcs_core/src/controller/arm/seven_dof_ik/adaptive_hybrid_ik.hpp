#pragma once

// AdaptiveHybrid：按实际末端到目标的笛卡尔变化量，在 DLS8 与原 Hybrid
// 之间做互斥选择。互斥是时间预算的关键：一个周期只运行一个求解器，
// 不采用“DLS 失败后同周期再跑 Hybrid”的串行回退。

#include "dls_ik.hpp"
#include "hybrid_ik.hpp"

namespace rmcs_core::controller::arm {
namespace detail {

inline bool validate_adaptive_config(const Config& cfg) {
    return validate_config(cfg)
        && cfg.solver.adaptive_dls_position_threshold_mm >= static_cast<Real>(0)
        && cfg.solver.adaptive_dls_orientation_threshold_rad >= static_cast<Real>(0);
}

} // namespace detail

// ArmSolver 主入口。current_world 由 ArmSolver 每周期已算出的 FK 位姿提供，
inline bool solve_adaptive_hybrid(
    const Pose& target_world, const Pose& current_world, const Vec7& q_current, const Config& cfg,
    IkResult& result) {
    if (!detail::validate_adaptive_config(cfg)) {
        result = IkResult{};
        result.q_best = q_current;
        return false;
    }

    // 两个位姿先统一转换到机器人基座系，避免安装旋转/平移影响阈值含义。
    const Pose current = detail::world_target_to_robot(current_world, cfg.installation);
    const Pose target = detail::world_target_to_robot(target_world, cfg.installation);
    const Real position_delta = detail::norm(detail::sub(target.p, current.p));
    const Real orientation_delta = detail::rotation_distance(current.r, target.r);

    // 必须同时满足平移和旋转阈值才属于局部变化。任一方向较大都交给
    // 具有全局解析/流形恢复能力的 Hybrid，避免 DLS8 超出 20deg 信赖域。
    const bool use_dls = position_delta <= cfg.solver.adaptive_dls_position_threshold_mm
                      && orientation_delta <= cfg.solver.adaptive_dls_orientation_threshold_rad;
    // 上层保持上一目标，从而维持 max(T_DLS8, T_Hybrid) 的结构预算。
    const bool ok = use_dls ? solve_dls(target_world, q_current, cfg, result)
                            : solve_hybrid(target_world, q_current, cfg, result);

    // 无论成功失败都保留路由依据，便于日志和上位机定位误路由。
    result.adaptive_route = use_dls ? static_cast<uint8_t>(1) : static_cast<uint8_t>(2);
    result.adaptive_position_delta_mm = position_delta;
    result.adaptive_orientation_delta_rad = orientation_delta;
    return ok;
}

inline bool solve_adaptive_hybrid(
    const Pose& target_world, const Vec7& q_current, const Config& cfg, IkResult& result) {
    if (!detail::validate_adaptive_config(cfg)) {
        result = IkResult{};
        result.q_best = q_current;
        return false;
    }

    const detail::Robot robot = detail::build_robot(cfg);
    const Pose current = detail::forward_kinematics(q_current, robot);

    // forward_kinematics() 返回机器人基座系位姿；转换到世界系后复用主入口，
    // 确保两个重载的阈值计算与安装配置完全一致。
    Pose current_world{};
    current_world.r = detail::mul(cfg.installation.rotation_world_from_robot, current.r);
    current_world.p = detail::add(
        detail::mul(cfg.installation.rotation_world_from_robot, current.p),
        cfg.installation.position_world_mm);
    return solve_adaptive_hybrid(target_world, current_world, q_current, cfg, result);
}

} // namespace rmcs_core::controller::arm
