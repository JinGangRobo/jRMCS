#pragma once

// 7R 机械臂有界混合 IK。
//
// 基础数据结构、FK 与固定-psi 闭式候选法由 seven_dof_ik.hpp 提供；本文件
// 负责 SEW 流形/腕部命令曲线投影、Hybrid 风险判定、评分与调度逻辑。
// 两个头文件使用同一个 rmcs_core::controller::arm 命名空间，调用接口保持为：
//   solve_hybrid(target, q_current, cfg, result)

#include "seven_dof_ik.hpp"

namespace rmcs_core::controller::arm {
namespace detail {

// 将当前物理肘部方向投影到新目标的肩-腕交圆，再在目标自己的 SEW 图中
// 计算 psi。该连续化只服务于 Hybrid 的腕部命令曲线投影。
inline bool infer_transferred_psi(
    const Pose& target_world, const Vec7& q_current, const Config& cfg, Real& psi) {
    if (!validate_config(cfg)) {
        return false;
    }
    const Robot robot = build_robot(cfg);
    const Pose target = world_target_to_robot(target_world, cfg.installation);
    const Mat3 R07 = mul(target.r, transpose(robot.r_tool_home));
    const Vec3 target_w = sub(target.p, mul(R07, robot.p[7]));

    Vec3 S;
    Vec3 current_e;
    Vec3 current_w;
    arm_points(q_current, robot, S, current_e, current_w);
    (void)current_w;

    const Vec3 p_sw = sub(target_w, S);
    const Real distance_sw = norm(p_sw);
    const Real upper_length = norm(robot.p[3]);
    const Real forearm_length = norm(robot.p[4]);
    if (distance_sw <= cfg.tolerance.geometry
        || distance_sw > upper_length + forearm_length + cfg.tolerance.geometry
        || distance_sw < absr(upper_length - forearm_length) - cfg.tolerance.geometry) {
        return false;
    }

    const Vec3 e_sw = scale(p_sw, static_cast<Real>(1) / distance_sw);
    const Real circle_axial =
        (upper_length * upper_length - forearm_length * forearm_length + distance_sw * distance_sw)
        / (static_cast<Real>(2) * distance_sw);
    Real circle_radius_sq = upper_length * upper_length - circle_axial * circle_axial;
    if (circle_radius_sq < -cfg.tolerance.geometry) {
        return false;
    }
    circle_radius_sq = maxr(circle_radius_sq, static_cast<Real>(0));
    const Real circle_radius = std::sqrt(circle_radius_sq);
    const Vec3 circle_center = add(S, scale(e_sw, circle_axial));
    Vec3 radial = sub(current_e, circle_center);
    radial = sub(radial, scale(e_sw, dot(radial, e_sw)));
    const Real radial_norm = norm(radial);
    if (radial_norm <= cfg.tolerance.geometry || circle_radius <= cfg.tolerance.geometry) {
        return false;
    }
    const Vec3 target_e = add(circle_center, scale(radial, circle_radius / radial_norm));
    return sew_forward(S, target_e, target_w, cfg, psi);
}

} // namespace detail

// 流形投影思路（带可行域恢复的最优解析迭代）逆解入口。
// 正常路径只在 psi_0 及其左右微扰处估计梯度/Hessian；中心点无解时，
// 才分级搜索 +/-5/10/20/40deg 恢复可行域。
inline bool solve_manifold_projection(
    const Pose& target_world, const Vec7& q_current, const Config& cfg, IkResult& final_result) {
    Real psi0 = static_cast<Real>(0);
    if (!detail::infer_current_psi(q_current, cfg, psi0)) {
        const bool ok = solve(target_world, q_current, cfg, final_result);
        final_result.manifold_used_feasibility_recovery = true;
        return ok;
    }

    const Real delta_psi = static_cast<Real>(0.01);
    static constexpr Real kDegToRad = kPi / static_cast<Real>(180);
    static constexpr Real kRecoveryOffsets[] = {
        static_cast<Real>(5) * kDegToRad,  static_cast<Real>(-5) * kDegToRad,
        static_cast<Real>(10) * kDegToRad, static_cast<Real>(-10) * kDegToRad,
        static_cast<Real>(20) * kDegToRad, static_cast<Real>(-20) * kDegToRad,
        static_cast<Real>(40) * kDegToRad, static_cast<Real>(-40) * kDegToRad,
    };
    int evaluation_count = 0;

    auto evaluate = [&](Real psi, IkResult& result) {
        ++evaluation_count;
        return detail::solve_with_resolved_psi(target_world, q_current, psi, true, cfg, result);
    };

    bool have_best = false;
    IkResult best_result;
    Real best_cost = static_cast<Real>(0);
    auto consider = [&](bool ok, const IkResult& candidate) {
        if (!ok) {
            return;
        }
        const Real cost = detail::weighted_command_distance_sq(candidate.q_best, q_current, cfg);
        if (!have_best || cost < best_cost) {
            have_best = true;
            best_cost = cost;
            best_result = candidate;
        }
    };
    auto run_expanded_search = [&]() {
        for (Real const offset : kRecoveryOffsets) {
            IkResult recovered;
            const bool ok_recovered = evaluate(psi0 + offset, recovered);
            consider(ok_recovered, recovered);
        }
    };

    IkResult res_center, res_plus, res_minus;
    const bool ok_center = evaluate(psi0, res_center);
    const bool ok_plus = evaluate(psi0 + delta_psi, res_plus);
    const bool ok_minus = evaluate(psi0 - delta_psi, res_minus);
    consider(ok_center, res_center);
    consider(ok_plus, res_plus);
    consider(ok_minus, res_minus);

    const bool used_recovery = !ok_center;
    Real local_psi = psi0;
    IkResult local_center = res_center;
    bool ok_local_center = ok_center;
    IkResult local_plus = res_plus;
    IkResult local_minus = res_minus;
    bool ok_local_plus = ok_plus;
    bool ok_local_minus = ok_minus;

    if (!ok_center) {
        run_expanded_search();
        if (!have_best) {
            final_result = res_center;
            final_result.manifold_evaluation_count = evaluation_count;
            final_result.manifold_used_feasibility_recovery = true;
            return false;
        }

        local_psi = best_result.psi;
        local_center = best_result;
        ok_local_center = true;
        ok_local_plus = evaluate(local_psi + delta_psi, local_plus);
        ok_local_minus = evaluate(local_psi - delta_psi, local_minus);
        consider(ok_local_plus, local_plus);
        consider(ok_local_minus, local_minus);
    }

    if (ok_local_center && ok_local_plus && ok_local_minus) {
        const Real cost_center =
            detail::weighted_command_distance_sq(local_center.q_best, q_current, cfg);
        const Real cost_plus =
            detail::weighted_command_distance_sq(local_plus.q_best, q_current, cfg);
        const Real cost_minus =
            detail::weighted_command_distance_sq(local_minus.q_best, q_current, cfg);
        const Real gradient = (cost_plus - cost_minus) / (static_cast<Real>(2) * delta_psi);
        const Real hessian =
            (cost_plus - static_cast<Real>(2) * cost_center + cost_minus) / (delta_psi * delta_psi);

        Real step = static_cast<Real>(0);
        const Real hessian_epsilon = static_cast<Real>(1e-5);
        if (hessian > hessian_epsilon) {
            step =
                detail::clamp(-gradient / hessian, static_cast<Real>(-0.1), static_cast<Real>(0.1));
        } else {
            step = detail::clamp(
                -gradient * static_cast<Real>(0.05), static_cast<Real>(-0.05),
                static_cast<Real>(0.05));
        }

        if (detail::absr(step) > static_cast<Real>(1e-5)) {
            IkResult res_optimal;
            const bool ok_optimal = evaluate(local_psi + step, res_optimal);
            consider(ok_optimal, res_optimal);
        }
    }

    if (!have_best) {
        final_result = res_center;
        final_result.manifold_evaluation_count = evaluation_count;
        final_result.manifold_used_feasibility_recovery = used_recovery;
        return false;
    }

    final_result = best_result;
    final_result.manifold_evaluation_count = evaluation_count;
    final_result.manifold_used_feasibility_recovery = used_recovery;
    return true;
}

// 目标特定的电机命令曲线前向割线投影。固定末端位姿后，精确 IK 解
// q(psi) 是一维曲线；这里用中心点和单侧微扰近似切向并执行一次投影。
// 腕部外轴候选仍受 90deg 单步保护，避免用解析等价支换取表面上的成功。
inline bool solve_motor_curve_projection(
    const Pose& target_world, const Vec7& q_current, const Config& cfg, IkResult& final_result) {
    Real psi0 = static_cast<Real>(0);
    if (!detail::infer_transferred_psi(target_world, q_current, cfg, psi0)
        && !detail::infer_current_psi(q_current, cfg, psi0)) {
        const bool ok = solve(target_world, q_current, cfg, final_result);
        final_result.manifold_used_feasibility_recovery = true;
        return ok;
    }

    const Real delta_psi = static_cast<Real>(0.01);
    const Real max_wrist_outer_step = kPi / static_cast<Real>(2);
    int evaluation_count = 0;
    auto evaluate = [&](Real psi, IkResult& result) {
        ++evaluation_count;
        const auto wrist_outer_step_accepted = [&](const Vec7& candidate) {
            return detail::absr(candidate.v[4] - q_current.v[4]) <= max_wrist_outer_step
                && detail::absr(candidate.v[6] - q_current.v[6]) <= max_wrist_outer_step;
        };
        return detail::solve_with_resolved_psi_filtered(
            target_world, q_current, psi, true, cfg, result, wrist_outer_step_accepted);
    };

    bool have_best = false;
    Real best_cost = static_cast<Real>(0);
    IkResult best_result;
    auto consider = [&](bool ok, const IkResult& candidate) {
        if (!ok) {
            return;
        }
        const Real cost = detail::weighted_command_distance_sq(candidate.q_best, q_current, cfg);
        if (!have_best || cost < best_cost) {
            have_best = true;
            best_cost = cost;
            best_result = candidate;
        }
    };

    IkResult center;
    IkResult plus;
    const bool ok_center = evaluate(psi0, center);
    const bool ok_plus = evaluate(psi0 + delta_psi, plus);
    consider(ok_center, center);
    consider(ok_plus, plus);

    if (!ok_center) {
        static constexpr Real kDegToRad = kPi / static_cast<Real>(180);
        IkResult recovered_minus;
        consider(evaluate(psi0 - delta_psi, recovered_minus), recovered_minus);
        if (!have_best) {
            auto search_radius = [&](Real radius_deg) {
                IkResult recovered_plus;
                IkResult recovered_minus_radius;
                consider(evaluate(psi0 + radius_deg * kDegToRad, recovered_plus), recovered_plus);
                consider(
                    evaluate(psi0 - radius_deg * kDegToRad, recovered_minus_radius),
                    recovered_minus_radius);
            };
            for (int radius_deg = 1; radius_deg <= 10 && !have_best; ++radius_deg) {
                search_radius(static_cast<Real>(radius_deg));
            }
            for (int radius_deg = 15; radius_deg <= 180 && !have_best; radius_deg += 5) {
                search_radius(static_cast<Real>(radius_deg));
            }
        }
        final_result = have_best ? best_result : center;
        final_result.manifold_evaluation_count = evaluation_count;
        final_result.manifold_used_feasibility_recovery = true;
        return have_best;
    }

    if (ok_plus) {
        Real numerator = static_cast<Real>(0);
        Real denominator = static_cast<Real>(0);
        for (int i = 0; i < 7; ++i) {
            const Real tangent = (plus.q_best.v[i] - center.q_best.v[i]) / delta_psi;
            const Real residual = center.q_best.v[i] - q_current.v[i];
            const Real weight = cfg.selection.weights[i];
            numerator += weight * residual * tangent;
            denominator += weight * tangent * tangent;
        }

        const Real tangent_epsilon = static_cast<Real>(1e-8);
        if (denominator > tangent_epsilon) {
            const Real step = detail::clamp(
                -numerator / denominator, static_cast<Real>(-0.1), static_cast<Real>(0.1));
            if (detail::absr(step) > static_cast<Real>(1e-5)
                && detail::absr(step - delta_psi) > static_cast<Real>(1e-5)) {
                IkResult trial;
                consider(evaluate(psi0 + step, trial), trial);
            }
        }
    }

    final_result = best_result;
    final_result.manifold_evaluation_count = evaluation_count;
    final_result.manifold_used_feasibility_recovery = false;
    return have_best;
}

// Hybrid 使用的有界启发代价。第一项是真实关节命令变化；其余项只在
// 关节限位或解析奇异附近逐渐生效，用来在两个精确 IK 解之间做选择。
inline Real calculate_hybrid_score(const Vec7& q, const Vec7& q_current, const Config& cfg) {
    Real score = detail::weighted_command_distance_sq(q, q_current, cfg);

    // 限位最外侧 8% 使用软惩罚。它不改变解，只影响候选/流形结果二选一。
    const Real soft_limit_ratio = static_cast<Real>(0.08);
    const Real limit_penalty = static_cast<Real>(0.25);
    for (int i = 0; i < 7; ++i) {
        const Real lo = cfg.joint.lower_rad[i];
        const Real hi = cfg.joint.upper_rad[i];
        const Real span = hi - lo;
        const Real margin = detail::minr(q.v[i] - lo, hi - q.v[i]) / span;
        if (margin < soft_limit_ratio) {
            const Real x = detail::clamp(
                (soft_limit_ratio - margin) / soft_limit_ratio, static_cast<Real>(0),
                static_cast<Real>(1));
            score += cfg.selection.weights[i] * limit_penalty * x * x;
        }
    }

    // 当前轴序列中：q2 是肩部中间角，q4 是肘角，q6 是腕部中间角。
    // 这些廉价指标避免每周期计算完整 Jacobian SVD。
    const Real shoulder_metric = detail::absr(std::cos(q.v[1]));
    const Real elbow_metric = detail::absr(std::sin(q.v[3]));
    const Real wrist_metric = detail::absr(std::sin(q.v[5]));
    auto singular_penalty = [&](Real metric, Real threshold, Real weight) {
        if (metric >= threshold) {
            return static_cast<Real>(0);
        }
        const Real x = (threshold - metric) / threshold;
        return weight * x * x;
    };
    score += singular_penalty(shoulder_metric, static_cast<Real>(0.10), static_cast<Real>(0.10));
    score += singular_penalty(elbow_metric, static_cast<Real>(0.10), static_cast<Real>(0.20));
    score += singular_penalty(wrist_metric, static_cast<Real>(0.15), static_cast<Real>(0.30));

    // 腕部接近奇异时 q5/q7 强耦合；对两侧外角同时大幅变化额外惩罚。
    if (wrist_metric < static_cast<Real>(0.20)) {
        const Real wrist_motion =
            detail::absr(q.v[4] - q_current.v[4]) + detail::absr(q.v[6] - q_current.v[6]);
        const Real free_motion = static_cast<Real>(20) * kPi / static_cast<Real>(180);
        if (wrist_motion > free_motion) {
            const Real excess = wrist_motion - free_motion;
            score += static_cast<Real>(0.50) * excess * excess;
        }
    }
    return score;
}

inline bool hybrid_needs_robust_path(const Vec7& q, const Vec7& q_current, const Config& cfg) {
    const Real large_step = kPi / static_cast<Real>(2); // 任一关节超过 90deg
    for (int i = 0; i < 7; ++i) {
        if (detail::absr(q.v[i] - q_current.v[i]) > large_step) {
            return true;
        }
        const Real lo = cfg.joint.lower_rad[i];
        const Real hi = cfg.joint.upper_rad[i];
        const Real normalized_margin = detail::minr(q.v[i] - lo, hi - q.v[i]) / (hi - lo);
        if (normalized_margin < static_cast<Real>(0.04)) {
            return true;
        }
    }
    return detail::absr(std::cos(q.v[1])) < static_cast<Real>(0.08)
        || detail::absr(std::sin(q.v[3])) < static_cast<Real>(0.08)
        || detail::absr(std::sin(q.v[5])) < static_cast<Real>(0.12);
}

// 有界混合 IK：
//   1. 先运行候选法；中心 psi 可行时通常只需一次闭式候选快解；
//   2. 快解远离限位/奇异且单步不大时立即返回；
//   3. 只有风险路径才运行流形投影，并在两个精确解之间按 Hybrid score 选择。
// 正常路径仅一次固定-psi 闭式调用；风险路径的额外调用数由流形诊断给出。
inline bool solve_hybrid(
    const Pose& target_world, const Vec7& q_current, const Config& cfg, IkResult& final_result) {
    Real psi0 = static_cast<Real>(0);
    if (!detail::infer_current_psi(q_current, cfg, psi0)) {
        const bool ok = solve_manifold_projection(target_world, q_current, cfg, final_result);
        final_result.hybrid_risk_triggered = true;
        final_result.hybrid_stage = ok ? HybridStage::kManifold : HybridStage::kGeometricFallback;
        final_result.hybrid_evaluation_count = final_result.manifold_evaluation_count;
        if (ok) {
            final_result.hybrid_score = calculate_hybrid_score(final_result.q_best, q_current, cfg);
        }
        return ok;
    }

    IkResult fast_result;
    const bool ok_fast = solve(target_world, q_current, cfg, fast_result);
    const bool fast_used_center =
        ok_fast
        && detail::absr(detail::wrap_to_pi(fast_result.psi - psi0)) < static_cast<Real>(1e-5);
    int evaluation_count = fast_used_center ? 1 : 9;
    bool force_wrist_manifold = false;

    // solve() 已经完成 9 点恢复时不再重复执行流形恢复。这样 Hybrid 的
    // 固定-psi 闭式调用硬上限保持为 9；恢复仍失败则让上层保持当前目标。
    if (!ok_fast) {
        final_result = fast_result;
        final_result.hybrid_evaluation_count = evaluation_count;
        final_result.hybrid_risk_triggered = true;
        final_result.hybrid_stage = HybridStage::kGeometricFallback;
        return false;
    }
    if (!fast_used_center) {
        final_result = fast_result;
        final_result.hybrid_evaluation_count = evaluation_count;
        final_result.hybrid_risk_triggered = true;
        final_result.hybrid_stage = HybridStage::kGeometricFallback;
        final_result.hybrid_score = calculate_hybrid_score(final_result.q_best, q_current, cfg);
        return true;
    }

    {
        const Real wrist_metric = detail::absr(std::sin(fast_result.q_best.v[5]));
        Real max_command_step = static_cast<Real>(0);
        for (int i = 0; i < 7; ++i) {
            max_command_step = detail::maxr(
                max_command_step, detail::absr(fast_result.q_best.v[i] - q_current.v[i]));
        }
        const Real wrist_outer_step = detail::absr(fast_result.q_best.v[4] - q_current.v[4])
                                    + detail::absr(fast_result.q_best.v[6] - q_current.v[6]);
        const Real wrist5_margin = detail::minr(
                                       fast_result.q_best.v[4] - cfg.joint.lower_rad[4],
                                       cfg.joint.upper_rad[4] - fast_result.q_best.v[4])
                                 / (cfg.joint.upper_rad[4] - cfg.joint.lower_rad[4]);
        const Real wrist7_margin = detail::minr(
                                       fast_result.q_best.v[6] - cfg.joint.lower_rad[6],
                                       cfg.joint.upper_rad[6] - fast_result.q_best.v[6])
                                 / (cfg.joint.upper_rad[6] - cfg.joint.lower_rad[6]);
        force_wrist_manifold =
            wrist_metric < static_cast<Real>(0.50)
            && (wrist5_margin < static_cast<Real>(0.08) || wrist7_margin < static_cast<Real>(0.08));

        // 精确球腕奇异附近优先保留候选法已有的外角连续化结果。
        // 只有它本身已经发生大翻腕或腕外角靠近限位时才进入流形路径。
        if (!force_wrist_manifold && wrist_metric < static_cast<Real>(0.20)
            && max_command_step < kPi / static_cast<Real>(2)
            && wrist_outer_step < kPi / static_cast<Real>(2)
            && wrist5_margin > static_cast<Real>(0.08) && wrist7_margin > static_cast<Real>(0.08)) {
            final_result = fast_result;
            final_result.hybrid_evaluation_count = evaluation_count;
            final_result.hybrid_risk_triggered = true;
            final_result.hybrid_stage = HybridStage::kFastAnalytic;
            final_result.hybrid_score = calculate_hybrid_score(final_result.q_best, q_current, cfg);
            return true;
        }
    }

    if (!force_wrist_manifold && !hybrid_needs_robust_path(fast_result.q_best, q_current, cfg)) {
        final_result = fast_result;
        final_result.hybrid_evaluation_count = evaluation_count;
        final_result.hybrid_risk_triggered = false;
        final_result.hybrid_stage = HybridStage::kFastAnalytic;
        final_result.hybrid_score = calculate_hybrid_score(final_result.q_best, q_current, cfg);
        return true;
    }

    IkResult manifold_result;
    const bool ok_manifold =
        solve_manifold_projection(target_world, q_current, cfg, manifold_result);
    evaluation_count += manifold_result.manifold_evaluation_count;

    const bool have_any = ok_fast || ok_manifold;
    if (!have_any) {
        final_result = manifold_result;
        final_result.hybrid_evaluation_count = evaluation_count;
        final_result.hybrid_risk_triggered = true;
        final_result.hybrid_stage = HybridStage::kGeometricFallback;
        return false;
    }

    const Real fast_score =
        ok_fast ? calculate_hybrid_score(fast_result.q_best, q_current, cfg) : static_cast<Real>(0);
    const Real manifold_score = ok_manifold
                                  ? calculate_hybrid_score(manifold_result.q_best, q_current, cfg)
                                  : static_cast<Real>(0);
    final_result = ok_fast ? fast_result : manifold_result;
    Real selected_score = ok_fast ? fast_score : manifold_score;
    HybridStage selected_stage = ok_fast ? HybridStage::kFastAnalytic : HybridStage::kManifold;
    if (ok_manifold && (force_wrist_manifold || !ok_fast || manifold_score < selected_score)) {
        final_result = manifold_result;
        selected_score = manifold_score;
        selected_stage = HybridStage::kManifold;
    }
    final_result.hybrid_evaluation_count = evaluation_count;
    final_result.hybrid_risk_triggered = true;
    final_result.hybrid_stage = selected_stage;
    final_result.hybrid_score = selected_score;
    return true;
}

} // namespace rmcs_core::controller::arm
