#pragma once

// 7R 机械臂固定预算电机命令空间 DLS。
//
// 与 SEW 流形法不同，本解算器从当前真实反馈对应的 q_current 直接做局部
// 位姿投影，不先把冗余自由度降维成臂角，因此不会因为 SEW 坐标图换支而
// 主动选择远端腕解析支。实现约束：无动态内存、解析几何 Jacobian、固定
// 6x6 带主元线性求解，适合在控制周期内实时求解。

#include "seven_dof_ik.hpp"

namespace rmcs_core::controller::arm {
namespace detail {

inline Vec3 matrix_column(Mat3 const& matrix, int column) {
    return vec(matrix.m[0][column], matrix.m[1][column], matrix.m[2][column]);
}

inline Vec3 dls_orientation_error(Mat3 const& actual, Mat3 const& target) {
    Vec3 error = vec(0, 0, 0);
    for (int column = 0; column < 3; ++column) {
        error = add(error, cross(matrix_column(actual, column), matrix_column(target, column)));
    }
    return scale(error, static_cast<Real>(0.5));
}

// 同时计算 FK 和世界/机器人基坐标系下的 6x7 几何 Jacobian。
// 平移行使用 m，旋转行乘 orientation_scale_m，从而与误差向量量纲一致。
inline Pose dls_forward_and_jacobian(
    Vec7 const& q, Robot const& robot, Real orientation_scale_m, Real jacobian[6][7]) {
    Mat3 rotation = eye();
    Vec3 position = robot.p[0];
    Vec3 axes[7]{};
    Vec3 origins[7]{};
    for (int joint = 0; joint < 7; ++joint) {
        axes[joint] = mul(rotation, robot.h[joint]);
        origins[joint] = position;
        rotation = mul(rotation, rot(robot.h[joint], q.v[joint]));
        position = add(position, mul(rotation, robot.p[joint + 1]));
    }

    for (int joint = 0; joint < 7; ++joint) {
        const Vec3 linear_mm = cross(axes[joint], sub(position, origins[joint]));
        jacobian[0][joint] = linear_mm.x * static_cast<Real>(1.0e-3);
        jacobian[1][joint] = linear_mm.y * static_cast<Real>(1.0e-3);
        jacobian[2][joint] = linear_mm.z * static_cast<Real>(1.0e-3);
        jacobian[3][joint] = axes[joint].x * orientation_scale_m;
        jacobian[4][joint] = axes[joint].y * orientation_scale_m;
        jacobian[5][joint] = axes[joint].z * orientation_scale_m;
    }

    Pose pose{};
    pose.r = mul(rotation, robot.r_tool_home);
    pose.p = position;
    return pose;
}

inline bool solve_linear6(Real matrix[6][6], Real rhs[6], Real solution[6]) {
    const Real pivot_tolerance = static_cast<Real>(1.0e-12);
    for (int column = 0; column < 6; ++column) {
        int pivot_row = column;
        Real pivot_abs = absr(matrix[column][column]);
        for (int row = column + 1; row < 6; ++row) {
            const Real candidate_abs = absr(matrix[row][column]);
            if (candidate_abs > pivot_abs) {
                pivot_abs = candidate_abs;
                pivot_row = row;
            }
        }
        if (pivot_abs <= pivot_tolerance) {
            return false;
        }
        if (pivot_row != column) {
            for (int k = column; k < 6; ++k) {
                const Real temporary = matrix[column][k];
                matrix[column][k] = matrix[pivot_row][k];
                matrix[pivot_row][k] = temporary;
            }
            const Real temporary = rhs[column];
            rhs[column] = rhs[pivot_row];
            rhs[pivot_row] = temporary;
        }

        const Real pivot = matrix[column][column];
        for (int row = column + 1; row < 6; ++row) {
            const Real factor = matrix[row][column] / pivot;
            matrix[row][column] = static_cast<Real>(0);
            for (int k = column + 1; k < 6; ++k) {
                matrix[row][k] -= factor * matrix[column][k];
            }
            rhs[row] -= factor * rhs[column];
        }
    }

    for (int row = 5; row >= 0; --row) {
        Real value = rhs[row];
        for (int column = row + 1; column < 6; ++column) {
            value -= matrix[row][column] * solution[column];
        }
        solution[row] = value / matrix[row][row];
    }
    return true;
}

inline bool validate_dls_joint_weights(const Real weights[7]) {
    for (int joint = 0; joint < 7; ++joint) {
        if (!std::isfinite(weights[joint]) || weights[joint] <= static_cast<Real>(0)) {
            return false;
        }
    }
    return true;
}

inline bool validate_dls_shoulder_constraint(const SolverConfig& solver) {
    return std::isfinite(solver.dls_shoulder_deadband_rad)
        && solver.dls_shoulder_deadband_rad > static_cast<Real>(0)
        && solver.dls_shoulder_deadband_rad < kPi / static_cast<Real>(2)
        && std::isfinite(solver.dls_shoulder_outer_max_delta_rad)
        && solver.dls_shoulder_outer_max_delta_rad > static_cast<Real>(0)
        && std::isfinite(solver.dls_shoulder_relaxed_position_mm)
        && solver.dls_shoulder_relaxed_position_mm > static_cast<Real>(0)
        && std::isfinite(solver.dls_shoulder_relaxed_orientation_rad)
        && solver.dls_shoulder_relaxed_orientation_rad > static_cast<Real>(0);
}

// 球肩轴序列 Z/-Y/-X 在 q2=pi/2+k*pi 处第一、第三轴共线。
// 这里直接对物理关节角判断，不使用全局构型分支符号。
inline bool is_in_shoulder_singularity_deadband(Real q2, Real half_width) {
    const Real half_pi = kPi / static_cast<Real>(2);
    const Real center = half_pi + std::round((q2 - half_pi) / kPi) * kPi;
    return absr(q2 - center) <= half_width;
}

// 检查真实关节命令线段是否触碰任一周期肩奇异死区。不能只检查端点，
// 否则未来把单步上限调得大于死区宽度时可能一步跨过整个死区。
inline bool
    shoulder_step_touches_singularity_deadband(Real q2_begin, Real q2_end, Real half_width) {
    const Real half_pi = kPi / static_cast<Real>(2);
    const Real lower = minr(q2_begin, q2_end);
    const Real upper = maxr(q2_begin, q2_end);
    const Real first_index = std::ceil((lower - half_width - half_pi) / kPi);
    const Real first_center = half_pi + first_index * kPi;
    return first_center - half_width <= upper;
}

inline Real
    dls_task_metric(Real position_error_mm, Real orientation_error_rad, Real orientation_scale_m) {
    const Real position_error_m = position_error_mm * static_cast<Real>(1.0e-3);
    const Real scaled_orientation = orientation_error_rad * orientation_scale_m;
    return position_error_m * position_error_m + scaled_orientation * scaled_orientation;
}

// 求解加权 DLS 的单次未裁剪关节步长：
//
//   min ||J*dq-error||^2 + lambda^2 * dq^T*Wq*dq
//
//   dq = Wq^-1*J^T*(J*Wq^-1*J^T + lambda^2*I)^-1*error
//
// Wq 是正对角矩阵。权重越大，对应关节在冗余方向上的运动越小。
inline bool solve_weighted_dls_step(
    const Real jacobian[6][7], const Real error[6], Real damping_sq, const Real weights[7],
    Real joint_step[7], const bool enabled_joints[7] = nullptr) {
    if (!std::isfinite(damping_sq) || damping_sq <= static_cast<Real>(0)
        || !validate_dls_joint_weights(weights)) {
        return false;
    }

    Real inverse_weights[7]{};
    for (int joint = 0; joint < 7; ++joint) {
        if (enabled_joints == nullptr || enabled_joints[joint]) {
            inverse_weights[joint] = static_cast<Real>(1) / weights[joint];
        }
    }

    Real normal[6][6]{};
    for (int row = 0; row < 6; ++row) {
        for (int column = 0; column < 6; ++column) {
            Real value = static_cast<Real>(0);
            for (int joint = 0; joint < 7; ++joint) {
                value += jacobian[row][joint] * inverse_weights[joint] * jacobian[column][joint];
            }
            normal[row][column] = value;
        }
        normal[row][row] += damping_sq;
    }

    Real rhs[6]{};
    for (int row = 0; row < 6; ++row) {
        rhs[row] = error[row];
    }
    Real dual_step[6]{};
    if (!solve_linear6(normal, rhs, dual_step)) {
        return false;
    }

    for (int joint = 0; joint < 7; ++joint) {
        Real value = static_cast<Real>(0);
        for (int row = 0; row < 6; ++row) {
            value += jacobian[row][joint] * dual_step[row];
        }
        joint_step[joint] = inverse_weights[joint] * value;
    }
    return true;
}

} // namespace detail

inline bool solve_dls(
    const Pose& target_world, const Vec7& q_current, const Config& cfg, IkResult& result) {
    result = IkResult{};
    result.q_best = q_current;
    if (!detail::validate_config(cfg) || cfg.solver.dls_max_iterations < 1
        || cfg.solver.dls_damping <= static_cast<Real>(0)
        || cfg.solver.dls_max_iteration_step_rad <= static_cast<Real>(0)
        || cfg.solver.dls_trust_region_rad <= static_cast<Real>(0)
        || cfg.solver.dls_orientation_scale_mm <= static_cast<Real>(0)
        || !std::isfinite(cfg.solver.dls_max_target_position_delta_mm)
        || cfg.solver.dls_max_target_position_delta_mm <= static_cast<Real>(0)
        || !detail::validate_dls_joint_weights(cfg.solver.dls_joint_weights)
        || !detail::validate_dls_shoulder_constraint(cfg.solver)) {
        return false;
    }

    const detail::Robot robot = detail::build_robot(cfg);
    const Pose target = detail::world_target_to_robot(target_world, cfg.installation);
    const Pose current = detail::forward_kinematics(q_current, robot);
    const Real initial_position_error_mm = detail::norm(detail::sub(target.p, current.p));
    const Real initial_orientation_error_rad = detail::rotation_distance(current.r, target.r);
    result.best_position_error_mm = initial_position_error_mm;
    result.best_orientation_error_rad = initial_orientation_error_rad;
    if (!std::isfinite(initial_position_error_mm) || !std::isfinite(initial_orientation_error_rad)
        || initial_position_error_mm > cfg.solver.dls_max_target_position_delta_mm) {
        // 输入拒绝发生在第一次迭代之前。保留真实反馈关节角，同时把
        // 初始残差写入既有诊断字段，供上层区分超距/非有限输入。
        return false;
    }

    const Real orientation_scale_m =
        cfg.solver.dls_orientation_scale_mm * static_cast<Real>(1.0e-3);
    const Real damping_sq = cfg.solver.dls_damping * cfg.solver.dls_damping;
    const Real stop_position_mm =
        detail::minr(cfg.tolerance.position_mm, static_cast<Real>(1.0e-2));
    const Real stop_orientation_rad =
        detail::minr(cfg.tolerance.orientation_rad, static_cast<Real>(5.0e-5));
    Vec7 q = q_current;
    Vec7 trust_lower{};
    Vec7 trust_upper{};
    for (int joint = 0; joint < 7; ++joint) {
        trust_lower.v[joint] = detail::maxr(
            cfg.joint.lower_rad[joint], q_current.v[joint] - cfg.solver.dls_trust_region_rad);
        trust_upper.v[joint] = detail::minr(
            cfg.joint.upper_rad[joint], q_current.v[joint] + cfg.solver.dls_trust_region_rad);
    }

    const Real shoulder_outer_lower[2] = {
        detail::maxr(
            trust_lower.v[0], q_current.v[0] - cfg.solver.dls_shoulder_outer_max_delta_rad),
        detail::maxr(
            trust_lower.v[2], q_current.v[2] - cfg.solver.dls_shoulder_outer_max_delta_rad),
    };
    const Real shoulder_outer_upper[2] = {
        detail::minr(
            trust_upper.v[0], q_current.v[0] + cfg.solver.dls_shoulder_outer_max_delta_rad),
        detail::minr(
            trust_upper.v[2], q_current.v[2] + cfg.solver.dls_shoulder_outer_max_delta_rad),
    };
    bool shoulder_zone_touched = detail::is_in_shoulder_singularity_deadband(
        q_current.v[1], cfg.solver.dls_shoulder_deadband_rad);

    Vec7 best_shoulder_constrained_q = q_current;
    Real initial_task_metric = static_cast<Real>(0);
    Real best_shoulder_constrained_position_error_mm = static_cast<Real>(0);
    Real best_shoulder_constrained_orientation_error_rad = static_cast<Real>(0);
    Real best_shoulder_constrained_metric = static_cast<Real>(0);
    bool best_shoulder_constrained_valid = false;

    bool trust_region_hit = false;
    for (int iteration = 0; iteration < cfg.solver.dls_max_iterations; ++iteration) {
        result.dls_iteration_count = iteration + 1;
        Real jacobian[6][7]{};
        const Pose actual =
            detail::dls_forward_and_jacobian(q, robot, orientation_scale_m, jacobian);
        const Vec3 position_error_mm = detail::sub(target.p, actual.p);
        const Vec3 orientation_error = detail::dls_orientation_error(actual.r, target.r);
        const Real position_error_norm_mm = detail::norm(position_error_mm);

        if (iteration == 0 || shoulder_zone_touched) {
            const Real orientation_error_rad = detail::rotation_distance(actual.r, target.r);
            const Real metric = detail::dls_task_metric(
                position_error_norm_mm, orientation_error_rad, orientation_scale_m);
            if (iteration == 0) {
                initial_task_metric = metric;
            }
            if (shoulder_zone_touched
                && (!best_shoulder_constrained_valid
                    || metric < best_shoulder_constrained_metric)) {
                best_shoulder_constrained_q = q;
                best_shoulder_constrained_position_error_mm = position_error_norm_mm;
                best_shoulder_constrained_orientation_error_rad = orientation_error_rad;
                best_shoulder_constrained_metric = metric;
                best_shoulder_constrained_valid = true;
            }
        }

        // 提前停止也要满足本次求解的最终验收容差。
        if (position_error_norm_mm <= stop_position_mm
            && detail::rotation_distance(actual.r, target.r) <= stop_orientation_rad) {
            break;
        }

        Real error[6] = {
            position_error_mm.x * static_cast<Real>(1.0e-3),
            position_error_mm.y * static_cast<Real>(1.0e-3),
            position_error_mm.z * static_cast<Real>(1.0e-3),
            orientation_error.x * orientation_scale_m,
            orientation_error.y * orientation_scale_m,
            orientation_error.z * orientation_scale_m,
        };
        Real joint_steps[7]{};
        if (!detail::solve_weighted_dls_step(
                jacobian, error, damping_sq, cfg.solver.dls_joint_weights, joint_steps)) {
            result.dls_trust_region_hit = trust_region_hit;
            return false;
        }

        const Real q2_step = detail::clamp(
            joint_steps[1], -cfg.solver.dls_max_iteration_step_rad,
            cfg.solver.dls_max_iteration_step_rad);
        const Real q2_candidate =
            detail::clamp(q.v[1] + q2_step, trust_lower.v[1], trust_upper.v[1]);
        shoulder_zone_touched = shoulder_zone_touched
                             || detail::shoulder_step_touches_singularity_deadband(
                                    q.v[1], q2_candidate, cfg.solver.dls_shoulder_deadband_rad);

        // q1/q3 达到死区专用总变化边界且步长仍向外时，把对应列从本次
        // 有效 Jacobian 中移除并重算，使其他可用关节接管剩余任务误差。
        if (shoulder_zone_touched) {
            bool enabled_joints[7] = {
                true, true, true, true, true, true, true,
            };
            bool must_resolve = false;
            const int outer_joints[2] = {0, 2};
            for (int outer = 0; outer < 2; ++outer) {
                const int joint = outer_joints[outer];
                const Real boundary_tolerance = static_cast<Real>(1.0e-7);
                const bool pushing_below_lower =
                    q.v[joint] <= shoulder_outer_lower[outer] + boundary_tolerance
                    && joint_steps[joint] < static_cast<Real>(0);
                const bool pushing_above_upper =
                    q.v[joint] >= shoulder_outer_upper[outer] - boundary_tolerance
                    && joint_steps[joint] > static_cast<Real>(0);
                if (pushing_below_lower || pushing_above_upper) {
                    enabled_joints[joint] = false;
                    must_resolve = true;
                }
            }
            if (must_resolve) {
                trust_region_hit = true;
                if (!detail::solve_weighted_dls_step(
                        jacobian, error, damping_sq, cfg.solver.dls_joint_weights, joint_steps,
                        enabled_joints)) {
                    result.dls_trust_region_hit = trust_region_hit;
                    return false;
                }
            }
        }

        for (int joint = 0; joint < 7; ++joint) {
            const Real joint_step = detail::clamp(
                joint_steps[joint], -cfg.solver.dls_max_iteration_step_rad,
                cfg.solver.dls_max_iteration_step_rad);
            const Real unconstrained = q.v[joint] + joint_step;
            Real constrained =
                detail::clamp(unconstrained, trust_lower.v[joint], trust_upper.v[joint]);
            if (shoulder_zone_touched && joint == 0) {
                constrained =
                    detail::clamp(constrained, shoulder_outer_lower[0], shoulder_outer_upper[0]);
            } else if (shoulder_zone_touched && joint == 2) {
                constrained =
                    detail::clamp(constrained, shoulder_outer_lower[1], shoulder_outer_upper[1]);
            }
            trust_region_hit =
                trust_region_hit
                || detail::absr(constrained - unconstrained) > static_cast<Real>(1.0e-7);
            q.v[joint] = constrained;
        }
    }

    const Pose final_pose = detail::forward_kinematics(q, robot);
    const Real position_error_mm = detail::norm(detail::sub(final_pose.p, target.p));
    const Real orientation_error_rad = detail::rotation_distance(final_pose.r, target.r);

    if (shoulder_zone_touched) {
        const Real final_metric =
            detail::dls_task_metric(position_error_mm, orientation_error_rad, orientation_scale_m);
        if (!best_shoulder_constrained_valid || final_metric < best_shoulder_constrained_metric) {
            best_shoulder_constrained_q = q;
            best_shoulder_constrained_position_error_mm = position_error_mm;
            best_shoulder_constrained_orientation_error_rad = orientation_error_rad;
            best_shoulder_constrained_metric = final_metric;
            best_shoulder_constrained_valid = true;
        }
    }

    result.dls_trust_region_hit = trust_region_hit;
    result.best_position_error_mm = position_error_mm;
    result.best_orientation_error_rad = orientation_error_rad;

    const bool exact_solution = position_error_mm <= cfg.tolerance.position_mm
                             && orientation_error_rad <= cfg.tolerance.orientation_rad;
    const bool relaxed_shoulder_solution = !exact_solution && shoulder_zone_touched
                                        && best_shoulder_constrained_valid
                                        && best_shoulder_constrained_metric < initial_task_metric
                                        && best_shoulder_constrained_position_error_mm
                                               <= cfg.solver.dls_shoulder_relaxed_position_mm
                                        && best_shoulder_constrained_orientation_error_rad
                                               <= cfg.solver.dls_shoulder_relaxed_orientation_rad;
    if (!exact_solution && !relaxed_shoulder_solution) {
        // 未收敛中间量不作为电机目标输出。
        result.q_best = q_current;
        return false;
    }

    result.success = true;
    if (relaxed_shoulder_solution) {
        result.q_best = best_shoulder_constrained_q;
        result.best_position_error_mm = best_shoulder_constrained_position_error_mm;
        result.best_orientation_error_rad = best_shoulder_constrained_orientation_error_rad;
    } else {
        result.q_best = q;
    }
    result.best_cost = detail::weighted_command_distance_sq(result.q_best, q_current, cfg);
    result.valid_candidate_count = 1;
    result.selected_valid_index = 0;
    return true;
}

} // namespace rmcs_core::controller::arm
