#pragma once

// 坐标与单位约定：
//   - target_world.r 是目标工具坐标系相对世界坐标系的旋转矩阵；
//   - target_world.p 是目标工具坐标系原点在世界坐标系下的位置，单位 mm；
//   - q_current、psi、输出 q_best 均为弧度；
//   - cfg.installation 描述机器人基坐标系相对世界坐标系的安装位姿；
//   - cfg.geometry.tool_axis_rotation 与上游基线保持一致，默认将原工具 z 轴映射为命令 x 轴。
//

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numbers>

#ifndef SEVEN_DOF_IK_SCALAR
# define SEVEN_DOF_IK_SCALAR float
#endif

namespace rmcs_core::controller::arm {

using Real = SEVEN_DOF_IK_SCALAR;

static constexpr Real kPi = std::numbers::pi_v<float>;
static constexpr Real kTwoPi = static_cast<Real>(2) * kPi;
// 闭式解候选数量的固定上限。正常 3R-R-3R 构型远小于这个值；
// 保留 96 是为了覆盖球肩/球腕奇异处分裂出的附加候选，同时避免动态分配。
static constexpr int kMaxCandidates = 96;

// 三维向量。为便于调试，没有使用数组封装 x/y/z。
struct Vec3 {
    Real x;
    Real y;
    Real z;
};

// 七个关节角，单位 rad。v[0] 对应 q1，v[6] 对应 q7。
struct Vec7 {
    Real v[7];
};

// 3x3 旋转矩阵，按 m[row][col] 存储。
struct Mat3 {
    Real m[3][3];
};

// 末端位姿：r 是旋转矩阵，p 是位置，位置单位 mm。
struct Pose {
    Mat3 r;
    Vec3 p;
};

enum class SewMode : uint8_t {
    kStereographic = 0,
    kConventional = 1,
};

enum class HybridStage : uint8_t {
    kNone = 0,
    kFastAnalytic = 1,
    kManifold = 2,
    kGeometricFallback = 3,
};

struct Geometry {
    // 基座到肩部的 z 向偏置，对应上游基线的 d_L0。
    Real base_offset_mm = static_cast<Real>(0.0);

    // 肩到肘的主臂长，对应 d_L1_1 + d_L1_2。
    Real upper_arm_mm = static_cast<Real>(158.50);

    // 肘部偏置，对应 d_L1_3。不要把它简单合并成一根等效斜边，
    // 因为它相对 q4 轴的方向会影响闭式逆解。
    Real elbow_offset_mm = static_cast<Real>(60.0);

    // 肘到腕的前臂长度，对应 d_L2_1。
    Real forearm_mm = static_cast<Real>(249.38);

    // 法兰额外延长量 d_f。
    Real flange_extension_mm = static_cast<Real>(0.0);

    // 末端工具长度 d_ee。
    Real end_effector_mm = static_cast<Real>(92.57);

    // 工具绕本地 z 轴的固定偏航角 theta_ee。
    Real tool_yaw_rad = static_cast<Real>(0.0);

    // 工具坐标系重映射矩阵。默认值与上游基线一致：
    //   新 x = 原 z，新 y = -原 y，新 z = 原 x。
    Mat3 tool_axis_rotation;
};

struct Installation {
    // 机器人基坐标系到世界坐标系的旋转 R_WR。
    // 世界目标会先通过 R_WR^T 转回机器人本地坐标系再求逆解。
    Mat3 rotation_world_from_robot;

    // 机器人基坐标系原点在世界坐标系下的位置 p_WR，单位 mm。
    Vec3 position_world_mm;
};

struct JointConfig {
    // 每个关节的命令角限位，单位 rad。注意这里是命令角，
    // 与上游基线的 MDH 关节 2 固定 pi/2 偏置无关。
    Real lower_rad[7];
    Real upper_rad[7];
};

struct SelectionConfig {
    // 候选解选择权重。权重越大，该关节越不愿意离开当前角度。
    // q 已由 fit_to_joint_limits() 映射为限位内的实际命令角，因此目标函数为：
    //   J = sum_i weights[i] * (q[i] - q_current[i])^2
    // 这里不能 wrap_to_pi，否则会把跨越 +/-pi 的大幅电机动作误判为短路径。
    Real weights[7];
};

struct SewConfig {
    // 默认使用 stereographic SEW，避开传统 SEW 的双向奇异线。
    SewMode mode = SewMode::kStereographic;

    // stereographic SEW 的奇异方向，默认 -Z。
    Vec3 stereographic_singularity_direction;

    // stereographic SEW 的参考方向，默认 +Y。
    Vec3 stereographic_reference;

    // conventional SEW 的参考方向，只有 mode=kConventional 时使用。
    Vec3 conventional_reference;
};

struct Tolerance {
    // FK 校验用的位置误差阈值，单位 mm。float 版本不宜设得过小。
    Real position_mm = static_cast<Real>(1.0e-3);

    // FK 校验用的姿态误差阈值，单位 rad。
    Real orientation_rad = static_cast<Real>(1.0e-4);

    // 旋转矩阵正交性检查预留阈值。
    Real rotation_matrix = static_cast<Real>(1.0e-4);

    // 几何退化判断阈值，例如肩腕距离接近 0。
    Real geometry = static_cast<Real>(1.0e-6);

    // IK-Geo 子问题求解阈值。
    Real subproblem = static_cast<Real>(1.0e-6);

    // 球肩/球腕欧拉分解奇异判断阈值。
    Real spherical_singularity = static_cast<Real>(1.0e-5);

    // 候选解去重阈值。
    Real duplicate_joint = static_cast<Real>(1.0e-5);

    // 关节限位判断阈值。
    Real joint_limit = static_cast<Real>(1.0e-6);
};

struct SolverConfig {
    // 子问题不可精确满足时会产生最近二乘解。实际位置控制通常应拒绝；
    // 调试或容错时可置 true，但必须结合 FK 误差判断使用。
    bool accept_least_squares = false;

    // 电机/关节命令空间加权 DLS。dls_joint_weights 是关节增量二次型
    // dq^T*Wq*dq 的对角元素；值越大，DLS 越不愿意使用对应关节。
    // 当前重点抑制球肩 q1-q3，尤其是中间轴 q2。
    int dls_max_iterations = 32;
    Real dls_damping = static_cast<Real>(5.0e-3);
    Real dls_max_iteration_step_rad = static_cast<Real>(10) * kPi / static_cast<Real>(180);
    Real dls_trust_region_rad = static_cast<Real>(30) * kPi / static_cast<Real>(180);
    // 将旋转误差/Jw 缩放成等效平移长度。100mm 表示 1rad 姿态误差
    // 在 DLS 任务度量中等效为 100mm 平移误差。
    Real dls_orientation_scale_mm = static_cast<Real>(100);

    // 纯 DLS 只处理相对 q_current 正运动学位置不超过该距离的局部目标。
    // 超过上限时直接拒绝，不截断目标、不插值，也不切换其他 IK。
    Real dls_max_target_position_delta_mm = static_cast<Real>(100);
    Real dls_joint_weights[7] = {
        static_cast<Real>(7), static_cast<Real>(6), static_cast<Real>(5), static_cast<Real>(1),
        static_cast<Real>(2), static_cast<Real>(2), static_cast<Real>(2),
    };

    // 球肩 Z/-Y/-X 轴序列在 q2=pi/2+k*pi 时退化。DLS 允许 q2 连续
    // 穿越奇异点，但只要本次迭代路径触碰奇异死区，就把 q1/q3 相对
    // q_current 的总变化限制在 dls_shoulder_outer_max_delta_rad 内。
    Real dls_shoulder_deadband_rad = static_cast<Real>(3) * kPi / static_cast<Real>(180);
    Real dls_shoulder_outer_max_delta_rad = static_cast<Real>(0.2) * kPi / static_cast<Real>(180);

    // 只有本次 DLS 确实触碰肩奇异死区时才允许使用这组放宽容差。
    // 未触碰死区的普通 DLS 仍严格使用 cfg.tolerance。
    Real dls_shoulder_relaxed_position_mm = static_cast<Real>(25);
    Real dls_shoulder_relaxed_orientation_rad =
        static_cast<Real>(12) * kPi / static_cast<Real>(180);

    // AdaptiveHybrid 的互斥路由阈值。只有平移和旋转变化同时不超过阈值时
    // 才调用加权 DLS，否则只调用原 Hybrid；同一周期不会先后运行两个求解器。
    // 90mm / 30deg 是当前互斥路由基线；修改后必须重新统计路由和腕翻转。
    Real adaptive_dls_position_threshold_mm = static_cast<Real>(90);
    Real adaptive_dls_orientation_threshold_rad =
        static_cast<Real>(30) * kPi / static_cast<Real>(180);
};

// 由 MDH 表换算出的固定 POE 模型。设置 enabled 后，build_robot() 会直接使用
// 这组 h/p/r_tool_home，而不再从上面的 legacy Geometry 常量反推。这样可以把
// 现场校验过的 MDH 作为唯一几何基线，同时保持原闭式/DLS/Hybrid 算法不变。
struct RobotModel {
    bool enabled = false;
    Vec3 h[7];
    Vec3 p[8];
    Mat3 r_tool_home;
};

struct Config {
    // 所有可调参数集中在这里。实际上板时通常只需要改：
    //   geometry.*        机械尺寸
    //   installation.*    安装位姿
    //   joint.*           关节限位
    //   selection.weights 候选解偏好
    Geometry geometry;
    Installation installation;
    JointConfig joint;
    SelectionConfig selection;
    SewConfig sew;
    Tolerance tolerance;
    SolverConfig solver;
    RobotModel robot;
};

struct IkResult {
    // true 表示找到满足限位和 FK 误差阈值的精确候选解。
    bool success = false;

    // 闭式几何阶段生成的原始候选数量。
    int raw_candidate_count = 0;

    // 通过限位、最小二乘过滤、FK 误差过滤后的有效候选数量。
    int valid_candidate_count = 0;

    // 被选中的有效候选索引。失败时为 -1。
    int selected_valid_index = -1;

    // 本次求解使用的 SEW 角。
    Real psi = static_cast<Real>(0);

    // true 表示 psi 是从 q_current 自动推断的。
    bool psi_was_inferred_from_current = false;

    // 最终选中的 7 轴角度，单位 rad。
    Vec7 q_best = {{0, 0, 0, 0, 0, 0, 0}};

    // 最优候选的选择代价。
    Real best_cost = static_cast<Real>(0);

    // 最优候选的 FK 位置/姿态误差。
    Real best_position_error_mm = static_cast<Real>(0);
    Real best_orientation_error_rad = static_cast<Real>(0);

    // 流形法内部调用闭式固定-psi 求解器的次数；普通 solve() 保持为 0。
    int manifold_evaluation_count = 0;

    // true 表示中心 psi 无解，流形法启用了分级可行域恢复搜索。
    bool manifold_used_feasibility_recovery = false;

    // Hybrid 诊断：估算的固定-psi 闭式调用数、是否触发风险路径、
    // 最终采用的阶段和启发式风险代价。普通 solve()/流形入口保持默认值。
    int hybrid_evaluation_count = 0;
    bool hybrid_risk_triggered = false;
    HybridStage hybrid_stage = HybridStage::kNone;
    Real hybrid_score = static_cast<Real>(0);

    // DLS 诊断。失败时 q_best 保持 q_current，调用方不会收到未收敛的中间量。
    int dls_iteration_count = 0;
    bool dls_trust_region_hit = false;

    // 固定-psi 解析器最终选中的主分支编码，用于诊断解析支切换：
    // bit0=q4 距离分支，bit1=肩部 q1/q2 分支，bit2=腕部 q5/q6 分支。
    // -1 表示求解失败、未经过解析分支，或调用方没有请求该诊断。
    int selected_major_branch = -1;

    // AdaptiveHybrid 路由诊断：0=非自适应入口，1=DLS8，2=Hybrid。
    // delta 记录路由判定时使用的笛卡尔变化量，便于上板统计和重新标定阈值。
    uint8_t adaptive_route = 0;
    Real adaptive_position_delta_mm = static_cast<Real>(0);
    Real adaptive_orientation_delta_rad = static_cast<Real>(0);
};

namespace detail {

struct Robot {
    Vec3 h[7];
    Vec3 p[8];
    Mat3 r_tool_home;
};

struct Candidate {
    Vec7 q;
    bool least_squares;
    // 生成该候选时经过的 q4/肩/腕三层主分支，编码规则同
    // IkResult::selected_major_branch，仅用于诊断最终选择了哪条解析支。
    uint8_t major_branch;
};

struct CandidateSet {
    Candidate c[kMaxCandidates];
    int n = 0;
};

struct AngleSet {
    Real a[2];
    int n = 0;
    bool least_squares = false;
};

struct Sp2Result {
    Real theta1[2];
    Real theta2[2];
    int n = 0;
    bool least_squares = false;
};

inline Real absr(Real x) { return x < static_cast<Real>(0) ? -x : x; }
inline Real maxr(Real a, Real b) { return a > b ? a : b; }
inline Real minr(Real a, Real b) { return a < b ? a : b; }
inline Real clamp(Real x, Real lo, Real hi) { return minr(hi, maxr(lo, x)); }

inline Vec3 vec(Real x, Real y, Real z) { return Vec3{x, y, z}; }
inline Vec3 add(Vec3 a, Vec3 b) { return vec(a.x + b.x, a.y + b.y, a.z + b.z); }
inline Vec3 sub(Vec3 a, Vec3 b) { return vec(a.x - b.x, a.y - b.y, a.z - b.z); }
inline Vec3 scale(Vec3 a, Real s) { return vec(a.x * s, a.y * s, a.z * s); }
inline Real dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 cross(Vec3 a, Vec3 b) {
    return vec(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}
inline Real norm2(Vec3 a) { return dot(a, a); }
inline Real norm(Vec3 a) { return std::sqrt(norm2(a)); }

inline Vec3 unit(Vec3 a, Real tolerance, bool* ok = nullptr) {
    const Real n = norm(a);
    const bool valid = n > tolerance;
    if (ok) {
        *ok = valid;
    }
    if (!valid) {
        return vec(0, 0, 0);
    }
    return scale(a, static_cast<Real>(1) / n);
}

inline Mat3 eye() { return Mat3{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}}; }

inline Mat3
    mat(Real a00, Real a01, Real a02, Real a10, Real a11, Real a12, Real a20, Real a21, Real a22) {
    return Mat3{{{a00, a01, a02}, {a10, a11, a12}, {a20, a21, a22}}};
}

inline Mat3 transpose(Mat3 A) {
    return mat(
        A.m[0][0], A.m[1][0], A.m[2][0], A.m[0][1], A.m[1][1], A.m[2][1], A.m[0][2], A.m[1][2],
        A.m[2][2]);
}

inline Vec3 mul(Mat3 A, Vec3 v) {
    return vec(
        A.m[0][0] * v.x + A.m[0][1] * v.y + A.m[0][2] * v.z,
        A.m[1][0] * v.x + A.m[1][1] * v.y + A.m[1][2] * v.z,
        A.m[2][0] * v.x + A.m[2][1] * v.y + A.m[2][2] * v.z);
}

inline Mat3 mul(Mat3 A, Mat3 B) {
    Mat3 C = {};
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            C.m[r][c] = A.m[r][0] * B.m[0][c] + A.m[r][1] * B.m[1][c] + A.m[r][2] * B.m[2][c];
        }
    }
    return C;
}

inline Mat3 rot(Vec3 axis, Real theta) {
    bool ok = false;
    const Vec3 k = unit(axis, static_cast<Real>(1.0e-12), &ok);
    if (!ok) {
        return eye();
    }
    const Real s = std::sin(theta);
    const Real c = std::cos(theta);
    const Real v = static_cast<Real>(1) - c;

    return mat(
        c + k.x * k.x * v, k.x * k.y * v - k.z * s, k.x * k.z * v + k.y * s,
        k.y * k.x * v + k.z * s, c + k.y * k.y * v, k.y * k.z * v - k.x * s,
        k.z * k.x * v - k.y * s, k.z * k.y * v + k.x * s, c + k.z * k.z * v);
}

inline Real wrap_to_pi(Real a) {
    a = std::fmod(a + kPi, kTwoPi);
    if (a < static_cast<Real>(0)) {
        a += kTwoPi;
    }
    return a - kPi;
}

inline Vec7 wrap_to_pi(Vec7 q) {
    for (int i = 0; i < 7; ++i) {
        q.v[i] = wrap_to_pi(q.v[i]);
    }
    return q;
}

inline Real rotation_distance(Mat3 R1, Mat3 R2) {
    const Mat3 Rt = mul(transpose(R1), R2);
    Real c = (Rt.m[0][0] + Rt.m[1][1] + Rt.m[2][2] - static_cast<Real>(1)) / static_cast<Real>(2);
    c = clamp(c, static_cast<Real>(-1), static_cast<Real>(1));
    return std::acos(c);
}

inline bool add_candidate(CandidateSet& set, Vec7 q, bool least_squares, uint8_t major_branch = 0) {
    if (set.n >= kMaxCandidates) {
        return false;
    }
    set.c[set.n].q = wrap_to_pi(q);
    set.c[set.n].least_squares = least_squares;
    set.c[set.n].major_branch = major_branch;
    ++set.n;
    return true;
}

inline Config make_default_config() {
    Config cfg;
    cfg.geometry.tool_axis_rotation = mat(0, 0, 1, 0, -1, 0, 1, 0, 0);
    cfg.installation.rotation_world_from_robot = eye();
    cfg.installation.position_world_mm = vec(0, 0, 0);

    for (int i = 0; i < 7; ++i) {
        cfg.joint.lower_rad[i] = -kPi;
        cfg.joint.upper_rad[i] = kPi;
        cfg.selection.weights[i] = static_cast<Real>(1);
    }

    cfg.sew.mode = SewMode::kStereographic;
    cfg.sew.stereographic_singularity_direction = vec(0, 0, -1);
    cfg.sew.stereographic_reference = vec(0, 1, 0);
    cfg.sew.conventional_reference = vec(1, 0, 0);
    return cfg;
}

inline Robot build_robot(const Config& cfg) {
    if (cfg.robot.enabled) {
        Robot robot;
        for (int i = 0; i < 7; ++i) {
            robot.h[i] = cfg.robot.h[i];
            robot.p[i] = cfg.robot.p[i];
        }
        robot.p[7] = cfg.robot.p[7];
        robot.r_tool_home = cfg.robot.r_tool_home;
        return robot;
    }

    const Vec3 ex = vec(1, 0, 0);
    const Vec3 ey = vec(0, 1, 0);
    const Vec3 ez = vec(0, 0, 1);

    Robot robot;
    robot.h[0] = ez;
    robot.h[1] = scale(ey, -1);
    robot.h[2] = scale(ex, -1);
    robot.h[3] = scale(ey, -1);
    robot.h[4] = ex;
    robot.h[5] = scale(ey, -1);
    robot.h[6] = ex;

    robot.p[0] = vec(0, 0, cfg.geometry.base_offset_mm);
    robot.p[1] = vec(0, 0, 0);
    robot.p[2] = vec(0, 0, 0);
    robot.p[3] = vec(-cfg.geometry.upper_arm_mm, 0, 0);
    robot.p[4] = vec(cfg.geometry.forearm_mm, 0, cfg.geometry.elbow_offset_mm);
    robot.p[5] = vec(0, 0, 0);
    robot.p[6] = vec(0, 0, 0);
    robot.p[7] = vec(cfg.geometry.flange_extension_mm + cfg.geometry.end_effector_mm, 0, 0);

    const Mat3 r_flange_home = mat(0, 0, 1, 0, -1, 0, 1, 0, 0);
    robot.r_tool_home = mul(
        mul(r_flange_home, rot(ez, cfg.geometry.tool_yaw_rad)), cfg.geometry.tool_axis_rotation);
    return robot;
}

inline Pose world_target_to_robot(const Pose& world_target, const Installation& installation) {
    const Mat3 Rt = transpose(installation.rotation_world_from_robot);
    Pose robot_target;
    robot_target.r = mul(Rt, world_target.r);
    robot_target.p = mul(Rt, sub(world_target.p, installation.position_world_mm));
    return robot_target;
}

inline Pose forward_kinematics(const Vec7& q, const Robot& robot) {
    Mat3 R = eye();
    Vec3 p = robot.p[0];
    for (int i = 0; i < 7; ++i) {
        R = mul(R, rot(robot.h[i], q.v[i]));
        p = add(p, mul(R, robot.p[i + 1]));
    }
    Pose T;
    T.r = mul(R, robot.r_tool_home);
    T.p = p;
    return T;
}

inline void arm_points(const Vec7& q, const Robot& robot, Vec3& S, Vec3& E, Vec3& W) {
    S = robot.p[0];
    const Mat3 R03 =
        mul(mul(rot(robot.h[0], q.v[0]), rot(robot.h[1], q.v[1])), rot(robot.h[2], q.v[2]));
    E = add(S, mul(R03, robot.p[3]));
    W = add(E, mul(mul(R03, rot(robot.h[3], q.v[3])), robot.p[4]));
}

inline AngleSet sp1(Vec3 p1, Vec3 p2, Vec3 k, Real tolerance) {
    // 子问题 1：
    //   已知旋转轴 k 和两个向量 p1/p2，求 theta 使 R(k,theta)*p1 = p2。
    // 实现方式是把 p1/p2 投影到垂直于 k 的平面，在该平面内用 atan2 求圆周角。
    AngleSet out;
    bool ok = false;
    k = unit(k, tolerance, &ok);
    if (!ok) {
        out.n = 0;
        out.least_squares = true;
        return out;
    }

    const Vec3 p1_parallel = scale(k, dot(k, p1));
    const Vec3 p2_parallel = scale(k, dot(k, p2));
    const Vec3 p1_perp = sub(p1, p1_parallel);
    const Vec3 p2_perp = sub(p2, p2_parallel);

    const Real perp_norm_product = norm(p1_perp) * norm(p2_perp);
    if (perp_norm_product <= tolerance) {
        out.a[0] = 0;
        out.n = 1;
        out.least_squares = norm(sub(p1, p2)) > tolerance;
        return out;
    }

    const Real theta = std::atan2(dot(k, cross(p1_perp, p2_perp)), dot(p1_perp, p2_perp));
    const Vec3 residual = sub(mul(rot(k, theta), p1), p2);
    const Real scale_val = maxr(static_cast<Real>(1), maxr(norm(p1), norm(p2)));
    out.a[0] = theta;
    out.n = 1;
    out.least_squares = norm(residual) > tolerance * scale_val;
    return out;
}

inline AngleSet sp4(Vec3 h, Vec3 p, Vec3 k, Real d, Real tolerance) {
    // 子问题 4：
    //   求 theta，使 h^T * R(k,theta) * p = d。
    // 展开后是 A*cos(theta) + B*sin(theta) = C，因此最多两个解。
    AngleSet out;
    bool ok = false;
    k = unit(k, tolerance, &ok);
    if (!ok) {
        out.n = 0;
        out.least_squares = true;
        return out;
    }

    const Vec3 u = sub(p, scale(k, dot(k, p)));
    const Real A = dot(h, u);
    const Real B = dot(h, cross(k, p));
    const Real C = d - dot(h, k) * dot(k, p);
    const Real radius = std::hypot(A, B);

    if (radius <= tolerance) {
        out.a[0] = 0;
        out.n = 1;
        out.least_squares = absr(C) > tolerance;
        return out;
    }

    const Real phase = std::atan2(B, A);
    Real normalized_c = C / radius;
    if (normalized_c > static_cast<Real>(1) + tolerance) {
        out.a[0] = phase;
        out.n = 1;
        out.least_squares = true;
    } else if (normalized_c < static_cast<Real>(-1) - tolerance) {
        out.a[0] = wrap_to_pi(phase + kPi);
        out.n = 1;
        out.least_squares = true;
    } else {
        normalized_c = clamp(normalized_c, static_cast<Real>(-1), static_cast<Real>(1));
        const Real opening = std::acos(normalized_c);
        if (opening <= tolerance) {
            out.a[0] = phase;
            out.n = 1;
        } else {
            out.a[0] = wrap_to_pi(phase + opening);
            out.a[1] = wrap_to_pi(phase - opening);
            out.n = 2;
        }
        out.least_squares = false;
    }
    return out;
}

inline Sp2Result sp2(Vec3 p1, Vec3 p2, Vec3 k1, Vec3 k2, Real tolerance) {
    // 子问题 2：
    //   求 theta1/theta2，使 R(k1,theta1)*p1 = R(k2,theta2)*p2。
    // 这里复用两个 SP4 先分别求 theta1 和 theta2，再按 IK-Geo 的配对规则组合。
    Sp2Result out;
    const Real p1_norm = norm(p1);
    const Real p2_norm = norm(p2);
    if (p1_norm <= tolerance || p2_norm <= tolerance) {
        out.least_squares = true;
        return out;
    }

    const Vec3 p1_unit = scale(p1, static_cast<Real>(1) / p1_norm);
    const Vec3 p2_unit = scale(p2, static_cast<Real>(1) / p2_norm);
    bool ok1 = false;
    bool ok2 = false;
    k1 = unit(k1, tolerance, &ok1);
    k2 = unit(k2, tolerance, &ok2);
    if (!ok1 || !ok2) {
        out.least_squares = true;
        return out;
    }

    const AngleSet theta1_all = sp4(k2, p1_unit, k1, dot(k2, p2_unit), tolerance);
    const AngleSet theta2_all = sp4(k1, p2_unit, k2, dot(k1, p1_unit), tolerance);
    if (theta1_all.n == 0 || theta2_all.n == 0) {
        out.least_squares = true;
        return out;
    }

    if (theta1_all.n > 1 || theta2_all.n > 1) {
        out.n = 2;
        out.theta1[0] = theta1_all.a[0];
        out.theta1[1] = theta1_all.a[theta1_all.n - 1];
        out.theta2[0] = theta2_all.a[theta2_all.n - 1];
        out.theta2[1] = theta2_all.a[0];
    } else {
        out.n = 1;
        out.theta1[0] = theta1_all.a[0];
        out.theta2[0] = theta2_all.a[0];
    }

    const Real scale_val = maxr(static_cast<Real>(1), maxr(p1_norm, p2_norm));
    out.least_squares = absr(p1_norm - p2_norm) > tolerance * scale_val || theta1_all.least_squares
                     || theta2_all.least_squares;

    for (int i = 0; i < out.n; ++i) {
        const Vec3 residual =
            sub(mul(rot(k1, out.theta1[i]), p1_unit), mul(rot(k2, out.theta2[i]), p2_unit));
        if (norm(residual) > static_cast<Real>(20) * tolerance) {
            out.least_squares = true;
        }
    }
    return out;
}

inline AngleSet sp3(Vec3 p1, Vec3 p2, Vec3 k, Real d, Real tolerance) {
    // 子问题 3：
    //   求 theta，使 ||R(k,theta)*p1 - p2|| = d。
    // 对当前机械臂，它用于根据肩腕距离求肘关节 q4。
    const Real projection = static_cast<Real>(0.5) * (dot(p1, p1) + dot(p2, p2) - d * d);
    return sp4(p2, p1, k, projection, tolerance);
}

inline bool sew_inverse(Vec3 S, Vec3 W, Real psi, const Config& cfg, Vec3& e_ce, Vec3& n_sew) {
    // 根据肩点 S、腕点 W 和给定 SEW 角 psi，构造目标肘平面：
    //   e_sw：肩到腕方向；
    //   n_sew：肩-肘-腕平面法向；
    //   e_ce：在该平面中决定肘部处于哪一个半平面。
    const Vec3 p_sw = sub(W, S);
    const Real p_sw_norm = norm(p_sw);
    if (p_sw_norm <= cfg.tolerance.geometry) {
        return false;
    }
    const Vec3 e_sw = scale(p_sw, static_cast<Real>(1) / p_sw_norm);

    if (cfg.sew.mode == SewMode::kStereographic) {
        bool ok_a = false;
        bool ok_b = false;
        const Vec3 singular_direction =
            unit(cfg.sew.stereographic_singularity_direction, cfg.tolerance.geometry, &ok_a);
        const Vec3 reference = unit(cfg.sew.stereographic_reference, cfg.tolerance.geometry, &ok_b);
        if (!ok_a || !ok_b) {
            return false;
        }
        const Vec3 reference_axis = cross(sub(e_sw, singular_direction), reference);
        const Vec3 x_axis = cross(reference_axis, p_sw);
        const Real x_norm = norm(x_axis);
        if (x_norm <= cfg.tolerance.geometry) {
            return false;
        }
        e_ce = mul(rot(e_sw, psi), scale(x_axis, static_cast<Real>(1) / x_norm));
        n_sew = cross(e_sw, e_ce);
    } else {
        bool ok = false;
        const Vec3 reference = unit(cfg.sew.conventional_reference, cfg.tolerance.geometry, &ok);
        if (!ok) {
            return false;
        }
        Vec3 y_axis = cross(e_sw, reference);
        const Real y_norm = norm(y_axis);
        if (y_norm <= cfg.tolerance.geometry) {
            return false;
        }
        y_axis = scale(y_axis, static_cast<Real>(1) / y_norm);
        n_sew = mul(rot(e_sw, psi), y_axis);
        e_ce = cross(n_sew, e_sw);
    }

    e_ce = scale(e_ce, static_cast<Real>(1) / norm(e_ce));
    n_sew = scale(n_sew, static_cast<Real>(1) / norm(n_sew));
    return true;
}

inline bool sew_forward(Vec3 S, Vec3 E, Vec3 W, const Config& cfg, Real& psi) {
    // 从当前构型的 S/E/W 三点反算 SEW 角。
    // 当用户不显式传入 psi 时，solve() 会调用它，从而尽量保持当前肘部姿态。
    const Vec3 p_se = sub(E, S);
    const Vec3 p_sw = sub(W, S);
    if (norm(p_sw) <= cfg.tolerance.geometry || norm(cross(p_sw, p_se)) <= cfg.tolerance.geometry) {
        return false;
    }

    const Vec3 e_sw = scale(p_sw, static_cast<Real>(1) / norm(p_sw));
    Vec3 n_sew = cross(p_sw, p_se);
    n_sew = scale(n_sew, static_cast<Real>(1) / norm(n_sew));

    if (cfg.sew.mode == SewMode::kStereographic) {
        bool ok_a = false;
        bool ok_b = false;
        const Vec3 singular_direction =
            unit(cfg.sew.stereographic_singularity_direction, cfg.tolerance.geometry, &ok_a);
        const Vec3 reference = unit(cfg.sew.stereographic_reference, cfg.tolerance.geometry, &ok_b);
        if (!ok_a || !ok_b) {
            return false;
        }
        Vec3 n_reference = cross(sub(e_sw, singular_direction), reference);
        const Real n_ref_norm = norm(n_reference);
        if (n_ref_norm <= cfg.tolerance.geometry) {
            return false;
        }
        n_reference = scale(n_reference, static_cast<Real>(1) / n_ref_norm);
        psi = std::atan2(dot(n_sew, cross(e_sw, n_reference)), dot(n_sew, n_reference));
    } else {
        bool ok = false;
        const Vec3 reference = unit(cfg.sew.conventional_reference, cfg.tolerance.geometry, &ok);
        if (!ok) {
            return false;
        }
        const Vec3 reference_perp = sub(reference, scale(e_sw, dot(e_sw, reference)));
        const Vec3 elbow_perp = sub(p_se, scale(e_sw, dot(e_sw, p_se)));
        if (norm(reference_perp) <= cfg.tolerance.geometry
            || norm(elbow_perp) <= cfg.tolerance.geometry) {
            return false;
        }
        psi = std::atan2(
            dot(e_sw, cross(reference_perp, elbow_perp)), dot(reference_perp, elbow_perp));
    }
    psi = wrap_to_pi(psi);
    return true;
}

inline int singular_outer_angle_candidates(
    Real q_first, Real q_middle, Real q_last, Vec3 h_first, Vec3 h_middle, Vec3 h_last,
    Real q_first_current, Real q_last_current, Real weight_first, Real weight_last, Real tolerance,
    Real out[2]) {
    // 球肩/球腕在中间轴角度接近奇异时，外侧两个关节不是唯一的。
    // 此时只有 q_first +/- q_last 的组合可观测。这里返回：
    //   1. 原始代表解；
    //   2. 按 weights 和 q_current 分配后的最近解。
    const Vec3 transformed_last_axis = mul(rot(h_middle, q_middle), h_last);
    const Real alignment = dot(h_first, transformed_last_axis);
    if (absr(absr(alignment) - static_cast<Real>(1)) > tolerance) {
        out[0] = q_first;
        return 1;
    }

    Real sign = alignment >= static_cast<Real>(0) ? static_cast<Real>(1) : static_cast<Real>(-1);
    const Real observable_angle = q_first + sign * q_last;
    const Real current_observable_angle = q_first_current + sign * q_last_current;
    // 这里处理的是球关节奇异时唯一可观测的周期组合角，并非最终关节命令
    // 距离；候选落到限位内之后仍由 weighted_command_distance_sq() 线性评分。
    const Real observable_delta = wrap_to_pi(observable_angle - current_observable_angle);

    const Real total_weight = weight_first + weight_last;
    Real q_first_optimal = q_first_current;
    if (total_weight > tolerance) {
        q_first_optimal = q_first_current + weight_last / total_weight * observable_delta;
    }

    out[0] = wrap_to_pi(q_first);
    out[1] = wrap_to_pi(q_first_optimal);
    return 2;
}

inline void ik3rr3r(
    Mat3 R07, Vec3 p0_target, Real psi, const Vec7& q_current, const Robot& robot,
    const Config& cfg, CandidateSet& Q) {
    // 3R-R-3R 闭式逆解主流程，与上游基线 local_ik3rr3r 对应：
    //   1. 去掉工具长度，得到球腕中心 W；
    //   2. 用 SP3 根据肩腕距离求 q4；
    //   3. 用 SEW 平面构造肩部目标姿态 R03；
    //   4. 分解球肩得到 q1-q3；
    //   5. 根据 R47 分解球腕得到 q5-q7。
    const Vec3 W = sub(p0_target, mul(R07, robot.p[7]));
    const Vec3 S = robot.p[0];
    const Vec3 p_sw = sub(W, S);
    const Real shoulder_wrist_distance = norm(p_sw);
    if (shoulder_wrist_distance <= cfg.tolerance.geometry) {
        return;
    }

    const Vec3 e_sw = scale(p_sw, static_cast<Real>(1) / shoulder_wrist_distance);
    Vec3 e_ce;
    Vec3 n_sew;
    if (!sew_inverse(S, W, psi, cfg, e_ce, n_sew)) {
        return;
    }

    const AngleSet q4_set =
        sp3(robot.p[4], scale(robot.p[3], -1), robot.h[3], shoulder_wrist_distance,
            cfg.tolerance.subproblem);
    for (int q4_index = 0; q4_index < q4_set.n; ++q4_index) {
        const Real q4 = q4_set.a[q4_index];
        const Vec3 elbow_vector = add(robot.p[3], mul(rot(robot.h[3], q4), robot.p[4]));

        const Sp2Result bc =
            sp2(p_sw, elbow_vector, scale(n_sew, -1), e_sw, cfg.tolerance.subproblem);
        if (bc.n == 0) {
            continue;
        }

        const Real theta_b = bc.theta1[0];
        const Real theta_c = bc.theta2[0];
        const Mat3 middle_shoulder_rotation = mul(rot(n_sew, theta_b), rot(e_sw, theta_c));

        const AngleSet theta_a_set = sp4(
            n_sew, mul(middle_shoulder_rotation, robot.p[3]), e_sw, 0, cfg.tolerance.subproblem);
        for (int theta_a_index = 0; theta_a_index < theta_a_set.n; ++theta_a_index) {
            const Real theta_a = theta_a_set.a[theta_a_index];
            const Mat3 R03 = mul(rot(e_sw, theta_a), middle_shoulder_rotation);

            if (dot(e_ce, mul(R03, robot.p[3])) < -cfg.tolerance.geometry) {
                continue;
            }

            const Sp2Result q12 =
                sp2(robot.h[2], mul(R03, robot.h[2]), robot.h[1], scale(robot.h[0], -1),
                    cfg.tolerance.subproblem);
            for (int shoulder_index = 0; shoulder_index < q12.n; ++shoulder_index) {
                const Real q1_representative = q12.theta2[shoulder_index];
                const Real q2 = q12.theta1[shoulder_index];
                const Mat3 r02_representative =
                    mul(rot(robot.h[0], q1_representative), rot(robot.h[1], q2));
                const AngleSet q3_representative_set =
                    sp1(robot.h[1], mul(mul(transpose(r02_representative), R03), robot.h[1]),
                        robot.h[2], cfg.tolerance.subproblem);

                for (int q3_rep_index = 0; q3_rep_index < q3_representative_set.n; ++q3_rep_index) {
                    const Real q3_representative = q3_representative_set.a[q3_rep_index];
                    Real q1_candidates[2] = {};
                    const int q1_count = singular_outer_angle_candidates(
                        q1_representative, q2, q3_representative, robot.h[0], robot.h[1],
                        robot.h[2], q_current.v[0], q_current.v[2], cfg.selection.weights[0],
                        cfg.selection.weights[2], cfg.tolerance.spherical_singularity,
                        q1_candidates);

                    for (int q1_candidate_index = 0; q1_candidate_index < q1_count;
                         ++q1_candidate_index) {
                        const Real q1 = q1_candidates[q1_candidate_index];
                        const Mat3 R02 = mul(rot(robot.h[0], q1), rot(robot.h[1], q2));
                        const AngleSet q3_set =
                            sp1(robot.h[1], mul(mul(transpose(R02), R03), robot.h[1]), robot.h[2],
                                cfg.tolerance.subproblem);

                        for (int q3_index = 0; q3_index < q3_set.n; ++q3_index) {
                            const Real q3 = q3_set.a[q3_index];
                            const Mat3 R34 = rot(robot.h[3], q4);
                            const Mat3 R47 =
                                mul(transpose(mul(mul(R02, rot(robot.h[2], q3)), R34)), R07);

                            const Sp2Result q56 =
                                sp2(robot.h[6], mul(R47, robot.h[6]), robot.h[5],
                                    scale(robot.h[4], -1), cfg.tolerance.subproblem);
                            for (int wrist_index = 0; wrist_index < q56.n; ++wrist_index) {
                                const Real q5_representative = q56.theta2[wrist_index];
                                const Real q6 = q56.theta1[wrist_index];
                                const Mat3 r56_representative =
                                    mul(rot(robot.h[4], q5_representative), rot(robot.h[5], q6));
                                const AngleSet q7_representative_set =
                                    sp1(robot.h[5],
                                        mul(mul(transpose(r56_representative), R47), robot.h[5]),
                                        robot.h[6], cfg.tolerance.subproblem);

                                for (int q7_rep_index = 0; q7_rep_index < q7_representative_set.n;
                                     ++q7_rep_index) {
                                    const Real q7_representative =
                                        q7_representative_set.a[q7_rep_index];
                                    Real q5_candidates[2] = {};
                                    const int q5_count = singular_outer_angle_candidates(
                                        q5_representative, q6, q7_representative, robot.h[4],
                                        robot.h[5], robot.h[6], q_current.v[4], q_current.v[6],
                                        cfg.selection.weights[4], cfg.selection.weights[6],
                                        cfg.tolerance.spherical_singularity, q5_candidates);

                                    for (int q5_candidate_index = 0; q5_candidate_index < q5_count;
                                         ++q5_candidate_index) {
                                        const Real q5 = q5_candidates[q5_candidate_index];
                                        const Mat3 R56 =
                                            mul(rot(robot.h[4], q5), rot(robot.h[5], q6));
                                        const AngleSet q7_set = sp1(
                                            robot.h[5], mul(mul(transpose(R56), R47), robot.h[5]),
                                            robot.h[6], cfg.tolerance.subproblem);

                                        for (int q7_index = 0; q7_index < q7_set.n; ++q7_index) {
                                            Vec7 q = {{q1, q2, q3, q4, q5, q6, q7_set.a[q7_index]}};
                                            const bool is_ls =
                                                q4_set.least_squares || bc.least_squares
                                                || theta_a_set.least_squares || q12.least_squares
                                                || q3_representative_set.least_squares
                                                || q3_set.least_squares || q56.least_squares
                                                || q7_representative_set.least_squares
                                                || q7_set.least_squares;
                                            // 只编码三层会成倍扩展候选数的主分支；
                                            // 奇异外角内部展开仍由该主分支内的最近解逻辑处理。
                                            const uint8_t major_branch = static_cast<uint8_t>(
                                                (q4_index & 1) | ((shoulder_index & 1) << 1)
                                                | ((wrist_index & 1) << 2));
                                            add_candidate(Q, q, is_ls, major_branch);
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

inline void deduplicate(CandidateSet& set, Real tolerance) {
    // 候选解去重：闭式推导的不同路径有时会得到 2*pi 等价解。
    // 比较时使用 wrap_to_pi(q_a-q_b)，避免因为多转一圈导致重复保留。
    CandidateSet unique;
    for (int i = 0; i < set.n; ++i) {
        int duplicate = -1;
        for (int j = 0; j < unique.n; ++j) {
            Real max_delta = 0;
            for (int k = 0; k < 7; ++k) {
                max_delta = maxr(max_delta, absr(wrap_to_pi(set.c[i].q.v[k] - unique.c[j].q.v[k])));
            }
            if (max_delta <= tolerance) {
                duplicate = j;
                break;
            }
        }

        if (duplicate < 0) {
            unique.c[unique.n++] = set.c[i];
        } else {
            unique.c[duplicate].least_squares =
                unique.c[duplicate].least_squares && set.c[i].least_squares;
        }
    }
    set = unique;
}

inline bool
    fit_to_joint_limits(const Vec7& q, const Vec7& q_current, const Config& cfg, Vec7& out) {
    // 将一个 wrap 到 [-pi,pi) 的候选解调整到实际关节限位内。
    // 如果 q、q+2pi、q-2pi 中有多个等价角都在限位内，则选离 q_current 最近的那个。
    for (int i = 0; i < 7; ++i) {
        const Real lo = cfg.joint.lower_rad[i];
        const Real hi = cfg.joint.upper_rad[i];
        const Real tol = cfg.tolerance.joint_limit;
        const int min_turn = static_cast<int>(std::ceil((lo - q.v[i] - tol) / kTwoPi));
        const int max_turn = static_cast<int>(std::floor((hi - q.v[i] + tol) / kTwoPi));
        if (min_turn > max_turn) {
            return false;
        }

        const Real current_turn = std::round((q_current.v[i] - q.v[i]) / kTwoPi);
        const int center_turn = static_cast<int>(
            clamp(current_turn, static_cast<Real>(min_turn), static_cast<Real>(max_turn)));

        Real best = q.v[i] + kTwoPi * static_cast<Real>(center_turn);
        Real best_abs = absr(best - q_current.v[i]);
        const int first_turn = center_turn - 1 < min_turn ? min_turn : center_turn - 1;
        const int last_turn = center_turn + 1 > max_turn ? max_turn : center_turn + 1;
        for (int turn = first_turn; turn <= last_turn; ++turn) {
            const Real candidate = q.v[i] + kTwoPi * static_cast<Real>(turn);
            const Real d = absr(candidate - q_current.v[i]);
            if (d < best_abs) {
                best = candidate;
                best_abs = d;
            }
        }

        out.v[i] = clamp(best, lo, hi);
    }
    return true;
}

inline Real weighted_command_distance_sq(const Vec7& q, const Vec7& q_current, const Config& cfg) {
    // q 是 fit_to_joint_limits() 选出的实际关节命令表示。有限位关节不能默认
    // 跨越 +/-pi 边界，因此这里必须计算线性命令空间距离，不能 wrap_to_pi。
    Real cost = 0;
    for (int i = 0; i < 7; ++i) {
        const Real d = q.v[i] - q_current.v[i];
        cost += cfg.selection.weights[i] * d * d;
    }
    return cost;
}

inline bool validate_config(const Config& cfg) {
    for (int i = 0; i < 7; ++i) {
        if (!(cfg.joint.lower_rad[i] < cfg.joint.upper_rad[i])) {
            return false;
        }
        if (cfg.selection.weights[i] < static_cast<Real>(0)) {
            return false;
        }
    }
    if (cfg.robot.enabled) {
        return norm(cfg.robot.p[3]) > cfg.tolerance.geometry
            && norm(cfg.robot.p[4]) > cfg.tolerance.geometry;
    }
    return cfg.geometry.upper_arm_mm > static_cast<Real>(0)
        && cfg.geometry.forearm_mm > static_cast<Real>(0)
        && cfg.geometry.end_effector_mm >= static_cast<Real>(0)
        && cfg.geometry.flange_extension_mm >= static_cast<Real>(0);
}

} // namespace detail

inline Config default_config() { return detail::make_default_config(); }

inline Pose forward_kinematics(const Vec7& q, const Config& cfg = default_config()) {
    return detail::forward_kinematics(q, detail::build_robot(cfg));
}

namespace detail {

// 固定臂角解析求解的内部实现。候选通过限位映射和 FK 检查后，由 CandidateFilter
// 检查额外约束；基础 solve() 接受所有通过校验的候选。
template <typename CandidateFilter>
inline bool solve_with_resolved_psi_filtered(
    const Pose& target_world, const Vec7& q_current, Real psi, bool psi_was_inferred_from_current,
    const Config& cfg, IkResult& result, const CandidateFilter& candidate_accepted) {
    result = IkResult{};
    result.psi_was_inferred_from_current = psi_was_inferred_from_current;

    if (!detail::validate_config(cfg)) {
        return false;
    }

    const detail::Robot robot = detail::build_robot(cfg);
    const Pose target = detail::world_target_to_robot(target_world, cfg.installation);

    psi = detail::wrap_to_pi(psi);
    result.psi = psi;

    const Mat3 R07 = detail::mul(target.r, detail::transpose(robot.r_tool_home));

    // 完整生成解析候选，再统一执行去重、限位和 FK 验证。
    detail::CandidateSet raw;
    detail::ik3rr3r(R07, target.p, psi, q_current, robot, cfg, raw);
    detail::deduplicate(raw, cfg.tolerance.duplicate_joint);
    result.raw_candidate_count = raw.n;

    bool have_best = false;
    Real best_cost = static_cast<Real>(0);
    Vec7 best_q = {{0, 0, 0, 0, 0, 0, 0}};
    Real best_pos_err = static_cast<Real>(0);
    Real best_ori_err = static_cast<Real>(0);

    for (int i = 0; i < raw.n; ++i) {
        Vec7 q_limited;
        if (!detail::fit_to_joint_limits(raw.c[i].q, q_current, cfg, q_limited)) {
            continue;
        }
        if (!cfg.solver.accept_least_squares && raw.c[i].least_squares) {
            continue;
        }

        const Pose check = detail::forward_kinematics(q_limited, robot);
        const Real pos_err = detail::norm(detail::sub(check.p, target.p));
        const Real ori_err = detail::rotation_distance(check.r, target.r);
        if (pos_err > cfg.tolerance.position_mm || ori_err > cfg.tolerance.orientation_rad) {
            continue;
        }
        if (!candidate_accepted(q_limited)) {
            continue;
        }

        const Real cost = detail::weighted_command_distance_sq(q_limited, q_current, cfg);
        if (!have_best || cost < best_cost) {
            have_best = true;
            best_cost = cost;
            best_q = q_limited;
            best_pos_err = pos_err;
            best_ori_err = ori_err;
            result.selected_valid_index = result.valid_candidate_count;
            // 与 q_best 同步保存分支，调用方只能在 success=true 时使用它延续状态。
            result.selected_major_branch = static_cast<int>(raw.c[i].major_branch);
        }
        ++result.valid_candidate_count;
    }

    if (!have_best) {
        result.success = false;
        return false;
    }

    result.success = true;
    result.q_best = best_q;
    result.best_cost = best_cost;
    result.best_position_error_mm = best_pos_err;
    result.best_orientation_error_rad = best_ori_err;
    return true;
}

// 基础固定-psi 入口完整接受所有通过限位和 FK 校验的解析候选。
inline bool solve_with_resolved_psi(
    const Pose& target_world, const Vec7& q_current, Real psi, bool psi_was_inferred_from_current,
    const Config& cfg, IkResult& result) {
    const auto accept_all = [](const Vec7&) { return true; };
    return solve_with_resolved_psi_filtered(
        target_world, q_current, psi, psi_was_inferred_from_current, cfg, result, accept_all);
}

inline bool infer_current_psi(const Vec7& q_current, const Config& cfg, Real& psi) {
    if (!validate_config(cfg)) {
        return false;
    }

    const Robot robot = build_robot(cfg);
    Vec3 S;
    Vec3 E;
    Vec3 W;
    arm_points(q_current, robot, S, E, W);
    return sew_forward(S, E, W, cfg, psi);
}

} // namespace detail

// 逆解主入口。
// psi_is_given=true：使用传入的 psi；
// psi_is_given=false：从 q_current 自动推断当前 SEW 角。若当前 psi 无解，
// 再按 +/-5deg、+/-10deg、+/-20deg、+/-40deg 做局部臂角搜索。
inline bool solve(
    const Pose& target_world, const Vec7& q_current, Real psi, bool psi_is_given, const Config& cfg,
    IkResult& result) {
    if (psi_is_given) {
        return detail::solve_with_resolved_psi(target_world, q_current, psi, false, cfg, result);
    }

    if (!detail::infer_current_psi(q_current, cfg, psi)) {
        result = IkResult{};
        result.psi_was_inferred_from_current = true;
        return false;
    }

    IkResult base_result;
    if (detail::solve_with_resolved_psi(target_world, q_current, psi, true, cfg, base_result)) {
        result = base_result;
        return true;
    }

    static constexpr Real kDegToRad = kPi / static_cast<Real>(180);
    static constexpr Real kPsiSearchOffsets[] = {
        static_cast<Real>(5) * kDegToRad,  static_cast<Real>(-5) * kDegToRad,
        static_cast<Real>(10) * kDegToRad, static_cast<Real>(-10) * kDegToRad,
        static_cast<Real>(20) * kDegToRad, static_cast<Real>(-20) * kDegToRad,
        static_cast<Real>(40) * kDegToRad, static_cast<Real>(-40) * kDegToRad,
    };

    bool have_best = false;
    IkResult best_result;
    for (Real const offset : kPsiSearchOffsets) {
        IkResult candidate_result;
        if (!detail::solve_with_resolved_psi(
                target_world, q_current, psi + offset, true, cfg, candidate_result)) {
            continue;
        }
        if (!have_best || candidate_result.best_cost < best_result.best_cost) {
            have_best = true;
            best_result = candidate_result;
        }
    }

    result = have_best ? best_result : base_result;
    return have_best;
}

inline bool
    solve(const Pose& target_world, const Vec7& q_current, const Config& cfg, IkResult& result) {
    return solve(target_world, q_current, static_cast<Real>(0), false, cfg, result);
}

} // namespace rmcs_core::controller::arm
