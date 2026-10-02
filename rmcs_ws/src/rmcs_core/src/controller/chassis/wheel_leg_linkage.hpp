#pragma once

// 并联(闭链)轮腿的连杆几何/力映射，供虚拟关节状态组件与力矩控制器共用。
// 与 wheel_leg_real2sim / infantry_binglian 的 solve_leg_geometry / solve_phi3_* /
// solve_phi_piandao / solve_jacobian_analytic / force_map 保持一致。

#include <algorithm>
#include <cmath>
#include <numbers>

namespace rmcs_core::controller::chassis::linkage {

struct LegGeometry {
    double phi2 = 0.0;
    double phi3 = 0.0;
    double phi0 = 0.0;
    double l0 = 0.0;
};

inline LegGeometry solve_leg_geometry(double phi1, double phi4, double l1, double l2) {
    const double x_b = l1 * std::cos(phi1);
    const double y_b = l1 * std::sin(phi1);
    const double x_d = l1 * std::cos(phi4);
    const double y_d = l1 * std::sin(phi4);

    const double dx = x_d - x_b;
    const double dy = y_d - y_b;
    const double a0 = 2.0 * l2 * dx;
    const double b0 = 2.0 * l2 * dy;
    const double c0 = dx * dx + dy * dy;
    const double disc = std::max(a0 * a0 + b0 * b0 - c0 * c0, 0.0);
    const double phi2 = 2.0 * std::atan2(b0 + std::sqrt(disc), a0 + c0);

    const double x_c = x_b + l2 * std::cos(phi2);
    const double y_c = y_b + l2 * std::sin(phi2);
    const double phi3 = std::atan2(y_c - y_d, x_c - x_d);
    return {phi2, phi3, std::atan2(y_c, x_c), std::hypot(x_c, y_c)};
}

inline double solve_phi3_left(double phi1, double phi4, double l1, double l2) {
    return solve_leg_geometry(phi1, phi4, l1, l2).phi3 - phi4 - std::numbers::pi / 2.0;
}

inline double solve_phi3_right(double phi1, double phi4, double l1, double l2) {
    return -solve_leg_geometry(phi1, phi4, l1, l2).phi3 + phi4 + std::numbers::pi / 2.0;
}

struct Jacobian {
    double d_phi1 = 0.0;
    double d_phi4 = 0.0;
};

inline Jacobian solve_phi_piandao(
    double (*solver)(double, double, double, double), double phi1, double phi4, double l1,
    double l2, double eps) {
    return {
        (solver(phi1 + eps, phi4, l1, l2) - solver(phi1 - eps, phi4, l1, l2)) / (2.0 * eps),
        (solver(phi1, phi4 + eps, l1, l2) - solver(phi1, phi4 - eps, l1, l2)) / (2.0 * eps)};
}

inline Jacobian solve_jacobian_analytic(double phi1, double phi4, double l1, double l2) {
    const auto geometry = solve_leg_geometry(phi1, phi4, l1, l2);
    const double phi_lf = std::abs(geometry.phi3 - phi4 - std::numbers::pi / 2.0);
    const double beta = 0.5 * std::abs(phi1 - phi4);
    double denom = 2.0 * l2 * std::cos(beta - phi_lf);
    if (std::abs(denom) < 1e-6)
        denom = std::copysign(1e-6, denom != 0.0 ? denom : 1.0);
    const double k = (l1 * std::cos(beta) + l2 * std::cos(beta - phi_lf)) / denom;
    return {k, -k};
}

struct ForceMap {
    double j00 = 0.0, j01 = 0.0, j10 = 0.0, j11 = 0.0;
    double i00 = 0.0, i01 = 0.0, i10 = 0.0, i11 = 0.0;
    double l0 = 0.0;
};

inline ForceMap force_map(double phi1, double phi4, double l1, double l2) {
    const auto geometry = solve_leg_geometry(phi1, phi4, l1, l2);
    const double j11 = std::sin(geometry.phi0 - geometry.phi3) * l1
                     * std::sin(phi1 - geometry.phi2) / std::sin(geometry.phi3 - geometry.phi2);
    const double j12 = std::cos(geometry.phi0 - geometry.phi3) * l1
                     * std::sin(phi1 - geometry.phi2)
                     / (std::sin(geometry.phi3 - geometry.phi2) * geometry.l0);
    const double j21 = std::sin(geometry.phi0 - geometry.phi2) * l1
                     * std::sin(geometry.phi3 - phi4) / std::sin(geometry.phi3 - geometry.phi2);
    const double j22 = std::cos(geometry.phi0 - geometry.phi2) * l1
                     * std::sin(geometry.phi3 - phi4)
                     / (std::sin(geometry.phi3 - geometry.phi2) * geometry.l0);
    const double det = j11 * j22 - j12 * j21;
    ForceMap map;
    map.j00 = j11;
    map.j01 = j12;
    map.j10 = j21;
    map.j11 = j22;
    map.i00 = j22 / det;
    map.i01 = -j12 / det;
    map.i10 = -j21 / det;
    map.i11 = j11 / det;
    map.l0 = geometry.l0;
    return map;
}

} // namespace rmcs_core::controller::chassis::linkage
