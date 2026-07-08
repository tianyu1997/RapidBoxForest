#include "planning_forest_obb_zonotope.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <utility>
#include <vector>

namespace rbf {

struct ObbPortalAffineScalar {
    double ctr = 0.0;
    std::array<double, MAX_JOINTS> lin{};
    double rem = 0.0;
};

struct ObbPortalAffineMatrix4 {
    std::array<ObbPortalAffineScalar, 16> m{};
};

Eigen::MatrixXd obb_compress_generator_columns(const Eigen::MatrixXd& generators,
                                               double column_norm_tol) {
    if (generators.rows() <= 0 || generators.cols() <= 0) {
        return Eigen::MatrixXd::Zero(generators.rows(), 0);
    }
    std::vector<int> active;
    active.reserve(static_cast<std::size_t>(generators.cols()));
    for (int col = 0; col < generators.cols(); ++col) {
        if (generators.col(col).lpNorm<1>() > column_norm_tol) {
            active.push_back(col);
        }
    }
    Eigen::MatrixXd compressed(generators.rows(), static_cast<int>(active.size()));
    for (int out = 0; out < static_cast<int>(active.size()); ++out) {
        compressed.col(out) = generators.col(active[static_cast<std::size_t>(out)]);
    }
    return compressed;
}

void obb_affine_zero(ObbPortalAffineScalar& scalar) {
    scalar.ctr = 0.0;
    scalar.lin.fill(0.0);
    scalar.rem = 0.0;
}

void obb_affine_identity(ObbPortalAffineMatrix4& matrix) {
    for (auto& scalar : matrix.m) {
        obb_affine_zero(scalar);
    }
    matrix.m[0].ctr = 1.0;
    matrix.m[5].ctr = 1.0;
    matrix.m[10].ctr = 1.0;
    matrix.m[15].ctr = 1.0;
}

double obb_affine_linear_radius(const ObbPortalAffineScalar& scalar, int n_vars) {
    double radius = 0.0;
    for (int var = 0; var < n_vars; ++var) {
        radius += std::abs(scalar.lin[static_cast<std::size_t>(var)]);
    }
    return radius;
}

void obb_affine_mat_mul(const ObbPortalAffineMatrix4& lhs,
                        const ObbPortalAffineMatrix4& rhs,
                        ObbPortalAffineMatrix4& out,
                        int n_vars) {
    for (auto& scalar : out.m) {
        obb_affine_zero(scalar);
    }
    out.m[15].ctr = 1.0;

    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 4; ++col) {
            ObbPortalAffineScalar& result = out.m[static_cast<std::size_t>(row * 4 + col)];
            obb_affine_zero(result);
            for (int k = 0; k < 3; ++k) {
                const auto& te = lhs.m[static_cast<std::size_t>(row * 4 + k)];
                const auto& ae = rhs.m[static_cast<std::size_t>(k * 4 + col)];
                result.ctr += te.ctr * ae.ctr;
                for (int var = 0; var < n_vars; ++var) {
                    const std::size_t idx = static_cast<std::size_t>(var);
                    result.lin[idx] += te.ctr * ae.lin[idx] + te.lin[idx] * ae.ctr;
                }
                const double te_rad = obb_affine_linear_radius(te, n_vars);
                const double ae_rad = obb_affine_linear_radius(ae, n_vars);
                const double te_rem = std::abs(te.rem);
                const double ae_rem = std::abs(ae.rem);
                result.rem += te_rad * ae_rad
                    + te_rem * (std::abs(ae.ctr) + ae_rad + ae_rem)
                    + te_rad * ae_rem
                    + std::abs(te.ctr) * ae_rem
                    + te_rem * ae_rem;
            }
            if (col == 3) {
                const auto& te3 = lhs.m[static_cast<std::size_t>(row * 4 + 3)];
                result.ctr += te3.ctr;
                for (int var = 0; var < n_vars; ++var) {
                    const std::size_t idx = static_cast<std::size_t>(var);
                    result.lin[idx] += te3.lin[idx];
                }
                result.rem += std::abs(te3.rem);
            }
        }
    }
}

void obb_affine_build_dh_joint(const Robot& robot,
                               int joint_idx,
                               const Eigen::VectorXd& center,
                               const Eigen::MatrixXd& generators,
                               ObbPortalAffineMatrix4& matrix,
                               int n_vars) {
    obb_affine_identity(matrix);
    for (auto& scalar : matrix.m) {
        obb_affine_zero(scalar);
    }
    matrix.m[15].ctr = 1.0;

    const auto& dh = robot.dh_params()[static_cast<std::size_t>(joint_idx)];
    const double ca = std::cos(dh.alpha);
    const double sa = std::sin(dh.alpha);

    if (dh.joint_type == 0) {
        const double theta0 = center[joint_idx] + dh.theta;
        const double ct0 = std::cos(theta0);
        const double st0 = std::sin(theta0);
        double theta_radius = 0.0;
        for (int var = 0; var < n_vars; ++var) {
            theta_radius += std::abs(generators(joint_idx, var));
        }
        const double trig_rem = 0.5 * theta_radius * theta_radius;
        const double d_val = dh.d;

        matrix.m[0].ctr = ct0;
        matrix.m[1].ctr = -st0;
        matrix.m[3].ctr = dh.a;
        matrix.m[4].ctr = st0 * ca;
        matrix.m[5].ctr = ct0 * ca;
        matrix.m[6].ctr = -sa;
        matrix.m[7].ctr = -d_val * sa;
        matrix.m[8].ctr = st0 * sa;
        matrix.m[9].ctr = ct0 * sa;
        matrix.m[10].ctr = ca;
        matrix.m[11].ctr = d_val * ca;

        for (int var = 0; var < n_vars; ++var) {
            const double g = generators(joint_idx, var);
            const std::size_t idx = static_cast<std::size_t>(var);
            matrix.m[0].lin[idx] = -st0 * g;
            matrix.m[1].lin[idx] = -ct0 * g;
            matrix.m[4].lin[idx] = ct0 * ca * g;
            matrix.m[5].lin[idx] = -st0 * ca * g;
            matrix.m[8].lin[idx] = ct0 * sa * g;
            matrix.m[9].lin[idx] = -st0 * sa * g;
        }
        matrix.m[0].rem = trig_rem;
        matrix.m[1].rem = trig_rem;
        matrix.m[4].rem = trig_rem * std::abs(ca);
        matrix.m[5].rem = trig_rem * std::abs(ca);
        matrix.m[8].rem = trig_rem * std::abs(sa);
        matrix.m[9].rem = trig_rem * std::abs(sa);
    } else {
        const double theta = dh.theta;
        const double ct0 = std::cos(theta);
        const double st0 = std::sin(theta);
        const double d0 = center[joint_idx] + dh.d;

        matrix.m[0].ctr = ct0;
        matrix.m[1].ctr = -st0;
        matrix.m[3].ctr = dh.a;
        matrix.m[4].ctr = st0 * ca;
        matrix.m[5].ctr = ct0 * ca;
        matrix.m[6].ctr = -sa;
        matrix.m[7].ctr = -d0 * sa;
        matrix.m[8].ctr = st0 * sa;
        matrix.m[9].ctr = ct0 * sa;
        matrix.m[10].ctr = ca;
        matrix.m[11].ctr = d0 * ca;
        for (int var = 0; var < n_vars; ++var) {
            const double g = generators(joint_idx, var);
            const std::size_t idx = static_cast<std::size_t>(var);
            matrix.m[7].lin[idx] = -sa * g;
            matrix.m[11].lin[idx] = ca * g;
        }
    }
}

void obb_affine_build_tool(const DHParam& tool, ObbPortalAffineMatrix4& matrix) {
    obb_affine_identity(matrix);
    const double ca = std::cos(tool.alpha);
    const double sa = std::sin(tool.alpha);
    const double ct = std::cos(tool.theta);
    const double st = std::sin(tool.theta);
    matrix.m[0].ctr = ct;
    matrix.m[1].ctr = -st;
    matrix.m[3].ctr = tool.a;
    matrix.m[4].ctr = st * ca;
    matrix.m[5].ctr = ct * ca;
    matrix.m[6].ctr = -sa;
    matrix.m[7].ctr = -tool.d * sa;
    matrix.m[8].ctr = st * sa;
    matrix.m[9].ctr = ct * sa;
    matrix.m[10].ctr = ca;
    matrix.m[11].ctr = tool.d * ca;
}

detail::Vec3 obb_affine_vec3(const ObbPortalAffineMatrix4& transform,
                             int var,
                             bool linear) {
    if (linear) {
        const std::size_t idx = static_cast<std::size_t>(var);
        return detail::make_vec3(static_cast<float>(transform.m[3].lin[idx]),
                                 static_cast<float>(transform.m[7].lin[idx]),
                                 static_cast<float>(transform.m[11].lin[idx]));
    }
    return detail::make_vec3(static_cast<float>(transform.m[3].ctr),
                             static_cast<float>(transform.m[7].ctr),
                             static_cast<float>(transform.m[11].ctr));
}

ObbEndpointZonotope obb_extract_endpoint_zonotope(const ObbPortalAffineMatrix4& transform,
                                                  int n_vars) {
    ObbEndpointZonotope endpoint;
    endpoint.center = obb_affine_vec3(transform, 0, false);
    endpoint.generators.reserve(static_cast<std::size_t>(n_vars));
    for (int var = 0; var < n_vars; ++var) {
        endpoint.generators.push_back(obb_affine_vec3(transform, var, true));
    }
    endpoint.remainder = detail::make_vec3(static_cast<float>(std::abs(transform.m[3].rem)),
                                           static_cast<float>(std::abs(transform.m[7].rem)),
                                           static_cast<float>(std::abs(transform.m[11].rem)));
    return endpoint;
}

std::vector<ObbLinkZonotopeHull> obb_compute_link_zonotopes(const Robot& robot,
                                                            const Eigen::VectorXd& center,
                                                            const Eigen::MatrixXd& generators,
                                                            int n_vars) {
    const int n_joints = robot.n_joints();
    std::array<ObbPortalAffineMatrix4, MAX_TF> prefix;
    ObbPortalAffineMatrix4 joint_matrix;
    obb_affine_identity(prefix[0]);
    for (int joint = 0; joint < n_joints; ++joint) {
        obb_affine_build_dh_joint(robot, joint, center, generators, joint_matrix, n_vars);
        obb_affine_mat_mul(prefix[static_cast<std::size_t>(joint)],
                           joint_matrix,
                           prefix[static_cast<std::size_t>(joint + 1)],
                           n_vars);
    }
    if (robot.has_tool()) {
        obb_affine_build_tool(*robot.tool_frame(), joint_matrix);
        obb_affine_mat_mul(prefix[static_cast<std::size_t>(n_joints)],
                           joint_matrix,
                           prefix[static_cast<std::size_t>(n_joints + 1)],
                           n_vars);
    }

    const int n_active_links = robot.n_active_links();
    const int* active_link_map = robot.active_link_map();
    const double* radii = robot.active_link_radii();
    std::vector<ObbLinkZonotopeHull> links;
    links.reserve(static_cast<std::size_t>(n_active_links));
    for (int active = 0; active < n_active_links; ++active) {
        const int link_idx = active_link_map[active];
        ObbLinkZonotopeHull link;
        link.proximal = obb_extract_endpoint_zonotope(prefix[static_cast<std::size_t>(link_idx)],
                                                      n_vars);
        link.distal = obb_extract_endpoint_zonotope(prefix[static_cast<std::size_t>(link_idx + 1)],
                                                    n_vars);
        link.radius = radii != nullptr ? radii[active] : 0.0;
        links.push_back(std::move(link));
    }
    return links;
}

detail::Vec3 obb_support_endpoint(const ObbEndpointZonotope& endpoint,
                                  const detail::Vec3& dir) {
    detail::Vec3 support = endpoint.center;
    for (const auto& generator : endpoint.generators) {
        if (detail::dot(generator, dir) >= 0.0f) {
            support = support + generator;
        } else {
            support = support - generator;
        }
    }
    support.x += dir.x >= 0.0f ? endpoint.remainder.x : -endpoint.remainder.x;
    support.y += dir.y >= 0.0f ? endpoint.remainder.y : -endpoint.remainder.y;
    support.z += dir.z >= 0.0f ? endpoint.remainder.z : -endpoint.remainder.z;
    return support;
}

void obb_endpoint_bounds(const ObbEndpointZonotope& endpoint, float out[6]) {
    float rx = std::abs(endpoint.remainder.x);
    float ry = std::abs(endpoint.remainder.y);
    float rz = std::abs(endpoint.remainder.z);
    for (const auto& generator : endpoint.generators) {
        rx += std::abs(generator.x);
        ry += std::abs(generator.y);
        rz += std::abs(generator.z);
    }
    out[0] = endpoint.center.x - rx;
    out[1] = endpoint.center.y - ry;
    out[2] = endpoint.center.z - rz;
    out[3] = endpoint.center.x + rx;
    out[4] = endpoint.center.y + ry;
    out[5] = endpoint.center.z + rz;
}

void obb_compute_link_aabb(ObbLinkZonotopeHull& link, double pad) {
    float prox[6];
    float dist[6];
    obb_endpoint_bounds(link.proximal, prox);
    obb_endpoint_bounds(link.distal, dist);
    const float p = static_cast<float>(std::max(0.0, link.radius + pad));
    for (int axis = 0; axis < 3; ++axis) {
        link.aabb[axis] = std::min(prox[axis], dist[axis]) - p;
        link.aabb[axis + 3] = std::max(prox[axis + 3], dist[axis + 3]) + p;
    }
}

detail::Vec3 obb_support_link(const ObbLinkZonotopeHull& link,
                              const detail::Vec3& dir) {
    const detail::Vec3 prox = obb_support_endpoint(link.proximal, dir);
    const detail::Vec3 dist = obb_support_endpoint(link.distal, dir);
    return detail::dot(prox, dir) >= detail::dot(dist, dir) ? prox : dist;
}

detail::Vec3 obb_support_capsule_link(const ObbLinkZonotopeHull& link,
                                      const detail::Vec3& dir,
                                      float radius) {
    detail::Vec3 support = obb_support_link(link, dir);
    const float norm_sq = detail::norm_sq(dir);
    if (radius > 0.0f && norm_sq > detail::kCollisionEps) {
        const float inv_norm = 1.0f / std::sqrt(norm_sq);
        support = support + dir * (radius * inv_norm);
    }
    return support;
}

detail::Vec3 obb_support_minkowski_link_vs_obstacle(const ObbLinkZonotopeHull& link,
                                                    const float* obstacle,
                                                    float radius,
                                                    const detail::Vec3& dir) {
    return obb_support_capsule_link(link, dir, radius) -
           detail::support_aabb(obstacle, -dir, 0.0f);
}

bool obb_zonotope_link_separates_obstacle(const ObbLinkZonotopeHull& link,
                                          const float* obstacle,
                                          double pad,
                                          ObbPortalValidationStats& stats) {
    stats.aabb_tests += 1;
    if (!detail::aabb_overlap_padded(link.aabb, obstacle, 0.0f)) {
        stats.aabb_rejects += 1;
        return true;
    }

    const detail::Vec3 link_center =
        (link.proximal.center + link.distal.center) * 0.5f;
    detail::Vec3 direction = detail::obstacle_center(obstacle) - link_center;
    if (detail::norm_sq(direction) <= detail::kCollisionEps) {
        direction = detail::make_vec3(1.0f, 0.0f, 0.0f);
    }

    detail::Simplex simplex;
    const float capsule_radius = static_cast<float>(std::max(0.0, link.radius + pad));
    simplex.push_front(obb_support_minkowski_link_vs_obstacle(link, obstacle, capsule_radius, direction));
    direction = -simplex.points[0];
    stats.gjk_tests += 1;
    if (detail::norm_sq(direction) <= detail::kCollisionEps) {
        stats.maybe_pairs += 1;
        return false;
    }

    constexpr int kMaxIterations = 32;
    for (int iter = 0; iter < kMaxIterations; ++iter) {
        stats.gjk_iterations += 1;
        const detail::Vec3 support =
            obb_support_minkowski_link_vs_obstacle(link, obstacle, capsule_radius, direction);
        if (detail::dot(support, direction) < 0.0f) {
            stats.gjk_rejects += 1;
            return true;
        }
        simplex.push_front(support);
        if (detail::update_simplex(simplex, direction)) {
            stats.maybe_pairs += 1;
            return false;
        }
    }
    stats.maybe_pairs += 1;
    return false;
}

}  // namespace rbf
