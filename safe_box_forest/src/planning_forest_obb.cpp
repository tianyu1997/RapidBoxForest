#include "planning_forest_obb.h"

#include <sbf/envelope/envelope_collision.h>

#include "env_config.h"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
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

struct ObbEndpointZonotope {
    detail::Vec3 center{};
    std::vector<detail::Vec3> generators;
    detail::Vec3 remainder{};
};

struct ObbLinkZonotopeHull {
    ObbEndpointZonotope proximal;
    ObbEndpointZonotope distal;
    double radius = 0.0;
    float aabb[6] = {};
};


struct ObbPortalCandidate {
    Eigen::VectorXd center_q;
    Eigen::VectorXd center_y;
    Eigen::MatrixXd basis_y;
    Eigen::VectorXd radii_y;
    Eigen::MatrixXd generators_q;
    double score = -std::numeric_limits<double>::infinity();
};

bool obb_sampled_support_enabled() {
    return detail::env_flag_or_default("RBF_OBB_SAMPLED_SUPPORT", false);
}

Eigen::MatrixXd obb_compress_generator_columns(const Eigen::MatrixXd& generators,
                                               double column_norm_tol = 1e-12) {
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

struct ObbSampledEndpointHull {
    std::vector<detail::Vec3> samples;
    float error_radius = 0.0f;
};

struct ObbSampledLinkHull {
    ObbSampledEndpointHull proximal;
    ObbSampledEndpointHull distal;
    double radius = 0.0;
    float aabb[6] = {};
};

double obb_transform_translation_bound(const Robot& robot, int transform_index) {
    if (transform_index < robot.n_joints()) {
        const auto& dh = robot.dh_params()[static_cast<std::size_t>(transform_index)];
        return std::hypot(dh.a, dh.d);
    }
    if (transform_index == robot.n_joints() && robot.has_tool()) {
        const auto& tool = *robot.tool_frame();
        return std::hypot(tool.a, tool.d);
    }
    return 0.0;
}

double obb_endpoint_lipschitz_error(const Robot& robot,
                                    int frame_index,
                                    const std::vector<double>& joint_deviation) {
    const int n = robot.n_joints();
    const int clamped_frame = std::clamp(frame_index, 0, n + (robot.has_tool() ? 1 : 0));
    double error = 0.0;
    for (int joint = 0; joint < n && joint < static_cast<int>(joint_deviation.size()); ++joint) {
        if (joint >= clamped_frame) {
            continue;
        }
        const auto& dh = robot.dh_params()[static_cast<std::size_t>(joint)];
        if (dh.joint_type == 1) {
            error += std::abs(joint_deviation[static_cast<std::size_t>(joint)]);
            continue;
        }
        double reach = 1e-9;
        for (int transform = joint; transform < clamped_frame; ++transform) {
            reach += obb_transform_translation_bound(robot, transform);
        }
        error += reach * std::abs(joint_deviation[static_cast<std::size_t>(joint)]);
    }
    return error;
}

std::vector<detail::Vec3> obb_sample_frame_positions(const Robot& robot,
                                                     const Eigen::VectorXd& q) {
    const int n = robot.n_joints();
    const int n_tf = n + 1 + (robot.has_tool() ? 1 : 0);
    std::vector<detail::Vec3> frames(static_cast<std::size_t>(n_tf));
    double T[16] = {};
    T[0] = 1.0;
    T[5] = 1.0;
    T[10] = 1.0;
    T[15] = 1.0;
    frames[0] = detail::make_vec3(0.0f, 0.0f, 0.0f);

    auto multiply_dh = [&](const DHParam& dh, double q_value) {
        const double d_val = dh.joint_type == 1 ? q_value + dh.d : dh.d;
        const double angle = dh.joint_type == 0 ? q_value + dh.theta : dh.theta;
        const double ct = std::cos(angle);
        const double st = std::sin(angle);
        const double ca = std::cos(dh.alpha);
        const double sa = std::sin(dh.alpha);
        const double A[16] = {
            ct,       -st,      0.0, dh.a,
            st * ca,   ct * ca, -sa, -d_val * sa,
            st * sa,   ct * sa,  ca,  d_val * ca,
            0.0,       0.0,     0.0, 1.0
        };
        double R[16] = {};
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 4; ++col) {
                R[row * 4 + col] =
                    T[row * 4 + 0] * A[0 * 4 + col] +
                    T[row * 4 + 1] * A[1 * 4 + col] +
                    T[row * 4 + 2] * A[2 * 4 + col] +
                    T[row * 4 + 3] * A[3 * 4 + col];
            }
        }
        R[12] = 0.0;
        R[13] = 0.0;
        R[14] = 0.0;
        R[15] = 1.0;
        std::copy(std::begin(R), std::end(R), std::begin(T));
    };

    for (int joint = 0; joint < n; ++joint) {
        multiply_dh(robot.dh_params()[static_cast<std::size_t>(joint)], q[joint]);
        frames[static_cast<std::size_t>(joint + 1)] =
            detail::make_vec3(static_cast<float>(T[3]),
                              static_cast<float>(T[7]),
                              static_cast<float>(T[11]));
    }
    if (robot.has_tool()) {
        multiply_dh(*robot.tool_frame(), 0.0);
        frames[static_cast<std::size_t>(n + 1)] =
            detail::make_vec3(static_cast<float>(T[3]),
                              static_cast<float>(T[7]),
                              static_cast<float>(T[11]));
    }
    return frames;
}

detail::Vec3 obb_support_sampled_endpoint(const ObbSampledEndpointHull& endpoint,
                                          const detail::Vec3& dir) {
    if (endpoint.samples.empty()) {
        return detail::make_vec3(0.0f, 0.0f, 0.0f);
    }
    detail::Vec3 support = endpoint.samples.front();
    float best = detail::dot(support, dir);
    for (const auto& sample : endpoint.samples) {
        const float value = detail::dot(sample, dir);
        if (value > best) {
            best = value;
            support = sample;
        }
    }
    const float norm_sq = detail::norm_sq(dir);
    if (endpoint.error_radius > 0.0f && norm_sq > detail::kCollisionEps) {
        support = support + dir * (endpoint.error_radius / std::sqrt(norm_sq));
    }
    return support;
}

void obb_sampled_link_aabb(ObbSampledLinkHull& link, double pad) {
    const float p = static_cast<float>(std::max(0.0, link.radius + pad));
    const float prox_error = std::max(0.0f, link.proximal.error_radius);
    const float dist_error = std::max(0.0f, link.distal.error_radius);
    for (int axis = 0; axis < 3; ++axis) {
        link.aabb[axis] = std::numeric_limits<float>::infinity();
        link.aabb[axis + 3] = -std::numeric_limits<float>::infinity();
    }
    auto expand = [&](const std::vector<detail::Vec3>& samples, float error) {
        for (const auto& sample : samples) {
            const float values[3] = {sample.x, sample.y, sample.z};
            for (int axis = 0; axis < 3; ++axis) {
                link.aabb[axis] = std::min(link.aabb[axis], values[axis] - error - p);
                link.aabb[axis + 3] = std::max(link.aabb[axis + 3], values[axis] + error + p);
            }
        }
    };
    expand(link.proximal.samples, prox_error);
    expand(link.distal.samples, dist_error);
}

detail::Vec3 obb_support_sampled_link(const ObbSampledLinkHull& link,
                                      const detail::Vec3& dir) {
    const detail::Vec3 prox = obb_support_sampled_endpoint(link.proximal, dir);
    const detail::Vec3 dist = obb_support_sampled_endpoint(link.distal, dir);
    return detail::dot(prox, dir) >= detail::dot(dist, dir) ? prox : dist;
}

detail::Vec3 obb_support_sampled_capsule_link(const ObbSampledLinkHull& link,
                                              const detail::Vec3& dir,
                                              float radius) {
    detail::Vec3 support = obb_support_sampled_link(link, dir);
    const float norm_sq = detail::norm_sq(dir);
    if (radius > 0.0f && norm_sq > detail::kCollisionEps) {
        support = support + dir * (radius / std::sqrt(norm_sq));
    }
    return support;
}

detail::Vec3 obb_support_minkowski_sampled_link_vs_obstacle(const ObbSampledLinkHull& link,
                                                            const float* obstacle,
                                                            float radius,
                                                            const detail::Vec3& dir) {
    return obb_support_sampled_capsule_link(link, dir, radius) -
           detail::support_aabb(obstacle, -dir, 0.0f);
}

bool obb_sampled_link_separates_obstacle(const ObbSampledLinkHull& link,
                                         const float* obstacle,
                                         double pad,
                                         ObbPortalValidationStats& stats) {
    stats.aabb_tests += 1;
    if (!detail::aabb_overlap_padded(link.aabb, obstacle, 0.0f)) {
        stats.aabb_rejects += 1;
        return true;
    }
    detail::Vec3 link_center = detail::make_vec3(0.0f, 0.0f, 0.0f);
    int count = 0;
    for (const auto& sample : link.proximal.samples) {
        link_center = link_center + sample;
        ++count;
    }
    for (const auto& sample : link.distal.samples) {
        link_center = link_center + sample;
        ++count;
    }
    if (count > 0) {
        link_center = link_center * (1.0f / static_cast<float>(count));
    }
    detail::Vec3 direction = detail::obstacle_center(obstacle) - link_center;
    if (detail::norm_sq(direction) <= detail::kCollisionEps) {
        direction = detail::make_vec3(1.0f, 0.0f, 0.0f);
    }

    detail::Simplex simplex;
    const float capsule_radius = static_cast<float>(std::max(0.0, link.radius + pad));
    simplex.push_front(obb_support_minkowski_sampled_link_vs_obstacle(link, obstacle, capsule_radius, direction));
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
            obb_support_minkowski_sampled_link_vs_obstacle(link, obstacle, capsule_radius, direction);
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

bool validate_obb_sampled_support_candidate(const Robot& robot,
                                            const Scene& scene,
                                            const ObbPortalCandidate& candidate,
                                            double safety_epsilon,
                                            ObbPortalValidationStats& stats) {
    ++stats.sampled_support_attempts;
    const int dims = static_cast<int>(candidate.center_q.size());
    if (dims <= 0 || candidate.generators_q.rows() != dims || candidate.generators_q.cols() <= 0) {
        ++stats.sampled_support_fail;
        return false;
    }
    int line_col = -1;
    double best_col_norm = 0.0;
    for (int col = 0; col < candidate.generators_q.cols(); ++col) {
        const double norm = candidate.generators_q.col(col).norm();
        if (norm > best_col_norm) {
            best_col_norm = norm;
            line_col = col;
        }
    }
    if (line_col < 0 || best_col_norm <= 1e-12) {
        ++stats.sampled_support_fail;
        return false;
    }

    std::vector<double> joint_deviation(static_cast<std::size_t>(dims), 0.0);
    double lateral_joint_radius = 0.0;
    for (int joint = 0; joint < dims; ++joint) {
        for (int col = 0; col < candidate.generators_q.cols(); ++col) {
            if (col != line_col) {
                lateral_joint_radius += std::abs(candidate.generators_q(joint, col));
            }
        }
    }
    if (lateral_joint_radius > 1e-3) {
        ++stats.sampled_support_fail;
        return false;
    }
    const int kSamples = lateral_joint_radius <= 1e-8 ? 65 : 33;
    stats.sampled_support_samples += kSamples;
    const double xi_half_step = 1.0 / static_cast<double>(kSamples - 1);
    for (int joint = 0; joint < dims; ++joint) {
        joint_deviation[static_cast<std::size_t>(joint)] +=
            std::abs(candidate.generators_q(joint, line_col)) * xi_half_step;
        for (int col = 0; col < candidate.generators_q.cols(); ++col) {
            if (col != line_col) {
                joint_deviation[static_cast<std::size_t>(joint)] +=
                    std::abs(candidate.generators_q(joint, col));
            }
        }
    }

    const int n_active_links = robot.n_active_links();
    const int* active_link_map = robot.active_link_map();
    const double* radii = robot.active_link_radii();
    std::vector<ObbSampledLinkHull> links(static_cast<std::size_t>(n_active_links));
    for (int active = 0; active < n_active_links; ++active) {
        auto& link = links[static_cast<std::size_t>(active)];
        const int link_idx = active_link_map[active];
        link.proximal.samples.reserve(kSamples);
        link.distal.samples.reserve(kSamples);
        link.proximal.error_radius = static_cast<float>(
            std::max(0.0, obb_endpoint_lipschitz_error(robot, link_idx, joint_deviation)));
        link.distal.error_radius = static_cast<float>(
            std::max(0.0, obb_endpoint_lipschitz_error(robot, link_idx + 1, joint_deviation)));
        stats.sampled_support_error_radius = std::max(
            stats.sampled_support_error_radius,
            static_cast<double>(std::max(link.proximal.error_radius, link.distal.error_radius)));
    }

    for (int sample = 0; sample < kSamples; ++sample) {
        const double xi = -1.0 + 2.0 * static_cast<double>(sample) /
            static_cast<double>(kSamples - 1);
        Eigen::VectorXd q = candidate.center_q + candidate.generators_q.col(line_col) * xi;
        const auto frames = obb_sample_frame_positions(robot, q);
        for (int active = 0; active < n_active_links; ++active) {
            const int link_idx = active_link_map[active];
            if (link_idx < 0 ||
                link_idx + 1 >= static_cast<int>(frames.size())) {
                ++stats.sampled_support_fail;
                return false;
            }
            auto& link = links[static_cast<std::size_t>(active)];
            link.proximal.samples.push_back(frames[static_cast<std::size_t>(link_idx)]);
            link.distal.samples.push_back(frames[static_cast<std::size_t>(link_idx + 1)]);
            link.radius = radii != nullptr ? radii[active] : 0.0;
        }
    }

    for (auto& link : links) {
        obb_sampled_link_aabb(link, safety_epsilon);
    }
    const auto& obstacles = scene.obstacles();
    for (const auto& obstacle : obstacles) {
        const float* bounds = obstacle.bounds;
        for (const auto& link : links) {
            if (!obb_sampled_link_separates_obstacle(link, bounds, safety_epsilon, stats)) {
                ++stats.sampled_support_fail;
                return false;
            }
        }
    }
    ++stats.sampled_support_success;
    return true;
}

double obb_point_aabb_distance_sq_at_t(const detail::Vec3& origin,
                                       const detail::Vec3& dir,
                                       const float* obstacle,
                                       double t) {
    const double px = static_cast<double>(origin.x) + static_cast<double>(dir.x) * t;
    const double py = static_cast<double>(origin.y) + static_cast<double>(dir.y) * t;
    const double pz = static_cast<double>(origin.z) + static_cast<double>(dir.z) * t;
    double distance_sq = 0.0;
    const double values[3] = {px, py, pz};
    for (int axis = 0; axis < 3; ++axis) {
        const double lo = static_cast<double>(obstacle[axis]);
        const double hi = static_cast<double>(obstacle[axis + 3]);
        double delta = 0.0;
        if (values[axis] < lo) {
            delta = lo - values[axis];
        } else if (values[axis] > hi) {
            delta = values[axis] - hi;
        }
        distance_sq += delta * delta;
    }
    return distance_sq;
}

double obb_segment_aabb_distance_sq(const detail::Vec3& origin,
                                    const detail::Vec3& end,
                                    const float* obstacle) {
    const detail::Vec3 dir = end - origin;
    double best = std::min(obb_point_aabb_distance_sq_at_t(origin, dir, obstacle, 0.0),
                           obb_point_aabb_distance_sq_at_t(origin, dir, obstacle, 1.0));
    const double d[3] = {
        static_cast<double>(dir.x),
        static_cast<double>(dir.y),
        static_cast<double>(dir.z)
    };
    const double o[3] = {
        static_cast<double>(origin.x),
        static_cast<double>(origin.y),
        static_cast<double>(origin.z)
    };
    for (int axis = 0; axis < 3; ++axis) {
        if (std::abs(d[axis]) <= 1e-14) {
            continue;
        }
        for (int side = 0; side < 2; ++side) {
            const double plane =
                static_cast<double>(obstacle[axis + (side == 0 ? 0 : 3)]);
            const double t = (plane - o[axis]) / d[axis];
            if (t > 0.0 && t < 1.0) {
                best = std::min(best,
                                obb_point_aabb_distance_sq_at_t(origin,
                                                                dir,
                                                                obstacle,
                                                                std::clamp(t, 0.0, 1.0)));
            }
        }
    }
    return best;
}

bool obb_clearance_sampled_enabled() {
    return detail::env_flag_or_default("RBF_OBB_CLEARANCE_SAMPLED_SUPPORT", true);
}

bool validate_obb_clearance_sampled_candidate(const Robot& robot,
                                              const Scene& scene,
                                              const ObbPortalCandidate& candidate,
                                              double safety_epsilon,
                                              ObbPortalValidationStats& stats) {
    if (!obb_clearance_sampled_enabled()) {
        return false;
    }
    ++stats.clearance_support_attempts;
    const int dims = static_cast<int>(candidate.center_q.size());
    if (dims <= 0 ||
        candidate.generators_q.rows() != dims ||
        candidate.generators_q.cols() <= 0) {
        ++stats.clearance_support_fail;
        return false;
    }

    int line_col = -1;
    double best_col_norm = 0.0;
    for (int col = 0; col < candidate.generators_q.cols(); ++col) {
        const double norm = candidate.generators_q.col(col).norm();
        if (norm > best_col_norm) {
            best_col_norm = norm;
            line_col = col;
        }
    }
    if (line_col < 0 || best_col_norm <= 1e-12) {
        ++stats.clearance_support_fail;
        return false;
    }

    double lateral_l1 = 0.0;
    for (int joint = 0; joint < dims; ++joint) {
        for (int col = 0; col < candidate.generators_q.cols(); ++col) {
            if (col != line_col) {
                lateral_l1 += std::abs(candidate.generators_q(joint, col));
            }
        }
    }
    // The pointwise clearance proof is designed for thin bridge tubes.  Large
    // transverse OBBs should use the affine support-hull validator instead.
    if (lateral_l1 > detail::env_double_or_default("RBF_OBB_CLEARANCE_LATERAL_L1_MAX", 5e-3)) {
        ++stats.clearance_support_fail;
        return false;
    }

    const double line_l1 = candidate.generators_q.col(line_col).lpNorm<1>();
    int samples = detail::env_int_or_default("RBF_OBB_CLEARANCE_SAMPLES", 17);
    const double dense_line_l1_threshold =
        detail::env_double_or_default("RBF_OBB_CLEARANCE_DENSE_LINE_L1_THRESHOLD", 0.03);
    const int dense_samples = detail::env_int_or_default("RBF_OBB_CLEARANCE_DENSE_SAMPLES", 17);
    if (dense_line_l1_threshold > 0.0 &&
        line_l1 > dense_line_l1_threshold) {
        samples = std::max(samples, dense_samples);
    }
    samples = std::clamp(samples, 9, 257);
    const int fast_samples = std::clamp(
        detail::env_int_or_default("RBF_OBB_CLEARANCE_FAST_SAMPLES", 0),
        0,
        257);
    std::vector<int> sample_schedule;
    if (fast_samples >= 9 && fast_samples < samples) {
        sample_schedule.push_back(fast_samples);
    }
    sample_schedule.push_back(samples);

    const int n_active_links = robot.n_active_links();
    const int* active_link_map = robot.active_link_map();
    const double* radii = robot.active_link_radii();
    const auto& obstacles = scene.obstacles();
    bool any_failed = false;
    double best_min_margin = std::numeric_limits<double>::infinity();

    auto run_sample_count = [&](int sample_count, double& min_margin) {
        stats.clearance_support_samples += sample_count;
        const double xi_half_step = 1.0 / static_cast<double>(sample_count - 1);

        std::vector<double> joint_deviation(static_cast<std::size_t>(dims), 0.0);
        for (int joint = 0; joint < dims; ++joint) {
            joint_deviation[static_cast<std::size_t>(joint)] +=
                std::abs(candidate.generators_q(joint, line_col)) * xi_half_step;
            for (int col = 0; col < candidate.generators_q.cols(); ++col) {
                if (col != line_col) {
                    joint_deviation[static_cast<std::size_t>(joint)] +=
                        std::abs(candidate.generators_q(joint, col));
                }
            }
        }

        std::vector<double> frame_error(static_cast<std::size_t>(robot.n_joints() + 2), 0.0);
        for (int frame = 0; frame < static_cast<int>(frame_error.size()); ++frame) {
            frame_error[static_cast<std::size_t>(frame)] =
                std::max(0.0, obb_endpoint_lipschitz_error(robot, frame, joint_deviation));
            stats.clearance_support_error_radius =
                std::max(stats.clearance_support_error_radius,
                         frame_error[static_cast<std::size_t>(frame)]);
        }

        min_margin = std::numeric_limits<double>::infinity();
        for (int sample = 0; sample < sample_count; ++sample) {
            const double xi = -1.0 + 2.0 * static_cast<double>(sample) /
                static_cast<double>(sample_count - 1);
            Eigen::VectorXd q = candidate.center_q + candidate.generators_q.col(line_col) * xi;
            const auto frames = obb_sample_frame_positions(robot, q);
            for (int active = 0; active < n_active_links; ++active) {
                const int link_idx = active_link_map[active];
                if (link_idx < 0 ||
                    link_idx + 1 >= static_cast<int>(frames.size())) {
                    return false;
                }
                const double link_radius = radii != nullptr ? radii[active] : 0.0;
                const double motion_error = std::max(
                    frame_error[static_cast<std::size_t>(link_idx)],
                    frame_error[static_cast<std::size_t>(link_idx + 1)]);
                for (const auto& obstacle : obstacles) {
                    const double distance = std::sqrt(std::max(
                        0.0,
                        obb_segment_aabb_distance_sq(frames[static_cast<std::size_t>(link_idx)],
                                                     frames[static_cast<std::size_t>(link_idx + 1)],
                                                     obstacle.bounds)));
                    const double margin = distance - link_radius - motion_error - safety_epsilon;
                    min_margin = std::min(min_margin, margin);
                    if (!(margin > 1e-10)) {
                        return false;
                    }
                }
            }
        }
        return true;
    };

    for (int sample_count : sample_schedule) {
        double min_margin = std::numeric_limits<double>::infinity();
        if (run_sample_count(sample_count, min_margin)) {
            stats.clearance_support_min_margin =
                std::min(stats.clearance_support_min_margin, min_margin);
            ++stats.clearance_support_success;
            return true;
        }
        any_failed = true;
        best_min_margin = std::min(best_min_margin, min_margin);
    }
    if (any_failed) {
        stats.clearance_support_min_margin =
            std::min(stats.clearance_support_min_margin, best_min_margin);
    }
    ++stats.clearance_support_fail;
    return false;
}

bool obb_generators_within_domain(const Eigen::VectorXd& center,
                                  const Eigen::MatrixXd& generators,
                                  const std::vector<Interval>& domain,
                                  double tol) {
    const int dims = static_cast<int>(center.size());
    if (static_cast<int>(domain.size()) != dims || generators.rows() != dims) {
        return false;
    }
    for (int dim = 0; dim < dims; ++dim) {
        double radius = 0.0;
        for (int var = 0; var < generators.cols(); ++var) {
            radius += std::abs(generators(dim, var));
        }
        if (center[dim] - radius < domain[static_cast<std::size_t>(dim)].lo - tol ||
            center[dim] + radius > domain[static_cast<std::size_t>(dim)].hi + tol) {
            return false;
        }
    }
    return true;
}

Eigen::VectorXd obb_domain_reference(const std::vector<Interval>& domain) {
    Eigen::VectorXd ref(static_cast<int>(domain.size()));
    for (int dim = 0; dim < ref.size(); ++dim) {
        ref[dim] = domain[static_cast<std::size_t>(dim)].center();
    }
    return ref;
}

Eigen::VectorXd obb_joint_scales(const std::vector<Interval>& domain) {
    Eigen::VectorXd scales(static_cast<int>(domain.size()));
    for (int dim = 0; dim < scales.size(); ++dim) {
        const double width = domain[static_cast<std::size_t>(dim)].width();
        scales[dim] = width > 1e-12 ? 1.0 / width : 1.0;
    }
    return scales;
}

Eigen::VectorXd obb_to_scaled(const Eigen::VectorXd& q,
                              const Eigen::VectorXd& ref,
                              const Eigen::VectorXd& scales) {
    return (q - ref).cwiseProduct(scales);
}

bool obb_orthonormalize_columns(Eigen::MatrixXd& matrix) {
    const int dims = static_cast<int>(matrix.rows());
    int cols = 0;
    for (int input = 0; input < matrix.cols() && cols < dims; ++input) {
        Eigen::VectorXd candidate = matrix.col(input);
        for (int col = 0; col < cols; ++col) {
            candidate -= matrix.col(col) * matrix.col(col).dot(candidate);
        }
        const double norm = candidate.norm();
        if (norm > 1e-10) {
            matrix.col(cols++) = candidate / norm;
        }
    }
    for (int dim = 0; dim < dims && cols < dims; ++dim) {
        Eigen::VectorXd candidate = Eigen::VectorXd::Zero(dims);
        candidate[dim] = 1.0;
        for (int col = 0; col < cols; ++col) {
            candidate -= matrix.col(col) * matrix.col(col).dot(candidate);
        }
        const double norm = candidate.norm();
        if (norm > 1e-10) {
            matrix.col(cols++) = candidate / norm;
        }
    }
    return cols == dims;
}

std::vector<int> obb_low_risk_joint_order(const Robot& robot, int dims) {
    std::vector<std::pair<double, int>> scored;
    scored.reserve(static_cast<std::size_t>(dims));
    for (int joint = 0; joint < dims; ++joint) {
        double sensitivity = 1e-6;
        for (int link = joint; link < robot.n_joints(); ++link) {
            const auto& dh = robot.dh_params()[static_cast<std::size_t>(link)];
            sensitivity += std::abs(dh.a) + 0.25 * std::abs(dh.d) + 1e-3;
        }
        scored.emplace_back(sensitivity, joint);
    }
    std::sort(scored.begin(), scored.end());
    std::vector<int> order;
    order.reserve(scored.size());
    for (const auto& item : scored) {
        order.push_back(item.second);
    }
    return order;
}

bool obb_make_candidate_from_scaled(const Eigen::VectorXd& center_y,
                                    const Eigen::MatrixXd& basis_y,
                                    const Eigen::VectorXd& radii_y,
                                    const std::vector<Interval>& domain,
                                    ObbPortalCandidate& candidate) {
    const int dims = static_cast<int>(center_y.size());
    if (dims <= 0 ||
        basis_y.rows() != dims ||
        basis_y.cols() != dims ||
        radii_y.size() != dims) {
        return false;
    }
    const Eigen::VectorXd ref = obb_domain_reference(domain);
    const Eigen::VectorXd scales = obb_joint_scales(domain);
    candidate.center_y = center_y;
    candidate.basis_y = basis_y;
    candidate.radii_y = radii_y;
    candidate.center_q.resize(dims);
    for (int dim = 0; dim < dims; ++dim) {
        candidate.center_q[dim] = ref[dim] + center_y[dim] / scales[dim];
    }
    candidate.generators_q = Eigen::MatrixXd::Zero(dims, dims);
    for (int row = 0; row < dims; ++row) {
        for (int col = 0; col < dims; ++col) {
            candidate.generators_q(row, col) = basis_y(row, col) * radii_y[col] / scales[row];
        }
    }
    if (!obb_generators_within_domain(candidate.center_q,
                                      candidate.generators_q,
                                      domain,
                                      1e-10)) {
        return false;
    }
    double score = 0.0;
    for (int col = 0; col < dims; ++col) {
        score += std::log(std::max(1e-14, radii_y[col]));
    }
    candidate.score = score;
    return true;
}

bool obb_fit_scaled_path_with_basis(const std::vector<Eigen::VectorXd>& path,
                                    const std::vector<Interval>& domain,
                                    const Eigen::MatrixXd& basis_y,
                                    double lateral_radius,
                                    double longitudinal_margin,
                                    ObbPortalCandidate& candidate,
                                    ObbPortalValidationStats& stats) {
    const int dims = static_cast<int>(path.front().size());
    const Eigen::VectorXd ref = obb_domain_reference(domain);
    const Eigen::VectorXd scales = obb_joint_scales(domain);
    std::vector<Eigen::VectorXd> scaled_path;
    scaled_path.reserve(path.size());
    for (const auto& waypoint : path) {
        if (waypoint.size() != dims) {
            ++stats.degenerate_rejects;
            return false;
        }
        scaled_path.push_back(obb_to_scaled(waypoint, ref, scales));
    }
    Eigen::VectorXd min_proj = Eigen::VectorXd::Constant(dims, std::numeric_limits<double>::infinity());
    Eigen::VectorXd max_proj = Eigen::VectorXd::Constant(dims, -std::numeric_limits<double>::infinity());
    for (const auto& y : scaled_path) {
        const Eigen::VectorXd z = basis_y.transpose() * y;
        for (int col = 0; col < dims; ++col) {
            min_proj[col] = std::min(min_proj[col], z[col]);
            max_proj[col] = std::max(max_proj[col], z[col]);
        }
    }
    Eigen::VectorXd center_proj = 0.5 * (min_proj + max_proj);
    Eigen::VectorXd radii_y = 0.5 * (max_proj - min_proj);
    radii_y[0] += std::max(0.0, longitudinal_margin);
    const double lateral = std::max(0.0, lateral_radius);
    for (int col = 1; col < dims; ++col) {
        radii_y[col] += lateral;
    }
    for (int col = 0; col < dims; ++col) {
        const double min_radius = (col == 0 || lateral > 0.0) ? 1e-8 : 0.0;
        radii_y[col] = std::max(radii_y[col], min_radius);
    }
    const Eigen::VectorXd center_y = basis_y * center_proj;
    if (!obb_make_candidate_from_scaled(center_y, basis_y, radii_y, domain, candidate)) {
        ++stats.joint_limit_rejects;
        return false;
    }
    stats.longitudinal_radius = std::max(stats.longitudinal_radius, radii_y[0]);
    stats.lateral_radius = std::max(stats.lateral_radius, lateral);
    stats.variables = dims;
    return true;
}

std::vector<Eigen::MatrixXd> obb_orientation_candidates(const Robot& robot,
                                                        const std::vector<Eigen::VectorXd>& path,
                                                        const std::vector<Interval>& domain,
                                                        ObbPortalValidationStats& stats,
                                                        bool primary_only = false) {
    std::vector<Eigen::MatrixXd> candidates;
    const int dims = static_cast<int>(path.front().size());
    const Eigen::VectorXd ref = obb_domain_reference(domain);
    const Eigen::VectorXd scales = obb_joint_scales(domain);
    std::vector<Eigen::VectorXd> scaled_path;
    scaled_path.reserve(path.size());
    for (const auto& waypoint : path) {
        scaled_path.push_back(obb_to_scaled(waypoint, ref, scales));
    }

    Eigen::VectorXd tangent = scaled_path.back() - scaled_path.front();
    if (tangent.norm() <= 1e-12) {
        ++stats.degenerate_rejects;
        return candidates;
    }
    tangent.normalize();

    {
        Eigen::MatrixXd basis = Eigen::MatrixXd::Zero(dims, dims);
        basis.col(0) = tangent;
        const auto risk_order = obb_low_risk_joint_order(robot, dims);
        int col = 1;
        for (int joint : risk_order) {
            if (col >= dims) {
                break;
            }
            basis.col(col) = Eigen::VectorXd::Unit(dims, joint);
            ++col;
        }
        if (obb_orthonormalize_columns(basis)) {
            candidates.push_back(basis);
        }
    }
    if (primary_only) {
        stats.candidates += static_cast<int>(candidates.size());
        return candidates;
    }

    for (int preferred_axis = 0; preferred_axis < dims; ++preferred_axis) {
        Eigen::MatrixXd basis = Eigen::MatrixXd::Zero(dims, dims);
        basis.col(0) = tangent;
        int col = 1;
        basis.col(col++) = Eigen::VectorXd::Unit(dims, preferred_axis);
        for (int axis = 0; axis < dims && col < dims; ++axis) {
            if (axis == preferred_axis) {
                continue;
            }
            basis.col(col++) = Eigen::VectorXd::Unit(dims, axis);
        }
        if (obb_orthonormalize_columns(basis)) {
            candidates.push_back(basis);
        }
    }

    if (path.size() >= 3U) {
        Eigen::VectorXd mean = Eigen::VectorXd::Zero(dims);
        for (const auto& y : scaled_path) {
            mean += y;
        }
        mean /= static_cast<double>(scaled_path.size());
        Eigen::MatrixXd cov = Eigen::MatrixXd::Zero(dims, dims);
        for (const auto& y : scaled_path) {
            const Eigen::VectorXd d = y - mean;
            cov.noalias() += d * d.transpose();
        }
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(cov);
        if (solver.info() == Eigen::Success) {
            Eigen::MatrixXd basis = Eigen::MatrixXd::Zero(dims, dims);
            for (int col = 0; col < dims; ++col) {
                basis.col(col) = solver.eigenvectors().col(dims - 1 - col);
            }
            if (basis.col(0).dot(tangent) < 0.0) {
                basis.col(0) *= -1.0;
            }
            if (obb_orthonormalize_columns(basis)) {
                candidates.push_back(basis);
            }
        }
    }

    {
        Eigen::MatrixXd basis = Eigen::MatrixXd::Identity(dims, dims);
        candidates.push_back(basis);
    }
    stats.candidates += static_cast<int>(candidates.size());
    return candidates;
}

bool validate_obb_zonotope_candidate(const Robot& robot,
                                     const Scene& scene,
                                     const ObbPortalCandidate& candidate,
                                     double safety_epsilon,
                                     ObbPortalValidationStats& stats) {
    ++stats.validations;
    const bool clearance_first =
        detail::env_int_or_default("RBF_OBB_CLEARANCE_FIRST", 0) != 0;
    bool clearance_attempted = false;
    if (clearance_first) {
        clearance_attempted = true;
        if (validate_obb_clearance_sampled_candidate(robot,
                                                     scene,
                                                     candidate,
                                                     safety_epsilon,
                                                     stats)) {
            return true;
        }
    }
    const Eigen::MatrixXd compressed_generators =
        obb_compress_generator_columns(candidate.generators_q);
    stats.variables = std::max(stats.variables, static_cast<int>(compressed_generators.cols()));
    std::vector<ObbLinkZonotopeHull> links =
        obb_compute_link_zonotopes(robot,
                                   candidate.center_q,
                                   compressed_generators,
                                   compressed_generators.cols());
    stats.active_links = static_cast<int>(links.size());
    for (auto& link : links) {
        obb_compute_link_aabb(link, safety_epsilon);
    }

    const auto& obstacles = scene.obstacles();
    bool affine_maybe = false;
    for (const auto& obstacle : obstacles) {
        const float* bounds = obstacle.bounds;
        for (const auto& link : links) {
            if (!obb_zonotope_link_separates_obstacle(link, bounds, safety_epsilon, stats)) {
                affine_maybe = true;
                break;
            }
        }
        if (affine_maybe) {
            break;
        }
    }
    if (!affine_maybe) {
        return true;
    }
    if (!clearance_attempted &&
        validate_obb_clearance_sampled_candidate(robot,
                                                 scene,
                                                 candidate,
                                                 safety_epsilon,
                                                 stats)) {
        return true;
    }
    if (!obb_sampled_support_enabled()) {
        return false;
    }
    return validate_obb_sampled_support_candidate(robot,
                                                  scene,
                                                  candidate,
                                                  safety_epsilon,
                                                  stats);
}

bool obb_try_candidate_with_radii(const Robot& robot,
                                  const Scene& scene,
                                  const std::vector<Interval>& domain,
                                  const ObbPortalCandidate& base,
                                  const Eigen::VectorXd& radii_y,
                                  double safety_epsilon,
                                  ObbPortalCandidate& out,
                                  ObbPortalValidationStats& stats) {
    ObbPortalCandidate candidate;
    if (!obb_make_candidate_from_scaled(base.center_y,
                                        base.basis_y,
                                        radii_y,
                                        domain,
                                        candidate)) {
        ++stats.joint_limit_rejects;
        return false;
    }
    if (!validate_obb_zonotope_candidate(robot, scene, candidate, safety_epsilon, stats)) {
        return false;
    }
    out = std::move(candidate);
    ++stats.valid_candidates;
    return true;
}

ObbPortalCandidate obb_grow_candidate(const Robot& robot,
                                      const Scene& scene,
                                      const std::vector<Interval>& domain,
                                      const ObbPortalCandidate& seed,
                                      double safety_epsilon,
                                      int grow_iterations,
                                      int binary_iterations,
                                      int max_validations,
                                      ObbPortalValidationStats& stats) {
    ObbPortalCandidate good = seed;
    const int dims = static_cast<int>(seed.radii_y.size());
    const int grow_cap = std::max(0, grow_iterations);
    const int binary_cap = std::max(0, binary_iterations);
    constexpr double kGrow = 1.7;
    auto budget_exhausted = [&]() {
        return max_validations > 0 && stats.validations >= max_validations;
    };

    auto grow_radii = [&](const Eigen::VectorXd& base, int axis) {
        Eigen::VectorXd radii = base;
        if (axis < 0) {
            for (int col = 1; col < dims; ++col) {
                radii[col] *= kGrow;
            }
        } else {
            radii[axis] *= kGrow;
        }
        return radii;
    };

    auto refine_between = [&](const Eigen::VectorXd& lo, const Eigen::VectorXd& hi) {
        Eigen::VectorXd good_r = lo;
        Eigen::VectorXd bad_r = hi;
        for (int iter = 0; iter < binary_cap; ++iter) {
            if (budget_exhausted()) {
                break;
            }
            Eigen::VectorXd mid = 0.5 * (good_r + bad_r);
            ObbPortalCandidate mid_candidate;
            ++stats.grow_attempts;
            if (obb_try_candidate_with_radii(robot,
                                            scene,
                                            domain,
                                            seed,
                                            mid,
                                            safety_epsilon,
                                            mid_candidate,
                                            stats)) {
                good = std::move(mid_candidate);
                good_r = mid;
            } else {
                bad_r = mid;
            }
        }
    };

    Eigen::VectorXd current = good.radii_y;
    for (int iter = 0; iter < grow_cap; ++iter) {
        if (budget_exhausted()) {
            break;
        }
        const Eigen::VectorXd next = grow_radii(current, -1);
        ObbPortalCandidate next_candidate;
        ++stats.grow_attempts;
        if (obb_try_candidate_with_radii(robot,
                                        scene,
                                        domain,
                                        seed,
                                        next,
                                        safety_epsilon,
                                        next_candidate,
                                        stats)) {
            good = std::move(next_candidate);
            current = next;
        } else {
            refine_between(current, next);
            current = good.radii_y;
            break;
        }
    }

    for (int axis = 1; axis < dims; ++axis) {
        current = good.radii_y;
        for (int iter = 0; iter < grow_cap; ++iter) {
            if (budget_exhausted()) {
                break;
            }
            const Eigen::VectorXd next = grow_radii(current, axis);
            ObbPortalCandidate next_candidate;
            ++stats.grow_attempts;
            if (obb_try_candidate_with_radii(robot,
                                            scene,
                                            domain,
                                            seed,
                                            next,
                                            safety_epsilon,
                                            next_candidate,
                                            stats)) {
                good = std::move(next_candidate);
                current = next;
            } else {
                refine_between(current, next);
                break;
            }
        }
    }
    return good;
}

bool validate_obb_zonotope_portal(const Robot& robot,
                                  const Scene& scene,
                                  const std::vector<Interval>& domain,
                                  const std::vector<Eigen::VectorXd>& path,
                                  double lateral_radius,
                                  double longitudinal_margin,
                                  double safety_epsilon,
                                  int grow_iterations,
                                  int binary_iterations,
                                  int max_validations,
                                  ObbPortalValidationStats& stats,
                                  Eigen::VectorXd* out_center,
                                  Eigen::MatrixXd* out_generators) {
    if (path.size() < 2U || path.front().size() <= 0 || path.front().size() > MAX_JOINTS) {
        ++stats.degenerate_rejects;
        return false;
    }
    const int dims = static_cast<int>(path.front().size());
    if (static_cast<int>(domain.size()) != dims) {
        ++stats.degenerate_rejects;
        return false;
    }
    for (const auto& waypoint : path) {
        if (waypoint.size() != dims) {
            ++stats.degenerate_rejects;
            return false;
        }
    }

    const bool fast_primary_orientation =
        detail::env_flag_or_default("RBF_OBB_FAST_PRIMARY_ORIENTATION", true);
    const bool fallback_orientations_on_fail =
        detail::env_flag_or_default("RBF_OBB_FALLBACK_ORIENTATIONS_ON_PRIMARY_FAIL", false);
    const bool primary_only =
        fast_primary_orientation && !fallback_orientations_on_fail;
    const auto orientations = obb_orientation_candidates(robot,
                                                         path,
                                                         domain,
                                                         stats,
                                                         primary_only);
    bool found = false;
    ObbPortalCandidate best;

    auto try_orientation_range = [&](std::size_t begin, std::size_t end) {
        for (std::size_t index = begin; index < end && index < orientations.size(); ++index) {
            if (max_validations > 0 && stats.validations >= max_validations) {
                break;
            }
            const auto& basis_y = orientations[index];
            ObbPortalCandidate candidate;
            if (!obb_fit_scaled_path_with_basis(path,
                                                domain,
                                                basis_y,
                                                lateral_radius,
                                                longitudinal_margin,
                                                candidate,
                                                stats)) {
                continue;
            }
            if (!validate_obb_zonotope_candidate(robot, scene, candidate, safety_epsilon, stats)) {
                continue;
            }
            ++stats.valid_candidates;
            if (!found || candidate.score > best.score) {
                best = std::move(candidate);
                found = true;
            }
        }
    };

    const std::size_t primary_end =
        fast_primary_orientation ? std::min<std::size_t>(orientations.size(), 1U) : orientations.size();
    try_orientation_range(0U, primary_end);
    if (!found && fast_primary_orientation && fallback_orientations_on_fail &&
        primary_end < orientations.size()) {
        if (max_validations > 0 && stats.validations >= max_validations) {
            return false;
        }
        try_orientation_range(primary_end, orientations.size());
    }
    if (!found) {
        return false;
    }
    if (grow_iterations > 0 &&
        (max_validations <= 0 || stats.validations < max_validations)) {
        best = obb_grow_candidate(robot,
                                  scene,
                                  domain,
                                  best,
                                  safety_epsilon,
                                  grow_iterations,
                                  binary_iterations,
                                  max_validations,
                                  stats);
    }
    if (out_center != nullptr) {
        *out_center = best.center_q;
    }
    if (out_generators != nullptr) {
        *out_generators = best.generators_q;
    }
    stats.longitudinal_radius = best.radii_y.size() > 0 ? best.radii_y[0] : stats.longitudinal_radius;
    if (best.radii_y.size() > 1) {
        double max_lateral = 0.0;
        for (int col = 1; col < best.radii_y.size(); ++col) {
            max_lateral = std::max(max_lateral, best.radii_y[col]);
        }
        stats.lateral_radius = max_lateral;
    }
    return true;
}



double obb_generator_parallelotope_log_volume(const Eigen::MatrixXd& generators) {
    const int rows = generators.rows();
    const int cols = generators.cols();
    if (rows <= 0 || cols < rows) {
        return -std::numeric_limits<double>::infinity();
    }
    Eigen::MatrixXd gram = generators * generators.transpose();
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(gram, Eigen::EigenvaluesOnly);
    if (solver.info() != Eigen::Success) {
        return -std::numeric_limits<double>::infinity();
    }
    double log_volume = static_cast<double>(rows) * std::log(2.0);
    for (int dim = 0; dim < rows; ++dim) {
        const double eig = solver.eigenvalues()[dim];
        if (!(eig > 0.0) || !std::isfinite(eig)) {
            return -std::numeric_limits<double>::infinity();
        }
        log_volume += 0.5 * std::log(eig);
    }
    return log_volume;
}

double obb_generator_parallelotope_volume(const Eigen::MatrixXd& generators) {
    const double log_volume = obb_generator_parallelotope_log_volume(generators);
    if (!std::isfinite(log_volume)) {
        return 0.0;
    }
    if (log_volume > std::log(std::numeric_limits<double>::max())) {
        return std::numeric_limits<double>::infinity();
    }
    if (log_volume < std::log(std::numeric_limits<double>::min())) {
        return 0.0;
    }
    return std::exp(log_volume);
}

void obb_record_region_volume(ObbPortalValidationStats& stats,
                              const Eigen::MatrixXd& generators) {
    const double volume = obb_generator_parallelotope_volume(generators);
    const double log_volume = obb_generator_parallelotope_log_volume(generators);
    stats.region_volume_sum += volume;
    stats.region_volume_max = std::max(stats.region_volume_max, volume);
    if (std::isfinite(log_volume)) {
        stats.region_log_volume_sum += log_volume;
    }
    stats.region_volume_count += 1;
}

void obb_accumulate_stats(ObbPortalValidationStats& dst,
                          const ObbPortalValidationStats& src) {
    dst.joint_limit_rejects += src.joint_limit_rejects;
    dst.degenerate_rejects += src.degenerate_rejects;
    dst.candidates += src.candidates;
    dst.validations += src.validations;
    dst.valid_candidates += src.valid_candidates;
    dst.grow_attempts += src.grow_attempts;
    dst.aabb_tests += src.aabb_tests;
    dst.aabb_rejects += src.aabb_rejects;
    dst.gjk_tests += src.gjk_tests;
    dst.gjk_rejects += src.gjk_rejects;
    dst.gjk_iterations += src.gjk_iterations;
    dst.maybe_pairs += src.maybe_pairs;
    dst.sampled_support_attempts += src.sampled_support_attempts;
    dst.sampled_support_success += src.sampled_support_success;
    dst.sampled_support_fail += src.sampled_support_fail;
    dst.sampled_support_samples += src.sampled_support_samples;
    dst.clearance_support_attempts += src.clearance_support_attempts;
    dst.clearance_support_success += src.clearance_support_success;
    dst.clearance_support_fail += src.clearance_support_fail;
    dst.clearance_support_samples += src.clearance_support_samples;
    dst.active_links = std::max(dst.active_links, src.active_links);
    dst.variables = std::max(dst.variables, src.variables);
    dst.longitudinal_radius = std::max(dst.longitudinal_radius, src.longitudinal_radius);
    dst.lateral_radius = std::max(dst.lateral_radius, src.lateral_radius);
    dst.sampled_support_error_radius =
        std::max(dst.sampled_support_error_radius, src.sampled_support_error_radius);
    dst.clearance_support_error_radius =
        std::max(dst.clearance_support_error_radius, src.clearance_support_error_radius);
    dst.clearance_support_min_margin =
        std::min(dst.clearance_support_min_margin, src.clearance_support_min_margin);
    dst.region_volume_sum += src.region_volume_sum;
    dst.region_volume_max = std::max(dst.region_volume_max, src.region_volume_max);
    dst.region_log_volume_sum += src.region_log_volume_sum;
    dst.region_volume_count += src.region_volume_count;
}

std::vector<Eigen::VectorXd> obb_path_slice(const std::vector<Eigen::VectorXd>& path,
                                            std::size_t begin,
                                            std::size_t end) {
    std::vector<Eigen::VectorXd> out;
    if (path.empty() || begin >= path.size() || end >= path.size() || begin >= end) {
        return out;
    }
    out.reserve(end - begin + 1U);
    for (std::size_t index = begin; index <= end; ++index) {
        out.push_back(path[index]);
    }
    return out;
}

bool obb_validate_path_window(const Robot& robot,
                              const Scene& scene,
                              const std::vector<Interval>& domain,
                              const std::vector<Eigen::VectorXd>& path,
                              std::size_t begin,
                              std::size_t end,
                              double lateral_radius,
                              double longitudinal_margin,
                              double safety_epsilon,
                              int grow_iterations,
                              int binary_iterations,
                              int max_validations,
                              ObbPathCoverResult& result,
                              Eigen::VectorXd& center,
                              Eigen::MatrixXd& generators) {
    ++result.windows_attempted;
    std::vector<Eigen::VectorXd> window = obb_path_slice(path, begin, end);
    ObbPortalValidationStats local_stats;
    const bool ok = validate_obb_zonotope_portal(robot,
                                                 scene,
                                                 domain,
                                                 window,
                                                 lateral_radius,
                                                 longitudinal_margin,
                                                 safety_epsilon,
                                                 grow_iterations,
                                                 binary_iterations,
                                                 max_validations,
                                                 local_stats,
                                                 &center,
                                                 &generators);
    obb_accumulate_stats(result.stats, local_stats);
    if (ok) {
        ++result.windows_success;
    }
    return ok;
}

bool obb_cover_segment_recursive(const Robot& robot,
                                 const Scene& scene,
                                 const std::vector<Interval>& domain,
                                 const Eigen::VectorXd& a,
                                 const Eigen::VectorXd& b,
                                 int depth_remaining,
                                 double lateral_radius,
                                 double longitudinal_margin,
                                 double safety_epsilon,
                                 int grow_iterations,
                                 int binary_iterations,
                                 int max_validations,
                                 ObbPathCoverResult& result,
                                 std::vector<Eigen::VectorXd>& centerline) {
    std::vector<Eigen::VectorXd> segment_path{a, b};
    Eigen::VectorXd center;
    Eigen::MatrixXd generators;
    if (obb_validate_path_window(robot,
                                 scene,
                                 domain,
                                 segment_path,
                                 0,
                                 1,
                                 lateral_radius,
                                 longitudinal_margin,
                                 safety_epsilon,
                                 grow_iterations,
                                 binary_iterations,
                                 max_validations,
                                 result,
                                 center,
                                 generators)) {
        ObbPathCoverRegion region;
        region.begin = centerline.empty() ? 0U : centerline.size() - 1U;
        region.end = region.begin + 1U;
        region.center = std::move(center);
        region.generators = std::move(generators);
        obb_record_region_volume(result.stats, region.generators);
        result.regions.push_back(std::move(region));
        result.covered_length += (b - a).norm();
        if (centerline.empty()) {
            centerline.push_back(a);
        }
        if ((centerline.back() - b).norm() > 1e-12) {
            centerline.push_back(b);
        }
        return true;
    }
    if (depth_remaining <= 0) {
        std::vector<Eigen::VectorXd> segment_path{a, b};
        Eigen::VectorXd fallback_center;
        Eigen::MatrixXd fallback_generators;
        const int fallback_budget = max_validations > 0 ? std::max(max_validations, 16) : 16;
        if (obb_validate_path_window(robot,
                                     scene,
                                     domain,
                                     segment_path,
                                     0,
                                     1,
                                     0.0,
                                     0.0,
                                     safety_epsilon,
                                     0,
                                     0,
                                     fallback_budget,
                                     result,
                                     fallback_center,
                                     fallback_generators)) {
            Eigen::VectorXd best_center = fallback_center;
            Eigen::MatrixXd best_generators = fallback_generators;
            double lo = 0.0;
            double hi = std::max(0.0, lateral_radius);
            for (int iter = 0; iter < std::max(0, binary_iterations) && hi > 0.0; ++iter) {
                const double mid = 0.5 * (lo + hi);
                Eigen::VectorXd mid_center;
                Eigen::MatrixXd mid_generators;
                if (obb_validate_path_window(robot,
                                             scene,
                                             domain,
                                             segment_path,
                                             0,
                                             1,
                                             mid,
                                             0.0,
                                             safety_epsilon,
                                             0,
                                             0,
                                             fallback_budget,
                                             result,
                                             mid_center,
                                             mid_generators)) {
                    lo = mid;
                    best_center = std::move(mid_center);
                    best_generators = std::move(mid_generators);
                } else {
                    hi = mid;
                }
            }
            ObbPathCoverRegion region;
            region.begin = centerline.empty() ? 0U : centerline.size() - 1U;
            region.end = region.begin + 1U;
            region.center = std::move(best_center);
            region.generators = std::move(best_generators);
            obb_record_region_volume(result.stats, region.generators);
            result.regions.push_back(std::move(region));
            result.covered_length += (b - a).norm();
            if (centerline.empty()) {
                centerline.push_back(a);
            }
            if ((centerline.back() - b).norm() > 1e-12) {
                centerline.push_back(b);
            }
            return true;
        }
        ++result.failed_leaf_windows;
        const double failed_length = (b - a).norm();
        result.failed_leaf_length_sum += failed_length;
        result.failed_leaf_length_max = std::max(result.failed_leaf_length_max, failed_length);
        if (!result.has_first_failed_leaf) {
            result.has_first_failed_leaf = true;
            result.first_failed_leaf_a = a;
            result.first_failed_leaf_b = b;
        }
        if (centerline.empty()) {
            centerline.push_back(a);
        }
        if ((centerline.back() - b).norm() > 1e-12) {
            centerline.push_back(b);
        }
        return false;
    }
    ++result.recursive_splits;
    const Eigen::VectorXd mid = 0.5 * (a + b);
    const bool left_ok = obb_cover_segment_recursive(robot,
                                                     scene,
                                                     domain,
                                                     a,
                                                     mid,
                                                     depth_remaining - 1,
                                                     lateral_radius,
                                                     longitudinal_margin,
                                                     safety_epsilon,
                                                     grow_iterations,
                                                     binary_iterations,
                                                     max_validations,
                                                     result,
                                                     centerline);
    const bool right_ok = obb_cover_segment_recursive(robot,
                                                      scene,
                                                      domain,
                                                      mid,
                                                      b,
                                                      depth_remaining - 1,
                                                      lateral_radius,
                                                      longitudinal_margin,
                                                      safety_epsilon,
                                                      grow_iterations,
                                                      binary_iterations,
                                                      max_validations,
                                                      result,
                                                      centerline);
    return left_ok && right_ok;
}

ObbPathCoverResult cover_segment_or_bridge_path_with_obbs(
    const Robot& robot,
    const Scene& scene,
    const std::vector<Interval>& domain,
    const std::vector<Eigen::VectorXd>& path,
    bool greedy_bridge_cover,
    int segment_split_depth,
    int max_window_segments,
    double lateral_radius,
    double longitudinal_margin,
    double safety_epsilon,
    int grow_iterations,
    int binary_iterations,
    int max_validations,
    std::vector<Eigen::VectorXd>& out_centerline) {
    ObbPathCoverResult result;
    out_centerline.clear();
    if (path.size() < 2U) {
        ++result.stats.degenerate_rejects;
        return result;
    }
    if (!greedy_bridge_cover || path.size() == 2U) {
        result.success = obb_cover_segment_recursive(robot,
                                                     scene,
                                                     domain,
                                                     path.front(),
                                                     path.back(),
                                                     std::max(0, segment_split_depth),
                                                     lateral_radius,
                                                     longitudinal_margin,
                                                     safety_epsilon,
                                                     grow_iterations,
                                                     binary_iterations,
                                                     max_validations,
                                                     result,
                                                     out_centerline);
        return result;
    }

    out_centerline.push_back(path.front());
    const std::size_t last = path.size() - 1U;
    const int window_cap = std::max(1, max_window_segments);
    std::size_t begin = 0;
    while (begin < last) {
        const std::size_t max_end =
            std::min(last, begin + static_cast<std::size_t>(window_cap));
        std::size_t good_end = begin;
        Eigen::VectorXd good_center;
        Eigen::MatrixXd good_generators;
        std::size_t step = 1;
        std::size_t first_fail = 0;
        while (begin + step <= max_end) {
            Eigen::VectorXd center;
            Eigen::MatrixXd generators;
            const std::size_t end = begin + step;
            if (obb_validate_path_window(robot,
                                         scene,
                                         domain,
                                         path,
                                         begin,
                                         end,
                                         lateral_radius,
                                         longitudinal_margin,
                                         safety_epsilon,
                                         grow_iterations,
                                         binary_iterations,
                                         max_validations,
                                         result,
                                         center,
                                         generators)) {
                good_end = end;
                good_center = std::move(center);
                good_generators = std::move(generators);
                step *= 2U;
            } else {
                first_fail = end;
                break;
            }
        }
        if (good_end == max_end) {
            first_fail = 0;
        } else if (first_fail == 0 && begin + step > max_end) {
            first_fail = max_end + 1U;
        }
        if (first_fail > good_end + 1U && good_end > begin) {
            std::size_t lo = good_end + 1U;
            std::size_t hi = std::min(first_fail - 1U, max_end);
            while (lo <= hi) {
                const std::size_t mid = lo + (hi - lo) / 2U;
                Eigen::VectorXd center;
                Eigen::MatrixXd generators;
                if (obb_validate_path_window(robot,
                                             scene,
                                             domain,
                                             path,
                                             begin,
                                             mid,
                                             lateral_radius,
                                             longitudinal_margin,
                                             safety_epsilon,
                                             grow_iterations,
                                             binary_iterations,
                                             max_validations,
                                             result,
                                             center,
                                             generators)) {
                    good_end = mid;
                    good_center = std::move(center);
                    good_generators = std::move(generators);
                    lo = mid + 1U;
                } else {
                    if (mid == 0U) {
                        break;
                    }
                    hi = mid - 1U;
                }
            }
        }
        if (good_end <= begin) {
            std::vector<Eigen::VectorXd> split_line;
            const bool split_ok = obb_cover_segment_recursive(robot,
                                                              scene,
                                                              domain,
                                                              path[begin],
                                                              path[begin + 1U],
                                                              std::max(0, segment_split_depth),
                                                              lateral_radius,
                                                              longitudinal_margin,
                                                              safety_epsilon,
                                                              grow_iterations,
                                                              binary_iterations,
                                                              max_validations,
                                                              result,
                                                              split_line);
            for (const auto& waypoint : split_line) {
                if (out_centerline.empty() || (out_centerline.back() - waypoint).norm() > 1e-12) {
                    out_centerline.push_back(waypoint);
                }
            }
            if (!split_ok) {
                for (std::size_t index = begin + 1U; index <= last; ++index) {
                    if ((out_centerline.back() - path[index]).norm() > 1e-12) {
                        out_centerline.push_back(path[index]);
                    }
                }
                result.success = false;
                return result;
            }
            begin += 1U;
            continue;
        }
        ObbPathCoverRegion region;
        region.begin = begin;
        region.end = good_end;
        region.center = std::move(good_center);
        region.generators = std::move(good_generators);
        obb_record_region_volume(result.stats, region.generators);
        result.regions.push_back(std::move(region));
        for (std::size_t index = begin + 1U; index <= good_end; ++index) {
            result.covered_length += (path[index] - path[index - 1U]).norm();
        }
        if ((out_centerline.back() - path[good_end]).norm() > 1e-12) {
            out_centerline.push_back(path[good_end]);
        }
        begin = good_end;
    }
    result.success = !result.regions.empty() &&
                     !out_centerline.empty() &&
                     (out_centerline.back() - path.back()).norm() <= 1e-10;
    return result;
}


} // namespace rbf
