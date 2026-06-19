#include "planning_forest_obb_sampled.h"

#include <sbf/envelope/envelope_collision.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace rbf {

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

bool validate_obb_clearance_sampled_candidate(const Robot& robot,
                                              const Scene& scene,
                                              const ObbPortalCandidate& candidate,
                                              double safety_epsilon,
                                              ObbPortalValidationStats& stats,
                                              const ObbValidationOptions& options) {
    if (!options.clearance_sampled_support_enabled) {
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
    // The pointwise clearance proof is designed for thin bridge tubes. Large
    // transverse OBBs should use the affine support-hull validator instead.
    if (lateral_l1 > options.clearance_lateral_l1_max) {
        ++stats.clearance_support_fail;
        return false;
    }

    const double line_l1 = candidate.generators_q.col(line_col).lpNorm<1>();
    int samples = options.clearance_samples;
    const double dense_line_l1_threshold = options.clearance_dense_line_l1_threshold;
    const int dense_samples = options.clearance_dense_samples;
    if (dense_line_l1_threshold > 0.0 &&
        line_l1 > dense_line_l1_threshold) {
        samples = std::max(samples, dense_samples);
    }
    samples = std::clamp(samples, 9, 257);
    const int fast_samples = std::clamp(
        options.clearance_fast_samples,
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

}  // namespace rbf
