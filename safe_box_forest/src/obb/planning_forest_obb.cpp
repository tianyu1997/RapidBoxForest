#include "planning_forest_obb.h"

#include "planning_forest_obb_candidate.h"
#include "planning_forest_obb_geometry.h"
#include "planning_forest_obb_sampled.h"
#include "planning_forest_obb_zonotope.h"

#include <algorithm>
#include <utility>
#include <vector>

namespace rbf {

ObbValidationOptions obb_validation_options_from_config(const AdaptiveLeafSweepConfig& config) {
    ObbValidationOptions options;
    options.fast_primary_orientation = config.obb_fast_primary_orientation;
    options.fallback_orientations_on_primary_fail =
        config.obb_fallback_orientations_on_primary_fail;
    options.sampled_support_enabled = config.obb_sampled_support_enabled;
    options.clearance_sampled_support_enabled =
        config.obb_clearance_sampled_support_enabled;
    options.clearance_lateral_l1_max = config.obb_clearance_lateral_l1_max;
    options.clearance_samples = config.obb_clearance_samples;
    options.clearance_dense_line_l1_threshold =
        config.obb_clearance_dense_line_l1_threshold;
    options.clearance_dense_samples = config.obb_clearance_dense_samples;
    options.clearance_fast_samples = config.obb_clearance_fast_samples;
    options.clearance_first = config.obb_clearance_first;
    options.clearance_retry_attempts = config.obb_clearance_retry_attempts;
    options.clearance_retry_values = config.obb_clearance_retry_values;
    options.clearance_retry_iters = config.obb_clearance_retry_iters;
    options.clearance_retry_timeout_ms = config.obb_clearance_retry_timeout_ms;
    return options;
}

bool obb_affine_zonotope_candidate_separates_scene(
    const Robot& robot,
    const Scene& scene,
    const ObbPortalCandidate& candidate,
    double safety_epsilon,
    ObbPortalValidationStats& stats) {
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
    for (const auto& obstacle : obstacles) {
        const float* bounds = obstacle.bounds;
        for (const auto& link : links) {
            if (!obb_zonotope_link_separates_obstacle(link, bounds, safety_epsilon, stats)) {
                return false;
            }
        }
    }
    return true;
}

bool validate_obb_zonotope_candidate(const Robot& robot,
                                     const Scene& scene,
                                     const ObbPortalCandidate& candidate,
                                     double safety_epsilon,
                                     ObbPortalValidationStats& stats,
                                     const ObbValidationOptions& options) {
    ++stats.validations;
    const bool clearance_first = options.clearance_first;
    bool clearance_attempted = false;
    if (clearance_first) {
        clearance_attempted = true;
        if (validate_obb_clearance_sampled_candidate(robot,
                                                     scene,
                                                     candidate,
                                                     safety_epsilon,
                                                     stats,
                                                     options)) {
            return true;
        }
    }
    if (obb_affine_zonotope_candidate_separates_scene(robot,
                                                      scene,
                                                      candidate,
                                                      safety_epsilon,
                                                      stats)) {
        return true;
    }
    if (!clearance_attempted &&
        validate_obb_clearance_sampled_candidate(robot,
                                                 scene,
                                                 candidate,
                                                 safety_epsilon,
                                                 stats,
                                                 options)) {
        return true;
    }
    if (!options.sampled_support_enabled) {
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
                                  ObbPortalValidationStats& stats,
                                  const ObbValidationOptions& options) {
    ObbPortalCandidate candidate;
    if (!obb_make_candidate_from_scaled(base.center_y,
                                        base.basis_y,
                                        radii_y,
                                        domain,
                                        candidate)) {
        ++stats.joint_limit_rejects;
        return false;
    }
    if (!validate_obb_zonotope_candidate(robot, scene, candidate, safety_epsilon, stats, options)) {
        return false;
    }
    out = std::move(candidate);
    ++stats.valid_candidates;
    return true;
}

bool obb_validation_budget_exhausted(const ObbPortalValidationStats& stats,
                                     int max_validations) {
    return max_validations > 0 && stats.validations >= max_validations;
}

Eigen::VectorXd obb_grown_radii(const Eigen::VectorXd& base,
                                int axis,
                                double growth_factor) {
    Eigen::VectorXd radii = base;
    if (axis < 0) {
        for (int col = 1; col < radii.size(); ++col) {
            radii[col] *= growth_factor;
        }
    } else if (axis < radii.size()) {
        radii[axis] *= growth_factor;
    }
    return radii;
}

void obb_refine_candidate_radii_between(const Robot& robot,
                                        const Scene& scene,
                                        const std::vector<Interval>& domain,
                                        const ObbPortalCandidate& seed,
                                        const Eigen::VectorXd& lo,
                                        const Eigen::VectorXd& hi,
                                        double safety_epsilon,
                                        int binary_iterations,
                                        int max_validations,
                                        ObbPortalValidationStats& stats,
                                        const ObbValidationOptions& options,
                                        ObbPortalCandidate& good) {
    Eigen::VectorXd good_r = lo;
    Eigen::VectorXd bad_r = hi;
    for (int iter = 0; iter < std::max(0, binary_iterations); ++iter) {
        if (obb_validation_budget_exhausted(stats, max_validations)) {
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
                                         stats,
                                         options)) {
            good = std::move(mid_candidate);
            good_r = mid;
        } else {
            bad_r = mid;
        }
    }
}

void obb_grow_candidate_along_axis(const Robot& robot,
                                   const Scene& scene,
                                   const std::vector<Interval>& domain,
                                   const ObbPortalCandidate& seed,
                                   int axis,
                                   int grow_iterations,
                                   double growth_factor,
                                   double safety_epsilon,
                                   int binary_iterations,
                                   int max_validations,
                                   ObbPortalValidationStats& stats,
                                   const ObbValidationOptions& options,
                                   ObbPortalCandidate& good) {
    Eigen::VectorXd current = good.radii_y;
    for (int iter = 0; iter < std::max(0, grow_iterations); ++iter) {
        if (obb_validation_budget_exhausted(stats, max_validations)) {
            break;
        }
        const Eigen::VectorXd next = obb_grown_radii(current, axis, growth_factor);
        ObbPortalCandidate next_candidate;
        ++stats.grow_attempts;
        if (obb_try_candidate_with_radii(robot,
                                         scene,
                                         domain,
                                         seed,
                                         next,
                                         safety_epsilon,
                                         next_candidate,
                                         stats,
                                         options)) {
            good = std::move(next_candidate);
            current = next;
        } else {
            obb_refine_candidate_radii_between(robot,
                                               scene,
                                               domain,
                                               seed,
                                               current,
                                               next,
                                               safety_epsilon,
                                               binary_iterations,
                                               max_validations,
                                               stats,
                                               options,
                                               good);
            break;
        }
    }
}

ObbPortalCandidate obb_grow_candidate(const Robot& robot,
                                      const Scene& scene,
                                      const std::vector<Interval>& domain,
                                      const ObbPortalCandidate& seed,
                                      double safety_epsilon,
                                      int grow_iterations,
                                      int binary_iterations,
                                      int max_validations,
                                      ObbPortalValidationStats& stats,
                                      const ObbValidationOptions& options) {
    ObbPortalCandidate good = seed;
    const int dims = static_cast<int>(seed.radii_y.size());
    const int grow_cap = std::max(0, grow_iterations);
    constexpr double kGrow = 1.7;

    obb_grow_candidate_along_axis(robot,
                                  scene,
                                  domain,
                                  seed,
                                  -1,
                                  grow_cap,
                                  kGrow,
                                  safety_epsilon,
                                  binary_iterations,
                                  max_validations,
                                  stats,
                                  options,
                                  good);

    for (int axis = 1; axis < dims; ++axis) {
        obb_grow_candidate_along_axis(robot,
                                      scene,
                                      domain,
                                      seed,
                                      axis,
                                      grow_cap,
                                      kGrow,
                                      safety_epsilon,
                                      binary_iterations,
                                      max_validations,
                                      stats,
                                      options,
                                      good);
    }
    return good;
}

void obb_try_orientation_candidate_range(
    const Robot& robot,
    const Scene& scene,
    const std::vector<Interval>& domain,
    const std::vector<Eigen::VectorXd>& path,
    const std::vector<Eigen::MatrixXd>& orientations,
    std::size_t begin,
    std::size_t end,
    double lateral_radius,
    double longitudinal_margin,
    double safety_epsilon,
    int max_validations,
    ObbPortalValidationStats& stats,
    const ObbValidationOptions& options,
    bool& found,
    ObbPortalCandidate& best) {
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
        if (!validate_obb_zonotope_candidate(robot,
                                             scene,
                                             candidate,
                                             safety_epsilon,
                                             stats,
                                             options)) {
            continue;
        }
        ++stats.valid_candidates;
        if (!found || candidate.score > best.score) {
            best = std::move(candidate);
            found = true;
        }
    }
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
                                  Eigen::MatrixXd* out_generators,
                                  ObbValidationOptions options) {
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

    const bool fast_primary_orientation = options.fast_primary_orientation;
    const bool fallback_orientations_on_fail =
        options.fallback_orientations_on_primary_fail;
    const bool primary_only =
        fast_primary_orientation && !fallback_orientations_on_fail;
    const auto orientations = obb_orientation_candidates(robot,
                                                         path,
                                                         domain,
                                                         stats,
                                                         primary_only);
    bool found = false;
    ObbPortalCandidate best;

    const std::size_t primary_end =
        fast_primary_orientation ? std::min<std::size_t>(orientations.size(), 1U) : orientations.size();
    obb_try_orientation_candidate_range(robot,
                                        scene,
                                        domain,
                                        path,
                                        orientations,
                                        0U,
                                        primary_end,
                                        lateral_radius,
                                        longitudinal_margin,
                                        safety_epsilon,
                                        max_validations,
                                        stats,
                                        options,
                                        found,
                                        best);
    if (!found && fast_primary_orientation && fallback_orientations_on_fail &&
        primary_end < orientations.size()) {
        if (max_validations > 0 && stats.validations >= max_validations) {
            return false;
        }
        obb_try_orientation_candidate_range(robot,
                                            scene,
                                            domain,
                                            path,
                                            orientations,
                                            primary_end,
                                            orientations.size(),
                                            lateral_radius,
                                            longitudinal_margin,
                                            safety_epsilon,
                                            max_validations,
                                            stats,
                                            options,
                                            found,
                                            best);
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
                                  stats,
                                  options);
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


} // namespace rbf
