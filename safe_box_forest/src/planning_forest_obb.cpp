#include "planning_forest_obb.h"

#include "planning_forest_obb_candidate.h"
#include "planning_forest_obb_options.h"
#include "planning_forest_obb_sampled.h"
#include "planning_forest_obb_zonotope.h"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace rbf {

bool obb_sampled_support_enabled() {
    return obb_sampled_support_enabled_from_env();
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
    const bool clearance_first = obb_clearance_first_from_env();
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

    const bool fast_primary_orientation = obb_fast_primary_orientation_from_env();
    const bool fallback_orientations_on_fail =
        obb_fallback_orientations_on_primary_fail_from_env();
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
