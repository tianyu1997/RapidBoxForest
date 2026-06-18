#include "planning_forest_query_bridge_detour_utils.h"

#include "env_config.h"
#include "planning_forest_audit.h"
#include "planning_forest_query_bridge_batch_utils.h"
#include "planning_forest_query_utils.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <random>

namespace rbf {

QueryBridgeDetourOptions query_bridge_detour_options_from_env() {
    QueryBridgeDetourOptions options;
    options.enabled =
        detail::env_int_or_default("RBF_QUERY_BRIDGE_DETOUR_ON_NO_PATH", 0) != 0;
    options.candidate =
        detail::env_int_or_default("RBF_QUERY_BRIDGE_DETOUR_CANDIDATE", 0) != 0;
    options.replace_factor = std::max(
        0.0,
        detail::env_double_or_default("RBF_QUERY_BRIDGE_DETOUR_REPLACE_FACTOR", 1.0));
    options.dims =
        std::max(0, detail::env_int_or_default("RBF_QUERY_BRIDGE_DETOUR_DIMS", 4));
    options.rounds =
        std::max(1, detail::env_int_or_default("RBF_QUERY_BRIDGE_DETOUR_ROUNDS", 2));
    options.max_candidates =
        std::max(1, detail::env_int_or_default("RBF_QUERY_BRIDGE_DETOUR_MAX_CANDIDATES", 32));
    options.multi_axis =
        detail::env_int_or_default("RBF_QUERY_BRIDGE_DETOUR_MULTI_AXIS", 0) != 0;
    options.random_candidates =
        std::max(0, detail::env_int_or_default("RBF_QUERY_BRIDGE_DETOUR_RANDOM_CANDIDATES", 0));
    options.offset = std::max(
        1e-4,
        detail::env_double_or_default("RBF_QUERY_BRIDGE_DETOUR_OFFSET", 0.35));
    options.two_bend_alpha = std::min(
        0.45,
        std::max(0.15,
                 detail::env_double_or_default(
                     "RBF_QUERY_BRIDGE_DETOUR_TWO_BEND_ALPHA",
                     0.35)));
    return options;
}

void record_query_bridge_detour_diagnostics(StageContext& context,
                                            const QueryBridgeDetourOptions& options) {
    context.diagnostics().set_value("query_bridge.detour_on_no_path",
                                    options.enabled ? 1.0 : 0.0);
    context.diagnostics().set_value("query_bridge.detour_candidate",
                                    options.candidate ? 1.0 : 0.0);
}

std::vector<Eigen::VectorXd> query_bridge_deterministic_detour_fallback_path(
    const QueryBridgeSearchTask& task,
    const Robot& audit_robot,
    const Scene& scene,
    const QueryConfig& query_config,
    const std::vector<Interval>& planning_domain,
    const QueryBridgeDetourOptions& options,
    int rng_seed_base,
    StageContext& context) {
    if (!options.enabled ||
        task.start.size() != task.goal.size() ||
        task.start.size() <= 0 ||
        static_cast<int>(planning_domain.size()) != task.start.size()) {
        return {};
    }
    CollisionChecker checker = make_audit_checker(audit_robot, scene, query_config);
    const Eigen::VectorXd delta = task.goal - task.start;
    const double direct_length = delta.norm();
    if (direct_length <= 1e-9) {
        return {};
    }
    std::vector<int> dims(static_cast<std::size_t>(task.start.size()));
    std::iota(dims.begin(), dims.end(), 0);
    std::sort(dims.begin(), dims.end(), [&](int lhs, int rhs) {
        const double lhs_width = std::max(1e-9, planning_domain[static_cast<std::size_t>(lhs)].width());
        const double rhs_width = std::max(1e-9, planning_domain[static_cast<std::size_t>(rhs)].width());
        const double lhs_along = std::abs(delta[lhs]) / lhs_width;
        const double rhs_along = std::abs(delta[rhs]) / rhs_width;
        if (std::abs(lhs_along - rhs_along) > 1e-12) {
            return lhs_along < rhs_along;
        }
        return lhs < rhs;
    });
    const int dim_limit = std::min<int>(options.dims, static_cast<int>(dims.size()));
    const int rounds = options.rounds;
    const int max_candidates = options.max_candidates;
    const bool multi_axis_detour = options.multi_axis;
    const int random_candidates = options.random_candidates;
    const double base_offset = options.offset;
    const double two_bend_alpha = options.two_bend_alpha;
    const Eigen::VectorXd mid = 0.5 * (task.start + task.goal);
    double best_length = std::numeric_limits<double>::infinity();
    std::vector<Eigen::VectorXd> best_path;
    int candidates = 0;
    auto clamp_to_domain = [&](Eigen::VectorXd point) {
        for (int dim = 0; dim < point.size(); ++dim) {
            point[dim] = std::min(planning_domain[static_cast<std::size_t>(dim)].hi,
                                  std::max(planning_domain[static_cast<std::size_t>(dim)].lo,
                                           point[dim]));
        }
        return point;
    };
    auto try_path = [&](std::vector<Eigen::VectorXd> path) {
        if (candidates >= max_candidates) {
            return;
        }
        ++candidates;
        context.diagnostics().add_counter("query_bridge.detour_on_no_path_candidates");
        double length = path_length(path);
        if (!std::isfinite(length) || length + 1e-12 >= best_length) {
            return;
        }
        const PathAuditCheck audit =
            audit_waypoint_path(path,
                                checker,
                                query_config.audit_resolution,
                                query_config.audit_segment_step);
        if (!audit.passed) {
            context.diagnostics().add_counter("query_bridge.detour_on_no_path_rejects");
            return;
        }
        best_length = length;
        best_path = std::move(path);
    };
    for (int item = 0; item < dim_limit && candidates < max_candidates; ++item) {
        const int dim = dims[static_cast<std::size_t>(item)];
        const double width = planning_domain[static_cast<std::size_t>(dim)].width();
        for (int round = 1; round <= rounds && candidates < max_candidates; ++round) {
            const double magnitude = std::min(0.45 * std::max(0.0, width),
                                              base_offset * static_cast<double>(round));
            if (magnitude <= 1e-9) {
                continue;
            }
            for (double sign : {1.0, -1.0}) {
                Eigen::VectorXd single = mid;
                single[dim] += sign * magnitude;
                single = clamp_to_domain(std::move(single));
                if ((single - mid).norm() > 1e-9) {
                    try_path({task.start, single, task.goal});
                }
                if (candidates >= max_candidates) {
                    break;
                }
                Eigen::VectorXd first = task.start + two_bend_alpha * delta;
                Eigen::VectorXd second = task.start + (1.0 - two_bend_alpha) * delta;
                first[dim] += sign * magnitude;
                second[dim] += sign * magnitude;
                first = clamp_to_domain(std::move(first));
                second = clamp_to_domain(std::move(second));
                if ((first - (task.start + two_bend_alpha * delta)).norm() > 1e-9 ||
                    (second - (task.start + (1.0 - two_bend_alpha) * delta)).norm() > 1e-9) {
                    try_path({task.start, first, second, task.goal});
                }
                if (candidates >= max_candidates) {
                    break;
                }
            }
        }
    }
    if (multi_axis_detour && dim_limit >= 2) {
        for (int first_item = 0; first_item < dim_limit && candidates < max_candidates; ++first_item) {
            const int first_dim = dims[static_cast<std::size_t>(first_item)];
            const double first_width = planning_domain[static_cast<std::size_t>(first_dim)].width();
            for (int second_item = first_item + 1;
                 second_item < dim_limit && candidates < max_candidates;
                 ++second_item) {
                const int second_dim = dims[static_cast<std::size_t>(second_item)];
                const double second_width = planning_domain[static_cast<std::size_t>(second_dim)].width();
                for (int round = 1; round <= rounds && candidates < max_candidates; ++round) {
                    const double first_mag = std::min(0.35 * std::max(0.0, first_width),
                                                      base_offset * static_cast<double>(round));
                    const double second_mag = std::min(0.35 * std::max(0.0, second_width),
                                                       base_offset * static_cast<double>(round));
                    if (first_mag <= 1e-9 || second_mag <= 1e-9) {
                        continue;
                    }
                    for (double first_sign : {1.0, -1.0}) {
                        for (double second_sign : {1.0, -1.0}) {
                            Eigen::VectorXd single = mid;
                            single[first_dim] += first_sign * first_mag;
                            single[second_dim] += second_sign * second_mag;
                            single = clamp_to_domain(std::move(single));
                            if ((single - mid).norm() > 1e-9) {
                                try_path({task.start, single, task.goal});
                            }
                            if (candidates >= max_candidates) {
                                break;
                            }
                            Eigen::VectorXd first = task.start + two_bend_alpha * delta;
                            Eigen::VectorXd second = task.start + (1.0 - two_bend_alpha) * delta;
                            first[first_dim] += first_sign * first_mag;
                            first[second_dim] += second_sign * second_mag;
                            second[first_dim] += first_sign * first_mag;
                            second[second_dim] += second_sign * second_mag;
                            first = clamp_to_domain(std::move(first));
                            second = clamp_to_domain(std::move(second));
                            try_path({task.start, first, second, task.goal});
                            if (candidates >= max_candidates) {
                                break;
                            }
                        }
                        if (candidates >= max_candidates) {
                            break;
                        }
                    }
                }
            }
        }
    }
    if (random_candidates > 0 && dim_limit > 0 && candidates < max_candidates) {
        const int random_budget = std::min(random_candidates, max_candidates - candidates);
        std::mt19937 rng(static_cast<std::uint32_t>(
            derived_planner_seed(rng_seed_base,
                                 kSeedBatchBridgeOffset,
                                 static_cast<int>(task.index),
                                 task.query_index,
                                 41443)));
        std::uniform_int_distribution<int> dim_pick(0, dim_limit - 1);
        std::uniform_real_distribution<double> unit(-1.0, 1.0);
        const double max_scale = std::max(1.0, static_cast<double>(rounds));
        for (int sample = 0; sample < random_budget && candidates < max_candidates; ++sample) {
            const int first_dim = dims[static_cast<std::size_t>(dim_pick(rng))];
            int second_dim = first_dim;
            if (dim_limit > 1) {
                for (int guard = 0; guard < 4 && second_dim == first_dim; ++guard) {
                    second_dim = dims[static_cast<std::size_t>(dim_pick(rng))];
                }
            }
            Eigen::VectorXd offset = Eigen::VectorXd::Zero(task.start.size());
            auto apply_random_dim = [&](int dim) {
                const double width = planning_domain[static_cast<std::size_t>(dim)].width();
                const double limit = std::min(0.35 * std::max(0.0, width),
                                              base_offset * max_scale);
                if (limit > 1e-9) {
                    offset[dim] += unit(rng) * limit;
                }
            };
            apply_random_dim(first_dim);
            if (second_dim != first_dim) {
                apply_random_dim(second_dim);
            }
            if (offset.norm() <= 1e-9) {
                continue;
            }
            if ((sample & 1) == 0) {
                Eigen::VectorXd single = clamp_to_domain(mid + offset);
                try_path({task.start, single, task.goal});
            } else {
                Eigen::VectorXd first = clamp_to_domain(task.start + two_bend_alpha * delta + offset);
                Eigen::VectorXd second = clamp_to_domain(task.start + (1.0 - two_bend_alpha) * delta + offset);
                try_path({task.start, first, second, task.goal});
            }
        }
    }
    context.diagnostics().add_counter("query_bridge.detour_on_no_path_attempts");
    if (!best_path.empty()) {
        context.diagnostics().add_counter("query_bridge.detour_on_no_path_successes");
    }
    return best_path;
}

bool query_bridge_maybe_apply_detour_path(
    const QueryBridgeSearchTask& task,
    const Robot& audit_robot,
    const Scene& scene,
    const QueryConfig& query_config,
    const std::vector<Interval>& planning_domain,
    const QueryBridgeDetourOptions& options,
    int rng_seed_base,
    StageContext& context,
    double& best_length,
    std::vector<Eigen::VectorXd>& waypoint_path) {
    if (!waypoint_path.empty() && !options.candidate) {
        return false;
    }
    auto detour_path = query_bridge_deterministic_detour_fallback_path(
        task,
        audit_robot,
        scene,
        query_config,
        planning_domain,
        options,
        rng_seed_base,
        context);
    if (detour_path.empty()) {
        return false;
    }
    const double detour_length = path_length(detour_path);
    if (!waypoint_path.empty() &&
        detour_length > best_length * options.replace_factor + 1e-12) {
        context.diagnostics().add_counter(
            "query_bridge.detour_candidate_not_shorter");
        return false;
    }
    best_length = detour_length;
    waypoint_path = std::move(detour_path);
    context.diagnostics().add_counter(
        "query_bridge.detour_candidate_selected");
    return true;
}

}  // namespace rbf
