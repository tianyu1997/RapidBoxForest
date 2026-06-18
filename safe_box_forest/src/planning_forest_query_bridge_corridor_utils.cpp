#include "planning_forest_query_bridge_corridor_utils.h"

#include "env_config.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rbf {

QueryBridgeLocalDsu::QueryBridgeLocalDsu(std::size_t count) : parent(count) {
    for (std::size_t index = 0; index < parent.size(); ++index) {
        parent[index] = static_cast<int>(index);
    }
}

int QueryBridgeLocalDsu::add() {
    const int id = static_cast<int>(parent.size());
    parent.push_back(id);
    return id;
}

int QueryBridgeLocalDsu::find(int value) {
    int root = value;
    while (parent[static_cast<std::size_t>(root)] != root) {
        root = parent[static_cast<std::size_t>(root)];
    }
    while (parent[static_cast<std::size_t>(value)] != value) {
        const int next = parent[static_cast<std::size_t>(value)];
        parent[static_cast<std::size_t>(value)] = root;
        value = next;
    }
    return root;
}

void QueryBridgeLocalDsu::unite(int lhs, int rhs) {
    if (lhs < 0 || rhs < 0 ||
        lhs >= static_cast<int>(parent.size()) ||
        rhs >= static_cast<int>(parent.size())) {
        return;
    }
    const int left = find(lhs);
    const int right = find(rhs);
    if (left != right) {
        parent[static_cast<std::size_t>(right)] = left;
    }
}

double query_bridge_seed_path_param(const std::vector<Eigen::VectorXd>& samples,
                                    const Eigen::VectorXd& seed,
                                    int transition_hint) {
    if (samples.empty()) {
        return 0.0;
    }
    if (transition_hint >= 0 &&
        transition_hint + 1 < static_cast<int>(samples.size())) {
        const Eigen::VectorXd& a = samples[static_cast<std::size_t>(transition_hint)];
        const Eigen::VectorXd& b = samples[static_cast<std::size_t>(transition_hint + 1)];
        const Eigen::VectorXd delta = b - a;
        const double denom = delta.squaredNorm();
        double u = 0.5;
        if (denom > 1e-18) {
            u = (seed - a).dot(delta) / denom;
            u = std::min(1.0, std::max(0.0, u));
        }
        return static_cast<double>(transition_hint) + u;
    }
    double best_distance = std::numeric_limits<double>::infinity();
    std::size_t best_index = 0;
    for (std::size_t index = 0; index < samples.size(); ++index) {
        const double distance = (seed - samples[index]).squaredNorm();
        if (distance < best_distance) {
            best_distance = distance;
            best_index = index;
        }
    }
    return static_cast<double>(best_index);
}

double query_bridge_transition_length(const std::vector<Eigen::VectorXd>& samples,
                                      int transition) {
    if (transition < 0 ||
        transition + 1 >= static_cast<int>(samples.size())) {
        return 0.0;
    }
    return (samples[static_cast<std::size_t>(transition + 1)] -
            samples[static_cast<std::size_t>(transition)]).norm();
}

double query_bridge_transition_length_sum(const std::vector<Eigen::VectorXd>& samples,
                                          const std::vector<int>& transitions) {
    double total = 0.0;
    for (int transition : transitions) {
        total += query_bridge_transition_length(samples, transition);
    }
    return total;
}

double query_bridge_transition_fraction(const std::vector<Eigen::VectorXd>& samples,
                                        const std::vector<int>& transitions,
                                        double audited_bridge_length,
                                        double fallback_path_length) {
    const double denominator = audited_bridge_length > 1e-12
        ? audited_bridge_length
        : std::max(1e-12, fallback_path_length);
    return query_bridge_transition_length_sum(samples, transitions) / denominator;
}

double query_bridge_waypoint_length(const std::vector<Eigen::VectorXd>& path) {
    double total = 0.0;
    for (std::size_t index = 1; index < path.size(); ++index) {
        total += (path[index] - path[index - 1]).norm();
    }
    return total;
}

QueryBridgeEdgeRuntimeOptions query_bridge_edge_runtime_options() {
    QueryBridgeEdgeRuntimeOptions options;
    options.scene_reusable_edges =
        detail::env_int_or_default("RBF_QUERY_BRIDGE_SCENE_REUSABLE_EDGES", 0) != 0;
    options.direct_segment_after_rrt =
        detail::env_int_or_default("RBF_QUERY_BRIDGE_DIRECT_SEGMENT_AFTER_RRT", 0) != 0;
    options.direct_segment_after_rrt_min_length = std::max(
        0.0,
        detail::env_double_or_default("RBF_QUERY_BRIDGE_DIRECT_SEGMENT_AFTER_RRT_MIN_LENGTH",
                                      0.0));
    return options;
}

QueryBridgeWaypointShortcutOptions query_bridge_waypoint_shortcut_options(
    bool direct_segment_after_rrt_candidate) {
    QueryBridgeWaypointShortcutOptions options;
    options.enabled =
        detail::env_int_or_default("RBF_QUERY_BRIDGE_WAYPOINT_SHORTCUT",
                                   direct_segment_after_rrt_candidate ? 1 : 0) != 0;
    options.min_gain =
        std::max(0.0,
                 detail::env_double_or_default("RBF_QUERY_BRIDGE_WAYPOINT_SHORTCUT_MIN_GAIN",
                                               1e-6));
    return options;
}

bool query_bridge_internal_simplify_enabled(bool direct_segment_after_rrt_candidate) {
    return detail::env_int_or_default("RBF_QUERY_BRIDGE_INTERNAL_SIMPLIFY",
                                      direct_segment_after_rrt_candidate ? 1 : 0) != 0;
}

QueryBridgeDirectCorridorRuntimeOptions query_bridge_direct_corridor_runtime_options(
    int query_index,
    double audit_step) {
    QueryBridgeDirectCorridorRuntimeOptions options;
    options.max_length =
        std::max(0.0, detail::env_double_or_default("RBF_QUERY_BRIDGE_DIRECT_MAX_LENGTH", 6.5));
    options.audit_step = audit_step > 0.0 ? audit_step : 0.01;
    const double base_sample_step =
        detail::env_double_or_default("RBF_QUERY_BRIDGE_DIRECT_SAMPLE_STEP",
                                      options.audit_step);
    options.sample_step = std::max(
        1e-4,
        detail::env_indexed_double_or_default("RBF_QUERY_BRIDGE_DIRECT_SAMPLE_STEP",
                                              query_index,
                                              base_sample_step));
    options.partition_neighbor_candidates =
        detail::env_int_or_default("RBF_QUERY_BRIDGE_PARTITION_NEIGHBOR_CANDIDATES", 0) != 0;
    options.immediate_partition_append =
        detail::env_int_or_default("RBF_QUERY_BRIDGE_DIRECT_APPEND_PARTITION_IMMEDIATE", 0) != 0;
    options.partition_append_batch_size = options.immediate_partition_append
        ? std::max(1,
                   detail::env_int_or_default(
                       "RBF_QUERY_BRIDGE_DIRECT_PARTITION_APPEND_BATCH_SIZE",
                       32))
        : 0;
    options.detailed_timing =
        detail::env_int_or_default("RBF_QUERY_BRIDGE_DETAILED_TIMING", 0) != 0;
    options.local_sample_assimilation =
        detail::env_int_or_default("RBF_QUERY_BRIDGE_LOCAL_SAMPLE_ASSIMILATION", 1) != 0;
    options.ffb_diagnostics =
        detail::env_int_or_default("RBF_QUERY_BRIDGE_FFB_DIAGNOSTICS", 0) != 0;
    options.group_residual_gaps =
        detail::env_int_or_default("RBF_QUERY_BRIDGE_GROUP_RESIDUAL_GAPS", 0) != 0;
    options.residual_milestone_segments =
        detail::env_int_or_default("RBF_QUERY_BRIDGE_RESIDUAL_MILESTONE_SEGMENTS", 0) != 0;
    options.full_residual_overlay_when_connected =
        detail::env_int_or_default("RBF_QUERY_BRIDGE_FULL_RESIDUAL_OVERLAY_WHEN_CONNECTED",
                                   0) != 0;
    return options;
}

std::vector<int> query_bridge_order_transitions_by_gap_length(
    const std::vector<Eigen::VectorXd>& samples,
    const std::vector<int>& transitions,
    int priority_mode) {
    if (priority_mode <= 0 || transitions.size() < 2) {
        return transitions;
    }
    struct GapGroup {
        int begin = 0;
        int end = 0;
        double length = 0.0;
    };
    std::vector<GapGroup> groups;
    groups.reserve(transitions.size());
    for (int transition : transitions) {
        if (groups.empty() || transition > groups.back().end + 1) {
            groups.push_back({transition,
                              transition,
                              query_bridge_transition_length(samples, transition)});
        } else {
            groups.back().end = transition;
            groups.back().length += query_bridge_transition_length(samples, transition);
        }
    }
    std::stable_sort(groups.begin(), groups.end(), [](const GapGroup& lhs,
                                                       const GapGroup& rhs) {
        if (std::abs(lhs.length - rhs.length) > 1e-12) {
            return lhs.length > rhs.length;
        }
        return lhs.begin < rhs.begin;
    });
    std::vector<int> ordered;
    ordered.reserve(transitions.size());
    for (const auto& group : groups) {
        const int center = (group.begin + group.end) / 2;
        ordered.push_back(center);
        for (int radius = 1;
             center - radius >= group.begin || center + radius <= group.end;
             ++radius) {
            if (center - radius >= group.begin) {
                ordered.push_back(center - radius);
            }
            if (center + radius <= group.end) {
                ordered.push_back(center + radius);
            }
        }
    }
    return ordered;
}

int query_bridge_nearest_nonempty_layer(const std::vector<std::vector<int>>& sample_layers,
                                        int start_index,
                                        int direction) {
    int index = start_index;
    while (index >= 0 && index < static_cast<int>(sample_layers.size())) {
        if (!sample_layers[static_cast<std::size_t>(index)].empty()) {
            return index;
        }
        index += direction;
    }
    return -1;
}

QueryBridgeDirectFfbTaskBuildResult query_bridge_build_direct_ffb_tasks(
    const std::vector<Eigen::VectorXd>& samples,
    const std::vector<bool>& covered,
    const QueryBridgeDirectFfbTaskBuildOptions& options) {
    QueryBridgeDirectFfbTaskBuildResult result;
    result.tasks.reserve(samples.size());
    const int max_transition_hint = std::max(0, options.max_transition_hint);
    auto append_sample = [&](std::size_t sample_index) {
        result.tasks.push_back(
            {samples[sample_index],
             sample_index,
             std::min(static_cast<int>(sample_index), max_transition_hint)});
    };
    auto sample_covered = [&](std::size_t sample_index) {
        return sample_index < covered.size() && covered[sample_index];
    };
    if (options.grouped_direct_seeds) {
        const int max_group_seeds = std::max(1, options.max_group_seeds);
        std::size_t sample_index = 0;
        while (sample_index < samples.size()) {
            while (sample_index < samples.size() && sample_covered(sample_index)) {
                ++sample_index;
            }
            if (sample_index >= samples.size()) {
                break;
            }
            const std::size_t begin = sample_index;
            while (sample_index < samples.size() && !sample_covered(sample_index)) {
                ++sample_index;
            }
            const std::size_t end = sample_index - 1;
            result.uncovered_gap_groups += 1;
            std::vector<std::size_t> chosen;
            chosen.reserve(static_cast<std::size_t>(
                std::min(max_group_seeds, static_cast<int>(end - begin + 1))));
            auto push_unique_index = [&](std::size_t index) {
                index = std::min(end, std::max(begin, index));
                if (std::find(chosen.begin(), chosen.end(), index) == chosen.end()) {
                    chosen.push_back(index);
                }
            };
            const std::size_t group_count = end - begin + 1;
            if (group_count <= static_cast<std::size_t>(max_group_seeds)) {
                for (std::size_t index = begin; index <= end; ++index) {
                    push_unique_index(index);
                }
            } else {
                push_unique_index((begin + end) / 2);
                if (static_cast<int>(chosen.size()) < max_group_seeds) {
                    push_unique_index(begin);
                }
                if (static_cast<int>(chosen.size()) < max_group_seeds) {
                    push_unique_index(end);
                }
                if (static_cast<int>(chosen.size()) < max_group_seeds && end > begin + 1) {
                    push_unique_index(begin + (end - begin) / 4);
                }
                if (static_cast<int>(chosen.size()) < max_group_seeds && end > begin + 1) {
                    push_unique_index(begin + (3 * (end - begin)) / 4);
                }
                for (int rank = 1;
                     static_cast<int>(chosen.size()) < max_group_seeds &&
                     rank < max_group_seeds - 1;
                     ++rank) {
                    const double alpha = static_cast<double>(rank) /
                                         static_cast<double>(max_group_seeds - 1);
                    const auto offset = static_cast<std::size_t>(
                        std::llround(alpha * static_cast<double>(end - begin)));
                    push_unique_index(begin + offset);
                }
            }
            for (std::size_t index : chosen) {
                append_sample(index);
            }
        }
    } else if (options.center_out_direct_tasks) {
        std::size_t sample_index = 0;
        while (sample_index < samples.size()) {
            while (sample_index < samples.size() && sample_covered(sample_index)) {
                ++sample_index;
            }
            if (sample_index >= samples.size()) {
                break;
            }
            const std::size_t begin = sample_index;
            while (sample_index < samples.size() && !sample_covered(sample_index)) {
                ++sample_index;
            }
            const std::size_t end = sample_index - 1;
            result.uncovered_gap_groups += 1;
            const std::size_t center = (begin + end) / 2;
            append_sample(center);
            for (std::size_t radius = 1;
                 center >= begin + radius || center + radius <= end;
                 ++radius) {
                if (center >= begin + radius) {
                    append_sample(center - radius);
                }
                if (center + radius <= end) {
                    append_sample(center + radius);
                }
            }
        }
    } else {
        for (std::size_t sample_index = 0; sample_index < samples.size(); ++sample_index) {
            if (!sample_covered(sample_index)) {
                append_sample(sample_index);
            }
        }
    }
    return result;
}

QueryBridgeDirectFfbTaskRuntimeOptions query_bridge_direct_ffb_task_runtime_options(
    std::size_t sample_count) {
    const int max_transition_hint = std::max(0, static_cast<int>(sample_count) - 2);
    const bool grouped_direct_seeds =
        detail::env_int_or_default("RBF_QUERY_BRIDGE_DIRECT_GROUPED_SEEDS", 0) != 0;
    const int max_group_seeds =
        std::max(1, detail::env_int_or_default("RBF_QUERY_BRIDGE_DIRECT_MAX_SEEDS_PER_GAP", 3));
    const bool coverage_order_direct_tasks =
        detail::env_int_or_default("RBF_QUERY_BRIDGE_COVERAGE_ORDER_DIRECT_TASKS", 1) != 0;
    const bool center_out_direct_tasks =
        !coverage_order_direct_tasks &&
        detail::env_int_or_default("RBF_QUERY_BRIDGE_CENTER_OUT_DIRECT_TASKS", 1) != 0;
    return {{max_transition_hint,
             max_group_seeds,
             grouped_direct_seeds,
             center_out_direct_tasks},
            coverage_order_direct_tasks};
}

std::vector<double> query_bridge_center_ordered_fractions(int subdivisions) {
    std::vector<double> fractions;
    if (subdivisions <= 1) {
        return fractions;
    }
    fractions.reserve(static_cast<std::size_t>(std::max(0, subdivisions - 1)));
    for (int item = 1; item < subdivisions; ++item) {
        fractions.push_back(static_cast<double>(item) / static_cast<double>(subdivisions));
    }
    std::stable_sort(fractions.begin(), fractions.end(), [](double lhs, double rhs) {
        return std::abs(lhs - 0.5) < std::abs(rhs - 0.5);
    });
    return fractions;
}

QueryBridgeRepairSubdivisionOptions query_bridge_repair_subdivision_options(int query_index) {
    QueryBridgeRepairSubdivisionOptions options;
    options.base_subdivisions =
        detail::env_int_or_default("RBF_QUERY_BRIDGE_REPAIR_SUBDIVISIONS", 6);
    options.subdivisions = std::max(
        0,
        detail::env_indexed_int_or_default("RBF_QUERY_BRIDGE_REPAIR_SUBDIVISIONS",
                                           query_index,
                                           options.base_subdivisions));
    options.fractions = query_bridge_center_ordered_fractions(options.subdivisions);
    return options;
}

QueryBridgeAdaptiveRepairOptions query_bridge_adaptive_repair_options(int query_index,
                                                                      int subdivisions,
                                                                      double audit_step,
                                                                      double sample_step) {
    QueryBridgeAdaptiveRepairOptions options;
    options.priority_mode =
        detail::env_int_or_default("RBF_QUERY_BRIDGE_ADAPTIVE_REPAIR_PRIORITY", 1);
    options.enabled =
        detail::env_int_or_default("RBF_QUERY_BRIDGE_ADAPTIVE_STEP_REPAIR", 1) != 0;
    options.target_segment_fraction =
        std::max(0.0,
                 detail::env_double_or_default(
                     "RBF_QUERY_BRIDGE_ADAPTIVE_REPAIR_TARGET_SEGMENT_FRACTION",
                     0.0));
    options.max_subdivisions = std::max(
        subdivisions + 1,
        detail::env_int_or_default("RBF_QUERY_BRIDGE_ADAPTIVE_MAX_REPAIR_SUBDIVISIONS",
                                   std::max(2, subdivisions * 2)));
    options.fine_step = std::max(
        1e-4,
        detail::env_double_or_default("RBF_QUERY_BRIDGE_ADAPTIVE_FINE_STEP",
                                      std::max(audit_step, sample_step * 0.5)));
    options.max_calls = std::max(
        0,
        detail::env_indexed_int_or_default(
            "RBF_QUERY_BRIDGE_ADAPTIVE_MAX_REPAIR_CALLS",
            query_index,
            detail::env_int_or_default("RBF_QUERY_BRIDGE_ADAPTIVE_MAX_REPAIR_CALLS",
                                       std::numeric_limits<int>::max())));
    return options;
}

QueryBridgeLateralRepairOptions query_bridge_lateral_repair_options(double sample_step) {
    QueryBridgeLateralRepairOptions options;
    options.enabled =
        detail::env_int_or_default("RBF_QUERY_BRIDGE_LATERAL_REPAIR", 0) != 0;
    options.dims =
        std::max(0, detail::env_int_or_default("RBF_QUERY_BRIDGE_LATERAL_REPAIR_DIMS", 2));
    options.rounds =
        std::max(1, detail::env_int_or_default("RBF_QUERY_BRIDGE_LATERAL_REPAIR_ROUNDS", 1));
    options.max_calls =
        std::max(0, detail::env_int_or_default("RBF_QUERY_BRIDGE_LATERAL_REPAIR_MAX_CALLS", 24));
    options.offset =
        std::max(1e-6,
                 detail::env_double_or_default("RBF_QUERY_BRIDGE_LATERAL_REPAIR_OFFSET",
                                               std::max(0.01, sample_step * 0.25)));
    return options;
}

std::vector<Eigen::VectorXd> query_bridge_lateral_candidates(
    const Eigen::VectorXd& seed,
    const Eigen::VectorXd& direction,
    const std::vector<Interval>& domain,
    int lateral_dims,
    int lateral_rounds,
    double lateral_offset) {
    std::vector<int> dims;
    dims.reserve(static_cast<std::size_t>(seed.size()));
    for (int dim = 0; dim < seed.size(); ++dim) {
        dims.push_back(dim);
    }
    std::sort(dims.begin(), dims.end(), [&](int lhs, int rhs) {
        const double lhs_abs = std::abs(direction[lhs]);
        const double rhs_abs = std::abs(direction[rhs]);
        if (std::abs(lhs_abs - rhs_abs) > 1e-12) {
            return lhs_abs < rhs_abs;
        }
        return lhs < rhs;
    });
    std::vector<Eigen::VectorXd> out;
    const int dim_limit = std::min<int>(std::max(0, lateral_dims),
                                        static_cast<int>(dims.size()));
    const int rounds = std::max(0, lateral_rounds);
    out.reserve(static_cast<std::size_t>(dim_limit * rounds * 2));
    for (int item = 0; item < dim_limit; ++item) {
        const int dim = dims[static_cast<std::size_t>(item)];
        for (int round = 1; round <= rounds; ++round) {
            for (double sign : {1.0, -1.0}) {
                Eigen::VectorXd candidate = seed;
                candidate[dim] += sign * lateral_offset * static_cast<double>(round);
                if (dim < static_cast<int>(domain.size())) {
                    candidate[dim] = std::min(domain[static_cast<std::size_t>(dim)].hi,
                                              std::max(domain[static_cast<std::size_t>(dim)].lo,
                                                       candidate[dim]));
                }
                if ((candidate - seed).norm() > 1e-12) {
                    out.push_back(std::move(candidate));
                }
            }
        }
    }
    return out;
}

std::vector<std::pair<int, int>> query_bridge_group_residual_gap_transitions(
    const std::vector<int>& final_bad,
    std::size_t layer_count,
    bool group_residual_gaps) {
    std::vector<std::pair<int, int>> gap_groups;
    gap_groups.reserve(final_bad.size());
    for (int transition : final_bad) {
        if (transition < 0 || transition + 1 >= static_cast<int>(layer_count)) {
            continue;
        }
        if (!group_residual_gaps ||
            gap_groups.empty() ||
            transition > gap_groups.back().second + 1) {
            gap_groups.emplace_back(transition, transition);
        } else {
            gap_groups.back().second = transition;
        }
    }
    return gap_groups;
}

std::vector<QueryBridgeResidualMilestone> query_bridge_compact_residual_milestones(
    const std::vector<Eigen::VectorXd>& samples,
    const std::vector<std::vector<int>>& sample_layers,
    const std::vector<QueryBridgeResidualMilestone>& repair_milestones,
    int box_count,
    QueryBridgeLocalDsu& dsu) {
    std::vector<QueryBridgeResidualMilestone> milestones;
    milestones.reserve(samples.size() + repair_milestones.size());
    for (std::size_t sample_index = 0; sample_index < sample_layers.size(); ++sample_index) {
        const auto& layer = sample_layers[sample_index];
        if (!layer.empty()) {
            milestones.push_back({static_cast<double>(sample_index),
                                  samples[sample_index],
                                  layer.front()});
        }
    }
    for (const auto& milestone : repair_milestones) {
        if (milestone.box_index >= 0 && milestone.box_index < box_count) {
            milestones.push_back(milestone);
        }
    }
    std::stable_sort(milestones.begin(),
                     milestones.end(),
                     [](const QueryBridgeResidualMilestone& lhs,
                        const QueryBridgeResidualMilestone& rhs) {
                         if (std::abs(lhs.param - rhs.param) > 1e-9) {
                             return lhs.param < rhs.param;
                         }
                         return lhs.box_index < rhs.box_index;
                     });
    std::vector<QueryBridgeResidualMilestone> compact;
    compact.reserve(milestones.size());
    for (const auto& milestone : milestones) {
        if (milestone.box_index < 0) {
            continue;
        }
        if (!compact.empty() &&
            std::abs(compact.back().param - milestone.param) <= 1e-9 &&
            dsu.find(compact.back().box_index) == dsu.find(milestone.box_index)) {
            continue;
        }
        compact.push_back(milestone);
    }
    return compact;
}

}  // namespace rbf
