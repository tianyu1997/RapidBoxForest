#include "planning_forest_query_bridge_corridor_repair.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace rbf {

QueryBridgeSubdivisionRepairStats query_bridge_run_subdivision_repair_pass(
    StageContext& context,
    const std::vector<Eigen::VectorXd>& samples,
    const std::vector<int>& transitions,
    const std::vector<double>& fractions,
    const std::function<bool(int)>& transition_connected,
    const std::function<bool(const Eigen::VectorXd&)>& seed_covered,
    const std::function<FindFreeBoxResult(const Eigen::VectorXd&, int)>& find_box,
    const std::function<QueryBridgeFfbTaskCommitResult(FindFreeBoxResult&&,
                                                       const Eigen::VectorXd&,
                                                       int)>& commit_box,
    bool detailed_timing) {
    using Clock = std::chrono::steady_clock;
    QueryBridgeSubdivisionRepairStats stats;
    const auto loop_t0 = detailed_timing ? Clock::now() : Clock::time_point{};
    for (int transition : transitions) {
        if (transition_connected(transition)) {
            continue;
        }
        const Eigen::VectorXd& a = samples[static_cast<std::size_t>(transition)];
        const Eigen::VectorXd& b = samples[static_cast<std::size_t>(transition + 1)];
        for (double u : fractions) {
            if (transition_connected(transition)) {
                break;
            }
            const Eigen::VectorXd seed = (1.0 - u) * a + u * b;
            if (seed_covered(seed)) {
                context.diagnostics().add_counter(
                    "query_bridge.direct_corridor_repair_skip_covered");
                continue;
            }
            const auto ffb_t0 = Clock::now();
            FindFreeBoxResult result = find_box(seed, transition);
            stats.ffb_ms +=
                std::chrono::duration<double, std::milli>(Clock::now() - ffb_t0).count();
            stats.calls += 1;
            const QueryBridgeFfbTaskCommitResult commit =
                commit_box(std::move(result), seed, transition);
            if (commit.box_index >= 0) {
                if (std::find(stats.committed_indices.begin(),
                              stats.committed_indices.end(),
                              commit.box_index) == stats.committed_indices.end()) {
                    stats.committed_indices.push_back(commit.box_index);
                }
                if (commit.added_box) {
                    stats.added += 1;
                }
                if (transition_connected(transition)) {
                    break;
                }
            }
        }
    }
    if (detailed_timing) {
        stats.loop_ms =
            std::chrono::duration<double, std::milli>(Clock::now() - loop_t0).count();
    }
    return stats;
}

QueryBridgeLateralRepairStats query_bridge_run_lateral_repair_pass(
    StageContext& context,
    const std::vector<Eigen::VectorXd>& samples,
    const std::vector<int>& transitions,
    const std::vector<Interval>& domain,
    const QueryBridgeLateralRepairOptions& options,
    const std::function<bool(int)>& transition_connected,
    const std::function<bool(const Eigen::VectorXd&)>& seed_covered,
    const std::function<FindFreeBoxResult(const Eigen::VectorXd&, int)>& find_box,
    const std::function<QueryBridgeFfbTaskCommitResult(FindFreeBoxResult&&,
                                                       const Eigen::VectorXd&,
                                                       int)>& commit_box,
    bool detailed_timing) {
    using Clock = std::chrono::steady_clock;
    QueryBridgeLateralRepairStats stats;
    const auto loop_t0 = detailed_timing ? Clock::now() : Clock::time_point{};
    for (int transition : transitions) {
        if (stats.calls >= options.max_calls) {
            break;
        }
        if (transition_connected(transition) ||
            transition < 0 ||
            transition + 1 >= static_cast<int>(samples.size())) {
            continue;
        }
        const Eigen::VectorXd& a = samples[static_cast<std::size_t>(transition)];
        const Eigen::VectorXd& b = samples[static_cast<std::size_t>(transition + 1)];
        const Eigen::VectorXd seed = 0.5 * (a + b);
        const Eigen::VectorXd direction = b - a;
        for (const Eigen::VectorXd& lateral_seed :
             query_bridge_lateral_candidates(seed,
                                             direction,
                                             domain,
                                             options.dims,
                                             options.rounds,
                                             options.offset)) {
            if (stats.calls >= options.max_calls) {
                break;
            }
            if (transition_connected(transition)) {
                break;
            }
            if (seed_covered(lateral_seed)) {
                context.diagnostics().add_counter(
                    "query_bridge.direct_corridor_lateral_repair_skip_covered");
                continue;
            }
            const auto ffb_t0 = Clock::now();
            FindFreeBoxResult result = find_box(lateral_seed, transition);
            stats.ffb_ms +=
                std::chrono::duration<double, std::milli>(Clock::now() - ffb_t0).count();
            stats.calls += 1;
            const QueryBridgeFfbTaskCommitResult commit =
                commit_box(std::move(result), lateral_seed, transition);
            if (commit.box_index >= 0) {
                if (std::find(stats.committed_indices.begin(),
                              stats.committed_indices.end(),
                              commit.box_index) == stats.committed_indices.end()) {
                    stats.committed_indices.push_back(commit.box_index);
                }
                if (commit.added_box) {
                    stats.added += 1;
                }
                if (transition_connected(transition)) {
                    break;
                }
            }
        }
    }
    if (detailed_timing) {
        stats.loop_ms =
            std::chrono::duration<double, std::milli>(Clock::now() - loop_t0).count();
    }
    return stats;
}

QueryBridgeAdaptiveRepairStats query_bridge_run_adaptive_repair_pass(
    StageContext& context,
    const std::vector<Eigen::VectorXd>& samples,
    const std::vector<int>& initial_bad,
    int base_subdivisions,
    const QueryBridgeAdaptiveRepairOptions& options,
    const std::function<bool(int)>& transition_connected,
    const std::function<bool(const Eigen::VectorXd&)>& seed_covered,
    const std::function<std::vector<int>()>& bad_transitions,
    const std::function<double(const std::vector<int>&)>& bad_fraction,
    const std::function<FindFreeBoxResult(const Eigen::VectorXd&, int)>& find_box,
    const std::function<QueryBridgeFfbTaskCommitResult(FindFreeBoxResult&&,
                                                       const Eigen::VectorXd&,
                                                       int)>& commit_box,
    bool detailed_timing) {
    using Clock = std::chrono::steady_clock;
    QueryBridgeAdaptiveRepairStats stats;
    stats.max_subdivisions_used = base_subdivisions;
    stats.final_bad = initial_bad;
    stats.initial_bad_fraction = bad_fraction(stats.final_bad);

    context.diagnostics().set_value(
        "query_bridge.direct_corridor_adaptive_repair_priority",
        static_cast<double>(options.priority_mode));
    context.diagnostics().set_value(
        "query_bridge.direct_corridor_adaptive_repair_target_segment_fraction",
        options.target_segment_fraction);
    context.diagnostics().set_value(
        "query_bridge.direct_corridor_adaptive_initial_bad_fraction",
        stats.initial_bad_fraction);

    if (options.enabled && !stats.final_bad.empty()) {
        const auto loop_t0 = detailed_timing ? Clock::now() : Clock::time_point{};
        std::vector<int> ordered_final_bad =
            query_bridge_order_transitions_by_gap_length(samples,
                                                        stats.final_bad,
                                                        options.priority_mode);
        for (int transition : ordered_final_bad) {
            if (stats.calls >= options.max_calls) {
                break;
            }
            if (options.target_segment_fraction > 0.0 &&
                bad_fraction(stats.final_bad) <= options.target_segment_fraction) {
                context.diagnostics().add_counter(
                    "query_bridge.direct_corridor_adaptive_repair_target_stops");
                break;
            }
            if (transition_connected(transition) ||
                transition < 0 ||
                transition + 1 >= static_cast<int>(samples.size())) {
                continue;
            }
            const Eigen::VectorXd& a = samples[static_cast<std::size_t>(transition)];
            const Eigen::VectorXd& b = samples[static_cast<std::size_t>(transition + 1)];
            const double gap_length = (b - a).norm();
            const int target_subdivisions = std::min(
                options.max_subdivisions,
                std::max(base_subdivisions + 1,
                         static_cast<int>(std::ceil(gap_length / options.fine_step))));
            stats.max_subdivisions_used =
                std::max(stats.max_subdivisions_used, target_subdivisions);
            const std::vector<double> adaptive_fractions =
                query_bridge_center_ordered_fractions(target_subdivisions);
            for (double u : adaptive_fractions) {
                if (stats.calls >= options.max_calls) {
                    break;
                }
                if (transition_connected(transition)) {
                    break;
                }
                const Eigen::VectorXd seed = (1.0 - u) * a + u * b;
                if (seed_covered(seed)) {
                    context.diagnostics().add_counter(
                        "query_bridge.direct_corridor_adaptive_repair_skip_covered");
                    continue;
                }
                const auto ffb_t0 = Clock::now();
                FindFreeBoxResult result = find_box(seed, transition);
                stats.ffb_ms +=
                    std::chrono::duration<double, std::milli>(Clock::now() - ffb_t0).count();
                stats.calls += 1;
                const QueryBridgeFfbTaskCommitResult commit =
                    commit_box(std::move(result), seed, transition);
                if (commit.box_index >= 0) {
                    if (std::find(stats.committed_indices.begin(),
                                  stats.committed_indices.end(),
                                  commit.box_index) == stats.committed_indices.end()) {
                        stats.committed_indices.push_back(commit.box_index);
                    }
                    if (commit.added_box) {
                        stats.added += 1;
                    }
                    if (options.target_segment_fraction > 0.0) {
                        stats.final_bad = bad_transitions();
                    }
                    if (transition_connected(transition)) {
                        break;
                    }
                }
            }
        }
        stats.final_bad = bad_transitions();
        if (detailed_timing) {
            stats.loop_ms =
                std::chrono::duration<double, std::milli>(Clock::now() - loop_t0).count();
        }
    }
    stats.final_bad_fraction = bad_fraction(stats.final_bad);
    context.diagnostics().set_value(
        "query_bridge.direct_corridor_adaptive_final_bad_fraction",
        stats.final_bad_fraction);
    return stats;
}

void query_bridge_run_residual_segment_gap_pass(
    StageContext& context,
    const std::vector<Eigen::VectorXd>& samples,
    const std::vector<std::vector<int>>& sample_layers,
    const std::vector<QueryBridgeResidualMilestone>& repair_milestones,
    const std::vector<int>& final_bad,
    int box_count,
    bool group_residual_gaps,
    bool residual_milestone_segments,
    QueryBridgeLocalDsu& dsu,
    const std::function<bool(int,
                             int,
                             const Eigen::VectorXd&,
                             const Eigen::VectorXd&,
                             int)>& insert_segment) {
    const std::vector<std::pair<int, int>> gap_groups =
        query_bridge_group_residual_gap_transitions(final_bad,
                                                    sample_layers.size(),
                                                    group_residual_gaps);
    context.diagnostics().set_value(
        "query_bridge.direct_corridor_segment_gap_groups",
        static_cast<double>(gap_groups.size()));
    context.diagnostics().set_value(
        "query_bridge.direct_corridor_residual_milestone_segments",
        residual_milestone_segments ? 1.0 : 0.0);
    context.diagnostics().set_value(
        "query_bridge.direct_corridor_repair_milestones",
        static_cast<double>(repair_milestones.size()));

    if (residual_milestone_segments) {
        const std::vector<QueryBridgeResidualMilestone> compact =
            query_bridge_compact_residual_milestones(samples,
                                                     sample_layers,
                                                     repair_milestones,
                                                     box_count,
                                                     dsu);
        context.diagnostics().set_value(
            "query_bridge.direct_corridor_residual_milestones",
            static_cast<double>(compact.size()));
        for (std::size_t index = 0; index + 1 < compact.size(); ++index) {
            const auto& lhs = compact[index];
            const auto& rhs = compact[index + 1];
            if (rhs.param <= lhs.param + 1e-9) {
                continue;
            }
            const int sample_gap = static_cast<int>(
                std::ceil(std::max(0.0, rhs.param - lhs.param)));
            insert_segment(lhs.box_index,
                           rhs.box_index,
                           lhs.point,
                           rhs.point,
                           sample_gap);
        }
        return;
    }

    std::vector<std::pair<int, int>> pending_gap_groups;
    pending_gap_groups.reserve(gap_groups.size());
    for (auto it = gap_groups.rbegin(); it != gap_groups.rend(); ++it) {
        pending_gap_groups.push_back(*it);
    }
    while (!pending_gap_groups.empty()) {
        const auto gap_group = pending_gap_groups.back();
        pending_gap_groups.pop_back();
        const int lhs_sample =
            query_bridge_nearest_nonempty_layer(sample_layers, gap_group.first, -1);
        const int rhs_sample =
            query_bridge_nearest_nonempty_layer(sample_layers, gap_group.second + 1, 1);
        if (lhs_sample < 0 || rhs_sample < 0 || lhs_sample >= rhs_sample) {
            continue;
        }
        const auto& lhs_layer = sample_layers[static_cast<std::size_t>(lhs_sample)];
        const auto& rhs_layer = sample_layers[static_cast<std::size_t>(rhs_sample)];
        if (lhs_layer.empty() || rhs_layer.empty()) {
            continue;
        }
        const int lhs_index = lhs_layer.front();
        const int rhs_index = rhs_layer.front();
        const Eigen::VectorXd& lhs_point = samples[static_cast<std::size_t>(lhs_sample)];
        const Eigen::VectorXd& rhs_point = samples[static_cast<std::size_t>(rhs_sample)];
        const bool inserted = insert_segment(lhs_index,
                                             rhs_index,
                                             lhs_point,
                                             rhs_point,
                                             rhs_sample - lhs_sample);
        if (!inserted) {
            if (group_residual_gaps && gap_group.first < gap_group.second) {
                const int mid = (gap_group.first + gap_group.second) / 2;
                pending_gap_groups.emplace_back(mid + 1, gap_group.second);
                pending_gap_groups.emplace_back(gap_group.first, mid);
                context.diagnostics().add_counter(
                    "query_bridge.direct_corridor_segment_group_splits");
            }
            continue;
        }
    }
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
