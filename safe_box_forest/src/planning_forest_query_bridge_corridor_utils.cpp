#include "planning_forest_query_bridge_corridor_utils.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>

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

void query_bridge_record_direct_corridor_detailed_timing(
    StageContext& context,
    int query_index,
    const QueryBridgeDirectCorridorDetailedTimingStats& stats) {
    auto add = [&](const char* suffix, double value) {
        context.diagnostics().add_counter(
            std::string("query_bridge.direct_corridor_") + suffix,
            value);
    };
    auto set_task = [&](const char* suffix, double value) {
        if (query_index < 0) {
            return;
        }
        context.diagnostics().set_value(
            "query_bridge.batch_task." + std::to_string(query_index) +
                ".direct_corridor_" + suffix,
            value);
    };

    add("transition_connected_ms", stats.transition_connected_ms);
    add("transition_connected_calls",
        static_cast<double>(stats.transition_connected_calls));
    add("bad_transitions_ms", stats.bad_transitions_ms);
    add("bad_transitions_calls", static_cast<double>(stats.bad_transitions_calls));
    add("current_cover_ms", stats.current_cover_ms);
    add("current_cover_calls", static_cast<double>(stats.current_cover_calls));
    add("current_cover_partition_ms", stats.current_cover_partition_ms);
    add("current_cover_corridor_scan_ms", stats.current_cover_corridor_scan_ms);
    add("current_cover_direct_index_ms", stats.current_cover_direct_index_ms);
    add("duplicate_lookup_ms", stats.duplicate_lookup_ms);
    add("duplicate_lookup_calls", static_cast<double>(stats.duplicate_lookup_calls));
    add("commit_total_ms", stats.commit_total_ms);
    add("commit_calls", static_cast<double>(stats.commit_calls));
    add("commit_dynamic_policy_ms", stats.commit_dynamic_policy_ms);
    add("commit_partition_append_ms", stats.commit_partition_append_ms);
    add("partition_append_calls",
        static_cast<double>(stats.direct_partition_append_calls));
    add("partition_append_boxes",
        static_cast<double>(stats.direct_partition_append_boxes));
    add("assimilate_calls", static_cast<double>(stats.assimilate_calls));
    add("assimilate_sample_scan_ms", stats.assimilate_sample_scan_ms);
    add("assimilate_local_hits",
        static_cast<double>(stats.assimilate_local_hits));
    add("assimilate_full_scan_fallbacks",
        static_cast<double>(stats.assimilate_full_scan_fallbacks));
    add("assimilate_local_sample_tests",
        static_cast<double>(stats.assimilate_local_sample_tests));
    add("assimilate_candidate_build_ms", stats.assimilate_candidate_build_ms);
    add("assimilate_adjacency_ms", stats.assimilate_adjacency_ms);
    add("segment_insert_ms", stats.segment_insert_ms);
    add("segment_insert_calls", static_cast<double>(stats.segment_insert_calls));
    add("direct_task_build_ms", stats.direct_task_build_ms);
    add("direct_loop_ms", stats.direct_loop_ms);
    add("repair_loop_ms", stats.repair_loop_ms);
    add("adaptive_loop_ms", stats.adaptive_loop_ms);
    add("lateral_loop_ms", stats.lateral_loop_ms);
    add("residual_segment_loop_ms", stats.residual_segment_loop_ms);

    set_task("transition_connected_ms", stats.transition_connected_ms);
    set_task("transition_connected_calls",
             static_cast<double>(stats.transition_connected_calls));
    set_task("bad_transitions_ms", stats.bad_transitions_ms);
    set_task("bad_transitions_calls",
             static_cast<double>(stats.bad_transitions_calls));
    set_task("current_cover_ms", stats.current_cover_ms);
    set_task("current_cover_calls", static_cast<double>(stats.current_cover_calls));
    set_task("current_cover_partition_ms", stats.current_cover_partition_ms);
    set_task("current_cover_corridor_scan_ms",
             stats.current_cover_corridor_scan_ms);
    set_task("current_cover_direct_index_ms", stats.current_cover_direct_index_ms);
    set_task("duplicate_lookup_ms", stats.duplicate_lookup_ms);
    set_task("duplicate_lookup_calls",
             static_cast<double>(stats.duplicate_lookup_calls));
    set_task("commit_total_ms", stats.commit_total_ms);
    set_task("commit_calls", static_cast<double>(stats.commit_calls));
    set_task("commit_dynamic_policy_ms", stats.commit_dynamic_policy_ms);
    set_task("commit_partition_append_ms", stats.commit_partition_append_ms);
    set_task("partition_append_calls",
             static_cast<double>(stats.direct_partition_append_calls));
    set_task("partition_append_boxes",
             static_cast<double>(stats.direct_partition_append_boxes));
    set_task("assimilate_calls", static_cast<double>(stats.assimilate_calls));
    set_task("assimilate_sample_scan_ms", stats.assimilate_sample_scan_ms);
    set_task("assimilate_local_hits",
             static_cast<double>(stats.assimilate_local_hits));
    set_task("assimilate_full_scan_fallbacks",
             static_cast<double>(stats.assimilate_full_scan_fallbacks));
    set_task("assimilate_local_sample_tests",
             static_cast<double>(stats.assimilate_local_sample_tests));
    set_task("assimilate_candidate_build_ms", stats.assimilate_candidate_build_ms);
    set_task("assimilate_adjacency_ms", stats.assimilate_adjacency_ms);
    set_task("segment_insert_ms", stats.segment_insert_ms);
    set_task("segment_insert_calls", static_cast<double>(stats.segment_insert_calls));
    set_task("direct_task_build_ms", stats.direct_task_build_ms);
    set_task("assimilate_coverage_span_max",
             static_cast<double>(stats.assimilate_coverage_span_max));
    set_task("assimilate_coverage_span_mean",
             stats.assimilate_coverage_boxes > 0
                 ? stats.assimilate_coverage_span_sum /
                       static_cast<double>(stats.assimilate_coverage_boxes)
                 : 0.0);
    set_task("direct_loop_ms", stats.direct_loop_ms);
    set_task("repair_loop_ms", stats.repair_loop_ms);
    set_task("adaptive_loop_ms", stats.adaptive_loop_ms);
    set_task("lateral_loop_ms", stats.lateral_loop_ms);
    set_task("residual_segment_loop_ms", stats.residual_segment_loop_ms);
}

void query_bridge_record_direct_corridor_summary(
    StageContext& context,
    int query_index,
    const QueryBridgeDirectCorridorSummaryStats& stats) {
    auto set = [&](const char* suffix, double value) {
        context.diagnostics().set_value(
            std::string("query_bridge.direct_corridor_") + suffix,
            value);
    };
    auto add_total = [&](const char* suffix, double value) {
        context.diagnostics().add_counter(
            std::string("query_bridge.direct_corridor_") + suffix + "_total",
            value);
    };
    auto set_task = [&](const char* suffix, double value) {
        if (query_index < 0) {
            return;
        }
        context.diagnostics().set_value(
            "query_bridge.batch_task." + std::to_string(query_index) +
                ".direct_corridor_" + suffix,
            value);
    };
    auto set_ffb_task = [&](const char* ffb_key, const char* suffix) {
        set_task(suffix, context.diagnostics().value(std::string("ffb.") + ffb_key, 0.0));
    };

    const int all_ffb_calls = stats.direct_calls + stats.repair_calls +
                              stats.adaptive_repair_calls +
                              stats.lateral_repair_calls;
    const double sample_count = static_cast<double>(stats.sample_count);
    const double direct_calls = static_cast<double>(stats.direct_calls);
    const double all_ffb = static_cast<double>(all_ffb_calls);
    const double direct_added = static_cast<double>(stats.direct_added);
    const double repair_calls = static_cast<double>(stats.repair_calls);
    const double repair_added = static_cast<double>(stats.repair_added);
    const double adaptive_repair_calls =
        static_cast<double>(stats.adaptive_repair_calls);
    const double adaptive_repair_added =
        static_cast<double>(stats.adaptive_repair_added);
    const double lateral_repair_calls =
        static_cast<double>(stats.lateral_repair_calls);
    const double lateral_repair_added =
        static_cast<double>(stats.lateral_repair_added);
    const double initial_bad = static_cast<double>(stats.initial_bad_count);
    const double final_bad = static_cast<double>(stats.final_bad_count);
    const double segment_edges =
        static_cast<double>(stats.local_segment_edges_added);
    const double coverage_span_mean =
        stats.assimilate_coverage_boxes > 0
            ? stats.assimilate_coverage_span_sum /
                  static_cast<double>(stats.assimilate_coverage_boxes)
            : 0.0;

    set("ms", stats.elapsed_ms);
    add_total("ms", stats.elapsed_ms);
    set("samples", sample_count);
    add_total("samples", sample_count);
    set("ffb_calls", direct_calls);
    add_total("ffb_calls", direct_calls);
    set("direct_ffb_ms", stats.direct_ffb_ms);
    set("repair_ffb_ms", stats.repair_ffb_ms);
    set("adaptive_repair_ffb_ms", stats.adaptive_repair_ffb_ms);
    set("lateral_repair_ffb_ms", stats.lateral_repair_ffb_ms);
    set("segment_audit_ms", stats.residual_segment_audit_ms);
    set("all_ffb_calls", all_ffb);
    add_total("all_ffb_calls", all_ffb);
    set("added", direct_added);
    add_total("added", direct_added);
    set("repair_calls", repair_calls);
    add_total("repair_calls", repair_calls);
    set("repair_added", repair_added);
    add_total("repair_added", repair_added);
    set("adaptive_repair_calls", adaptive_repair_calls);
    add_total("adaptive_repair_calls", adaptive_repair_calls);
    set("adaptive_repair_added", adaptive_repair_added);
    add_total("adaptive_repair_added", adaptive_repair_added);
    set("lateral_repair_enabled", stats.lateral_repair_enabled ? 1.0 : 0.0);
    set("lateral_repair_calls", lateral_repair_calls);
    add_total("lateral_repair_calls", lateral_repair_calls);
    set("lateral_repair_added", lateral_repair_added);
    add_total("lateral_repair_added", lateral_repair_added);
    set("adaptive_repair_max_subdivisions",
        static_cast<double>(stats.adaptive_repair_max_subdivisions_used));
    set("repair_subdivisions", static_cast<double>(stats.repair_subdivisions));
    set("bad_initial", initial_bad);
    add_total("bad_initial", initial_bad);
    set("bad_final", final_bad);
    add_total("bad_final", final_bad);
    set("segment_edges", segment_edges);
    add_total("segment_edges", segment_edges);
    set("segment_gap_samples_max",
        static_cast<double>(stats.local_segment_gap_samples_max));
    set("assimilate_coverage_span_max",
        static_cast<double>(stats.assimilate_coverage_span_max));
    set("assimilate_coverage_span_mean", coverage_span_mean);
    set("local_connected", stats.local_corridor_connected ? 1.0 : 0.0);

    set_task("ms", stats.elapsed_ms);
    set_task("samples", sample_count);
    set_task("ffb_calls", direct_calls);
    set_task("direct_ffb_ms", stats.direct_ffb_ms);
    set_task("repair_ffb_ms", stats.repair_ffb_ms);
    set_task("adaptive_repair_ffb_ms", stats.adaptive_repair_ffb_ms);
    set_task("lateral_repair_ffb_ms", stats.lateral_repair_ffb_ms);
    set_task("segment_audit_ms", stats.residual_segment_audit_ms);
    set_task("all_ffb_calls", all_ffb);
    set_task("added", direct_added);
    set_task("repair_calls", repair_calls);
    set_task("repair_added", repair_added);
    set_task("adaptive_repair_calls", adaptive_repair_calls);
    set_task("adaptive_repair_added", adaptive_repair_added);
    set_task("lateral_repair_calls", lateral_repair_calls);
    set_task("lateral_repair_added", lateral_repair_added);
    set_task("bad_initial", initial_bad);
    set_task("bad_final", final_bad);
    set_task("segment_edges", segment_edges);
    set_task("local_connected", stats.local_corridor_connected ? 1.0 : 0.0);
    set_ffb_task("find_calls", "ffb_find_calls");
    set_ffb_task("binary_requested", "ffb_binary_requested");
    set_ffb_task("virtual_sparse_binary_attempts",
                 "ffb_virtual_sparse_binary_attempts");
    set_ffb_task("virtual_sparse_binary_successes",
                 "ffb_virtual_sparse_binary_successes");
    set_ffb_task("virtual_sparse_binary_probes",
                 "ffb_virtual_sparse_binary_probes");
    set_ffb_task("binary_materialized_fallback_calls",
                 "ffb_binary_materialized_fallback_calls");
    set_ffb_task("binary_blocked_adaptive_depths",
                 "ffb_binary_blocked_adaptive_depths");
    set_ffb_task("binary_virtual_unsupported",
                 "ffb_binary_virtual_unsupported");
    set_ffb_task("linear_descent_calls", "ffb_linear_descent_calls");
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

QueryBridgeDirectFfbTaskPlan query_bridge_prepare_direct_ffb_task_plan(
    StageContext& context,
    const std::vector<Eigen::VectorXd>& samples,
    const std::vector<bool>& covered,
    int ffb_start_depth,
    bool detailed_timing) {
    using Clock = std::chrono::steady_clock;
    QueryBridgeDirectFfbTaskPlan plan;
    plan.runtime = query_bridge_direct_ffb_task_runtime_options(samples.size());
    const auto build_t0 = detailed_timing ? Clock::now() : Clock::time_point{};
    QueryBridgeDirectFfbTaskBuildResult build =
        query_bridge_build_direct_ffb_tasks(samples, covered, plan.runtime.build);
    if (detailed_timing) {
        plan.build_ms =
            std::chrono::duration<double, std::milli>(Clock::now() - build_t0).count();
    }
    plan.uncovered_gap_groups = build.uncovered_gap_groups;
    plan.tasks = std::move(build.tasks);

    context.diagnostics().set_value("query_bridge.direct_corridor_direct_grouped_seeds",
                                    plan.runtime.build.grouped_direct_seeds ? 1.0 : 0.0);
    context.diagnostics().set_value("query_bridge.direct_corridor_coverage_order_direct_tasks",
                                    plan.runtime.coverage_order_direct_tasks ? 1.0 : 0.0);
    context.diagnostics().set_value("query_bridge.direct_corridor_center_out_direct_tasks",
                                    plan.runtime.build.center_out_direct_tasks ? 1.0 : 0.0);
    context.diagnostics().set_value("query_bridge.direct_corridor_ffb_start_depth",
                                    static_cast<double>(ffb_start_depth));
    context.diagnostics().set_value("query_bridge.direct_corridor_uncovered_gap_groups",
                                    static_cast<double>(plan.uncovered_gap_groups));
    context.diagnostics().set_value("query_bridge.direct_corridor_direct_max_seeds_per_gap",
                                    static_cast<double>(plan.runtime.build.max_group_seeds));
    context.diagnostics().set_value("query_bridge.direct_corridor_direct_tasks",
                                    static_cast<double>(plan.tasks.size()));
    return plan;
}

QueryBridgeFfbTaskExecutionStats query_bridge_run_direct_ffb_tasks(
    StageContext& context,
    const std::vector<QueryBridgeDirectFfbTask>& tasks,
    const std::vector<bool>& covered,
    const std::function<FindFreeBoxResult(const QueryBridgeDirectFfbTask&)>& find_box,
    const std::function<QueryBridgeFfbTaskCommitResult(FindFreeBoxResult&&,
                                                       const QueryBridgeDirectFfbTask&)>& commit_box,
    bool detailed_timing) {
    using Clock = std::chrono::steady_clock;
    QueryBridgeFfbTaskExecutionStats stats;
    const auto loop_t0 = detailed_timing ? Clock::now() : Clock::time_point{};
    for (const auto& task : tasks) {
        if (task.sample_index < covered.size() && covered[task.sample_index]) {
            context.diagnostics().add_counter(
                "query_bridge.direct_corridor_direct_skip_covered");
            continue;
        }
        const auto ffb_t0 = Clock::now();
        FindFreeBoxResult result = find_box(task);
        stats.ffb_ms +=
            std::chrono::duration<double, std::milli>(Clock::now() - ffb_t0).count();
        stats.calls += 1;
        const QueryBridgeFfbTaskCommitResult commit = commit_box(std::move(result), task);
        if (commit.box_index >= 0 && commit.added_box) {
            stats.added += 1;
        }
    }
    if (detailed_timing) {
        stats.loop_ms =
            std::chrono::duration<double, std::milli>(Clock::now() - loop_t0).count();
    }
    return stats;
}

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
