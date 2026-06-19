#include "planning_forest_query_bridge_diagnostics.h"

#include <string>

namespace rbf {

void add_query_bridge_oracle_counter_delta(BuildProfile& profile,
                                           const OracleCounters& before,
                                           const OracleCounters& after) {
    auto add_counter_delta = [&](const std::string& key, auto after_value, auto before_value) {
        profile.diagnostics[key] += static_cast<double>(after_value - before_value);
    };
    add_counter_delta("query_bridge.oracle_node_validations",
                      after.node_validations,
                      before.node_validations);
    add_counter_delta("query_bridge.oracle_validation_cache_hits",
                      after.validation_cache_hits,
                      before.validation_cache_hits);
    add_counter_delta("query_bridge.oracle_validation_cache_misses",
                      after.validation_cache_misses,
                      before.validation_cache_misses);
    add_counter_delta("query_bridge.oracle_materializations",
                      after.materializations,
                      before.materializations);
    add_counter_delta("query_bridge.oracle_external_exact_hits",
                      after.materialization_external_exact_hits,
                      before.materialization_external_exact_hits);
    add_counter_delta("query_bridge.oracle_external_exact_misses",
                      after.materialization_external_exact_misses,
                      before.materialization_external_exact_misses);
    add_counter_delta("query_bridge.oracle_interval_replay_compatibility_checks",
                      after.interval_replay_compatibility_checks,
                      before.interval_replay_compatibility_checks);
    add_counter_delta("query_bridge.oracle_interval_replay_compatible",
                      after.interval_replay_compatible,
                      before.interval_replay_compatible);
    add_counter_delta("query_bridge.oracle_interval_replay_incompatible",
                      after.interval_replay_incompatible,
                      before.interval_replay_incompatible);
    add_counter_delta("query_bridge.oracle_interval_replay_direct_exact_hits",
                      after.interval_replay_direct_exact_hits,
                      before.interval_replay_direct_exact_hits);
    add_counter_delta("query_bridge.oracle_interval_replay_key_only_blocked",
                      after.interval_replay_key_only_blocked,
                      before.interval_replay_key_only_blocked);
    add_counter_delta("query_bridge.oracle_shared_endpoint_cache_hits",
                      after.materialization_reused_shared_endpoint_cache,
                      before.materialization_reused_shared_endpoint_cache);
    add_counter_delta("query_bridge.oracle_endpoint_path_ms",
                      after.validate_node_endpoint_path_time_us * 1.0e-3,
                      before.validate_node_endpoint_path_time_us * 1.0e-3);
    add_counter_delta("query_bridge.oracle_classify_ms",
                      after.validate_node_classify_time_us * 1.0e-3,
                      before.validate_node_classify_time_us * 1.0e-3);
    add_counter_delta("query_bridge.oracle_validate_total_ms",
                      after.validate_node_total_time_us * 1.0e-3,
                      before.validate_node_total_time_us * 1.0e-3);
    add_counter_delta("query_bridge.oracle_materialization_endpoint_ms",
                      after.materialization_endpoint_time_us * 1.0e-3,
                      before.materialization_endpoint_time_us * 1.0e-3);
    add_counter_delta("query_bridge.oracle_materialization_envelope_ms",
                      after.materialization_envelope_time_us * 1.0e-3,
                      before.materialization_envelope_time_us * 1.0e-3);
    add_counter_delta("query_bridge.oracle_envelope_collision_queries",
                      after.envelope_collision_queries,
                      before.envelope_collision_queries);
    add_counter_delta("query_bridge.oracle_envelope_gjk_tests",
                      after.envelope_collision_gjk_tests,
                      before.envelope_collision_gjk_tests);
}

void query_bridge_mark_task_skip(BuildProfile& profile,
                                 std::size_t index,
                                 double code,
                                 const char* reason) {
    profile.diagnostics[query_bridge_task_key(index, "skip_reason_code")] = code;
    if (reason != nullptr && reason[0] != '\0') {
        profile.diagnostics[std::string("query_bridge.batch_task_skip.") + reason] += 1.0;
    }
}

void record_query_bridge_partition_path_first_task(StageContext& context,
                                                   std::size_t index) {
    context.diagnostics().add_counter("query_bridge.partition_path_first_tasks");
    context.diagnostics().set_value(
        query_bridge_task_key(index, "partition_path_first"),
        1.0);
}

void record_query_bridge_partition_path_first_rrt_skipped(StageContext& context,
                                                          std::size_t index) {
    context.diagnostics().add_counter("query_bridge.partition_path_first_rrt_skipped");
    context.diagnostics().set_value(
        query_bridge_task_key(index, "waypoint_from_partition_path"),
        1.0);
}

void record_query_bridge_batch_task_no_path(StageContext& context,
                                            std::size_t index,
                                            double total_ms) {
    context.diagnostics().add_counter("query_bridge.batch_tasks_no_path");
    context.diagnostics().set_value(query_bridge_task_key(index, "no_path"), 1.0);
    context.diagnostics().set_value(query_bridge_task_key(index, "total_ms"), total_ms);
}

void record_query_bridge_batch_task_already_satisfied(
    StageContext& context,
    const QueryBridgeSearchTask& task,
    double probe_ms) {
    context.diagnostics().add_counter("query_bridge.batch_tasks_skipped");
    context.diagnostics().record_timing("query_bridge.batch_probe_ms_total", probe_ms);
    context.diagnostics().set_value(query_bridge_task_key(task.index, "skipped"), 1.0);
    if (task.hipac_online_satisfied) {
        context.diagnostics().set_value(
            query_bridge_task_key(task.index, "skipped_by_hipac_online"),
            1.0);
    }
    if (task.direct_start_goal_satisfied) {
        context.diagnostics().set_value(
            query_bridge_task_key(task.index, "skipped_by_direct_start_goal_segment"),
            1.0);
    }
}

void record_query_bridge_batch_task_skipped_after_rrt(StageContext& context,
                                                      std::size_t index,
                                                      bool forced_task,
                                                      double probe_ms,
                                                      double total_ms) {
    context.diagnostics().add_counter("query_bridge.batch_tasks_skipped_after_rrt");
    if (forced_task) {
        context.diagnostics().add_counter("query_bridge.batch_forced_tasks_skipped_after_rrt");
    }
    context.diagnostics().record_timing("query_bridge.batch_probe_ms_total", probe_ms);
    context.diagnostics().set_value(query_bridge_task_key(index, "skipped_after_rrt"), 1.0);
    context.diagnostics().set_value(query_bridge_task_key(index, "total_ms"), total_ms);
}

void record_query_bridge_batch_task_skipped_by_hipac_after_rrt(
    StageContext& context,
    std::size_t index,
    double total_ms) {
    context.diagnostics().add_counter(
        "query_bridge.batch_tasks_skipped_by_hipac_after_rrt");
    context.diagnostics().set_value(
        query_bridge_task_key(index, "skipped_by_hipac_after_rrt"),
        1.0);
    context.diagnostics().set_value(query_bridge_task_key(index, "total_ms"),
                                    total_ms);
}

void record_query_bridge_forced_attempts(StageContext& context,
                                         std::size_t index,
                                         bool forced_task,
                                         int attempts) {
    if (forced_task) {
        context.diagnostics().set_value(query_bridge_task_key(index, "forced"), 1.0);
    }
    context.diagnostics().set_value(query_bridge_task_key(index, "attempts"),
                                    static_cast<double>(attempts));
}

void accumulate_query_bridge_direct_corridor_totals(const BuildProfile& profile,
                                                    StageContext& context,
                                                    std::size_t task_index) {
    auto add = [&](const std::string& suffix, const std::string& total_key) {
        const auto it = profile.diagnostics.find(query_bridge_task_key(task_index, suffix));
        if (it != profile.diagnostics.end()) {
            context.diagnostics().add_counter(total_key, it->second);
        }
    };
    add("direct_corridor_ms", "query_bridge.direct_corridor_ms_total");
    add("direct_corridor_samples", "query_bridge.direct_corridor_samples_total");
    add("direct_corridor_ffb_calls", "query_bridge.direct_corridor_ffb_calls_total");
    add("direct_corridor_all_ffb_calls", "query_bridge.direct_corridor_all_ffb_calls_total");
    add("direct_corridor_direct_ffb_ms", "query_bridge.direct_corridor_direct_ffb_ms");
    add("direct_corridor_repair_ffb_ms", "query_bridge.direct_corridor_repair_ffb_ms");
    add("direct_corridor_adaptive_repair_ffb_ms",
        "query_bridge.direct_corridor_adaptive_repair_ffb_ms");
    add("direct_corridor_lateral_repair_ffb_ms",
        "query_bridge.direct_corridor_lateral_repair_ffb_ms");
    add("direct_corridor_segment_audit_ms",
        "query_bridge.direct_corridor_segment_audit_ms");
    add("direct_corridor_added", "query_bridge.direct_corridor_added_total");
    add("direct_corridor_repair_calls", "query_bridge.direct_corridor_repair_calls_total");
    add("direct_corridor_repair_added", "query_bridge.direct_corridor_repair_added_total");
    add("direct_corridor_adaptive_repair_calls",
        "query_bridge.direct_corridor_adaptive_repair_calls_total");
    add("direct_corridor_adaptive_repair_added",
        "query_bridge.direct_corridor_adaptive_repair_added_total");
    add("direct_corridor_lateral_repair_calls",
        "query_bridge.direct_corridor_lateral_repair_calls_total");
    add("direct_corridor_lateral_repair_added",
        "query_bridge.direct_corridor_lateral_repair_added_total");
    add("direct_corridor_bad_initial", "query_bridge.direct_corridor_bad_initial_total");
    add("direct_corridor_bad_final", "query_bridge.direct_corridor_bad_final_total");
    add("direct_corridor_segment_edges", "query_bridge.direct_corridor_segment_edges_total");
    add("direct_corridor_ffb_find_calls", "query_bridge.direct_corridor_ffb_find_calls_total");
    add("direct_corridor_ffb_binary_requested",
        "query_bridge.direct_corridor_ffb_binary_requested_total");
    add("direct_corridor_ffb_virtual_sparse_binary_attempts",
        "query_bridge.direct_corridor_ffb_virtual_sparse_binary_attempts_total");
    add("direct_corridor_ffb_virtual_sparse_binary_successes",
        "query_bridge.direct_corridor_ffb_virtual_sparse_binary_successes_total");
    add("direct_corridor_ffb_virtual_sparse_binary_probes",
        "query_bridge.direct_corridor_ffb_virtual_sparse_binary_probes_total");
    add("direct_corridor_ffb_binary_materialized_fallback_calls",
        "query_bridge.direct_corridor_ffb_binary_materialized_fallback_calls_total");
    add("direct_corridor_ffb_binary_blocked_adaptive_depths",
        "query_bridge.direct_corridor_ffb_binary_blocked_adaptive_depths_total");
    add("direct_corridor_ffb_binary_virtual_unsupported",
        "query_bridge.direct_corridor_ffb_binary_virtual_unsupported_total");
    add("direct_corridor_ffb_linear_descent_calls",
        "query_bridge.direct_corridor_ffb_linear_descent_calls_total");
}

}  // namespace rbf
