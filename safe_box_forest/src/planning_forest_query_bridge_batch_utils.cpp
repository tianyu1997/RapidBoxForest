#include "planning_forest_query_bridge_batch_utils.h"

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

void accumulate_query_bridge_direct_corridor_totals(const BuildProfile& profile,
                                                    StageContext& context,
                                                    std::size_t task_index) {
    auto task_key = [](std::size_t index, const std::string& suffix) {
        return "query_bridge.batch_task." + std::to_string(index) + "." + suffix;
    };
    auto add = [&](const std::string& suffix, const std::string& total_key) {
        const auto it = profile.diagnostics.find(task_key(task_index, suffix));
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
    add("direct_corridor_transition_connected_ms",
        "query_bridge.direct_corridor_transition_connected_ms");
    add("direct_corridor_transition_connected_calls",
        "query_bridge.direct_corridor_transition_connected_calls");
    add("direct_corridor_bad_transitions_ms",
        "query_bridge.direct_corridor_bad_transitions_ms");
    add("direct_corridor_bad_transitions_calls",
        "query_bridge.direct_corridor_bad_transitions_calls");
    add("direct_corridor_current_cover_ms", "query_bridge.direct_corridor_current_cover_ms");
    add("direct_corridor_current_cover_calls", "query_bridge.direct_corridor_current_cover_calls");
    add("direct_corridor_current_cover_partition_ms",
        "query_bridge.direct_corridor_current_cover_partition_ms");
    add("direct_corridor_current_cover_corridor_scan_ms",
        "query_bridge.direct_corridor_current_cover_corridor_scan_ms");
    add("direct_corridor_current_cover_direct_index_ms",
        "query_bridge.direct_corridor_current_cover_direct_index_ms");
    add("direct_corridor_duplicate_lookup_ms", "query_bridge.direct_corridor_duplicate_lookup_ms");
    add("direct_corridor_duplicate_lookup_calls",
        "query_bridge.direct_corridor_duplicate_lookup_calls");
    add("direct_corridor_commit_total_ms", "query_bridge.direct_corridor_commit_total_ms");
    add("direct_corridor_commit_calls", "query_bridge.direct_corridor_commit_calls");
    add("direct_corridor_commit_dynamic_policy_ms",
        "query_bridge.direct_corridor_commit_dynamic_policy_ms");
    add("direct_corridor_commit_partition_append_ms",
        "query_bridge.direct_corridor_commit_partition_append_ms");
    add("direct_corridor_partition_append_calls",
        "query_bridge.direct_corridor_partition_append_calls");
    add("direct_corridor_partition_append_boxes",
        "query_bridge.direct_corridor_partition_append_boxes");
    add("direct_corridor_assimilate_calls", "query_bridge.direct_corridor_assimilate_calls");
    add("direct_corridor_assimilate_sample_scan_ms",
        "query_bridge.direct_corridor_assimilate_sample_scan_ms");
    add("direct_corridor_assimilate_local_hits",
        "query_bridge.direct_corridor_assimilate_local_hits");
    add("direct_corridor_assimilate_full_scan_fallbacks",
        "query_bridge.direct_corridor_assimilate_full_scan_fallbacks");
    add("direct_corridor_assimilate_local_sample_tests",
        "query_bridge.direct_corridor_assimilate_local_sample_tests");
    add("direct_corridor_assimilate_candidate_build_ms",
        "query_bridge.direct_corridor_assimilate_candidate_build_ms");
    add("direct_corridor_assimilate_adjacency_ms",
        "query_bridge.direct_corridor_assimilate_adjacency_ms");
    add("direct_corridor_segment_insert_ms", "query_bridge.direct_corridor_segment_insert_ms");
    add("direct_corridor_segment_insert_calls",
        "query_bridge.direct_corridor_segment_insert_calls");
    add("direct_corridor_direct_task_build_ms",
        "query_bridge.direct_corridor_direct_task_build_ms");
    add("direct_corridor_direct_loop_ms", "query_bridge.direct_corridor_direct_loop_ms");
    add("direct_corridor_repair_loop_ms", "query_bridge.direct_corridor_repair_loop_ms");
    add("direct_corridor_adaptive_loop_ms", "query_bridge.direct_corridor_adaptive_loop_ms");
    add("direct_corridor_lateral_loop_ms", "query_bridge.direct_corridor_lateral_loop_ms");
    add("direct_corridor_residual_segment_loop_ms",
        "query_bridge.direct_corridor_residual_segment_loop_ms");
}

}  // namespace rbf
