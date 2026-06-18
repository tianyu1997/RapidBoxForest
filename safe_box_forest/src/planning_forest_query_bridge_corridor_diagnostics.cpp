#include "planning_forest_query_bridge_corridor_utils.h"

#include <SBF/runtime.h>

#include <string>

namespace rbf {

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

}  // namespace rbf
