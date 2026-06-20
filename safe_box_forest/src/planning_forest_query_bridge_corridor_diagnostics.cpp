#include "planning_forest_query_bridge_corridor_diagnostics.h"

#include <SBF/runtime.h>

#include <string>

namespace rbf {

QueryBridgeDirectCorridorSummaryStats query_bridge_make_direct_corridor_summary(
    double elapsed_ms,
    double direct_ffb_ms,
    double repair_ffb_ms,
    double adaptive_repair_ffb_ms,
    double residual_segment_audit_ms,
    const QueryBridgeDirectCorridorRuntimeStats& runtime_stats,
    std::size_t sample_count,
    int direct_calls,
    int repair_calls,
    int adaptive_repair_calls,
    int direct_added,
    int repair_added,
    int adaptive_repair_added,
    int adaptive_repair_max_subdivisions_used,
    int repair_subdivisions,
    int initial_bad_count,
    int final_bad_count,
    int local_segment_edges_added,
    int local_segment_gap_samples_max,
    bool local_corridor_connected) {
    QueryBridgeDirectCorridorSummaryStats stats;
    stats.elapsed_ms = elapsed_ms;
    stats.direct_ffb_ms = direct_ffb_ms;
    stats.repair_ffb_ms = repair_ffb_ms;
    stats.adaptive_repair_ffb_ms = adaptive_repair_ffb_ms;
    stats.residual_segment_audit_ms = residual_segment_audit_ms;
    stats.assimilate_coverage_span_sum =
        runtime_stats.assimilate_coverage_span_sum;
    stats.sample_count = sample_count;
    stats.direct_calls = direct_calls;
    stats.repair_calls = repair_calls;
    stats.adaptive_repair_calls = adaptive_repair_calls;
    stats.direct_added = direct_added;
    stats.repair_added = repair_added;
    stats.adaptive_repair_added = adaptive_repair_added;
    stats.adaptive_repair_max_subdivisions_used =
        adaptive_repair_max_subdivisions_used;
    stats.repair_subdivisions = repair_subdivisions;
    stats.initial_bad_count = initial_bad_count;
    stats.final_bad_count = final_bad_count;
    stats.local_segment_edges_added = local_segment_edges_added;
    stats.local_segment_gap_samples_max = local_segment_gap_samples_max;
    stats.assimilate_coverage_boxes =
        runtime_stats.assimilate_coverage_boxes;
    stats.assimilate_coverage_span_max =
        runtime_stats.assimilate_coverage_span_max;
    stats.local_corridor_connected = local_corridor_connected;
    return stats;
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
                              stats.adaptive_repair_calls;
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
    set_task("segment_audit_ms", stats.residual_segment_audit_ms);
    set_task("all_ffb_calls", all_ffb);
    set_task("added", direct_added);
    set_task("repair_calls", repair_calls);
    set_task("repair_added", repair_added);
    set_task("adaptive_repair_calls", adaptive_repair_calls);
    set_task("adaptive_repair_added", adaptive_repair_added);
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
