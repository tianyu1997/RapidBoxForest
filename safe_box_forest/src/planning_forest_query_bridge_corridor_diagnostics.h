#pragma once

#include <SBF/runtime.h>

#include <cstddef>

namespace rbf {

struct QueryBridgeDirectCorridorRuntimeStats {
    double assimilate_coverage_span_sum = 0.0;
    int assimilate_coverage_boxes = 0;
    int assimilate_coverage_span_max = 0;
    int assimilate_local_hits = 0;
    int assimilate_full_scan_fallbacks = 0;
    int assimilate_local_sample_tests = 0;
};

struct QueryBridgeDirectCorridorSummaryStats {
    double elapsed_ms = 0.0;
    double direct_ffb_ms = 0.0;
    double repair_ffb_ms = 0.0;
    double adaptive_repair_ffb_ms = 0.0;
    double residual_segment_audit_ms = 0.0;
    double assimilate_coverage_span_sum = 0.0;
    std::size_t sample_count = 0;
    int direct_calls = 0;
    int repair_calls = 0;
    int adaptive_repair_calls = 0;
    int direct_added = 0;
    int repair_added = 0;
    int adaptive_repair_added = 0;
    int adaptive_repair_max_subdivisions_used = 0;
    int repair_subdivisions = 0;
    int initial_bad_count = 0;
    int final_bad_count = 0;
    int local_segment_edges_added = 0;
    int local_segment_gap_samples_max = 0;
    int assimilate_coverage_boxes = 0;
    int assimilate_coverage_span_max = 0;
    bool local_corridor_connected = false;
};

void query_bridge_record_direct_corridor_summary(
    StageContext& context,
    int query_index,
    const QueryBridgeDirectCorridorSummaryStats& stats);

}  // namespace rbf
