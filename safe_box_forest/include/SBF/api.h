#pragma once

#include <rbf/core.h>
#include <rbf/envelope.h>

#include <Eigen/Core>

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace rbf {

enum class ExecutionMode : std::uint8_t {
    Inline = 0,
    Parallel = 1,
};

struct RuntimeConfig {
    ExecutionMode mode = ExecutionMode::Inline;
    int n_threads = 1;
    int batch_size = 0;
    int parallel_threshold = 0;
    bool deterministic_reduce = true;
};

enum class PathAuditStatus : std::uint8_t {
    NotRun = 0,
    Passed = 1,
    Failed = 2,
    Repaired = 3,
};

struct QueryResult {
    bool success = false;
    int start_box_id = -1;
    int goal_box_id = -1;
    std::vector<int> box_sequence;
    std::vector<int> segment_edge_sequence;
    int segment_edges_used = 0;
    std::vector<Eigen::VectorXd> path;
    double path_length = 0.0;
    double raw_path_length = 0.0;
    double query_time_ms = 0.0;
    bool audit_passed = false;
    PathAuditStatus audit_status = PathAuditStatus::NotRun;
    int failed_segment_index = -1;
    int repair_count = 0;
    int remaining_unsafe_assumptions = 0;
    double audit_time_ms = 0.0;
    double repair_time_ms = 0.0;
    double final_simplify_time_ms = 0.0;
    double certified_box_length = 0.0;
    double provisional_audited_length = 0.0;
    double segment_edge_length = 0.0;
};

struct BuildProfile {
    int raw_boxes = 0;
    int final_boxes = 0;
    int segment_edges = 0;
    int grow_adjacency_islands = 0;
    int grow_largest_island = 0;
    int adjacency_islands = 0;
    int bridge_boxes_added = 0;
    int segment_edges_added = 0;
    int rrt_segment_edges_added = 0;
    int point_gap_segment_edges_added = 0;
    int connector_attempted_pairs = 0;
    bool connector_connected = false;
    double grow_ms = 0.0;
    double merge_ms = 0.0;
    double connector_ms = 0.0;
    double adjacency_ms = 0.0;
    double collision_check_ms = 0.0;
    double envelope_ms = 0.0;
    double total_ms = 0.0;
    int envelope_batches = 0;
    int envelope_threads = 1;
    std::unordered_map<std::string, double> diagnostics;
};

}  // namespace rbf
