#pragma once

#include <Eigen/Core>

#include <cstdint>
#include <vector>

namespace rbf {

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
    int obb_edges_used = 0;
    int obb_regions_used = 0;
    double obb_edge_length = 0.0;
    int partition_cells_used = 0;
    int non_grid_cells_used = 0;
    double partition_search_ms = 0.0;
    double partition_repair_ms = 0.0;
    double residual_segment_fraction = 0.0;
};

}  // namespace rbf
