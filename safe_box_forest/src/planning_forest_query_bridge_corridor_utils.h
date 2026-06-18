#pragma once

#include <Eigen/Core>

#include <SBF/box_graph.h>
#include <SBF/find_free_box.h>
#include <SBF/runtime.h>

#include <cstddef>
#include <functional>
#include <utility>
#include <vector>

namespace rbf {

struct QueryBridgeResidualMilestone {
    double param = 0.0;
    Eigen::VectorXd point;
    int box_index = -1;
};

struct QueryBridgeDirectFfbTask {
    Eigen::VectorXd seed;
    std::size_t sample_index = 0;
    int transition_hint = 0;
};

struct QueryBridgeDirectFfbTaskBuildOptions {
    int max_transition_hint = 0;
    int max_group_seeds = 3;
    bool grouped_direct_seeds = false;
    bool center_out_direct_tasks = false;
};

struct QueryBridgeDirectFfbTaskBuildResult {
    std::vector<QueryBridgeDirectFfbTask> tasks;
    int uncovered_gap_groups = 0;
};

struct QueryBridgeDirectFfbTaskRuntimeOptions {
    QueryBridgeDirectFfbTaskBuildOptions build;
    bool coverage_order_direct_tasks = true;
};

struct QueryBridgeDirectFfbTaskPlan {
    std::vector<QueryBridgeDirectFfbTask> tasks;
    QueryBridgeDirectFfbTaskRuntimeOptions runtime;
    int uncovered_gap_groups = 0;
    double build_ms = 0.0;
};

struct QueryBridgeFfbTaskCommitResult {
    int box_index = -1;
    bool added_box = false;
};

struct QueryBridgeFfbTaskExecutionStats {
    int calls = 0;
    int added = 0;
    double ffb_ms = 0.0;
    double loop_ms = 0.0;
};

struct QueryBridgeSubdivisionRepairStats {
    int calls = 0;
    int added = 0;
    double ffb_ms = 0.0;
    double loop_ms = 0.0;
    std::vector<int> committed_indices;
};

struct QueryBridgeRepairSubdivisionOptions {
    int base_subdivisions = 0;
    int subdivisions = 0;
    std::vector<double> fractions;
};

struct QueryBridgeAdaptiveRepairOptions {
    int priority_mode = 1;
    bool enabled = true;
    double target_segment_fraction = 0.0;
    int max_subdivisions = 0;
    double fine_step = 0.0;
    int max_calls = 0;
};

struct QueryBridgeLateralRepairOptions {
    bool enabled = false;
    int dims = 0;
    int rounds = 1;
    int max_calls = 0;
    double offset = 0.0;
};

struct QueryBridgeEdgeRuntimeOptions {
    bool scene_reusable_edges = false;
    bool direct_segment_after_rrt = false;
    bool direct_start_goal_segment = true;
    bool fast_direct_segment_after_rrt = false;
    bool fast_direct_shortcut = true;
    int fast_direct_random_shortcut_iters = 0;
    double direct_segment_after_rrt_min_length = 0.0;
};

struct QueryBridgeWaypointShortcutOptions {
    bool enabled = false;
    double min_gain = 0.0;
};

struct QueryBridgeDirectCorridorRuntimeOptions {
    double max_length = 0.0;
    double audit_step = 0.01;
    double sample_step = 0.01;
    bool partition_neighbor_candidates = false;
    bool immediate_partition_append = false;
    int partition_append_batch_size = 0;
    bool detailed_timing = false;
    bool local_sample_assimilation = true;
    bool ffb_diagnostics = false;
    bool group_residual_gaps = false;
    bool residual_milestone_segments = false;
    bool full_residual_overlay_when_connected = false;
};

struct QueryBridgeDirectCorridorDetailedTimingStats {
    double transition_connected_ms = 0.0;
    double bad_transitions_ms = 0.0;
    double current_cover_ms = 0.0;
    double current_cover_partition_ms = 0.0;
    double current_cover_corridor_scan_ms = 0.0;
    double current_cover_direct_index_ms = 0.0;
    double duplicate_lookup_ms = 0.0;
    double commit_total_ms = 0.0;
    double commit_dynamic_policy_ms = 0.0;
    double commit_partition_append_ms = 0.0;
    double assimilate_sample_scan_ms = 0.0;
    double assimilate_candidate_build_ms = 0.0;
    double assimilate_adjacency_ms = 0.0;
    double segment_insert_ms = 0.0;
    double direct_task_build_ms = 0.0;
    double direct_loop_ms = 0.0;
    double repair_loop_ms = 0.0;
    double adaptive_loop_ms = 0.0;
    double lateral_loop_ms = 0.0;
    double residual_segment_loop_ms = 0.0;
    double assimilate_coverage_span_sum = 0.0;
    int transition_connected_calls = 0;
    int bad_transitions_calls = 0;
    int current_cover_calls = 0;
    int duplicate_lookup_calls = 0;
    int commit_calls = 0;
    int assimilate_calls = 0;
    int assimilate_coverage_boxes = 0;
    int assimilate_coverage_span_max = 0;
    int segment_insert_calls = 0;
    int direct_partition_append_calls = 0;
    int direct_partition_append_boxes = 0;
    int assimilate_local_hits = 0;
    int assimilate_full_scan_fallbacks = 0;
    int assimilate_local_sample_tests = 0;
};

struct QueryBridgeDirectCorridorSummaryStats {
    double elapsed_ms = 0.0;
    double direct_ffb_ms = 0.0;
    double repair_ffb_ms = 0.0;
    double adaptive_repair_ffb_ms = 0.0;
    double lateral_repair_ffb_ms = 0.0;
    double residual_segment_audit_ms = 0.0;
    double assimilate_coverage_span_sum = 0.0;
    std::size_t sample_count = 0;
    int direct_calls = 0;
    int repair_calls = 0;
    int adaptive_repair_calls = 0;
    int lateral_repair_calls = 0;
    int direct_added = 0;
    int repair_added = 0;
    int adaptive_repair_added = 0;
    int lateral_repair_added = 0;
    int adaptive_repair_max_subdivisions_used = 0;
    int repair_subdivisions = 0;
    int initial_bad_count = 0;
    int final_bad_count = 0;
    int local_segment_edges_added = 0;
    int local_segment_gap_samples_max = 0;
    int assimilate_coverage_boxes = 0;
    int assimilate_coverage_span_max = 0;
    bool lateral_repair_enabled = false;
    bool local_corridor_connected = false;
};

struct QueryBridgeLocalDsu {
    std::vector<int> parent;

    explicit QueryBridgeLocalDsu(std::size_t count = 0);

    int add();
    int find(int value);
    void unite(int lhs, int rhs);
};

double query_bridge_seed_path_param(const std::vector<Eigen::VectorXd>& samples,
                                    const Eigen::VectorXd& seed,
                                    int transition_hint);

double query_bridge_transition_length(const std::vector<Eigen::VectorXd>& samples,
                                      int transition);

double query_bridge_transition_length_sum(const std::vector<Eigen::VectorXd>& samples,
                                          const std::vector<int>& transitions);

double query_bridge_transition_fraction(const std::vector<Eigen::VectorXd>& samples,
                                        const std::vector<int>& transitions,
                                        double audited_bridge_length,
                                        double fallback_path_length);

double query_bridge_waypoint_length(const std::vector<Eigen::VectorXd>& path);

QueryBridgeEdgeRuntimeOptions query_bridge_edge_runtime_options();

QueryBridgeWaypointShortcutOptions query_bridge_waypoint_shortcut_options(
    bool direct_segment_after_rrt_candidate);

bool query_bridge_internal_simplify_enabled(bool direct_segment_after_rrt_candidate);

QueryBridgeDirectCorridorRuntimeOptions query_bridge_direct_corridor_runtime_options(
    int query_index,
    double audit_step);

void query_bridge_record_direct_corridor_detailed_timing(
    StageContext& context,
    int query_index,
    const QueryBridgeDirectCorridorDetailedTimingStats& stats);

void query_bridge_record_direct_corridor_summary(
    StageContext& context,
    int query_index,
    const QueryBridgeDirectCorridorSummaryStats& stats);

std::vector<int> query_bridge_order_transitions_by_gap_length(
    const std::vector<Eigen::VectorXd>& samples,
    const std::vector<int>& transitions,
    int priority_mode);

int query_bridge_nearest_nonempty_layer(const std::vector<std::vector<int>>& sample_layers,
                                        int start_index,
                                        int direction);

QueryBridgeDirectFfbTaskBuildResult query_bridge_build_direct_ffb_tasks(
    const std::vector<Eigen::VectorXd>& samples,
    const std::vector<bool>& covered,
    const QueryBridgeDirectFfbTaskBuildOptions& options);

QueryBridgeDirectFfbTaskRuntimeOptions query_bridge_direct_ffb_task_runtime_options(
    std::size_t sample_count);

QueryBridgeDirectFfbTaskPlan query_bridge_prepare_direct_ffb_task_plan(
    StageContext& context,
    const std::vector<Eigen::VectorXd>& samples,
    const std::vector<bool>& covered,
    int ffb_start_depth,
    bool detailed_timing);

QueryBridgeFfbTaskExecutionStats query_bridge_run_direct_ffb_tasks(
    StageContext& context,
    const std::vector<QueryBridgeDirectFfbTask>& tasks,
    const std::vector<bool>& covered,
    const std::function<FindFreeBoxResult(const QueryBridgeDirectFfbTask&)>& find_box,
    const std::function<QueryBridgeFfbTaskCommitResult(FindFreeBoxResult&&,
                                                       const QueryBridgeDirectFfbTask&)>& commit_box,
    bool detailed_timing);

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
    bool detailed_timing);

std::vector<double> query_bridge_center_ordered_fractions(int subdivisions);

QueryBridgeRepairSubdivisionOptions query_bridge_repair_subdivision_options(int query_index);

QueryBridgeAdaptiveRepairOptions query_bridge_adaptive_repair_options(int query_index,
                                                                      int subdivisions,
                                                                      double audit_step,
                                                                      double sample_step);

QueryBridgeLateralRepairOptions query_bridge_lateral_repair_options(double sample_step);

std::vector<Eigen::VectorXd> query_bridge_lateral_candidates(
    const Eigen::VectorXd& seed,
    const Eigen::VectorXd& direction,
    const std::vector<Interval>& domain,
    int lateral_dims,
    int lateral_rounds,
    double lateral_offset);

std::vector<std::pair<int, int>> query_bridge_group_residual_gap_transitions(
    const std::vector<int>& final_bad,
    std::size_t layer_count,
    bool group_residual_gaps);

std::vector<QueryBridgeResidualMilestone> query_bridge_compact_residual_milestones(
    const std::vector<Eigen::VectorXd>& samples,
    const std::vector<std::vector<int>>& sample_layers,
    const std::vector<QueryBridgeResidualMilestone>& repair_milestones,
    int box_count,
    QueryBridgeLocalDsu& dsu);

}  // namespace rbf
