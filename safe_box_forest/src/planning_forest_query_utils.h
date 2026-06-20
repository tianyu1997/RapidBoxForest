#pragma once

#include <SBF/box_graph.h>
#include <SBF/connector_types.h>
#include <SBF/query.h>
#include <SBF/runtime.h>
#include <LECTDatabase/sbf/scene.h>
#include <LECTDatabase/sbf/oracle_types.h>

#include <Eigen/Core>

#include <cstdint>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

namespace rbf {

struct QueryBridgeParallelRrtOptions;

inline constexpr int kSeedAttemptStride = 7919;
inline constexpr int kSeedQueryStride = 104729;
inline constexpr int kSeedRepairLocalOffset = 101;
inline constexpr int kSeedRepairGlobalOffset = 211;
inline constexpr int kSeedFinalSimplifyOffset = 307;
inline constexpr int kSeedQueryBridgeOffset = 401;
inline constexpr int kSeedBridgeSimplifyOffset = 503;
inline constexpr int kSeedBatchBridgeOffset = 601;
inline constexpr int kSeedDebugBridgeOffset = 701;
inline constexpr int kSeedCorridorRefineOffset = 809;

std::vector<Eigen::VectorXd> collision_shortcut_path(const std::vector<Eigen::VectorXd>& path,
                                                     const CollisionChecker& checker,
                                                     int segment_resolution);

std::vector<Eigen::VectorXd> hybridize_collision_free_paths(
    const std::vector<std::vector<Eigen::VectorXd>>& paths,
    const CollisionChecker& checker,
    int segment_resolution,
    int max_paths,
    int max_vertices,
    int max_cross_checks);

std::vector<Eigen::VectorXd> random_collision_shortcut_path(std::vector<Eigen::VectorXd> path,
                                                            const CollisionChecker& checker,
                                                            int segment_resolution,
                                                            int iterations,
                                                            std::uint32_t seed);

int collision_shortcut_resolution(const QueryConfig& config);

RRTConnectConfig with_query_root_hull_domain(const RRTConnectConfig& config,
                                             const BoxOracle& oracle,
                                             const Eigen::Ref<const Eigen::VectorXd>& start,
                                             const Eigen::Ref<const Eigen::VectorXd>& goal);

const SegmentEdge* find_segment_edge_by_id(const SegmentEdgeList& edges, int edge_id);

Robot make_sbf_clearance_robot(const Robot& robot, double clearance);

const BoxNode* find_box_by_id(const std::vector<BoxNode>& boxes, int box_id);

bool partition_boxes_connected_local(const BoxNode& lhs,
                                     const BoxNode& rhs,
                                     double tolerance);

Eigen::VectorXd partition_shared_face_center_local(const BoxNode& lhs,
                                                   const BoxNode& rhs);

Eigen::VectorXd partition_transition_waypoint_local(const BoxNode& lhs,
                                                    const BoxNode& rhs,
                                                    const Eigen::Ref<const Eigen::VectorXd>& from,
                                                    const Eigen::Ref<const Eigen::VectorXd>& goal,
                                                    double tolerance);

std::vector<Eigen::VectorXd> extract_partition_waypoints_local(
    const std::vector<int>& box_sequence,
    const std::vector<int>& segment_edge_sequence,
    const std::vector<BoxNode>& boxes,
    const SegmentEdgeList& segment_edges,
    const Eigen::Ref<const Eigen::VectorXd>& start,
    const Eigen::Ref<const Eigen::VectorXd>& goal,
    double adjacency_tolerance);

Eigen::VectorXd closest_point_in_box(const BoxNode& box,
                                     const Eigen::Ref<const Eigen::VectorXd>& point);

double segment_exit_parameter_from_intervals(const std::vector<Interval>& intervals,
                                             const Eigen::Ref<const Eigen::VectorXd>& from,
                                             const Eigen::Ref<const Eigen::VectorXd>& to);

Eigen::VectorXd boundary_seed_from_intervals(const std::vector<Interval>& intervals,
                                             const Eigen::Ref<const Eigen::VectorXd>& from,
                                             const Eigen::Ref<const Eigen::VectorXd>& to,
                                             const std::vector<Interval>& domain,
                                             double face_epsilon);

std::vector<Eigen::VectorXd> lateral_offset_seeds_local(
    const Eigen::Ref<const Eigen::VectorXd>& seed,
    const Eigen::Ref<const Eigen::VectorXd>& direction,
    const std::vector<Interval>& domain,
    int lateral_rounds,
    double lateral_offset);

double interval_point_gap_local(const Interval& interval, double value);

double intervals_point_gap_local(const std::vector<Interval>& intervals,
                                 const Eigen::Ref<const Eigen::VectorXd>& point);

bool intervals_contain_point_local(const std::vector<Interval>& intervals,
                                   const Eigen::Ref<const Eigen::VectorXd>& point,
                                   double tolerance);

bool intervals_contain_point_strict_local(const std::vector<Interval>& intervals,
                                          const Eigen::Ref<const Eigen::VectorXd>& point,
                                          double tolerance = 0.0);

bool intervals_equal_local(const std::vector<Interval>& lhs,
                           const std::vector<Interval>& rhs,
                           double tolerance = 0.0);

std::unordered_map<OracleNodeId, int> build_box_node_index(
    const std::vector<BoxNode>& boxes,
    std::size_t reserve_extra = 0);

int find_box_index_by_node_or_intervals(
    const std::vector<BoxNode>& boxes,
    const std::unordered_map<OracleNodeId, int>& node_to_box_index,
    OracleNodeId node,
    const std::vector<Interval>& intervals,
    double tolerance);

Eigen::VectorXd adaptive_center_of_intervals(const std::vector<Interval>& intervals);

BoxNode adaptive_make_box_from_intervals(const std::vector<Interval>& intervals,
                                         OracleNodeId node,
                                         int id,
                                         BoxSafetyStatus status,
                                         bool strict_audit_required);

std::optional<std::pair<double, double>> segment_box_parameter_interval(
    const Eigen::Ref<const Eigen::VectorXd>& a,
    const Eigen::Ref<const Eigen::VectorXd>& b,
    const BoxNode& box,
    double tolerance);

double certified_box_covered_segment_length(const Eigen::Ref<const Eigen::VectorXd>& a,
                                            const Eigen::Ref<const Eigen::VectorXd>& b,
                                            const std::vector<BoxNode>& boxes,
                                            double tolerance = 1e-9);

double uncovered_segment_edge_length(const SegmentEdge& edge,
                                     const std::vector<BoxNode>& boxes,
                                     double tolerance = 1e-9);

bool same_waypoint(const Eigen::VectorXd& lhs, const Eigen::VectorXd& rhs);

void append_waypoint_unique(std::vector<Eigen::VectorXd>& path,
                            const Eigen::VectorXd& waypoint);

std::vector<Eigen::VectorXd> densify_waypoint_path_local(const std::vector<Eigen::VectorXd>& path,
                                                         double max_step);

bool csv_index_list_contains(const std::string& csv, int value);

int derived_planner_seed(int base_seed,
                         int offset,
                         int attempt = 0,
                         int query_index = 0,
                         int extra = 0);

std::vector<Eigen::VectorXd> best_audited_rrt_bridge_path(
    const Eigen::Ref<const Eigen::VectorXd>& start,
    const Eigen::Ref<const Eigen::VectorXd>& goal,
    const CollisionChecker& checker,
    const Robot& robot,
    StageContext& context,
    const RRTConnectConfig& base_config,
    int attempts,
    double total_timeout_ms,
    int seed_base,
    int audit_resolution,
    double audit_segment_step,
    const QueryBridgeParallelRrtOptions& parallel_options,
    const std::vector<RRTConnectConfig>* attempt_configs = nullptr,
    int seed_stride = kSeedAttemptStride);

} // namespace rbf
