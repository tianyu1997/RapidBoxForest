#pragma once

#include <SBF/box_adjacency_types.h>
#include <SBF/connector_types.h>
#include <SBF/find_free_box_types.h>
#include <SBF/runtime_fwd.h>

#include <LECTDatabase/sbf/oracle_types.h>

#include <Eigen/Core>

#include <rbf/core.h>

#include <string>
#include <unordered_map>
#include <vector>

namespace rbf {

class CollisionChecker;

int segment_resolution_for_step(const Eigen::Ref<const Eigen::VectorXd>& a,
                                const Eigen::Ref<const Eigen::VectorXd>& b,
                                int base_resolution,
                                double segment_step);
bool check_segment_with_step(const CollisionChecker& checker,
                             const Eigen::Ref<const Eigen::VectorXd>& a,
                             const Eigen::Ref<const Eigen::VectorXd>& b,
                             int base_resolution,
                             double segment_step);
bool allow_connector_box_commit(BoxOracle& oracle,
                                FindFreeBoxResult& result,
                                BoxCommitPolicy policy,
                                StageContext& context);
RRTConnectConfig with_query_root_hull_domain(const RRTConnectConfig& config,
                                             BoxOracle& oracle,
                                             const Eigen::Ref<const Eigen::VectorXd>& start,
                                             const Eigen::Ref<const Eigen::VectorXd>& goal);
bool graph_has_path(const AdjacencyGraph& graph, int source_box_id, int target_box_id);
void append_graph_edge_unique(AdjacencyGraph& graph, int lhs, int rhs);
int connect_new_boxes_to_island(std::vector<BoxNode>& boxes,
                                AdjacencyGraph& graph,
                                int first_new_box_id,
                                int next_box_id,
                                const std::vector<int>& target_island,
                                double tolerance);
std::vector<Eigen::VectorXd> densify_path_by_step(const std::vector<Eigen::VectorXd>& path,
                                                  double max_step);
std::unordered_map<int, const BoxNode*> make_box_map(const std::vector<BoxNode>& boxes);
double interval_max_gap(const BoxNode& lhs, const BoxNode& rhs);
double interval_point_gap(const Interval& interval, double value);
double intervals_point_gap(const std::vector<Interval>& intervals,
                           const Eigen::Ref<const Eigen::VectorXd>& point);
std::vector<Eigen::VectorXd> closest_box_point_segment(const BoxNode& source,
                                                       const BoxNode& target,
                                                       const CollisionChecker& checker,
                                                       int segment_resolution,
                                                       double segment_step = 0.0);
bool intervals_contain_point(const std::vector<Interval>& intervals,
                             const Eigen::Ref<const Eigen::VectorXd>& point,
                             double tolerance);
double box_gap_squared(const BoxNode& lhs, const BoxNode& rhs);
double point_box_gap_squared(const Eigen::Ref<const Eigen::VectorXd>& point, const BoxNode& box);
bool box_contains_point_exact(const BoxNode& box, const Eigen::Ref<const Eigen::VectorXd>& point);
bool point_covered_by_existing_box(const std::vector<BoxNode>& boxes,
                                   const Eigen::Ref<const Eigen::VectorXd>& point);
double boundary_max_depth_failure_count(const StageContext& context);
bool can_step_outside_face(const BoxNode& box,
                           const std::vector<Interval>& root,
                           int dim,
                           int side,
                           double epsilon);
double face_seed_score(const BoxNode& box,
                       const std::vector<Interval>& root,
                       const Eigen::Ref<const Eigen::VectorXd>& target,
                       int face_dim,
                       int side,
                       double epsilon);
Eigen::VectorXd make_face_seed(const BoxNode& box,
                               const std::vector<Interval>& root,
                               const Eigen::Ref<const Eigen::VectorXd>& target,
                               int face_dim,
                               int side,
                               double epsilon);
void set_max_diagnostic(StageContext& context, const std::string& key, double value);
std::vector<BridgePairTask> broadphase_bridge_pairs(const std::unordered_map<int, const BoxNode*>& map,
                                                    const std::vector<int>& lhs_ids,
                                                    const std::vector<int>& rhs_ids,
                                                    int result_limit,
                                                    int window_hint);
std::vector<BridgePairTask> interval_gap_broadphase_pairs(const std::unordered_map<int, const BoxNode*>& map,
                                                          const std::vector<int>& lhs_ids,
                                                          const std::vector<int>& rhs_ids,
                                                          double gap_tolerance);

}  // namespace rbf
