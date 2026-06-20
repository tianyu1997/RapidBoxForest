#pragma once

#include <SBF/adaptive_grid_partition.h>
#include <SBF/box_graph.h>
#include <SBF/connector.h>
#include <SBF/debug.h>
#include <SBF/grower.h>
#include <SBF/leaf_sweep_grower.h>
#include <SBF/merger.h>
#include <SBF/oracle.h>
#include <SBF/planning_config.h>
#include <SBF/planning_result.h>
#include <SBF/query.h>
#include <SBF/runtime.h>
#include <SBF/scene.h>

#include <LECTDatabase/online_cache.h>
#include <rbf/lect_database.h>

#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace rbf {

struct QueryBridgeAcceptanceThresholds;
struct QueryBridgeRetryOptions;
struct QueryBridgeSearchTask;
struct ObbPathCoverResult;
struct ObbValidationOptions;

class RBFPlanningForest {
public:
	RBFPlanningForest(Robot robot, RBFPlanningConfig config = {});

	BuildProfile build(const Eigen::Ref<const Eigen::VectorXd>& start,
					   const Eigen::Ref<const Eigen::VectorXd>& goal,
					   const std::vector<Obstacle>& obstacles);
	BuildProfile build(const Eigen::Ref<const Eigen::VectorXd>& start,
					   const Eigen::Ref<const Eigen::VectorXd>& goal,
					   const std::vector<Obstacle>& obstacles,
					   StageContext& context);
	BuildProfile build_coverage(const std::vector<Obstacle>& obstacles,
								const std::vector<Eigen::VectorXd>& seeds);
	BuildProfile build_coverage(const std::vector<Obstacle>& obstacles,
								const std::vector<Eigen::VectorXd>& seeds,
								StageContext& context);
	BuildProfile build_subtractive(const std::vector<SubtractiveObstacleGroup>& obstacle_groups,
								   const std::vector<Eigen::VectorXd>& seeds,
								   const SubtractiveBuildOptions& options = {});
	LeafSweepResult build_leaf_sweep(const std::vector<Obstacle>& obstacles,
									 int start_depth,
									 int max_depth,
									 const LeafSweepConfig& leaf_sweep_config = {});
	LeafSweepResult build_leaf_sweep(const std::vector<Obstacle>& obstacles,
									 int start_depth,
									 int max_depth,
									 const LeafSweepConfig& leaf_sweep_config,
									 StageContext& context);
	LeafSweepRefineResult build_leaf_sweep_refined(
		const std::vector<Obstacle>& obstacles,
		const LeafSweepRefineConfig& refine_config = {},
		const std::vector<Eigen::VectorXd>& priority_points = {},
		const std::vector<Eigen::VectorXd>& offline_anchor_points = {});
	AdaptiveLeafSweepResult build_adaptive_deep_leaf_sweep_cover(
		const std::vector<Obstacle>& obstacles,
		const AdaptiveLeafSweepConfig& config = {});
	/// Isolated FFB benchmark — no RRT, no grower, no adjacency.
	/// Resets the oracle scene to @p obstacles, then calls ffb.find() once per
	/// seed.  LECT evidence accumulates across repeated calls (same as build()).
	/// Use this to measure the pure oracle / LECT-cache performance.
	PureFfbProfile run_pure_ffb(const std::vector<Obstacle>& obstacles,
	                            const std::vector<Eigen::VectorXd>& seeds);
	/// Access oracle counters from the most recent build/run_pure_ffb call.
	/// Returns nullptr if the oracle hasn't been initialised yet.
	const OracleCounters* oracle_counters() const {
		return oracle_ ? &oracle_->counters() : nullptr;
	}
	QueryResult query(const Eigen::Ref<const Eigen::VectorXd>& start,
					  const Eigen::Ref<const Eigen::VectorXd>& goal) const;
	QueryResult query(const Eigen::Ref<const Eigen::VectorXd>& start,
					  const Eigen::Ref<const Eigen::VectorXd>& goal,
					  const RBFQueryRuntimeOptions& runtime_options) const;
	int anchor_query_endpoint(const Eigen::Ref<const Eigen::VectorXd>& point);
	int connect_query_endpoint_to_main_island(const Eigen::Ref<const Eigen::VectorXd>& point,
											  double max_segment_length);
	int connect_query_endpoint_to_main_box_corridor(
		const Eigen::Ref<const Eigen::VectorXd>& point,
		const EndpointMainBoxCorridorConfig& corridor_config = {});
	int add_offline_shortcut_edges(int max_edges,
	                               int candidate_limit,
	                               double min_gain_ratio,
	                               double max_segment_length,
	                               bool allow_segment_fallback = false);
	int bridge_query(const Eigen::Ref<const Eigen::VectorXd>& start,
					 const Eigen::Ref<const Eigen::VectorXd>& goal);
	int bridge_query_known_needed(const Eigen::Ref<const Eigen::VectorXd>& start,
								  const Eigen::Ref<const Eigen::VectorXd>& goal);
	std::vector<int> bridge_queries(const std::vector<Eigen::VectorXd>& starts,
									const std::vector<Eigen::VectorXd>& goals);
	std::vector<int> bridge_queries(const std::vector<Eigen::VectorXd>& starts,
									const std::vector<Eigen::VectorXd>& goals,
									const QueryBridgeBatchOptions& options);
	/// Isolated chain_pave debug entry: build the BiRRT bridge between the boxes
	/// containing @p start and @p goal, then run chain_pave_along_path() with the
	/// supplied @p pave config (the IslandConnector gap step is skipped) so the
	/// committed boxes reflect chain_pave alone. Mutates the forest box set.
	DebugChainPaveResult debug_chain_pave(const Eigen::Ref<const Eigen::VectorXd>& start,
										  const Eigen::Ref<const Eigen::VectorXd>& goal,
										  const ChainPaveConfig& pave);
	DebugChainPaveResult debug_chain_pave_waypoints(const std::vector<Eigen::VectorXd>& waypoint_path,
													const ChainPaveConfig& pave);
	int refine_query_corridor(const Eigen::Ref<const Eigen::VectorXd>& start,
							  const Eigen::Ref<const Eigen::VectorXd>& goal,
							  int max_boxes_to_add);
	int refine_query_corridor(const Eigen::Ref<const Eigen::VectorXd>& start,
							  const Eigen::Ref<const Eigen::VectorXd>& goal,
							  int max_boxes_to_add,
							  CorridorRefineMode mode,
							  double long_path_ratio,
							  double long_path_min_delta);
	RebuildProfile add_obstacle_and_rebuild(const Obstacle& obstacle);
	RebuildProfile add_obstacles_and_rebuild(const std::vector<Obstacle>& obstacles);
	RebuildProfile connect_update_segment_fallback();
	RebuildProfile connect_update_endpoint_segment_fallback(const Eigen::Ref<const Eigen::VectorXd>& start,
															const Eigen::Ref<const Eigen::VectorXd>& goal);
	RebuildProfile remove_obstacle_and_regrow(int obstacle_index);
	RebuildProfile remove_obstacle_suffix_and_regrow(int target_obstacle_count);
	void clear_forest();

	const Robot& robot() const { return robot_; }
	const Robot& audit_robot() const { return audit_robot_; }
	const RBFPlanningConfig& config() const { return config_; }
	const Scene& scene() const { return scene_; }
	const std::vector<BoxNode>& boxes() const { return boxes_; }
	const std::vector<BoxNode>& raw_boxes() const { return raw_boxes_; }
	const AdjacencyGraph& adjacency() const { return adjacency_; }
	const SegmentEdgeList& segment_edges() const { return segment_edges_; }
	const BuildProfile& last_build_profile() const { return last_build_; }
	lect_database::LectDatabase& database() { return *database_; }
	const lect_database::LectDatabase& database() const { return *database_; }
	lect_database::OnlineEnvelopeCacheTree& online_cache() { return *online_cache_; }
	const lect_database::OnlineEnvelopeCacheTree& online_cache() const { return *online_cache_; }

private:
	struct CachedCollisionBox {
		BoxNode box;
		std::vector<int> blocking_obstacle_indices;
		bool active = true;
	};

	FindFreeBoxResult find_free_box_in_domain(const Eigen::Ref<const Eigen::VectorXd>& seed,
											  const std::vector<Interval>& domain,
											  StageContext& context,
											  const FindFreeBoxOptions& options);
	FindFreeBoxResult find_free_box_binary_in_domain(
		const Eigen::Ref<const Eigen::VectorXd>& seed,
		const std::vector<Interval>& domain,
		StageContext& context,
		const FindFreeBoxOptions& options,
		const OracleSplitOptions& split_options,
		int effective_max_depth,
		std::chrono::steady_clock::time_point start);
	QueryResult run_query_internal(const Eigen::Ref<const Eigen::VectorXd>& start,
								   const Eigen::Ref<const Eigen::VectorXd>& goal,
								   bool allow_collision_shortcut,
								   const RBFQueryRuntimeOptions& runtime_options = {}) const;
	int anchor_query_endpoint_box(const Eigen::Ref<const Eigen::VectorXd>& point,
								  StageContext& context);
	int anchor_query_endpoint_box_with_diagnostics(const Eigen::Ref<const Eigen::VectorXd>& point);
	int locate_box_partition_first(const Eigen::Ref<const Eigen::VectorXd>& point,
								   bool nearest_if_outside) const;
	int locate_query_bridge_box(const Eigen::Ref<const Eigen::VectorXd>& point) const;
	bool query_bridge_box_contains_point(int box_id,
										 const Eigen::Ref<const Eigen::VectorXd>& point) const;
	int refresh_query_bridge_box_or_anchor(int anchor_box_id,
										   const Eigen::Ref<const Eigen::VectorXd>& point,
										   const char* endpoint_name);
	void sync_query_bridge_partition_boxes(std::size_t& partition_refresh_base,
										   const char* diagnostic_prefix);
	bool box_only_path_connected_partition_first(int source_box_id,
												 int target_box_id) const;
	bool overlay_path_connected_partition_first(int source_box_id,
											   int target_box_id) const;
	bool partition_native_mode() const;
	int island_count_partition_first() const;
	int bridge_query_with_waypoint_path(const Eigen::Ref<const Eigen::VectorXd>& start,
										const Eigen::Ref<const Eigen::VectorXd>& goal,
										const std::vector<Eigen::VectorXd>& waypoint_path,
										bool short_local_bridge,
										const RRTConnectConfig& bridge_rrt,
										int query_index = -1,
										bool allow_residual_segments = true);
	int try_query_bridge_direct_ffb_corridor(const Eigen::Ref<const Eigen::VectorXd>& start,
											 const Eigen::Ref<const Eigen::VectorXd>& goal,
											 const std::vector<Eigen::VectorXd>& corridor_path,
											 const RRTConnectConfig& bridge_rrt,
											 CollisionChecker& checker,
											 StageContext& context,
											 int query_index,
											 int bridge_edge_query_index,
											 int query_bridge_ffb_depth,
											 double audited_bridge_length,
											 bool allow_residual_segments,
											 int& next_id);
	int try_promote_query_bridge_direct_transition(
		int source_box_id,
		int target_box_id,
		const std::vector<std::vector<int>>& sample_layers,
		std::size_t boxes_before_direct_corridor,
		StageContext& context,
		int query_index,
		int bridge_edge_query_index,
		const char* reason,
		bool& attempted);
		int try_add_query_box_corridor_edge(int source_box_id,
											int target_box_id,
											const std::vector<Eigen::VectorXd>& waypoint_path,
											double segment_resolution,
											int query_index);
		int add_verified_query_box_corridor_edge(int source_box_id,
												 int target_box_id,
												 const std::vector<Eigen::VectorXd>& waypoint_path,
												 double segment_resolution,
												 int query_index);
		int try_add_query_direct_segment_after_rrt_edge(int source_box_id,
														int target_box_id,
													const std::vector<Eigen::VectorXd>& waypoint_path,
													const RRTConnectConfig& bridge_rrt,
													const CollisionChecker& checker,
													StageContext& context,
													double original_path_length,
													double audited_path_length,
													int query_index,
													bool enabled);
	int try_add_query_direct_start_goal_segment_edge(
		int source_box_id,
		int target_box_id,
		const Eigen::Ref<const Eigen::VectorXd>& start,
		const Eigen::Ref<const Eigen::VectorXd>& goal,
		StageContext& context,
		int query_index,
		int batch_task_index = -1);
	int try_add_query_direct_start_goal_segment_for_points(
		const Eigen::Ref<const Eigen::VectorXd>& start,
		const Eigen::Ref<const Eigen::VectorXd>& goal,
		StageContext& context,
		int query_index,
		int batch_task_index = -1);
	int try_add_query_fast_direct_segment_after_rrt_edge(
		int source_box_id,
		int target_box_id,
		const std::vector<std::vector<Eigen::VectorXd>>& candidate_paths,
		const RRTConnectConfig& bridge_rrt,
		StageContext& context,
		int query_index,
		int batch_task_index = -1);
	int try_add_query_fast_direct_segment_after_rrt_path(
		const Eigen::Ref<const Eigen::VectorXd>& start,
		const Eigen::Ref<const Eigen::VectorXd>& goal,
		const std::vector<Eigen::VectorXd>& waypoint_path,
		const RRTConnectConfig& bridge_rrt,
		StageContext& context,
		bool enabled,
		int random_shortcut_iters,
		int shortcut_query_index,
		int edge_query_index,
		int batch_task_index = -1);
		int try_add_query_residual_segment_edge(int source_box_id,
												int target_box_id,
												const std::vector<Eigen::VectorXd>& waypoint_path,
											const RRTConnectConfig& bridge_rrt,
											const CollisionChecker& checker,
											StageContext& context,
											double depth_failures_before,
											int query_index,
											bool enabled);
	int try_add_query_direct_corridor_full_residual_edge(
		int source_box_id,
		int target_box_id,
		const std::vector<Eigen::VectorXd>& waypoint_path,
		const RRTConnectConfig& bridge_rrt,
		const CollisionChecker& checker,
		StageContext& context,
		int edge_query_index,
		int batch_task_query_index,
		bool local_overlay_connected,
		bool count_without_local_overlay_attempt);
	AdaptiveLeafSweepResult build_fixed_virtual_leaf_sweep_cover(
		const std::vector<Obstacle>& obstacles,
		const AdaptiveLeafSweepConfig& adaptive_config,
		int initial_leaf_depth,
		int adaptive_depth_min,
		int target_leaf_depth,
		LeafSweepConfig leaf_config,
		const AdaptiveLeafSweepConfig& partition_config,
		std::chrono::steady_clock::time_point total_start);
	AdaptiveLeafSweepResult build_adaptive_fast_virtual_checkpoint_cover(
		const std::vector<Obstacle>& obstacles,
		const AdaptiveLeafSweepConfig& adaptive_config,
		int initial_leaf_depth,
		int adaptive_depth_min,
		int target_leaf_depth,
		LeafSweepConfig leaf_config,
		const AdaptiveLeafSweepConfig& partition_config,
		std::chrono::steady_clock::time_point total_start);
	std::pair<int, int> locate_query_bridge_boxes(const Eigen::Ref<const Eigen::VectorXd>& start,
												  const Eigen::Ref<const Eigen::VectorXd>& goal,
												  StageContext& context);
	int run_query_bridge_chain_pave(const std::vector<Eigen::VectorXd>& waypoint_path,
									int start_box_id,
									int& next_id,
									StageContext& context,
									const ChainPaveConfig& pave_config,
									const char* partition_prefix);
	bool skip_graph_query_bridge_pave_if_partition_native(StageContext& context,
														  const char* counter_name) const;
	std::pair<int, int> run_query_bridge_reverse_boundary_pave(
		const Eigen::Ref<const Eigen::VectorXd>& start,
		const Eigen::Ref<const Eigen::VectorXd>& goal,
		const std::vector<Eigen::VectorXd>& waypoint_path,
		const ChainPaveConfig& forward_config,
		int forward_added,
		int& accumulated_added,
		int& next_id,
		StageContext& context);
	void refresh_query_bridge_direct_corridor_partition(std::size_t boxes_before);
	int finish_query_bridge_direct_corridor(std::size_t boxes_before, int value);
	int add_partition_box_corridor_overlay(const Eigen::Ref<const Eigen::VectorXd>& start,
										   const Eigen::Ref<const Eigen::VectorXd>& goal,
										   const std::vector<Eigen::VectorXd>& waypoint_path,
										   const char* diagnostic_prefix,
										   bool anchor_endpoints,
										   bool skip_if_connected,
										   int query_index = -1,
										   BuildProfile* profile = nullptr);
	int add_partition_portal_corridor_overlay(const Eigen::Ref<const Eigen::VectorXd>& start,
											  const Eigen::Ref<const Eigen::VectorXd>& goal,
											  const std::vector<Eigen::VectorXd>& waypoint_path,
											  const char* diagnostic_prefix,
											  bool anchor_endpoints,
											  bool skip_if_connected,
											  int query_index = -1,
											  BuildProfile* profile = nullptr);
	int try_hipac_online_bridge_task(
		QueryBridgeSearchTask& task,
		const QueryBridgeAcceptanceThresholds& bridge_acceptance,
		StageContext& context,
		int query_index);
	int try_hipac_prebridge_portal_task(
		QueryBridgeSearchTask& task,
		const QueryBridgeAcceptanceThresholds& bridge_acceptance,
		StageContext& context,
		int query_index);
	bool run_query_bridge_hipac_online_sequence_task(
		QueryBridgeSearchTask& task,
		int& added_for_task,
		StageContext& context,
		bool scene_reusable_edges,
		const QueryBridgeAcceptanceThresholds& bridge_acceptance);
	std::vector<int> finish_query_bridge_batch_result(
		const std::vector<int>& added_by_query,
		std::size_t partition_refresh_base,
		std::size_t segment_edges_before_partition_refresh,
		bool oracle_counters_before_valid,
		const OracleCounters& oracle_counters_before);
	void run_query_bridge_direct_start_goal_segments(
		std::vector<QueryBridgeSearchTask>& tasks,
		std::vector<int>& added_by_query,
		StageContext& context,
		bool scene_reusable_edges);
	int run_query_bridge_waypoint_path(
		QueryBridgeSearchTask& task,
		int& added_for_task,
		StageContext& context,
		bool scene_reusable_edges);
		void finish_query_bridge_ready_waypoint_task(
			QueryBridgeSearchTask& task,
			int& added_for_task,
			bool forced_task,
			double best_length,
			StageContext& context,
		bool scene_reusable_edges,
		const std::unordered_set<int>& forced_query_indices,
		const QueryBridgeAcceptanceThresholds& bridge_acceptance,
		bool fast_direct_segment_after_rrt,
		int fast_direct_random_shortcut_iters,
		const std::function<double()>& task_elapsed_ms);
	int try_promote_query_repair_to_hipac(
		const Eigen::Ref<const Eigen::VectorXd>& start,
		const Eigen::Ref<const Eigen::VectorXd>& goal,
		const std::vector<Eigen::VectorXd>& waypoint_path,
		int bridge_added,
		int query_index,
		int batch_task_index,
		StageContext& context);
	void reset_oracle(Scene scene);
	void reserve_existing_boxes();
	void rebuild_adjacency();
	void rebuild_adaptive_partition(const AdaptiveLeafSweepConfig& config, BuildProfile* profile);
	void refresh_adaptive_partition_diagnostics(BuildProfile* profile) const;
	void refresh_dynamic_partition_after_update(RebuildProfile& profile,
												const char* diagnostic_prefix);
	void refresh_dynamic_partition_after_append(RebuildProfile& profile,
												std::size_t first_box_index,
												const char* diagnostic_prefix);
	void refresh_dynamic_partition_after_remove_append(RebuildProfile& profile,
													   const std::unordered_set<int>& removed_box_ids,
													   std::size_t first_box_index,
													   const char* diagnostic_prefix);
	int append_adaptive_partition_boxes(std::size_t first_box_index,
										 BuildProfile* profile,
										 const char* diagnostic_prefix);
	int sync_adaptive_partition_segment_edges(BuildProfile* profile,
											 const char* diagnostic_prefix);
	int add_segment_edge_partition_first(int source_box_id,
										 int target_box_id,
										 std::vector<Eigen::VectorXd> waypoints,
										 SegmentEdgeType type,
										 int segment_resolution,
										 SegmentEdgeValidation validation,
										 bool strict_audit_required = false,
										 int query_index = -1,
										 BuildProfile* profile = nullptr,
										 const char* diagnostic_prefix = nullptr);
	int try_add_clearance_retry_obb_edge(
		int source_box_id,
		int target_box_id,
		const BoxNode& source_box,
		const BoxNode& target_box,
		const std::vector<Eigen::VectorXd>& waypoints,
		const ObbValidationOptions& obb_validation_options,
		double obb_safety_epsilon,
		const std::string& diagnostic_prefix,
		const std::string& obb_diag,
		int query_index,
		BuildProfile* profile,
		ObbPathCoverResult& cover);
	bool build_cell_native_portal_corridor_chain(
		const BoxNode& source_box,
		const BoxNode& target_box,
		const std::vector<Eigen::VectorXd>& waypoint_path,
		const std::vector<Interval>& domain,
		const std::string& diagnostic_prefix,
		int requested_depth,
		int max_internal_boxes,
		int max_recursion_depth,
		double adjacency_tolerance,
		BuildProfile* profile,
		std::vector<BoxNode>& internal_boxes,
		int& next_internal_id);
	bool build_ffb_portal_corridor_chain(
		const BoxNode& source_box,
		const BoxNode& target_box,
		const std::vector<Eigen::VectorXd>& waypoint_path,
		const std::vector<Interval>& domain,
		const std::string& diagnostic_prefix,
		int requested_depth,
		int max_internal_boxes,
		int max_recursion_depth,
		double adjacency_tolerance,
		bool online_portal,
		BuildProfile* profile,
		StageContext& context,
		std::vector<BoxNode>& internal_boxes,
		int& next_internal_id);
	void invalidate_query_cache() const;
	const QueryGraphCache& query_cache() const;
	int next_box_id() const;
	void populate_dynamic_collision_cache(const LeafSweepResult& result,
										  int obstacle_count);
	void clear_dynamic_collision_cache();
	void rebuild_dynamic_collision_cache_index();
	void add_dynamic_collision_cache_box(const BoxNode& box,
										 std::vector<int> blocking_obstacle_indices);
	int promote_unblocked_collision_cache(const std::unordered_set<int>& removed_obstacle_indices,
										  RebuildProfile& profile);
	int refill_removed_box_with_leaf_sweep(const BoxNode& removed_box,
										   int new_obstacle_index,
										   int max_depth,
										   int& next_id,
										   RebuildProfile& profile);

	Robot robot_;
	Robot audit_robot_;
	RBFPlanningConfig config_;
	Scene scene_;
	std::unique_ptr<lect_database::LectDatabase> database_;
	std::unique_ptr<lect_database::LectDatabase> external_evidence_database_;
	std::unique_ptr<lect_database::LectDatabaseEvidenceSource> external_evidence_database_source_;
	std::unique_ptr<lect_database::LectReadSnapshot> external_evidence_snapshot_;
	std::unique_ptr<lect_database::LectSnapshotEvidenceSource> external_evidence_snapshot_source_;
	const lect_database::LectExternalEvidenceSource* external_evidence_source_ = nullptr;
	const lect_database::LectDatabase* direct_external_evidence_database_ = nullptr;
	std::unique_ptr<lect_database::OnlineEnvelopeCacheTree> online_cache_;
	std::unique_ptr<DatabaseBoxOracle> oracle_;
	// Persists endpoint evidence across oracle resets / queries (endpoints are
	// scene-independent robot-link envelopes). Bounded by the oracle validation
	// config to guard against unbounded growth (OOM).
	std::shared_ptr<lect_database::SharedEndpointEvidenceCache> shared_endpoint_cache_;
	std::vector<BoxNode> boxes_;
	std::vector<BoxNode> raw_boxes_;
	AdjacencyGraph adjacency_;
	SegmentEdgeList segment_edges_;
	std::unique_ptr<AdaptiveGridPartition> adaptive_partition_;
	bool adaptive_partition_query_enabled_ = false;
	bool has_adaptive_partition_config_ = false;
	AdaptiveLeafSweepConfig last_adaptive_partition_config_;
	std::vector<CachedCollisionBox> dynamic_collision_box_cache_;
	std::unordered_map<int, std::vector<std::size_t>> dynamic_collision_cache_blocker_index_;
	int dynamic_collision_cache_active_count_ = 0;
	BuildProfile last_build_;
	std::vector<Eigen::VectorXd> last_build_seeds_;
	mutable QueryGraphCache query_cache_;
	mutable bool query_cache_dirty_ = true;
};

}  // namespace rbf
