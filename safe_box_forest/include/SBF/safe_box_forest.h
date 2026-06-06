#pragma once

#include <SBF/connector.h>
#include <SBF/grower.h>
#include <SBF/leaf_sweep_grower.h>
#include <SBF/merger.h>
#include <SBF/oracle.h>
#include <SBF/query.h>
#include <SBF/runtime.h>
#include <SBF/scene.h>

#include <LECTDatabase/online_cache.h>
#include <rbf/lect_database.h>

#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace rbf {

struct DynamicUpdateConfig {
	bool enable_spatial_dirty_region = true;
	double dirty_region_padding = 0.0;
	int dirty_seed_limit = 64;
	int dirty_anchor_limit = 64;
	int local_regrow_box_limit = 0;
	double local_regrow_timeout_ms = 0.0;
	int insertion_leaf_sweep_max_depth = 28;
	int insertion_leaf_sweep_relative_depth = -1;
	bool enable_warm_rebuild_fallback = true;
	bool warm_rebuild_on_empty_forest = true;
	bool warm_rebuild_on_empty_dirty_region = true;
	int warm_rebuild_dirty_box_threshold = 0;
	double warm_rebuild_dirty_box_fraction = 1.0;
	int warm_rebuild_min_local_boxes_added = -1;
};

struct SubtractiveObstacleGroup {
	std::string name;
	std::vector<Obstacle> carving_obstacles;
	std::vector<Obstacle> validation_obstacles;
};

struct SubtractiveBuildOptions {
	bool run_connector = true;
	bool use_validation_obstacles_for_final_scene = true;
};

struct LeafSweepRefineConfig {
	int leaf_start_depth = 10;
	int leaf_max_depth = 18;
	double obstacle_cluster_gap = 1000.0;
	bool use_virtual_topology = true;
	bool parallel_virtual_validation = true;
	bool store_group_results = false;
	int validation_batch_size = 512;
	int leaf_threads = 8;
	double leaf_timeout_ms = 0.0;
	int deep_max_boxes = 600;
	int deep_ffb_depth = 34;
	int domain_seed_cap = 24;
	int domain_success_cap = 8;
	int domain_attempt_cap = 24;
	bool allow_anchor_roots = true;
	double refine_timeout_ms = 800.0;
	bool run_rrt_grower = false;
	int rrt_grower_extra_boxes = 0;
	double rrt_grower_timeout_ms = 0.0;
	double priority_prune_radius = 0.0;
	int collision_overlap_prune_min_depth = -1;
	double collision_overlap_prune_threshold = 0.0;
	double collision_overlap_prune_ratio_threshold = 0.0;
};

struct LectDatabaseRuntimeConfig {
	std::filesystem::path path;
	std::filesystem::path external_evidence_path;
	std::filesystem::path external_evidence_snapshot_path;
	std::vector<Interval> root_intervals_override;
	std::vector<Interval> coverage_intervals_override;
	lect_database::SplitPolicyDescriptor split_policy;
	lect_database::OnlineEnvelopeCacheConfig online_cache;
	bool external_evidence_use_snapshot = true;
	bool external_evidence_auto_build_snapshot = true;
	bool read_only = false;
	bool create_if_missing = true;
	bool verify_identity = true;
	bool replay_journal = true;
	bool propagate_parent_hulls = true;
	bool defer_parent_hull_writes = false;
	bool canonical_mode = true;
	bool checkpoint_after_build = true;
	std::string symmetry_descriptor = "joint_symmetry_native_v1";
	std::uint32_t page_size_bytes = 64u * 1024u;
	std::uint32_t max_resident_pages = 256u;
	int max_tree_depth = 64;
};

struct RBFPlanningConfig {
	RBFPlanningConfig();

	EndpointSourceConfig endpoint_source;
	EnvelopeTypeConfig envelope_type;
	OracleValidationConfig validation;
	GrowerConfig grower;
	MergerConfig merger;
	IslandConnectorConfig connector;
	QueryConfig query;
	LectDatabaseRuntimeConfig database;
	RuntimeConfig runtime;
	DynamicUpdateConfig dynamic_update;
	bool enable_merger = true;
	bool enable_connector = true;
	/// Optional query-bridge chain-pave FFB depth. <=0 reuses connector.pave.
	int query_bridge_pave_depth = 0;
	/// Optional shallow-to-deep query bridge FFB schedule. Empty reuses
	/// connector.pave.adaptive_ffb_depths.
	std::vector<int> query_bridge_adaptive_ffb_depths;

	/// RSS threshold for session-level evidence spill during online cache updates.
	/// 0 = disabled.
	std::size_t database_evidence_spill_rss_threshold_bytes = 0;
	/// Check RSS after this many dirty evidence updates when online spill is enabled.
	std::uint64_t database_evidence_spill_check_interval_updates = 4096;
	/// Optional payload path for mmap-backed evidence spill. Empty = cache path + ".flat_payload".
	std::filesystem::path database_evidence_spill_path;
	/// Also checkpoint the database after an automatic online spill.
	bool database_evidence_spill_checkpoint_after_spill = false;
};

struct RebuildProfile {
	int obstacles_before = 0;
	int obstacles_after = 0;
	int removed_obstacle_index = -1;
	int boxes_before = 0;
	int boxes_after = 0;
	int boxes_removed = 0;
	int boxes_added = 0;
	int raw_boxes_before = 0;
	int raw_boxes_after = 0;
	int raw_boxes_removed = 0;
	int raw_boxes_added = 0;
	int dirty_boxes = 0;
	int dirty_boxes_used = 0;
	int dirty_seed_count = 0;
	int regrow_attempts = 0;
	int bridge_boxes_added = 0;
	int segment_edges_added = 0;
	int rrt_segment_edges_added = 0;
	int point_gap_segment_edges_added = 0;
	int adjacency_islands = 0;
	int collision_cache_boxes_before = 0;
	int collision_cache_boxes_after = 0;
	int collision_cache_candidates = 0;
	int collision_cache_promoted = 0;
	int collision_cache_rejected_collision = 0;
	int collision_cache_rejected_contained = 0;
	int collision_cache_rejected_disconnected = 0;
	bool used_spatial_dirty_region = false;
	bool used_warm_rebuild = false;
	std::string fallback_reason;
	double dirty_region_ms = 0.0;
	double collision_check_ms = 0.0;
	double regrow_ms = 0.0;
	double warm_rebuild_ms = 0.0;
	double adjacency_ms = 0.0;
	double total_ms = 0.0;
	std::unordered_map<std::string, double> diagnostics;
};

/// Profile returned by RBFPlanningForest::run_pure_ffb().
/// Contains timing and oracle diagnostics for a batch of isolated FFB calls,
/// with no RRT / grower / adjacency overhead.
struct PureFfbProfile {
	int n_attempts = 0;  ///< Number of seed points attempted.
	int n_found    = 0;  ///< Number of seeds that produced a free box.
	double total_ms = 0.0;  ///< Wall time for all FFB calls in this batch.
	/// Sum of FindFreeBoxResult::decisions (validate_node calls) across all seeds.
	int sum_decisions = 0;
	/// Sum of FindFreeBoxResult::splits (tree node splits) across all seeds.
	int sum_splits = 0;
	/// Sum of oracle.depth(found_node) for every seed that produced a free box.
	int sum_found_depth = 0;
	std::unordered_map<std::string, double> diagnostics;
};

/// Result of the isolated chain_pave debug entry. Captures the BiRRT bridge
/// polyline and the boxes chain_pave committed along it, so callers can measure
/// how completely the committed boxes tile the connector segment.
struct DebugChainPaveResult {
	std::vector<Eigen::VectorXd> waypoints;             ///< BiRRT bridge polyline.
	std::vector<std::vector<Interval>> committed_boxes; ///< Intervals of boxes chain_pave added.
	std::vector<std::vector<Interval>> all_boxes;       ///< Intervals of EVERY forest box after gap-fill (committed + reused).
	std::vector<DebugBoundaryFfbFailure> boundary_failures;
	std::vector<Interval> start_box;                    ///< Anchor box intervals.
	std::vector<Interval> goal_box;                     ///< Goal-containing box intervals.
	int start_box_id = -1;
	int goal_box_id = -1;
	int added = 0;
	int fast_gap_fill_ffb_calls = 0;
	double fast_gap_fill_ms = 0.0;
	int boundary_ffb_calls = 0;
	int boundary_commits = 0;
	int boundary_reject_not_free = 0;
	int boundary_reject_non_adjacent = 0;
	int boundary_fail_seed_collision = 0;
	int boundary_fail_depth_cap = 0;
	int boundary_fail_unknown_depth_cap = 0;
	int boundary_fail_reserved_depth_cap = 0;
	int boundary_fail_occupied = 0;
	int boundary_fail_deadline = 0;
	int boundary_fail_out_of_domain = 0;
	int boundary_fail_split = 0;
	int boundary_failed_seed_memoized = 0;
	int boundary_skip_failed_seed = 0;
	int boundary_stall = 0;
	int boundary_target_hits = 0;
	bool bridge_found = false;
	bool audit_passed = false;
};

struct LeafSweepRefineResult {
	LeafSweepResult leaf_sweep;
	BuildProfile profile;
	int leaf_free_count = 0;
	int leaf_collision_count = 0;
	int deep_boxes_added = 0;
	int deep_domain_attempts = 0;
	int deep_ffb_success = 0;
	int deep_ffb_fail = 0;
	int deep_commit_rejects = 0;
	int deep_domain_rejects = 0;
	int deep_contained_rejects = 0;
	int deep_adjacency_rejects = 0;
	int deep_anchor_roots_added = 0;
	int rrt_grower_boxes_added = 0;
	int rrt_grower_ffb_success = 0;
	int rrt_grower_ffb_fail = 0;
	double leaf_sweep_ms = 0.0;
	double deep_refine_ms = 0.0;
	double rrt_grower_ms = 0.0;
	double connector_ms = 0.0;
	double total_ms = 0.0;
	std::unordered_map<std::string, double> diagnostics;
};

enum class CorridorRefineMode : std::uint8_t {
	LegacyBridge = 0,
	BoxOnlyLongPath = 1,
};

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
		const std::vector<Eigen::VectorXd>& priority_points = {});
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
	int bridge_query(const Eigen::Ref<const Eigen::VectorXd>& start,
					 const Eigen::Ref<const Eigen::VectorXd>& goal);
	int bridge_query_known_needed(const Eigen::Ref<const Eigen::VectorXd>& start,
								  const Eigen::Ref<const Eigen::VectorXd>& goal);
	std::vector<int> bridge_queries(const std::vector<Eigen::VectorXd>& starts,
									const std::vector<Eigen::VectorXd>& goals);
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
	};

	FindFreeBoxResult find_free_box_in_domain(const Eigen::Ref<const Eigen::VectorXd>& seed,
											  const std::vector<Interval>& domain,
											  StageContext& context,
											  const FindFreeBoxOptions& options);
	QueryResult run_query_internal(const Eigen::Ref<const Eigen::VectorXd>& start,
								   const Eigen::Ref<const Eigen::VectorXd>& goal,
								   bool allow_collision_shortcut) const;
	int bridge_query_with_waypoint_path(const Eigen::Ref<const Eigen::VectorXd>& start,
										const Eigen::Ref<const Eigen::VectorXd>& goal,
										const std::vector<Eigen::VectorXd>& waypoint_path,
										bool short_local_bridge,
										const RRTConnectConfig& bridge_rrt,
										int query_index = -1);
	void reset_oracle(Scene scene);
	void reserve_existing_boxes();
	void rebuild_adjacency();
	void invalidate_query_cache() const;
	const QueryGraphCache& query_cache() const;
	int next_box_id() const;
	void populate_dynamic_collision_cache(const LeafSweepResult& result,
										  int obstacle_count);
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
	std::vector<CachedCollisionBox> dynamic_collision_box_cache_;
	BuildProfile last_build_;
	std::vector<Eigen::VectorXd> last_build_seeds_;
	mutable QueryGraphCache query_cache_;
	mutable bool query_cache_dirty_ = true;
};

}  // namespace rbf
