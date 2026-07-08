#pragma once

#include <SBF/adaptive_leaf_sweep_config.h>
#include <SBF/box_adjacency_types.h>
#include <SBF/connector_types.h>
#include <SBF/find_free_box_types.h>
#include <SBF/leaf_sweep_types.h>
#include <SBF/planning_config.h>
#include <SBF/planning_result.h>
#include <SBF/query_bridge_config.h>
#include <SBF/query_graph_cache_types.h>
#include <SBF/query_result.h>
#include <SBF/query_runtime_config.h>
#include <SBF/runtime_fwd.h>
#include <SBF/scene.h>
#include <SBF/segment_edge_types.h>

#include <chrono>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

namespace rbf::lect_database {
class LectDatabase;
class LectDatabaseEvidenceSource;
class LectExternalEvidenceSource;
class LectReadSnapshot;
class LectSnapshotEvidenceSource;
class OnlineEnvelopeCacheTree;
class SharedEndpointEvidenceCache;
} // namespace rbf::lect_database

namespace rbf {

struct QueryBridgeAcceptanceThresholds;
struct QueryBridgeEdgeRuntimeOptions;
struct QueryBridgeHybridizeAttemptOptions;
struct QueryBridgeParallelRrtOptions;
struct QueryBridgeRetryOptions;
struct QueryBridgeLocalDsu;
struct QueryBridgeSearchTask;
struct ObbPathCoverResult;
struct ObbValidationOptions;
struct AdaptiveDepthSnapshot;
struct BudgetedMergeStats;
#if defined(SBF_DIAGNOSTIC_API) && SBF_DIAGNOSTIC_API
struct DebugChainPaveResult;
struct RebuildProfile;
struct SubtractiveBuildOptions;
struct SubtractiveObstacleGroup;
struct DynamicCollisionCacheState;
struct DynamicCollisionCacheStateDeleter {
	void operator()(DynamicCollisionCacheState* state) const;
};
#endif
class AdaptiveGridPartition;
class DatabaseBoxOracle;

class RBFPlanningForest {
public:
	RBFPlanningForest(Robot robot, RBFPlanningConfig config = {});
	~RBFPlanningForest();

	// Build and coverage construction.
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

	// Query, endpoint repair, and bridge construction.
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
	void clear_forest();

	// Diagnostic-only facade entry points. These are excluded from default
	// builds and public-release source lists unless SBF_DIAGNOSTIC_API is set.
#if defined(SBF_DIAGNOSTIC_API) && SBF_DIAGNOSTIC_API
#include <SBF/detail/planning_forest_diagnostic_public_methods.inc>
#endif

	// State accessors and cache handles.
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
	// FFB and query entry helpers.
#include <SBF/detail/planning_forest_private_entry_methods.inc>

	// Query bridge point-location and direct-corridor helpers.
#include <SBF/detail/planning_forest_private_query_bridge_direct_methods.inc>

	// Adaptive cover build orchestration.
#include <SBF/detail/planning_forest_private_adaptive_build_methods.inc>

	// Query bridge paving, HiPaC, and batch helpers.
#include <SBF/detail/planning_forest_private_query_bridge_batch_methods.inc>

	// Core forest topology and adaptive partition helpers.
#include <SBF/detail/planning_forest_private_topology_methods.inc>
#if defined(SBF_DIAGNOSTIC_API) && SBF_DIAGNOSTIC_API
#include <SBF/detail/planning_forest_diagnostic_partition_methods.inc>
#endif

	// OBB and portal corridor helpers.
#include <SBF/detail/planning_forest_private_overlay_methods.inc>

	// Query cache helpers.
#include <SBF/detail/planning_forest_private_cache_methods.inc>
#if defined(SBF_DIAGNOSTIC_API) && SBF_DIAGNOSTIC_API
#include <SBF/detail/planning_forest_diagnostic_cache_methods.inc>
#endif

	// Core state.
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
#if defined(SBF_DIAGNOSTIC_API) && SBF_DIAGNOSTIC_API
#include <SBF/detail/planning_forest_diagnostic_state.inc>
#endif
	BuildProfile last_build_;
	std::vector<Eigen::VectorXd> last_build_seeds_;
	mutable QueryGraphCache query_cache_;
	mutable bool query_cache_dirty_ = true;
};

}  // namespace rbf
