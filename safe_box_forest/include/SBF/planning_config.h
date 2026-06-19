#pragma once

#include <SBF/connector.h>
#include <SBF/grower.h>
#include <SBF/merger.h>
#include <SBF/oracle.h>
#include <SBF/query.h>
#include <SBF/runtime.h>
#include <SBF/scene.h>

#include <LECTDatabase/online_cache.h>
#include <rbf/lect_database.h>

#include <cstdint>
#include <filesystem>
#include <string>
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

enum class PortalMembershipPolicy : std::uint8_t {
	GlobalForestOnly = 0,
	PortalInteriorIndex = 1,
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
	/// Optional query-bridge FFB start/skip depth. <0 reuses connector.pave.
	int query_bridge_ffb_start_depth = -1;
	/// Optional endpoint-anchor FFB depth for online queries. <=0 reuses
	/// query_bridge_pave_depth, then connector.pave.
	int query_endpoint_anchor_ffb_depth = 0;
	/// Optional ordered endpoint-anchor depth schedule. Empty means use the
	/// single endpoint-anchor depth above.
	std::vector<int> query_endpoint_anchor_ffb_depths;
	bool query_endpoint_point_anchor = false;
	double endpoint_shortlink_max_length = 0.25;
	/// Query-bridge acceptance thresholds used before spending online repair
	/// budget. These are explicit configuration fields rather than process
	/// environment overrides so batch planning remains reproducible.
	double query_bridge_accept_segment_fraction = 0.25;
	double query_bridge_accept_path_ratio = 1.50;
	double query_bridge_accept_path_additive = 0.75;
	double query_bridge_accept_max_path_length = 4.5;
	/// Query-bridge RRT retry policy. The defaults preserve the historical C++
	/// fallback behavior; experiment runners set these explicitly.
	int query_bridge_no_path_retry_attempts = 1;
	bool query_bridge_no_path_retry_stop_on_first_success = false;
	int query_bridge_forced_attempts = 1;
	int query_bridge_attempt_offset = 0;
	int query_bridge_rrt_fixed_iters = 0;
	std::vector<double> query_bridge_local_radius_schedule;
	std::vector<int> query_bridge_no_path_retry_budget_iters;
	std::vector<int> query_bridge_no_path_retry_budget_attempts;
	bool query_bridge_hybridize_attempt_paths = false;
	int query_bridge_hybrid_max_paths = 8;
	int query_bridge_hybrid_max_vertices = 128;
	int query_bridge_hybrid_max_cross_checks = 4096;
	bool query_bridge_parallel_rrt_early_stop = false;
	int query_bridge_parallel_rrt_early_stop_min_successes = 1;
	double query_bridge_parallel_rrt_early_stop_ratio = 1.75;
	double query_bridge_parallel_rrt_early_stop_additive = 0.75;
	/// Query-bridge corridor/runtime policy. These used to be process-wide
	/// environment overrides; keeping them in the planner config makes
	/// multi-run experiments deterministic within one Python process.
	bool query_bridge_scene_reusable_edges = false;
	bool query_bridge_direct_segment_after_rrt = false;
	bool query_bridge_fast_direct_segment_after_rrt = false;
	int query_bridge_fast_direct_random_shortcut_iters = 0;
	double query_bridge_direct_max_length = 6.5;
	double query_bridge_direct_sample_step = 0.01;
	std::vector<double> query_bridge_direct_sample_steps_by_query;
	bool query_bridge_full_residual_overlay_when_connected = false;
	/// Direct corridor adaptive repair policy. Negative values preserve the
	/// C++ fallback derived from the current sample/audit step.
	int query_bridge_adaptive_max_repair_subdivisions = -1;
	double query_bridge_adaptive_fine_step = -1.0;
	int query_bridge_adaptive_max_repair_calls = -1;
	std::vector<int> query_bridge_adaptive_max_repair_calls_by_query;
	/// Query graph search cost policy. Dynamic active-query ownership is passed
	/// through RBFQueryRuntimeOptions, while these fields are stable planner
	/// configuration.
	double query_box_transition_edge_cost_penalty = 0.0;
	double query_box_transition_nonprogress_penalty = 0.0;
	double query_box_transition_line_deviation_penalty = 0.0;
	double query_bridge_edge_cost_penalty = 0.0;
	double query_foreign_edge_cost_penalty = 0.0;
	/// Endpoint membership policy for compressed corridor/portal internals.
	/// The production default is low-risk GlobalForestOnly: start/goal lookup
	/// ignores hidden portal/corridor internals and falls back to local repair.
	/// PortalInteriorIndex is reserved for a future explicit interior index.
	PortalMembershipPolicy portal_membership_policy = PortalMembershipPolicy::GlobalForestOnly;

	/// RSS threshold for session-level evidence spill during online cache updates.
	/// 0 = disabled.
	std::size_t database_evidence_spill_rss_threshold_bytes = 0;
	/// Check RSS after this many dirty evidence updates when online spill is enabled.
	std::uint64_t database_evidence_spill_check_interval_updates = 4096;
	/// Optional payload path for mmap-backed evidence spill. Empty = cache path + ".flat_payload".
	std::filesystem::path database_evidence_spill_path;
};

struct RBFQueryRuntimeOptions {
	/// Stable query id used to distinguish same-query and foreign query repair
	/// edges during graph search. -1 disables query ownership penalties.
	int active_query_index = -1;
};

struct QueryBridgeBatchOptions {
	/// Local indices in the starts/goals batch that should be bridged even if
	/// the current graph query already satisfies the acceptance thresholds.
	std::vector<int> forced_query_indices;
	/// Optional stable query ids, one per starts/goals batch entry. These ids are
	/// stored on query-bridge edges so later graph searches can distinguish
	/// same-query and cross-query reuse without relying on process environment.
	std::vector<int> global_query_indices;
};

struct AdaptiveLeafSweepConfig {
	int shallow_start_depth = 8;
	int shallow_max_depth = 14;
	int target_max_depth = 50;
	double time_budget_ms = 60000.0;
	int node_budget = 50000;
	int threads = 8;
	int validation_batch_size = 512;
	double obstacle_cluster_gap = 1000.0;
	bool use_virtual_topology = true;
	bool parallel_virtual_validation = true;
	bool store_group_results = false;
	bool fast_virtual_checkpoint_mode = false;
	int defer_min_depth = 16;
	double overlap_depth_threshold = 0.05;
	double overlap_depth_min_threshold = 0.01;
	double overlap_depth_decay_per_depth = 0.04;
	double overlap_ratio_threshold = 0.0;
	int seed_probe_count = 4096;
	int seed_probe_rng_seed = 20260607;
	bool seed_promote_uncovered = true;
	int seed_anchor_probe_cap = 256;
	int promotion_interval = 1024;
	bool adaptive_depth_enabled = true;
	int adaptive_depth_min = 13;
	int adaptive_depth_max = 16;
	int adaptive_depth_probe_count = 512;
	int adaptive_depth_anchor_probe_cap = 32;
	int adaptive_depth_probe_seed = 20260607;
	int adaptive_depth_min_free_probes = 64;
	int adaptive_depth_min_covered_probes = 12;
	int adaptive_depth_min_main_probes = 8;
	double adaptive_depth_min_main_ratio = 0.35;
	int adaptive_depth_min_cells = 0;
	int adaptive_depth_min_main_cells = 0;
	int adaptive_depth_max_online_cells = 180;
	double adaptive_depth_max_probe_ms = 5.0;
	double max_merge_ms = 1500.0;
	int max_merge_rounds = 2;
	int max_merge_input_boxes = 100000;
	int max_free_boxes = 50000;
	int max_unresolved_domains = 100000;
	std::string planning_backend = "partition_native";
	int grid_target_depth = 0;
	bool grid_face_index_enabled = true;
	int grid_planning_max_expansions = 0;
	bool hipac_portal_connectivity = false;
	bool hipac_portal_cell_native_validate = true;
	int hipac_portal_max_internal_boxes = 64;
	int hipac_portal_max_recursion_depth = 8;
	int hipac_portal_ffb_depth = 0;
	double hipac_portal_ffb_deadline_ms = 5.0;
	bool hipac_online_connectivity = false;
	bool hipac_online_before_query_bridge = true;
	bool hipac_promote_query_repairs = false;
	double hipac_online_candidate_max_length = 3.0;
	int hipac_online_max_resolves_per_query = 1;
	int hipac_online_max_hidden_boxes_per_portal = 32;
	int hipac_online_max_ffb_calls_per_portal = 64;
	bool hipac_online_prebridge_portal = false;
	int hipac_online_prebridge_candidate_limit = 32;
	double hipac_online_prebridge_max_pair_distance = 1.25;
	double hipac_online_prebridge_route_distance_weight = 1.0;
	double hipac_online_prebridge_pair_distance_weight = 0.25;
	bool hipac_transition_obb_portal = false;
	double hipac_transition_obb_lateral_radius = 0.01;
	double hipac_transition_obb_longitudinal_margin = 0.0;
	double hipac_transition_obb_safety_epsilon = 0.0;
	bool segment_edge_obb_cover = false;
	bool rrt_bridge_obb_cover = false;
	bool strict_obb_bridge_cover = false;
	double segment_edge_obb_lateral_radius = 0.01;
	double segment_edge_obb_longitudinal_margin = 0.0;
	double segment_edge_obb_safety_epsilon = 0.0;
	int segment_edge_obb_grow_iterations = 5;
	int segment_edge_obb_binary_iterations = 5;
	int segment_edge_obb_split_depth = 1;
	int obb_max_window_segments = 16;
	int obb_max_validations_per_window = 16;
	bool obb_fast_primary_orientation = true;
	bool obb_fallback_orientations_on_primary_fail = false;
	bool obb_sampled_support_enabled = false;
	bool obb_clearance_sampled_support_enabled = true;
	double obb_clearance_lateral_l1_max = 5e-3;
	int obb_clearance_samples = 17;
	double obb_clearance_dense_line_l1_threshold = 0.03;
	int obb_clearance_dense_samples = 17;
	int obb_clearance_fast_samples = 0;
	bool obb_clearance_first = false;
	int obb_clearance_retry_attempts = 0;
	std::vector<double> obb_clearance_retry_values;
	int obb_clearance_retry_iters = -1;
	double obb_clearance_retry_timeout_ms = -1.0;
	bool segment_edge_obb_metadata_only = false;
	bool segment_edge_obb_metadata_require_cover = false;
	bool hipac_promote_transition_slices = false;
	std::string hipac_promote_transition_target_query_indices = "2,3";
	int hipac_promote_transition_min_boxes = 8;
	int hipac_promote_transition_max_boxes = 64;
	int hipac_promote_transition_max_attempts_per_query = 1;
};

enum class CorridorRefineMode : std::uint8_t {
	SegmentBridge = 0,
	BoxOnlyLongPath = 1,
};

struct EndpointMainBoxCorridorConfig {
	int target_k = 8;
	double coarse_step = 0.08;
	double fine_step = 0.02;
	int max_ffb_calls = 48;
	int max_boxes = 64;
	double residual_segment_max_length = 0.25;
	double lateral_offset = 0.03;
	int lateral_rounds = 2;
	double face_epsilon = 1e-6;
};

} // namespace rbf
