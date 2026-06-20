#pragma once

#include <SBF/build_config.h>
#include <SBF/connector_types.h>
#include <SBF/grower_types.h>
#include <SBF/merger.h>
#include <SBF/oracle.h>
#include <SBF/query_bridge_config.h>
#include <SBF/query.h>
#include <SBF/runtime.h>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <vector>

namespace rbf {

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

} // namespace rbf
