#pragma once

#include <SBF/box_adjacency_types.h>
#include <SBF/find_free_box_types.h>

#include <Eigen/Core>

#include <rbf/core.h>

#include <cstdint>
#include <filesystem>
#include <limits>
#include <vector>

namespace rbf {

enum class GrowTargetType : uint8_t {
	Unknown = 0,
	RootSeed,
	ComponentConnect,
	QueryRoot,
	Unexplored,
	Uniform,
	IntertreeRoot,
};

inline const char* grow_target_type_str(GrowTargetType t) noexcept {
	switch (t) {
		case GrowTargetType::RootSeed:         return "root_seed";
		case GrowTargetType::ComponentConnect: return "component_connect";
		case GrowTargetType::QueryRoot:        return "query_root";
		case GrowTargetType::Unexplored:       return "unexplored";
		case GrowTargetType::Uniform:          return "uniform";
		case GrowTargetType::IntertreeRoot:    return "intertree_root";
		default:                               return "";
	}
}

struct GrowTraceFace {
	bool valid = false;
	int rank = -1;
	bool selected = false;
	bool seed_covered = false;
	int parent_index = -1;
	int parent_box_id = -1;
	int dim = -1;
	int side = 0;
	double face_value = 0.0;
	double score = std::numeric_limits<double>::infinity();
	double priority = std::numeric_limits<double>::infinity();
	int scanned_boxes = 0;
	int scanned_faces = 0;
};

struct GrowTask {
	int task_id = -1;
	int iteration = -1;
	Eigen::VectorXd seed;
	Eigen::VectorXd target;
	GrowTargetType target_type = GrowTargetType::Unknown;
	int parent_box_id = -1;
	int root_id = -1;
	int source_root_id = -1;
	int target_root_id = -1;
	OracleNodeId domain_root_node = kInvalidOracleNodeId;
	int ffb_depth = 0;
	double intertree_goal_bias = 0.0;
	double component_connect_gap_sq = std::numeric_limits<double>::infinity();
	bool component_connect_target = false;
	bool component_connect_staged_target = false;
	int component_pair_unknown_failures = 0;
	std::vector<GrowTraceFace> face_candidates;
	GrowTraceFace selected_face;
};

struct GrowTaskRequest {
	Eigen::VectorXd target;
	Eigen::VectorXd seed;
	GrowTargetType target_type = GrowTargetType::Unknown;
	int parent_box_id = -1;
	int source_root_id = -1;
	int root_id = -1;
	int target_root_id = -1;
	int iteration = -1;
	bool has_seed = false;
	bool intertree = false;
	bool component_connect = false;
	int component_pair_unknown_failures = 0;
	bool component_connect_staged_target = false;
	double component_connect_gap_sq = 0.0;
	GrowTraceFace selected_face;
	std::vector<GrowTraceFace> face_candidates;
};

struct GrowWorkerResult : GrowTask {
	int worker_id = -1;
	bool accepted_by_worker = false;
	FindFreeBoxResult free_box;
};

struct FailureCoolingEntry {
	int fail_count = 0;
	int last_failed_box_count = 0;
	int max_failed_depth = 0;
	int cool_until_box_count = 0;
	int cooled_at_depth = 0;
};

struct GrowerConfig {
	enum class Mode : std::uint8_t {
		RRT = 0,
		Frontwave = 1,
	};
	struct DepthStage {
		int box_limit = 0;
		int ffb_depth = 0;
		int component_connect_ffb_depth_increment = -1;
		int component_connect_ffb_max_depth = 0;
	};
	struct FrontwaveStage {
		int box_limit = 0;
	};

	Mode mode = Mode::RRT;
	BoxCommitPolicy commit_policy = BoxCommitPolicy::CommitCertifiedOnly;
	FindFreeBoxOptions find_free_box;
	int max_boxes = 1000;
	double timeout_ms = 0.0;
	int max_consecutive_miss = 1000;
	int rng_seed = 7;
	double rrt_goal_bias = 0.2;
	double intertree_goal_bias = 0.25;
	double rrt_step_ratio = 0.08;
	double unexplored_sample_prob = 0.45;
	// When true, the per-sample target category is drawn from a single
	// categorical distribution over {component_connect, intertree, rrt_goal,
	// unexplored, uniform}. The configured probabilities are interpreted
	// directly; if they sum to less than 1, the remainder is assigned to the
	// pure-uniform bucket.
	bool sample_categorical_allocation = false;
	double sample_uniform_prob = 0.0;
	bool connect_mode = true;
	double component_connect_prob = 0.0;
	int component_connect_candidate_limit = 4;
	double component_connect_stage_normalized_linf = 0.35;
	bool component_connect_island_aware = true;
	bool component_connect_staged_growth = true;
	bool component_connect_frontier_cache = true;
	int component_connect_max_parent_failures = 0;
	double component_connect_neighbor_root_bias = 0.0;
	int component_connect_neighbor_root_window = 0;
	double component_connect_lateral_sample_prob = 0.0;
	int component_connect_lateral_sample_attempts = 1;
	bool component_connect_require_target_direction = true;
	bool component_connect_adaptive_ffb = true;
	bool component_connect_depth_after_unknown_only = true;
	int component_connect_ffb_depth_increment = 2;
	int component_connect_ffb_max_depth = 0;
	int component_connect_chain_steps = 0;
	int component_connect_chain_max_boxes = 0;
	bool frontier_face_memory = false;
	int frontier_face_bins_per_dim = 4;
	int frontier_face_min_attempts = 1;
	int frontier_face_max_attempts = 12;
	double frontier_face_area_attempt_scale = 16.0;
	int frontier_face_candidate_limit = 128;
	int frontwave_bootstrap_boxes = 0;
	int frontwave_bootstrap_depth = 0;
	int frontwave_bootstrap_boundary_samples = 14;
	int n_boundary_samples = 1;
	double boundary_epsilon = 1e-9;
	bool expand_all_roots_per_sample = false;
	int extra_random_roots = 0;
	int random_anchor_targets = 0;
	double anchor_target_prob = 0.0;
	int anchor_target_candidate_count = 0;
	int anchor_target_max_lca_depth = -1;
	int anchor_wave_targets_per_batch = 0;
	std::vector<Eigen::VectorXd> fixed_anchor_targets;
	bool root_seed_include_user_seeds = true;
	int root_seed_candidate_count = 0;
	double root_seed_min_normalized_linf = 0.0;
	int root_seed_max_lca_depth = -1;
	bool stop_after_connect = false;
	int post_connect_extra_boxes = 0;
	double post_connect_time_budget_ms = 0.0;
	int quality_min_connected_boxes = 0;
	bool coverage_first_stop_loss = false;
	int high_goal_bias_pulse_period = 0;
	double sustained_goal_bias_cap = 0.0;
	bool failure_cooling_enabled = false;
	int failure_cooling_threshold = 3;
	int failure_cooling_box_horizon = 8;
	int failure_cooling_min_depth = 0;
	bool failure_cooling_unknown_only = true;
	bool failure_cooling_retry_on_depth_raise = true;
	int hard_frontier_failure_threshold = 0;
	int hard_frontier_box_horizon = 0;
	std::vector<DepthStage> depth_stages;
	std::vector<FrontwaveStage> frontwave_stages;
	int n_threads = 1;
	int task_batch_size = 0;
	int parallel_threshold = 0;
	bool deterministic_reduce = true;
	bool worker_local_ffb = true;
	double adjacency_tolerance = 1e-9;
	bool trace_enabled = false;
	std::filesystem::path trace_path;
	int trace_max_events = 0;
	int trace_face_candidate_limit = 8;
};

struct GrowerResult {
	std::vector<BoxNode> boxes;
	AdjacencyGraph adjacency;
	bool all_connected = false;
	int adjacency_islands = 0;
	int adjacency_largest_island = 0;
	int n_roots = 0;
	int n_ffb_success = 0;
	int n_ffb_fail = 0;
	int decisions = 0;
	int splits = 0;
	bool deadline_reached = false;
	double build_time_ms = 0.0;
};

}  // namespace rbf
