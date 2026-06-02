#pragma once

#include <SBF/box_graph.h>
#include <SBF/find_free_box.h>
#include <SBF/runtime.h>

#include <Eigen/Core>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <ostream>
#include <random>
#include <string>
#include <unordered_map>
#include <unordered_set>
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

class IGrower {
public:
	virtual ~IGrower() = default;
	virtual GrowerResult grow(const std::vector<Eigen::VectorXd>& seeds) = 0;
	virtual GrowerResult grow(const std::vector<Eigen::VectorXd>& seeds, StageContext& context) = 0;
};

class RrtGrower final : public IGrower {
public:
	RrtGrower(BoxOracle& oracle, GrowerConfig config = {});
	GrowerResult grow(const std::vector<Eigen::VectorXd>& seeds) override;
	GrowerResult grow(const std::vector<Eigen::VectorXd>& seeds, StageContext& context) override;

private:
	void open_trace();
	void close_trace();
	bool trace_enabled() const;
	void write_trace_event(const std::string& event,
						   const std::function<void(std::ostream&)>& write_fields) const;
	void trace_root_seed(int iteration,
						 int root_id,
						 const Eigen::Ref<const Eigen::VectorXd>& seed) const;
	void trace_task_plan(const GrowTask& task) const;
	void trace_ffb_result(const std::string& event,
						  const Eigen::Ref<const Eigen::VectorXd>& seed,
						  const FindFreeBoxResult& ffb_result,
						  int parent_box_id,
						  int root_id,
						  const GrowTask* task,
						  const GrowWorkerResult* worker_result,
						  int worker_id,
						  int ffb_depth) const;
	void trace_box_added(const BoxNode& box,
						 const GrowTask* task,
						 const GrowWorkerResult* worker_result,
						 int worker_id) const;
	void trace_box_rejected(const std::string& reason,
							const Eigen::Ref<const Eigen::VectorXd>& seed,
							int parent_box_id,
							int root_id,
							const GrowTask* task,
							const GrowWorkerResult* worker_result,
							int worker_id,
							const FindFreeBoxResult* ffb_result = nullptr) const;
	std::vector<Eigen::VectorXd> select_initial_roots(const std::vector<Eigen::VectorXd>& seeds,
													  StageContext& context);
	int create_box(const Eigen::VectorXd& seed,
				   int parent_box_id,
				   int root_id,
				   std::vector<BoxNode>& boxes,
				   FindFreeBoxService& ffb,
				   StageContext& context,
				   const FindFreeBoxOptions* override_options,
				   const GrowTask* trace_task = nullptr,
				   int worker_id = -1,
				   FindFreeBoxResult* observed_result = nullptr);
	int commit_box(const Eigen::VectorXd& seed,
				   FindFreeBoxResult ffb_result,
				   int parent_box_id,
				   int root_id,
				   std::vector<BoxNode>& boxes,
				   StageContext& context,
				   const GrowWorkerResult* trace_result = nullptr);
	std::vector<GrowWorkerResult> run_worker_ffb_tasks(const std::vector<GrowTask>& tasks,
													   const FindFreeBoxOptions& base_options,
													   int depth_stage_index,
													   StageContext& context);
	int resolve_task_batch_size(const std::vector<BoxNode>& boxes, StageContext& context) const;
	std::vector<GrowTask> make_growth_tasks(const std::vector<BoxNode>& boxes,
											const std::vector<Eigen::VectorXd>& roots,
											int first_task_id,
											int n_tasks,
											const FindFreeBoxOptions& base_options,
											StageContext& context);
	bool seed_covered_by_frontier_cache(const std::vector<BoxNode>& boxes,
										const Eigen::Ref<const Eigen::VectorXd>& seed,
										StageContext* context = nullptr) const;
	bool best_uncovered_directed_face_score(const std::vector<BoxNode>& boxes,
											const BoxNode& parent,
											const Eigen::Ref<const Eigen::VectorXd>& target,
											double& best_score,
											StageContext* context = nullptr) const;
	Eigen::VectorXd staged_component_target(const BoxNode& parent,
											const Eigen::Ref<const Eigen::VectorXd>& target,
											bool& staged,
											double& normalized_linf) const;
	int component_pair_unknown_failures(int source_root_id, int target_root_id) const;
	void record_component_connect_result(int source_root_id,
										 int target_root_id,
										 bool success,
										 const FindFreeBoxResult* ffb_result,
										 StageContext& context);
	int grow_component_connect_chain(std::vector<BoxNode>& boxes,
									 FindFreeBoxService& ffb,
									 const FindFreeBoxOptions& base_options,
									 int depth_stage_index,
									 int source_root_id,
									 StageContext& context);
	bool make_component_connect_seed(const std::vector<BoxNode>& boxes,
									 Eigen::VectorXd& seed,
									 Eigen::VectorXd& target,
									 int& parent_box_id,
									 int& root_id,
									 int& target_root_id,
									 int& pair_unknown_failures,
									 bool& staged_target,
									 double& component_gap_sq,
									 StageContext& context);
	bool make_component_connect_seed_for_root(const std::vector<BoxNode>& boxes,
											  int source_root_id,
											  Eigen::VectorXd& seed,
											  Eigen::VectorXd& target,
											  int& parent_box_id,
											  int& root_id,
											  int& target_root_id,
											  int& pair_unknown_failures,
											  bool& staged_target,
											  double& component_gap_sq,
											  GrowTraceFace* face,
											  std::vector<GrowTraceFace>* face_candidates,
											  StageContext& context,
											  const void* component_graph_override = nullptr);
	bool node_in_failure_cooling(OracleNodeId node,
								 int active_depth,
								 int box_count,
								 StageContext& context);
	bool seed_in_failure_cooling(const Eigen::Ref<const Eigen::VectorXd>& seed,
								 int active_depth,
								 int box_count,
								 StageContext& context,
								 OracleNodeId* domain_node = nullptr);
	void record_failure_cooling(const FindFreeBoxResult& result,
								OracleNodeId fallback_node,
								int active_depth,
								int box_count,
								StageContext& context);
	void record_failure_cooling_success(OracleNodeId node, StageContext& context);
	bool hard_frontier_stop_loss_enabled() const;
	int hard_frontier_failure_threshold() const;
	int hard_frontier_box_horizon() const;
	Eigen::VectorXd sample_uniform();
	Eigen::VectorXd sample_unexplored();
	bool make_frontier_seed(const std::vector<BoxNode>& boxes,
							const Eigen::VectorXd& target,
							Eigen::VectorXd& seed,
							int& parent_box_id,
							int& root_id,
							StageContext* context = nullptr,
							GrowTraceFace* selected_face = nullptr,
							std::vector<GrowTraceFace>* candidates = nullptr) const;
	bool make_frontier_seed_for_root(const std::vector<BoxNode>& boxes,
									 int root_id,
									 const Eigen::VectorXd& target,
									 Eigen::VectorXd& seed,
									 int& parent_box_id,
									 int& resolved_root_id,
									 StageContext* context = nullptr,
									 GrowTraceFace* selected_face = nullptr,
									 std::vector<GrowTraceFace>* candidates = nullptr) const;
	bool make_frontier_seed_from_parent(const std::vector<BoxNode>& boxes,
										int parent_index,
										const Eigen::VectorXd& target,
										Eigen::VectorXd& seed,
										int& parent_box_id,
										int& root_id,
										bool require_target_direction,
										GrowTraceFace* selected_face = nullptr,
										std::vector<GrowTraceFace>* candidates = nullptr,
										StageContext* context = nullptr) const;
	bool prepare_frontier_seed_with_memory(const std::vector<BoxNode>& boxes,
										   const BoxNode& parent,
										   const Eigen::VectorXd& target,
										   int face_dim,
										   int side,
										   Eigen::VectorXd& seed,
										   StageContext* context = nullptr) const;
	bool connected(const std::vector<BoxNode>& boxes) const;

	BoxOracle& oracle_;
	GrowerConfig config_;
	std::mt19937 rng_;
	std::vector<Eigen::VectorXd> random_anchor_targets_;
	int next_box_id_ = 0;
	mutable std::mutex trace_mutex_;
	mutable std::ofstream trace_file_;
	mutable bool trace_opened_ = false;
	mutable std::uint64_t trace_event_count_ = 0;
	mutable std::mutex frontier_cache_mutex_;
	mutable std::unordered_set<std::string> covered_frontier_seed_cache_;
	mutable std::mutex frontier_face_memory_mutex_;
	mutable std::unordered_map<std::uint64_t, std::unordered_set<std::uint64_t>> frontier_face_bins_;
	std::unordered_map<std::uint64_t, int> component_pair_unknown_failures_;
	std::unordered_map<int, int> component_parent_failures_;
	std::unordered_map<OracleNodeId, FailureCoolingEntry> failure_cooling_;
};

class FrontwaveGrower final : public IGrower {
public:
	struct BoundarySeed {
		Eigen::VectorXd q;
		int parent_box_id = -1;
		int root_id = -1;
	};

	FrontwaveGrower(BoxOracle& oracle, GrowerConfig config = {});
	GrowerResult grow(const std::vector<Eigen::VectorXd>& seeds) override;
	GrowerResult grow(const std::vector<Eigen::VectorXd>& seeds, StageContext& context) override;

private:
	int create_box(const Eigen::VectorXd& seed,
				   int parent_box_id,
				   int root_id,
				   std::vector<BoxNode>& boxes,
				   FindFreeBoxService& ffb,
				   StageContext& context);
	int commit_box(const Eigen::VectorXd& seed,
				   FindFreeBoxResult ffb_result,
				   int parent_box_id,
				   int root_id,
				   std::vector<BoxNode>& boxes,
				   StageContext& context);
	std::vector<GrowWorkerResult> run_worker_ffb_tasks(const std::vector<GrowTask>& tasks,
													   StageContext& context);
	std::vector<BoundarySeed> boundary_seeds(const BoxNode& box,
											 const Eigen::VectorXd* bias_target);

	BoxOracle& oracle_;
	GrowerConfig config_;
	std::mt19937 rng_;
	int next_box_id_ = 0;
};

std::unique_ptr<IGrower> make_grower(BoxOracle& oracle, const GrowerConfig& config);

}  // namespace rbf
