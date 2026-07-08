#pragma once

#include <SBF/find_free_box_types.h>
#include <SBF/grower_types.h>
#include <SBF/runtime_fwd.h>

#include <LECTDatabase/sbf/oracle_types.h>

#include <Eigen/Core>

#include <chrono>
#include <cstdint>
#include <fstream>
#include <functional>
#include <memory>
#include <mutex>
#include <ostream>
#include <random>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace rbf {

class FindFreeBoxService;

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
	GrowerResult grow_from_existing(const std::vector<BoxNode>& initial_boxes,
									const std::vector<Eigen::VectorXd>& seeds,
									StageContext& context);

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
	void initialize_anchor_targets(const std::vector<Eigen::VectorXd>& roots,
								   const std::vector<Eigen::VectorXd>& seeds,
								   StageContext& context);
	void run_frontwave_bootstrap(GrowerResult& result,
								 FindFreeBoxService& ffb,
								 StageContext& context,
								 std::chrono::steady_clock::time_point deadline);
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
	std::vector<GrowTaskRequest> make_growth_task_requests(const std::vector<BoxNode>& boxes,
														   const std::vector<Eigen::VectorXd>& roots,
														   int first_task_id,
														   int n_tasks,
														   StageContext& context);
	std::vector<GrowTask> filter_growth_tasks(const std::vector<BoxNode>& boxes,
											  std::vector<GrowTask> tasks,
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
	void record_component_connect_success(int parent_box_id,
										  int source_root_id,
										  int target_root_id,
										  StageContext& context);
	void record_component_connect_failure(int parent_box_id,
										  int source_root_id,
										  int target_root_id,
										  const FindFreeBoxResult* ffb_result,
										  StageContext& context);
	int record_component_connect_success_and_extend(std::vector<BoxNode>& boxes,
													FindFreeBoxService& ffb,
													const FindFreeBoxOptions& base_options,
													int depth_stage_index,
													int parent_box_id,
													int source_root_id,
													int target_root_id,
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
	std::vector<BoxNode> initial_boxes_;
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
