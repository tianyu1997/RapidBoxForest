	std::pair<int, int> locate_query_bridge_boxes(const Eigen::Ref<const Eigen::VectorXd>& start,
												  const Eigen::Ref<const Eigen::VectorXd>& goal,
												  StageContext& context);
	int run_query_bridge_chain_pave(const std::vector<Eigen::VectorXd>& waypoint_path,
									int start_box_id,
									int& next_id,
									StageContext& context,
									const ChainPaveConfig& pave_config,
									const char* partition_prefix);
	std::pair<int, int> run_query_bridge_reverse_boundary_pave(
		const Eigen::Ref<const Eigen::VectorXd>& start,
		const Eigen::Ref<const Eigen::VectorXd>& goal,
		const std::vector<Eigen::VectorXd>& waypoint_path,
		const ChainPaveConfig& forward_config,
		int forward_added,
		int& accumulated_added,
		int& next_id,
		StageContext& context);
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
	std::vector<QueryBridgeSearchTask> prepare_query_bridge_batch_tasks(
		const std::vector<Eigen::VectorXd>& starts,
		const std::vector<Eigen::VectorXd>& goals,
		const QueryBridgeBatchOptions& options,
		const std::unordered_set<int>& forced_query_indices,
		const QueryBridgeAcceptanceThresholds& bridge_acceptance,
		std::size_t& partition_refresh_base);
	void run_query_bridge_batch_parallel_rrt(
		std::vector<QueryBridgeSearchTask>& tasks,
		std::vector<int>& added_by_query,
		const std::unordered_set<int>& forced_query_indices,
		const QueryBridgeAcceptanceThresholds& bridge_acceptance,
		const QueryBridgeRetryOptions& retry_options,
		const QueryBridgeParallelRrtOptions& parallel_rrt_options,
		const QueryBridgeHybridizeAttemptOptions& hybrid_options,
		const QueryBridgeEdgeRuntimeOptions& edge_options,
		bool scene_reusable_edges,
		StageContext& batch_context,
		std::chrono::steady_clock::time_point batch_t0);
	void run_query_bridge_batch_serial_rrt(
		std::vector<QueryBridgeSearchTask>& tasks,
		std::vector<int>& added_by_query,
		const std::unordered_set<int>& forced_query_indices,
		const QueryBridgeAcceptanceThresholds& bridge_acceptance,
		const QueryBridgeRetryOptions& retry_options,
		const QueryBridgeParallelRrtOptions& parallel_rrt_options,
		const QueryBridgeHybridizeAttemptOptions& hybrid_options,
		const QueryBridgeEdgeRuntimeOptions& edge_options,
		bool scene_reusable_edges,
		StageContext& batch_context);
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
