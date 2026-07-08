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
	void run_query_bridge_direct_corridor_residual_segments(
		const std::vector<Eigen::VectorXd>& samples,
		const std::vector<std::vector<int>>& sample_layers,
		const std::vector<int>& final_bad,
		const RRTConnectConfig& bridge_rrt,
		CollisionChecker& checker,
		StageContext& context,
		int bridge_edge_query_index,
		QueryBridgeLocalDsu& dsu,
		int& local_segment_edges_added,
		int& local_segment_gap_samples_max,
		double& residual_segment_audit_ms);
	void refresh_query_bridge_direct_corridor_partition(std::size_t boxes_before);
	int finish_query_bridge_direct_corridor(std::size_t boxes_before, int value);
	int finish_query_bridge_direct_corridor_attempt(
		const std::vector<Eigen::VectorXd>& corridor_path,
		const RRTConnectConfig& bridge_rrt,
		const CollisionChecker& checker,
		StageContext& context,
		int query_index,
		int bridge_edge_query_index,
		std::size_t boxes_before_direct_corridor,
		int direct_added,
		int repair_added,
		int local_segment_edges_added,
		bool allow_residual_segments,
		bool add_full_residual_overlay_when_connected,
		bool local_corridor_connected,
		int source_box_id,
		int target_box_id,
		const std::vector<std::vector<int>>& sample_layers,
		const std::vector<int>& final_bad);
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
	int add_audited_query_bridge_segment_edge(
		int source_box_id,
		int target_box_id,
		const std::vector<Eigen::VectorXd>& waypoint_path,
		const CollisionChecker& checker,
		int segment_resolution,
		int query_index);
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
