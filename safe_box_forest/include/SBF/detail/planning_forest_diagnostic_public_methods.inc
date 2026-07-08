	/// Access oracle counters from the most recent build call.
	/// Returns nullptr if the oracle hasn't been initialised yet.
	const OracleCounters* oracle_counters() const;
	BuildProfile build_subtractive(const std::vector<SubtractiveObstacleGroup>& obstacle_groups,
								   const std::vector<Eigen::VectorXd>& seeds);
	BuildProfile build_subtractive(const std::vector<SubtractiveObstacleGroup>& obstacle_groups,
								   const std::vector<Eigen::VectorXd>& seeds,
								   const SubtractiveBuildOptions& options);
	/// Isolated chain_pave diagnostic entry: build the BiRRT bridge between the
	/// boxes containing @p start and @p goal, then run chain_pave_along_path()
	/// with the supplied @p pave config. Mutates the forest box set.
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
	RebuildProfile connect_update_segment_fallback();
	RebuildProfile connect_update_endpoint_segment_fallback(const Eigen::Ref<const Eigen::VectorXd>& start,
															const Eigen::Ref<const Eigen::VectorXd>& goal);
	RebuildProfile add_obstacle_and_rebuild(const Obstacle& obstacle);
	RebuildProfile add_obstacles_and_rebuild(const std::vector<Obstacle>& obstacles);
	RebuildProfile remove_obstacle_and_regrow(int obstacle_index);
	RebuildProfile remove_obstacle_suffix_and_regrow(int target_obstacle_count);
