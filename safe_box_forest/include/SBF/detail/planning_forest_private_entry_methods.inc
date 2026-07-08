	FindFreeBoxResult find_free_box_in_domain(const Eigen::Ref<const Eigen::VectorXd>& seed,
											  const std::vector<Interval>& domain,
											  StageContext& context,
											  const FindFreeBoxOptions& options);
	FindFreeBoxResult find_free_box_binary_in_domain(
		const Eigen::Ref<const Eigen::VectorXd>& seed,
		const std::vector<Interval>& domain,
		StageContext& context,
		const FindFreeBoxOptions& options,
		const OracleSplitOptions& split_options,
		int effective_max_depth,
		std::chrono::steady_clock::time_point start);
	QueryResult run_query_internal(const Eigen::Ref<const Eigen::VectorXd>& start,
								   const Eigen::Ref<const Eigen::VectorXd>& goal,
								   bool allow_collision_shortcut,
								   const RBFQueryRuntimeOptions& runtime_options = {}) const;
	int anchor_query_endpoint_box(const Eigen::Ref<const Eigen::VectorXd>& point,
								  StageContext& context);
	int anchor_query_endpoint_box_with_diagnostics(const Eigen::Ref<const Eigen::VectorXd>& point);
	int locate_box_partition_first(const Eigen::Ref<const Eigen::VectorXd>& point,
								   bool nearest_if_outside) const;
