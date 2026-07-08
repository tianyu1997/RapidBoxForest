	void reset_oracle(Scene scene);
	void reserve_existing_boxes();
	void rebuild_adjacency();
	void rebuild_adaptive_partition(const AdaptiveLeafSweepConfig& config, BuildProfile* profile);
	void refresh_adaptive_partition_diagnostics(BuildProfile* profile) const;
	int append_adaptive_partition_boxes(std::size_t first_box_index,
										 BuildProfile* profile,
										 const char* diagnostic_prefix);
	int sync_adaptive_partition_segment_edges(BuildProfile* profile,
											 const char* diagnostic_prefix);
	std::vector<int> endpoint_main_largest_island_partition_first(
		const std::vector<int>& preferred_island) const;
	int add_segment_edge_partition_first(int source_box_id,
										 int target_box_id,
										 std::vector<Eigen::VectorXd> waypoints,
										 SegmentEdgeType type,
										 int segment_resolution,
										 SegmentEdgeValidation validation,
										 bool strict_audit_required = false,
										 int query_index = -1,
										 BuildProfile* profile = nullptr,
										 const char* diagnostic_prefix = nullptr);
	bool try_add_endpoint_main_residual_segment_edge(
		int front_box_id,
		int target_box_id,
		const Eigen::Ref<const Eigen::VectorXd>& target_point,
		bool max_depth_ffb_failed,
		double max_segment_length);
