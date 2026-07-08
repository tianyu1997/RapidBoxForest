	void initialize_dynamic_collision_cache();
	int dynamic_collision_cache_box_count() const;
	void populate_dynamic_collision_cache(const LeafSweepResult& result,
										  int obstacle_count);
	void clear_dynamic_collision_cache();
	void rebuild_dynamic_collision_cache_index();
	void add_dynamic_collision_cache_box(const BoxNode& box,
										 std::vector<int> blocking_obstacle_indices);
	int promote_unblocked_collision_cache(const std::unordered_set<int>& removed_obstacle_indices,
										  RebuildProfile& profile);
	int refill_removed_box_with_leaf_sweep(const BoxNode& removed_box,
										   int new_obstacle_index,
										   int max_depth,
										   int& next_id,
										   RebuildProfile& profile);
