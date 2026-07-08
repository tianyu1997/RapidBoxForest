	void invalidate_query_cache() const;
	const QueryGraphCache& query_cache() const;
	int next_box_id() const;
	void initialize_optional_collision_cache();
	void clear_optional_collision_cache();
	void populate_optional_collision_cache_from_leaf_sweep(const LeafSweepResult& result,
													 int obstacle_count);
	void record_optional_collision_cache_box_count(BuildProfile& profile,
															 const char* diagnostic_key) const;
