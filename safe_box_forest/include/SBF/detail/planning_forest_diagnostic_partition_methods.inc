	void refresh_adaptive_partition_diagnostics(RebuildProfile& profile) const;
	void refresh_dynamic_partition_after_update(RebuildProfile& profile,
												const char* diagnostic_prefix);
	void refresh_dynamic_partition_after_append(RebuildProfile& profile,
												std::size_t first_box_index,
												const char* diagnostic_prefix);
	void refresh_dynamic_partition_after_remove_append(RebuildProfile& profile,
													   const std::unordered_set<int>& removed_box_ids,
													   std::size_t first_box_index,
													   const char* diagnostic_prefix);
