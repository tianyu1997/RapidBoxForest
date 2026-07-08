#pragma once

#include <LECTDatabase/online_cache/config.h>
#include <rbf/core.h>
#include <rbf/lect_database/split_policy.h>

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace rbf {

struct LectDatabaseRuntimeConfig {
	std::filesystem::path path;
	std::filesystem::path external_evidence_path;
	std::filesystem::path external_evidence_snapshot_path;
	std::vector<Interval> root_intervals_override;
	std::vector<Interval> coverage_intervals_override;
	lect_database::SplitPolicyDescriptor split_policy;
	lect_database::OnlineEnvelopeCacheConfig online_cache;
	bool external_evidence_use_snapshot = true;
	bool external_evidence_auto_build_snapshot = true;
	bool read_only = false;
	bool create_if_missing = true;
	bool verify_identity = true;
	bool replay_journal = true;
	bool propagate_parent_hulls = true;
	bool defer_parent_hull_writes = false;
	bool canonical_mode = true;
	bool checkpoint_after_build = true;
	std::string symmetry_descriptor = "joint_symmetry_native_v1";
	std::uint32_t page_size_bytes = 64u * 1024u;
	std::uint32_t max_resident_pages = 256u;
	int max_tree_depth = 64;
};

} // namespace rbf
