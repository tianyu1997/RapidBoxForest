#pragma once

#include <rbf/lect_database/types.h>

#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace rbf::lect_database {

using SnapshotManifestValues = std::unordered_map<std::string, std::string>;

SnapshotManifestValues read_snapshot_manifest_values(const std::filesystem::path& path);

std::uint64_t snapshot_manifest_u64(const SnapshotManifestValues& values,
                                    std::string_view key,
                                    std::uint64_t fallback = 0);
int snapshot_manifest_int(const SnapshotManifestValues& values,
                          std::string_view key,
                          int fallback = 0);
double snapshot_manifest_double(const SnapshotManifestValues& values,
                                std::string_view key,
                                double fallback = 0.0);

std::vector<Interval> parse_snapshot_root_intervals(const SnapshotManifestValues& values);

}  // namespace rbf::lect_database
