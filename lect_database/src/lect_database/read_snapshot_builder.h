#pragma once

#include <filesystem>
#include <string>

namespace rbf::lect_database {

bool build_read_snapshot_from_legacy(const std::filesystem::path& legacy_root,
                                     const std::filesystem::path& snapshot_path,
                                     std::string* reason);

}  // namespace rbf::lect_database
