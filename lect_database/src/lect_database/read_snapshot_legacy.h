#pragma once

#include "read_snapshot_format.h"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

namespace rbf::lect_database {

std::optional<LegacyNodeIndexSidecarEntry> parse_legacy_node_pages_record(
    const std::string& line);

std::size_t legacy_path_code_storage_bytes(std::uint32_t word_count);

}  // namespace rbf::lect_database
