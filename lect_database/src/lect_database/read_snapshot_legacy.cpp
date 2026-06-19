#include "read_snapshot_legacy.h"

#include <sstream>
#include <vector>

namespace rbf::lect_database {
namespace {

std::vector<std::string> split_text(const std::string& line, char delimiter) {
    std::vector<std::string> parts;
    std::stringstream stream(line);
    std::string item;
    while (std::getline(stream, item, delimiter)) {
        parts.push_back(item);
    }
    return parts;
}

}  // namespace

std::optional<LegacyNodeIndexSidecarEntry> parse_legacy_node_pages_record(
    const std::string& line) {
    const auto parts = split_text(line, '|');
    if (parts.size() < 7) {
        return std::nullopt;
    }
    LegacyNodeIndexSidecarEntry entry;
    try {
        entry.node_id = static_cast<std::uint64_t>(std::stoull(parts[0]));
        entry.parent = static_cast<std::uint64_t>(std::stoull(parts[1]));
        entry.left = static_cast<std::uint64_t>(std::stoull(parts[2]));
        entry.right = static_cast<std::uint64_t>(std::stoull(parts[3]));
        entry.depth = static_cast<std::int32_t>(std::stoi(parts[4]));
        entry.split_dim = static_cast<std::int32_t>(std::stoi(parts[5]));
        entry.split_value = std::stod(parts[6]);
    } catch (const std::exception&) {
        return std::nullopt;
    }
    return entry;
}

std::size_t legacy_path_code_storage_bytes(std::uint32_t word_count) {
    return static_cast<std::size_t>(word_count) * sizeof(std::uint64_t);
}

}  // namespace rbf::lect_database
