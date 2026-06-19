#include "read_snapshot_paths.h"

#include <system_error>

namespace rbf::lect_database {

std::filesystem::path legacy_manifest_path(const std::filesystem::path& root) {
    return root / "manifest.json";
}

std::filesystem::path legacy_node_index_path(const std::filesystem::path& root) {
    return root / "nodes.index";
}

std::filesystem::path legacy_nodes_pages_path(const std::filesystem::path& root) {
    return root / "nodes.pages";
}

std::filesystem::path legacy_evidence_index_path(const std::filesystem::path& root) {
    return root / "evidence.index";
}

std::filesystem::path legacy_evidence_path(const std::filesystem::path& root) {
    return root / "evidence.pages";
}

std::filesystem::path snapshot_manifest_path(const std::filesystem::path& root) {
    return root / "manifest.bin";
}

std::filesystem::path snapshot_nodes_path(const std::filesystem::path& root) {
    return root / "nodes.bin";
}

std::filesystem::path snapshot_evidence_table_path(const std::filesystem::path& root) {
    return root / "evidence_table.bin";
}

std::filesystem::path snapshot_direct_evidence_path(const std::filesystem::path& root) {
    return root / "direct_evidence.bin";
}

std::filesystem::path snapshot_payload_path(const std::filesystem::path& root) {
    return root / "payload.bin";
}

bool replace_directory(const std::filesystem::path& staging,
                       const std::filesystem::path& target) {
    std::error_code ignored;
    std::filesystem::remove_all(target, ignored);
    std::error_code error;
    std::filesystem::rename(staging, target, error);
    return !error;
}

}  // namespace rbf::lect_database
