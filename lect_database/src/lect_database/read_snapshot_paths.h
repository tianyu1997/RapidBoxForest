#pragma once

#include <filesystem>

namespace rbf::lect_database {

std::filesystem::path legacy_manifest_path(const std::filesystem::path& root);
std::filesystem::path legacy_node_index_path(const std::filesystem::path& root);
std::filesystem::path legacy_nodes_pages_path(const std::filesystem::path& root);
std::filesystem::path legacy_evidence_index_path(const std::filesystem::path& root);
std::filesystem::path legacy_evidence_path(const std::filesystem::path& root);

std::filesystem::path snapshot_manifest_path(const std::filesystem::path& root);
std::filesystem::path snapshot_nodes_path(const std::filesystem::path& root);
std::filesystem::path snapshot_evidence_table_path(const std::filesystem::path& root);
std::filesystem::path snapshot_direct_evidence_path(const std::filesystem::path& root);
std::filesystem::path snapshot_payload_path(const std::filesystem::path& root);

bool replace_directory(const std::filesystem::path& staging,
                       const std::filesystem::path& target);

}  // namespace rbf::lect_database
