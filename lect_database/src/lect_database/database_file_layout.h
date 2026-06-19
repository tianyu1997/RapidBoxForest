#pragma once

#include <rbf/lect_database/types.h>

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace rbf::lect_database::database_file {

inline constexpr std::string_view kCurrentNodeIdScheme = "path_handle_v3";
inline constexpr std::string_view kCurrentEvidenceEncoding = "pathcode_v3";

std::filesystem::path manifest_path(const std::filesystem::path& root);
std::filesystem::path nodes_path(const std::filesystem::path& root);
std::filesystem::path node_pages_path(const std::filesystem::path& root);
std::filesystem::path node_page_path(const std::filesystem::path& root, std::uint64_t page_id);
std::filesystem::path evidence_path(const std::filesystem::path& root);
std::filesystem::path evidence_index_path(const std::filesystem::path& root);
std::filesystem::path journal_path(const std::filesystem::path& root);

bool replace_file(const std::filesystem::path& tmp, const std::filesystem::path& path);

std::unordered_map<std::string, std::string> read_key_values(const std::filesystem::path& path);
std::string get_value(const std::unordered_map<std::string, std::string>& values,
                      const std::string& key,
                      const std::string& fallback = {});
std::uint64_t get_u64(const std::unordered_map<std::string, std::string>& values,
                      const std::string& key,
                      std::uint64_t fallback = 0);
int get_int(const std::unordered_map<std::string, std::string>& values,
            const std::string& key,
            int fallback = 0);
double get_double(const std::unordered_map<std::string, std::string>& values,
                  const std::string& key,
                  double fallback = 0.0);

std::vector<std::string> split(const std::string& line, char delimiter);
std::string serialize_depth_dimensions(const std::vector<int>& dims);
std::vector<int> parse_depth_dimensions(const std::string& text);
std::string serialize_node_record(const NodeRecord& record);
std::optional<NodeRecord> parse_node_record(const std::string& line);

}  // namespace rbf::lect_database::database_file
