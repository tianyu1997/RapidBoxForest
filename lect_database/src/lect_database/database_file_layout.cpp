#include "database_file_layout.h"

#include <fstream>
#include <iomanip>
#include <sstream>

namespace rbf::lect_database::database_file {

std::filesystem::path manifest_path(const std::filesystem::path& root) {
    return root / "manifest.json";
}

std::filesystem::path nodes_path(const std::filesystem::path& root) {
    return root / "nodes.pages";
}

std::filesystem::path node_pages_path(const std::filesystem::path& root) {
    return root / "node_pages";
}

std::filesystem::path node_page_path(const std::filesystem::path& root, std::uint64_t page_id) {
    return node_pages_path(root) / ("page_" + std::to_string(page_id) + ".rows");
}

std::filesystem::path evidence_path(const std::filesystem::path& root) {
    return root / "evidence.pages";
}

std::filesystem::path evidence_index_path(const std::filesystem::path& root) {
    return root / "evidence.index";
}

std::filesystem::path journal_path(const std::filesystem::path& root) {
    return root / "journal.log";
}

bool replace_file(const std::filesystem::path& tmp, const std::filesystem::path& path) {
    std::error_code ignored;
    std::filesystem::remove(path, ignored);
    std::error_code error;
    std::filesystem::rename(tmp, path, error);
    return !error;
}

std::unordered_map<std::string, std::string> read_key_values(const std::filesystem::path& path) {
    std::unordered_map<std::string, std::string> values;
    std::ifstream input(path);
    std::string line;
    while (std::getline(input, line)) {
        const auto pos = line.find('=');
        if (pos == std::string::npos) {
            continue;
        }
        values[line.substr(0, pos)] = line.substr(pos + 1);
    }
    return values;
}

std::string get_value(const std::unordered_map<std::string, std::string>& values,
                      const std::string& key,
                      const std::string& fallback) {
    const auto it = values.find(key);
    return it == values.end() ? fallback : it->second;
}

std::uint64_t get_u64(const std::unordered_map<std::string, std::string>& values,
                      const std::string& key,
                      std::uint64_t fallback) {
    const auto text = get_value(values, key);
    return text.empty() ? fallback : static_cast<std::uint64_t>(std::stoull(text));
}

int get_int(const std::unordered_map<std::string, std::string>& values,
            const std::string& key,
            int fallback) {
    const auto text = get_value(values, key);
    return text.empty() ? fallback : std::stoi(text);
}

double get_double(const std::unordered_map<std::string, std::string>& values,
                  const std::string& key,
                  double fallback) {
    const auto text = get_value(values, key);
    return text.empty() ? fallback : std::stod(text);
}

std::vector<std::string> split(const std::string& line, char delimiter) {
    std::vector<std::string> parts;
    std::stringstream stream(line);
    std::string item;
    while (std::getline(stream, item, delimiter)) {
        parts.push_back(item);
    }
    return parts;
}

std::string serialize_depth_dimensions(const std::vector<int>& dims) {
    std::ostringstream out;
    for (std::size_t index = 0; index < dims.size(); ++index) {
        if (index > 0) {
            out << ',';
        }
        out << dims[index];
    }
    return out.str();
}

std::vector<int> parse_depth_dimensions(const std::string& text) {
    std::vector<int> dims;
    if (text.empty()) {
        return dims;
    }
    const auto parts = split(text, ',');
    dims.reserve(parts.size());
    for (const auto& part : parts) {
        if (!part.empty()) {
            dims.push_back(std::stoi(part));
        }
    }
    return dims;
}

namespace {

std::string serialize_path(const PathCode& path) {
    std::ostringstream out;
    out << path.bit_count << ':' << path.words.size();
    for (std::uint64_t word : path.words) {
        out << ':' << word;
    }
    return out.str();
}

PathCode parse_path(const std::string& text) {
    PathCode path;
    const auto parts = split(text, ':');
    if (parts.size() < 2) {
        return path;
    }
    path.bit_count = std::stoi(parts[0]);
    const std::size_t n_words = static_cast<std::size_t>(std::stoull(parts[1]));
    path.words.reserve(n_words);
    for (std::size_t index = 0; index < n_words && index + 2 < parts.size(); ++index) {
        path.words.push_back(static_cast<std::uint64_t>(std::stoull(parts[index + 2])));
    }
    return path;
}

}  // namespace

std::string serialize_node_record(const NodeRecord& record) {
    std::ostringstream out;
    out << record.id << '|'
        << record.parent << '|'
        << record.left << '|'
        << record.right << '|'
        << record.depth << '|'
        << record.split_dim << '|'
        << std::setprecision(17) << record.split_value << '|'
        << record.generation << '|'
        << record.page_id << '|'
        << (record.dirty ? 1 : 0) << '|'
        << (record.evidence_dirty ? 1 : 0) << '|'
        << serialize_path(record.path);
    return out.str();
}

std::optional<NodeRecord> parse_node_record(const std::string& line) {
    const auto parts = split(line, '|');
    if (parts.size() < 12) {
        return std::nullopt;
    }
    NodeRecord record;
    record.id = static_cast<NodeId>(std::stoull(parts[0]));
    record.parent = static_cast<NodeId>(std::stoull(parts[1]));
    record.left = static_cast<NodeId>(std::stoull(parts[2]));
    record.right = static_cast<NodeId>(std::stoull(parts[3]));
    record.depth = std::stoi(parts[4]);
    record.split_dim = std::stoi(parts[5]);
    record.split_value = std::stod(parts[6]);
    record.generation = static_cast<std::uint64_t>(std::stoull(parts[7]));
    record.page_id = static_cast<std::uint64_t>(std::stoull(parts[8]));
    record.dirty = std::stoi(parts[9]) != 0;
    record.evidence_dirty = std::stoi(parts[10]) != 0;
    record.path = parse_path(parts[11]);
    return record;
}

}  // namespace rbf::lect_database::database_file
