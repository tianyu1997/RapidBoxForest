#include "read_snapshot_manifest.h"

#include <algorithm>
#include <charconv>
#include <cstdlib>
#include <fstream>
#include <string>

namespace rbf::lect_database {

SnapshotManifestValues read_snapshot_manifest_values(const std::filesystem::path& path) {
    SnapshotManifestValues values;
    std::ifstream input(path);
    std::string line;
    while (std::getline(input, line)) {
        const auto pos = line.find('=');
        if (pos == std::string::npos) {
            continue;
        }
        values.emplace(line.substr(0, pos), line.substr(pos + 1));
    }
    return values;
}

std::uint64_t snapshot_manifest_u64(const SnapshotManifestValues& values,
                                    std::string_view key,
                                    std::uint64_t fallback) {
    const auto it = values.find(std::string(key));
    if (it == values.end()) {
        return fallback;
    }
    std::uint64_t out = fallback;
    auto first = it->second.data();
    auto last = first + it->second.size();
    std::from_chars(first, last, out);
    return out;
}

int snapshot_manifest_int(const SnapshotManifestValues& values,
                          std::string_view key,
                          int fallback) {
    return static_cast<int>(snapshot_manifest_u64(values,
                                                  key,
                                                  static_cast<std::uint64_t>(fallback)));
}

double snapshot_manifest_double(const SnapshotManifestValues& values,
                                std::string_view key,
                                double fallback) {
    const auto it = values.find(std::string(key));
    if (it == values.end()) {
        return fallback;
    }
    char* end = nullptr;
    const double value = std::strtod(it->second.c_str(), &end);
    return end == it->second.c_str() ? fallback : value;
}

std::vector<Interval> parse_snapshot_root_intervals(const SnapshotManifestValues& values) {
    const int dims = std::max(0, snapshot_manifest_int(values, "root_dims"));
    std::vector<Interval> root;
    root.reserve(static_cast<std::size_t>(dims));
    for (int dim = 0; dim < dims; ++dim) {
        root.push_back({snapshot_manifest_double(values, "root_" + std::to_string(dim) + "_lo"),
                        snapshot_manifest_double(values, "root_" + std::to_string(dim) + "_hi")});
    }
    return root;
}

}  // namespace rbf::lect_database
