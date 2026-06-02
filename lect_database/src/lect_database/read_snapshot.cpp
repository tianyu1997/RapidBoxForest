#include <rbf/lect_database/read_snapshot.h>

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <span>
#include <sstream>
#include <string_view>
#include <system_error>
#include <unordered_map>
#include <vector>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

namespace rbf::lect_database {
namespace {

constexpr std::uint64_t kSnapshotManifestMagic = 0x324d46424454434cull;
constexpr std::uint64_t kSnapshotNodesMagic = 0x32444f4e4254434cull;
constexpr std::uint64_t kSnapshotEvidenceTableMagic = 0x325456454254434cull;
constexpr std::uint32_t kSnapshotFormatVersion = 2;
constexpr std::uint32_t kSnapshotNodePresent = 1u << 0;
constexpr std::uint32_t kSnapshotEvidencePresent = 1u << 0;
constexpr std::uint32_t kSnapshotEvidenceChildHull = 1u << 1;
constexpr std::uint32_t kSnapshotEvidenceUnavailable = 1u << 2;
constexpr std::uint32_t kLegacyEvidenceStoreSchemaVersion = 4;
constexpr std::uint64_t kLegacyEvidenceIndexSidecarMagic = 0x3158444945424652ull;
constexpr std::uint64_t kLegacyNodeIndexSidecarMagic = 0x31584449444f4e52ull;

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

// Decode an IEEE binary16 (half) value to float32. Mirrors the encoder used by
// the authoritative evidence store (lect_database/database.cpp). Snapshot
// payloads are stored as outward-rounded halves (schema v4); decode on read.
float f32_from_f16(std::uint16_t h) {
    const std::uint32_t sign = static_cast<std::uint32_t>(h & 0x8000u) << 16;
    const std::uint32_t exp = (h >> 10) & 0x1fu;
    const std::uint32_t mant = h & 0x3ffu;
    std::uint32_t bits;
    if (exp == 0) {
        if (mant == 0) {
            bits = sign;  // +/- zero
        } else {
            // Subnormal half -> normalized float.
            std::uint32_t m = mant;
            std::int32_t e = -1;
            do {
                m <<= 1;
                ++e;
            } while ((m & 0x400u) == 0);
            m &= 0x3ffu;
            const std::uint32_t fexp = static_cast<std::uint32_t>(127 - 15 - e);
            bits = sign | (fexp << 23) | (m << 13);
        }
    } else if (exp == 0x1fu) {
        bits = sign | 0x7f800000u | (mant << 13);  // inf / nan
    } else {
        const std::uint32_t fexp = exp + (127u - 15u);
        bits = sign | (fexp << 23) | (mant << 13);
    }
    float out;
    std::memcpy(&out, &bits, sizeof(out));
    return out;
}

// Decode `count` halves starting at `base` into a freshly owned float buffer.
std::shared_ptr<std::vector<float>> decode_half_payload(const std::uint16_t* base, std::size_t count) {
    auto buffer = std::make_shared<std::vector<float>>();
    buffer->resize(count);
    for (std::size_t i = 0; i < count; ++i) {
        std::uint16_t h;
        std::memcpy(&h, base + i, sizeof(h));
        (*buffer)[i] = f32_from_f16(h);
    }
    return buffer;
}

std::size_t next_power_of_two(std::size_t value) {
    std::size_t result = 1;
    while (result < value) {
        result <<= 1;
    }
    return result;
}

std::uint64_t hash_evidence_key(NodeId node_id,
                                SectorId sector,
                                EvidenceChannel channel,
                                EndpointSource endpoint_source,
                                EvidencePayloadKind payload_kind) {
    std::uint64_t hash = 1469598103934665603ull;
    hash = stable_hash_append(hash, &node_id, sizeof(node_id));
    hash = stable_hash_append(hash, &sector, sizeof(sector));
    const auto channel_value = static_cast<std::uint32_t>(channel);
    const auto endpoint_value = static_cast<std::uint32_t>(endpoint_source);
    const auto payload_value = static_cast<std::uint32_t>(payload_kind);
    hash = stable_hash_append(hash, &channel_value, sizeof(channel_value));
    hash = stable_hash_append(hash, &endpoint_value, sizeof(endpoint_value));
    hash = stable_hash_append(hash, &payload_value, sizeof(payload_value));
    return hash;
}

bool replace_directory(const std::filesystem::path& staging, const std::filesystem::path& target) {
    std::error_code ignored;
    std::filesystem::remove_all(target, ignored);
    std::error_code error;
    std::filesystem::rename(staging, target, error);
    return !error;
}

std::unordered_map<std::string, std::string> read_manifest_values(const std::filesystem::path& path) {
    std::unordered_map<std::string, std::string> values;
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

std::uint64_t get_u64(const std::unordered_map<std::string, std::string>& values,
                      std::string_view key,
                      std::uint64_t fallback = 0) {
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

int get_int(const std::unordered_map<std::string, std::string>& values,
            std::string_view key,
            int fallback = 0) {
    return static_cast<int>(get_u64(values, key, static_cast<std::uint64_t>(fallback)));
}

double get_double(const std::unordered_map<std::string, std::string>& values,
                  std::string_view key,
                  double fallback = 0.0) {
    const auto it = values.find(std::string(key));
    if (it == values.end()) {
        return fallback;
    }
    char* end = nullptr;
    const double value = std::strtod(it->second.c_str(), &end);
    return end == it->second.c_str() ? fallback : value;
}

std::vector<std::string> split_text(const std::string& line, char delimiter) {
    std::vector<std::string> parts;
    std::stringstream stream(line);
    std::string item;
    while (std::getline(stream, item, delimiter)) {
        parts.push_back(item);
    }
    return parts;
}

std::vector<Interval> parse_root_intervals(const std::unordered_map<std::string, std::string>& values) {
    const int dims = std::max(0, get_int(values, "root_dims"));
    std::vector<Interval> root;
    root.reserve(static_cast<std::size_t>(dims));
    for (int dim = 0; dim < dims; ++dim) {
        root.push_back({get_double(values, "root_" + std::to_string(dim) + "_lo"),
                        get_double(values, "root_" + std::to_string(dim) + "_hi")});
    }
    return root;
}

std::size_t path_code_storage_bytes(std::uint32_t word_count) {
    return static_cast<std::size_t>(word_count) * sizeof(std::uint64_t);
}

#pragma pack(push, 1)
struct LegacyNodeIndexSidecarHeader {
    std::uint64_t magic = 0;
    std::uint32_t version = 0;
    std::uint32_t header_size = 0;
    std::uint64_t generation = 0;
    std::uint64_t node_count = 0;
    std::uint64_t max_node_id = 0;
    std::uint64_t entry_count = 0;
};

struct LegacyNodeIndexSidecarEntry {
    std::uint64_t node_id = 0;
    std::uint64_t parent = kInvalidNodeId;
    std::uint64_t left = kInvalidNodeId;
    std::uint64_t right = kInvalidNodeId;
    std::int32_t depth = 0;
    std::int32_t split_dim = -1;
    double split_value = 0.0;
};

struct LegacyEvidenceIndexSidecarHeader {
    std::uint64_t magic = 0;
    std::uint32_t version = 0;
    std::uint32_t header_size = 0;
    std::uint64_t evidence_file_size = 0;
    std::uint64_t entry_count = 0;
    std::uint64_t path_blob_offset = 0;
};

struct LegacyEvidenceIndexSidecarEntry {
    std::uint64_t node_id = 0;
    std::int32_t sector = 0;
    std::uint32_t channel = 0;
    std::uint32_t endpoint_source = 0;
    std::uint32_t payload_kind = 0;
    std::uint32_t flags = 0;
    std::uint32_t size = 0;
    std::uint32_t path_word_count = 0;
    std::uint32_t path_bit_count = 0;
    std::uint64_t path_blob_offset = 0;
    std::uint64_t offset = 0;
    std::uint64_t generation = 0;
    std::uint64_t checksum = 0;
};
#pragma pack(pop)

static_assert(sizeof(LegacyNodeIndexSidecarHeader) == 48);
static_assert(sizeof(LegacyNodeIndexSidecarEntry) == 48);
static_assert(sizeof(LegacyEvidenceIndexSidecarHeader) == 40);
static_assert(sizeof(LegacyEvidenceIndexSidecarEntry) == 72);

struct SnapshotBoxIndexKey {
    std::uint64_t primary = 0;
    std::uint64_t secondary = 0;

    bool operator==(const SnapshotBoxIndexKey& other) const noexcept {
        return primary == other.primary && secondary == other.secondary;
    }
};

struct SnapshotBoxIndexKeyHash {
    std::size_t operator()(const SnapshotBoxIndexKey& key) const noexcept {
        return static_cast<std::size_t>(key.primary ^ (key.secondary + 0x9e3779b97f4a7c15ull + (key.primary << 6u) + (key.primary >> 2u)));
    }
};

SnapshotBoxIndexKey make_snapshot_box_index_key(const std::vector<Interval>& intervals) {
    SnapshotBoxIndexKey key;
    key.primary = fingerprint_intervals(intervals);
    std::uint64_t secondary = 1099511628211ull;
    for (const auto& interval : intervals) {
        secondary = stable_hash_append(secondary, &interval.hi, sizeof(interval.hi));
        secondary = stable_hash_append(secondary, &interval.lo, sizeof(interval.lo));
    }
    key.secondary = secondary;
    return key;
}

std::optional<LegacyNodeIndexSidecarEntry> parse_legacy_node_pages_record(const std::string& line) {
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

struct SnapshotManifestHeader {
    std::uint64_t magic = kSnapshotManifestMagic;
    std::uint32_t version = kSnapshotFormatVersion;
    std::uint32_t header_size = sizeof(SnapshotManifestHeader);
    std::uint64_t generation = 0;
    std::uint64_t node_count = 0;
    std::uint64_t max_node_id = 0;
    std::uint64_t evidence_count = 0;
    std::uint64_t root_domain_fingerprint = 0;
    std::uint64_t split_policy_hash = 0;
    std::uint64_t payload_file_size = 0;
    std::uint32_t root_dims = 0;
    std::uint32_t split_strategy = 0;
    double split_min_width = 0.0;
    std::uint32_t split_midpoint = 1;
    std::uint32_t split_deterministic_tie_break = 1;
};

struct SnapshotNodesHeader {
    std::uint64_t magic = kSnapshotNodesMagic;
    std::uint32_t version = kSnapshotFormatVersion;
    std::uint32_t header_size = sizeof(SnapshotNodesHeader);
    std::uint64_t row_count = 0;
    std::uint64_t node_count = 0;
    std::uint64_t max_node_id = 0;
};

struct SnapshotNodeRow {
    std::uint64_t parent = kInvalidNodeId;
    std::uint64_t left = kInvalidNodeId;
    std::uint64_t right = kInvalidNodeId;
    std::int32_t depth = 0;
    std::int32_t split_dim = -1;
    double split_value = 0.0;
    std::uint64_t generation = 0;
    std::uint32_t flags = 0;
    std::uint32_t reserved = 0;
};

struct SnapshotDirectEvidenceHeader {
    std::uint64_t magic = kSnapshotEvidenceTableMagic ^ 0x444952454354ull;
    std::uint32_t version = kSnapshotFormatVersion;
    std::uint32_t header_size = sizeof(SnapshotDirectEvidenceHeader);
    std::uint64_t row_count = 0;
    std::uint64_t evidence_count = 0;
    std::uint64_t payload_file_size = 0;
};

struct SnapshotDirectEvidenceEntry {
    std::uint64_t payload_offset = 0;
    std::uint64_t generation = 0;
    std::uint64_t checksum = 0;
    std::int32_t sector = 0;
    std::uint32_t payload_count = 0;
    std::uint8_t channel = 0;
    std::uint8_t endpoint_source = 0;
    std::uint8_t payload_kind = 0;
    std::uint8_t flags = 0;
};

struct SnapshotEvidenceTableHeader {
    std::uint64_t magic = kSnapshotEvidenceTableMagic;
    std::uint32_t version = kSnapshotFormatVersion;
    std::uint32_t header_size = sizeof(SnapshotEvidenceTableHeader);
    std::uint64_t slot_count = 0;
    std::uint64_t evidence_count = 0;
    std::uint64_t payload_file_size = 0;
};

struct SnapshotEvidenceSlot {
    std::uint64_t node_id = kInvalidNodeId;
    std::int32_t sector = 0;
    std::uint32_t channel = 0;
    std::uint32_t endpoint_source = 0;
    std::uint32_t payload_kind = 0;
    std::uint32_t flags = 0;
    std::uint64_t payload_offset = 0;
    std::uint32_t payload_count = 0;
    std::uint32_t reserved = 0;
    std::uint64_t generation = 0;
    std::uint64_t checksum = 0;
};

struct MappedFile {
    ~MappedFile() { close(); }

    bool open_read_only(const std::filesystem::path& path) {
        close();
#ifdef _WIN32
        file_ = ::CreateFileW(path.wstring().c_str(), GENERIC_READ, FILE_SHARE_READ, nullptr,
                              OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
        if (file_ == INVALID_HANDLE_VALUE) {
            return false;
        }
        LARGE_INTEGER file_size;
        if (!::GetFileSizeEx(file_, &file_size) || file_size.QuadPart < 0) {
            close();
            return false;
        }
        size_ = static_cast<std::size_t>(file_size.QuadPart);
        if (size_ == 0) {
            return true;
        }
        mapping_ = ::CreateFileMappingW(file_, nullptr, PAGE_READONLY, 0, 0, nullptr);
        if (mapping_ == nullptr) {
            close();
            return false;
        }
        view_ = ::MapViewOfFile(mapping_, FILE_MAP_READ, 0, 0, 0);
        if (view_ == nullptr) {
            close();
            return false;
        }
        data_ = static_cast<const std::byte*>(view_);
        return true;
#else
        fd_ = ::open(path.c_str(), O_RDONLY);
        if (fd_ < 0) {
            return false;
        }
        struct stat statbuf;
        if (::fstat(fd_, &statbuf) != 0 || statbuf.st_size < 0) {
            close();
            return false;
        }
        size_ = static_cast<std::size_t>(statbuf.st_size);
        if (size_ == 0) {
            return true;
        }
        void* mapped = ::mmap(nullptr, size_, PROT_READ, MAP_PRIVATE, fd_, 0);
        if (mapped == MAP_FAILED) {
            close();
            return false;
        }
#ifdef MADV_RANDOM
        ::madvise(mapped, size_, MADV_RANDOM);
#endif
        data_ = static_cast<const std::byte*>(mapped);
        return true;
#endif
    }

    void close() {
#ifdef _WIN32
        if (view_ != nullptr) {
            ::UnmapViewOfFile(view_);
            view_ = nullptr;
        }
        if (mapping_ != nullptr) {
            ::CloseHandle(mapping_);
            mapping_ = nullptr;
        }
        if (file_ != INVALID_HANDLE_VALUE) {
            ::CloseHandle(file_);
            file_ = INVALID_HANDLE_VALUE;
        }
#else
        if (data_ != nullptr && size_ != 0) {
            ::munmap(const_cast<std::byte*>(data_), size_);
        }
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
#endif
        data_ = nullptr;
        size_ = 0;
    }

    std::span<const std::byte> bytes() const noexcept {
        return data_ == nullptr ? std::span<const std::byte>{} : std::span<const std::byte>(data_, size_);
    }

    std::size_t size() const noexcept { return size_; }

#ifdef _WIN32
    HANDLE file_ = INVALID_HANDLE_VALUE;
    HANDLE mapping_ = nullptr;
    void* view_ = nullptr;
#else
    int fd_ = -1;
#endif
    const std::byte* data_ = nullptr;
    std::size_t size_ = 0;
};

bool intervals_equal(const std::vector<Interval>& lhs,
                     const std::vector<Interval>& rhs,
                     double tolerance) {
    if (lhs.size() != rhs.size()) {
        return false;
    }
    for (std::size_t dim = 0; dim < lhs.size(); ++dim) {
        if (std::abs(lhs[dim].lo - rhs[dim].lo) > tolerance ||
            std::abs(lhs[dim].hi - rhs[dim].hi) > tolerance) {
            return false;
        }
    }
    return true;
}

bool interval_contains(const Interval& outer, const Interval& inner, double tolerance) {
    return outer.lo <= inner.lo + tolerance && outer.hi + tolerance >= inner.hi;
}

bool box_contains(const std::vector<Interval>& outer,
                  const std::vector<Interval>& inner,
                  double tolerance) {
    if (outer.size() != inner.size()) {
        return false;
    }
    for (std::size_t dim = 0; dim < outer.size(); ++dim) {
        if (!interval_contains(outer[dim], inner[dim], tolerance)) {
            return false;
        }
    }
    return true;
}

bool box_overlaps(const std::vector<Interval>& lhs,
                  const std::vector<Interval>& rhs,
                  double tolerance) {
    if (lhs.size() != rhs.size()) {
        return false;
    }
    for (std::size_t dim = 0; dim < lhs.size(); ++dim) {
        if (lhs[dim].hi + tolerance < rhs[dim].lo || rhs[dim].hi + tolerance < lhs[dim].lo) {
            return false;
        }
    }
    return true;
}

template <typename T>
bool write_object(std::ofstream& out, const T& value) {
    out.write(reinterpret_cast<const char*>(&value), static_cast<std::streamsize>(sizeof(T)));
    return static_cast<bool>(out);
}

template <typename T>
std::span<const T> span_after_header(std::span<const std::byte> bytes, std::size_t header_size, std::size_t count) {
    const auto payload_bytes = count * sizeof(T);
    if (payload_bytes / sizeof(T) != count || bytes.size() != header_size + payload_bytes) {
        return {};
    }
    return std::span<const T>(reinterpret_cast<const T*>(bytes.data() + header_size), count);
}

}  // namespace

struct LectReadSnapshot::Impl {
    std::filesystem::path path;
    SnapshotManifestHeader manifest;
    std::vector<Interval> root;
    std::shared_ptr<MappedFile> manifest_file;
    std::shared_ptr<MappedFile> nodes_file;
    std::shared_ptr<MappedFile> direct_evidence_file;
    std::shared_ptr<MappedFile> evidence_table_file;
    std::shared_ptr<MappedFile> payload_file;
    std::span<const SnapshotNodeRow> nodes;
    std::span<const SnapshotDirectEvidenceEntry> direct_evidence;
    std::span<const SnapshotEvidenceSlot> evidence_slots;
    // Populated lazily on endpoint/box exact lookups; avoids O(node_count) work at open.
    mutable std::unordered_map<SnapshotBoxIndexKey, NodeId, SnapshotBoxIndexKeyHash> exact_box_index;

    bool has_node(NodeId node_id) const noexcept {
        const auto index = static_cast<std::size_t>(node_id);
        return node_id != kInvalidNodeId && index < nodes.size() &&
               (nodes[index].flags & kSnapshotNodePresent) != 0;
    }

    const SnapshotNodeRow* node(NodeId node_id) const noexcept {
        return has_node(node_id) ? &nodes[static_cast<std::size_t>(node_id)] : nullptr;
    }

    static bool valid_cached_node(NodeId node_id) noexcept {
        return node_id != kInvalidNodeId;
    }

    std::optional<NodeId> cached_exact_box_node(const SnapshotBoxIndexKey& key) const {
        const auto found = exact_box_index.find(key);
        if (found == exact_box_index.end() || !valid_cached_node(found->second)) {
            return std::nullopt;
        }
        return found->second;
    }

    void remember_exact_box_node(const SnapshotBoxIndexKey& key, NodeId node_id) const {
        if (!valid_cached_node(node_id)) {
            return;
        }
        auto [it, inserted] = exact_box_index.emplace(key, node_id);
        if (!inserted && it->second != node_id) {
            it->second = kInvalidNodeId;
        }
    }

    std::optional<NodeId> locate_exact_box_node(const std::vector<Interval>& box_intervals,
                                                  double tolerance) const {
        if (!has_node(0) || box_intervals.size() != root.size()) {
            return std::nullopt;
        }
        NodeId cursor = 0;
        auto intervals = root;
        while (has_node(cursor)) {
            if (intervals_equal(intervals, box_intervals, tolerance)) {
                return cursor;
            }
            const auto* row = node(cursor);
            if (row == nullptr || (row->left == kInvalidNodeId && row->right == kInvalidNodeId) ||
                row->split_dim < 0 || row->split_dim >= static_cast<int>(intervals.size())) {
                break;
            }
            const auto dim = static_cast<std::size_t>(row->split_dim);
            if (box_intervals[dim].hi <= row->split_value + tolerance && has_node(row->left)) {
                intervals[dim].hi = row->split_value;
                cursor = row->left;
            } else if (box_intervals[dim].lo + tolerance >= row->split_value && has_node(row->right)) {
                intervals[dim].lo = row->split_value;
                cursor = row->right;
            } else {
                break;
            }
        }
        return std::nullopt;
    }
};

std::optional<EvidenceRecordView> direct_evidence_view(std::span<const SnapshotDirectEvidenceEntry> direct_evidence,
                                                       const EvidenceKey& key,
                                                       const std::shared_ptr<MappedFile>& payload_file) {
    const auto direct_index = static_cast<std::size_t>(key.node_id);
    if (direct_index >= direct_evidence.size()) {
        return std::nullopt;
    }
    const auto& direct = direct_evidence[direct_index];
    if ((direct.flags & kSnapshotEvidencePresent) == 0 ||
        direct.sector != key.sector ||
        direct.channel != static_cast<std::uint8_t>(key.channel) ||
        direct.endpoint_source != static_cast<std::uint8_t>(key.endpoint_source) ||
        direct.payload_kind != static_cast<std::uint8_t>(key.payload_kind)) {
        return std::nullopt;
    }
    const auto payload_bytes = static_cast<std::uint64_t>(direct.payload_count) * sizeof(std::uint16_t);
    if (direct.payload_offset > payload_file->size() || payload_bytes > payload_file->size() - direct.payload_offset) {
        return std::nullopt;
    }
    const auto payload = payload_file->bytes();
    EvidenceRecordView view;
    view.key = key;
    view.key.node_path = {};
    view.key.node_path_valid = false;
    view.child_hull = (direct.flags & kSnapshotEvidenceChildHull) != 0;
    view.unavailable = (direct.flags & kSnapshotEvidenceUnavailable) != 0;
    view.generation = direct.generation;
    view.checksum = direct.checksum;
    auto decoded = decode_half_payload(
        reinterpret_cast<const std::uint16_t*>(payload.data() + static_cast<std::size_t>(direct.payload_offset)),
        direct.payload_count);
    view.payload = std::span<const float>(decoded->data(), decoded->size());
    view.storage_owner = std::static_pointer_cast<const void>(std::move(decoded));
    return view;
}

std::optional<EvidenceRecordView> evidence_slot_view(const SnapshotEvidenceSlot& slot,
                                                     const EvidenceKey& key,
                                                     const std::shared_ptr<MappedFile>& payload_file) {
    const auto payload_bytes = static_cast<std::uint64_t>(slot.payload_count) * sizeof(std::uint16_t);
    if (slot.payload_offset > payload_file->size() || payload_bytes > payload_file->size() - slot.payload_offset) {
        return std::nullopt;
    }
    const auto payload = payload_file->bytes();
    EvidenceRecordView view;
    view.key = key;
    view.key.node_path = {};
    view.key.node_path_valid = false;
    view.child_hull = (slot.flags & kSnapshotEvidenceChildHull) != 0;
    view.unavailable = (slot.flags & kSnapshotEvidenceUnavailable) != 0;
    view.generation = slot.generation;
    view.checksum = slot.checksum;
    auto decoded = decode_half_payload(
        reinterpret_cast<const std::uint16_t*>(payload.data() + static_cast<std::size_t>(slot.payload_offset)),
        slot.payload_count);
    view.payload = std::span<const float>(decoded->data(), decoded->size());
    view.storage_owner = std::static_pointer_cast<const void>(std::move(decoded));
    return view;
}

std::optional<EvidenceRecordView> lookup_evidence_slot(std::span<const SnapshotEvidenceSlot> evidence_slots,
                                                       const EvidenceKey& key,
                                                       const std::shared_ptr<MappedFile>& payload_file) {
    if (evidence_slots.empty()) {
        return std::nullopt;
    }
    const auto hash = hash_evidence_key(key.node_id, key.sector, key.channel, key.endpoint_source, key.payload_kind);
    std::size_t position = static_cast<std::size_t>(hash) & (evidence_slots.size() - 1u);
    for (std::size_t probe = 0; probe < evidence_slots.size(); ++probe) {
        const auto& slot = evidence_slots[position];
        if ((slot.flags & kSnapshotEvidencePresent) == 0) {
            return std::nullopt;
        }
        if (slot.node_id == key.node_id && slot.sector == key.sector &&
            slot.channel == static_cast<std::uint32_t>(key.channel) &&
            slot.endpoint_source == static_cast<std::uint32_t>(key.endpoint_source) &&
            slot.payload_kind == static_cast<std::uint32_t>(key.payload_kind)) {
            return evidence_slot_view(slot, key, payload_file);
        }
        position = (position + 1u) & (evidence_slots.size() - 1u);
    }
    return std::nullopt;
}

std::optional<EvidenceRecordView> lookup_evidence_uncached(std::span<const SnapshotDirectEvidenceEntry> direct_evidence,
                                                           std::span<const SnapshotEvidenceSlot> evidence_slots,
                                                           const EvidenceKey& key,
                                                           const std::shared_ptr<MappedFile>& payload_file) {
    if (auto view = direct_evidence_view(direct_evidence, key, payload_file)) {
        return view;
    }
    return lookup_evidence_slot(evidence_slots, key, payload_file);
}

std::optional<EvidenceRecordView> lookup_endpoint_exact_uncached(std::span<const SnapshotNodeRow> nodes,
                                                                 const std::vector<Interval>& root,
                                                                 std::span<const SnapshotDirectEvidenceEntry> direct_evidence,
                                                                 std::span<const SnapshotEvidenceSlot> evidence_slots,
                                                                 const std::shared_ptr<MappedFile>& payload_file,
                                                                 const std::vector<Interval>& box_intervals,
                                                                 double tolerance,
                                                                 EvidenceKey key_template) {
    auto has_node = [&](NodeId node_id) {
        const auto index = static_cast<std::size_t>(node_id);
        return node_id != kInvalidNodeId && index < nodes.size() &&
               (nodes[index].flags & kSnapshotNodePresent) != 0;
    };
    auto node = [&](NodeId node_id) -> const SnapshotNodeRow* {
        return has_node(node_id) ? &nodes[static_cast<std::size_t>(node_id)] : nullptr;
    };

    NodeId cursor = 0;
    auto intervals = root;
    while (has_node(cursor)) {
        if (intervals_equal(intervals, box_intervals, tolerance)) {
            key_template.node_id = cursor;
            key_template.node_path = {};
            key_template.node_path_valid = false;
            return lookup_evidence_uncached(direct_evidence, evidence_slots, key_template, payload_file);
        }
        const auto* row = node(cursor);
        if (row == nullptr || (row->left == kInvalidNodeId && row->right == kInvalidNodeId) ||
            row->split_dim < 0 || row->split_dim >= static_cast<int>(intervals.size())) {
            break;
        }
        const auto dim = static_cast<std::size_t>(row->split_dim);
        if (box_intervals[dim].hi <= row->split_value + tolerance && has_node(row->left)) {
            intervals[dim].hi = row->split_value;
            cursor = row->left;
        } else if (box_intervals[dim].lo + tolerance >= row->split_value && has_node(row->right)) {
            intervals[dim].lo = row->split_value;
            cursor = row->right;
        } else {
            break;
        }
    }
    return std::nullopt;
}

LectReadSnapshot::LectReadSnapshot() : impl_(std::make_unique<Impl>()) {}
LectReadSnapshot::~LectReadSnapshot() = default;
LectReadSnapshot::LectReadSnapshot(LectReadSnapshot&&) noexcept = default;
LectReadSnapshot& LectReadSnapshot::operator=(LectReadSnapshot&&) noexcept = default;

std::filesystem::path LectReadSnapshot::default_snapshot_path(const std::filesystem::path& legacy_root) {
    return legacy_root / "lect_snapshot";
}

bool LectReadSnapshot::build_from_legacy(const std::filesystem::path& legacy_root,
                                           const std::filesystem::path& snapshot_path,
                                           std::string* reason) {
    const auto values = read_manifest_values(legacy_manifest_path(legacy_root));
    if (values.empty()) {
        if (reason) *reason = "legacy manifest is missing or empty";
        return false;
    }
    const auto root = parse_root_intervals(values);
    if (root.empty()) {
        if (reason) *reason = "legacy manifest has no root intervals";
        return false;
    }

    const auto generation = get_u64(values, "generation");
    const auto node_count = get_u64(values, "node_count");
    const auto max_node_id = get_u64(values, "max_node_id");
    if (node_count == 0 || max_node_id == kInvalidNodeId || max_node_id + 1 < node_count) {
        if (reason) *reason = "legacy manifest node counts are invalid";
        return false;
    }
    if (max_node_id + 1 > node_count * 2) {
        if (reason) *reason = "snapshot direct node store requires dense node ids";
        return false;
    }

    std::vector<SnapshotNodeRow> node_rows(static_cast<std::size_t>(max_node_id) + 1u);
    std::vector<SnapshotDirectEvidenceEntry> direct_evidence_rows(static_cast<std::size_t>(max_node_id) + 1u);
    std::size_t loaded_nodes = 0;

    const auto ingest_node_entry = [&](const LegacyNodeIndexSidecarEntry& entry) -> bool {
        if (!valid_node_id(entry.node_id) || entry.node_id > max_node_id) {
            if (reason) *reason = "legacy node table contains an invalid node";
            return false;
        }
        auto& row = node_rows[static_cast<std::size_t>(entry.node_id)];
        if ((row.flags & kSnapshotNodePresent) != 0) {
            if (reason) *reason = "legacy node table contains duplicate nodes";
            return false;
        }
        row.parent = entry.parent;
        row.left = entry.left;
        row.right = entry.right;
        row.depth = entry.depth;
        row.split_dim = entry.split_dim;
        row.split_value = entry.split_value;
        row.generation = generation;
        row.flags = kSnapshotNodePresent;
        ++loaded_nodes;
        return true;
    };

    std::ifstream node_input(legacy_node_index_path(legacy_root), std::ios::binary);
    if (node_input) {
        LegacyNodeIndexSidecarHeader legacy_node_header;
        node_input.read(reinterpret_cast<char*>(&legacy_node_header), static_cast<std::streamsize>(sizeof(legacy_node_header)));
        if (!node_input || legacy_node_header.magic != kLegacyNodeIndexSidecarMagic ||
            legacy_node_header.header_size != sizeof(LegacyNodeIndexSidecarHeader) ||
            legacy_node_header.generation != generation || legacy_node_header.node_count != node_count ||
            legacy_node_header.max_node_id != max_node_id) {
            if (reason) *reason = "legacy nodes.index header is incompatible";
            return false;
        }
        for (std::uint64_t index = 0; index < legacy_node_header.entry_count; ++index) {
            LegacyNodeIndexSidecarEntry entry;
            node_input.read(reinterpret_cast<char*>(&entry), static_cast<std::streamsize>(sizeof(entry)));
            if (!node_input || !ingest_node_entry(entry)) {
                return false;
            }
        }
    } else {
        std::ifstream pages_input(legacy_nodes_pages_path(legacy_root));
        if (!pages_input) {
            if (reason) *reason = "legacy node table is missing: expected nodes.index or nodes.pages";
            return false;
        }
        std::string line;
        while (std::getline(pages_input, line)) {
            if (line.empty()) {
                continue;
            }
            const auto entry = parse_legacy_node_pages_record(line);
            if (!entry || !ingest_node_entry(*entry)) {
                if (reason && reason->empty()) *reason = "legacy nodes.pages contains an invalid node";
                return false;
            }
        }
    }
    if (loaded_nodes != node_count) {
        if (reason) *reason = "legacy node table count does not match manifest";
        return false;
    }

    std::uint64_t evidence_file_size = 0;
    std::uint64_t evidence_count = 0;
    std::vector<SnapshotEvidenceSlot> evidence_slots;
    {
        std::ifstream evidence_input(legacy_evidence_index_path(legacy_root), std::ios::binary);
        if (!evidence_input) {
            if (reason) *reason = "legacy evidence.index is missing";
            return false;
        }
        LegacyEvidenceIndexSidecarHeader legacy_evidence_header;
        evidence_input.read(reinterpret_cast<char*>(&legacy_evidence_header),
                            static_cast<std::streamsize>(sizeof(legacy_evidence_header)));
        if (!evidence_input || legacy_evidence_header.magic != kLegacyEvidenceIndexSidecarMagic ||
            legacy_evidence_header.header_size != sizeof(LegacyEvidenceIndexSidecarHeader)) {
            if (reason) *reason = "legacy evidence.index header is incompatible";
            return false;
        }
        evidence_file_size = legacy_evidence_header.evidence_file_size;
        evidence_count = legacy_evidence_header.entry_count;
        const auto slot_count = next_power_of_two(std::max<std::size_t>(16, static_cast<std::size_t>(evidence_count) * 2u));
        evidence_slots.resize(slot_count);
        for (auto& slot : evidence_slots) {
            slot.node_id = kInvalidNodeId;
        }
        for (std::uint64_t index = 0; index < evidence_count; ++index) {
            LegacyEvidenceIndexSidecarEntry raw;
            evidence_input.read(reinterpret_cast<char*>(&raw), static_cast<std::streamsize>(sizeof(raw)));
            if (!evidence_input || !valid_node_id(raw.node_id) || raw.node_id > max_node_id) {
                if (reason) *reason = "legacy evidence.index contains an invalid node id";
                return false;
            }
            constexpr std::uint64_t legacy_record_header_size = 60u;
            const auto relative_payload_offset = legacy_record_header_size +
                path_code_storage_bytes(raw.path_word_count);
            if (relative_payload_offset > raw.size || raw.offset + raw.size > evidence_file_size ||
                (raw.size - relative_payload_offset) % sizeof(std::uint16_t) != 0) {
                if (reason) *reason = "legacy evidence.index contains an invalid payload range";
                return false;
            }
            SnapshotEvidenceSlot slot;
            slot.node_id = raw.node_id;
            slot.sector = raw.sector;
            slot.channel = raw.channel;
            slot.endpoint_source = raw.endpoint_source;
            slot.payload_kind = raw.payload_kind;
            slot.flags = kSnapshotEvidencePresent;
            if ((raw.flags & 1u) != 0) slot.flags |= kSnapshotEvidenceChildHull;
            if ((raw.flags & 2u) != 0) slot.flags |= kSnapshotEvidenceUnavailable;
            slot.payload_offset = raw.offset + relative_payload_offset;
            slot.payload_count = static_cast<std::uint32_t>((raw.size - relative_payload_offset) / sizeof(std::uint16_t));
            slot.generation = raw.generation;
            slot.checksum = raw.checksum;

            auto& direct = direct_evidence_rows[static_cast<std::size_t>(raw.node_id)];
            if ((direct.flags & kSnapshotEvidencePresent) == 0) {
                direct.payload_offset = slot.payload_offset;
                direct.generation = slot.generation;
                direct.checksum = slot.checksum;
                direct.sector = raw.sector;
                direct.channel = static_cast<std::uint8_t>(raw.channel);
                direct.endpoint_source = static_cast<std::uint8_t>(raw.endpoint_source);
                direct.payload_kind = static_cast<std::uint8_t>(raw.payload_kind);
                direct.flags = static_cast<std::uint8_t>(slot.flags);
                direct.payload_count = slot.payload_count;
            }

            const auto hash = hash_evidence_key(slot.node_id,
                                                slot.sector,
                                                static_cast<EvidenceChannel>(slot.channel),
                                                static_cast<EndpointSource>(slot.endpoint_source),
                                                static_cast<EvidencePayloadKind>(slot.payload_kind));
            std::size_t position = static_cast<std::size_t>(hash) & (evidence_slots.size() - 1u);
            for (;;) {
                if ((evidence_slots[position].flags & kSnapshotEvidencePresent) == 0) {
                    evidence_slots[position] = slot;
                    break;
                }
                position = (position + 1u) & (evidence_slots.size() - 1u);
            }
        }
    }

    const auto staging = snapshot_path.string() + ".staging";
    std::error_code ignored;
    std::filesystem::remove_all(staging, ignored);
    std::filesystem::create_directories(staging, ignored);
    if (ignored) {
        if (reason) *reason = "failed to create snapshot staging directory";
        return false;
    }

    std::filesystem::copy_file(legacy_evidence_path(legacy_root), snapshot_payload_path(staging),
                               std::filesystem::copy_options::overwrite_existing, ignored);
    if (ignored) {
        if (reason) *reason = "failed to copy legacy evidence payload";
        return false;
    }

    {
        std::ofstream out(snapshot_nodes_path(staging), std::ios::binary | std::ios::trunc);
        const SnapshotNodesHeader header{kSnapshotNodesMagic, kSnapshotFormatVersion, sizeof(SnapshotNodesHeader),
                                   static_cast<std::uint64_t>(node_rows.size()), node_count, max_node_id};
        if (!write_object(out, header)) {
            if (reason) *reason = "failed to write snapshot nodes header";
            return false;
        }
        out.write(reinterpret_cast<const char*>(node_rows.data()),
                  static_cast<std::streamsize>(node_rows.size() * sizeof(SnapshotNodeRow)));
        if (!out) {
            if (reason) *reason = "failed to write snapshot nodes";
            return false;
        }
    }

    {
        std::ofstream out(snapshot_evidence_table_path(staging), std::ios::binary | std::ios::trunc);
        const SnapshotEvidenceTableHeader header{kSnapshotEvidenceTableMagic, kSnapshotFormatVersion,
                                           sizeof(SnapshotEvidenceTableHeader),
                                           static_cast<std::uint64_t>(evidence_slots.size()),
                                           evidence_count,
                                           evidence_file_size};
        if (!write_object(out, header)) {
            if (reason) *reason = "failed to write snapshot evidence table header";
            return false;
        }
        out.write(reinterpret_cast<const char*>(evidence_slots.data()),
                  static_cast<std::streamsize>(evidence_slots.size() * sizeof(SnapshotEvidenceSlot)));
        if (!out) {
            if (reason) *reason = "failed to write snapshot evidence table";
            return false;
        }
    }

    {
        std::ofstream out(snapshot_direct_evidence_path(staging), std::ios::binary | std::ios::trunc);
        const SnapshotDirectEvidenceHeader header{kSnapshotEvidenceTableMagic ^ 0x444952454354ull,
                                            kSnapshotFormatVersion,
                                            sizeof(SnapshotDirectEvidenceHeader),
                                            static_cast<std::uint64_t>(direct_evidence_rows.size()),
                                            evidence_count,
                                            evidence_file_size};
        if (!write_object(out, header)) {
            if (reason) *reason = "failed to write snapshot direct evidence header";
            return false;
        }

        out.write(reinterpret_cast<const char*>(direct_evidence_rows.data()),
                  static_cast<std::streamsize>(direct_evidence_rows.size() * sizeof(SnapshotDirectEvidenceEntry)));
        if (!out) {
            if (reason) *reason = "failed to write snapshot direct evidence rows";
            return false;
        }
    }

    {
        const SnapshotManifestHeader header{kSnapshotManifestMagic,
                                      kSnapshotFormatVersion,
                                      sizeof(SnapshotManifestHeader),
                                      generation,
                                      node_count,
                                      max_node_id,
                                      evidence_count,
                                      get_u64(values, "root_domain_fingerprint", fingerprint_intervals(root)),
                                      get_u64(values, "split_policy_hash"),
                                      evidence_file_size,
                                      static_cast<std::uint32_t>(root.size()),
                                      static_cast<std::uint32_t>(get_int(values, "split_strategy")),
                                      get_double(values, "split_min_width"),
                                      static_cast<std::uint32_t>(get_int(values, "split_midpoint", 1)),
                                      static_cast<std::uint32_t>(get_int(values, "split_deterministic_tie_break", 1))};
        std::ofstream out(snapshot_manifest_path(staging), std::ios::binary | std::ios::trunc);
        if (!write_object(out, header)) {
            if (reason) *reason = "failed to write snapshot manifest header";
            return false;
        }
        out.write(reinterpret_cast<const char*>(root.data()), static_cast<std::streamsize>(root.size() * sizeof(Interval)));
        if (!out) {
            if (reason) *reason = "failed to write snapshot manifest root intervals";
            return false;
        }
    }

    if (!replace_directory(staging, snapshot_path)) {
        if (reason) *reason = "failed to publish snapshot";
        return false;
    }
    return true;
}

bool LectReadSnapshot::open(const std::filesystem::path& snapshot_path, std::string* reason) {
    close();
    impl_->path = snapshot_path;
    impl_->manifest_file = std::make_shared<MappedFile>();
    impl_->nodes_file = std::make_shared<MappedFile>();
    impl_->direct_evidence_file = std::make_shared<MappedFile>();
    impl_->evidence_table_file = std::make_shared<MappedFile>();
    impl_->payload_file = std::make_shared<MappedFile>();

    if (!impl_->manifest_file->open_read_only(snapshot_manifest_path(snapshot_path))) {
        if (reason) *reason = "failed to mmap snapshot manifest";
        close();
        return false;
    }
    const auto manifest_bytes = impl_->manifest_file->bytes();
    if (manifest_bytes.size() < sizeof(SnapshotManifestHeader)) {
        if (reason) *reason = "snapshot manifest is truncated";
        close();
        return false;
    }
    std::memcpy(&impl_->manifest, manifest_bytes.data(), sizeof(SnapshotManifestHeader));
    if (impl_->manifest.magic != kSnapshotManifestMagic || impl_->manifest.version != kSnapshotFormatVersion ||
        impl_->manifest.header_size != sizeof(SnapshotManifestHeader)) {
        if (reason) *reason = "snapshot manifest header is incompatible";
        close();
        return false;
    }
    const auto root_bytes = static_cast<std::size_t>(impl_->manifest.root_dims) * sizeof(Interval);
    if (manifest_bytes.size() != sizeof(SnapshotManifestHeader) + root_bytes) {
        if (reason) *reason = "snapshot manifest root interval payload is malformed";
        close();
        return false;
    }
    impl_->root.assign(reinterpret_cast<const Interval*>(manifest_bytes.data() + sizeof(SnapshotManifestHeader)),
                       reinterpret_cast<const Interval*>(manifest_bytes.data() + sizeof(SnapshotManifestHeader) + root_bytes));

    if (!impl_->nodes_file->open_read_only(snapshot_nodes_path(snapshot_path))) {
        if (reason) *reason = "failed to mmap snapshot nodes";
        close();
        return false;
    }
    const auto node_bytes = impl_->nodes_file->bytes();
    if (node_bytes.size() < sizeof(SnapshotNodesHeader)) {
        if (reason) *reason = "snapshot nodes file is truncated";
        close();
        return false;
    }
    SnapshotNodesHeader nodes_header;
    std::memcpy(&nodes_header, node_bytes.data(), sizeof(nodes_header));
    if (nodes_header.magic != kSnapshotNodesMagic || nodes_header.version != kSnapshotFormatVersion ||
        nodes_header.header_size != sizeof(SnapshotNodesHeader) ||
        nodes_header.node_count != impl_->manifest.node_count ||
        nodes_header.max_node_id != impl_->manifest.max_node_id) {
        if (reason) *reason = "snapshot nodes header is incompatible";
        close();
        return false;
    }
    impl_->nodes = span_after_header<SnapshotNodeRow>(node_bytes, sizeof(SnapshotNodesHeader),
                                                static_cast<std::size_t>(nodes_header.row_count));
    if (impl_->nodes.empty() && nodes_header.row_count != 0) {
        if (reason) *reason = "snapshot nodes rows are malformed";
        close();
        return false;
    }
    if (nodes_header.row_count != 0 && !impl_->has_node(0)) {
        if (reason) *reason = "snapshot root node is missing";
        close();
        return false;
    }

    if (!impl_->direct_evidence_file->open_read_only(snapshot_direct_evidence_path(snapshot_path))) {
        if (reason) *reason = "failed to mmap snapshot direct evidence";
        close();
        return false;
    }
    const auto direct_bytes = impl_->direct_evidence_file->bytes();
    if (direct_bytes.size() < sizeof(SnapshotDirectEvidenceHeader)) {
        if (reason) *reason = "snapshot direct evidence file is truncated";
        close();
        return false;
    }
    SnapshotDirectEvidenceHeader direct_header;
    std::memcpy(&direct_header, direct_bytes.data(), sizeof(direct_header));
    if (direct_header.magic != (kSnapshotEvidenceTableMagic ^ 0x444952454354ull) ||
        direct_header.version != kSnapshotFormatVersion ||
        direct_header.header_size != sizeof(SnapshotDirectEvidenceHeader) ||
        direct_header.row_count != nodes_header.row_count ||
        direct_header.evidence_count != impl_->manifest.evidence_count ||
        direct_header.payload_file_size != impl_->manifest.payload_file_size) {
        if (reason) *reason = "snapshot direct evidence header is incompatible";
        close();
        return false;
    }
    impl_->direct_evidence = span_after_header<SnapshotDirectEvidenceEntry>(direct_bytes,
                                                                      sizeof(SnapshotDirectEvidenceHeader),
                                                                      static_cast<std::size_t>(direct_header.row_count));
    if (impl_->direct_evidence.empty() && direct_header.row_count != 0) {
        if (reason) *reason = "snapshot direct evidence rows are malformed";
        close();
        return false;
    }

    if (!impl_->evidence_table_file->open_read_only(snapshot_evidence_table_path(snapshot_path))) {
        if (reason) *reason = "failed to mmap snapshot evidence table";
        close();
        return false;
    }
    const auto evidence_bytes = impl_->evidence_table_file->bytes();
    if (evidence_bytes.size() < sizeof(SnapshotEvidenceTableHeader)) {
        if (reason) *reason = "snapshot evidence table is truncated";
        close();
        return false;
    }
    SnapshotEvidenceTableHeader evidence_header;
    std::memcpy(&evidence_header, evidence_bytes.data(), sizeof(evidence_header));
    if (evidence_header.magic != kSnapshotEvidenceTableMagic || evidence_header.version != kSnapshotFormatVersion ||
        evidence_header.header_size != sizeof(SnapshotEvidenceTableHeader) ||
        evidence_header.evidence_count != impl_->manifest.evidence_count ||
        evidence_header.payload_file_size != impl_->manifest.payload_file_size ||
        evidence_header.slot_count == 0 || (evidence_header.slot_count & (evidence_header.slot_count - 1u)) != 0) {
        if (reason) *reason = "snapshot evidence table header is incompatible";
        close();
        return false;
    }
    impl_->evidence_slots = span_after_header<SnapshotEvidenceSlot>(evidence_bytes, sizeof(SnapshotEvidenceTableHeader),
                                                              static_cast<std::size_t>(evidence_header.slot_count));
    if (impl_->evidence_slots.empty()) {
        if (reason) *reason = "snapshot evidence table slots are malformed";
        close();
        return false;
    }

    if (!impl_->payload_file->open_read_only(snapshot_payload_path(snapshot_path)) ||
        impl_->payload_file->size() != impl_->manifest.payload_file_size) {
        if (reason) *reason = "failed to mmap snapshot payload";
        close();
        return false;
    }
    return true;
}

void LectReadSnapshot::close() {
    if (!impl_) {
        return;
    }
    impl_->nodes = {};
    impl_->direct_evidence = {};
    impl_->evidence_slots = {};
    impl_->exact_box_index.clear();
    impl_->root.clear();
    impl_->manifest = SnapshotManifestHeader{};
    impl_->manifest_file.reset();
    impl_->nodes_file.reset();
    impl_->direct_evidence_file.reset();
    impl_->evidence_table_file.reset();
    impl_->payload_file.reset();
    impl_->path.clear();
}

bool LectReadSnapshot::is_open() const noexcept {
        return impl_ && impl_->manifest_file && impl_->nodes_file && impl_->direct_evidence_file &&
            impl_->evidence_table_file && impl_->payload_file;
}

std::size_t LectReadSnapshot::node_count() const noexcept {
    return impl_ ? static_cast<std::size_t>(impl_->manifest.node_count) : 0;
}

std::size_t LectReadSnapshot::evidence_count() const noexcept {
    return impl_ ? static_cast<std::size_t>(impl_->manifest.evidence_count) : 0;
}

std::uint64_t LectReadSnapshot::generation() const noexcept {
    return impl_ ? impl_->manifest.generation : 0;
}

const std::vector<Interval>& LectReadSnapshot::root_intervals() const noexcept {
    return impl_->root;
}

BoxKey LectReadSnapshot::make_box_key(std::vector<Interval> intervals) const {
    BoxKey key;
    key.intervals = std::move(intervals);
    key.root_domain_fingerprint = impl_->manifest.root_domain_fingerprint;
    key.split_policy_hash = impl_->manifest.split_policy_hash;
    key.tolerance = 1e-12;
    return key;
}

std::optional<std::vector<Interval>> LectReadSnapshot::node_box(NodeId node_id) const {
    if (!impl_->has_node(node_id)) {
        return std::nullopt;
    }
    if (node_id == 0) {
        return impl_->root;
    }
    std::array<NodeId, 256> inline_lineage;
    std::vector<NodeId> overflow_lineage;
    NodeId cursor = node_id;
    std::size_t lineage_size = 0;
    while (cursor != 0) {
        const auto* row = impl_->node(cursor);
        if (row == nullptr || !impl_->has_node(row->parent)) {
            return std::nullopt;
        }
        if (lineage_size < inline_lineage.size()) {
            inline_lineage[lineage_size] = cursor;
        } else {
            if (overflow_lineage.empty()) {
                overflow_lineage.assign(inline_lineage.begin(), inline_lineage.end());
            }
            overflow_lineage.push_back(cursor);
        }
        ++lineage_size;
        cursor = row->parent;
    }
    auto intervals = impl_->root;
    NodeId parent_id = 0;
    auto child_at = [&](std::size_t reverse_index) {
        const auto index = lineage_size - reverse_index - 1u;
        return overflow_lineage.empty() ? inline_lineage[index] : overflow_lineage[index];
    };
    for (std::size_t index = 0; index < lineage_size; ++index) {
        const NodeId child_id = child_at(index);
        const auto* parent = impl_->node(parent_id);
        if (parent == nullptr || parent->split_dim < 0 ||
            parent->split_dim >= static_cast<int>(intervals.size())) {
            return std::nullopt;
        }
        auto& interval = intervals[static_cast<std::size_t>(parent->split_dim)];
        if (child_id == parent->left) {
            interval.hi = parent->split_value;
        } else if (child_id == parent->right) {
            interval.lo = parent->split_value;
        } else {
            return std::nullopt;
        }
        parent_id = child_id;
    }
    return intervals;
}

BoxLookupResult LectReadSnapshot::box_to_node_exact(const BoxKey& box) const {
    BoxLookupResult result;
    if (box.root_domain_fingerprint != impl_->manifest.root_domain_fingerprint) {
        result.reason = "root domain fingerprint differs";
        return result;
    }
    if (box.split_policy_hash != impl_->manifest.split_policy_hash) {
        result.reason = "split policy hash differs";
        return result;
    }
    if (box.intervals.size() != impl_->root.size()) {
        result.reason = "dimension mismatch";
        return result;
    }
    const auto box_index_key = make_snapshot_box_index_key(box.intervals);
    if (const auto cached = impl_->cached_exact_box_node(box_index_key)) {
        result.found = true;
        result.node_id = *cached;
        return result;
    }
    const auto located = impl_->locate_exact_box_node(box.intervals, box.tolerance);
    if (!located) {
        result.reason = "box does not match a stored node";
        return result;
    }
    impl_->remember_exact_box_node(box_index_key, *located);
    result.found = true;
    result.node_id = *located;
    return result;
}

std::vector<NodeId> LectReadSnapshot::range_query(const std::vector<Interval>& box,
                                                    RangeQueryMode mode,
                                                    LectDatabaseStats* stats) const {
    std::vector<NodeId> out;
    if (box.size() != impl_->root.size() || !impl_->has_node(0)) {
        return out;
    }
    struct StackItem {
        NodeId node_id = kInvalidNodeId;
        std::vector<Interval> intervals;
    };
    std::vector<StackItem> stack;
    stack.push_back({0, impl_->root});
    while (!stack.empty()) {
        auto item = std::move(stack.back());
        stack.pop_back();
        if (stats != nullptr) {
            ++stats->range_nodes_visited;
        }
        const bool match = [&]() {
            switch (mode) {
                case RangeQueryMode::Containing:
                    return box_contains(item.intervals, box, 1e-12);
                case RangeQueryMode::ContainedBy:
                    return box_contains(box, item.intervals, 1e-12);
                case RangeQueryMode::Intersecting:
                case RangeQueryMode::CoveringFrontier:
                    return box_overlaps(item.intervals, box, 1e-12);
            }
            return false;
        }();
        if (!match) {
            continue;
        }
        out.push_back(item.node_id);
        const auto* row = impl_->node(item.node_id);
        if (row == nullptr || row->split_dim < 0 || row->split_dim >= static_cast<int>(item.intervals.size())) {
            continue;
        }
        const auto dim = static_cast<std::size_t>(row->split_dim);
        if (impl_->has_node(row->right)) {
            auto right = item.intervals;
            right[dim].lo = row->split_value;
            stack.push_back({row->right, std::move(right)});
        }
        if (impl_->has_node(row->left)) {
            auto left = std::move(item.intervals);
            left[dim].hi = row->split_value;
            stack.push_back({row->left, std::move(left)});
        }
    }
    return out;
}

std::optional<EvidenceRecordView> LectReadSnapshot::evidence(const EvidenceKey& key) const {
    if (!impl_->has_node(key.node_id)) {
        return std::nullopt;
    }
    return lookup_evidence_uncached(impl_->direct_evidence,
                                    impl_->evidence_slots,
                                    key,
                                    impl_->payload_file);
}

std::optional<EvidenceRecordView> LectReadSnapshot::endpoint_for_box_exact(const BoxKey& box,
                                                                             EvidenceKey key_template) const {
    if (box.root_domain_fingerprint != impl_->manifest.root_domain_fingerprint ||
        box.split_policy_hash != impl_->manifest.split_policy_hash ||
        box.intervals.size() != impl_->root.size()) {
        return std::nullopt;
    }
    return endpoint_for_box_exact(box.intervals, key_template, box.tolerance);
}

std::optional<EvidenceRecordView> LectReadSnapshot::endpoint_for_box_exact(const std::vector<Interval>& intervals,
                                                                             EvidenceKey key_template,
                                                                             double tolerance) const {
    if (intervals.size() != impl_->root.size()) {
        return std::nullopt;
    }
    const auto box_index_key = make_snapshot_box_index_key(intervals);
    if (const auto cached = impl_->cached_exact_box_node(box_index_key)) {
        key_template.node_id = *cached;
        key_template.node_path = {};
        key_template.node_path_valid = false;
        if (auto view = lookup_evidence_uncached(impl_->direct_evidence,
                                                 impl_->evidence_slots,
                                                 key_template,
                                                 impl_->payload_file)) {
            return view;
        }
    }
    if (const auto located = impl_->locate_exact_box_node(intervals, tolerance)) {
        key_template.node_id = *located;
        key_template.node_path = {};
        key_template.node_path_valid = false;
        if (auto view = lookup_evidence_uncached(impl_->direct_evidence,
                                                 impl_->evidence_slots,
                                                 key_template,
                                                 impl_->payload_file)) {
            impl_->remember_exact_box_node(box_index_key, *located);
            return view;
        }
    }
    if (auto view = lookup_endpoint_exact_uncached(impl_->nodes,
                                                   impl_->root,
                                                   impl_->direct_evidence,
                                                   impl_->evidence_slots,
                                                   impl_->payload_file,
                                                   intervals,
                                                   tolerance,
                                                   key_template)) {
        if (valid_node_id(key_template.node_id)) {
            impl_->remember_exact_box_node(box_index_key, key_template.node_id);
        }
        return view;
    }
    return std::nullopt;
}

}  // namespace rbf::lect_database
