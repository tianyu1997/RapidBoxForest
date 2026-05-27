#include <rbf/lect_database/database.h>

#include <algorithm>
#include <cerrno>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <system_error>
#include <type_traits>
#include <unordered_map>

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

constexpr std::size_t kBufferedIoBytes = 1u << 20;
constexpr std::size_t kMaxResidentEvidenceRecords = 8192;
constexpr std::uint64_t kEvidenceAppendsPerFlush = 1024;
constexpr std::size_t kEvidenceIndexLoadFactorNumerator = 13;
constexpr std::size_t kEvidenceIndexLoadFactorDenominator = 16;
constexpr std::uint32_t kEvidenceStoreSchemaVersion = 1;
constexpr std::uint64_t kEvidenceStoreMagic = 0x3156454242464652ull;
constexpr std::uint32_t kEvidenceIndexSidecarSchemaVersion = 2;
constexpr std::uint64_t kEvidenceIndexSidecarMagic = 0x3158444945424652ull;
constexpr std::string_view kLegacyNodeIdScheme = "bfs_heap_v1";
constexpr std::string_view kCurrentNodeIdScheme = "path_handle_v3";

enum EvidenceIndexSidecarFlags : std::uint32_t {
    kEvidenceIndexFlagChildHull = 1u << 0,
    kEvidenceIndexFlagUnavailable = 1u << 1,
};

#pragma pack(push, 1)
struct EvidenceStoreFileHeader {
    std::uint64_t magic = kEvidenceStoreMagic;
    std::uint32_t version = kEvidenceStoreSchemaVersion;
    std::uint32_t header_size = 32u;
    std::uint64_t reserved0 = 0;
    std::uint64_t reserved1 = 0;
};

struct EvidenceStoreRecordHeader {
    std::uint64_t node_id = 0;
    std::int32_t sector = 0;
    std::uint32_t channel = 0;
    std::uint32_t endpoint_source = 0;
    std::uint32_t payload_kind = 0;
    std::uint32_t flags = 0;
    std::uint32_t payload_count = 0;
    std::uint32_t record_size = 0;
    std::uint64_t generation = 0;
    std::uint64_t checksum = 0;
    std::uint32_t reserved0 = 0;
    std::uint32_t reserved1 = 0;
};

struct EvidenceIndexSidecarHeader {
    std::uint64_t magic = kEvidenceIndexSidecarMagic;
    std::uint32_t version = kEvidenceIndexSidecarSchemaVersion;
    std::uint32_t reserved = 0;
    std::uint64_t evidence_file_size = 0;
    std::uint64_t entry_count = 0;
};

struct EvidenceIndexSidecarEntry {
    std::uint64_t node_id = 0;
    std::int32_t sector = 0;
    std::uint32_t channel = 0;
    std::uint32_t endpoint_source = 0;
    std::uint32_t payload_kind = 0;
    std::uint32_t flags = 0;
    std::uint32_t size = 0;
    std::uint32_t reserved = 0;
    std::uint64_t offset = 0;
    std::uint64_t generation = 0;
    std::uint64_t checksum = 0;
};
#pragma pack(pop)

static_assert(sizeof(EvidenceStoreFileHeader) == 32);
static_assert(sizeof(EvidenceStoreRecordHeader) == 60);
static_assert(sizeof(EvidenceIndexSidecarHeader) == 32);
static_assert(sizeof(EvidenceIndexSidecarEntry) == 60);
static_assert(alignof(EvidenceIndexSidecarHeader) == 1);
static_assert(alignof(EvidenceIndexSidecarEntry) == 1);
static_assert(std::is_trivially_copyable_v<EvidenceStoreFileHeader>);
static_assert(std::is_trivially_copyable_v<EvidenceStoreRecordHeader>);
static_assert(std::is_trivially_copyable_v<EvidenceIndexSidecarHeader>);
static_assert(std::is_trivially_copyable_v<EvidenceIndexSidecarEntry>);

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

std::size_t next_power_of_two(std::size_t value) {
    std::size_t result = 1;
    while (result < value) {
        result <<= 1;
    }
    return result;
}

std::size_t evidence_index_slot_count(std::size_t item_count) {
    if (item_count == 0) {
        return 16;
    }
    const auto required_slots = (item_count * kEvidenceIndexLoadFactorDenominator +
                                 kEvidenceIndexLoadFactorNumerator - 1) /
        kEvidenceIndexLoadFactorNumerator;
    return next_power_of_two(std::max<std::size_t>(16, required_slots));
}

bool evidence_sidecar_entry_less(const EvidenceIndexSidecarEntry& lhs,
                                 const EvidenceIndexSidecarEntry& rhs) {
    if (lhs.offset != rhs.offset) {
        return lhs.offset < rhs.offset;
    }
    if (lhs.node_id != rhs.node_id) {
        return lhs.node_id < rhs.node_id;
    }
    if (lhs.sector != rhs.sector) {
        return lhs.sector < rhs.sector;
    }
    if (lhs.channel != rhs.channel) {
        return lhs.channel < rhs.channel;
    }
    if (lhs.endpoint_source != rhs.endpoint_source) {
        return lhs.endpoint_source < rhs.endpoint_source;
    }
    return lhs.payload_kind < rhs.payload_kind;
}

bool evidence_sidecar_offsets_sorted(std::span<const EvidenceIndexSidecarEntry> entries) {
    for (std::size_t index = 1; index < entries.size(); ++index) {
        if (evidence_sidecar_entry_less(entries[index], entries[index - 1])) {
            return false;
        }
    }
    return true;
}

template <typename T>
std::string to_text(const T& value) {
    std::ostringstream out;
    out << std::setprecision(17) << value;
    return out.str();
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
                      const std::string& fallback = {}) {
    const auto it = values.find(key);
    return it == values.end() ? fallback : it->second;
}

std::uint64_t get_u64(const std::unordered_map<std::string, std::string>& values,
                      const std::string& key,
                      std::uint64_t fallback = 0) {
    const auto text = get_value(values, key);
    return text.empty() ? fallback : static_cast<std::uint64_t>(std::stoull(text));
}

int get_int(const std::unordered_map<std::string, std::string>& values,
            const std::string& key,
            int fallback = 0) {
    const auto text = get_value(values, key);
    return text.empty() ? fallback : std::stoi(text);
}

double get_double(const std::unordered_map<std::string, std::string>& values,
                  const std::string& key,
                  double fallback = 0.0) {
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

std::uint64_t payload_checksum(std::span<const float> payload) {
    std::uint64_t hash = 1469598103934665603ull;
    if (!payload.empty()) {
        hash = stable_hash_append(hash, payload.data(), payload.size() * sizeof(float));
    }
    return hash;
}

std::string serialize_payload(const std::vector<float>& payload) {
    std::ostringstream out;
    out << payload.size();
    for (float value : payload) {
        out << ',' << std::setprecision(9) << value;
    }
    return out.str();
}

std::string_view trim_line_ending(std::string_view text) {
    while (!text.empty() && (text.back() == '\r' || text.back() == '\n')) {
        text.remove_suffix(1);
    }
    return text;
}

bool take_field(std::string_view* remainder, char delimiter, std::string_view* field) {
    const auto pos = remainder->find(delimiter);
    if (pos == std::string_view::npos) {
        return false;
    }
    *field = remainder->substr(0, pos);
    remainder->remove_prefix(pos + 1);
    return true;
}

template <typename Integer>
bool parse_integer(std::string_view text, Integer* value) {
    text = trim_line_ending(text);
    if (text.empty()) {
        return false;
    }
    Integer parsed = {};
    const char* begin = text.data();
    const char* end = begin + text.size();
    const auto [cursor, error] = std::from_chars(begin, end, parsed);
    if (error != std::errc() || cursor != end) {
        return false;
    }
    *value = parsed;
    return true;
}

struct ParsedEvidenceHeader {
    EvidenceKey key;
    bool child_hull = false;
    bool unavailable = false;
    std::uint64_t generation = 0;
    std::uint64_t checksum = 0;
    std::string_view payload_text;
};

EvidenceRecordView make_evidence_view(const std::shared_ptr<const EvidenceRecord>& record) {
    EvidenceRecordView view;
    if (!record) {
        return view;
    }
    view.key = record->key;
    view.child_hull = record->child_hull;
    view.unavailable = record->unavailable;
    view.generation = record->generation;
    view.checksum = record->checksum;
    view.payload = std::span<const float>(record->payload.data(), record->payload.size());
    view.storage = record;
    return view;
}

EvidenceRecord clone_evidence_record(const EvidenceRecordView& view) {
    EvidenceRecord record;
    record.key = view.key;
    record.child_hull = view.child_hull;
    record.unavailable = view.unavailable;
    record.generation = view.generation;
    record.checksum = view.checksum;
    record.payload.assign(view.payload.begin(), view.payload.end());
    return record;
}

std::uint32_t evidence_binary_record_size(std::size_t payload_count) {
    const auto payload_bytes = payload_count * sizeof(float);
    const auto total_bytes = sizeof(EvidenceStoreRecordHeader) + payload_bytes;
    return total_bytes > std::numeric_limits<std::uint32_t>::max()
        ? 0u
        : static_cast<std::uint32_t>(total_bytes);
}

EvidenceStoreRecordHeader make_evidence_store_record_header(const EvidenceRecord& record) {
    EvidenceStoreRecordHeader header;
    if (record.payload.size() > std::numeric_limits<std::uint32_t>::max()) {
        return header;
    }
    header.node_id = record.key.node_id;
    header.sector = static_cast<std::int32_t>(record.key.sector);
    header.channel = static_cast<std::uint32_t>(record.key.channel);
    header.endpoint_source = static_cast<std::uint32_t>(record.key.endpoint_source);
    header.payload_kind = static_cast<std::uint32_t>(record.key.payload_kind);
    header.flags = (record.child_hull ? kEvidenceIndexFlagChildHull : 0u) |
                   (record.unavailable ? kEvidenceIndexFlagUnavailable : 0u);
    header.payload_count = static_cast<std::uint32_t>(record.payload.size());
    header.record_size = evidence_binary_record_size(record.payload.size());
    header.generation = record.generation;
    header.checksum = record.checksum;
    return header;
}

bool read_evidence_store_file_header(std::ifstream& input, EvidenceStoreFileHeader* header) {
    input.clear();
    input.seekg(0, std::ios::beg);
    input.read(reinterpret_cast<char*>(header), static_cast<std::streamsize>(sizeof(*header)));
    if (!input) {
        input.clear();
        input.seekg(0, std::ios::beg);
        return false;
    }
    if (header->magic != kEvidenceStoreMagic ||
        header->version != kEvidenceStoreSchemaVersion ||
        header->header_size != sizeof(EvidenceStoreFileHeader)) {
        input.clear();
        input.seekg(0, std::ios::beg);
        return false;
    }
    return true;
}

bool write_evidence_store_file_header(std::ofstream& output) {
    const EvidenceStoreFileHeader header;
    output.write(reinterpret_cast<const char*>(&header), static_cast<std::streamsize>(sizeof(header)));
    return static_cast<bool>(output);
}

std::optional<EvidenceRecord> parse_binary_evidence_record(std::span<const std::byte> bytes) {
    if (bytes.size() < sizeof(EvidenceStoreRecordHeader)) {
        return std::nullopt;
    }
    EvidenceStoreRecordHeader header;
    std::memcpy(&header, bytes.data(), sizeof(header));
    if (header.record_size != bytes.size()) {
        return std::nullopt;
    }
    const auto payload_bytes = static_cast<std::size_t>(header.payload_count) * sizeof(float);
    if (sizeof(EvidenceStoreRecordHeader) + payload_bytes != bytes.size()) {
        return std::nullopt;
    }

    EvidenceRecord record;
    record.key.node_id = header.node_id;
    record.key.sector = header.sector;
    record.key.channel = static_cast<EvidenceChannel>(header.channel);
    record.key.endpoint_source = static_cast<EndpointSource>(header.endpoint_source);
    record.key.payload_kind = static_cast<EvidencePayloadKind>(header.payload_kind);
    record.child_hull = (header.flags & kEvidenceIndexFlagChildHull) != 0;
    record.unavailable = (header.flags & kEvidenceIndexFlagUnavailable) != 0;
    record.generation = header.generation;
    record.checksum = header.checksum;
    record.payload.resize(header.payload_count);
    if (payload_bytes > 0) {
        std::memcpy(record.payload.data(),
                    bytes.data() + sizeof(EvidenceStoreRecordHeader),
                    payload_bytes);
    }
    return record;
}

std::optional<ParsedEvidenceHeader> parse_evidence_header(std::string_view line) {
    line = trim_line_ending(line);
    std::string_view field;
    ParsedEvidenceHeader header;
    int channel = 0;
    int endpoint_source = 0;
    int payload_kind = 0;
    int child_hull = 0;
    int unavailable = 0;

    if (!take_field(&line, '|', &field) || !parse_integer(field, &header.key.node_id)) {
        return std::nullopt;
    }
    if (!take_field(&line, '|', &field) || !parse_integer(field, &header.key.sector)) {
        return std::nullopt;
    }
    if (!take_field(&line, '|', &field) || !parse_integer(field, &channel)) {
        return std::nullopt;
    }
    if (!take_field(&line, '|', &field) || !parse_integer(field, &endpoint_source)) {
        return std::nullopt;
    }
    if (!take_field(&line, '|', &field) || !parse_integer(field, &payload_kind)) {
        return std::nullopt;
    }
    if (!take_field(&line, '|', &field) || !parse_integer(field, &child_hull)) {
        return std::nullopt;
    }
    if (!take_field(&line, '|', &field) || !parse_integer(field, &unavailable)) {
        return std::nullopt;
    }
    if (!take_field(&line, '|', &field) || !parse_integer(field, &header.generation)) {
        return std::nullopt;
    }
    if (!take_field(&line, '|', &field) || !parse_integer(field, &header.checksum)) {
        return std::nullopt;
    }

    header.key.channel = static_cast<EvidenceChannel>(channel);
    header.key.endpoint_source = static_cast<EndpointSource>(endpoint_source);
    header.key.payload_kind = static_cast<EvidencePayloadKind>(payload_kind);
    header.child_hull = child_hull != 0;
    header.unavailable = unavailable != 0;
    header.payload_text = trim_line_ending(line);
    return header;
}

std::vector<float> parse_payload(std::string_view text) {
    text = trim_line_ending(text);
    if (text.empty()) {
        return {};
    }
    std::string buffer(text);
    char* cursor = buffer.data();
    char* end = nullptr;
    const auto n_values_raw = std::strtoull(cursor, &end, 10);
    if (end == cursor) {
        return {};
    }
    const std::size_t n_values = static_cast<std::size_t>(n_values_raw);
    std::vector<float> payload;
    payload.reserve(n_values);
    if (*end == ',') {
        ++end;
    }
    while (*end != '\0' && payload.size() < n_values) {
        char* value_end = end;
        const float value = std::strtof(end, &value_end);
        if (value_end == end) {
            break;
        }
        payload.push_back(value);
        end = value_end;
        if (*end == ',') {
            ++end;
        }
    }
    return payload;
}

std::string serialize_evidence_record(const EvidenceRecord& record) {
    std::ostringstream out;
    out << record.key.node_id << '|'
        << record.key.sector << '|'
        << static_cast<int>(record.key.channel) << '|'
        << static_cast<int>(record.key.endpoint_source) << '|'
        << static_cast<int>(record.key.payload_kind) << '|'
        << (record.child_hull ? 1 : 0) << '|'
        << (record.unavailable ? 1 : 0) << '|'
        << record.generation << '|'
        << record.checksum << '|'
        << serialize_payload(record.payload);
    return out.str();
}

std::optional<EvidenceRecord> parse_evidence_record(std::string_view line) {
    const auto header = parse_evidence_header(line);
    if (!header) {
        return std::nullopt;
    }
    EvidenceRecord record;
    record.key = header->key;
    record.child_hull = header->child_hull;
    record.unavailable = header->unavailable;
    record.generation = header->generation;
    record.checksum = header->checksum;
    record.payload = parse_payload(header->payload_text);
    return record;
}

bool payloads_equal(std::span<const float> lhs, std::span<const float> rhs) {
    if (lhs.size() != rhs.size()) {
        return false;
    }
    for (std::size_t index = 0; index < lhs.size(); ++index) {
        if (std::abs(lhs[index] - rhs[index]) > 1e-6f) {
            return false;
        }
    }
    return true;
}

bool merge_payload_hull(EvidencePayloadKind kind,
                        std::span<const float> lhs,
                        std::span<const float> rhs,
                        std::vector<float>& out) {
    if (lhs.size() != rhs.size()) {
        return false;
    }
    out.resize(lhs.size());
    if (kind == EvidencePayloadKind::EndpointEnvelope && (lhs.size() % 6) == 0) {
        for (std::size_t index = 0; index < lhs.size(); index += 6) {
            out[index + 0] = std::min(lhs[index + 0], rhs[index + 0]);
            out[index + 1] = std::min(lhs[index + 1], rhs[index + 1]);
            out[index + 2] = std::min(lhs[index + 2], rhs[index + 2]);
            out[index + 3] = std::max(lhs[index + 3], rhs[index + 3]);
            out[index + 4] = std::max(lhs[index + 4], rhs[index + 4]);
            out[index + 5] = std::max(lhs[index + 5], rhs[index + 5]);
        }
        return true;
    }
    for (std::size_t index = 0; index < out.size(); index += 2) {
        if (index + 1 < out.size()) {
            out[index] = std::min(lhs[index], rhs[index]);
            out[index + 1] = std::max(lhs[index + 1], rhs[index + 1]);
        } else {
            out[index] = std::min(lhs[index], rhs[index]);
        }
    }
    return true;
}

bool valid_tree_depth_limit(int max_tree_depth) {
    return max_tree_depth >= 1;
}

}  // namespace

struct LectDatabase::EvidenceMappedFile {
    EvidenceMappedFile() = default;
    EvidenceMappedFile(const EvidenceMappedFile&) = delete;
    EvidenceMappedFile& operator=(const EvidenceMappedFile&) = delete;
    EvidenceMappedFile(EvidenceMappedFile&&) noexcept = default;
    EvidenceMappedFile& operator=(EvidenceMappedFile&&) noexcept = default;
    ~EvidenceMappedFile() { close(); }

    bool open_read_only(const std::filesystem::path& path) {
        close();
#ifdef _WIN32
        file_ = ::CreateFileW(path.c_str(),
                              GENERIC_READ,
                              FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE,
                              nullptr,
                              OPEN_EXISTING,
                              FILE_ATTRIBUTE_NORMAL | FILE_FLAG_RANDOM_ACCESS,
                              nullptr);
        if (file_ == INVALID_HANDLE_VALUE) {
            file_ = INVALID_HANDLE_VALUE;
            return false;
        }
        LARGE_INTEGER size_value{};
        if (!::GetFileSizeEx(file_, &size_value) || size_value.QuadPart <= 0) {
            close();
            return false;
        }
        if (static_cast<unsigned long long>(size_value.QuadPart) >
            static_cast<unsigned long long>(std::numeric_limits<std::size_t>::max())) {
            close();
            return false;
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
        size_ = static_cast<std::size_t>(size_value.QuadPart);
        return true;
#else
        fd_ = ::open(path.c_str(), O_RDONLY | O_CLOEXEC);
        if (fd_ < 0) {
            fd_ = -1;
            return false;
        }
        struct stat status {};
        if (::fstat(fd_, &status) != 0 || status.st_size <= 0) {
            close();
            return false;
        }
        if (static_cast<unsigned long long>(status.st_size) >
            static_cast<unsigned long long>(std::numeric_limits<std::size_t>::max())) {
            close();
            return false;
        }
        void* mapped = ::mmap(nullptr, static_cast<std::size_t>(status.st_size), PROT_READ, MAP_PRIVATE, fd_, 0);
        if (mapped == MAP_FAILED) {
            close();
            return false;
        }
#ifdef MADV_RANDOM
        ::madvise(mapped, static_cast<std::size_t>(status.st_size), MADV_RANDOM);
#endif
        data_ = static_cast<const std::byte*>(mapped);
        size_ = static_cast<std::size_t>(status.st_size);
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
        if (data_ != nullptr) {
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

    bool is_open() const noexcept {
        return data_ != nullptr && size_ > 0;
    }

    std::span<const std::byte> bytes() const noexcept {
        return is_open() ? std::span<const std::byte>(data_, size_) : std::span<const std::byte>{};
    }

    void prefetch(std::uint64_t offset, std::uint64_t size) const {
        if (!is_open() || offset >= size_) {
            return;
        }
        const auto safe_size = std::min<std::uint64_t>(size, static_cast<std::uint64_t>(size_) - offset);
        if (safe_size == 0) {
            return;
        }
#ifdef _WIN32
        using PrefetchVirtualMemoryFn = BOOL(WINAPI*)(HANDLE, ULONG_PTR, PVOID, ULONG);
        struct MemoryRangeEntry {
            PVOID VirtualAddress;
            SIZE_T NumberOfBytes;
        };
        static const auto prefetch_virtual_memory = []() -> PrefetchVirtualMemoryFn {
            HMODULE module = ::GetModuleHandleW(L"kernel32.dll");
            if (module == nullptr) {
                return nullptr;
            }
            return reinterpret_cast<PrefetchVirtualMemoryFn>(::GetProcAddress(module, "PrefetchVirtualMemory"));
        }();
        if (prefetch_virtual_memory == nullptr) {
            return;
        }
        MemoryRangeEntry range;
        range.VirtualAddress = const_cast<std::byte*>(data_ + offset);
        range.NumberOfBytes = static_cast<SIZE_T>(safe_size);
        prefetch_virtual_memory(::GetCurrentProcess(), 1, &range, 0);
#else
#ifdef MADV_WILLNEED
    const long raw_page_size = ::sysconf(_SC_PAGESIZE);
    const auto page_size = raw_page_size > 0 ? static_cast<std::size_t>(raw_page_size) : 4096u;
        const auto aligned_offset = (static_cast<std::size_t>(offset) / page_size) * page_size;
        const auto aligned_end = ((static_cast<std::size_t>(offset + safe_size) + page_size - 1) / page_size) * page_size;
        const auto advised_size = std::min<std::size_t>(aligned_end - aligned_offset, size_ - aligned_offset);
        ::madvise(const_cast<std::byte*>(data_ + aligned_offset), advised_size, MADV_WILLNEED);
#else
        (void)offset;
        (void)safe_size;
#endif
#endif
    }

    std::size_t size() const noexcept { return size_; }

private:
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

LectDatabase::~LectDatabase() {
    close_evidence_streams();
    close_journal_append_stream();
}

LectDatabase::LectDatabase(LectDatabase&&) noexcept = default;

LectDatabase& LectDatabase::operator=(LectDatabase&&) noexcept = default;

std::optional<LectDatabase> LectDatabase::open_existing(const std::filesystem::path& path,
                                                        bool read_only,
                                                        std::string* reason) {
    LectDatabase database;
    LectDatabaseConfig config;
    config.path = path;
    config.open.read_only = read_only;
    config.open.create_if_missing = false;
    config.open.verify_identity = false;
    if (!database.open(std::move(config), reason)) {
        return std::nullopt;
    }
    return database;
}

bool LectDatabase::open(LectDatabaseConfig config, std::string* reason) {
    close_journal_append_stream();
    close_evidence_streams();
    config_ = std::move(config);
    opened_ = false;
    node_count_ = 0;
    max_node_id_ = 0;
    next_node_id_ = 0;
    node_ids_.clear();
    node_path_index_.clear();
    node_pages_.clear();
    node_page_clock_ = 0;
    layer_index_.clear();
    evidence_.clear();
    clear_evidence_index();
    evidence_mapped_file_.reset();
    evidence_mapping_stale_ = false;
    evidence_read_buffer_.clear();
    evidence_append_offset_ = 0;
    evidence_appends_since_flush_ = 0;
    generation_ = 0;
    deferred_parent_hull_writes_.clear();
    pending_changes_ = false;
    stats_ = {};

    if (config_.path.empty()) {
        if (reason) *reason = "database path is empty";
        return false;
    }
    if (!valid_tree_depth_limit(config_.max_tree_depth)) {
        if (reason) *reason = "max tree depth must be positive";
        return false;
    }
    std::error_code ignored;
    if (config_.open.create_if_missing) {
        std::filesystem::create_directories(config_.path, ignored);
    }

    const bool has_manifest = std::filesystem::is_regular_file(manifest_path(config_.path));
    LectDatabaseIdentity requested_identity = config_.identity;
    const bool has_requested_identity = requested_identity.robot_fingerprint != 0 ||
                                        requested_identity.root_domain_fingerprint != 0 ||
                                        requested_identity.split_policy_hash != 0;
    if (has_manifest) {
        if (!load_manifest(reason)) {
            return false;
        }
        identity_.schema_version = std::max(identity_.schema_version, kLectDatabaseSchemaVersion);
        if (config_.open.verify_identity && has_requested_identity) {
            std::string mismatch;
            if (!identity_compatible(identity_, requested_identity, &mismatch)) {
                if (reason) *reason = mismatch;
                return false;
            }
        }
        if (!load_nodes(reason)) {
            return false;
        }
        if (!load_evidence(reason)) {
            return false;
        }
        if (config_.open.replay_journal) {
            replay_journal();
        }
        rebuild_layer_index();
        assign_page_ids();
        opened_ = true;
        return true;
    }

    if (!config_.open.create_if_missing || config_.open.read_only) {
        if (reason) *reason = "database manifest does not exist";
        return false;
    }
    if (config_.root_intervals.empty()) {
        if (reason) *reason = "new database requires root intervals";
        return false;
    }

    root_intervals_ = config_.root_intervals;
    split_policy_ = SplitPolicy(config_.split_policy);
    identity_ = config_.identity;
    identity_.schema_version = std::max(identity_.schema_version, kLectDatabaseSchemaVersion);
    if (identity_.root_domain_fingerprint == 0) {
        identity_.root_domain_fingerprint = fingerprint_intervals(root_intervals_);
    }
    if (identity_.split_policy_hash == 0) {
        identity_.split_policy_hash = split_policy_.hash();
    }
    if (identity_.split_policy_descriptor.empty()) {
        identity_.split_policy_descriptor = ::rbf::lect_database::split_policy_descriptor(split_policy_.descriptor());
    }

    NodeRecord root;
    root.id = 0;
    root.depth = 0;
    write_node_record(std::move(root));
    rebuild_layer_index();
    assign_page_ids();
    opened_ = true;
    const bool saved = save_manifest() && save_nodes() && save_evidence();
    if (saved) {
        pending_changes_ = false;
    }
    return saved;
}

BoxKey LectDatabase::make_box_key(std::vector<Interval> intervals) const {
    BoxKey key;
    key.intervals = std::move(intervals);
    key.root_domain_fingerprint = identity_.root_domain_fingerprint;
    key.split_policy_hash = identity_.split_policy_hash;
    key.tolerance = config_.exact_box_tolerance;
    return key;
}

std::optional<NodeRecord> LectDatabase::node(NodeId node_id) const {
    return read_node(node_id);
}

NodeTopology LectDatabase::topology(NodeId node_id) const {
    NodeTopology out;
    const auto item = read_node(node_id);
    if (!item) {
        return out;
    }
    out.id = item->id;
    out.parent = item->parent;
    out.left = item->left;
    out.right = item->right;
    out.depth = item->depth;
    out.split_dim = item->split_dim;
    out.split_value = item->split_value;
    out.leaf = item->is_leaf();
    out.path = item->path;
    out.sibling = sibling(node_id);
    return out;
}

NodeId LectDatabase::parent(NodeId node_id) const {
    const auto item = read_node(node_id);
    return item ? item->parent : kInvalidNodeId;
}

std::pair<NodeId, NodeId> LectDatabase::children(NodeId node_id) const {
    const auto item = read_node(node_id);
    return item ? std::pair<NodeId, NodeId>{item->left, item->right}
                : std::pair<NodeId, NodeId>{kInvalidNodeId, kInvalidNodeId};
}

NodeId LectDatabase::sibling(NodeId node_id) const {
    const auto item = read_node(node_id);
    if (!item) {
        return kInvalidNodeId;
    }
    const NodeId parent_id = item->parent;
    const auto parent_node = read_node(parent_id);
    if (!parent_node) {
        return kInvalidNodeId;
    }
    return parent_node->left == node_id ? parent_node->right : parent_node->left;
}

bool LectDatabase::is_ancestor(NodeId ancestor, NodeId node_id) const {
    const auto a = read_node(ancestor);
    const auto n = read_node(node_id);
    if (!a || !n) {
        return false;
    }
    return a->path.is_prefix_of(n->path);
}

NodeId LectDatabase::lca(NodeId lhs, NodeId rhs) const {
    if (!has_node(lhs) || !has_node(rhs)) {
        return kInvalidNodeId;
    }
    NodeId left = lhs;
    NodeId right = rhs;
    auto left_record = read_node(left);
    auto right_record = read_node(right);
    while (left_record && right_record && left_record->depth > right_record->depth) {
        left = parent(left);
        left_record = read_node(left);
    }
    while (left_record && right_record && right_record->depth > left_record->depth) {
        right = parent(right);
        right_record = read_node(right);
    }
    while (left != right && has_node(left) && has_node(right)) {
        left = parent(left);
        right = parent(right);
    }
    return left == right ? left : kInvalidNodeId;
}

std::vector<NodeId> LectDatabase::node_ids() const {
    return sorted_node_ids();
}

std::vector<NodeId> LectDatabase::layer_nodes(int depth) const {
    const auto it = layer_index_.find(depth);
    return it == layer_index_.end() ? std::vector<NodeId>{} : it->second;
}

std::optional<std::vector<Interval>> LectDatabase::node_box(NodeId node_id) const {
    if (!has_node(node_id)) {
        return std::nullopt;
    }
    return node_box_unchecked(node_id);
}

BoxLookupResult LectDatabase::box_to_node_exact(const BoxKey& box) const {
    BoxLookupResult result;
    if (box.root_domain_fingerprint != identity_.root_domain_fingerprint) {
        result.reason = "root domain fingerprint differs";
        return result;
    }
    if (box.split_policy_hash != identity_.split_policy_hash) {
        result.reason = "split policy hash differs";
        return result;
    }
    if (box.intervals.size() != root_intervals_.size()) {
        result.reason = "dimension count differs";
        return result;
    }
    if (!box_contains(root_intervals_, box.intervals, box.tolerance)) {
        result.reason = "box is outside root domain";
        return result;
    }

    NodeId cursor = root_node();
    auto current_box = root_intervals_;
    while (has_node(cursor)) {
        if (intervals_equal(current_box, box.intervals, box.tolerance)) {
            result.found = true;
            result.node_id = cursor;
            return result;
        }
        const auto current = read_node(cursor);
        if (!current) {
            break;
        }
        if (current->is_leaf()) {
            result.reason = "tree has not split far enough for exact box";
            return result;
        }
        const int dim = current->split_dim;
        if (dim < 0 || dim >= static_cast<int>(box.intervals.size())) {
            result.reason = "stored split dimension is invalid";
            return result;
        }
        const auto& target = box.intervals[static_cast<std::size_t>(dim)];
        if (target.hi <= current->split_value + box.tolerance) {
            current_box[static_cast<std::size_t>(dim)].hi = current->split_value;
            cursor = current->left;
        } else if (target.lo >= current->split_value - box.tolerance) {
            current_box[static_cast<std::size_t>(dim)].lo = current->split_value;
            cursor = current->right;
        } else {
            std::ostringstream out;
            out << "box crosses split dimension " << dim << " at " << current->split_value;
            result.reason = out.str();
            return result;
        }
    }
    result.reason = "descended to invalid node id";
    return result;
}

std::vector<NodeId> LectDatabase::range_query(const std::vector<Interval>& box,
                                              RangeQueryMode mode,
                                              LectDatabaseStats* stats) const {
    std::vector<NodeId> out;
    if (box.size() != root_intervals_.size()) {
        return out;
    }
    const auto add_stats = [&]() {
        ++stats_.range_nodes_visited;
        if (stats != nullptr) {
            ++stats->range_nodes_visited;
        }
    };

    if (!valid_node_id(root_node())) {
        return out;
    }

    auto current_box = root_intervals_;
    const auto child_contributes = [&](const std::vector<Interval>& child_box) {
        return mode == RangeQueryMode::Containing
            ? box_contains(child_box, box, config_.exact_box_tolerance)
            : box_overlaps(child_box, box, config_.exact_box_tolerance);
    };

    const auto visit = [&](auto&& self, NodeId node_id) -> void {
        if (!has_node(node_id)) {
            return;
        }
        const auto node_record = read_node(node_id);
        if (!node_record) {
            return;
        }
        add_stats();
        bool keep = false;
        bool descend = false;
        switch (mode) {
        case RangeQueryMode::Containing:
            keep = box_contains(current_box, box, config_.exact_box_tolerance);
            descend = keep;
            break;
        case RangeQueryMode::ContainedBy:
            if (!box_overlaps(current_box, box, config_.exact_box_tolerance)) {
                return;
            }
            keep = box_contains(box, current_box, config_.exact_box_tolerance);
            descend = !node_record->is_leaf();
            break;
        case RangeQueryMode::Intersecting:
            keep = box_overlaps(current_box, box, config_.exact_box_tolerance);
            descend = keep;
            break;
        case RangeQueryMode::CoveringFrontier:
            if (!box_overlaps(current_box, box, config_.exact_box_tolerance)) {
                return;
            }
            keep = node_record->is_leaf();
            descend = !node_record->is_leaf();
            break;
        }
        if (keep) {
            out.push_back(node_record->id);
        }
        if (!descend || node_record->is_leaf()) {
            return;
        }
        const int dim = node_record->split_dim;
        if (dim < 0 || dim >= static_cast<int>(current_box.size())) {
            return;
        }

        if (valid_node_id(node_record->left)) {
            auto& left = current_box[static_cast<std::size_t>(dim)];
            const double saved_hi = left.hi;
            left.hi = node_record->split_value;
            if (child_contributes(current_box)) {
                self(self, node_record->left);
            }
            left.hi = saved_hi;
        }
        if (valid_node_id(node_record->right)) {
            auto& right = current_box[static_cast<std::size_t>(dim)];
            const double saved_lo = right.lo;
            right.lo = node_record->split_value;
            if (child_contributes(current_box)) {
                self(self, node_record->right);
            }
            right.lo = saved_lo;
        }
    };

    visit(visit, root_node());
    return out;
}

std::pair<NodeId, NodeId> LectDatabase::split_leaf(NodeId node_id) {
    const auto parent_record = read_node(node_id);
    if (config_.open.read_only || !parent_record) {
        return {kInvalidNodeId, kInvalidNodeId};
    }
    if (!parent_record->is_leaf()) {
        return {parent_record->left, parent_record->right};
    }
    const auto intervals = node_box_unchecked(node_id);
    const int parent_depth = parent_record->depth;
    const int split_dim = split_policy_.choose_dimension(root_intervals_, intervals, parent_depth);
    if (split_dim < 0 || split_dim >= static_cast<int>(intervals.size())) {
        return {kInvalidNodeId, kInvalidNodeId};
    }
    const double split_value = split_policy_.choose_split_value(intervals[static_cast<std::size_t>(split_dim)]);
    return split_leaf(node_id, split_dim, split_value);
}

std::pair<NodeId, NodeId> LectDatabase::split_leaf(NodeId node_id, int split_dim, double split_value) {
    const auto parent_record = read_node(node_id);
    if (config_.open.read_only || !parent_record) {
        return {kInvalidNodeId, kInvalidNodeId};
    }
    if (!parent_record->is_leaf()) {
        return {parent_record->left, parent_record->right};
    }
    const auto intervals = node_box_unchecked(node_id);
    const int parent_depth = parent_record->depth;
    const PathCode parent_path = parent_record->path;
    if (parent_depth + 1 >= config_.max_tree_depth) {
        return {kInvalidNodeId, kInvalidNodeId};
    }
    if (split_dim < 0 || split_dim >= static_cast<int>(intervals.size())) {
        return {kInvalidNodeId, kInvalidNodeId};
    }
    if (split_value <= intervals[static_cast<std::size_t>(split_dim)].lo ||
        split_value >= intervals[static_cast<std::size_t>(split_dim)].hi) {
        return {kInvalidNodeId, kInvalidNodeId};
    }

    ++generation_;
    pending_changes_ = true;
    PathCode left_path = parent_path;
    PathCode right_path = parent_path;
    left_path.push_child(false);
    right_path.push_child(true);

    const NodeId left = allocate_node_id();
    const NodeId right = allocate_node_id();
    if (!valid_node_id(left) || !valid_node_id(right) || left == right) {
        return {kInvalidNodeId, kInvalidNodeId};
    }

    LectDbTransaction transaction;
    transaction.generation = generation_;
    transaction.records.push_back("split|" + std::to_string(node_id) + "|" + std::to_string(left) + "|" +
                                  std::to_string(right) + "|" + std::to_string(split_dim) + "|" +
                                  to_text(split_value));
    transaction.committed = true;
    append_committed_transaction(transaction);

    NodeRecord left_record;
    left_record.id = left;
    left_record.parent = node_id;
    left_record.depth = parent_depth + 1;
    left_record.path = std::move(left_path);
    left_record.generation = generation_;
    left_record.dirty = true;
    write_node_record(std::move(left_record));

    NodeRecord right_record;
    right_record.id = right;
    right_record.parent = node_id;
    right_record.depth = parent_depth + 1;
    right_record.path = std::move(right_path);
    right_record.generation = generation_;
    right_record.dirty = true;
    write_node_record(std::move(right_record));

    if (auto* item = mutable_node(node_id)) {
        item->left = left;
        item->right = right;
        item->split_dim = split_dim;
        item->split_value = split_value;
        item->generation = generation_;
        item->dirty = true;
    }
    layer_index_[parent_depth + 1].push_back(left);
    layer_index_[parent_depth + 1].push_back(right);
    return {left, right};
}

bool LectDatabase::ensure_depth(int target_depth) {
    if (target_depth < 0) {
        return false;
    }
    if (target_depth >= config_.max_tree_depth) {
        return false;
    }
    bool changed = false;
    for (int depth = 0; depth < target_depth; ++depth) {
        const auto layer = layer_nodes(depth);
        for (NodeId id : layer) {
            const auto node_record = read_node(id);
            if (node_record && node_record->is_leaf()) {
                const auto children_pair = split_leaf(id);
                if (!valid_node_id(children_pair.first) || !valid_node_id(children_pair.second)) {
                    return false;
                }
                changed = true;
            }
        }
    }
    if (changed && !config_.open.read_only) {
        return flush_all_node_pages() && save_manifest();
    }
    return true;
}

BoxLookupResult LectDatabase::split_to_box(const BoxKey& box, int max_depth) {
    BoxLookupResult result;
    if (config_.open.read_only) {
        result.reason = "database is read-only";
        return result;
    }
    if (box.root_domain_fingerprint != identity_.root_domain_fingerprint ||
        box.split_policy_hash != identity_.split_policy_hash) {
        return box_to_node_exact(box);
    }
    if (!box_contains(root_intervals_, box.intervals, box.tolerance)) {
        result.reason = "box is outside root domain";
        return result;
    }

    NodeId cursor = root_node();
    auto current_box = root_intervals_;
    bool changed = false;
    while (has_node(cursor)) {
        if (intervals_equal(current_box, box.intervals, box.tolerance)) {
            if (changed && (!flush_all_node_pages() || !save_manifest())) {
                result.reason = "exact box was split but persistence failed";
                return result;
            }
            result.found = true;
            result.node_id = cursor;
            return result;
        }
        auto current = read_node(cursor);
        if (!current) {
            break;
        }
        const int effective_max_depth = std::min(max_depth, config_.max_tree_depth - 1);
        if (current->depth >= effective_max_depth) {
            result.reason = "max split depth reached before exact box";
            return result;
        }
        if (current->is_leaf()) {
            const auto children_pair = split_leaf(cursor);
            if (!valid_node_id(children_pair.first) || !valid_node_id(children_pair.second)) {
                result.reason = "leaf could not be split toward exact box";
                return result;
            }
            changed = true;
            current = read_node(cursor);
            if (!current) {
                break;
            }
        }
        const int dim = current->split_dim;
        if (dim < 0 || dim >= static_cast<int>(box.intervals.size())) {
            result.reason = "stored split dimension is invalid";
            return result;
        }
        const auto& target = box.intervals[static_cast<std::size_t>(dim)];
        if (target.hi <= current->split_value + box.tolerance) {
            current_box[static_cast<std::size_t>(dim)].hi = current->split_value;
            cursor = current->left;
        } else if (target.lo >= current->split_value - box.tolerance) {
            current_box[static_cast<std::size_t>(dim)].lo = current->split_value;
            cursor = current->right;
        } else {
            std::ostringstream out;
            out << "box is not representable: crosses split dimension " << dim << " at " << current->split_value;
            result.reason = out.str();
            return result;
        }
    }
    result.reason = "descended to invalid node id";
    return result;
}

bool LectDatabase::put_evidence(EvidenceRecord record) {
    if (config_.open.read_only) {
        return false;
    }
    if (!normalize_evidence_key(&record.key)) {
        return false;
    }
    const EvidenceKey key = record.key;
    const auto node_item = read_node(key.node_id);
    if (!node_item) {
        return false;
    }

    ++generation_;
    pending_changes_ = true;
    record.generation = generation_;
    record.checksum = payload_checksum(record.payload);
    const std::string journal_record = "evidence|" + serialize_evidence_record(record);
    const bool direct_evidence = !record.child_hull;
    const bool node_is_internal = !node_item->is_leaf();
    const NodeId parent_id = node_item->parent;

    auto stored_record = std::make_shared<EvidenceRecord>(std::move(record));
    auto [stored_it, stored_inserted] = evidence_.insert_or_assign(key, stored_record);
    (void)stored_inserted;
    if (!append_evidence_record_to_store(*stored_it->second)) {
        return false;
    }
    if (config_.propagate_parent_hulls) {
        if (config_.defer_parent_hull_writes) {
            deferred_parent_hull_writes_.push_back(DeferredParentHullWrite{key});
        } else {
            std::shared_ptr<const EvidenceRecord> propagated_child = stored_it->second;
            if (direct_evidence && node_is_internal) {
                if (auto child_hull = build_parent_hull_from_node(*node_item, key)) {
                    child_hull->generation = generation_;
                    child_hull->checksum = payload_checksum(child_hull->payload);
                    auto child_hull_record = std::make_shared<EvidenceRecord>(std::move(*child_hull));
                    auto [child_hull_it, child_hull_inserted] = evidence_.insert_or_assign(child_hull_record->key,
                                                                                           child_hull_record);
                    (void)child_hull_inserted;
                    if (!append_evidence_record_to_store(*child_hull_it->second)) {
                        return false;
                    }
                    propagated_child = child_hull_it->second;
                }
            }
            if (!propagate_parent_hulls_from(parent_id, key, propagated_child)) {
                return false;
            }
        }
    }

    LectDbTransaction transaction;
    transaction.generation = generation_;
    transaction.records.push_back(journal_record);
    transaction.committed = true;
    append_committed_transaction(transaction);
    ++stats_.evidence_writes;
    if (!maybe_flush_incremental_storage()) {
        return false;
    }
    trim_evidence_cache();
    return true;
}

std::optional<EvidenceRecordView> LectDatabase::evidence(const EvidenceKey& key) const {
    ++stats_.evidence_reads;
    EvidenceKey normalized_key = key;
    if (!normalize_evidence_key(&normalized_key)) {
        return std::nullopt;
    }
    auto it = evidence_.find(normalized_key);
    if (it == evidence_.end()) {
        auto loaded = load_indexed_evidence(normalized_key);
        if (!loaded) {
            return std::nullopt;
        }
        if (loaded->unavailable) {
            return std::nullopt;
        }
        trim_evidence_cache();
        return make_evidence_view(std::move(loaded));
    }
    if (it->second == nullptr || it->second->unavailable) {
        return std::nullopt;
    }
    auto cached_record = it->second;
    trim_evidence_cache();
    return make_evidence_view(cached_record);
}

std::optional<std::uint64_t> LectDatabase::evidence_offset(const EvidenceKey& key) const {
    const auto* index_entry = find_evidence_index(key);
    if (index_entry == nullptr || index_entry->unavailable) {
        return std::nullopt;
    }
    return index_entry->offset;
}

bool LectDatabase::has_evidence(const EvidenceKey& key) const {
    EvidenceKey normalized_key = key;
    if (!normalize_evidence_key(&normalized_key)) {
        return false;
    }
    const auto loaded = evidence_.find(normalized_key);
    if (loaded != evidence_.end()) {
        return loaded->second != nullptr && !loaded->second->unavailable;
    }
    const auto* indexed = find_evidence_index(normalized_key);
    return indexed != nullptr && !indexed->unavailable;
}

std::vector<EvidenceRecord> LectDatabase::evidence_records() const {
    std::vector<EvidenceRecord> records;
    records.reserve(evidence_count());
    for (const auto& slot : evidence_index_) {
        if (slot.key.node_id == kInvalidNodeId || slot.entry.unavailable) {
            continue;
        }
        if (auto record = evidence(slot.key)) {
            records.push_back(clone_evidence_record(*record));
        }
    }
    return records;
}

std::optional<EvidenceRecordView> LectDatabase::endpoint_for_box_exact(const BoxKey& box,
                                                                       EvidenceKey key_template) const {
    const auto lookup = box_to_node_exact(box);
    if (!lookup.found) {
        return std::nullopt;
    }
    key_template.node_id = lookup.node_id;
    key_template.payload_kind = EvidencePayloadKind::EndpointEnvelope;
    return evidence(key_template);
}

std::size_t LectDatabase::delete_node_payloads(NodeId node_id) {
    if (config_.open.read_only) {
        return 0;
    }
    std::vector<EvidenceKey> keys;
    for (const auto& slot : evidence_index_) {
        if (slot.key.node_id == kInvalidNodeId) {
            continue;
        }
        if (slot.key.node_id == node_id && !slot.entry.unavailable) {
            keys.push_back(slot.key);
        }
    }
    for (const auto& [key, record] : evidence_) {
        if (record != nullptr && key.node_id == node_id && !record->unavailable &&
            std::find(keys.begin(), keys.end(), key) == keys.end()) {
            keys.push_back(key);
        }
    }
    std::size_t erased = 0;
    for (const auto& key : keys) {
        EvidenceRecord tombstone;
        tombstone.key = key;
        tombstone.unavailable = true;
        tombstone.generation = ++generation_;
        tombstone.checksum = 0;
        if (!append_evidence_record_to_store(tombstone)) {
            break;
        }
        evidence_.erase(key);
        ++erased;
    }
    if (erased > 0) {
        pending_changes_ = true;
        maybe_flush_incremental_storage();
    }
    return erased;
}

LectDbQuerySession LectDatabase::make_query_session() const {
    return LectDbQuerySession(*this);
}

VerificationResult LectDatabase::verify(bool strict) const {
    VerificationResult result;
    std::unordered_map<EvidenceKey,
                       std::shared_ptr<const EvidenceRecord>,
                       EvidenceKeyHash>
        strict_records;
    std::unordered_map<NodeId, NodeRecord> strict_parent_nodes;
    if (strict) {
        strict_records.reserve(evidence_index_count_);
        strict_parent_nodes.reserve(std::min<std::size_t>(node_count_, evidence_index_count_));
    }
    if (!opened_) {
        result.add_error("database is not open");
        return result;
    }
    if (identity_.root_domain_fingerprint != fingerprint_intervals(root_intervals_)) {
        result.add_error("root domain fingerprint does not match stored root intervals");
    }
    if (identity_.split_policy_hash != split_policy_.hash()) {
        result.add_error("split policy hash does not match descriptor");
    }
    const auto root_record = read_node(root_node());
    if (node_count_ == 0 || !root_record || root_record->id != 0 || valid_node_id(root_record->parent)) {
        result.add_error("root node is missing or malformed");
    }
    for (NodeId node_id : sorted_node_ids()) {
        const auto item_opt = read_node(node_id);
        if (!item_opt) {
            result.add_error("node page is missing a row");
            continue;
        }
        const auto& item = *item_opt;
        if (item.id != node_id) {
            result.add_error("node id does not match table position");
        }
        if (valid_node_id(item.parent)) {
            const auto parent_node = read_node(item.parent);
            if (!parent_node) {
                result.add_error("node has invalid parent id");
            } else {
                if (parent_node->left != item.id && parent_node->right != item.id) {
                    result.add_error("parent does not reference child");
                }
            }
        }
        if (!item.is_leaf()) {
            const auto left_node = read_node(item.left);
            const auto right_node = read_node(item.right);
            if (!left_node || !right_node) {
                result.add_error("internal node has invalid child id");
            } else if (left_node->parent != item.id || right_node->parent != item.id) {
                result.add_error("child parent pointer mismatch");
            }
        }
        const auto key = make_box_key(node_box_unchecked(item.id));
        const auto lookup = box_to_node_exact(key);
        if (!lookup.found || lookup.node_id != item.id) {
            result.add_error("node box does not round-trip through exact box lookup");
        }
    }
    for (const auto& [depth, ids] : layer_index_) {
        for (NodeId id : ids) {
            const auto item = read_node(id);
            if (!item || item->depth != depth) {
                result.add_error("layer index contains an invalid node");
            }
        }
    }
    for (const auto& slot : evidence_index_) {
        if (slot.key.node_id == kInvalidNodeId || slot.entry.unavailable) {
            continue;
        }
        std::shared_ptr<const EvidenceRecord> record_owner;
        const auto loaded_it = evidence_.find(slot.key);
        if (loaded_it != evidence_.end()) {
            record_owner = loaded_it->second;
        } else {
            record_owner = load_indexed_evidence(slot.key);
        }
        if (record_owner == nullptr || record_owner->unavailable) {
            result.add_error("evidence row could not be materialized");
            continue;
        }
        if (strict) {
            strict_records.insert_or_assign(slot.key, record_owner);
        }
        const auto& record = *record_owner;
        if (record.checksum != payload_checksum(record.payload)) {
            result.add_error("evidence payload checksum mismatch");
        }
    }
    if (strict) {
        std::vector<float> expected_payload;
        for (const auto& [key, record_owner] : strict_records) {
            const auto& record = *record_owner;
            if (!record.child_hull) {
                continue;
            }

            NodeRecord parent_node;
            const auto parent_it = strict_parent_nodes.find(record.key.node_id);
            if (parent_it != strict_parent_nodes.end()) {
                parent_node = parent_it->second;
            } else {
                const auto parent_opt = read_node(record.key.node_id);
                if (!parent_opt) {
                    continue;
                }
                parent_node = *parent_opt;
                strict_parent_nodes.emplace(parent_node.id, parent_node);
            }
            if (parent_node.is_leaf()) {
                continue;
            }

            EvidenceKey left_key = key;
            EvidenceKey right_key = key;
            left_key.node_id = parent_node.left;
            right_key.node_id = parent_node.right;
            const auto left_it = strict_records.find(left_key);
            const auto right_it = strict_records.find(right_key);
            if (left_it == strict_records.end() || right_it == strict_records.end()) {
                continue;
            }
            if (left_it->second == nullptr || right_it->second == nullptr) {
                continue;
            }
            if (left_it->second->payload.size() != right_it->second->payload.size()) {
                continue;
            }

            expected_payload.clear();
            if (!merge_payload_hull(record.key.payload_kind,
                                    left_it->second->payload,
                                    right_it->second->payload,
                                    expected_payload)) {
                continue;
            }
            if (!payloads_equal(expected_payload, record.payload)) {
                result.add_error("parent hull evidence does not match child hull");
            }
        }
    }
    if (!strict && !result.ok) {
        result.add_warning("quick verification found errors; run strict verification for details");
    }
    return result;
}

bool LectDatabase::checkpoint() {
    if (config_.open.read_only || !opened_) {
        return false;
    }
    if (!drain_deferred_parent_hulls()) {
        return false;
    }
    if (!pending_changes_) {
        return true;
    }
    close_journal_append_stream();
    const bool ok_manifest = save_manifest();
    const bool ok_nodes = save_nodes();
    const bool ok_evidence = save_evidence();
    if (ok_manifest && ok_nodes && ok_evidence) {
        close_evidence_streams();
        std::ofstream clear(journal_path(config_.path), std::ios::trunc);
        const bool ok = static_cast<bool>(clear);
        if (ok) {
            pending_changes_ = false;
        }
        return ok;
    }
    return false;
}

bool LectDatabase::compact() {
    return checkpoint();
}

std::string LectDatabase::inspect_summary() const {
    std::ostringstream out;
    out << "lect_database\n"
        << "  path=" << config_.path.string() << "\n"
        << "  identity_hash=" << identity_hash(identity_) << "\n"
        << "  identity=" << identity_descriptor(identity_) << "\n"
        << "  root=" << interval_descriptor(root_intervals_) << "\n"
        << "  split_policy=" << ::rbf::lect_database::split_policy_descriptor(split_policy_.descriptor()) << "\n"
        << "  generation=" << generation_ << "\n"
        << "  nodes=" << node_count_ << "\n"
        << "  resident_node_pages=" << node_pages_.size() << "\n"
        << "  evidence=" << evidence_count() << "\n";
    return out.str();
}

NodeId LectDatabase::append_child(NodeId parent_id, bool right_child, int depth, PathCode path) {
    const NodeId child_id = allocate_node_id();
    if (!valid_node_id(child_id)) {
        return kInvalidNodeId;
    }
    NodeRecord child;
    child.id = child_id;
    child.parent = parent_id;
    child.depth = depth;
    child.path = std::move(path);
    child.generation = generation_;
    child.dirty = true;
    write_node_record(std::move(child));
    if (auto* parent_node = mutable_node(parent_id)) {
        if (right_child) {
            parent_node->right = child_id;
        } else {
            parent_node->left = child_id;
        }
        parent_node->dirty = true;
    }
    return child_id;
}

bool LectDatabase::has_node(NodeId node_id) const noexcept {
    return node_ids_.find(node_id) != node_ids_.end();
}

std::size_t LectDatabase::rows_per_node_page() const noexcept {
    constexpr std::size_t estimated_row_size = 160;
    return std::max<std::size_t>(1, static_cast<std::size_t>(config_.page_size_bytes) / estimated_row_size);
}

std::uint64_t LectDatabase::page_id_for_node(NodeId node_id) const noexcept {
    return static_cast<std::uint64_t>(node_id / rows_per_node_page());
}

NodeId LectDatabase::first_node_id_for_page(std::uint64_t page_id) const noexcept {
    return static_cast<NodeId>(page_id * rows_per_node_page());
}

std::size_t LectDatabase::node_offset_in_page(NodeId node_id) const noexcept {
    return static_cast<std::size_t>(node_id % rows_per_node_page());
}

LectDatabase::NodePage& LectDatabase::touch_node_page(std::uint64_t page_id) const {
    auto existing = node_pages_.find(page_id);
    if (existing != node_pages_.end()) {
        ++stats_.node_page_cache_hits;
        existing->second.last_access = ++node_page_clock_;
        update_resident_page_stats();
        return existing->second;
    }

    ++stats_.node_page_cache_misses;
    ++stats_.node_page_reads;
    NodePage page;
    page.page_id = page_id;
    page.first_node_id = first_node_id_for_page(page_id);
    page.last_access = ++node_page_clock_;
    const auto rows_per_page = rows_per_node_page();

    std::ifstream input(node_page_path(config_.path, page_id));
    std::string line;
    while (std::getline(input, line)) {
        if (line.empty()) {
            continue;
        }
        auto record = parse_node_record(line);
        if (!record || record->id < page.first_node_id) {
            continue;
        }
        const auto offset = static_cast<std::size_t>(record->id - page.first_node_id);
        if (offset >= rows_per_page) {
            continue;
        }
        if (offset >= page.rows.size()) {
            page.rows.resize(offset + 1);
        }
        page.rows[offset] = std::move(*record);
    }

    auto [inserted, _] = node_pages_.emplace(page_id, std::move(page));
    (void)_;
    evict_node_pages_if_needed();
    update_resident_page_stats();
    return node_pages_.at(inserted->first);
}

bool LectDatabase::flush_node_page(std::uint64_t page_id) const {
    auto it = node_pages_.find(page_id);
    if (it == node_pages_.end() || !it->second.dirty) {
        return true;
    }
    if (config_.open.read_only) {
        return false;
    }
    std::filesystem::create_directories(node_pages_path(config_.path));
    const auto path = node_page_path(config_.path, page_id);
    const auto tmp = path.string() + ".tmp";
    std::ofstream out(tmp, std::ios::trunc);
    if (!out) {
        return false;
    }
    for (auto& row : it->second.rows) {
        if (!valid_node_id(row.id) || !has_node(row.id)) {
            continue;
        }
        row.page_id = page_id;
        row.dirty = false;
        out << serialize_node_record(row) << '\n';
    }
    out.close();
    if (!static_cast<bool>(out) || !replace_file(tmp, path)) {
        return false;
    }
    it->second.dirty = false;
    ++stats_.node_page_writes;
    ++stats_.node_page_dirty_flushes;
    return true;
}

bool LectDatabase::flush_all_node_pages() const {
    std::vector<std::uint64_t> page_ids;
    page_ids.reserve(node_pages_.size());
    for (const auto& [page_id, page] : node_pages_) {
        (void)page;
        page_ids.push_back(page_id);
    }
    for (const auto page_id : page_ids) {
        if (!flush_node_page(page_id)) {
            return false;
        }
    }
    return true;
}

void LectDatabase::evict_node_pages_if_needed() const {
    const std::size_t limit = std::max<std::size_t>(1, config_.max_resident_pages);
    const std::size_t hot_page_count = std::min<std::size_t>(2, limit / 2);
    const auto is_hot_page = [&](std::uint64_t page_id) {
        return hot_page_count > 0 && page_id < hot_page_count;
    };
    while (node_pages_.size() > limit) {
        auto victim = node_pages_.end();
        for (auto it = node_pages_.begin(); it != node_pages_.end(); ++it) {
            if (is_hot_page(it->first)) {
                continue;
            }
            if (victim == node_pages_.end() || it->second.last_access < victim->second.last_access) {
                victim = it;
            }
        }
        if (victim == node_pages_.end()) {
            for (auto it = node_pages_.begin(); it != node_pages_.end(); ++it) {
                if (victim == node_pages_.end() || it->second.last_access < victim->second.last_access) {
                    victim = it;
                }
            }
        }
        if (victim == node_pages_.end()) {
            break;
        }
        const bool dirty = victim->second.dirty;
        if (dirty && !flush_node_page(victim->first)) {
            break;
        }
        if (dirty) {
            ++stats_.node_page_dirty_evictions;
        }
        ++stats_.node_page_evictions;
        node_pages_.erase(victim);
        update_resident_page_stats();
    }
}

void LectDatabase::update_resident_page_stats() const noexcept {
    stats_.resident_node_pages = static_cast<std::uint64_t>(node_pages_.size());
    stats_.max_resident_node_pages = std::max<std::uint64_t>(stats_.max_resident_node_pages,
                                                            stats_.resident_node_pages);
}

std::optional<NodeRecord> LectDatabase::read_node(NodeId node_id) const {
    if (!has_node(node_id)) {
        return std::nullopt;
    }
    const auto page_id = page_id_for_node(node_id);
    const auto offset = node_offset_in_page(node_id);
    auto& page = touch_node_page(page_id);
    if (offset >= page.rows.size() || page.rows[offset].id != node_id) {
        return std::nullopt;
    }
    return page.rows[offset];
}

NodeRecord* LectDatabase::mutable_node(NodeId node_id) {
    if (!has_node(node_id)) {
        return nullptr;
    }
    const auto page_id = page_id_for_node(node_id);
    const auto offset = node_offset_in_page(node_id);
    auto& page = touch_node_page(page_id);
    if (offset >= page.rows.size() || page.rows[offset].id != node_id) {
        return nullptr;
    }
    page.dirty = true;
    page.rows[offset].dirty = true;
    return &page.rows[offset];
}

bool LectDatabase::write_node_record(NodeRecord record) {
    if (!valid_node_id(record.id)) {
        return false;
    }
    if (!remember_node_record(record)) {
        return false;
    }
    record.page_id = page_id_for_node(record.id);
    const auto page_id = record.page_id;
    const auto offset = node_offset_in_page(record.id);
    auto& page = touch_node_page(page_id);
    if (offset >= page.rows.size()) {
        page.rows.resize(offset + 1);
    }
    record.dirty = true;
    page.rows[offset] = std::move(record);
    page.dirty = true;
    evict_node_pages_if_needed();
    return true;
}

bool LectDatabase::remember_node_id(NodeId node_id) {
    if (!valid_node_id(node_id)) {
        return false;
    }
    const auto inserted = node_ids_.insert(node_id).second;
    if (inserted) {
        ++node_count_;
    }
    max_node_id_ = std::max(max_node_id_, node_id);
    if (node_id < kInvalidNodeId - 1) {
        next_node_id_ = std::max(next_node_id_, node_id + 1);
    }
    return true;
}

bool LectDatabase::remember_node_record(const NodeRecord& record) {
    if (!remember_node_id(record.id)) {
        return false;
    }
    const auto existing = node_path_index_.find(record.path);
    if (existing != node_path_index_.end() && existing->second != record.id) {
        return false;
    }
    node_path_index_[record.path] = record.id;
    return true;
}

NodeId LectDatabase::allocate_node_id() {
    while (valid_node_id(next_node_id_) && has_node(next_node_id_)) {
        if (next_node_id_ == kInvalidNodeId - 1) {
            next_node_id_ = kInvalidNodeId;
            break;
        }
        ++next_node_id_;
    }
    if (!valid_node_id(next_node_id_)) {
        return kInvalidNodeId;
    }
    const NodeId allocated = next_node_id_;
    if (next_node_id_ == kInvalidNodeId - 1) {
        next_node_id_ = kInvalidNodeId;
    } else {
        ++next_node_id_;
    }
    return allocated;
}

bool LectDatabase::normalize_evidence_key(EvidenceKey* key) const {
    if (key == nullptr) {
        return false;
    }
    if (!key->node_path_valid) {
        if (!valid_node_id(key->node_id)) {
            return false;
        }
        const auto node_item = read_node(key->node_id);
        if (!node_item) {
            return false;
        }
        key->node_path = node_item->path;
        key->node_path_valid = true;
        return true;
    }
    if (!valid_node_id(key->node_id)) {
        const auto found = node_path_index_.find(key->node_path);
        if (found == node_path_index_.end()) {
            return false;
        }
        key->node_id = found->second;
    }
    return true;
}

EvidenceKey LectDatabase::evidence_key_for_node(NodeId node_id, const EvidenceKey* key_template) const {
    EvidenceKey key = key_template == nullptr ? EvidenceKey{} : *key_template;
    key.node_id = node_id;
    key.node_path_valid = false;
    key.node_path = {};
    normalize_evidence_key(&key);
    return key;
}

std::vector<NodeId> LectDatabase::sorted_node_ids() const {
    std::vector<NodeId> ids(node_ids_.begin(), node_ids_.end());
    std::sort(ids.begin(), ids.end());
    return ids;
}

std::vector<Interval> LectDatabase::node_box_unchecked(NodeId node_id) const {
    const auto node_record = read_node(node_id);
    if (!node_record) {
        return root_intervals_;
    }
    if (node_id == root_node()) {
        return root_intervals_;
    }

    std::vector<Interval> intervals = root_intervals_;
    std::vector<NodeId> lineage;
    lineage.reserve(static_cast<std::size_t>(std::max(0, node_record->depth)));

    NodeId cursor = node_id;
    while (valid_node_id(cursor) && cursor != root_node()) {
        const auto current = read_node(cursor);
        if (!current || !valid_node_id(current->parent)) {
            return node_box_from_path(node_record->path);
        }
        lineage.push_back(cursor);
        cursor = current->parent;
    }
    std::reverse(lineage.begin(), lineage.end());

    NodeId parent_id = root_node();
    for (NodeId child_id : lineage) {
        const auto parent = read_node(parent_id);
        if (!parent || parent->split_dim < 0 ||
            parent->split_dim >= static_cast<int>(intervals.size())) {
            return node_box_from_path(node_record->path);
        }
        if (child_id == parent->left) {
            intervals[static_cast<std::size_t>(parent->split_dim)].hi = parent->split_value;
        } else if (child_id == parent->right) {
            intervals[static_cast<std::size_t>(parent->split_dim)].lo = parent->split_value;
        } else {
            return node_box_from_path(node_record->path);
        }
        parent_id = child_id;
    }
    return intervals;
}

std::vector<Interval> LectDatabase::node_box_from_path(const PathCode& path) const {
    auto intervals = root_intervals_;
    for (int depth = 0; depth < path.bit_count; ++depth) {
        const int dim = split_policy_.choose_dimension(root_intervals_, intervals, depth);
        if (dim < 0 || dim >= static_cast<int>(intervals.size())) {
            return intervals;
        }
        const double split_value = split_policy_.choose_split_value(intervals[static_cast<std::size_t>(dim)]);
        if (path.bit(depth)) {
            intervals[static_cast<std::size_t>(dim)].lo = split_value;
        } else {
            intervals[static_cast<std::size_t>(dim)].hi = split_value;
        }
    }
    return intervals;
}

bool LectDatabase::intervals_equal(const std::vector<Interval>& lhs,
                                   const std::vector<Interval>& rhs,
                                   double tolerance) const {
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

bool LectDatabase::interval_contains(const Interval& outer, const Interval& inner, double tolerance) const {
    return outer.lo <= inner.lo + tolerance && outer.hi >= inner.hi - tolerance;
}

bool LectDatabase::box_contains(const std::vector<Interval>& outer,
                                const std::vector<Interval>& inner,
                                double tolerance) const {
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

bool LectDatabase::box_overlaps(const std::vector<Interval>& lhs,
                                const std::vector<Interval>& rhs,
                                double tolerance) const {
    if (lhs.size() != rhs.size()) {
        return false;
    }
    for (std::size_t dim = 0; dim < lhs.size(); ++dim) {
        if (!lhs[dim].overlaps(rhs[dim], tolerance)) {
            return false;
        }
    }
    return true;
}

void LectDatabase::rebuild_layer_index() {
    layer_index_.clear();
    for (NodeId node_id : sorted_node_ids()) {
        const auto node_record = read_node(node_id);
        if (node_record) {
            layer_index_[node_record->depth].push_back(node_record->id);
        }
    }
}

void LectDatabase::assign_page_ids() {
    for (NodeId node_id : sorted_node_ids()) {
        auto node_record = read_node(node_id);
        if (!node_record) {
            continue;
        }
        const auto expected_page_id = page_id_for_node(node_id);
        if (node_record->page_id != expected_page_id) {
            if (auto* item = mutable_node(node_id)) {
                item->page_id = expected_page_id;
                item->dirty = true;
            }
        }
    }
}

bool LectDatabase::load_manifest(std::string* reason) {
    const auto values = read_key_values(manifest_path(config_.path));
    if (values.empty()) {
        if (reason) *reason = "manifest is empty or unreadable";
        return false;
    }
    const std::string node_id_scheme = get_value(values, "node_id_scheme");
    if (node_id_scheme != kLegacyNodeIdScheme && node_id_scheme != kCurrentNodeIdScheme) {
        if (reason) *reason = "node id scheme is missing or unsupported; rebuild the database";
        return false;
    }
    identity_.schema_version = static_cast<std::uint32_t>(get_u64(values, "schema_version", kLectDatabaseSchemaVersion));
    identity_.robot_fingerprint = get_u64(values, "robot_fingerprint");
    identity_.root_domain_fingerprint = get_u64(values, "root_domain_fingerprint");
    identity_.split_policy_hash = get_u64(values, "split_policy_hash");
    identity_.symmetry_hash = get_u64(values, "symmetry_hash");
    identity_.canonical_mode = get_int(values, "canonical_mode") != 0;
    identity_.symmetry_descriptor = get_value(values, "symmetry_descriptor");
    identity_.split_policy_descriptor = get_value(values, "split_policy_descriptor");
    identity_.endpoint_descriptor = get_value(values, "endpoint_descriptor", identity_.endpoint_descriptor);
    identity_.envelope_descriptor = get_value(values, "envelope_descriptor", identity_.envelope_descriptor);
    identity_.payload_layout = get_value(values, "payload_layout", identity_.payload_layout);
    identity_.builder_version = get_value(values, "builder_version");

    SplitPolicyDescriptor descriptor;
    descriptor.strategy = static_cast<SplitStrategy>(get_int(values, "split_strategy"));
    descriptor.min_width = get_double(values, "split_min_width");
    descriptor.midpoint = get_int(values, "split_midpoint", 1) != 0;
    descriptor.deterministic_tie_break = get_int(values, "split_deterministic_tie_break", 1) != 0;
    descriptor.dimension_schedule_hash = get_value(values, "split_dimension_schedule_hash");
    descriptor.depth_dimensions = parse_depth_dimensions(get_value(values, "split_depth_dimensions"));
    split_policy_ = SplitPolicy(descriptor);
    generation_ = get_u64(values, "generation");
    config_.page_size_bytes = static_cast<std::uint32_t>(get_u64(values, "page_size_bytes", config_.page_size_bytes));
    config_.max_resident_pages = static_cast<std::uint32_t>(get_u64(values, "max_resident_pages", config_.max_resident_pages));
    config_.max_tree_depth = get_int(values, "max_tree_depth", config_.max_tree_depth);
    if (!valid_tree_depth_limit(config_.max_tree_depth)) {
        if (reason) *reason = "manifest max_tree_depth must be positive";
        return false;
    }
    node_count_ = static_cast<NodeId>(get_u64(values, "node_count", node_count_));
    max_node_id_ = static_cast<NodeId>(get_u64(values, "max_node_id", max_node_id_));

    const int dims = get_int(values, "root_dims");
    root_intervals_.clear();
    root_intervals_.reserve(static_cast<std::size_t>(std::max(0, dims)));
    for (int dim = 0; dim < dims; ++dim) {
        root_intervals_.push_back({get_double(values, "root_" + std::to_string(dim) + "_lo"),
                                   get_double(values, "root_" + std::to_string(dim) + "_hi")});
    }
    config_.root_intervals = root_intervals_;
    config_.split_policy = descriptor;
    return true;
}

bool LectDatabase::save_manifest() const {
    std::filesystem::create_directories(config_.path);
    const auto path = manifest_path(config_.path);
    const auto tmp = path.string() + ".tmp";
    std::ofstream out(tmp, std::ios::trunc);
    if (!out) {
        return false;
    }
    const auto& descriptor = split_policy_.descriptor();
    out << "schema_version=" << identity_.schema_version << '\n'
        << "robot_fingerprint=" << identity_.robot_fingerprint << '\n'
        << "root_domain_fingerprint=" << identity_.root_domain_fingerprint << '\n'
        << "split_policy_hash=" << identity_.split_policy_hash << '\n'
        << "symmetry_hash=" << identity_.symmetry_hash << '\n'
        << "canonical_mode=" << (identity_.canonical_mode ? 1 : 0) << '\n'
        << "symmetry_descriptor=" << identity_.symmetry_descriptor << '\n'
        << "split_policy_descriptor=" << identity_.split_policy_descriptor << '\n'
        << "endpoint_descriptor=" << identity_.endpoint_descriptor << '\n'
        << "envelope_descriptor=" << identity_.envelope_descriptor << '\n'
        << "payload_layout=" << identity_.payload_layout << '\n'
        << "builder_version=" << identity_.builder_version << '\n'
        << "node_id_scheme=" << kCurrentNodeIdScheme << '\n'
        << "split_strategy=" << static_cast<int>(descriptor.strategy) << '\n'
        << "split_min_width=" << std::setprecision(17) << descriptor.min_width << '\n'
        << "split_midpoint=" << (descriptor.midpoint ? 1 : 0) << '\n'
        << "split_deterministic_tie_break=" << (descriptor.deterministic_tie_break ? 1 : 0) << '\n'
        << "split_dimension_schedule_hash=" << descriptor.dimension_schedule_hash << '\n'
        << "split_depth_dimensions=" << serialize_depth_dimensions(descriptor.depth_dimensions) << '\n'
        << "root_dims=" << root_intervals_.size() << '\n'
        << "page_size_bytes=" << config_.page_size_bytes << '\n'
        << "max_resident_pages=" << config_.max_resident_pages << '\n'
        << "max_tree_depth=" << config_.max_tree_depth << '\n'
        << "node_count=" << node_count_ << '\n'
        << "max_node_id=" << max_node_id_ << '\n'
        << "generation=" << generation_ << '\n';
    for (std::size_t dim = 0; dim < root_intervals_.size(); ++dim) {
        out << "root_" << dim << "_lo=" << std::setprecision(17) << root_intervals_[dim].lo << '\n'
            << "root_" << dim << "_hi=" << std::setprecision(17) << root_intervals_[dim].hi << '\n';
    }
    out.close();
    return static_cast<bool>(out) && replace_file(tmp, path);
}

bool LectDatabase::load_nodes(std::string* reason) {
    node_pages_.clear();
    node_page_clock_ = 0;
    node_ids_.clear();
    node_path_index_.clear();
    node_count_ = 0;
    max_node_id_ = 0;
    next_node_id_ = 0;
    const bool has_page_dir = std::filesystem::is_directory(node_pages_path(config_.path));
    if (has_page_dir) {
        for (const auto& entry : std::filesystem::directory_iterator(node_pages_path(config_.path))) {
            if (!entry.is_regular_file()) {
                continue;
            }
            std::ifstream page_input(entry.path());
            std::string line;
            while (std::getline(page_input, line)) {
                if (line.empty()) {
                    continue;
                }
                auto record = parse_node_record(line);
                if (record && valid_node_id(record->id)) {
                    if (!remember_node_record(*record)) {
                        if (reason) *reason = "node path index is malformed";
                        return false;
                    }
                }
            }
        }
        update_resident_page_stats();
        return true;
    }

    std::ifstream input(nodes_path(config_.path));
    if (!input) {
        if (reason) *reason = "nodes.pages is missing";
        return false;
    }
    std::string line;
    while (std::getline(input, line)) {
        if (line.empty()) {
            continue;
        }
        auto record = parse_node_record(line);
        if (!record) {
            if (reason) *reason = "node row is malformed";
            return false;
        }
        write_node_record(std::move(*record));
    }
    if (!config_.open.read_only) {
        flush_all_node_pages();
    }
    return true;
}

bool LectDatabase::save_nodes() const {
    if (!flush_all_node_pages()) {
        return false;
    }
    const auto path = nodes_path(config_.path);
    const auto tmp = path.string() + ".tmp";
    std::ofstream out(tmp, std::ios::trunc);
    if (!out) {
        return false;
    }
    for (NodeId node_id : sorted_node_ids()) {
        const auto record = read_node(node_id);
        if (!record) {
            return false;
        }
        out << serialize_node_record(*record) << '\n';
    }
    out.close();
    return static_cast<bool>(out) && replace_file(tmp, path);
}

bool LectDatabase::load_evidence(std::string* reason) {
    evidence_.clear();
    clear_evidence_index();
    close_evidence_streams();
    evidence_append_offset_ = 0;
    evidence_appends_since_flush_ = 0;
    evidence_index_sidecar_dirty_ = false;
    evidence_store_format_ = EvidenceStoreFormat::Binary;
    const auto path = evidence_path(config_.path);
    std::error_code error;
    const bool evidence_exists = std::filesystem::exists(path, error);
    if (error || !evidence_exists) {
        return true;
    }
    const std::uint64_t evidence_file_size = std::filesystem::file_size(path, error);
    if (error) {
        if (reason) *reason = "failed to size evidence file";
        return false;
    }

    std::ifstream input(path, std::ios::binary);
    if (!input) {
        if (reason) *reason = "failed to open evidence file";
        return false;
    }
    EvidenceStoreFileHeader store_header;
    const bool binary_store = read_evidence_store_file_header(input, &store_header);
    evidence_store_format_ = binary_store ? EvidenceStoreFormat::Binary : EvidenceStoreFormat::LegacyText;
    if (binary_store) {
        evidence_append_offset_ = evidence_file_size;
    }

    if (binary_store && load_evidence_index_sidecar(evidence_file_size)) {
        return true;
    }

    if (binary_store) {
        if (!scan_binary_evidence_store(input, evidence_file_size, reason)) {
            return false;
        }
        evidence_index_sidecar_dirty_ = true;
        if (!config_.open.read_only && !save_evidence_index_sidecar(evidence_append_offset_)) {
            evidence_index_sidecar_dirty_ = true;
        }
        prefetch_indexed_evidence_ranges();
        return true;
    }

    std::vector<EvidenceRecord> legacy_records;
    if (!scan_legacy_text_evidence_store(input,
                                         config_.open.read_only ? nullptr : &legacy_records,
                                         reason)) {
        return false;
    }
    if (config_.open.read_only) {
        evidence_append_offset_ = evidence_file_size;
        return true;
    }
    input.close();
    input.clear();
    if (!rewrite_evidence_store_binary(legacy_records, reason)) {
        return false;
    }
    if (!save_evidence_index_sidecar(evidence_append_offset_)) {
        evidence_index_sidecar_dirty_ = true;
    }
    return true;
}

bool LectDatabase::save_evidence() const {
    const auto path = evidence_path(config_.path);
    std::filesystem::create_directories(config_.path);
    if (evidence_append_stream_.is_open()) {
        evidence_append_stream_.flush();
        if (!evidence_append_stream_) {
            return false;
        }
    }
    if (!ensure_binary_evidence_store_file()) {
        return false;
    }
    if (evidence_index_sidecar_dirty_ && !save_evidence_index_sidecar(evidence_append_offset_)) {
        return false;
    }
    return true;
}

void LectDatabase::clear_evidence_index() noexcept {
    evidence_index_.clear();
    evidence_index_count_ = 0;
}

void LectDatabase::reserve_evidence_index(std::size_t item_count) {
    const auto min_slots = evidence_index_slot_count(item_count);
    if (evidence_index_.size() >= min_slots) {
        return;
    }

    std::vector<EvidenceIndexRecord> grown(min_slots);
    for (auto& slot : grown) {
        slot.key.node_id = kInvalidNodeId;
    }

    const auto mask = min_slots - 1;
    for (const auto& slot : evidence_index_) {
        if (slot.key.node_id == kInvalidNodeId) {
            continue;
        }
        std::size_t probe = static_cast<std::size_t>(EvidenceKeyHash{}(slot.key)) & mask;
        while (grown[probe].key.node_id != kInvalidNodeId) {
            probe = (probe + 1) & mask;
        }
        grown[probe] = slot;
    }
    evidence_index_ = std::move(grown);
}

const LectDatabase::EvidenceIndexEntry* LectDatabase::find_evidence_index(const EvidenceKey& key) const {
    if (evidence_index_count_ == 0 || evidence_index_.empty()) {
        return nullptr;
    }
    EvidenceKey normalized_key = key;
    if (!normalize_evidence_key(&normalized_key)) {
        return nullptr;
    }
    const auto mask = evidence_index_.size() - 1;
    std::size_t probe = static_cast<std::size_t>(EvidenceKeyHash{}(normalized_key)) & mask;
    while (true) {
        const auto& slot = evidence_index_[probe];
        if (slot.key.node_id == kInvalidNodeId) {
            return nullptr;
        }
        if (slot.key == normalized_key) {
            return &slot.entry;
        }
        probe = (probe + 1) & mask;
    }
}

LectDatabase::EvidenceIndexEntry* LectDatabase::find_evidence_index(const EvidenceKey& key) {
    if (evidence_index_count_ == 0 || evidence_index_.empty()) {
        return nullptr;
    }
    EvidenceKey normalized_key = key;
    if (!normalize_evidence_key(&normalized_key)) {
        return nullptr;
    }
    const auto mask = evidence_index_.size() - 1;
    std::size_t probe = static_cast<std::size_t>(EvidenceKeyHash{}(normalized_key)) & mask;
    while (true) {
        auto& slot = evidence_index_[probe];
        if (slot.key.node_id == kInvalidNodeId) {
            return nullptr;
        }
        if (slot.key == normalized_key) {
            return &slot.entry;
        }
        probe = (probe + 1) & mask;
    }
}

void LectDatabase::upsert_evidence_index(const EvidenceKey& key, EvidenceIndexEntry entry) {
    EvidenceKey normalized_key = key;
    if (!normalize_evidence_key(&normalized_key)) {
        normalized_key = key;
    }
    if (evidence_index_.empty() ||
        evidence_index_slot_count(evidence_index_count_ + 1) > evidence_index_.size()) {
        reserve_evidence_index(evidence_index_count_ + 1);
    }

    const auto mask = evidence_index_.size() - 1;
    std::size_t probe = static_cast<std::size_t>(EvidenceKeyHash{}(normalized_key)) & mask;
    while (true) {
        auto& slot = evidence_index_[probe];
        if (slot.key.node_id == kInvalidNodeId) {
            slot.key = normalized_key;
            slot.entry = entry;
            ++evidence_index_count_;
            return;
        }
        if (slot.key == normalized_key) {
            slot.entry = entry;
            return;
        }
        probe = (probe + 1) & mask;
    }
}

bool LectDatabase::ensure_evidence_mapped_file() const {
    if (evidence_append_offset_ == 0 || config_.path.empty()) {
        return false;
    }
    if (evidence_append_stream_.is_open()) {
        evidence_append_stream_.flush();
        if (!evidence_append_stream_) {
            return false;
        }
    }
    if (evidence_mapped_file_ != nullptr && evidence_mapped_file_->is_open() && !evidence_mapping_stale_ &&
        evidence_mapped_file_->size() >= static_cast<std::size_t>(evidence_append_offset_)) {
        return true;
    }

    if (evidence_mapped_file_ == nullptr) {
        evidence_mapped_file_ = std::make_shared<EvidenceMappedFile>();
    }
    if (!evidence_mapped_file_->open_read_only(evidence_path(config_.path))) {
        evidence_mapped_file_.reset();
        return false;
    }
    evidence_mapping_stale_ = false;
    return evidence_mapped_file_->size() >= static_cast<std::size_t>(evidence_append_offset_);
}

std::optional<std::span<const std::byte>> LectDatabase::load_evidence_bytes(std::uint64_t offset,
                                                                            std::uint32_t size) const {
    if (size == 0) {
        return std::span<const std::byte>{};
    }
    const auto end = offset + static_cast<std::uint64_t>(size);
    if (end > evidence_append_offset_) {
        return std::nullopt;
    }

    if (ensure_evidence_mapped_file()) {
        const auto mapped = evidence_mapped_file_->bytes();
        if (end <= mapped.size()) {
            return mapped.subspan(static_cast<std::size_t>(offset), size);
        }
    }

    evidence_read_buffer_.resize(size);
    std::ifstream input(evidence_path(config_.path), std::ios::binary);
    if (!input) {
        evidence_read_buffer_.clear();
        return std::nullopt;
    }
    input.seekg(static_cast<std::streamoff>(offset), std::ios::beg);
    if (!input) {
        evidence_read_buffer_.clear();
        return std::nullopt;
    }
    input.read(reinterpret_cast<char*>(evidence_read_buffer_.data()), static_cast<std::streamsize>(size));
    if (!input) {
        evidence_read_buffer_.clear();
        return std::nullopt;
    }
    return std::span<const std::byte>(evidence_read_buffer_.data(), evidence_read_buffer_.size());
}

void LectDatabase::prefetch_indexed_evidence_ranges() const {
    if (evidence_store_format_ != EvidenceStoreFormat::Binary || evidence_index_count_ == 0) {
        return;
    }
    if (!ensure_evidence_mapped_file()) {
        return;
    }

    struct EvidenceRange {
        std::uint64_t offset = 0;
        std::uint32_t size = 0;
    };

    std::vector<EvidenceRange> ranges;
    ranges.reserve(evidence_index_count_);
    for (const auto& slot : evidence_index_) {
        if (slot.key.node_id == kInvalidNodeId || slot.entry.unavailable || slot.entry.size == 0) {
            continue;
        }
        ranges.push_back({slot.entry.offset, slot.entry.size});
    }
    if (ranges.empty()) {
        return;
    }

    std::sort(ranges.begin(), ranges.end(), [](const EvidenceRange& lhs, const EvidenceRange& rhs) {
        return lhs.offset < rhs.offset;
    });

    const auto merge_gap = std::max<std::uint64_t>(4096u, config_.page_size_bytes);
    std::uint64_t range_begin = ranges.front().offset;
    std::uint64_t range_end = range_begin + ranges.front().size;
    for (std::size_t index = 1; index < ranges.size(); ++index) {
        const auto entry_begin = ranges[index].offset;
        const auto entry_end = entry_begin + ranges[index].size;
        if (entry_begin <= range_end + merge_gap) {
            range_end = std::max(range_end, entry_end);
            continue;
        }
        evidence_mapped_file_->prefetch(range_begin, range_end - range_begin);
        range_begin = entry_begin;
        range_end = entry_end;
    }
    evidence_mapped_file_->prefetch(range_begin, range_end - range_begin);
}

bool LectDatabase::load_evidence_index_sidecar(std::uint64_t evidence_file_size) {
    const auto path = evidence_index_path(config_.path);
    EvidenceMappedFile mapped_sidecar;
    if (!mapped_sidecar.open_read_only(path)) {
        return false;
    }
    const auto bytes = mapped_sidecar.bytes();
    if (bytes.size() < sizeof(EvidenceIndexSidecarHeader)) {
        return false;
    }

    EvidenceIndexSidecarHeader header;
    std::memcpy(&header, bytes.data(), sizeof(header));
    if (header.magic != kEvidenceIndexSidecarMagic ||
        header.version != kEvidenceIndexSidecarSchemaVersion ||
        header.evidence_file_size != evidence_file_size) {
        return false;
    }
    const auto expected_count = static_cast<std::size_t>(header.entry_count);
    const auto entries_bytes = expected_count * sizeof(EvidenceIndexSidecarEntry);
    const auto required_size = sizeof(EvidenceIndexSidecarHeader) + entries_bytes;
    if (entries_bytes / sizeof(EvidenceIndexSidecarEntry) != expected_count || bytes.size() != required_size) {
        return false;
    }
    const auto* raw_entry_data = reinterpret_cast<const EvidenceIndexSidecarEntry*>(
        bytes.data() + sizeof(EvidenceIndexSidecarHeader));
    const std::span<const EvidenceIndexSidecarEntry> raw_entries(raw_entry_data, expected_count);
    if (!evidence_sidecar_offsets_sorted(raw_entries)) {
        return false;
    }

    clear_evidence_index();
    reserve_evidence_index(expected_count);
    for (const auto& raw_entry : raw_entries) {
        EvidenceKey key;
        key.node_id = raw_entry.node_id;
        key.sector = raw_entry.sector;
        key.channel = static_cast<EvidenceChannel>(raw_entry.channel);
        key.endpoint_source = static_cast<EndpointSource>(raw_entry.endpoint_source);
        key.payload_kind = static_cast<EvidencePayloadKind>(raw_entry.payload_kind);
        if (!normalize_evidence_key(&key)) {
            return false;
        }
        EvidenceIndexEntry entry;
        entry.offset = raw_entry.offset;
        entry.size = raw_entry.size;
        entry.child_hull = (raw_entry.flags & kEvidenceIndexFlagChildHull) != 0;
        entry.unavailable = (raw_entry.flags & kEvidenceIndexFlagUnavailable) != 0;
        entry.generation = raw_entry.generation;
        entry.checksum = raw_entry.checksum;
        upsert_evidence_index(key, entry);
    }
    if (evidence_index_count_ != expected_count) {
        return false;
    }
    if (ensure_evidence_mapped_file()) {
        const auto merge_gap = std::max<std::uint64_t>(4096u, config_.page_size_bytes);
        bool have_range = false;
        std::uint64_t range_begin = 0;
        std::uint64_t range_end = 0;
        for (const auto& raw_entry : raw_entries) {
            if ((raw_entry.flags & kEvidenceIndexFlagUnavailable) != 0 || raw_entry.size == 0) {
                continue;
            }
            const auto entry_begin = raw_entry.offset;
            const auto entry_end = entry_begin + raw_entry.size;
            if (!have_range) {
                range_begin = entry_begin;
                range_end = entry_end;
                have_range = true;
                continue;
            }
            if (entry_begin <= range_end + merge_gap) {
                range_end = std::max(range_end, entry_end);
                continue;
            }
            evidence_mapped_file_->prefetch(range_begin, range_end - range_begin);
            range_begin = entry_begin;
            range_end = entry_end;
        }
        if (have_range) {
            evidence_mapped_file_->prefetch(range_begin, range_end - range_begin);
        }
    }
    evidence_index_sidecar_dirty_ = false;
    return true;
}

bool LectDatabase::scan_binary_evidence_store(std::ifstream& input,
                                              std::uint64_t evidence_file_size,
                                              std::string* reason) {
    EvidenceStoreFileHeader header;
    if (!read_evidence_store_file_header(input, &header)) {
        if (reason) *reason = "evidence store header is malformed";
        return false;
    }

    clear_evidence_index();
    std::uint64_t offset = sizeof(EvidenceStoreFileHeader);
    while (offset < evidence_file_size) {
        EvidenceStoreRecordHeader record_header;
        input.read(reinterpret_cast<char*>(&record_header), static_cast<std::streamsize>(sizeof(record_header)));
        if (!input) {
            if (reason) *reason = "evidence store record header is truncated";
            return false;
        }

        const auto payload_bytes = static_cast<std::uint64_t>(record_header.payload_count) * sizeof(float);
        const auto expected_record_size = sizeof(EvidenceStoreRecordHeader) + payload_bytes;
        if (record_header.record_size != expected_record_size ||
            record_header.record_size < sizeof(EvidenceStoreRecordHeader) ||
            offset + record_header.record_size > evidence_file_size) {
            if (reason) *reason = "evidence store record is malformed";
            return false;
        }

        EvidenceKey key;
        key.node_id = record_header.node_id;
        key.sector = record_header.sector;
        key.channel = static_cast<EvidenceChannel>(record_header.channel);
        key.endpoint_source = static_cast<EndpointSource>(record_header.endpoint_source);
        key.payload_kind = static_cast<EvidencePayloadKind>(record_header.payload_kind);
        if (!normalize_evidence_key(&key)) {
            if (reason) *reason = "evidence store references an unknown node handle";
            return false;
        }

        EvidenceIndexEntry entry;
        entry.offset = offset;
        entry.size = record_header.record_size;
        entry.child_hull = (record_header.flags & kEvidenceIndexFlagChildHull) != 0;
        entry.unavailable = (record_header.flags & kEvidenceIndexFlagUnavailable) != 0;
        entry.generation = record_header.generation;
        entry.checksum = record_header.checksum;
        upsert_evidence_index(key, entry);

        if (payload_bytes > 0) {
            input.seekg(static_cast<std::streamoff>(payload_bytes), std::ios::cur);
            if (!input) {
                if (reason) *reason = "evidence store payload is truncated";
                return false;
            }
        }
        offset += record_header.record_size;
    }
    if (offset != evidence_file_size) {
        if (reason) *reason = "evidence store has trailing bytes";
        return false;
    }
    evidence_store_format_ = EvidenceStoreFormat::Binary;
    return true;
}

bool LectDatabase::scan_legacy_text_evidence_store(std::ifstream& input,
                                                   std::vector<EvidenceRecord>* records,
                                                   std::string* reason) {
    input.clear();
    input.seekg(0, std::ios::beg);
    clear_evidence_index();

    std::string line_storage;
    while (true) {
        const auto raw_offset = input.tellg();
        if (!std::getline(input, line_storage)) {
            break;
        }
        const auto offset = raw_offset == std::streampos(-1)
            ? 0ull
            : static_cast<std::uint64_t>(raw_offset);
        std::string_view line = trim_line_ending(line_storage);
        if (line.empty()) {
            continue;
        }
        auto record = parse_evidence_record(line);
        if (!record) {
            if (reason) *reason = "legacy evidence row is malformed";
            return false;
        }
        if (!normalize_evidence_key(&record->key)) {
            if (reason) *reason = "legacy evidence row references an unknown node handle";
            return false;
        }
        if (line_storage.size() > std::numeric_limits<std::uint32_t>::max()) {
            if (reason) *reason = "legacy evidence row is too large";
            return false;
        }
        EvidenceIndexEntry entry;
        entry.offset = offset;
        entry.size = static_cast<std::uint32_t>(line_storage.size());
        entry.child_hull = record->child_hull;
        entry.unavailable = record->unavailable;
        entry.generation = record->generation;
        entry.checksum = record->checksum;
        upsert_evidence_index(record->key, entry);
        if (records != nullptr) {
            records->push_back(std::move(*record));
        }
    }
    input.clear();
    evidence_store_format_ = EvidenceStoreFormat::LegacyText;
    return true;
}

bool LectDatabase::rewrite_evidence_store_binary(const std::vector<EvidenceRecord>& records,
                                                 std::string* reason) {
    close_evidence_streams();
    std::filesystem::create_directories(config_.path);
    const auto path = evidence_path(config_.path);
    const auto tmp = path.string() + ".tmp";
    std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
    if (!out) {
        if (reason) *reason = "failed to rewrite binary evidence store";
        return false;
    }
    if (!write_evidence_store_file_header(out)) {
        if (reason) *reason = "failed to write evidence store header";
        return false;
    }

    clear_evidence_index();
    reserve_evidence_index(records.size());
    std::uint64_t offset = sizeof(EvidenceStoreFileHeader);
    for (const auto& record : records) {
        const auto header = make_evidence_store_record_header(record);
        if (header.record_size == 0) {
            if (reason) *reason = "evidence payload is too large to persist";
            return false;
        }
        out.write(reinterpret_cast<const char*>(&header), static_cast<std::streamsize>(sizeof(header)));
        if (!record.payload.empty()) {
            out.write(reinterpret_cast<const char*>(record.payload.data()),
                      static_cast<std::streamsize>(record.payload.size() * sizeof(float)));
        }
        if (!out) {
            if (reason) *reason = "failed while rewriting binary evidence store";
            return false;
        }

        EvidenceIndexEntry entry;
        entry.offset = offset;
        entry.size = header.record_size;
        entry.child_hull = record.child_hull;
        entry.unavailable = record.unavailable;
        entry.generation = record.generation;
        entry.checksum = record.checksum;
        upsert_evidence_index(record.key, entry);
        offset += header.record_size;
    }
    out.close();
    if (!static_cast<bool>(out) || !replace_file(tmp, path)) {
        if (reason) *reason = "failed to replace evidence store during migration";
        return false;
    }

    evidence_store_format_ = EvidenceStoreFormat::Binary;
    evidence_append_offset_ = offset;
    evidence_index_sidecar_dirty_ = true;
    return true;
}

bool LectDatabase::ensure_binary_evidence_store_file() const {
    if (config_.path.empty()) {
        return false;
    }
    std::filesystem::create_directories(config_.path);
    const auto path = evidence_path(config_.path);
    std::error_code error;
    const bool exists = std::filesystem::exists(path, error);
    if (error) {
        return false;
    }
    const auto file_size = exists ? std::filesystem::file_size(path, error) : 0;
    if (error) {
        return false;
    }
    if (!exists || file_size == 0) {
        std::ofstream out(path, std::ios::binary | std::ios::trunc);
        if (!out || !write_evidence_store_file_header(out)) {
            return false;
        }
        evidence_store_format_ = EvidenceStoreFormat::Binary;
        evidence_append_offset_ = sizeof(EvidenceStoreFileHeader);
        return true;
    }

    std::ifstream input(path, std::ios::binary);
    EvidenceStoreFileHeader header;
    if (!input || !read_evidence_store_file_header(input, &header)) {
        return false;
    }
    evidence_store_format_ = EvidenceStoreFormat::Binary;
    evidence_append_offset_ = file_size;
    return true;
}

bool LectDatabase::save_evidence_index_sidecar(std::uint64_t evidence_file_size) const {
    if (config_.path.empty()) {
        return false;
    }
    std::filesystem::create_directories(config_.path);
    const auto path = evidence_index_path(config_.path);
    const auto tmp = path.string() + ".tmp";
    std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
    if (!out) {
        return false;
    }
    const EvidenceIndexSidecarHeader header{
        kEvidenceIndexSidecarMagic,
        kEvidenceIndexSidecarSchemaVersion,
        0,
        evidence_file_size,
        static_cast<std::uint64_t>(evidence_index_count_),
    };
    out.write(reinterpret_cast<const char*>(&header), static_cast<std::streamsize>(sizeof(header)));
    std::vector<EvidenceIndexSidecarEntry> raw_entries;
    raw_entries.reserve(evidence_index_count_);
    for (const auto& slot : evidence_index_) {
        if (slot.key.node_id == kInvalidNodeId) {
            continue;
        }
        EvidenceIndexSidecarEntry raw_entry;
        raw_entry.node_id = slot.key.node_id;
        raw_entry.sector = static_cast<std::int32_t>(slot.key.sector);
        raw_entry.channel = static_cast<std::uint32_t>(slot.key.channel);
        raw_entry.endpoint_source = static_cast<std::uint32_t>(slot.key.endpoint_source);
        raw_entry.payload_kind = static_cast<std::uint32_t>(slot.key.payload_kind);
        raw_entry.flags = (slot.entry.child_hull ? kEvidenceIndexFlagChildHull : 0u) |
                          (slot.entry.unavailable ? kEvidenceIndexFlagUnavailable : 0u);
        raw_entry.size = slot.entry.size;
        raw_entry.offset = slot.entry.offset;
        raw_entry.generation = slot.entry.generation;
        raw_entry.checksum = slot.entry.checksum;
        raw_entries.push_back(raw_entry);
    }
    std::sort(raw_entries.begin(), raw_entries.end(), evidence_sidecar_entry_less);
    for (const auto& raw_entry : raw_entries) {
        out.write(reinterpret_cast<const char*>(&raw_entry), static_cast<std::streamsize>(sizeof(raw_entry)));
    }
    out.close();
    if (!static_cast<bool>(out) || !replace_file(tmp, path)) {
        return false;
    }
    evidence_index_sidecar_dirty_ = false;
    return true;
}

void LectDatabase::remember_evidence_metadata(const EvidenceRecord& record) {
    EvidenceIndexEntry entry;
    entry.child_hull = record.child_hull;
    entry.unavailable = record.unavailable;
    entry.generation = record.generation;
    entry.checksum = record.checksum;
    upsert_evidence_index(record.key, entry);
    evidence_index_sidecar_dirty_ = true;
}

bool LectDatabase::append_evidence_record_to_store(const EvidenceRecord& record) {
    if (config_.open.read_only) {
        return false;
    }
    if (!ensure_evidence_append_stream()) {
        return false;
    }
    const auto header = make_evidence_store_record_header(record);
    if (header.record_size == 0) {
        return false;
    }
    const std::uint64_t offset = evidence_append_offset_;
    evidence_append_stream_.write(reinterpret_cast<const char*>(&header),
                                  static_cast<std::streamsize>(sizeof(header)));
    if (!record.payload.empty()) {
        evidence_append_stream_.write(reinterpret_cast<const char*>(record.payload.data()),
                                      static_cast<std::streamsize>(record.payload.size() * sizeof(float)));
    }
    if (!evidence_append_stream_) {
        return false;
    }
    EvidenceIndexEntry entry;
    entry.offset = offset;
    entry.size = header.record_size;
    entry.child_hull = record.child_hull;
    entry.unavailable = record.unavailable;
    entry.generation = record.generation;
    entry.checksum = record.checksum;
    upsert_evidence_index(record.key, entry);
    evidence_append_offset_ += header.record_size;
    evidence_mapping_stale_ = true;
    ++evidence_appends_since_flush_;
    evidence_index_sidecar_dirty_ = true;
    return true;
}

std::shared_ptr<const EvidenceRecord> LectDatabase::load_indexed_evidence(const EvidenceKey& key) const {
    EvidenceKey normalized_key = key;
    if (!normalize_evidence_key(&normalized_key)) {
        return {};
    }
    const auto* index_entry = find_evidence_index(normalized_key);
    if (index_entry == nullptr) {
        return {};
    }
    const auto cached = evidence_.find(normalized_key);
    if (cached != evidence_.end()) {
        return cached->second;
    }
    if (index_entry->size == 0) {
        return {};
    }
    const auto bytes_view = load_evidence_bytes(index_entry->offset, index_entry->size);
    if (!bytes_view) {
        return {};
    }
    std::optional<EvidenceRecord> record;
    if (evidence_store_format_ == EvidenceStoreFormat::Binary) {
        record = parse_binary_evidence_record(*bytes_view);
    } else {
        record = parse_evidence_record(std::string_view(reinterpret_cast<const char*>(bytes_view->data()),
                                                        bytes_view->size()));
    }
    if (!record) {
        return {};
    }
    if (!normalize_evidence_key(&record->key)) {
        return {};
    }
    auto shared_record = std::make_shared<EvidenceRecord>(std::move(*record));
    auto [it, inserted] = evidence_.insert_or_assign(shared_record->key, shared_record);
    (void)inserted;
    return it->second;
}

bool LectDatabase::ensure_all_evidence_loaded() const {
    for (const auto& slot : evidence_index_) {
        if (slot.key.node_id == kInvalidNodeId) {
            continue;
        }
        if (evidence_.find(slot.key) == evidence_.end()) {
            if (!load_indexed_evidence(slot.key)) {
                return false;
            }
        }
    }
    return true;
}

bool LectDatabase::ensure_evidence_append_stream() const {
    if (config_.open.read_only || config_.path.empty()) {
        return false;
    }
    if (evidence_append_stream_.is_open()) {
        return static_cast<bool>(evidence_append_stream_);
    }
    if (!ensure_binary_evidence_store_file()) {
        return false;
    }
    evidence_append_stream_.open(evidence_path(config_.path), std::ios::binary | std::ios::app);
    return static_cast<bool>(evidence_append_stream_);
}

void LectDatabase::close_evidence_streams() const {
    if (evidence_append_stream_.is_open()) {
        evidence_append_stream_.flush();
        evidence_append_stream_.close();
    }
    evidence_append_stream_.clear();
    evidence_mapped_file_.reset();
    evidence_mapping_stale_ = false;
    evidence_read_buffer_.clear();
}

bool LectDatabase::flush_incremental_storage() const {
    if (!save_evidence()) {
        return false;
    }
    if (!flush_all_node_pages()) {
        return false;
    }
    if (!save_manifest()) {
        return false;
    }
    evidence_appends_since_flush_ = 0;
    return true;
}

bool LectDatabase::maybe_flush_incremental_storage() const {
    if (evidence_appends_since_flush_ < kEvidenceAppendsPerFlush) {
        return true;
    }
    return flush_incremental_storage();
}

void LectDatabase::trim_evidence_cache() const {
    if (evidence_.size() <= kMaxResidentEvidenceRecords) {
        return;
    }
    save_evidence();
    for (auto it = evidence_.begin(); it != evidence_.end();) {
        const auto* index_entry = find_evidence_index(it->first);
        if (index_entry != nullptr && index_entry->size > 0) {
            it = evidence_.erase(it);
        } else {
            ++it;
        }
    }
}

void LectDatabase::replay_journal() {
    std::ifstream input(journal_path(config_.path));
    if (!input) {
        return;
    }
    std::vector<std::string> pending;
    std::string line;
    while (std::getline(input, line)) {
        if (line.rfind("begin|", 0) == 0) {
            pending.clear();
            continue;
        }
        if (line.rfind("commit|", 0) == 0) {
            if (!pending.empty()) {
                pending_changes_ = true;
            }
            for (const auto& record : pending) {
                if (record.rfind("split|", 0) == 0) {
                    const auto parts = split(record, '|');
                    if (parts.size() < 6) {
                        continue;
                    }
                    const NodeId parent_id = static_cast<NodeId>(std::stoull(parts[1]));
                    const NodeId left_id = static_cast<NodeId>(std::stoull(parts[2]));
                    const NodeId right_id = static_cast<NodeId>(std::stoull(parts[3]));
                    const int split_dim = std::stoi(parts[4]);
                    const double split_value = std::stod(parts[5]);
                    if (!valid_node_id(left_id) || !valid_node_id(right_id) || left_id == right_id) {
                        continue;
                    }
                    const auto parent_record = read_node(parent_id);
                    if (parent_record) {
                        const auto left_existing = read_node(left_id);
                        const auto right_existing = read_node(right_id);
                        const bool already_applied = left_existing && right_existing &&
                            parent_record->left == left_id && parent_record->right == right_id &&
                            parent_record->split_dim == split_dim && parent_record->split_value == split_value;
                        if (already_applied) {
                            continue;
                        }
                        ++generation_;
                        const int depth = parent_record->depth + 1;
                        PathCode left_path = parent_record->path;
                        PathCode right_path = left_path;
                        left_path.push_child(false);
                        right_path.push_child(true);
                        if (!read_node(left_id)) {
                            NodeRecord left_record;
                            left_record.id = left_id;
                            left_record.parent = parent_id;
                            left_record.depth = depth;
                            left_record.path = std::move(left_path);
                            left_record.generation = generation_;
                            write_node_record(std::move(left_record));
                        }
                        if (!read_node(right_id)) {
                            NodeRecord right_record;
                            right_record.id = right_id;
                            right_record.parent = parent_id;
                            right_record.depth = depth;
                            right_record.path = std::move(right_path);
                            right_record.generation = generation_;
                            write_node_record(std::move(right_record));
                        }
                        if (auto* parent_node = mutable_node(parent_id)) {
                            parent_node->left = left_id;
                            parent_node->right = right_id;
                            parent_node->split_dim = split_dim;
                            parent_node->split_value = split_value;
                            parent_node->dirty = true;
                        }
                    }
                } else if (record.rfind("evidence|", 0) == 0) {
                    auto parsed = parse_evidence_record(record.substr(9));
                    if (parsed) {
                        if (!normalize_evidence_key(&parsed->key)) {
                            continue;
                        }
                        auto shared_record = std::make_shared<EvidenceRecord>(std::move(*parsed));
                        evidence_[shared_record->key] = shared_record;
                        remember_evidence_metadata(*shared_record);
                        if (!config_.open.read_only) {
                            append_evidence_record_to_store(*shared_record);
                        }
                        if (config_.propagate_parent_hulls) {
                            propagate_parent_hulls(shared_record->key.node_id, shared_record->key);
                        }
                    }
                }
            }
            pending.clear();
            ++stats_.journal_transactions;
            continue;
        }
        if (!line.empty()) {
            pending.push_back(line);
        }
    }
    rebuild_layer_index();
    assign_page_ids();
}

bool LectDatabase::ensure_journal_append_stream() {
    if (config_.open.read_only || config_.path.empty()) {
        return false;
    }
    if (journal_append_stream_.is_open()) {
        return static_cast<bool>(journal_append_stream_);
    }
    std::filesystem::create_directories(config_.path);
    journal_append_stream_.open(journal_path(config_.path), std::ios::app);
    return static_cast<bool>(journal_append_stream_);
}

void LectDatabase::close_journal_append_stream() {
    if (journal_append_stream_.is_open()) {
        journal_append_stream_.flush();
        journal_append_stream_.close();
    }
    journal_append_stream_.clear();
}

void LectDatabase::append_committed_transaction(const LectDbTransaction& transaction) {
    if (config_.path.empty() || !ensure_journal_append_stream()) {
        return;
    }
    journal_append_stream_ << "begin|" << transaction.generation << '\n';
    for (const auto& record : transaction.records) {
        journal_append_stream_ << record << '\n';
    }
    if (transaction.committed) {
        journal_append_stream_ << "commit|" << transaction.generation << '\n';
    }
    journal_append_stream_.flush();
    ++stats_.journal_transactions;
}

bool LectDatabase::propagate_parent_hulls(NodeId node_id, EvidenceKey key_template) {
    return propagate_parent_hulls_from(parent(node_id), std::move(key_template), nullptr);
}

bool LectDatabase::propagate_parent_hulls_from(NodeId parent_id,
                                               EvidenceKey key_template,
                                               std::shared_ptr<const EvidenceRecord> child_record) {
    NodeId cursor = parent_id;
    while (has_node(cursor)) {
        const auto parent_node = read_node(cursor);
        if (!parent_node) {
            break;
        }
        auto parent_record = child_record != nullptr
            ? build_parent_hull_from_child(*parent_node, *child_record, key_template)
            : build_parent_hull_from_node(*parent_node, key_template);
        if (!parent_record) {
            break;
        }
        parent_record->generation = generation_;
        parent_record->checksum = payload_checksum(parent_record->payload);
        auto shared_parent_record = std::make_shared<EvidenceRecord>(std::move(*parent_record));
        auto [parent_it, parent_inserted] = evidence_.insert_or_assign(shared_parent_record->key,
                                                                       shared_parent_record);
        (void)parent_inserted;
        remember_evidence_metadata(*parent_it->second);
        if (!config_.open.read_only && !append_evidence_record_to_store(*parent_it->second)) {
            return false;
        }
        child_record = parent_it->second;
        cursor = parent_node->parent;
    }
    return true;
}

bool LectDatabase::drain_deferred_parent_hulls() {
    if (deferred_parent_hull_writes_.empty()) {
        return true;
    }
    if (!config_.propagate_parent_hulls) {
        deferred_parent_hull_writes_.clear();
        return true;
    }

    std::vector<DeferredParentHullWrite> pending;
    pending.swap(deferred_parent_hull_writes_);
    for (const auto& item : pending) {
        const auto node_item = read_node(item.key.node_id);
        if (!node_item) {
            return false;
        }
        auto stored = evidence(item.key);
        if (!stored) {
            continue;
        }
        std::shared_ptr<const EvidenceRecord> propagated_child = stored->storage;
        if (!stored->child_hull && !node_item->is_leaf()) {
            if (auto child_hull = build_parent_hull_from_node(*node_item, item.key)) {
                child_hull->generation = generation_;
                child_hull->checksum = payload_checksum(child_hull->payload);
                auto child_hull_record = std::make_shared<EvidenceRecord>(std::move(*child_hull));
                auto [child_hull_it, child_hull_inserted] = evidence_.insert_or_assign(child_hull_record->key,
                                                                                       child_hull_record);
                (void)child_hull_inserted;
                if (!append_evidence_record_to_store(*child_hull_it->second)) {
                    return false;
                }
                propagated_child = child_hull_it->second;
                pending_changes_ = true;
            }
        }
        if (!propagate_parent_hulls_from(node_item->parent, item.key, propagated_child)) {
            return false;
        }
    }
    return true;
}

std::optional<EvidenceRecord> LectDatabase::build_parent_hull_from_child(const NodeRecord& parent_node,
                                                                         const EvidenceRecord& child_record,
                                                                         const EvidenceKey& key_template) const {
    if (parent_node.is_leaf() || child_record.unavailable) {
        return std::nullopt;
    }
    const bool child_is_left = child_record.key.node_id == parent_node.left;
    const bool child_is_right = child_record.key.node_id == parent_node.right;
    if (!child_is_left && !child_is_right) {
        return build_parent_hull_from_node(parent_node, key_template);
    }

    EvidenceKey sibling_key = evidence_key_for_node(child_is_left ? parent_node.right : parent_node.left,
                                                    &key_template);
    const auto sibling_record = evidence(sibling_key);
    if (!sibling_record || sibling_record->payload.size() != child_record.payload.size()) {
        return std::nullopt;
    }

    const auto left_payload = child_is_left ? std::span<const float>(child_record.payload)
                                            : sibling_record->payload;
    const auto right_payload = child_is_left ? sibling_record->payload
                                             : std::span<const float>(child_record.payload);
    EvidenceRecord out;
    out.key = evidence_key_for_node(parent_node.id, &key_template);
    out.child_hull = true;
    if (!merge_payload_hull(key_template.payload_kind,
                            left_payload,
                            right_payload,
                            out.payload)) {
        return std::nullopt;
    }
    return out;
}

std::optional<EvidenceRecord> LectDatabase::build_parent_hull_from_node(const NodeRecord& parent_node,
                                                                        const EvidenceKey& key_template) const {
    if (parent_node.is_leaf()) {
        return std::nullopt;
    }
    EvidenceKey left_key = evidence_key_for_node(parent_node.left, &key_template);
    EvidenceKey right_key = evidence_key_for_node(parent_node.right, &key_template);
    const auto left_record = evidence(left_key);
    const auto right_record = evidence(right_key);
    if (!left_record || !right_record || left_record->payload.size() != right_record->payload.size()) {
        return std::nullopt;
    }
    EvidenceRecord out;
    out.key = evidence_key_for_node(parent_node.id, &key_template);
    out.child_hull = true;
    if (!merge_payload_hull(key_template.payload_kind,
                            left_record->payload,
                            right_record->payload,
                            out.payload)) {
        return std::nullopt;
    }
    return out;
}

std::optional<EvidenceRecord> LectDatabase::build_parent_hull(NodeId parent_id,
                                                              const EvidenceKey& key_template) const {
    const auto parent_node = read_node(parent_id);
    if (!parent_node) {
        return std::nullopt;
    }
    return build_parent_hull_from_node(*parent_node, key_template);
}

LectDbQuerySession::LectDbQuerySession(const LectDatabase& database)
    : database_(&database) {}

std::optional<std::vector<Interval>> LectDbQuerySession::node_box(NodeId node_id) {
    if (database_ == nullptr) {
        return std::nullopt;
    }
    if (node_id == cached_node_) {
        ++stats_.query_path_cache_hits;
        return cached_box_;
    }
    ++stats_.query_path_cache_misses;
    auto box = database_->node_box(node_id);
    if (box) {
        cached_node_ = node_id;
        cached_box_ = *box;
    }
    return box;
}

BoxLookupResult LectDbQuerySession::box_to_node_exact(const BoxKey& box) {
    if (database_ == nullptr) {
        return {false, kInvalidNodeId, "query session is not attached"};
    }
    if (database_->has_node(cached_node_) && database_->intervals_equal(cached_box_, box.intervals, box.tolerance)) {
        ++stats_.query_path_cache_hits;
        return {true, cached_node_, {}};
    }
    ++stats_.query_path_cache_misses;
    auto result = database_->box_to_node_exact(box);
    if (result.found) {
        cached_node_ = result.node_id;
        cached_box_ = box.intervals;
    }
    return result;
}

}  // namespace rbf::lect_database
