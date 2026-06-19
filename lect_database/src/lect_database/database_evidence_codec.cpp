#include "database_evidence_codec.h"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>

namespace rbf::lect_database {

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
    if (lhs.payload_kind != rhs.payload_kind) {
        return lhs.payload_kind < rhs.payload_kind;
    }
    return lhs.path_blob_offset < rhs.path_blob_offset;
}

bool split_policy_prefix_compatible(const SplitPolicyDescriptor& stored,
                                    const SplitPolicyDescriptor& requested,
                                    int requested_max_depth) {
    if (stored.strategy != requested.strategy ||
        stored.min_width != requested.min_width ||
        stored.midpoint != requested.midpoint ||
        stored.deterministic_tie_break != requested.deterministic_tie_break) {
        return false;
    }
    if (stored.strategy != SplitStrategy::AAFKVolumeMin) {
        return true;
    }
    if (requested_max_depth <= 0) {
        return false;
    }
    const auto required = static_cast<std::size_t>(requested_max_depth);
    if (requested.depth_dimensions.size() < required ||
        stored.depth_dimensions.size() < required) {
        return false;
    }
    for (std::size_t index = 0; index < required; ++index) {
        if (stored.depth_dimensions[index] != requested.depth_dimensions[index]) {
            return false;
        }
    }
    return true;
}

bool evidence_sidecar_offsets_sorted(std::span<const EvidenceIndexSidecarEntry> entries) {
    for (std::size_t index = 1; index < entries.size(); ++index) {
        if (evidence_sidecar_entry_less(entries[index], entries[index - 1])) {
            return false;
        }
    }
    return true;
}

std::uint64_t payload_checksum(std::span<const float> payload) {
    std::uint64_t hash = 1469598103934665603ull;
    if (!payload.empty()) {
        hash = stable_hash_append(hash, payload.data(), payload.size() * sizeof(float));
    }
    return hash;
}

// ─── IEEE-754 binary16 (half) codec with directed (outward) rounding ─────────
// Evidence payloads are conservative outer envelopes (min xyz / max xyz). To
// halve on-disk size we persist each float as a 16-bit half. To keep the box a
// *sound* (never-shrinking) outer bound we round min slots toward -inf and max
// slots toward +inf, so the dequantized box always contains the original one.
// Payloads are snapped to half-representable float32 values *before* the
// checksum is taken, so the in-memory value, the checksum, and the on-disk half
// are all mutually consistent and the read path is exact.
std::uint16_t f16_from_f32_nearest(float value) {
    std::uint32_t x;
    std::memcpy(&x, &value, sizeof(x));
    const std::uint32_t sign = (x >> 16) & 0x8000u;
    const std::uint32_t biased = (x >> 23) & 0xFFu;
    std::uint32_t mant = x & 0x7FFFFFu;
    if (biased == 0xFFu) {  // inf / nan
        return static_cast<std::uint16_t>(sign | (mant ? 0x7E00u : 0x7C00u));
    }
    const std::int32_t exp = static_cast<std::int32_t>(biased) - 127 + 15;
    if (exp >= 0x1F) {  // overflow -> inf
        return static_cast<std::uint16_t>(sign | 0x7C00u);
    }
    if (exp <= 0) {  // subnormal / zero
        if (exp < -10) {
            return static_cast<std::uint16_t>(sign);
        }
        mant |= 0x800000u;
        const std::uint32_t shift = static_cast<std::uint32_t>(14 - exp);  // 14..24
        std::uint32_t half_mant = mant >> shift;
        const std::uint32_t remainder = mant & ((1u << shift) - 1u);
        const std::uint32_t halfway = 1u << (shift - 1u);
        if (remainder > halfway || (remainder == halfway && (half_mant & 1u))) {
            ++half_mant;
        }
        return static_cast<std::uint16_t>(sign | half_mant);
    }
    std::uint16_t half_bits = static_cast<std::uint16_t>((exp << 10) | (mant >> 13));
    const std::uint32_t remainder = mant & 0x1FFFu;
    if (remainder > 0x1000u || (remainder == 0x1000u && (half_bits & 1u))) {
        ++half_bits;  // may carry into exponent (and to inf), which stays sound
    }
    return half_bits | static_cast<std::uint16_t>(sign);
}

float f32_from_f16(std::uint16_t h) {
    const std::uint32_t sign = static_cast<std::uint32_t>(h & 0x8000u) << 16;
    std::uint32_t exp = (h >> 10) & 0x1Fu;
    std::uint32_t mant = h & 0x3FFu;
    std::uint32_t out;
    if (exp == 0) {
        if (mant == 0) {
            out = sign;
        } else {
            exp = 127 - 15 + 1;
            while ((mant & 0x400u) == 0u) {
                mant <<= 1;
                --exp;
            }
            mant &= 0x3FFu;
            out = sign | (exp << 23) | (mant << 13);
        }
    } else if (exp == 0x1F) {
        out = sign | 0x7F800000u | (mant << 13);
    } else {
        out = sign | ((exp - 15u + 127u) << 23) | (mant << 13);
    }
    float value;
    std::memcpy(&value, &out, sizeof(value));
    return value;
}

// Monotonic ordering key so we can step to the adjacent half value.
std::uint16_t half_order_key(std::uint16_t h) {
    return (h & 0x8000u) ? static_cast<std::uint16_t>(~h)
                         : static_cast<std::uint16_t>(h | 0x8000u);
}
std::uint16_t half_from_order_key(std::uint16_t k) {
    return (k & 0x8000u) ? static_cast<std::uint16_t>(k & 0x7FFFu)
                         : static_cast<std::uint16_t>(~k);
}
bool half_is_inf(std::uint16_t h) {
    return ((h >> 10) & 0x1Fu) == 0x1Fu && (h & 0x3FFu) == 0u;
}
std::uint16_t half_next_up(std::uint16_t h) {
    if (half_is_inf(h) && !(h & 0x8000u)) {
        return h;  // already +inf
    }
    return half_from_order_key(static_cast<std::uint16_t>(half_order_key(h) + 1u));
}
std::uint16_t half_next_down(std::uint16_t h) {
    if (half_is_inf(h) && (h & 0x8000u)) {
        return h;  // already -inf
    }
    return half_from_order_key(static_cast<std::uint16_t>(half_order_key(h) - 1u));
}

// Snap a float to a half-representable value, rounding outward in the requested
// direction, and return it as float32 (which is then bit-exact across the
// half round-trip).
float snap_to_half_outward(float value, bool round_up) {
    if (!std::isfinite(value)) {
        return value;
    }
    std::uint16_t h = f16_from_f32_nearest(value);
    const float decoded = f32_from_f16(h);
    if (round_up) {
        if (decoded < value) {
            h = half_next_up(h);
        }
    } else if (decoded > value) {
        h = half_next_down(h);
    }
    return f32_from_f16(h);
}

// Quantize a payload in place to outward-rounded half precision. Mirrors the
// min/max slot orientation used by merge_payload_hull so the box only grows.
void quantize_payload_outward(EvidencePayloadKind kind, std::vector<float>& payload) {
    const std::size_t n = payload.size();
    if (kind == EvidencePayloadKind::EndpointEnvelope && (n % 6) == 0) {
        for (std::size_t i = 0; i < n; i += 6) {
            payload[i + 0] = snap_to_half_outward(payload[i + 0], false);
            payload[i + 1] = snap_to_half_outward(payload[i + 1], false);
            payload[i + 2] = snap_to_half_outward(payload[i + 2], false);
            payload[i + 3] = snap_to_half_outward(payload[i + 3], true);
            payload[i + 4] = snap_to_half_outward(payload[i + 4], true);
            payload[i + 5] = snap_to_half_outward(payload[i + 5], true);
        }
        return;
    }
    for (std::size_t i = 0; i < n; i += 2) {
        payload[i] = snap_to_half_outward(payload[i], false);
        if (i + 1 < n) {
            payload[i + 1] = snap_to_half_outward(payload[i + 1], true);
        }
    }
}

std::uint32_t path_word_count_for_bits(int bit_count) {
    if (bit_count < 0) {
        return 0;
    }
    if (bit_count == 0) {
        return 0;
    }
    return static_cast<std::uint32_t>((static_cast<std::uint64_t>(bit_count) + 63u) / 64u);
}

bool path_code_storage_valid(const PathCode& path) {
    return path.words.size() == path_word_count_for_bits(path.bit_count);
}

std::uint64_t path_code_storage_bytes(std::uint32_t path_word_count) {
    return static_cast<std::uint64_t>(path_word_count) * sizeof(std::uint64_t);
}

std::optional<PathCode> parse_path_code_blob(std::span<const std::byte> bytes,
                                             std::uint32_t path_word_count,
                                             std::uint32_t path_bit_count) {
    if (path_word_count == 0 && path_bit_count == 0) {
        return PathCode{};
    }
    const auto expected_word_count = path_word_count_for_bits(static_cast<int>(path_bit_count));
    if (path_word_count != expected_word_count) {
        return std::nullopt;
    }
    const auto required_bytes = path_code_storage_bytes(path_word_count);
    if (bytes.size() != required_bytes) {
        return std::nullopt;
    }

    PathCode path;
    path.bit_count = static_cast<int>(path_bit_count);
    path.words.resize(path_word_count);
    if (required_bytes > 0) {
        std::memcpy(path.words.data(), bytes.data(), static_cast<std::size_t>(required_bytes));
    }
    if (!path_code_storage_valid(path)) {
        return std::nullopt;
    }
    return path;
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

std::uint32_t evidence_binary_record_size(std::size_t payload_count, std::uint32_t path_word_count) {
    const auto path_bytes = path_code_storage_bytes(path_word_count);
    const auto payload_bytes = static_cast<std::uint64_t>(payload_count) * sizeof(std::uint16_t);
    const auto total_bytes = sizeof(EvidenceStoreRecordHeader) + path_bytes + payload_bytes;
    return total_bytes > std::numeric_limits<std::uint32_t>::max()
        ? 0u
        : static_cast<std::uint32_t>(total_bytes);
}

EvidenceStoreRecordHeader make_evidence_store_record_header(const EvidenceRecord& record) {
    EvidenceStoreRecordHeader header;
    if (record.payload.size() > std::numeric_limits<std::uint32_t>::max() ||
        !record.key.node_path_valid ||
        !path_code_storage_valid(record.key.node_path)) {
        return header;
    }
    const auto path_word_count = path_word_count_for_bits(record.key.node_path.bit_count);
    header.node_id = record.key.node_id;
    header.sector = static_cast<std::int32_t>(record.key.sector);
    header.channel = static_cast<std::uint32_t>(record.key.channel);
    header.endpoint_source = static_cast<std::uint32_t>(record.key.endpoint_source);
    header.payload_kind = static_cast<std::uint32_t>(record.key.payload_kind);
    header.flags = (record.child_hull ? kEvidenceIndexFlagChildHull : 0u) |
                   (record.unavailable ? kEvidenceIndexFlagUnavailable : 0u);
    header.payload_count = static_cast<std::uint32_t>(record.payload.size());
    header.path_word_count = path_word_count;
    header.path_bit_count = static_cast<std::uint32_t>(record.key.node_path.bit_count);
    header.record_size = evidence_binary_record_size(record.payload.size(), path_word_count);
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
    const auto path_bytes = static_cast<std::size_t>(path_code_storage_bytes(header.path_word_count));
    const auto payload_bytes = static_cast<std::size_t>(header.payload_count) * sizeof(std::uint16_t);
    if (sizeof(EvidenceStoreRecordHeader) + path_bytes + payload_bytes != bytes.size()) {
        return std::nullopt;
    }
    const auto path = parse_path_code_blob(
        bytes.subspan(sizeof(EvidenceStoreRecordHeader), path_bytes),
        header.path_word_count,
        header.path_bit_count);
    if (!path) {
        return std::nullopt;
    }

    EvidenceRecord record;
    record.key.node_id = header.node_id;
    record.key.node_path = *path;
    record.key.node_path_valid = true;
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
        const auto* half_bytes = bytes.data() + sizeof(EvidenceStoreRecordHeader) + path_bytes;
        for (std::size_t i = 0; i < header.payload_count; ++i) {
            std::uint16_t h;
            std::memcpy(&h, half_bytes + i * sizeof(std::uint16_t), sizeof(h));
            record.payload[i] = f32_from_f16(h);
        }
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


}  // namespace rbf::lect_database
