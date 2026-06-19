#pragma once

#include <rbf/lect_database/database.h>

#include <cstddef>
#include <cstdint>
#include <iosfwd>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

namespace rbf::lect_database {

constexpr std::size_t kEvidenceIndexLoadFactorNumerator = 13;
constexpr std::size_t kEvidenceIndexLoadFactorDenominator = 16;
// v4: evidence payloads persisted as outward-rounded IEEE binary16 (half),
// halving on-disk size while keeping envelopes conservatively sound.
constexpr std::uint32_t kEvidenceStoreSchemaVersion = 4;
constexpr std::uint64_t kEvidenceStoreMagic = 0x3156454242464652ull;
constexpr std::uint32_t kEvidenceIndexSidecarSchemaVersion = 4;
constexpr std::uint64_t kEvidenceIndexSidecarMagic = 0x3158444945424652ull;

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
    std::uint32_t path_word_count = 0;
    std::uint32_t path_bit_count = 0;
    std::uint32_t record_size = 0;
    std::uint64_t generation = 0;
    std::uint64_t checksum = 0;
};

struct EvidenceIndexSidecarHeader {
    std::uint64_t magic = kEvidenceIndexSidecarMagic;
    std::uint32_t version = kEvidenceIndexSidecarSchemaVersion;
    std::uint32_t header_size = 40u;
    std::uint64_t evidence_file_size = 0;
    std::uint64_t entry_count = 0;
    std::uint64_t path_blob_offset = 0;
};

struct EvidenceIndexSidecarEntry {
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

static_assert(sizeof(EvidenceStoreFileHeader) == 32);
static_assert(sizeof(EvidenceStoreRecordHeader) == 60);
static_assert(sizeof(EvidenceIndexSidecarHeader) == 40);
static_assert(sizeof(EvidenceIndexSidecarEntry) == 72);
static_assert(alignof(EvidenceIndexSidecarHeader) == 1);
static_assert(alignof(EvidenceIndexSidecarEntry) == 1);
static_assert(std::is_trivially_copyable_v<EvidenceStoreFileHeader>);
static_assert(std::is_trivially_copyable_v<EvidenceStoreRecordHeader>);
static_assert(std::is_trivially_copyable_v<EvidenceIndexSidecarHeader>);
static_assert(std::is_trivially_copyable_v<EvidenceIndexSidecarEntry>);


std::size_t next_power_of_two(std::size_t value);
std::size_t evidence_index_slot_count(std::size_t item_count);
bool evidence_sidecar_entry_less(const EvidenceIndexSidecarEntry& lhs,
                                 const EvidenceIndexSidecarEntry& rhs);
bool split_policy_prefix_compatible(const SplitPolicyDescriptor& stored,
                                    const SplitPolicyDescriptor& requested,
                                    int requested_max_depth);
bool evidence_sidecar_offsets_sorted(std::span<const EvidenceIndexSidecarEntry> entries);
std::uint64_t payload_checksum(std::span<const float> payload);

std::uint16_t f16_from_f32_nearest(float value);
float f32_from_f16(std::uint16_t h);
std::uint16_t half_order_key(std::uint16_t h);
std::uint16_t half_from_order_key(std::uint16_t k);
bool half_is_inf(std::uint16_t h);
std::uint16_t half_next_up(std::uint16_t h);
std::uint16_t half_next_down(std::uint16_t h);
float snap_to_half_outward(float value, bool round_up);
void quantize_payload_outward(EvidencePayloadKind kind, std::vector<float>& payload);

std::uint32_t path_word_count_for_bits(int bit_count);
bool path_code_storage_valid(const PathCode& path);
std::uint64_t path_code_storage_bytes(std::uint32_t path_word_count);
std::optional<PathCode> parse_path_code_blob(std::span<const std::byte> bytes,
                                             std::uint32_t path_word_count,
                                             std::uint32_t path_bit_count);

EvidenceRecordView make_evidence_view(const std::shared_ptr<const EvidenceRecord>& record);
EvidenceRecord clone_evidence_record(const EvidenceRecordView& view);
std::uint32_t evidence_binary_record_size(std::size_t payload_count,
                                          std::uint32_t path_word_count);
EvidenceStoreRecordHeader make_evidence_store_record_header(const EvidenceRecord& record);
bool read_evidence_store_file_header(std::ifstream& input, EvidenceStoreFileHeader* header);
bool write_evidence_store_file_header(std::ofstream& output);
std::optional<EvidenceRecord> parse_binary_evidence_record(std::span<const std::byte> bytes);
std::string serialize_evidence_record(const EvidenceRecord& record);
std::optional<EvidenceRecord> parse_evidence_record(std::string_view line);
bool payloads_equal(std::span<const float> lhs, std::span<const float> rhs);
bool merge_payload_hull(EvidencePayloadKind kind,
                        std::span<const float> lhs,
                        std::span<const float> rhs,
                        std::vector<float>& out);

}  // namespace rbf::lect_database
