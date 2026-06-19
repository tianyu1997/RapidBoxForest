#pragma once

#include <rbf/lect_database/types.h>

#include <cstdint>

namespace rbf::lect_database {

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

}  // namespace rbf::lect_database
