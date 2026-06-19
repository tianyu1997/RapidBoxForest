#pragma once

#include "read_snapshot_format.h"
#include "read_snapshot_mapped_file.h"

#include <rbf/lect_database/types.h>

#include <cstdint>
#include <memory>
#include <optional>
#include <span>

namespace rbf::lect_database {

std::uint64_t hash_snapshot_evidence_key(NodeId node_id,
                                         SectorId sector,
                                         EvidenceChannel channel,
                                         EndpointSource endpoint_source,
                                         EvidencePayloadKind payload_kind);

std::optional<EvidenceRecordView> lookup_snapshot_evidence_uncached(
    std::span<const SnapshotDirectEvidenceEntry> direct_evidence,
    std::span<const SnapshotEvidenceSlot> evidence_slots,
    const EvidenceKey& key,
    const std::shared_ptr<ReadSnapshotMappedFile>& payload_file);

}  // namespace rbf::lect_database
