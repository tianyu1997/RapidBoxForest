#include "read_snapshot_evidence.h"

#include "read_snapshot_payload.h"

#include <cstddef>

namespace rbf::lect_database {
namespace {

std::optional<EvidenceRecordView> direct_evidence_view(
    std::span<const SnapshotDirectEvidenceEntry> direct_evidence,
    const EvidenceKey& key,
    const std::shared_ptr<ReadSnapshotMappedFile>& payload_file) {
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

std::optional<EvidenceRecordView> evidence_slot_view(
    const SnapshotEvidenceSlot& slot,
    const EvidenceKey& key,
    const std::shared_ptr<ReadSnapshotMappedFile>& payload_file) {
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

std::optional<EvidenceRecordView> lookup_evidence_slot(
    std::span<const SnapshotEvidenceSlot> evidence_slots,
    const EvidenceKey& key,
    const std::shared_ptr<ReadSnapshotMappedFile>& payload_file) {
    if (evidence_slots.empty()) {
        return std::nullopt;
    }
    const auto hash = hash_snapshot_evidence_key(
        key.node_id, key.sector, key.channel, key.endpoint_source, key.payload_kind);
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

}  // namespace

std::uint64_t hash_snapshot_evidence_key(NodeId node_id,
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

std::optional<EvidenceRecordView> lookup_snapshot_evidence_uncached(
    std::span<const SnapshotDirectEvidenceEntry> direct_evidence,
    std::span<const SnapshotEvidenceSlot> evidence_slots,
    const EvidenceKey& key,
    const std::shared_ptr<ReadSnapshotMappedFile>& payload_file) {
    if (auto view = direct_evidence_view(direct_evidence, key, payload_file)) {
        return view;
    }
    return lookup_evidence_slot(evidence_slots, key, payload_file);
}

}  // namespace rbf::lect_database
