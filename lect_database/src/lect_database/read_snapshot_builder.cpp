#include "read_snapshot_builder.h"

#include "read_snapshot_evidence.h"
#include "read_snapshot_format.h"
#include "read_snapshot_legacy.h"
#include "read_snapshot_manifest.h"
#include "read_snapshot_paths.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <ios>
#include <string>
#include <system_error>
#include <vector>

namespace rbf::lect_database {
namespace {

std::size_t next_power_of_two(std::size_t value) {
    std::size_t result = 1;
    while (result < value) {
        result <<= 1;
    }
    return result;
}

template <typename T>
bool write_object(std::ofstream& out, const T& value) {
    out.write(reinterpret_cast<const char*>(&value), static_cast<std::streamsize>(sizeof(T)));
    return static_cast<bool>(out);
}

}  // namespace

bool build_read_snapshot_from_legacy(const std::filesystem::path& legacy_root,
                                     const std::filesystem::path& snapshot_path,
                                     std::string* reason) {
    const auto values = read_snapshot_manifest_values(legacy_manifest_path(legacy_root));
    if (values.empty()) {
        if (reason) *reason = "legacy manifest is missing or empty";
        return false;
    }
    const auto root = parse_snapshot_root_intervals(values);
    if (root.empty()) {
        if (reason) *reason = "legacy manifest has no root intervals";
        return false;
    }

    const auto generation = snapshot_manifest_u64(values, "generation");
    const auto node_count = snapshot_manifest_u64(values, "node_count");
    const auto max_node_id = snapshot_manifest_u64(values, "max_node_id");
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
        node_input.read(reinterpret_cast<char*>(&legacy_node_header),
                        static_cast<std::streamsize>(sizeof(legacy_node_header)));
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
        const auto slot_count = next_power_of_two(
            std::max<std::size_t>(16, static_cast<std::size_t>(evidence_count) * 2u));
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
                legacy_path_code_storage_bytes(raw.path_word_count);
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
            slot.payload_count = static_cast<std::uint32_t>(
                (raw.size - relative_payload_offset) / sizeof(std::uint16_t));
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

            const auto hash = hash_snapshot_evidence_key(slot.node_id,
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
        const SnapshotEvidenceTableHeader header{kSnapshotEvidenceTableMagic,
                                                 kSnapshotFormatVersion,
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
        const SnapshotManifestHeader header{
            kSnapshotManifestMagic,
            kSnapshotFormatVersion,
            sizeof(SnapshotManifestHeader),
            generation,
            node_count,
            max_node_id,
            evidence_count,
            snapshot_manifest_u64(values, "root_domain_fingerprint", fingerprint_intervals(root)),
            snapshot_manifest_u64(values, "split_policy_hash"),
            evidence_file_size,
            static_cast<std::uint32_t>(root.size()),
            static_cast<std::uint32_t>(snapshot_manifest_int(values, "split_strategy")),
            snapshot_manifest_double(values, "split_min_width"),
            static_cast<std::uint32_t>(snapshot_manifest_int(values, "split_midpoint", 1)),
            static_cast<std::uint32_t>(snapshot_manifest_int(values, "split_deterministic_tie_break", 1))};
        std::ofstream out(snapshot_manifest_path(staging), std::ios::binary | std::ios::trunc);
        if (!write_object(out, header)) {
            if (reason) *reason = "failed to write snapshot manifest header";
            return false;
        }
        out.write(reinterpret_cast<const char*>(root.data()),
                  static_cast<std::streamsize>(root.size() * sizeof(Interval)));
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

}  // namespace rbf::lect_database
