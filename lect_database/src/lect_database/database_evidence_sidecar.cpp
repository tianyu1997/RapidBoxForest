#include <rbf/lect_database/database.h>

#include "database_evidence_codec.h"
#include "database_file_layout.h"
#include "database_mapped_file.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <span>
#include <vector>

namespace rbf::lect_database {

namespace {

using database_file::evidence_index_path;
using database_file::replace_file;

}  // namespace

void LectDatabase::prefetch_indexed_evidence_ranges() const {
    if (evidence_index_count_ == 0) {
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
        header.header_size != sizeof(EvidenceIndexSidecarHeader) ||
        header.evidence_file_size != evidence_file_size) {
        return false;
    }
    const auto expected_count = static_cast<std::size_t>(header.entry_count);
    const auto entries_bytes = expected_count * sizeof(EvidenceIndexSidecarEntry);
    const auto entries_offset = static_cast<std::size_t>(header.header_size);
    const auto expected_path_blob_offset = entries_offset + entries_bytes;
    if (entries_bytes / sizeof(EvidenceIndexSidecarEntry) != expected_count ||
        header.path_blob_offset != expected_path_blob_offset ||
        bytes.size() < expected_path_blob_offset) {
        return false;
    }
    const auto* raw_entry_data = reinterpret_cast<const EvidenceIndexSidecarEntry*>(
        bytes.data() + entries_offset);
    const std::span<const EvidenceIndexSidecarEntry> raw_entries(raw_entry_data, expected_count);
    if (!evidence_sidecar_offsets_sorted(raw_entries)) {
        return false;
    }
    const auto path_blob = bytes.subspan(static_cast<std::size_t>(header.path_blob_offset));

    clear_evidence_index();
    reserve_evidence_index(expected_count);
    for (const auto& raw_entry : raw_entries) {
        EvidenceKey key;
        key.node_id = raw_entry.node_id;
        const auto path_bytes = static_cast<std::size_t>(path_code_storage_bytes(raw_entry.path_word_count));
        if (raw_entry.path_blob_offset > path_blob.size() || path_bytes > path_blob.size() - raw_entry.path_blob_offset) {
            return false;
        }
        const auto path = parse_path_code_blob(
            path_blob.subspan(static_cast<std::size_t>(raw_entry.path_blob_offset), path_bytes),
            raw_entry.path_word_count,
            raw_entry.path_bit_count);
        if (!path) {
            return false;
        }
        key.node_path = *path;
        key.node_path_valid = true;
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
    std::vector<EvidenceIndexSidecarEntry> raw_entries;
    raw_entries.reserve(evidence_index_count_);
    std::vector<std::byte> path_blob;
    for (const auto& slot : evidence_index_) {
        if (slot.key.node_id == kInvalidNodeId) {
            continue;
        }
        EvidenceKey key = slot.key;
        if (!normalize_evidence_key(&key) || !key.node_path_valid || !path_code_storage_valid(key.node_path)) {
            return false;
        }
        EvidenceIndexSidecarEntry raw_entry;
        raw_entry.node_id = key.node_id;
        raw_entry.sector = static_cast<std::int32_t>(key.sector);
        raw_entry.channel = static_cast<std::uint32_t>(key.channel);
        raw_entry.endpoint_source = static_cast<std::uint32_t>(key.endpoint_source);
        raw_entry.payload_kind = static_cast<std::uint32_t>(key.payload_kind);
        raw_entry.flags = (slot.entry.child_hull ? kEvidenceIndexFlagChildHull : 0u) |
                          (slot.entry.unavailable ? kEvidenceIndexFlagUnavailable : 0u);
        raw_entry.size = slot.entry.size;
        raw_entry.path_word_count = path_word_count_for_bits(key.node_path.bit_count);
        raw_entry.path_bit_count = static_cast<std::uint32_t>(key.node_path.bit_count);
        raw_entry.path_blob_offset = path_blob.size();
        raw_entry.offset = slot.entry.offset;
        raw_entry.generation = slot.entry.generation;
        raw_entry.checksum = slot.entry.checksum;
        raw_entries.push_back(raw_entry);
        const auto* path_bytes = reinterpret_cast<const std::byte*>(key.node_path.words.data());
        path_blob.insert(path_blob.end(), path_bytes, path_bytes + path_code_storage_bytes(raw_entry.path_word_count));
    }
    std::sort(raw_entries.begin(), raw_entries.end(), evidence_sidecar_entry_less);
    const EvidenceIndexSidecarHeader header{
        kEvidenceIndexSidecarMagic,
        kEvidenceIndexSidecarSchemaVersion,
        sizeof(EvidenceIndexSidecarHeader),
        evidence_file_size,
        static_cast<std::uint64_t>(raw_entries.size()),
        sizeof(EvidenceIndexSidecarHeader) + raw_entries.size() * sizeof(EvidenceIndexSidecarEntry),
    };
    out.write(reinterpret_cast<const char*>(&header), static_cast<std::streamsize>(sizeof(header)));
    for (const auto& raw_entry : raw_entries) {
        out.write(reinterpret_cast<const char*>(&raw_entry), static_cast<std::streamsize>(sizeof(raw_entry)));
    }
    if (!path_blob.empty()) {
        out.write(reinterpret_cast<const char*>(path_blob.data()), static_cast<std::streamsize>(path_blob.size()));
    }
    out.close();
    if (!static_cast<bool>(out) || !replace_file(tmp, path)) {
        return false;
    }
    evidence_index_sidecar_dirty_ = false;
    return true;
}

}  // namespace rbf::lect_database
