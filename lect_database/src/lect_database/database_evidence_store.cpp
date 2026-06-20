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
#include <memory>
#include <optional>
#include <span>
#include <system_error>
#include <vector>

namespace rbf::lect_database {

namespace {

constexpr std::uint64_t kEvidenceAppendsPerFlush = 1024;

using database_file::evidence_index_path;
using database_file::evidence_path;
using database_file::replace_file;

}  // namespace

bool LectDatabase::load_evidence(std::string* reason) {
    evidence_.clear();
    clear_evidence_index();
    close_evidence_streams();
    evidence_append_offset_ = 0;
    evidence_appends_since_flush_ = 0;
    evidence_index_sidecar_dirty_ = false;
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
    if (binary_store) {
        evidence_append_offset_ = evidence_file_size;
    }
    if (!binary_store) {
        if (reason) *reason = "evidence store format is unsupported; rebuild the database";
        return false;
    }

    if (binary_store && load_evidence_index_sidecar(evidence_file_size)) {
        return true;
    }

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

        const auto path_bytes = path_code_storage_bytes(record_header.path_word_count);
        const auto payload_bytes = static_cast<std::uint64_t>(record_header.payload_count) * sizeof(std::uint16_t);
        const auto expected_record_size = sizeof(EvidenceStoreRecordHeader) + path_bytes + payload_bytes;
        if (record_header.record_size != expected_record_size ||
            record_header.record_size < sizeof(EvidenceStoreRecordHeader) ||
            offset + record_header.record_size > evidence_file_size) {
            if (reason) *reason = "evidence store record is malformed";
            return false;
        }

        EvidenceKey key;
        key.node_id = record_header.node_id;
        std::vector<std::byte> path_storage(static_cast<std::size_t>(path_bytes));
        if (path_bytes > 0) {
            input.read(reinterpret_cast<char*>(path_storage.data()), static_cast<std::streamsize>(path_bytes));
            if (!input) {
                if (reason) *reason = "evidence store path payload is truncated";
                return false;
            }
        }
        const auto path = parse_path_code_blob(path_storage, record_header.path_word_count, record_header.path_bit_count);
        if (!path) {
            if (reason) *reason = "evidence store path payload is malformed";
            return false;
        }
        key.node_path = *path;
        key.node_path_valid = true;
        key.sector = record_header.sector;
        key.channel = static_cast<EvidenceChannel>(record_header.channel);
        key.endpoint_source = static_cast<EndpointSource>(record_header.endpoint_source);
        key.payload_kind = static_cast<EvidencePayloadKind>(record_header.payload_kind);
        if (!normalize_evidence_key(&key)) {
            if (reason) *reason = "evidence store references an unknown node path";
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
        evidence_append_offset_ = sizeof(EvidenceStoreFileHeader);
        return true;
    }

    std::ifstream input(path, std::ios::binary);
    EvidenceStoreFileHeader header;
    if (!input || !read_evidence_store_file_header(input, &header)) {
        return false;
    }
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

void LectDatabase::remember_evidence_metadata(const EvidenceRecord& record) {
    EvidenceIndexEntry entry;
    if (const auto* existing = find_evidence_index(record.key)) {
        entry = *existing;
    }
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
    if (header.path_word_count > 0) {
        evidence_append_stream_.write(reinterpret_cast<const char*>(record.key.node_path.words.data()),
                                      static_cast<std::streamsize>(path_code_storage_bytes(header.path_word_count)));
    }
    if (!record.payload.empty()) {
        std::vector<std::uint16_t> halves(record.payload.size());
        for (std::size_t i = 0; i < record.payload.size(); ++i) {
            halves[i] = f16_from_f32_nearest(record.payload[i]);
        }
        evidence_append_stream_.write(reinterpret_cast<const char*>(halves.data()),
                                      static_cast<std::streamsize>(halves.size() * sizeof(std::uint16_t)));
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
    auto record = parse_binary_evidence_record(*bytes_view);
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
    // Prewarm defers the heavy incremental flush (index sidecar + manifest +
    // node pages) to the final checkpoint. Records are already appended to the
    // durable store and the read path flushes the append stream on demand, so
    // skipping the periodic full-sidecar rewrite keeps total store writes
    // O(records) instead of O(records^2 / kEvidenceAppendsPerFlush).
    if (bulk_prewarm_mode_ || streaming_prewarm_mode_) {
        return true;
    }
    if (evidence_appends_since_flush_ < kEvidenceAppendsPerFlush) {
        return true;
    }
    return flush_incremental_storage();
}

void LectDatabase::trim_evidence_cache() const {
    // Streaming prewarm: bound the resident cache to streaming_resident_cap_
    // records. The records are already in the append-only store, so eviction is
    // cheap -- flush the append stream so the bytes are durable, then drop
    // resident copies that have a committed on-disk index entry. The index
    // sidecar is written once at the final checkpoint (not here), so this is
    // O(evicted) with no full-store or sidecar rewrite. Any evicted child record
    // needed by the bottom-up parent sweep is reloaded on demand from the store.
    if (streaming_prewarm_mode_) {
        if (evidence_.size() <= streaming_resident_cap_) {
            return;
        }
        if (evidence_append_stream_.is_open()) {
            evidence_append_stream_.flush();
        }
        for (auto it = evidence_.begin();
             it != evidence_.end() && evidence_.size() > streaming_resident_cap_;) {
            const auto* index_entry = find_evidence_index(it->first);
            if (index_entry != nullptr && index_entry->size > 0) {
                it = evidence_.erase(it);
            } else {
                ++it;
            }
        }
        return;
    }
    // During bulk prewarm keep every materialized record resident so the store
    // is consolidated once at checkpoint instead of being fully rewritten each
    // time the resident set crosses kMaxResidentEvidenceRecords.
    const std::size_t cap = bulk_prewarm_mode_
        ? std::max<std::size_t>(bulk_prewarm_resident_cap_, kMaxResidentEvidenceRecords)
        : kMaxResidentEvidenceRecords;
    if (evidence_.size() <= cap) {
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


}  // namespace rbf::lect_database
