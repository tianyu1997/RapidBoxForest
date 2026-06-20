#include <rbf/lect_database/database.h>

#include "database_evidence_codec.h"

#include <algorithm>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

namespace rbf::lect_database {

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
    quantize_payload_outward(record.key.payload_kind, record.payload);
    record.checksum = payload_checksum(record.payload);
    // The text journal is a write-ahead log for crash recovery between
    // checkpoints. During bulk/streaming prewarm we skip it entirely: the final
    // checkpoint truncates the journal anyway and persists the authoritative
    // binary store, so per-record journaling is pure write amplification.
    const std::string journal_record =
        (bulk_prewarm_mode_ || streaming_prewarm_mode_)
            ? std::string()
            : ("evidence|" + serialize_evidence_record(record));
    const bool direct_evidence = !record.child_hull;
    const bool node_is_internal = !node_item->is_leaf();
    const NodeId parent_id = node_item->parent;
    const bool streaming_append_only = streaming_prewarm_mode_ &&
        !config_.propagate_parent_hulls &&
        streaming_resident_cap_ > 0 &&
        evidence_.size() >= streaming_resident_cap_;

    if (streaming_append_only) {
        if (!append_evidence_record_to_store(record)) {
            return false;
        }
    } else {
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
                        quantize_payload_outward(child_hull->key.payload_kind, child_hull->payload);
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
    key_template.node_path = {};
    key_template.node_path_valid = false;
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


}  // namespace rbf::lect_database
