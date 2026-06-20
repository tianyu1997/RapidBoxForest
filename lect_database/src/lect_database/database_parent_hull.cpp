#include <rbf/lect_database/database.h>

#include "database_evidence_codec.h"

#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <utility>
#include <vector>

namespace rbf::lect_database {

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
        quantize_payload_outward(parent_record->key.payload_kind, parent_record->payload);
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

std::size_t LectDatabase::materialize_internal_parent_hulls_bottom_up(
    int deepest_depth,
    const EvidenceKey& key_template,
    const std::function<void(int depth, std::size_t built)>& layer_progress) {
    std::size_t built = 0;
    for (int depth = deepest_depth - 1; depth >= 0; --depth) {
        for (NodeId node_id : layer_nodes(depth)) {
            const auto parent_node = read_node(node_id);
            if (!parent_node || parent_node->is_leaf()) {
                continue;
            }
            auto parent_record = build_parent_hull_from_node(*parent_node, key_template);
            if (!parent_record) {
                // Children not (yet) materialized (e.g. sector-boundary straddle
                // leaf with no stored evidence); leave this ancestor uncached.
                continue;
            }
            parent_record->generation = generation_;
            quantize_payload_outward(parent_record->key.payload_kind, parent_record->payload);
            parent_record->checksum = payload_checksum(parent_record->payload);
            auto shared_parent_record = std::make_shared<EvidenceRecord>(std::move(*parent_record));
            auto [parent_it, parent_inserted] = evidence_.insert_or_assign(shared_parent_record->key,
                                                                           shared_parent_record);
            (void)parent_inserted;
            remember_evidence_metadata(*parent_it->second);
            if (!config_.open.read_only && !append_evidence_record_to_store(*parent_it->second)) {
                return built;
            }
            pending_changes_ = true;
            ++built;
        }
        if (layer_progress) {
            layer_progress(depth, built);
        }
    }
    return built;
}


}  // namespace rbf::lect_database
