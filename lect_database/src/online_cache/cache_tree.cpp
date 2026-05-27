#include <LECTDatabase/online_cache/cache_tree.h>

#include <algorithm>
#include <utility>

namespace rbf::lect_database {

namespace {

EvidenceKey canonicalize_evidence_key(const LectDatabase& database, EvidenceKey key) {
    if (!key.node_path_valid && valid_node_id(key.node_id)) {
        const auto topology = database.topology(key.node_id);
        if (valid_node_id(topology.id)) {
            key.node_path = topology.path;
            key.node_path_valid = true;
        }
    }
    return key;
}

}  // namespace

OnlineEnvelopeCacheTree::OnlineEnvelopeCacheTree(LectDatabase& database,
                                                 OnlineEnvelopeCacheConfig config)
    : database_(database), config_(config) {}

std::optional<std::vector<Interval>> OnlineEnvelopeCacheTree::node_intervals(NodeId node_id) const {
    return database_.node_box(node_id);
}

NodeTopology OnlineEnvelopeCacheTree::topology(NodeId node_id) const {
    return database_.topology(node_id);
}

bool OnlineEnvelopeCacheTree::contains_point(NodeId node_id,
                                             const Eigen::Ref<const Eigen::VectorXd>& point) const {
    const auto intervals = database_.node_box(node_id);
    if (!intervals || point.size() != static_cast<int>(intervals->size())) {
        return false;
    }
    for (int dim = 0; dim < point.size(); ++dim) {
        if (!(*intervals)[static_cast<std::size_t>(dim)].contains(point[dim])) {
            return false;
        }
    }
    return true;
}

bool OnlineEnvelopeCacheTree::is_leaf(NodeId node_id) const {
    return database_.topology(node_id).leaf;
}

int OnlineEnvelopeCacheTree::depth(NodeId node_id) const {
    return database_.topology(node_id).depth;
}

std::pair<NodeId, NodeId> OnlineEnvelopeCacheTree::split_leaf(NodeId node_id) {
    stats_.split_requests += 1;
    const auto topology_before = database_.topology(node_id);
    if (!valid_node_id(topology_before.id)) {
        stats_.split_failures += 1;
        return {kInvalidNodeId, kInvalidNodeId};
    }
    if (!topology_before.leaf) {
        return {topology_before.left, topology_before.right};
    }
    const auto children = database_.split_leaf(node_id);
    if (!valid_node_id(children.first) || !valid_node_id(children.second)) {
        stats_.split_failures += 1;
    } else {
        stats_.parent_union_pending += 1;
    }
    return children;
}

std::optional<EvidenceRecord> OnlineEnvelopeCacheTree::evidence(EvidenceKey key) {
    key = canonicalize_evidence_key(database_, std::move(key));
    auto cache_it = payload_cache_.find(key);
    if (cache_it != payload_cache_.end()) {
        stats_.cache_hits += 1;
        touch(cache_it->second);
        return cache_it->second.record;
    }

    stats_.cache_misses += 1;
    auto stored = database_.evidence(key);
    if (!stored) {
        return std::nullopt;
    }

    EvidenceRecord record;
    record.key = stored->key;
    record.child_hull = stored->child_hull;
    record.unavailable = stored->unavailable;
    record.generation = stored->generation;
    record.checksum = stored->checksum;
    record.payload.assign(stored->payload.begin(), stored->payload.end());
    stats_.database_loads += 1;
    insert_cache_record(record);
    return record;
}

bool OnlineEnvelopeCacheTree::put_evidence(EvidenceRecord record, bool allow_backfill) {
    record.key = canonicalize_evidence_key(database_, std::move(record.key));
    if (!insert_cache_record(record)) {
        return false;
    }
    stats_.online_inserts += 1;
    if (allow_backfill && config_.allow_database_backfill && !database_.read_only()) {
        auto writeback = payload_cache_.find(record.key);
        if (writeback != payload_cache_.end() && database_.put_evidence(writeback->second.record)) {
            stats_.database_writes += 1;
        }
    }
    return true;
}

bool OnlineEnvelopeCacheTree::has_cached_payload(const EvidenceKey& key) const {
    return payload_cache_.find(canonicalize_evidence_key(database_, key)) != payload_cache_.end();
}

void OnlineEnvelopeCacheTree::clear_payloads() {
    payload_cache_.clear();
    payload_bytes_ = 0;
}

std::size_t OnlineEnvelopeCacheTree::payload_bytes(const EvidenceRecord& record) noexcept {
    return record.payload.size() * sizeof(float);
}

void OnlineEnvelopeCacheTree::touch(CacheEntry& entry) noexcept {
    entry.last_access = ++access_clock_;
}

bool OnlineEnvelopeCacheTree::insert_cache_record(EvidenceRecord record) {
    record.key = canonicalize_evidence_key(database_, std::move(record.key));
    const std::size_t bytes = payload_bytes(record);
    if (config_.max_payload_bytes > 0 && bytes > config_.max_payload_bytes) {
        stats_.memory_limit_rejections += 1;
        return false;
    }

    auto existing = payload_cache_.find(record.key);
    if (existing != payload_cache_.end()) {
        payload_bytes_ -= existing->second.bytes;
        existing->second.record = std::move(record);
        existing->second.bytes = bytes;
        touch(existing->second);
        payload_bytes_ += bytes;
        evict_if_needed();
        return true;
    }

    CacheEntry entry;
    entry.record = std::move(record);
    entry.bytes = bytes;
    touch(entry);
    payload_bytes_ += bytes;
    payload_cache_.emplace(entry.record.key, std::move(entry));
    evict_if_needed();
    return true;
}

void OnlineEnvelopeCacheTree::evict_if_needed() {
    if (config_.max_payload_bytes == 0) {
        return;
    }
    while (payload_bytes_ > config_.max_payload_bytes && !payload_cache_.empty()) {
        auto victim = std::min_element(
            payload_cache_.begin(),
            payload_cache_.end(),
            [](const auto& lhs, const auto& rhs) {
                return lhs.second.last_access < rhs.second.last_access;
            });
        if (victim == payload_cache_.end()) {
            break;
        }
        payload_bytes_ -= victim->second.bytes;
        payload_cache_.erase(victim);
        stats_.lru_evictions += 1;
    }
}

}  // namespace rbf::lect_database
