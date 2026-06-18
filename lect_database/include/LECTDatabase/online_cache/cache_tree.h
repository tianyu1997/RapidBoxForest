#pragma once

#include <rbf/lect_database.h>

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <unordered_map>
#include <vector>

namespace rbf::lect_database {

struct OnlineEnvelopeCacheConfig {
    std::size_t max_nodes = 0;
    std::size_t max_payload_bytes = 64u * 1024u * 1024u;
    bool allow_database_backfill = true;
};

struct OnlineEnvelopeCacheStats {
    std::uint64_t cache_hits = 0;
    std::uint64_t cache_misses = 0;
    std::uint64_t database_loads = 0;
    std::uint64_t online_inserts = 0;
    std::uint64_t database_writes = 0;
    std::uint64_t lru_evictions = 0;
    std::uint64_t split_requests = 0;
    std::uint64_t split_failures = 0;
    std::uint64_t parent_union_pending = 0;
    std::uint64_t memory_limit_rejections = 0;
};

class OnlineEnvelopeCacheTree {
public:
    explicit OnlineEnvelopeCacheTree(LectDatabase& database,
                                     OnlineEnvelopeCacheConfig config = {});

    LectDatabase& database() noexcept { return database_; }
    const LectDatabase& database() const noexcept { return database_; }
    const OnlineEnvelopeCacheConfig& config() const noexcept { return config_; }
    const OnlineEnvelopeCacheStats& stats() const noexcept { return stats_; }

    NodeId root_node() const noexcept { return database_.root_node(); }
    const std::vector<Interval>& root_intervals() const noexcept { return database_.root_intervals(); }
    const SplitPolicyDescriptor& split_policy_descriptor() const noexcept {
        return database_.split_policy_descriptor();
    }

    std::optional<std::vector<Interval>> node_intervals(NodeId node_id) const;
    NodeTopology topology(NodeId node_id) const;
    bool contains_point(NodeId node_id, const Eigen::Ref<const Eigen::VectorXd>& point) const;
    bool is_leaf(NodeId node_id) const;
    int depth(NodeId node_id) const;
    std::pair<NodeId, NodeId> split_leaf(NodeId node_id);

    std::optional<EvidenceRecord> evidence(EvidenceKey key,
                                           const std::vector<Interval>* exact_intervals = nullptr,
                                           const LectExternalEvidenceSource* external_evidence_source = nullptr,
                                           bool* reused_external_evidence = nullptr);
    bool put_evidence(EvidenceRecord record, bool allow_backfill = true);
    bool has_cached_payload(const EvidenceKey& key) const;
    bool flush_payloads_to_database();

    void clear_payloads();
    std::size_t memory_used_bytes() const noexcept { return payload_bytes_; }

private:
    struct CacheEntry {
        EvidenceRecord record;
        std::uint64_t last_access = 0;
        std::size_t bytes = 0;
    };

    static std::size_t payload_bytes(const EvidenceRecord& record) noexcept;
    void touch(CacheEntry& entry) noexcept;
    bool insert_cache_record(EvidenceRecord record);
    void evict_if_needed();

    LectDatabase& database_;
    OnlineEnvelopeCacheConfig config_;
    OnlineEnvelopeCacheStats stats_;
    std::unordered_map<EvidenceKey, CacheEntry, EvidenceKeyHash> payload_cache_;
    std::size_t payload_bytes_ = 0;
    std::uint64_t access_clock_ = 0;
};

}  // namespace rbf::lect_database
