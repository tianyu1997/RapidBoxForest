#pragma once

#include <rbf/core/types.h>
#include <rbf/envelope/endpoint_source.h>

#include <cstdint>
#include <filesystem>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace rbf::lect_database {

using NodeId = std::uint64_t;
using SectorId = int;

inline constexpr NodeId kInvalidNodeId = std::numeric_limits<NodeId>::max();
inline constexpr SectorId kPrimarySector = 0;
inline constexpr std::uint32_t kLectDatabaseSchemaVersion = 1;

bool valid_node_id(NodeId id) noexcept;

std::uint64_t stable_hash_append(std::uint64_t hash, const void* data, std::size_t size);
std::uint64_t stable_hash_append(std::uint64_t hash, std::string_view text);
std::uint64_t stable_hash(std::string_view text);
std::uint64_t fingerprint_intervals(const std::vector<Interval>& intervals);
std::string interval_descriptor(const std::vector<Interval>& intervals);

struct PathCode {
    std::vector<std::uint64_t> words;
    int bit_count = 0;

    bool empty() const noexcept { return bit_count == 0; }
    void push_child(bool right_child);
    bool bit(int index) const;
    bool is_prefix_of(const PathCode& other) const;
    int common_prefix_bits(const PathCode& other) const;
};

struct NodeRecord {
    NodeId id = kInvalidNodeId;
    NodeId parent = kInvalidNodeId;
    NodeId left = kInvalidNodeId;
    NodeId right = kInvalidNodeId;
    int depth = 0;
    int split_dim = -1;
    double split_value = 0.0;
    PathCode path;
    std::uint64_t generation = 0;
    std::uint64_t page_id = 0;
    bool dirty = false;
    bool evidence_dirty = false;

    bool is_leaf() const noexcept {
        return left == kInvalidNodeId && right == kInvalidNodeId;
    }
};

struct NodeTopology {
    NodeId id = kInvalidNodeId;
    NodeId parent = kInvalidNodeId;
    NodeId left = kInvalidNodeId;
    NodeId right = kInvalidNodeId;
    NodeId sibling = kInvalidNodeId;
    int depth = 0;
    int split_dim = -1;
    double split_value = 0.0;
    bool leaf = true;
    PathCode path;
};

struct BoxKey {
    std::vector<Interval> intervals;
    std::uint64_t root_domain_fingerprint = 0;
    std::uint64_t split_policy_hash = 0;
    double tolerance = 1e-12;
};

struct BoxLookupResult {
    bool found = false;
    NodeId node_id = kInvalidNodeId;
    std::string reason;
};

enum class RangeQueryMode : std::uint8_t {
    Containing = 0,
    ContainedBy = 1,
    Intersecting = 2,
    CoveringFrontier = 3,
};

enum class EvidenceChannel : std::uint8_t {
    Safe = 0,
    Rapid = 1,
};

enum class EvidencePayloadKind : std::uint8_t {
    EndpointEnvelope = 0,
    LinkEnvelope = 1,
};

struct EvidenceKey {
    NodeId node_id = kInvalidNodeId;
    SectorId sector = kPrimarySector;
    EvidenceChannel channel = EvidenceChannel::Safe;
    EndpointSource endpoint_source = EndpointSource::IFK;
    EvidencePayloadKind payload_kind = EvidencePayloadKind::EndpointEnvelope;

    bool operator==(const EvidenceKey& other) const noexcept;
};

struct EvidenceKeyHash {
    std::size_t operator()(const EvidenceKey& key) const noexcept;
};

struct EvidenceRecord {
    EvidenceKey key;
    bool child_hull = false;
    bool unavailable = false;
    std::uint64_t generation = 0;
    std::uint64_t checksum = 0;
    std::vector<float> payload;
};

struct EvidenceRecordView {
    EvidenceKey key;
    bool child_hull = false;
    bool unavailable = false;
    std::uint64_t generation = 0;
    std::uint64_t checksum = 0;
    std::span<const float> payload;
    std::shared_ptr<const EvidenceRecord> storage;
};

struct LectDatabaseStats {
    std::uint64_t node_page_reads = 0;
    std::uint64_t node_page_writes = 0;
    std::uint64_t node_page_cache_hits = 0;
    std::uint64_t node_page_cache_misses = 0;
    std::uint64_t node_page_evictions = 0;
    std::uint64_t node_page_dirty_evictions = 0;
    std::uint64_t node_page_dirty_flushes = 0;
    std::uint64_t resident_node_pages = 0;
    std::uint64_t max_resident_node_pages = 0;
    std::uint64_t evidence_reads = 0;
    std::uint64_t evidence_writes = 0;
    std::uint64_t journal_transactions = 0;
    std::uint64_t range_nodes_visited = 0;
    std::uint64_t query_path_cache_hits = 0;
    std::uint64_t query_path_cache_misses = 0;
};

struct VerificationResult {
    bool ok = true;
    std::vector<std::string> errors;
    std::vector<std::string> warnings;

    void add_error(std::string message) {
        ok = false;
        errors.push_back(std::move(message));
    }
    void add_warning(std::string message) {
        warnings.push_back(std::move(message));
    }
};

}  // namespace rbf::lect_database
