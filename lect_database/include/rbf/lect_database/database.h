#pragma once

#include <rbf/lect_database/identity.h>

#include <algorithm>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace rbf::lect_database {

// Upper bound on the resident evidence cache (records). Defined here so the
// inline streaming/bulk prewarm setters below can clamp against it; the
// streaming store reloads any evicted child records from the durable store.
inline constexpr std::size_t kMaxResidentEvidenceRecords = 8192;

struct LectDbOpenOptions {
    bool read_only = false;
    bool create_if_missing = true;
    bool verify_identity = true;
    bool replay_journal = true;
    // When true, open() loads only the manifest and (optionally) verifies the
    // requested identity, then returns without loading node pages, evidence, or
    // rebuilding indices. Intended for cheap existence/identity probes where the
    // database handle is discarded immediately (e.g. external-evidence snapshot
    // verification). A metadata-only handle must not be used for queries.
    bool metadata_only = false;
};

struct LectDatabaseConfig {
    std::filesystem::path path;
    LectDatabaseIdentity identity;
    std::vector<Interval> root_intervals;
    SplitPolicyDescriptor split_policy;
    LectDbOpenOptions open;
    bool propagate_parent_hulls = true;
    bool defer_parent_hull_writes = false;
    std::uint32_t page_size_bytes = 64u * 1024u;
    std::uint32_t max_resident_pages = 256u;
    int root_depth = 0;
    int max_tree_depth = 64;
    double exact_box_tolerance = 1e-12;
};

struct LectDbTransaction {
    std::uint64_t generation = 0;
    std::vector<std::string> records;
    bool committed = false;
};

class LectDatabase;

class LectDbQuerySession {
public:
    LectDbQuerySession() = default;
    explicit LectDbQuerySession(const LectDatabase& database);

    std::optional<std::vector<Interval>> node_box(NodeId node_id);
    BoxLookupResult box_to_node_exact(const BoxKey& box);
    const LectDatabaseStats& stats() const noexcept { return stats_; }

private:
    const LectDatabase* database_ = nullptr;
    NodeId cached_node_ = kInvalidNodeId;
    std::vector<Interval> cached_box_;
    LectDatabaseStats stats_;
};

class LectDatabase {
public:
    LectDatabase() = default;
    LectDatabase(const LectDatabase&) = delete;
    LectDatabase& operator=(const LectDatabase&) = delete;
    LectDatabase(LectDatabase&&) noexcept;
    LectDatabase& operator=(LectDatabase&&) noexcept;
    ~LectDatabase();

    static std::optional<LectDatabase> open_existing(const std::filesystem::path& path,
                                                     bool read_only = true,
                                                     std::string* reason = nullptr);

    bool open(LectDatabaseConfig config, std::string* reason = nullptr);
    bool is_open() const noexcept { return opened_; }
    bool read_only() const noexcept { return config_.open.read_only; }

    const LectDatabaseIdentity& identity() const noexcept { return identity_; }
    const SplitPolicyDescriptor& split_policy_descriptor() const noexcept { return split_policy_.descriptor(); }
    const std::vector<Interval>& root_intervals() const noexcept { return root_intervals_; }
    const LectDatabaseStats& stats() const noexcept { return stats_; }
    std::uint64_t generation() const noexcept { return generation_; }
    std::size_t node_count() const noexcept { return static_cast<std::size_t>(node_count_); }
    std::size_t evidence_count() const noexcept { return std::max(evidence_.size(), evidence_index_count_); }
    bool propagate_parent_hulls_enabled() const noexcept { return config_.propagate_parent_hulls; }
    // Runtime toggle used by LECT prewarm: per-leaf auto-propagation is disabled
    // during the leaf-materialization loop and replaced by a single bottom-up
    // sweep (materialize_internal_parent_hulls_bottom_up), trading scattered
    // O(leaves*depth) walk-ups for one O(leaves) pass.
    void set_propagate_parent_hulls(bool enabled) noexcept { config_.propagate_parent_hulls = enabled; }
    // Runtime toggle used by LECT bulk prewarm. While enabled the database keeps
    // every materialized record resident (up to `expected_records`) and skips the
    // per-record text journal, so the binary store is consolidated exactly once
    // at the final checkpoint instead of being rewritten every time the resident
    // set exceeds kMaxResidentEvidenceRecords. checkpoint() truncates the journal
    // anyway, so intra-prewarm journaling is pure write amplification.
    void set_bulk_prewarm_mode(bool enabled, std::size_t expected_records = 0) noexcept {
        bulk_prewarm_mode_ = enabled;
        bulk_prewarm_resident_cap_ = enabled ? expected_records : 0;
        if (enabled && expected_records > 0) {
            try {
                evidence_.reserve(expected_records);
                reserve_evidence_index(expected_records);
            } catch (...) {
                // Pre-reservation is an optimization only; normal growth remains valid.
            }
        }
    }
    bool bulk_prewarm_mode_enabled() const noexcept { return bulk_prewarm_mode_; }
    // Runtime toggle used by LECT streaming prewarm for depths whose full
    // resident set would exceed RAM (e.g. D25 ~62M records). Records are already
    // appended to the durable binary store as they are created; while streaming
    // is enabled the resident `evidence_` cache is bounded to `resident_cap`
    // records (evicting committed records once the cap is crossed) and the
    // index sidecar / manifest are written exactly once at the final
    // checkpoint instead of being rewritten on every flush. This keeps peak RAM
    // O(resident_cap) and total store writes O(records); the bottom-up parent
    // sweep transparently reloads any evicted child records from the store.
    void set_streaming_prewarm_mode(bool enabled, std::size_t resident_cap) noexcept {
        streaming_prewarm_mode_ = enabled;
        streaming_resident_cap_ = enabled
            ? std::max<std::size_t>(resident_cap, std::size_t{8192})
            : 0;
    }
    bool streaming_prewarm_mode_enabled() const noexcept { return streaming_prewarm_mode_; }
    int max_tree_depth() const noexcept { return config_.max_tree_depth; }
    NodeId root_node() const noexcept { return node_count_ == 0 ? kInvalidNodeId : 0; }

    BoxKey make_box_key(std::vector<Interval> intervals) const;
    std::optional<NodeRecord> node(NodeId node_id) const;
    NodeTopology topology(NodeId node_id) const;
    NodeId parent(NodeId node_id) const;
    std::pair<NodeId, NodeId> children(NodeId node_id) const;
    NodeId sibling(NodeId node_id) const;
    bool is_ancestor(NodeId ancestor, NodeId node_id) const;
    NodeId lca(NodeId lhs, NodeId rhs) const;
    std::vector<NodeId> node_ids() const;
    std::vector<NodeId> layer_nodes(int depth) const;

    std::optional<std::vector<Interval>> node_box(NodeId node_id) const;
    BoxLookupResult box_to_node_exact(const BoxKey& box) const;
    std::vector<NodeId> range_query(const std::vector<Interval>& box,
                                    RangeQueryMode mode,
                                    LectDatabaseStats* stats = nullptr) const;

    std::pair<NodeId, NodeId> split_leaf(NodeId node_id);
    std::pair<NodeId, NodeId> split_leaf(NodeId node_id, int split_dim, double split_value);
    bool ensure_depth(int target_depth);
    BoxLookupResult split_to_box(const BoxKey& box, int max_depth = 128);

    bool put_evidence(EvidenceRecord record);
    // Hardcoded LECT-prewarm strategy: after the leaf layer has been
    // FK-materialized, derive every internal-node envelope purely from its two
    // children via the cheap conservative AABB union (build_parent_hull_from_node),
    // bottom-up from `deepest_depth-1` to the root. Internal nodes are NEVER
    // FK-materialized. Children's narrower interval domains make the union a
    // strictly tighter (and far cheaper) parent envelope than direct parent FK.
    // Skips nodes whose children lack stored evidence (e.g. sector-boundary
    // straddle leaves) so the result stays sound; those few ancestors fall back
    // to query-time FK. Stores directly (no per-node re-propagation). Returns the
    // number of parent hulls written.
    // `layer_progress`, when set, is invoked once per processed depth layer with
    // (remaining_layers_including_current, depth, cumulative_built) so callers can
    // render a progress bar / ETA for the bottom-up sweep.
    std::size_t materialize_internal_parent_hulls_bottom_up(
        int deepest_depth,
        const EvidenceKey& key_template,
        const std::function<void(int depth, std::size_t built)>& layer_progress = {});
    std::optional<EvidenceRecordView> evidence(const EvidenceKey& key) const;
    std::optional<std::uint64_t> evidence_offset(const EvidenceKey& key) const;
    bool has_evidence(const EvidenceKey& key) const;
    std::vector<EvidenceRecord> evidence_records() const;
    std::optional<EvidenceRecordView> endpoint_for_box_exact(const BoxKey& box,
                                                             EvidenceKey key_template) const;
    std::size_t delete_node_payloads(NodeId node_id);

    LectDbQuerySession make_query_session() const;
    VerificationResult verify(bool strict = true) const;
    bool checkpoint();
    bool compact();
    std::string inspect_summary() const;

private:
    friend class LectDbQuerySession;

    struct EvidenceMappedFile;
    struct EvidenceIndexEntry;

    struct NodePage {
        std::uint64_t page_id = 0;
        NodeId first_node_id = 0;
        std::vector<NodeRecord> rows;
        bool dirty = false;
        std::uint64_t last_access = 0;
    };

    NodeId append_child(NodeId parent_id, bool right_child, int depth, PathCode path);
    bool has_node(NodeId node_id) const noexcept;
    std::size_t rows_per_node_page() const noexcept;
    std::uint64_t page_id_for_node(NodeId node_id) const noexcept;
    NodeId first_node_id_for_page(std::uint64_t page_id) const noexcept;
    std::size_t node_offset_in_page(NodeId node_id) const noexcept;
    NodePage& touch_node_page(std::uint64_t page_id) const;
    bool flush_node_page(std::uint64_t page_id) const;
    bool flush_all_node_pages() const;
    void evict_node_pages_if_needed() const;
    void update_resident_page_stats() const noexcept;
    std::optional<NodeRecord> read_node(NodeId node_id) const;
    NodeRecord* mutable_node(NodeId node_id);
    bool write_node_record(NodeRecord record);
    std::vector<Interval> node_box_unchecked(NodeId node_id) const;
    std::vector<Interval> node_box_from_path(const PathCode& path) const;
    bool intervals_equal(const std::vector<Interval>& lhs,
                         const std::vector<Interval>& rhs,
                         double tolerance) const;
    bool interval_contains(const Interval& outer, const Interval& inner, double tolerance) const;
    bool box_contains(const std::vector<Interval>& outer,
                      const std::vector<Interval>& inner,
                      double tolerance) const;
    bool box_overlaps(const std::vector<Interval>& lhs,
                      const std::vector<Interval>& rhs,
                      double tolerance) const;
    void rebuild_layer_index();
    void assign_page_ids();
    bool load_manifest(std::string* reason);
    bool save_manifest() const;
    bool load_nodes(std::string* reason);
    bool save_nodes() const;
    bool load_evidence(std::string* reason);
    bool save_evidence() const;
    bool load_evidence_index_sidecar(std::uint64_t evidence_file_size);
    bool save_evidence_index_sidecar(std::uint64_t evidence_file_size) const;
    void clear_evidence_index() noexcept;
    void reserve_evidence_index(std::size_t item_count);
    bool ensure_evidence_mapped_file() const;
    const EvidenceIndexEntry* find_evidence_index(const EvidenceKey& key) const;
    EvidenceIndexEntry* find_evidence_index(const EvidenceKey& key);
    void upsert_evidence_index(const EvidenceKey& key, EvidenceIndexEntry entry);
    bool scan_binary_evidence_store(std::ifstream& input,
                                    std::uint64_t evidence_file_size,
                                    std::string* reason);
    bool ensure_binary_evidence_store_file() const;
    std::optional<std::span<const std::byte>> load_evidence_bytes(std::uint64_t offset,
                                                                  std::uint32_t size) const;
    void prefetch_indexed_evidence_ranges() const;
    void remember_evidence_metadata(const EvidenceRecord& record);
    bool append_evidence_record_to_store(const EvidenceRecord& record);
    std::shared_ptr<const EvidenceRecord> load_indexed_evidence(const EvidenceKey& key) const;
    bool ensure_all_evidence_loaded() const;
    bool ensure_evidence_append_stream() const;
    void close_evidence_streams() const;
    bool flush_incremental_storage() const;
    bool maybe_flush_incremental_storage() const;
    void trim_evidence_cache() const;
    void replay_journal();
    bool ensure_journal_append_stream();
    void close_journal_append_stream();
    void append_committed_transaction(const LectDbTransaction& transaction);
    bool propagate_parent_hulls(NodeId node_id, EvidenceKey key_template);
    bool propagate_parent_hulls_from(NodeId parent_id,
                                     EvidenceKey key_template,
                                     std::shared_ptr<const EvidenceRecord> child_record = nullptr);
    bool drain_deferred_parent_hulls();
    std::optional<EvidenceRecord> build_parent_hull_from_child(const NodeRecord& parent_node,
                                                               const EvidenceRecord& child_record,
                                                               const EvidenceKey& key_template) const;
    std::optional<EvidenceRecord> build_parent_hull_from_node(const NodeRecord& parent_node,
                                                              const EvidenceKey& key_template) const;
    std::optional<EvidenceRecord> build_parent_hull(NodeId parent_id, const EvidenceKey& key_template) const;
    bool normalize_evidence_key(EvidenceKey* key) const;
    EvidenceKey evidence_key_for_node(NodeId node_id, const EvidenceKey* key_template = nullptr) const;
    bool remember_node_record(const NodeRecord& record);
    NodeId allocate_node_id();
    bool remember_node_id(NodeId node_id);
    std::vector<NodeId> sorted_node_ids() const;

    struct EvidenceIndexEntry {
        std::uint64_t offset = 0;
        std::uint32_t size = 0;
        bool child_hull = false;
        bool unavailable = false;
        std::uint64_t generation = 0;
        std::uint64_t checksum = 0;
    };

    struct EvidenceIndexRecord {
        EvidenceKey key;
        EvidenceIndexEntry entry;
    };

    struct DeferredParentHullWrite {
        EvidenceKey key;
    };

    bool opened_ = false;
    LectDatabaseConfig config_;
    LectDatabaseIdentity identity_;
    std::vector<Interval> root_intervals_;
    SplitPolicy split_policy_;
    NodeId node_count_ = 0;
    NodeId max_node_id_ = 0;
    NodeId next_node_id_ = 0;
    std::unordered_set<NodeId> node_ids_;
    std::unordered_map<PathCode, NodeId, PathCodeHash> node_path_index_;
    mutable std::unordered_map<std::uint64_t, NodePage> node_pages_;
    mutable std::uint64_t node_page_clock_ = 0;
    std::unordered_map<int, std::vector<NodeId>> layer_index_;
    mutable std::unordered_map<EvidenceKey, std::shared_ptr<const EvidenceRecord>, EvidenceKeyHash> evidence_;
    std::vector<EvidenceIndexRecord> evidence_index_;
    std::size_t evidence_index_count_ = 0;
    mutable std::shared_ptr<EvidenceMappedFile> evidence_mapped_file_;
    mutable bool evidence_mapping_stale_ = false;
    mutable std::vector<std::byte> evidence_read_buffer_;
    mutable std::ofstream evidence_append_stream_;
    mutable std::uint64_t evidence_append_offset_ = 0;
    mutable std::uint64_t evidence_appends_since_flush_ = 0;
    mutable bool evidence_index_sidecar_dirty_ = false;
    std::ofstream journal_append_stream_;
    std::vector<DeferredParentHullWrite> deferred_parent_hull_writes_;
    std::uint64_t generation_ = 0;
    bool pending_changes_ = false;
    bool bulk_prewarm_mode_ = false;
    std::size_t bulk_prewarm_resident_cap_ = 0;
    bool streaming_prewarm_mode_ = false;
    std::size_t streaming_resident_cap_ = 0;
    mutable LectDatabaseStats stats_;
};

}  // namespace rbf::lect_database
