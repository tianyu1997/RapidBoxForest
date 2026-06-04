#pragma once

#include <LECTDatabase/online_cache.h>
#include <LECTDatabase/sbf/scene.h>

#include <rbf/lect_database.h>
#include <rbf/lect_database/evidence_source.h>

#include <link_interval_envelope/endpoint.h>
#include <link_interval_envelope/envelope.h>

#include <Eigen/Core>

#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <span>
#include <unordered_map>
#include <vector>

namespace rbf {

using OracleNodeId = std::int64_t;
inline constexpr OracleNodeId kInvalidOracleNodeId = -1;

enum class BoxValidation : std::uint8_t {
    Free = 0,
    Occupied = 1,
    Unknown = 2,
};

enum class OracleValidationMode : std::uint8_t {
    Strict = 0,
    CoverageHeuristic = 1,
};

enum class BoxCommitPolicy : std::uint8_t {
    CommitCertifiedOnly = 0,
    CommitProvisionalAllowed = 1,
    AuditBeforeCommit = 2,
};

struct BestTightenOptions {
    bool depth_synchronous = true;
    bool prefer_sector_boundary = true;
    bool use_minimax = true;
    int max_candidate_dim = -1;
    double min_candidate_width = 0.0;
    double width_penalty = 0.0;
    bool shape_balancing = true;
    double max_child_aspect = 64.0;
    double min_split_width_fraction = 0.05;
    double shape_weight = 0.25;
    double balance_weight = 0.05;
    double relative_gain_weight = 0.10;
    double widest_tiebreak_weight = 0.02;
    bool recent_dim_cooling = true;
    int recent_dim_window = 6;
    double recent_dim_weight = 0.04;
    double recent_dim_shape_aspect_trigger = 16.0;
    // General per-dimension mask. Empty => all dims allowed. Otherwise an entry
    // of 0 forbids splitting that dim (e.g. kinematically inert dims that never
    // tighten the endpoint envelope). More general than max_candidate_dim since
    // masked dims need not be contiguous at the top. Precomputed once per robot.
    std::vector<int> dim_mask;
    // L2 soft dim-priority bias. Per-dim weights (typically per-robot envelope
    // sensitivity, larger => more useful to split). Empty => no bias. The term
    // dim_priority_weight * dim_priority_weights[dim] is subtracted from the
    // (minimized) balanced score, gently preferring high-sensitivity dims.
    std::vector<double> dim_priority_weights;
    double dim_priority_weight = 0.0;
};

struct OracleSplitOptions {
    bool use_best_tighten = true;
    BestTightenOptions best_tighten;
};

struct SplitNodeResult {
    bool split = false;
    OracleNodeId node = kInvalidOracleNodeId;
    OracleNodeId left = kInvalidOracleNodeId;
    OracleNodeId right = kInvalidOracleNodeId;
    int split_dim = -1;
    double split_value = 0.0;
};

struct OracleValidationConfig {
    OracleValidationMode mode = OracleValidationMode::Strict;
    bool accept_unsafe_free = false;
    bool enable_validation_cache = true;
    int validation_cache_max_entries = 4096;
    bool enable_endpoint_evidence_cache = true;
    bool store_endpoint_evidence_cache = true;
    double endpoint_cache_min_effective_width = 0.0;
    bool external_evidence_materialization = true;
    bool external_evidence_scoring = true;
    bool external_evidence_backfill_active = true;
    bool stateless_materialization_context = false;
    // When true, worker oracles share a thread-safe, interval-keyed endpoint
    // cache spawned from the master so concurrent build tasks reuse endpoints
    // computed by sibling tasks (mirroring single-thread cross-query reuse).
    bool enable_worker_shared_endpoint_cache = true;
    // Memory bounds for the shared endpoint cache. 0 means unbounded for the
    // corresponding dimension. Defaults cap the cache so multi-query / deep
    // builds cannot grow it without limit (OOM guard).
    std::size_t shared_endpoint_cache_max_entries = 200000;
    std::size_t shared_endpoint_cache_max_bytes = 512ull * 1024ull * 1024ull;
};

struct OracleValidationDetail {
    OracleNodeId node = kInvalidOracleNodeId;
    int depth = 0;
    OracleValidationMode mode = OracleValidationMode::Strict;
    BoxValidation validation = BoxValidation::Unknown;
    BoxSafetyStatus safety_status = BoxSafetyStatus::Unknown;
    bool collision_possible = true;
    bool strict_audit_required = false;
    EndpointSource endpoint_source = EndpointSource::IFK;
    bool endpoint_is_safe = false;
    EndpointSafetyLevel endpoint_safety_level = EndpointSafetyLevel::UnsafeHeuristic;
    bool materialized = false;
    int changed_dim = -1;
    bool used_incremental_fk = false;
    bool used_source_incremental_state = false;
    bool reused_fk = false;
    bool reused_endpoint_cache = false;
    bool reused_external_evidence = false;
    double endpoint_time_us = 0.0;
    double envelope_time_us = 0.0;
    int candidate_dirty_count = 0;
    int predh_rebuild_count = 0;
    bool aabb_overlap = false;
};

struct OracleCounters {
    int node_validations = 0;
    int interval_validations = 0;
    int certified_free = 0;
    int provisional_free = 0;
    int collision_possible = 0;
    int unsafe_free_rejected = 0;
    int validation_cache_hits = 0;
    int validation_cache_misses = 0;
    int materializations = 0;
    int materialization_stored_endpoint = 0;
    int materialization_skipped_endpoint_cache = 0;
    double materialization_endpoint_time_us = 0.0;
    double materialization_endpoint_wall_time_us = 0.0;
    double materialization_envelope_time_us = 0.0;
    double validate_node_total_time_us = 0.0;
    double validate_node_preamble_time_us = 0.0;
    double validate_node_endpoint_path_time_us = 0.0;
    double validate_node_classify_time_us = 0.0;
    double validate_node_overhead_time_us = 0.0;
    double materialization_cache_lookup_time_us = 0.0;
    double materialization_cache_read_time_us = 0.0;
    double materialization_external_lookup_time_us = 0.0;
    double materialization_external_read_time_us = 0.0;
    double materialization_envelope_compute_time_us = 0.0;
    double materialization_envelope_read_time_us = 0.0;
    double materialization_envelope_collision_time_us = 0.0;
    int materialization_incremental_fk = 0;
    int materialization_source_incremental_state = 0;
    int materialization_reused_fk = 0;
    int materialization_reused_endpoint_cache = 0;
    int materialization_reused_external_evidence = 0;
    int materialization_reused_shared_endpoint_cache = 0;
    int materialization_stored_shared_endpoint_cache = 0;
    int materialization_reused_cached_envelope = 0;
    int materialization_candidate_dirty_count = 0;
    int materialization_predh_rebuild_count = 0;
    int scoring_evaluations = 0;
    int scoring_changed_dim_inferred = 0;
    int scoring_incremental_fk = 0;
    int scoring_source_incremental_state = 0;
    int scoring_reused_fk = 0;
    int scoring_reused_endpoint_cache = 0;
    int scoring_reused_external_evidence = 0;
    double scoring_endpoint_time_us = 0.0;
    double scoring_envelope_time_us = 0.0;
    int scoring_candidate_dirty_count = 0;
    int scoring_predh_rebuild_count = 0;
    int envelope_collision_queries = 0;
    int envelope_collision_free = 0;
    int envelope_collision_maybe = 0;
    std::int64_t envelope_collision_envelope_aabb_tests = 0;
    std::int64_t envelope_collision_envelope_aabb_rejects = 0;
    std::int64_t envelope_collision_link_union_aabb_tests = 0;
    std::int64_t envelope_collision_link_union_aabb_rejects = 0;
    std::int64_t envelope_collision_link_aabb_tests = 0;
    std::int64_t envelope_collision_link_aabb_rejects = 0;
    std::int64_t envelope_collision_kdop_tests = 0;
    std::int64_t envelope_collision_kdop_rejects = 0;
    std::int64_t envelope_collision_kdop_axes_tested = 0;
    std::int64_t envelope_collision_gjk_tests = 0;
    std::int64_t envelope_collision_gjk_rejects = 0;
    std::int64_t envelope_collision_gjk_iterations = 0;
};

class BoxOracle;
class BoxOracleSession;

struct OracleSessionConfig {
    int worker_id = -1;
    OracleNodeId domain_root = kInvalidOracleNodeId;
    bool read_only = true;
};

class BoxOracle {
public:
    virtual ~BoxOracle() = default;
    virtual int n_dims() const = 0;
    virtual OracleNodeId root_node() const = 0;
    virtual int max_tree_depth() const { return 64; }
    virtual const std::vector<Interval>& root_intervals() const = 0;
    virtual std::vector<Interval> node_intervals(OracleNodeId node) const = 0;
    virtual Eigen::VectorXd tree_configuration_for_query(const Eigen::Ref<const Eigen::VectorXd>& q) const {
        return q;
    }
    virtual OracleNodeId child_containing_point(OracleNodeId node,
                                                const Eigen::Ref<const Eigen::VectorXd>& q) const {
        if (node < 0 || is_leaf(node) || q.size() != n_dims()) {
            return kInvalidOracleNodeId;
        }
        const int dim = split_dim(node);
        if (dim < 0 || dim >= q.size()) {
            return kInvalidOracleNodeId;
        }
        return q[dim] <= split_value(node) ? left_child(node) : right_child(node);
    }
    virtual std::vector<std::vector<Interval>> native_interval_copies_for_node(
        OracleNodeId node,
        const std::vector<Interval>& tree_intervals) const {
        (void)node;
        return {tree_intervals};
    }
    virtual std::vector<std::vector<Interval>> native_root_interval_copies() const {
        return native_interval_copies_for_node(root_node(), root_intervals());
    }
    virtual std::vector<Interval> native_root_hull() const {
        const auto copies = native_root_interval_copies();
        if (copies.empty()) {
            return root_intervals();
        }
        std::vector<Interval> hull = copies.front();
        for (std::size_t copy_index = 1; copy_index < copies.size(); ++copy_index) {
            if (copies[copy_index].size() != hull.size()) {
                continue;
            }
            for (std::size_t dim = 0; dim < hull.size(); ++dim) {
                hull[dim] = hull[dim].hull(copies[copy_index][dim]);
            }
        }
        return hull;
    }
    virtual std::vector<Interval> native_root_intervals_for_query(
        const Eigen::Ref<const Eigen::VectorXd>& q) const {
        return query_intervals_for_node(root_node(), root_intervals(), q);
    }
    virtual std::vector<Interval> query_intervals_for_node(OracleNodeId node,
                                                           const std::vector<Interval>& tree_intervals,
                                                           const Eigen::Ref<const Eigen::VectorXd>& q) const {
        (void)node;
        (void)q;
        return tree_intervals;
    }
    virtual bool contains_point(OracleNodeId node, const Eigen::Ref<const Eigen::VectorXd>& q) const = 0;
    virtual bool is_leaf(OracleNodeId node) const = 0;
    virtual int depth(OracleNodeId node) const = 0;
    virtual int split_dim(OracleNodeId node) const = 0;
    virtual double split_value(OracleNodeId node) const = 0;
    virtual OracleNodeId left_child(OracleNodeId node) const = 0;
    virtual OracleNodeId right_child(OracleNodeId node) const = 0;
    virtual SplitNodeResult split_node(OracleNodeId node,
                                       const std::vector<Interval>& intervals,
                                       int changed_dim,
                                       const OracleSplitOptions& options) = 0;
    virtual SplitNodeResult split_node_at(OracleNodeId node, int split_dim, double split_value) = 0;
    virtual bool point_in_collision(const Eigen::Ref<const Eigen::VectorXd>& q) const = 0;
    virtual BoxValidation validate_node(OracleNodeId node, const std::vector<Interval>& intervals, int changed_dim = -1) = 0;
    virtual bool validate_intervals(const std::vector<Interval>& intervals) = 0;
    virtual bool is_reserved(OracleNodeId node) const = 0;
    virtual std::optional<int> reservation_owner(OracleNodeId node) const = 0;
    virtual void reserve_node(OracleNodeId node, int box_id) = 0;
    virtual void release_node(OracleNodeId node) = 0;
    virtual void release_box(int box_id) = 0;
    virtual void clear_reservations() = 0;
    virtual void record_visit(OracleNodeId node) {
        (void)node;
    }
    virtual OracleNodeId select_unexplored_node() const = 0;
    virtual int common_ancestor_depth(OracleNodeId lhs_node, OracleNodeId rhs_node) const {
        (void)lhs_node;
        (void)rhs_node;
        return -1;
    }
    virtual std::unique_ptr<BoxOracleSession> make_session(const OracleSessionConfig& config) {
        (void)config;
        return nullptr;
    }
    virtual OracleValidationDetail last_validation_detail() const { return {}; }
    virtual const OracleCounters& counters() const = 0;
    virtual void reset_counters() = 0;
};

class BoxOracleSession {
public:
    virtual ~BoxOracleSession() = default;
    virtual BoxOracle& oracle() = 0;
    virtual const BoxOracle& oracle() const = 0;
    virtual OracleNodeId domain_root() const = 0;
    virtual bool commit() = 0;
    virtual OracleNodeId map_node_to_master(OracleNodeId worker_node) const = 0;
};

class BoxOracleFactory {
public:
    virtual ~BoxOracleFactory() = default;
    virtual std::unique_ptr<BoxOracleSession> make_session(const OracleSessionConfig& config) = 0;
};

class DatabaseBoxOracle final : public BoxOracle {
public:
    DatabaseBoxOracle(Robot robot,
                      lect_database::LectDatabase& database,
                      Scene scene = {},
                      EndpointSourceConfig endpoint_config = {},
                      EnvelopeTypeConfig envelope_config = {},
                      OracleValidationConfig validation_config = {},
                      const lect_database::LectExternalEvidenceSource* external_evidence_source = nullptr,
                      const lect_database::LectDatabase* direct_external_evidence_database = nullptr);
    ~DatabaseBoxOracle() override;

    DatabaseBoxOracle(const DatabaseBoxOracle&) = delete;
    DatabaseBoxOracle& operator=(const DatabaseBoxOracle&) = delete;
    DatabaseBoxOracle(Robot robot,
                      lect_database::OnlineEnvelopeCacheTree& online_cache,
                      Scene scene = {},
                      EndpointSourceConfig endpoint_config = {},
                      EnvelopeTypeConfig envelope_config = {},
                      OracleValidationConfig validation_config = {},
                      const lect_database::LectExternalEvidenceSource* external_evidence_source = nullptr,
                      const lect_database::LectDatabase* direct_external_evidence_database = nullptr);

    int n_dims() const override;
    OracleNodeId root_node() const override { return 0; }
    int max_tree_depth() const override;
    const std::vector<Interval>& root_intervals() const override;
    std::vector<Interval> node_intervals(OracleNodeId node) const override;
    Eigen::VectorXd tree_configuration_for_query(const Eigen::Ref<const Eigen::VectorXd>& q) const override;
    OracleNodeId child_containing_point(OracleNodeId node,
                                        const Eigen::Ref<const Eigen::VectorXd>& q) const override;
    std::vector<std::vector<Interval>> native_interval_copies_for_node(
        OracleNodeId node,
        const std::vector<Interval>& tree_intervals) const override;
    std::vector<Interval> query_intervals_for_node(OracleNodeId node,
                                                   const std::vector<Interval>& tree_intervals,
                                                   const Eigen::Ref<const Eigen::VectorXd>& q) const override;
    bool contains_point(OracleNodeId node, const Eigen::Ref<const Eigen::VectorXd>& q) const override;
    bool is_leaf(OracleNodeId node) const override;
    int depth(OracleNodeId node) const override;
    int split_dim(OracleNodeId node) const override;
    double split_value(OracleNodeId node) const override;
    OracleNodeId left_child(OracleNodeId node) const override;
    OracleNodeId right_child(OracleNodeId node) const override;
    SplitNodeResult split_node(OracleNodeId node,
                               const std::vector<Interval>& intervals,
                               int changed_dim,
                               const OracleSplitOptions& options) override;
    SplitNodeResult split_node_at(OracleNodeId node, int split_dim, double split_value) override;
    bool point_in_collision(const Eigen::Ref<const Eigen::VectorXd>& q) const override;
    BoxValidation validate_node(OracleNodeId node, const std::vector<Interval>& intervals, int changed_dim = -1) override;
    bool validate_intervals(const std::vector<Interval>& intervals) override;
    bool is_reserved(OracleNodeId node) const override;
    std::optional<int> reservation_owner(OracleNodeId node) const override;
    void reserve_node(OracleNodeId node, int box_id) override;
    void release_node(OracleNodeId node) override;
    void release_box(int box_id) override;
    void clear_reservations() override;
    void record_visit(OracleNodeId node) override;
    OracleNodeId select_unexplored_node() const override;
    int common_ancestor_depth(OracleNodeId lhs_node, OracleNodeId rhs_node) const override;
    std::unique_ptr<BoxOracleSession> make_session(const OracleSessionConfig& config) override;
    OracleValidationDetail last_validation_detail() const override { return last_validation_detail_; }
    const OracleCounters& counters() const override { return counters_; }
    void reset_counters() override { counters_ = {}; }

    void set_scene(Scene scene);
    const Robot& robot() const { return robot_; }
    const Scene& scene() const { return scene_; }
    const EndpointSourceConfig& endpoint_config() const { return endpoint_config_; }
    const EnvelopeTypeConfig& envelope_config() const { return envelope_config_; }
    const OracleValidationConfig& validation_config() const { return validation_config_; }
    const lect_database::LectExternalEvidenceSource* external_evidence_source() const { return external_evidence_source_; }
    const lect_database::LectDatabase* direct_external_evidence_database() const { return direct_external_evidence_database_; }
    lect_database::LectDatabase& database() { return database_; }
    const lect_database::LectDatabase& database() const { return database_; }
    bool envelope_cache_enabled() const { return enable_envelope_cache_; }
    void set_envelope_cache_enabled(bool enabled) { enable_envelope_cache_ = enabled; }

    // Public accessor exposing the canonical endpoint EvidenceKey (channel /
    // source / primary sector / payload_kind) so prewarm can build a key
    // template for bottom-up parent-hull materialization.
    lect_database::EvidenceKey endpoint_evidence_key(OracleNodeId node) const { return endpoint_key(node); }

    // Lazily create and return the master-owned shared endpoint cache used to
    // share endpoints across worker tasks. Returns the same instance on every
    // call so all workers spawned from this master read/write one cache.
    std::shared_ptr<lect_database::SharedEndpointEvidenceCache> shared_endpoint_cache();

    // Non-allocating peek for diagnostics; null when no cache has been created.
    const lect_database::SharedEndpointEvidenceCache* shared_endpoint_cache_peek() const {
        return shared_endpoint_cache_.get();
    }
    void set_shared_endpoint_cache(std::shared_ptr<lect_database::SharedEndpointEvidenceCache> cache) {
        shared_endpoint_cache_ = std::move(cache);
    }

    friend class DatabaseBoxOracleSession;

private:
    // Pre-seed best_tighten_depth_dims_ from the canonical FixedDepthSchedule so
    // the grower's split dimension sequence is determined by (robot, domain),
    // not by the first query that reaches each depth (keeps node paths canonical
    // and external-evidence keys reusable). No-op unless strategy is
    // AAFKVolumeMin with a non-empty schedule.
    void seed_best_tighten_schedule_from_policy();

    struct EndpointPayload {
        std::vector<float> owned_payload;
        std::shared_ptr<const lect_database::EvidenceRecord> record_storage;
        std::shared_ptr<const void> storage_owner;
        std::span<const float> payload;
        std::uint64_t envelope_cache_key = 0;
        bool envelope_cacheable = false;
    };

    lect_database::EvidenceKey endpoint_key(OracleNodeId node) const;
    std::optional<EndpointPayload> endpoint_payload_for_node(OracleNodeId node,
                                                             const std::vector<Interval>& intervals,
                                                             int changed_dim);
    BoxValidation classify_payload(OracleNodeId node,
                                   const std::vector<Interval>& intervals,
                                   const EndpointPayload& endpoint_payload);

    Robot robot_;
    lect_database::LectDatabase& database_;
    lect_database::OnlineEnvelopeCacheTree* online_cache_ = nullptr;
    const lect_database::LectExternalEvidenceSource* external_evidence_source_ = nullptr;
    const lect_database::LectDatabase* direct_external_evidence_database_ = nullptr;
    EndpointSourceConfig endpoint_config_;
    EnvelopeTypeConfig envelope_config_;
    OracleValidationConfig validation_config_;
    Scene scene_;
    CollisionChecker checker_;
    OracleCounters counters_;
    OracleValidationDetail last_validation_detail_;
    std::unordered_map<OracleNodeId, int> node_to_box_;
    std::unordered_map<int, OracleNodeId> box_to_node_;
    std::unordered_map<std::uint64_t, LinkEnvelope> envelope_cache_;
    bool enable_envelope_cache_ = true;
    mutable std::unordered_map<OracleNodeId, std::uint64_t> visit_counts_;
    struct UnexploredLeafCacheEntry {
        OracleNodeId node = kInvalidOracleNodeId;
        double volume = 0.0;
    };
    mutable std::vector<UnexploredLeafCacheEntry> unexplored_leaf_cache_;
    mutable bool unexplored_leaf_cache_dirty_ = true;
    std::vector<int> best_tighten_depth_dims_;
    std::vector<int> best_tighten_recent_dims_;
    std::vector<double> best_tighten_reference_volumes_;
    // Thread-safe interval-keyed endpoint cache shared across worker tasks. The
    // master lazily owns it via shared_endpoint_cache(); workers receive the same
    // instance via set_shared_endpoint_cache().
    std::shared_ptr<lect_database::SharedEndpointEvidenceCache> shared_endpoint_cache_;

    struct Impl;
    std::unique_ptr<Impl> impl_;
};

class DatabaseBoxOracleSession final : public BoxOracleSession {
public:
    DatabaseBoxOracleSession(DatabaseBoxOracle& master, const OracleSessionConfig& config);
    ~DatabaseBoxOracleSession() override;

    BoxOracle& oracle() override { return *worker_oracle_; }
    const BoxOracle& oracle() const override { return *worker_oracle_; }
    OracleNodeId domain_root() const override { return master_domain_root_; }
    bool commit() override;
    OracleNodeId map_node_to_master(OracleNodeId worker_node) const override;

private:
    bool replay_structure(OracleNodeId worker_node, OracleNodeId master_node);
    bool copy_worker_leaf_evidence();
    static std::filesystem::path make_temp_dir();

    DatabaseBoxOracle& master_;
    OracleNodeId master_domain_root_ = kInvalidOracleNodeId;
    bool read_only_ = true;
    bool committed_ = false;
    std::filesystem::path temp_dir_;
    std::optional<lect_database::LectDatabase> worker_database_;
    std::unique_ptr<DatabaseBoxOracle> worker_oracle_;
    std::unordered_map<OracleNodeId, OracleNodeId> node_remap_;
};

class DatabaseBoxOracleFactory final : public BoxOracleFactory {
public:
    explicit DatabaseBoxOracleFactory(DatabaseBoxOracle& master) : master_(master) {}
    std::unique_ptr<BoxOracleSession> make_session(const OracleSessionConfig& config) override;

private:
    DatabaseBoxOracle& master_;
};

}  // namespace rbf
