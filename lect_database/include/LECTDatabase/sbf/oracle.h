#pragma once

#include <LECTDatabase/online_cache.h>
#include <LECTDatabase/sbf/oracle_types.h>
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
#include <string>
#include <unordered_map>
#include <vector>

namespace rbf {

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
    virtual std::vector<Interval> planning_intervals() const {
        return native_root_hull();
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
    virtual OracleSplitPolicyDescriptor split_policy_descriptor() const {
        OracleSplitPolicyDescriptor descriptor;
        descriptor.strategy = lect_database::SplitStrategy::AAFKVolumeMin;
        return descriptor;
    }
    virtual OracleNodeId left_child(OracleNodeId node) const = 0;
    virtual OracleNodeId right_child(OracleNodeId node) const = 0;
    virtual OracleNodeTopology node_topology(OracleNodeId node) const {
        OracleNodeTopology topology;
        if (node < 0) {
            return topology;
        }
        topology.valid = true;
        topology.leaf = is_leaf(node);
        topology.depth = depth(node);
        if (!topology.leaf) {
            topology.split_dim = split_dim(node);
            topology.split_value = split_value(node);
            topology.left = left_child(node);
            topology.right = right_child(node);
        }
        return topology;
    }
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
    std::vector<Interval> planning_intervals() const override;
    std::vector<Interval> query_intervals_for_node(OracleNodeId node,
                                                   const std::vector<Interval>& tree_intervals,
                                                   const Eigen::Ref<const Eigen::VectorXd>& q) const override;
    bool contains_point(OracleNodeId node, const Eigen::Ref<const Eigen::VectorXd>& q) const override;
    bool is_leaf(OracleNodeId node) const override;
    int depth(OracleNodeId node) const override;
    int split_dim(OracleNodeId node) const override;
    double split_value(OracleNodeId node) const override;
    OracleSplitPolicyDescriptor split_policy_descriptor() const override;
    OracleNodeId left_child(OracleNodeId node) const override;
    OracleNodeId right_child(OracleNodeId node) const override;
    OracleNodeTopology node_topology(OracleNodeId node) const override;
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
    void set_collect_full_overlap_stats(bool enabled) {
        validation_config_.collect_full_overlap_stats = enabled;
    }

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
        bool reused_external_evidence = false;
    };

    lect_database::EvidenceKey endpoint_key(OracleNodeId node) const;
    std::optional<EndpointPayload> endpoint_payload_for_node(OracleNodeId node,
                                                             const std::vector<Interval>& intervals,
                                                             int changed_dim,
                                                             bool allow_external_evidence = true);
    BoxValidation classify_payload(OracleNodeId node,
                                   const std::vector<Interval>& intervals,
                                   const EndpointPayload& endpoint_payload);
    struct ValidationCacheEntry {
        BoxValidation result = BoxValidation::Unknown;
        OracleValidationDetail detail;
    };

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
    mutable OracleCounters counters_;
    OracleValidationDetail last_validation_detail_;
    std::unordered_map<std::uint64_t, ValidationCacheEntry> validation_cache_;
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
