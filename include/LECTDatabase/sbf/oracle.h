#pragma once

#include <LECTDatabase/sbf/scene.h>

#include <rbf/lect_database.h>

#include <sbf/envelope/envelope_collision.h>
#include <sbf/envelope/envelope_type.h>

#include <Eigen/Core>

#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <unordered_map>
#include <vector>

namespace rbf {

enum class BoxValidation : std::uint8_t {
    Free = 0,
    Occupied = 1,
    Unknown = 2,
};

enum class OracleValidationMode : std::uint8_t {
    Strict = 0,
    CoverageHeuristic = 1,
};

struct OracleSplitOptions {
    bool use_best_tighten = true;
};

struct SplitNodeResult {
    bool split = false;
    int node = -1;
    int left = -1;
    int right = -1;
    int split_dim = -1;
    double split_value = 0.0;
};

struct OracleValidationConfig {
    OracleValidationMode mode = OracleValidationMode::Strict;
    bool accept_unsafe_free = false;
};

struct OracleValidationDetail {
    int node = -1;
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
    double materialization_envelope_time_us = 0.0;
    double materialization_cache_lookup_time_us = 0.0;
    double materialization_cache_read_time_us = 0.0;
    double materialization_envelope_compute_time_us = 0.0;
    double materialization_envelope_read_time_us = 0.0;
    int materialization_incremental_fk = 0;
    int materialization_source_incremental_state = 0;
    int materialization_reused_fk = 0;
    int materialization_reused_endpoint_cache = 0;
    int materialization_reused_external_evidence = 0;
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
    int domain_root = -1;
    bool read_only = true;
};

class BoxOracle {
public:
    virtual ~BoxOracle() = default;
    virtual int n_dims() const = 0;
    virtual int root_node() const = 0;
    virtual const std::vector<Interval>& root_intervals() const = 0;
    virtual std::vector<Interval> node_intervals(int node) const = 0;
    virtual bool contains_point(int node, const Eigen::Ref<const Eigen::VectorXd>& q) const = 0;
    virtual bool is_leaf(int node) const = 0;
    virtual int depth(int node) const = 0;
    virtual int split_dim(int node) const = 0;
    virtual double split_value(int node) const = 0;
    virtual int left_child(int node) const = 0;
    virtual int right_child(int node) const = 0;
    virtual SplitNodeResult split_node(int node,
                                       const std::vector<Interval>& intervals,
                                       int changed_dim,
                                       const OracleSplitOptions& options) = 0;
    virtual SplitNodeResult split_node_at(int node, int split_dim, double split_value) = 0;
    virtual bool point_in_collision(const Eigen::Ref<const Eigen::VectorXd>& q) const = 0;
    virtual BoxValidation validate_node(int node, const std::vector<Interval>& intervals, int changed_dim = -1) = 0;
    virtual bool validate_intervals(const std::vector<Interval>& intervals) = 0;
    virtual bool is_reserved(int node) const = 0;
    virtual std::optional<int> reservation_owner(int node) const = 0;
    virtual void reserve_node(int node, int box_id) = 0;
    virtual void release_node(int node) = 0;
    virtual void release_box(int box_id) = 0;
    virtual void clear_reservations() = 0;
    virtual int select_unexplored_node() const = 0;
    virtual int common_ancestor_depth(int lhs_node, int rhs_node) const {
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
    virtual int domain_root() const = 0;
    virtual bool commit() = 0;
    virtual int map_node_to_master(int worker_node) const = 0;
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
                      OracleValidationConfig validation_config = {});

    int n_dims() const override;
    int root_node() const override { return 0; }
    const std::vector<Interval>& root_intervals() const override;
    std::vector<Interval> node_intervals(int node) const override;
    bool contains_point(int node, const Eigen::Ref<const Eigen::VectorXd>& q) const override;
    bool is_leaf(int node) const override;
    int depth(int node) const override;
    int split_dim(int node) const override;
    double split_value(int node) const override;
    int left_child(int node) const override;
    int right_child(int node) const override;
    SplitNodeResult split_node(int node,
                               const std::vector<Interval>& intervals,
                               int changed_dim,
                               const OracleSplitOptions& options) override;
    SplitNodeResult split_node_at(int node, int split_dim, double split_value) override;
    bool point_in_collision(const Eigen::Ref<const Eigen::VectorXd>& q) const override;
    BoxValidation validate_node(int node, const std::vector<Interval>& intervals, int changed_dim = -1) override;
    bool validate_intervals(const std::vector<Interval>& intervals) override;
    bool is_reserved(int node) const override;
    std::optional<int> reservation_owner(int node) const override;
    void reserve_node(int node, int box_id) override;
    void release_node(int node) override;
    void release_box(int box_id) override;
    void clear_reservations() override;
    int select_unexplored_node() const override;
    int common_ancestor_depth(int lhs_node, int rhs_node) const override;
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
    lect_database::LectDatabase& database() { return database_; }
    const lect_database::LectDatabase& database() const { return database_; }

private:
    lect_database::EvidenceKey endpoint_key(int node) const;
    std::optional<std::vector<float>> endpoint_payload_for_node(int node,
                                                                const std::vector<Interval>& intervals,
                                                                int changed_dim);
    BoxValidation classify_payload(int node,
                                   const std::vector<Interval>& intervals,
                                   const std::vector<float>& endpoint_payload);

    Robot robot_;
    lect_database::LectDatabase& database_;
    EndpointSourceConfig endpoint_config_;
    EnvelopeTypeConfig envelope_config_;
    OracleValidationConfig validation_config_;
    Scene scene_;
    CollisionChecker checker_;
    OracleCounters counters_;
    OracleValidationDetail last_validation_detail_;
    std::unordered_map<int, int> node_to_box_;
    std::unordered_map<int, int> box_to_node_;
};

class DatabaseBoxOracleSession final : public BoxOracleSession {
public:
    DatabaseBoxOracleSession(DatabaseBoxOracle& master, const OracleSessionConfig& config);
    ~DatabaseBoxOracleSession() override;

    BoxOracle& oracle() override { return *worker_oracle_; }
    const BoxOracle& oracle() const override { return *worker_oracle_; }
    int domain_root() const override { return master_domain_root_; }
    bool commit() override;
    int map_node_to_master(int worker_node) const override;

private:
    bool replay_structure(int worker_node, int master_node);
    bool copy_worker_leaf_evidence();
    static std::filesystem::path make_temp_dir();

    DatabaseBoxOracle& master_;
    int master_domain_root_ = -1;
    bool read_only_ = true;
    bool committed_ = false;
    std::filesystem::path temp_dir_;
    std::optional<lect_database::LectDatabase> worker_database_;
    std::unique_ptr<DatabaseBoxOracle> worker_oracle_;
    std::unordered_map<int, int> node_remap_;
};

class DatabaseBoxOracleFactory final : public BoxOracleFactory {
public:
    explicit DatabaseBoxOracleFactory(DatabaseBoxOracle& master) : master_(master) {}
    std::unique_ptr<BoxOracleSession> make_session(const OracleSessionConfig& config) override;

private:
    DatabaseBoxOracle& master_;
};

}  // namespace rbf
