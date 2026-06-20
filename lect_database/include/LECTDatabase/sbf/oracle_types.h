#pragma once

#include <rbf/envelope/endpoint_source.h>
#include <rbf/lect_database/split_policy.h>

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace rbf {

using OracleNodeId = std::int64_t;
inline constexpr OracleNodeId kInvalidOracleNodeId = -1;
// Oracle evidence replay compatibility is tied to the LECT split policy
// descriptor as well as root intervals and robot/envelope identity.
using OracleSplitPolicyDescriptor = lect_database::SplitPolicyDescriptor;

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

struct OccupiedCertificateConfig {
    bool enabled = false;
    double numerical_epsilon = 1e-9;
    double min_penetration_margin = 0.0;
};

struct MaterialPointOccupiedWitness {
    int link_id = -1;
    int obstacle_id = -1;
    Eigen::Vector3d link_point = Eigen::Vector3d::Zero();
    double center_signed_distance = 0.0;
    double motion_bound = 0.0;
    double epsilon_num = 1e-9;

    bool certifies_occupied() const {
        return center_signed_distance + motion_bound + epsilon_num < 0.0;
    }
};

struct OracleValidationBlocker {
    int active_link_index = -1;
    int link_id = -1;
    int obstacle_id = -1;
    int stage = 0;
    double margin = 0.0;
    double overlap_depth = 0.0;
    double overlap_volume_ratio = 0.0;
    std::vector<int> affected_joints;
};

struct SplitNodeResult {
    bool split = false;
    OracleNodeId node = kInvalidOracleNodeId;
    OracleNodeId left = kInvalidOracleNodeId;
    OracleNodeId right = kInvalidOracleNodeId;
    int split_dim = -1;
    double split_value = 0.0;
};

struct OracleNodeTopology {
    bool valid = false;
    bool leaf = true;
    int depth = 0;
    int split_dim = -1;
    double split_value = 0.0;
    OracleNodeId left = kInvalidOracleNodeId;
    OracleNodeId right = kInvalidOracleNodeId;
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
    bool external_evidence_live_retry_on_maybe = false;
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
    // Only enable for diagnostics or pruning heuristics that need an
    // envelope-level overlap ratio. Default validation keeps the early-exit
    // collision path.
    bool collect_full_overlap_stats = false;
    // Optional occupied pruning must be backed by a material-point signed
    // distance witness. Disabled by default; AABB/support-hull overlap is never
    // used as an occupied proof.
    OccupiedCertificateConfig occupied_certificate;
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
    double aabb_overlap_depth = 0.0;
    double aabb_overlap_volume_ratio = 0.0;
    int blocker_active_link_index = -1;
    int blocker_link_id = -1;
    int blocker_obstacle_id = -1;
    int blocker_stage = 0;
    double blocker_margin = 0.0;
    double blocker_overlap_depth = 0.0;
    double blocker_overlap_volume_ratio = 0.0;
    std::vector<int> blocker_affected_joints;
    std::vector<OracleValidationBlocker> blockers;
    std::uint64_t blocker_signature_hash = 0;
    bool occupied_certificate_checked = false;
    bool occupied_certificate_found = false;
    int occupied_witness_link_id = -1;
    int occupied_witness_obstacle_id = -1;
    double occupied_witness_center_signed_distance = 0.0;
    double occupied_witness_motion_bound = 0.0;
};

struct OracleCounters {
    int node_validations = 0;
    int interval_validations = 0;
    int certified_free = 0;
    int certified_occupied = 0;
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
    int materialization_external_exact_hits = 0;
    int materialization_external_exact_misses = 0;
    int materialization_external_live_fallbacks = 0;
    int materialization_external_maybe_live_retries = 0;
    int materialization_external_maybe_live_retry_free = 0;
    int interval_replay_compatibility_checks = 0;
    int interval_replay_compatible = 0;
    int interval_replay_incompatible = 0;
    int interval_replay_direct_exact_hits = 0;
    int interval_replay_key_only_blocked = 0;
    int materialization_reused_shared_endpoint_cache = 0;
    int materialization_stored_shared_endpoint_cache = 0;
    int materialization_reused_cached_envelope = 0;
    int materialization_candidate_dirty_count = 0;
    int materialization_predh_rebuild_count = 0;
    int canonical_frame_invalid = 0;
    int canonical_reflected_seed_misses = 0;
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
    double envelope_collision_overlap_depth_sum = 0.0;
    double envelope_collision_overlap_depth_max = 0.0;
    double envelope_collision_overlap_volume_ratio_max = 0.0;
};

struct IntervalEvidenceCompatibility {
    bool canonical_frame_valid = false;
    bool semantic_identity_match = false;
    bool exact_interval_lookup_required = true;
    bool compatible = false;
    bool direct_database = false;
    std::uint64_t lookup_interval_fingerprint = 0;
    std::string reason;
};

class BoxOracle;
class BoxOracleSession;

struct OracleSessionConfig {
    int worker_id = -1;
    OracleNodeId domain_root = kInvalidOracleNodeId;
    bool read_only = true;
};

}  // namespace rbf
