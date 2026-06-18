#include <SBF/safe_box_forest.h>

#include <sbf/core/joint_symmetry.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>

namespace rbf {
namespace {

std::string effective_symmetry_descriptor(const RBFPlanningConfig& config) {
    if (!config.database.canonical_mode) {
        return {};
    }
    return config.database.symmetry_descriptor.empty()
        ? std::string(lect_database::kJointSymmetryNativeV1)
        : config.database.symmetry_descriptor;
}

std::filesystem::path default_database_path(const Robot& robot) {
    return std::filesystem::current_path() /
        ".sbf_lect_database" /
        std::to_string(robot.fingerprint());
}

const char* endpoint_cache_channel_name(EndpointSource source) {
    return source_channel(source) == 0 ? "safe" : "rapid";
}

std::string endpoint_descriptor_for(const EndpointSourceConfig& config) {
    std::ostringstream out;
    out << "channel=" << endpoint_cache_channel_name(config.source)
        << "|source=" << endpoint_source_name(config.source)
        << "|source_id=" << static_cast<int>(config.source)
        << "|n_samples_crit=" << config.n_samples_crit
        << "|max_phase_analytical=" << config.max_phase_analytical
        << "|bypass_narrow_skip=" << (config.bypass_narrow_skip ? 1 : 0)
        << "|gcpc_match_analytical=" << (config.gcpc_match_analytical ? 1 : 0)
        << "|hifk_max_depth=" << config.hifk_max_depth
        << "|hifk_vol_ratio_thresh=" << config.hifk_vol_ratio_thresh;
    return out.str();
}

std::string envelope_descriptor_for(const EnvelopeTypeConfig& config) {
    std::ostringstream out;
    out << "type=" << static_cast<int>(config.type)
        << "|n_subdivisions=" << config.n_subdivisions
        << "|kdop_direction_set=" << static_cast<int>(config.kdop_config.direction_set)
        << "|kdop_safety_epsilon=" << config.kdop_config.safety_epsilon
        << "|support_safety_epsilon=" << config.support_hull_config.safety_epsilon;
    return out.str();
}

std::vector<Interval> database_root_intervals_for(const Robot& robot,
                                                  const RBFPlanningConfig& config) {
    const bool canonical_mode = config.database.canonical_mode;
    const std::string symmetry_descriptor = effective_symmetry_descriptor(config);
    std::vector<Interval> root_intervals = lect_database::canonical_root_intervals_for_robot(
        robot,
        canonical_mode,
        symmetry_descriptor);
    const auto& override_intervals = config.database.root_intervals_override;
    if (override_intervals.empty()) {
        return root_intervals;
    }
    if (override_intervals.size() != root_intervals.size()) {
        std::ostringstream out;
        out << "database root_intervals_override has " << override_intervals.size()
            << " dims, expected " << root_intervals.size();
        throw std::runtime_error(out.str());
    }
    const std::vector<Interval>& joint_limits = robot.joint_limits().limits;
    std::optional<JointSymmetry> primary_symmetry;
    if (canonical_mode && lect_database::uses_joint_symmetry_native(symmetry_descriptor)) {
        auto symmetries = detect_joint_symmetries(robot);
        if (!symmetries.empty() && symmetries.front().type != JointSymmetryType::NONE) {
            primary_symmetry = symmetries.front();
        }
    }
    for (std::size_t dim = 0; dim < override_intervals.size(); ++dim) {
        const Interval& requested = override_intervals[dim];
        if (requested.lo > requested.hi) {
            std::ostringstream out;
            out << "database root_intervals_override[" << dim << "] is invalid: ["
                << requested.lo << ", " << requested.hi << "]";
            throw std::runtime_error(out.str());
        }
        const bool symmetry_hull_override =
            primary_symmetry &&
            dim == static_cast<std::size_t>(primary_symmetry->joint_index) &&
            primary_symmetry->period > 0.0 &&
            std::abs(requested.lo - (primary_symmetry->canonical_lo -
                                     2.0 * primary_symmetry->period)) <= 1e-9 &&
            std::abs(requested.hi - (primary_symmetry->canonical_lo +
                                     2.0 * primary_symmetry->period)) <= 1e-9;
        if (symmetry_hull_override) {
            continue;
        }
        const Interval& allowed = (canonical_mode && !override_intervals.empty() &&
                                   dim < joint_limits.size() &&
                                   (requested.lo + 1e-12 < root_intervals[dim].lo ||
                                    requested.hi - 1e-12 > root_intervals[dim].hi))
            ? joint_limits[dim]
            : root_intervals[dim];
        if (requested.lo + 1e-12 < allowed.lo || requested.hi - 1e-12 > allowed.hi) {
            std::ostringstream out;
            out << "database root_intervals_override[" << dim << "]=["
                << requested.lo << ", " << requested.hi << "] exceeds allowed root ["
                << allowed.lo << ", " << allowed.hi << "]";
            throw std::runtime_error(out.str());
        }
    }
    return override_intervals;
}

std::vector<Interval> database_coverage_intervals_for(const Robot& robot,
                                                      const RBFPlanningConfig& config,
                                                      const std::vector<Interval>& root_intervals) {
    const auto& override_intervals = config.database.coverage_intervals_override;
    const std::vector<Interval>& joint_limits = robot.joint_limits().limits;
    if (!override_intervals.empty()) {
        if (override_intervals.size() != root_intervals.size()) {
            std::ostringstream out;
            out << "database coverage_intervals_override has " << override_intervals.size()
                << " dims, expected " << root_intervals.size();
            throw std::runtime_error(out.str());
        }
        for (std::size_t dim = 0; dim < override_intervals.size(); ++dim) {
            const Interval& requested = override_intervals[dim];
            if (requested.lo > requested.hi) {
                std::ostringstream out;
                out << "database coverage_intervals_override[" << dim << "] is invalid: ["
                    << requested.lo << ", " << requested.hi << "]";
                throw std::runtime_error(out.str());
            }
            if (dim < joint_limits.size() &&
                (requested.lo + 1e-12 < joint_limits[dim].lo ||
                 requested.hi - 1e-12 > joint_limits[dim].hi)) {
                std::ostringstream out;
                out << "database coverage_intervals_override[" << dim << "]=["
                    << requested.lo << ", " << requested.hi << "] exceeds joint limit ["
                    << joint_limits[dim].lo << ", " << joint_limits[dim].hi << "]";
                throw std::runtime_error(out.str());
            }
        }
        return override_intervals;
    }

    const bool canonical_mode = config.database.canonical_mode;
    const std::string symmetry_descriptor = effective_symmetry_descriptor(config);
    if (!canonical_mode || !lect_database::uses_joint_symmetry_native(symmetry_descriptor)) {
        return root_intervals;
    }
    auto symmetries = detect_joint_symmetries(robot);
    if (symmetries.empty()) {
        return root_intervals;
    }
    const JointSymmetry& symmetry = symmetries.front();
    if (symmetry.type == JointSymmetryType::NONE || symmetry.period <= 0.0 ||
        symmetry.joint_index < 0 ||
        static_cast<std::size_t>(symmetry.joint_index) >= root_intervals.size() ||
        static_cast<std::size_t>(symmetry.joint_index) >= joint_limits.size()) {
        return root_intervals;
    }

    const std::size_t dim = static_cast<std::size_t>(symmetry.joint_index);
    const Interval& root = root_intervals[dim];
    if (root.lo + 1e-12 < symmetry.canonical_lo ||
        root.hi - 1e-12 > symmetry.canonical_hi) {
        return root_intervals;
    }

    const Interval& limit = joint_limits[dim];
    std::vector<Interval> coverage = root_intervals;
    bool found = false;
    double lo = std::numeric_limits<double>::infinity();
    double hi = -std::numeric_limits<double>::infinity();
    for (int shift = -16; shift <= 16; ++shift) {
        const double shifted_lo = root.lo + static_cast<double>(shift) * symmetry.period;
        const double shifted_hi = root.hi + static_cast<double>(shift) * symmetry.period;
        if (shifted_hi < limit.lo - 1e-12 || shifted_lo > limit.hi + 1e-12) {
            continue;
        }
        lo = std::min(lo, std::max(shifted_lo, limit.lo));
        hi = std::max(hi, std::min(shifted_hi, limit.hi));
        found = true;
    }
    if (found && lo <= hi) {
        coverage[dim] = {lo, hi};
    }
    return coverage;
}

lect_database::LectDatabaseConfig make_database_config(const Robot& robot,
                                                       const RBFPlanningConfig& config) {
    lect_database::LectDatabaseConfig database_config;
    const bool canonical_mode = config.database.canonical_mode;
    const std::string symmetry_descriptor = effective_symmetry_descriptor(config);
    database_config.path = config.database.path.empty()
        ? default_database_path(robot)
        : config.database.path;
    database_config.root_intervals = database_root_intervals_for(robot, config);
    database_config.coverage_intervals = database_coverage_intervals_for(
        robot,
        config,
        database_config.root_intervals);
    database_config.split_policy = config.database.split_policy;
    database_config.open.read_only = config.database.read_only;
    database_config.open.create_if_missing = config.database.create_if_missing;
    database_config.open.verify_identity = config.database.verify_identity;
    database_config.open.replay_journal = config.database.replay_journal;
    database_config.propagate_parent_hulls = config.database.propagate_parent_hulls;
    database_config.defer_parent_hull_writes = config.database.defer_parent_hull_writes;
    database_config.page_size_bytes = config.database.page_size_bytes;
    database_config.max_resident_pages = config.database.max_resident_pages;
    database_config.max_tree_depth = config.database.max_tree_depth;
    database_config.identity = lect_database::make_identity_for_robot(
        robot,
        database_config.root_intervals,
        database_config.split_policy,
        canonical_mode,
        symmetry_descriptor,
        endpoint_descriptor_for(config.endpoint_source),
        envelope_descriptor_for(config.envelope_type),
        "endpoint_envelope_v1",
        "sbf_online_cache_v1");
    return database_config;
}

}  // namespace

RBFPlanningConfig::RBFPlanningConfig() {
    endpoint_source.source = EndpointSource::CritSample;
    envelope_type.type = EnvelopeType::SupportHull;
    envelope_type.n_subdivisions = 4;
    envelope_type.kdop_config.direction_set = KdopDirectionSet::DOP26;
    envelope_type.kdop_config.safety_epsilon = 1e-9;
    envelope_type.kdop_config.overlap_tolerance = 1e-5;
    envelope_type.support_hull_config.keep_kdop = true;
    envelope_type.support_hull_config.safety_epsilon = 1e-9;
    envelope_type.support_hull_config.overlap_tolerance = 1e-5;

    validation.mode = OracleValidationMode::CoverageHeuristic;
    validation.accept_unsafe_free = true;

    grower.commit_policy = BoxCommitPolicy::CommitProvisionalAllowed;
    grower.max_boxes = 5000;
    grower.timeout_ms = 60000.0;
    grower.find_free_box.max_depth = 120;
    grower.find_free_box.reject_seed_collision = false;
    grower.rrt_goal_bias = 0.2;
    grower.intertree_goal_bias = 0.25;
    grower.rrt_step_ratio = 0.08;
    grower.unexplored_sample_prob = 0.45;
    grower.component_connect_prob = 0.45;
    grower.component_connect_candidate_limit = 4;
    grower.component_connect_stage_normalized_linf = 0.35;

    connector.pave.commit_policy = BoxCommitPolicy::CommitProvisionalAllowed;
    connector.pave.max_chain = 0;
    connector.pave.max_steps_per_waypoint = 12;
    connector.pave.find_free_box.max_depth = 120;
    connector.pave.find_free_box.reject_seed_collision = false;
    connector.pave.fill_gaps = true;
    connector.pave.require_connected_chain = true;
    connector.pave.max_gap_fill_depth = 8;
    connector.per_pair_timeout_ms = 250.0;
    connector.max_pairs_per_gap = 8;
    connector.rrt.max_iters = 50000;
    connector.rrt.timeout_ms = 2000.0;
    connector.rrt.step_size = 0.25;
    connector.rrt.goal_bias = 0.4;
    connector.rrt.segment_resolution = 16;
    connector.rrt.segment_step = query.audit_segment_step;
    connector.point_validated_gap_step = query.audit_segment_step;
    connector.max_total_bridge_boxes = 0;
    connector.frontier_bridge = false;

    query.nearest_if_outside = false;
}

RBFPlanningForest::RBFPlanningForest(Robot robot, RBFPlanningConfig config)
    : robot_(std::move(robot)), audit_robot_(robot_), config_(std::move(config)) {
    if (config_.envelope_type.n_subdivisions <= 0) {
        config_.envelope_type.n_subdivisions = 4;
    }
    database_ = std::make_unique<lect_database::LectDatabase>();
    std::string open_reason;
    if (!database_->open(make_database_config(robot_, config_), &open_reason)) {
        throw std::runtime_error("failed to open LECTDatabase runtime: " + open_reason);
    }
    if (!config_.database.external_evidence_path.empty()) {
        auto external_config = make_database_config(robot_, config_);
        external_config.path = config_.database.external_evidence_path;
        external_config.open.read_only = true;
        external_config.open.create_if_missing = false;
        external_config.open.verify_identity = config_.database.verify_identity;
        external_config.open.replay_journal = config_.database.replay_journal;
        // External evidence reuse only consumes endpoint materialization, so
        // envelope families may differ from the active planning config.
        external_config.identity.envelope_descriptor.clear();
        if (config_.database.external_evidence_use_snapshot) {
            lect_database::LectDatabase verifier;
            std::string verify_reason;
            // The verifier only confirms the legacy external-evidence database
            // exists and its identity matches; the actual evidence is served by
            // the read-only mmap snapshot opened below. Use a metadata-only open
            // so we skip loading all node pages / evidence / indices.
            external_config.open.metadata_only = true;
            if (!verifier.open(external_config, &verify_reason)) {
                throw std::runtime_error(
                    "failed to verify external LECTDatabase evidence source: " + verify_reason);
            }
            const auto snapshot_path = config_.database.external_evidence_snapshot_path.empty()
                ? lect_database::LectReadSnapshot::default_snapshot_path(
                      config_.database.external_evidence_path)
                : config_.database.external_evidence_snapshot_path;
            if (config_.database.external_evidence_auto_build_snapshot &&
                !std::filesystem::exists(snapshot_path)) {
                std::string build_reason;
                if (!lect_database::LectReadSnapshot::build_from_legacy(
                        config_.database.external_evidence_path,
                        snapshot_path,
                        &build_reason)) {
                    throw std::runtime_error("failed to build external LECT snapshot: " +
                                             build_reason);
                }
            }
            external_evidence_snapshot_ = std::make_unique<lect_database::LectReadSnapshot>();
            std::string snapshot_reason;
            if (!external_evidence_snapshot_->open(snapshot_path, &snapshot_reason)) {
                throw std::runtime_error("failed to open external LECT snapshot evidence source: " +
                                         snapshot_reason);
            }
            external_evidence_snapshot_source_ =
                std::make_unique<lect_database::LectSnapshotEvidenceSource>(
                    *external_evidence_snapshot_);
            external_evidence_source_ = external_evidence_snapshot_source_.get();
        } else {
            external_evidence_database_ = std::make_unique<lect_database::LectDatabase>();
            std::string external_reason;
            if (!external_evidence_database_->open(external_config, &external_reason)) {
                throw std::runtime_error("failed to open external LECTDatabase evidence source: " +
                                         external_reason);
            }
            external_evidence_database_source_ =
                std::make_unique<lect_database::LectDatabaseEvidenceSource>(
                    *external_evidence_database_);
            external_evidence_source_ = external_evidence_database_source_.get();
            direct_external_evidence_database_ = external_evidence_database_.get();
        }
    }
    online_cache_ =
        std::make_unique<lect_database::OnlineEnvelopeCacheTree>(*database_,
                                                                 config_.database.online_cache);
    reset_oracle(Scene{});
}

}  // namespace rbf
