#include <SBF/safe_box_forest.h>

#include <sbf/core/joint_symmetry.h>

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdlib>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <iterator>
#include <limits>
#include <optional>
#include <queue>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include "virtual_sparse_ffb.h"

namespace rbf {

namespace {

bool legacy_query_boxcorridor_enabled() {
    const char* value = std::getenv("RBF_LEGACY_QUERY_BOX_CORRIDOR");
    return value != nullptr && value[0] == '1';
}

double boundary_max_depth_failure_count_local(const StageContext& context) {
    const auto& diagnostics = context.diagnostics();
    return diagnostics.value("connector.chain_pave_boundary_fail_depth_cap", 0.0) +
           diagnostics.value("connector.chain_pave_boundary_fail_unknown_depth_cap", 0.0) +
           diagnostics.value("connector.chain_pave_boundary_fail_reserved_depth_cap", 0.0);
}

double diagnostic_map_value(const std::unordered_map<std::string, double>& diagnostics,
                            const std::string& key) {
    const auto it = diagnostics.find(key);
    return it == diagnostics.end() ? 0.0 : it->second;
}

void set_diagnostic_max(std::unordered_map<std::string, double>& diagnostics,
                        const std::string& key,
                        double value) {
    auto it = diagnostics.find(key);
    if (it == diagnostics.end() || value > it->second) {
        diagnostics[key] = value;
    }
}

double portal_membership_policy_code(PortalMembershipPolicy policy) {
    switch (policy) {
    case PortalMembershipPolicy::GlobalForestOnly:
        return 0.0;
    case PortalMembershipPolicy::PortalInteriorIndex:
        return 1.0;
    }
    return -1.0;
}

void record_portal_membership_policy(std::unordered_map<std::string, double>& diagnostics,
                                     PortalMembershipPolicy policy,
                                     const std::string& prefix = "portal_membership.") {
    diagnostics[prefix + "policy"] = portal_membership_policy_code(policy);
    diagnostics[prefix + "global_forest_only"] =
        policy == PortalMembershipPolicy::GlobalForestOnly ? 1.0 : 0.0;
    diagnostics[prefix + "portal_interior_index"] =
        policy == PortalMembershipPolicy::PortalInteriorIndex ? 1.0 : 0.0;
    if (policy == PortalMembershipPolicy::PortalInteriorIndex) {
        diagnostics[prefix + "portal_interior_index_requested"] += 1.0;
        diagnostics[prefix + "portal_interior_index_unavailable"] += 1.0;
        diagnostics[prefix + "global_forest_only_fallback"] += 1.0;
    }
}

void record_portal_membership_policy(StageDiagnostics& diagnostics,
                                     PortalMembershipPolicy policy,
                                     const std::string& prefix = "portal_membership.") {
    diagnostics.set_value(prefix + "policy", portal_membership_policy_code(policy));
    diagnostics.set_value(prefix + "global_forest_only",
                          policy == PortalMembershipPolicy::GlobalForestOnly ? 1.0 : 0.0);
    diagnostics.set_value(prefix + "portal_interior_index",
                          policy == PortalMembershipPolicy::PortalInteriorIndex ? 1.0 : 0.0);
    if (policy == PortalMembershipPolicy::PortalInteriorIndex) {
        diagnostics.add_counter(prefix + "portal_interior_index_requested");
        diagnostics.add_counter(prefix + "portal_interior_index_unavailable");
        diagnostics.add_counter(prefix + "global_forest_only_fallback");
    }
}

bool is_latched_diagnostic_key(const std::string& key) {
    return key == "portal_membership.policy" ||
           key == "portal_membership.global_forest_only" ||
           key == "portal_membership.portal_interior_index" ||
           key == "ffb.free_ancestor_depth_max" ||
           key == "ffb.virtual_sparse_binary_probe_depth_max";
}

void merge_diagnostic_snapshot(std::unordered_map<std::string, double>& diagnostics,
                               const std::unordered_map<std::string, double>& snapshot) {
    for (const auto& [key, value] : snapshot) {
        if (is_latched_diagnostic_key(key)) {
            set_diagnostic_max(diagnostics, key, value);
        } else {
            diagnostics[key] += value;
        }
    }
}

void merge_diagnostic_snapshot(StageDiagnostics& diagnostics,
                               const std::unordered_map<std::string, double>& snapshot) {
    for (const auto& [key, value] : snapshot) {
        if (is_latched_diagnostic_key(key)) {
            diagnostics.set_value(key, std::max(diagnostics.value(key), value));
        } else {
            diagnostics.add_counter(key, value);
        }
    }
}

void record_oracle_cache_counter_snapshot(std::unordered_map<std::string, double>& diagnostics,
                                          const OracleCounters& counters,
                                          const std::string& prefix) {
    diagnostics[prefix + "node_validations"] =
        static_cast<double>(counters.node_validations);
    diagnostics[prefix + "interval_validations"] =
        static_cast<double>(counters.interval_validations);
    diagnostics[prefix + "certified_free"] =
        static_cast<double>(counters.certified_free);
    diagnostics[prefix + "certified_occupied"] =
        static_cast<double>(counters.certified_occupied);
    diagnostics[prefix + "materializations"] =
        static_cast<double>(counters.materializations);
    diagnostics[prefix + "materialization_reused_external_evidence"] =
        static_cast<double>(counters.materialization_reused_external_evidence);
    diagnostics[prefix + "materialization_external_exact_hits"] =
        static_cast<double>(counters.materialization_external_exact_hits);
    diagnostics[prefix + "materialization_external_exact_misses"] =
        static_cast<double>(counters.materialization_external_exact_misses);
    diagnostics[prefix + "materialization_external_live_fallbacks"] =
        static_cast<double>(counters.materialization_external_live_fallbacks);
    diagnostics[prefix + "materialization_external_maybe_live_retries"] =
        static_cast<double>(counters.materialization_external_maybe_live_retries);
    diagnostics[prefix + "materialization_external_maybe_live_retry_free"] =
        static_cast<double>(counters.materialization_external_maybe_live_retry_free);
    diagnostics[prefix + "interval_replay_compatibility_checks"] =
        static_cast<double>(counters.interval_replay_compatibility_checks);
    diagnostics[prefix + "interval_replay_compatible"] =
        static_cast<double>(counters.interval_replay_compatible);
    diagnostics[prefix + "interval_replay_incompatible"] =
        static_cast<double>(counters.interval_replay_incompatible);
    diagnostics[prefix + "interval_replay_direct_exact_hits"] =
        static_cast<double>(counters.interval_replay_direct_exact_hits);
    diagnostics[prefix + "interval_replay_key_only_blocked"] =
        static_cast<double>(counters.interval_replay_key_only_blocked);
    diagnostics[prefix + "canonical_frame_invalid"] =
        static_cast<double>(counters.canonical_frame_invalid);
    diagnostics[prefix + "canonical_reflected_seed_misses"] =
        static_cast<double>(counters.canonical_reflected_seed_misses);
    diagnostics[prefix + "scoring_reused_external_evidence"] =
        static_cast<double>(counters.scoring_reused_external_evidence);
}

void record_depth_semantics_diagnostics(std::unordered_map<std::string, double>& diagnostics,
                                        const std::string& prefix,
                                        int sweep_start_depth,
                                        int sweep_max_depth,
                                        int target_max_depth,
                                        const FindFreeBoxOptions& seed_ffb_options,
                                        int deep_ffb_depth) {
    diagnostics[prefix + "sweep_start_depth"] =
        static_cast<double>(sweep_start_depth);
    diagnostics[prefix + "sweep_max_depth"] =
        static_cast<double>(sweep_max_depth);
    diagnostics[prefix + "target_max_depth"] =
        static_cast<double>(target_max_depth);
    diagnostics[prefix + "seed_ffb_start_depth"] =
        static_cast<double>(seed_ffb_options.start_depth);
    diagnostics[prefix + "seed_ffb_skip_to_depth"] =
        static_cast<double>(seed_ffb_options.skip_to_depth);
    diagnostics[prefix + "seed_ffb_max_depth"] =
        static_cast<double>(seed_ffb_options.max_depth);
    diagnostics[prefix + "deep_ffb_depth"] =
        static_cast<double>(deep_ffb_depth);
    diagnostics[prefix + "sweep_seed_ffb_depths_independent"] = 1.0;
}

void normalize_external_evidence_diagnostics(
    std::unordered_map<std::string, double>& diagnostics,
    const OracleCounters* active_oracle_counters = nullptr) {
    if (active_oracle_counters != nullptr) {
        record_oracle_cache_counter_snapshot(diagnostics,
                                             *active_oracle_counters,
                                             "adaptive.oracle.");
    }

    auto max_of = [&](std::initializer_list<const char*> keys) {
        double value = 0.0;
        for (const char* key : keys) {
            value = std::max(value, diagnostic_map_value(diagnostics, key));
        }
        return value;
    };

    const double reused_hits = max_of({
        "oracle.materialization_reused_external_evidence",
        "adaptive.oracle.materialization_reused_external_evidence",
        "leaf_sweep.worker_oracle.materialization_reused_external_evidence",
        "grower.worker_oracle.materialization_reused_external_evidence",
    });
    const double exact_hits = max_of({
        "oracle.materialization_external_exact_hits",
        "adaptive.oracle.materialization_external_exact_hits",
        "leaf_sweep.worker_oracle.materialization_external_exact_hits",
        "grower.worker_oracle.materialization_external_exact_hits",
    });
    const double exact_misses = max_of({
        "oracle.materialization_external_exact_misses",
        "adaptive.oracle.materialization_external_exact_misses",
        "leaf_sweep.worker_oracle.materialization_external_exact_misses",
        "grower.worker_oracle.materialization_external_exact_misses",
    });
    const double replay_checks = max_of({
        "oracle.interval_replay_compatibility_checks",
        "adaptive.oracle.interval_replay_compatibility_checks",
        "leaf_sweep.worker_oracle.interval_replay_compatibility_checks",
        "grower.worker_oracle.interval_replay_compatibility_checks",
    });
    const double replay_compatible = max_of({
        "oracle.interval_replay_compatible",
        "adaptive.oracle.interval_replay_compatible",
        "leaf_sweep.worker_oracle.interval_replay_compatible",
        "grower.worker_oracle.interval_replay_compatible",
    });
    const double replay_incompatible = max_of({
        "oracle.interval_replay_incompatible",
        "adaptive.oracle.interval_replay_incompatible",
        "leaf_sweep.worker_oracle.interval_replay_incompatible",
        "grower.worker_oracle.interval_replay_incompatible",
    });
    const double replay_direct_hits = max_of({
        "oracle.interval_replay_direct_exact_hits",
        "adaptive.oracle.interval_replay_direct_exact_hits",
        "leaf_sweep.worker_oracle.interval_replay_direct_exact_hits",
        "grower.worker_oracle.interval_replay_direct_exact_hits",
    });
    const double replay_key_only_blocked = max_of({
        "oracle.interval_replay_key_only_blocked",
        "adaptive.oracle.interval_replay_key_only_blocked",
        "leaf_sweep.worker_oracle.interval_replay_key_only_blocked",
        "grower.worker_oracle.interval_replay_key_only_blocked",
    });

    set_diagnostic_max(diagnostics,
                       "oracle.materialization_reused_external_evidence",
                       reused_hits);
    set_diagnostic_max(diagnostics,
                       "oracle.materialization_external_exact_hits",
                       exact_hits);
    set_diagnostic_max(diagnostics,
                       "oracle.materialization_external_exact_misses",
                       exact_misses);
    diagnostics["adaptive.external_reused_hits_normalized"] = reused_hits;
    diagnostics["adaptive.external_exact_hits_normalized"] = exact_hits;
    diagnostics["adaptive.external_exact_misses_normalized"] = exact_misses;
    diagnostics["adaptive.interval_replay_compatibility_checks_normalized"] = replay_checks;
    diagnostics["adaptive.interval_replay_compatible_normalized"] = replay_compatible;
    diagnostics["adaptive.interval_replay_incompatible_normalized"] = replay_incompatible;
    diagnostics["adaptive.interval_replay_direct_exact_hits_normalized"] = replay_direct_hits;
    diagnostics["adaptive.interval_replay_key_only_blocked_normalized"] = replay_key_only_blocked;
}

std::vector<Eigen::VectorXd> collision_shortcut_path(const std::vector<Eigen::VectorXd>& path,
                                                     const CollisionChecker& checker,
                                                     int segment_resolution) {
    if (path.size() <= 2) {
        return path;
    }
    const int safe_resolution = std::max(1, segment_resolution);
    const std::size_t n = path.size();
    std::vector<double> dist(n, std::numeric_limits<double>::infinity());
    std::vector<int> parent(n, -1);
    using QueueItem = std::pair<double, int>;
    std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> queue;
    dist[0] = 0.0;
    queue.emplace(0.0, 0);
    while (!queue.empty()) {
        const auto [current_dist, i] = queue.top();
        queue.pop();
        if (current_dist > dist[static_cast<std::size_t>(i)] + 1e-12) {
            continue;
        }
        if (static_cast<std::size_t>(i) == n - 1) {
            break;
        }
        for (std::size_t j = static_cast<std::size_t>(i) + 1; j < n; ++j) {
            if (checker.check_segment(path[static_cast<std::size_t>(i)], path[j], safe_resolution)) {
                continue;
            }
            const double edge = (path[j] - path[static_cast<std::size_t>(i)]).norm();
            const double candidate = current_dist + edge;
            if (candidate + 1e-12 < dist[j]) {
                dist[j] = candidate;
                parent[j] = i;
                queue.emplace(candidate, static_cast<int>(j));
            }
        }
    }
    if (parent[n - 1] < 0) {
        return path;
    }
    std::vector<Eigen::VectorXd> reversed;
    bool reached_start = false;
    for (int at = static_cast<int>(n - 1); at >= 0; at = parent[static_cast<std::size_t>(at)]) {
        reversed.push_back(path[static_cast<std::size_t>(at)]);
        if (at == 0) {
            reached_start = true;
            break;
        }
    }
    if (!reached_start) {
        return path;
    }
    std::reverse(reversed.begin(), reversed.end());
    return reversed;
}

int collision_shortcut_resolution(const QueryConfig& config) {
    int resolution = std::max(1, config.collision_shortcut_resolution);
    if (config.strict_path_audit) {
        resolution = std::max(resolution, config.audit_resolution);
    }
    return resolution;
}

std::string effective_symmetry_descriptor(const RBFPlanningConfig& config) {
    if (!config.database.canonical_mode) {
        return {};
    }
    return config.database.symmetry_descriptor.empty()
        ? std::string(lect_database::kJointSymmetryNativeV1)
        : config.database.symmetry_descriptor;
}

RRTConnectConfig with_query_root_hull_domain(const RRTConnectConfig& config,
                                             const BoxOracle& oracle,
                                             const Eigen::Ref<const Eigen::VectorXd>& start,
                                             const Eigen::Ref<const Eigen::VectorXd>& goal) {
    RRTConnectConfig out = config;
    auto lhs = oracle.planning_intervals();
    auto rhs = oracle.planning_intervals();
    (void)start;
    (void)goal;
    if (lhs.size() == rhs.size()) {
        for (std::size_t index = 0; index < lhs.size(); ++index) {
            lhs[index] = lhs[index].hull(rhs[index]);
        }
    }
    out.domain_intervals = std::move(lhs);
    return out;
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
    // Fill residual gaps along the connector segment: when a box certified at a
    // seed is not face-adjacent to the current chain box, bisect and insert
    // intermediate connected boxes so the segment is fully covered by boxes.
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

namespace {

const SegmentEdge* find_segment_edge_by_id(const SegmentEdgeList& edges, int edge_id) {
    for (const auto& edge : edges) {
        if (edge.id == edge_id) {
            return &edge;
        }
    }
    return nullptr;
}

std::uint64_t partition_segment_pair_key_local(int lhs, int rhs) {
    if (lhs > rhs) {
        std::swap(lhs, rhs);
    }
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(lhs)) << 32) |
           static_cast<std::uint32_t>(rhs);
}

std::unordered_set<std::uint64_t> partition_segment_pair_set_local(const SegmentEdgeList& edges) {
    std::unordered_set<std::uint64_t> pairs;
    pairs.reserve(edges.size() * 2);
    for (const auto& edge : edges) {
        if (edge.source_box_id < 0 || edge.target_box_id < 0) {
            continue;
        }
        pairs.insert(partition_segment_pair_key_local(edge.source_box_id, edge.target_box_id));
    }
    return pairs;
}

Robot make_sbf_audit_robot(Robot robot) {
    return robot;
}

CollisionChecker make_audit_checker(const Robot& robot, const Scene& scene, const QueryConfig& query_config) {
    CollisionChecker checker(robot, scene);
    checker.set_collision_tolerance(query_config.audit_collision_tolerance);
    return checker;
}

const BoxNode* find_box_by_id(const std::vector<BoxNode>& boxes, int box_id) {
    for (const auto& box : boxes) {
        if (box.id == box_id) {
            return &box;
        }
    }
    return nullptr;
}

bool partition_boxes_connected_local(const BoxNode& lhs,
                                     const BoxNode& rhs,
                                     double tolerance) {
    const int nd = lhs.n_dims();
    if (nd != rhs.n_dims()) {
        return false;
    }
    int shared_dims = 0;
    int overlap_dims = 0;
    for (int dim = 0; dim < nd; ++dim) {
        const auto& lhs_iv = lhs.joint_intervals[static_cast<std::size_t>(dim)];
        const auto& rhs_iv = rhs.joint_intervals[static_cast<std::size_t>(dim)];
        const double overlap_lo = std::max(lhs_iv.lo, rhs_iv.lo);
        const double overlap_hi = std::min(lhs_iv.hi, rhs_iv.hi);
        if (overlap_hi < overlap_lo - tolerance) {
            return false;
        }
        if (overlap_hi - overlap_lo < tolerance) {
            shared_dims += 1;
        } else {
            overlap_dims += 1;
        }
    }
    return shared_dims >= 1 || overlap_dims == nd;
}

Eigen::VectorXd partition_shared_face_center_local(const BoxNode& lhs,
                                                   const BoxNode& rhs) {
    const int nd = lhs.n_dims();
    if (nd != rhs.n_dims()) {
        return (lhs.center() + rhs.center()) * 0.5;
    }
    Eigen::VectorXd center(nd);
    int face_dim = -1;
    for (int dim = 0; dim < nd; ++dim) {
        const auto& lhs_iv = lhs.joint_intervals[static_cast<std::size_t>(dim)];
        const auto& rhs_iv = rhs.joint_intervals[static_cast<std::size_t>(dim)];
        const double overlap_lo = std::max(lhs_iv.lo, rhs_iv.lo);
        const double overlap_hi = std::min(lhs_iv.hi, rhs_iv.hi);
        if (overlap_hi < overlap_lo - 1e-9) {
            return (lhs.center() + rhs.center()) * 0.5;
        }
        if (std::abs(overlap_hi - overlap_lo) <= 1e-9) {
            if (face_dim >= 0) {
                return (lhs.center() + rhs.center()) * 0.5;
            }
            face_dim = dim;
        }
        center[dim] = 0.5 * (overlap_lo + overlap_hi);
    }
    return face_dim >= 0 ? center : (lhs.center() + rhs.center()) * 0.5;
}

Eigen::VectorXd partition_transition_waypoint_local(const BoxNode& lhs,
                                                    const BoxNode& rhs,
                                                    const Eigen::Ref<const Eigen::VectorXd>& from,
                                                    const Eigen::Ref<const Eigen::VectorXd>& goal,
                                                    double tolerance) {
    const int nd = lhs.n_dims();
    if (nd != rhs.n_dims() || from.size() != nd) {
        return partition_shared_face_center_local(lhs, rhs);
    }
    if (!partition_boxes_connected_local(lhs, rhs, tolerance)) {
        return partition_shared_face_center_local(lhs, rhs);
    }
    Eigen::VectorXd target = rhs.center();
    if (goal.size() == nd) {
        target = goal;
    }
    Eigen::VectorXd overlap_mid(nd);
    Eigen::VectorXd overlap_lo(nd);
    Eigen::VectorXd overlap_hi(nd);
    for (int dim = 0; dim < nd; ++dim) {
        const auto& lhs_iv = lhs.joint_intervals[static_cast<std::size_t>(dim)];
        const auto& rhs_iv = rhs.joint_intervals[static_cast<std::size_t>(dim)];
        overlap_lo[dim] = std::max(lhs_iv.lo, rhs_iv.lo);
        overlap_hi[dim] = std::min(lhs_iv.hi, rhs_iv.hi);
        overlap_mid[dim] = 0.5 * (overlap_lo[dim] + overlap_hi[dim]);
    }
    const Eigen::VectorXd local_delta = target - from;
    const double denom = local_delta.squaredNorm();
    if (denom <= 1e-18) {
        return overlap_mid;
    }
    const double t = std::clamp((overlap_mid - from).dot(local_delta) / denom, 0.0, 1.0);
    Eigen::VectorXd waypoint = from + t * local_delta;
    for (int dim = 0; dim < nd; ++dim) {
        waypoint[dim] = std::clamp(waypoint[dim], overlap_lo[dim], overlap_hi[dim]);
    }
    return waypoint;
}

std::vector<Eigen::VectorXd> extract_partition_waypoints_local(
    const std::vector<int>& box_sequence,
    const std::vector<int>& segment_edge_sequence,
    const std::vector<BoxNode>& boxes,
    const SegmentEdgeList& segment_edges,
    const Eigen::Ref<const Eigen::VectorXd>& start,
    const Eigen::Ref<const Eigen::VectorXd>& goal,
    double adjacency_tolerance) {
    std::vector<Eigen::VectorXd> path;
    if (box_sequence.empty()) {
        return path;
    }
    std::unordered_map<int, const BoxNode*> box_by_id;
    box_by_id.reserve(boxes.size());
    for (const auto& box : boxes) {
        box_by_id[box.id] = &box;
    }
    std::unordered_map<int, const SegmentEdge*> edge_by_id;
    edge_by_id.reserve(segment_edges.size());
    for (const auto& edge : segment_edges) {
        edge_by_id[edge.id] = &edge;
    }
    auto box_ptr = [&](int id) -> const BoxNode* {
        const auto it = box_by_id.find(id);
        return it == box_by_id.end() ? nullptr : it->second;
    };
    auto edge_ptr = [&](std::size_t transition_index) -> const SegmentEdge* {
        if (transition_index >= segment_edge_sequence.size()) {
            return nullptr;
        }
        const int edge_id = segment_edge_sequence[transition_index];
        if (edge_id < 0) {
            return nullptr;
        }
        const auto it = edge_by_id.find(edge_id);
        return it == edge_by_id.end() ? nullptr : it->second;
    };
    auto append_if_new = [&](const Eigen::VectorXd& waypoint) {
        if (path.empty() || (path.back() - waypoint).norm() > 1e-12) {
            path.push_back(waypoint);
        }
    };
    path.push_back(start);
    for (std::size_t i = 1; i < box_sequence.size(); ++i) {
        const BoxNode* lhs_ptr = box_ptr(box_sequence[i - 1]);
        const BoxNode* rhs_ptr = box_ptr(box_sequence[i]);
        if (lhs_ptr == nullptr || rhs_ptr == nullptr) {
            continue;
        }
        const BoxNode& lhs = *lhs_ptr;
        const BoxNode& rhs = *rhs_ptr;
        if (const SegmentEdge* edge = edge_ptr(i - 1)) {
            std::vector<Eigen::VectorXd> edge_path = edge->waypoints;
            if (edge->source_box_id == rhs.id && edge->target_box_id == lhs.id) {
                std::reverse(edge_path.begin(), edge_path.end());
            }
            if (edge_path.empty()) {
                edge_path.push_back(lhs.center());
                edge_path.push_back(rhs.center());
            }
            for (const auto& waypoint : edge_path) {
                append_if_new(waypoint);
            }
            continue;
        }
        if (partition_boxes_connected_local(lhs, rhs, adjacency_tolerance)) {
            append_if_new(partition_transition_waypoint_local(lhs, rhs, path.back(), goal, adjacency_tolerance));
        } else {
            append_if_new(lhs.center());
            append_if_new(rhs.center());
        }
    }
    append_if_new(goal);
    return path;
}

Eigen::VectorXd closest_point_in_box(const BoxNode& box,
                                     const Eigen::Ref<const Eigen::VectorXd>& point) {
    Eigen::VectorXd out(point.size());
    for (int dim = 0; dim < point.size(); ++dim) {
        if (dim < static_cast<int>(box.joint_intervals.size())) {
            const auto& interval = box.joint_intervals[static_cast<std::size_t>(dim)];
            out[dim] = std::min(interval.hi, std::max(interval.lo, point[dim]));
        } else {
            out[dim] = point[dim];
        }
    }
    return out;
}

std::optional<std::pair<double, double>> segment_box_parameter_interval(
    const Eigen::Ref<const Eigen::VectorXd>& a,
    const Eigen::Ref<const Eigen::VectorXd>& b,
    const BoxNode& box,
    double tolerance) {
    if (box.n_dims() != a.size() || b.size() != a.size()) {
        return std::nullopt;
    }
    double lo = 0.0;
    double hi = 1.0;
    const Eigen::VectorXd delta = b - a;
    for (int dim = 0; dim < a.size(); ++dim) {
        const auto& interval = box.joint_intervals[static_cast<std::size_t>(dim)];
        const double slab_lo = interval.lo - tolerance;
        const double slab_hi = interval.hi + tolerance;
        if (std::abs(delta[dim]) < 1e-15) {
            if (a[dim] < slab_lo || a[dim] > slab_hi) {
                return std::nullopt;
            }
            continue;
        }
        double t0 = (slab_lo - a[dim]) / delta[dim];
        double t1 = (slab_hi - a[dim]) / delta[dim];
        if (t0 > t1) {
            std::swap(t0, t1);
        }
        lo = std::max(lo, t0);
        hi = std::min(hi, t1);
        if (lo > hi) {
            return std::nullopt;
        }
    }
    lo = std::max(0.0, lo);
    hi = std::min(1.0, hi);
    if (lo > hi) {
        return std::nullopt;
    }
    return std::pair<double, double>{lo, hi};
}

double certified_box_covered_segment_length(const Eigen::Ref<const Eigen::VectorXd>& a,
                                            const Eigen::Ref<const Eigen::VectorXd>& b,
                                            const std::vector<BoxNode>& boxes,
                                            double tolerance = 1e-9) {
    const double segment_length = (b - a).norm();
    if (segment_length <= 1e-15) {
        return 0.0;
    }
    std::vector<std::pair<double, double>> covered;
    covered.reserve(boxes.size());
    for (const auto& box : boxes) {
        if (box.safety_status != BoxSafetyStatus::CertifiedFree ||
            box.strict_audit_required) {
            continue;
        }
        auto interval = segment_box_parameter_interval(a, b, box, tolerance);
        if (interval && interval->second > interval->first) {
            covered.push_back(*interval);
        }
    }
    if (covered.empty()) {
        return 0.0;
    }
    std::sort(covered.begin(), covered.end());
    double covered_param = 0.0;
    double cur_lo = covered.front().first;
    double cur_hi = covered.front().second;
    for (std::size_t index = 1; index < covered.size(); ++index) {
        const auto [next_lo, next_hi] = covered[index];
        if (next_lo <= cur_hi + 1e-12) {
            cur_hi = std::max(cur_hi, next_hi);
        } else {
            covered_param += std::max(0.0, cur_hi - cur_lo);
            cur_lo = next_lo;
            cur_hi = next_hi;
        }
    }
    covered_param += std::max(0.0, cur_hi - cur_lo);
    return std::min(segment_length, std::max(0.0, covered_param) * segment_length);
}

double uncovered_segment_edge_length(const SegmentEdge& edge,
                                     const std::vector<BoxNode>& boxes,
                                     double tolerance = 1e-9) {
    if (edge.waypoints.size() < 2) {
        return edge.length;
    }
    double uncovered = 0.0;
    for (std::size_t index = 1; index < edge.waypoints.size(); ++index) {
        const auto& a = edge.waypoints[index - 1];
        const auto& b = edge.waypoints[index];
        const double segment_length = (b - a).norm();
        const double covered =
            certified_box_covered_segment_length(a, b, boxes, tolerance);
        uncovered += std::max(0.0, segment_length - covered);
    }
    return uncovered;
}

bool same_waypoint(const Eigen::VectorXd& lhs, const Eigen::VectorXd& rhs) {
    return lhs.size() == rhs.size() && (lhs - rhs).norm() <= 1e-10;
}

void append_waypoint_unique(std::vector<Eigen::VectorXd>& path, const Eigen::VectorXd& waypoint) {
    if (path.empty() || !same_waypoint(path.back(), waypoint)) {
        path.push_back(waypoint);
    }
}

std::vector<Eigen::VectorXd> densify_waypoint_path_local(const std::vector<Eigen::VectorXd>& path,
                                                         double max_step) {
    if (path.size() <= 1 || !(max_step > 0.0) || !std::isfinite(max_step)) {
        return path;
    }
    std::vector<Eigen::VectorXd> out;
    out.push_back(path.front());
    for (std::size_t index = 1; index < path.size(); ++index) {
        const Eigen::VectorXd& a = path[index - 1];
        const Eigen::VectorXd& b = path[index];
        const double length = (b - a).norm();
        const int count = std::max(1, static_cast<int>(std::ceil(length / max_step)));
        for (int sample = 1; sample <= count; ++sample) {
            const double u = static_cast<double>(sample) / static_cast<double>(count);
            Eigen::VectorXd point = a + u * (b - a);
            if ((out.back() - point).norm() > 1e-12) {
                out.push_back(std::move(point));
            }
        }
    }
    return out;
}

int env_int_or_default(const char* name, int fallback) {
    const char* raw = std::getenv(name);
    if (raw == nullptr || raw[0] == '\0') {
        return fallback;
    }
    char* end = nullptr;
    const long value = std::strtol(raw, &end, 10);
    if (end == raw) {
        return fallback;
    }
    return static_cast<int>(value);
}

double env_double_or_default(const char* name, double fallback) {
    const char* raw = std::getenv(name);
    if (raw == nullptr || raw[0] == '\0') {
        return fallback;
    }
    char* end = nullptr;
    const double value = std::strtod(raw, &end);
    if (end == raw || !std::isfinite(value)) {
        return fallback;
    }
    return value;
}

int env_indexed_int_or_default(const char* prefix, int index, int fallback) {
    if (index < 0) {
        return fallback;
    }
    const std::string indexed = std::string(prefix) + "_" + std::to_string(index);
    return env_int_or_default(indexed.c_str(), fallback);
}

double env_indexed_double_or_default(const char* prefix, int index, double fallback) {
    if (index < 0) {
        return fallback;
    }
    const std::string indexed = std::string(prefix) + "_" + std::to_string(index);
    return env_double_or_default(indexed.c_str(), fallback);
}

bool env_index_list_contains(const char* name, std::size_t target) {
    const char* raw = std::getenv(name);
    if (raw == nullptr || raw[0] == '\0') {
        return false;
    }
    std::stringstream stream(raw);
    std::string item;
    while (std::getline(stream, item, ',')) {
        item.erase(std::remove_if(item.begin(), item.end(), [](unsigned char c) {
            return std::isspace(c) != 0;
        }), item.end());
        if (item.empty()) {
            continue;
        }
        char* end = nullptr;
        const long value = std::strtol(item.c_str(), &end, 10);
        if (end != item.c_str() && value >= 0 &&
            static_cast<std::size_t>(value) == target) {
            return true;
        }
    }
    return false;
}

bool csv_index_list_contains(const std::string& csv, int value) {
    std::string compact = csv;
    compact.erase(std::remove_if(compact.begin(), compact.end(), [](unsigned char c) {
        return std::isspace(c) != 0;
    }), compact.end());
    std::string lowered = compact;
    std::transform(lowered.begin(), lowered.end(), lowered.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    if (compact.empty() || compact == "*" || lowered == "all") {
        return true;
    }
    std::stringstream stream(csv);
    std::string item;
    while (std::getline(stream, item, ',')) {
        item.erase(std::remove_if(item.begin(), item.end(), [](unsigned char c) {
            return std::isspace(c) != 0;
        }), item.end());
        if (item.empty()) {
            continue;
        }
        try {
            if (std::stoi(item) == value) {
                return true;
            }
        } catch (const std::exception&) {
        }
    }
    return false;
}

int env_index_list_value_or_default(const char* name, std::size_t position, int fallback) {
    const char* raw = std::getenv(name);
    if (raw == nullptr || raw[0] == '\0') {
        return fallback;
    }
    std::stringstream stream(raw);
    std::string item;
    std::size_t index = 0;
    while (std::getline(stream, item, ',')) {
        item.erase(std::remove_if(item.begin(), item.end(), [](unsigned char c) {
            return std::isspace(c) != 0;
        }), item.end());
        if (index == position) {
            if (item.empty()) {
                return fallback;
            }
            char* end = nullptr;
            const long value = std::strtol(item.c_str(), &end, 10);
            if (end != item.c_str()) {
                return static_cast<int>(value);
            }
            return fallback;
        }
        ++index;
    }
    return fallback;
}

struct PathAuditCheck {
    bool passed = false;
    int failed_segment_index = -1;
};

int effective_audit_segment_resolution(const Eigen::VectorXd& start,
                                       const Eigen::VectorXd& goal,
                                       int min_resolution,
                                       double segment_step) {
    const int safe_resolution = std::max(1, min_resolution);
    if (!(segment_step > 0.0) || !std::isfinite(segment_step)) {
        return safe_resolution;
    }
    const double distance = (goal - start).norm();
    if (!(distance > 0.0) || !std::isfinite(distance)) {
        return safe_resolution;
    }
    const int step_resolution = std::max(2, static_cast<int>(std::ceil(distance / segment_step)));
    return std::max(safe_resolution, step_resolution);
}

PathAuditCheck audit_waypoint_path(const std::vector<Eigen::VectorXd>& path,
                                   const CollisionChecker& checker,
                                   int resolution,
                                   double segment_step) {
    PathAuditCheck audit;
    if (path.empty()) {
        audit.failed_segment_index = 0;
        return audit;
    }
    const int safe_resolution = std::max(1, resolution);
    for (std::size_t index = 0; index < path.size(); ++index) {
        if (checker.check_config(path[index])) {
            audit.failed_segment_index = index == 0 ? 0 : static_cast<int>(index - 1);
            return audit;
        }
    }
    for (std::size_t index = 0; index + 1 < path.size(); ++index) {
        const int segment_resolution = effective_audit_segment_resolution(
            path[index],
            path[index + 1],
            safe_resolution,
            segment_step);
        if (checker.check_segment(path[index], path[index + 1], segment_resolution)) {
            audit.failed_segment_index = static_cast<int>(index);
            return audit;
        }
    }
    audit.passed = true;
    return audit;
}

constexpr int kSeedAttemptStride = 7919;
constexpr int kSeedQueryStride = 104729;
constexpr int kSeedRepairLocalOffset = 101;
constexpr int kSeedRepairGlobalOffset = 211;
constexpr int kSeedFinalSimplifyOffset = 307;
constexpr int kSeedQueryBridgeOffset = 401;
constexpr int kSeedBridgeSimplifyOffset = 503;
constexpr int kSeedBatchBridgeOffset = 601;
constexpr int kSeedDebugBridgeOffset = 701;
constexpr int kSeedCorridorRefineOffset = 809;

int derived_planner_seed(int base_seed,
                         int offset,
                         int attempt = 0,
                         int query_index = 0,
                         int extra = 0) {
    constexpr long long modulus = 2147483647LL;
    long long value = static_cast<long long>(base_seed);
    value += static_cast<long long>(offset);
    value += static_cast<long long>(attempt) * kSeedAttemptStride;
    value += static_cast<long long>(query_index) * kSeedQueryStride;
    value += static_cast<long long>(extra);
    value %= modulus;
    if (value < 0) {
        value += modulus;
    }
    return static_cast<int>(value);
}

std::vector<Eigen::VectorXd> best_audited_rrt_bridge_path(
    const Eigen::Ref<const Eigen::VectorXd>& start,
    const Eigen::Ref<const Eigen::VectorXd>& goal,
    const CollisionChecker& checker,
    const Robot& robot,
    StageContext& context,
    const RRTConnectConfig& base_config,
    int attempts,
    double total_timeout_ms,
    int seed_base,
    int audit_resolution,
    double audit_segment_step,
    const std::vector<RRTConnectConfig>* attempt_configs = nullptr,
    int seed_stride = 7919) {
    using Clock = std::chrono::steady_clock;
    std::vector<Eigen::VectorXd> best;
    double best_length = std::numeric_limits<double>::infinity();
    const int safe_attempts = std::max(1, attempts);
    const double safe_total_ms = total_timeout_ms > 0.0 ? total_timeout_ms : base_config.timeout_ms;

    if (context.executor().n_threads() > 1 && safe_attempts > 1) {
        const double per_attempt_ms =
            safe_total_ms > 0.0
                ? std::max(1.0, safe_total_ms / static_cast<double>(safe_attempts))
                : base_config.timeout_ms;
        std::vector<std::vector<Eigen::VectorXd>> audited_paths(static_cast<std::size_t>(safe_attempts));
        context.executor().parallel_for(0, safe_attempts, [&](int attempt) {
            if (context.should_stop()) {
                return;
            }
            RRTConnectConfig config =
                (attempt_configs != nullptr && !attempt_configs->empty())
                    ? (*attempt_configs)[static_cast<std::size_t>(attempt) % attempt_configs->size()]
                    : base_config;
            if (per_attempt_ms > 0.0) {
                config.timeout_ms = per_attempt_ms;
            }
            std::vector<Eigen::VectorXd> path =
                rrt_connect(start,
                            goal,
                            checker,
                            robot,
                            config,
                            seed_base + attempt * std::max(1, seed_stride),
                            context.native_cancel_flag());
            if (path.empty()) {
                return;
            }
            const PathAuditCheck audit =
                audit_waypoint_path(path, checker, audit_resolution, audit_segment_step);
            if (!audit.passed) {
                return;
            }
            audited_paths[static_cast<std::size_t>(attempt)] = std::move(path);
        });

        const int audited_successes = static_cast<int>(std::count_if(
            audited_paths.begin(),
            audited_paths.end(),
            [](const auto& path) { return !path.empty(); }));
        for (auto& path : audited_paths) {
            if (path.empty()) {
                continue;
            }
            const double length = path_length(path);
            if (length < best_length) {
                best_length = length;
                best = std::move(path);
            }
        }
        context.diagnostics().add_counter("query_bridge.parallel_rrt_attempts",
                                          static_cast<double>(safe_attempts));
        context.diagnostics().add_counter("query_bridge.parallel_rrt_successes",
                                          static_cast<double>(audited_successes));
        return best;
    }

    const auto t0 = Clock::now();
    auto elapsed_ms = [&]() {
        return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
    };
    for (int attempt = 0; attempt < safe_attempts; ++attempt) {
        if (context.should_stop()) {
            break;
        }
        RRTConnectConfig config =
            (attempt_configs != nullptr && !attempt_configs->empty())
                ? (*attempt_configs)[static_cast<std::size_t>(attempt) % attempt_configs->size()]
                : base_config;
        if (safe_total_ms > 0.0) {
            const double remaining_ms = safe_total_ms - elapsed_ms();
            if (remaining_ms <= 0.0) {
                break;
            }
            const int attempts_left = safe_attempts - attempt;
            config.timeout_ms = std::max(1.0, remaining_ms / static_cast<double>(attempts_left));
        }
        std::vector<Eigen::VectorXd> path =
            rrt_connect(start,
                        goal,
                        checker,
                        robot,
                        context,
                        config,
                        seed_base + attempt * std::max(1, seed_stride));
        if (path.empty()) {
            continue;
        }
        PathAuditCheck audit = audit_waypoint_path(path, checker, audit_resolution, audit_segment_step);
        if (!audit.passed) {
            continue;
        }
        const double length = path_length(path);
        if (length < best_length) {
            best_length = length;
            best = std::move(path);
        }
    }
    return best;
}

bool collision_bracket(const Eigen::VectorXd& lhs,
                       const Eigen::VectorXd& rhs,
                       const CollisionChecker& checker,
                       int resolution,
                       Eigen::VectorXd& repair_start,
                       Eigen::VectorXd& repair_goal) {
    if (!checker.check_config(lhs) && !checker.check_config(rhs)) {
        repair_start = lhs;
        repair_goal = rhs;
        return true;
    }
    const int samples = std::max(4, resolution);
    const Eigen::VectorXd diff = rhs - lhs;
    int first_collision = -1;
    int last_collision = -1;
    for (int sample = 0; sample <= samples; ++sample) {
        const double t = static_cast<double>(sample) / static_cast<double>(samples);
        const Eigen::VectorXd q = lhs + t * diff;
        if (checker.check_config(q)) {
            if (first_collision < 0) {
                first_collision = sample;
            }
            last_collision = sample;
        }
    }
    if (first_collision <= 0 || last_collision < 0 || last_collision >= samples) {
        return false;
    }
    const double t0 = static_cast<double>(first_collision - 1) / static_cast<double>(samples);
    const double t1 = static_cast<double>(last_collision + 1) / static_cast<double>(samples);
    repair_start = lhs + t0 * diff;
    repair_goal = lhs + t1 * diff;
    return !checker.check_config(repair_start) && !checker.check_config(repair_goal);
}

Obstacle inflate_obstacle(const Obstacle& obstacle, double padding) {
    const float pad = static_cast<float>(std::max(0.0, padding));
    return Obstacle(obstacle.bounds[0] - pad,
                    obstacle.bounds[1] - pad,
                    obstacle.bounds[2] - pad,
                    obstacle.bounds[3] + pad,
                    obstacle.bounds[4] + pad,
                    obstacle.bounds[5] + pad);
}

double interval_point_gap_local(const Interval& interval, double value) {
    if (value < interval.lo) {
        return interval.lo - value;
    }
    if (value > interval.hi) {
        return value - interval.hi;
    }
    return 0.0;
}

double intervals_point_gap_local(const std::vector<Interval>& intervals,
                                 const Eigen::Ref<const Eigen::VectorXd>& point) {
    if (point.size() != static_cast<int>(intervals.size())) {
        return std::numeric_limits<double>::infinity();
    }
    double gap = 0.0;
    for (int dim = 0; dim < point.size(); ++dim) {
        gap = std::max(gap, interval_point_gap_local(intervals[static_cast<std::size_t>(dim)], point[dim]));
    }
    return gap;
}

bool intervals_contain_point_local(const std::vector<Interval>& intervals,
                                   const Eigen::Ref<const Eigen::VectorXd>& point,
                                   double tolerance) {
    return intervals_point_gap_local(intervals, point) <= tolerance;
}

constexpr double kQueryRootBoundaryContainmentTolerance = 1e-3;

bool intervals_contain_point_with_boundary_tolerance(const std::vector<Interval>& intervals,
                                                     const Eigen::Ref<const Eigen::VectorXd>& point,
                                                     double tolerance) {
    return intervals_point_gap_local(intervals, point) <=
        std::max(tolerance, kQueryRootBoundaryContainmentTolerance);
}

bool expand_intervals_to_contain_boundary_seed(std::vector<Interval>& intervals,
                                               const Eigen::Ref<const Eigen::VectorXd>& point,
                                               double tolerance) {
    if (point.size() != static_cast<int>(intervals.size())) {
        return false;
    }
    bool expanded = false;
    const double allowed_gap = std::max(tolerance, kQueryRootBoundaryContainmentTolerance);
    for (int dim = 0; dim < point.size(); ++dim) {
        auto& interval = intervals[static_cast<std::size_t>(dim)];
        if (point[dim] < interval.lo) {
            const double gap = interval.lo - point[dim];
            if (gap > allowed_gap) {
                return false;
            }
            interval.lo = point[dim];
            expanded = true;
        } else if (point[dim] > interval.hi) {
            const double gap = point[dim] - interval.hi;
            if (gap > allowed_gap) {
                return false;
            }
            interval.hi = point[dim];
            expanded = true;
        }
    }
    return expanded;
}

double interval_bounds_gap_squared_local(const std::vector<Interval>& lhs,
                                         const std::vector<Interval>& rhs) {
    if (lhs.size() != rhs.size()) {
        return std::numeric_limits<double>::infinity();
    }
    double gap_sq = 0.0;
    for (int dim = 0; dim < static_cast<int>(lhs.size()); ++dim) {
        double gap = 0.0;
        const auto& left = lhs[static_cast<std::size_t>(dim)];
        const auto& right = rhs[static_cast<std::size_t>(dim)];
        if (left.hi < right.lo) {
            gap = right.lo - left.hi;
        } else if (right.hi < left.lo) {
            gap = left.lo - right.hi;
        }
        gap_sq += gap * gap;
    }
    return gap_sq;
}

bool box_contains_box_exact_local(const BoxNode& outer, const BoxNode& inner) {
    if (outer.n_dims() != inner.n_dims()) {
        return false;
    }
    for (int dim = 0; dim < outer.n_dims(); ++dim) {
        if (outer.joint_intervals[dim].lo > inner.joint_intervals[dim].lo ||
            outer.joint_intervals[dim].hi < inner.joint_intervals[dim].hi) {
            return false;
        }
    }
    return outer.id != inner.id;
}

bool box_contains_point_exact_local(const BoxNode& box, const Eigen::Ref<const Eigen::VectorXd>& point) {
    if (box.n_dims() != point.size()) {
        return false;
    }
    for (int dim = 0; dim < box.n_dims(); ++dim) {
        if (point[dim] < box.joint_intervals[dim].lo || point[dim] > box.joint_intervals[dim].hi) {
            return false;
        }
    }
    return true;
}

bool point_covered_by_existing_box_local(const std::vector<BoxNode>& boxes,
                                         const Eigen::Ref<const Eigen::VectorXd>& point) {
    return std::any_of(boxes.begin(), boxes.end(), [&](const BoxNode& box) {
        return box_contains_point_exact_local(box, point);
    });
}

struct SubtractiveSeedCandidate {
    Eigen::VectorXd seed;
    int parent_box_id = -1;
    int root_id = -1;
    int domain_index = -1;
};

Eigen::VectorXd clamped_domain_seed(const BoxNode& domain,
                                    const Eigen::Ref<const Eigen::VectorXd>& point,
                                    double epsilon);

bool intervals_subset_local(const std::vector<Interval>& inner,
                            const std::vector<Interval>& outer,
                            double tolerance = 0.0) {
    if (inner.size() != outer.size()) {
        return false;
    }
    for (std::size_t dim = 0; dim < inner.size(); ++dim) {
        if (inner[dim].lo < outer[dim].lo - tolerance ||
            inner[dim].hi > outer[dim].hi + tolerance) {
            return false;
        }
    }
    return true;
}

bool intervals_equal_local(const std::vector<Interval>& lhs,
                           const std::vector<Interval>& rhs,
                           double tolerance = 0.0) {
    if (lhs.size() != rhs.size()) {
        return false;
    }
    for (std::size_t dim = 0; dim < lhs.size(); ++dim) {
        if (std::abs(lhs[dim].lo - rhs[dim].lo) > tolerance ||
            std::abs(lhs[dim].hi - rhs[dim].hi) > tolerance) {
            return false;
        }
    }
    return true;
}

bool clip_intervals_to_domain_local(std::vector<Interval>& intervals,
                                    const std::vector<Interval>& domain) {
    if (domain.empty()) {
        return !intervals.empty();
    }
    if (intervals.size() != domain.size()) {
        return false;
    }
    for (std::size_t dim = 0; dim < intervals.size(); ++dim) {
        intervals[dim].lo = std::max(intervals[dim].lo, domain[dim].lo);
        intervals[dim].hi = std::min(intervals[dim].hi, domain[dim].hi);
        if (intervals[dim].lo > intervals[dim].hi) {
            return false;
        }
    }
    return true;
}

bool intervals_overlap_local(const std::vector<Interval>& lhs,
                             const std::vector<Interval>& rhs,
                             double tolerance = 0.0) {
    if (lhs.size() != rhs.size()) {
        return false;
    }
    for (std::size_t dim = 0; dim < lhs.size(); ++dim) {
        if (!lhs[dim].overlaps(rhs[dim], tolerance)) {
            return false;
        }
    }
    return true;
}

bool intervals_contain_point_strict_local(const std::vector<Interval>& intervals,
                                          const Eigen::Ref<const Eigen::VectorXd>& point,
                                          double tolerance = 0.0) {
    if (point.size() != static_cast<int>(intervals.size())) {
        return false;
    }
    for (int dim = 0; dim < point.size(); ++dim) {
        if (point[dim] < intervals[static_cast<std::size_t>(dim)].lo - tolerance ||
            point[dim] > intervals[static_cast<std::size_t>(dim)].hi + tolerance) {
            return false;
        }
    }
    return true;
}

bool graph_has_box_path_local(const AdjacencyGraph& graph, int start_id, int goal_id) {
    if (start_id < 0 || goal_id < 0) {
        return false;
    }
    if (start_id == goal_id) {
        return true;
    }
    std::queue<int> queue;
    std::unordered_set<int> visited;
    queue.push(start_id);
    visited.insert(start_id);
    while (!queue.empty()) {
        const int current = queue.front();
        queue.pop();
        auto it = graph.find(current);
        if (it == graph.end()) {
            continue;
        }
        for (int next : it->second) {
            if (next == goal_id) {
                return true;
            }
            if (visited.insert(next).second) {
                queue.push(next);
            }
        }
    }
    return false;
}

bool graph_has_certified_box_path_local(const std::vector<BoxNode>& boxes,
                                        const AdjacencyGraph& graph,
                                        int start_id,
                                        int goal_id,
                                        double adjacency_tolerance) {
    if (start_id < 0 || goal_id < 0) {
        return false;
    }
    if (start_id == goal_id) {
        return true;
    }
    std::queue<int> queue;
    std::unordered_set<int> visited;
    queue.push(start_id);
    visited.insert(start_id);
    while (!queue.empty()) {
        const int current = queue.front();
        queue.pop();
        auto it = graph.find(current);
        if (it == graph.end()) {
            continue;
        }
        const BoxNode* current_box = find_box_by_id(boxes, current);
        if (current_box == nullptr) {
            continue;
        }
        for (int next : it->second) {
            const BoxNode* next_box = find_box_by_id(boxes, next);
            if (next_box == nullptr ||
                !boxes_connected(*current_box, *next_box, adjacency_tolerance)) {
                continue;
            }
            if (next == goal_id) {
                return true;
            }
            if (visited.insert(next).second) {
                queue.push(next);
            }
        }
    }
    return false;
}

double box_priority_point_distance(const BoxNode& box,
                                   const std::vector<Eigen::VectorXd>& priority_points) {
    if (priority_points.empty()) {
        return std::numeric_limits<double>::infinity();
    }
    double best = std::numeric_limits<double>::infinity();
    for (const auto& point : priority_points) {
        if (point.size() != box.n_dims()) {
            continue;
        }
        best = std::min(best, intervals_point_gap_local(box.joint_intervals, point));
    }
    return best;
}

struct PriorityPruneStats {
    int free_before = 0;
    int free_after = 0;
    int collision_before = 0;
    int collision_after = 0;
};

PriorityPruneStats prune_leaf_sweep_to_priority(LeafSweepResult& result,
                                                std::vector<BoxNode>& live_boxes,
                                                std::vector<BoxNode>& raw_boxes,
                                                const std::vector<Eigen::VectorXd>& priority_points,
                                                double radius) {
    PriorityPruneStats stats;
    stats.free_before = static_cast<int>(result.free_boxes.size());
    stats.collision_before = static_cast<int>(result.collision_boxes.size());
    if (priority_points.empty() || !(radius > 0.0)) {
        stats.free_after = stats.free_before;
        stats.collision_after = stats.collision_before;
        return stats;
    }
    auto keep_box = [&](const BoxNode& box) {
        return box_priority_point_distance(box, priority_points) <= radius;
    };
    std::vector<BoxNode> kept_free;
    kept_free.reserve(result.free_boxes.size());
    for (const auto& box : result.free_boxes) {
        if (keep_box(box)) {
            kept_free.push_back(box);
        }
    }
    std::vector<BoxNode> kept_collision;
    std::vector<std::vector<int>> kept_collision_indices;
    kept_collision.reserve(result.collision_boxes.size());
    kept_collision_indices.reserve(result.collision_box_obstacle_indices.size());
    for (std::size_t index = 0; index < result.collision_boxes.size(); ++index) {
        const auto& box = result.collision_boxes[index];
        if (!keep_box(box)) {
            continue;
        }
        kept_collision.push_back(box);
        if (index < result.collision_box_obstacle_indices.size()) {
            kept_collision_indices.push_back(result.collision_box_obstacle_indices[index]);
        } else {
            kept_collision_indices.emplace_back();
        }
    }
    result.free_boxes = std::move(kept_free);
    result.collision_boxes = std::move(kept_collision);
    result.collision_box_obstacle_indices = std::move(kept_collision_indices);
    live_boxes = result.free_boxes;
    raw_boxes = live_boxes;
    stats.free_after = static_cast<int>(result.free_boxes.size());
    stats.collision_after = static_cast<int>(result.collision_boxes.size());
    return stats;
}

bool leaf_refine_has_adjacency(const std::vector<BoxNode>& boxes,
                               const BoxNode& box,
                               double tolerance,
                               int* parent_box_id) {
    for (const auto& existing : boxes) {
        if (boxes_connected(existing, box, tolerance)) {
            if (parent_box_id != nullptr) {
                *parent_box_id = existing.id;
            }
            return true;
        }
    }
    return false;
}

int containing_domain_index(const std::vector<BoxNode>& domains,
                            const Eigen::Ref<const Eigen::VectorXd>& point,
                            double tolerance) {
    for (int index = 0; index < static_cast<int>(domains.size()); ++index) {
        if (intervals_contain_point_strict_local(domains[static_cast<std::size_t>(index)].joint_intervals,
                                                point,
                                                tolerance)) {
            return index;
        }
    }
    return -1;
}

int containing_domain_index(const std::vector<BoxNode>& domains,
                            const std::vector<Interval>& intervals,
                            double tolerance) {
    for (int index = 0; index < static_cast<int>(domains.size()); ++index) {
        if (intervals_subset_local(intervals,
                                   domains[static_cast<std::size_t>(index)].joint_intervals,
                                   tolerance)) {
            return index;
        }
    }
    return -1;
}

bool seed_near_existing(const std::vector<SubtractiveSeedCandidate>& candidates,
                        const Eigen::Ref<const Eigen::VectorXd>& seed,
                        double tolerance) {
    for (const auto& candidate : candidates) {
        if (candidate.seed.size() == seed.size() && (candidate.seed - seed).norm() <= tolerance) {
            return true;
        }
    }
    return false;
}

bool append_subtractive_seed_candidate(std::vector<SubtractiveSeedCandidate>& seeds,
                                       const std::vector<BoxNode>& live_boxes,
                                       const std::vector<BoxNode>& domains,
                                       const Eigen::Ref<const Eigen::VectorXd>& seed,
                                       int parent_box_id,
                                       int root_id,
                                       int limit,
                                       double dedup_tolerance,
                                       double domain_tolerance) {
    if (static_cast<int>(seeds.size()) >= limit) {
        return false;
    }
    const int domain_index = containing_domain_index(domains, seed, domain_tolerance);
    if (domain_index < 0 || point_covered_by_existing_box_local(live_boxes, seed) ||
        seed_near_existing(seeds, seed, dedup_tolerance)) {
        return false;
    }
    seeds.push_back(SubtractiveSeedCandidate{seed, parent_box_id, root_id, domain_index});
    return true;
}

Eigen::VectorXd clamped_domain_seed(const BoxNode& domain,
                                    const Eigen::Ref<const Eigen::VectorXd>& point,
                                    double epsilon) {
    Eigen::VectorXd seed = point;
    if (seed.size() != domain.n_dims()) {
        seed = domain.center();
    }
    for (int dim = 0; dim < domain.n_dims(); ++dim) {
        const auto& interval = domain.joint_intervals[static_cast<std::size_t>(dim)];
        const double width = std::max(0.0, interval.width());
        const double inset = std::min(std::max(epsilon, 1e-12), 0.25 * width);
        if (width > 2.0 * inset) {
            seed[dim] = std::clamp(seed[dim], interval.lo + inset, interval.hi - inset);
        } else {
            seed[dim] = interval.center();
        }
    }
    return seed;
}

std::vector<SubtractiveSeedCandidate> make_subtractive_regrow_seeds(
    const std::vector<BoxNode>& live_boxes,
    const std::vector<BoxNode>& removed_boxes,
    const std::unordered_set<int>& removed_box_ids,
    const AdjacencyGraph& previous_adjacency,
    const std::vector<Eigen::VectorXd>& anchor_points,
    const DynamicUpdateConfig& config,
    double adjacency_tolerance,
    double boundary_epsilon) {
    std::vector<SubtractiveSeedCandidate> seeds;
    const int limit = std::max(0, config.dirty_seed_limit);
    if (limit == 0 || removed_boxes.empty()) {
        return seeds;
    }
    seeds.reserve(static_cast<std::size_t>(std::min(limit, 128)));
    const double epsilon = std::max({boundary_epsilon, 2.0 * adjacency_tolerance, 1e-10});
    const double dedup_tol = std::max(1e-9, 4.0 * epsilon);
    const double domain_tol = std::max(1e-10, adjacency_tolerance);
    std::unordered_map<int, const BoxNode*> live_by_id;
    live_by_id.reserve(live_boxes.size());
    for (const auto& box : live_boxes) {
        live_by_id[box.id] = &box;
    }

    for (int domain_index = 0; domain_index < static_cast<int>(removed_boxes.size()); ++domain_index) {
        const BoxNode& domain = removed_boxes[static_cast<std::size_t>(domain_index)];
        append_subtractive_seed_candidate(seeds,
                                          live_boxes,
                                          removed_boxes,
                                          domain.center(),
                                          -1,
                                          domain.root_id >= 0 ? domain.root_id : domain.id,
                                          limit,
                                          dedup_tol,
                                          domain_tol);

        for (int dim = 0; dim < domain.n_dims(); ++dim) {
            for (int side : {-1, 1}) {
                Eigen::VectorXd seed = domain.center();
                const auto& interval = domain.joint_intervals[static_cast<std::size_t>(dim)];
                seed[dim] = side < 0
                    ? interval.lo + std::min(std::max(epsilon, 1e-12), 0.25 * std::max(0.0, interval.width()))
                    : interval.hi - std::min(std::max(epsilon, 1e-12), 0.25 * std::max(0.0, interval.width()));
                append_subtractive_seed_candidate(seeds,
                                                  live_boxes,
                                                  removed_boxes,
                                                  seed,
                                                  -1,
                                                  domain.root_id >= 0 ? domain.root_id : domain.id,
                                                  limit,
                                                  dedup_tol,
                                                  domain_tol);
                if (static_cast<int>(seeds.size()) >= limit) {
                    return seeds;
                }
            }
        }

        auto adjacency_it = previous_adjacency.find(domain.id);
        if (adjacency_it != previous_adjacency.end()) {
            for (int neighbor_id : adjacency_it->second) {
                if (removed_box_ids.find(neighbor_id) != removed_box_ids.end()) {
                    continue;
                }
                auto live_it = live_by_id.find(neighbor_id);
                if (live_it == live_by_id.end()) {
                    continue;
                }
                const BoxNode& neighbor = *live_it->second;
                Eigen::VectorXd seed = clamped_domain_seed(domain, neighbor.center(), epsilon);
                append_subtractive_seed_candidate(seeds,
                                                  live_boxes,
                                                  removed_boxes,
                                                  seed,
                                                  neighbor.id,
                                                  neighbor.root_id >= 0 ? neighbor.root_id : neighbor.id,
                                                  limit,
                                                  dedup_tol,
                                                  domain_tol);
                if (static_cast<int>(seeds.size()) >= limit) {
                    return seeds;
                }
            }
        }
    }

    for (const auto& point : anchor_points) {
        const int domain_index = containing_domain_index(removed_boxes, point, domain_tol);
        if (domain_index < 0) {
            continue;
        }
        const BoxNode& domain = removed_boxes[static_cast<std::size_t>(domain_index)];
        const Eigen::VectorXd seed = clamped_domain_seed(domain, point, epsilon);
        append_subtractive_seed_candidate(seeds,
                                          live_boxes,
                                          removed_boxes,
                                          seed,
                                          -1,
                                          domain.root_id >= 0 ? domain.root_id : domain.id,
                                          limit,
                                          dedup_tol,
                                          domain_tol);
        if (static_cast<int>(seeds.size()) >= limit) {
            return seeds;
        }
    }
    return seeds;
}

void append_local_edge(AdjacencyGraph& graph, int lhs, int rhs) {
    if (lhs < 0 || rhs < 0 || lhs == rhs) {
        return;
    }
    auto append_one = [](std::vector<int>& neighbors, int value) {
        if (std::find(neighbors.begin(), neighbors.end(), value) == neighbors.end()) {
            neighbors.push_back(value);
        }
    };
    append_one(graph[lhs], rhs);
    append_one(graph[rhs], lhs);
}

void remove_local_edge(AdjacencyGraph& graph, int lhs, int rhs) {
    auto erase_one = [&](int from, int to) {
        auto it = graph.find(from);
        if (it == graph.end()) {
            return;
        }
        auto& neighbors = it->second;
        neighbors.erase(std::remove(neighbors.begin(), neighbors.end(), to), neighbors.end());
    };
    erase_one(lhs, rhs);
    erase_one(rhs, lhs);
}

void remove_adjacency_nodes(AdjacencyGraph& graph, const std::unordered_set<int>& removed_ids) {
    for (int id : removed_ids) {
        graph.erase(id);
    }
    if (removed_ids.empty()) {
        return;
    }
    for (auto& [_, neighbors] : graph) {
        neighbors.erase(std::remove_if(neighbors.begin(), neighbors.end(), [&](int id) {
            return removed_ids.find(id) != removed_ids.end();
        }), neighbors.end());
    }
}

void connect_incremental_boxes(AdjacencyGraph& graph,
                               const std::vector<BoxNode>& boxes,
                               std::size_t first_new_index,
                               double tolerance) {
    if (first_new_index >= boxes.size()) {
        return;
    }
    for (const auto& box : boxes) {
        graph[box.id];
    }
    for (std::size_t i = first_new_index; i < boxes.size(); ++i) {
        for (std::size_t j = 0; j < i; ++j) {
            if (boxes_connected(boxes[j], boxes[i], tolerance)) {
                append_local_edge(graph, boxes[j].id, boxes[i].id);
            }
        }
    }
}

Eigen::VectorXd adaptive_center_of_intervals(const std::vector<Interval>& intervals) {
    Eigen::VectorXd center(static_cast<int>(intervals.size()));
    for (int dim = 0; dim < center.size(); ++dim) {
        center[dim] = intervals[static_cast<std::size_t>(dim)].center();
    }
    return center;
}

double adaptive_interval_volume(const std::vector<Interval>& intervals) {
    double volume = 1.0;
    for (const auto& interval : intervals) {
        volume *= std::max(0.0, interval.width());
    }
    return volume;
}

int adaptive_virtual_depth(OracleNodeId node) {
    if (node <= 0) {
        return 0;
    }
    std::uint64_t value = static_cast<std::uint64_t>(node) + 1u;
    int depth = -1;
    while (value != 0u) {
        value >>= 1u;
        ++depth;
    }
    return std::max(0, depth);
}

struct AdaptiveFrontierItem {
    OracleNodeId node = -1;
    std::vector<Interval> intervals;
    int changed_dim = -1;
    int free_seed_hits = 0;
    double overlap_depth = 0.0;
    double overlap_ratio = 0.0;
    double score = 0.0;
};

struct AdaptiveDepthSnapshot {
    int depth = 0;
    int free_probe_count = 0;
    int covered_count = 0;
    int main_accessible_count = 0;
    int anchor_success_count = 0;
    int anchor_to_main_count = 0;
    int anchor_probe_attempts = 0;
    int cell_count = 0;
    int collision_count = 0;
    int island_count = 0;
    int main_island_cell_count = 0;
    double p_box_covered = 0.0;
    double p_main_accessible = 0.0;
    double main_connected_ratio = 0.0;
    double p_anchor_to_main_uncovered = 0.0;
    double probe_ms = 0.0;
    bool readiness_met = false;
    std::string stop_reason;
};

std::string adaptive_depth_snapshots_to_json(const std::vector<AdaptiveDepthSnapshot>& snapshots) {
    std::ostringstream out;
    out << '[';
    for (std::size_t i = 0; i < snapshots.size(); ++i) {
        const auto& snap = snapshots[i];
        if (i > 0) {
            out << ',';
        }
        out << '{'
            << "\"depth\":" << snap.depth
            << ",\"free_probe_count\":" << snap.free_probe_count
            << ",\"covered_count\":" << snap.covered_count
            << ",\"main_accessible_count\":" << snap.main_accessible_count
            << ",\"anchor_success_count\":" << snap.anchor_success_count
            << ",\"anchor_to_main_count\":" << snap.anchor_to_main_count
            << ",\"anchor_probe_attempts\":" << snap.anchor_probe_attempts
            << ",\"cell_count\":" << snap.cell_count
            << ",\"collision_count\":" << snap.collision_count
            << ",\"island_count\":" << snap.island_count
            << ",\"main_island_cell_count\":" << snap.main_island_cell_count
            << ",\"p_box_covered\":" << snap.p_box_covered
            << ",\"p_main_accessible\":" << snap.p_main_accessible
            << ",\"main_connected_ratio\":" << snap.main_connected_ratio
            << ",\"p_anchor_to_main_uncovered\":" << snap.p_anchor_to_main_uncovered
            << ",\"probe_ms\":" << snap.probe_ms
            << ",\"readiness_met\":" << (snap.readiness_met ? "true" : "false")
            << ",\"stop_reason\":\"" << snap.stop_reason << "\""
            << '}';
    }
    out << ']';
    return out.str();
}

bool adaptive_virtual_split_node(const lect_database::SplitPolicyDescriptor& descriptor,
                                 const AdaptiveFrontierItem& item,
                                 AdaptiveFrontierItem& left,
                                 AdaptiveFrontierItem& right) {
    if (item.node < 0 || item.intervals.empty()) {
        return false;
    }
    const int depth = adaptive_virtual_depth(item.node);
    int split_dim = -1;
    if (!descriptor.depth_dimensions.empty() &&
        depth >= 0 &&
        depth < static_cast<int>(descriptor.depth_dimensions.size())) {
        split_dim = descriptor.depth_dimensions[static_cast<std::size_t>(depth)];
    } else {
        split_dim = depth % static_cast<int>(item.intervals.size());
    }
    if (split_dim < 0 || split_dim >= static_cast<int>(item.intervals.size())) {
        return false;
    }
    const auto dim = static_cast<std::size_t>(split_dim);
    const double split_value = item.intervals[dim].center();
    if (!(split_value > item.intervals[dim].lo && split_value < item.intervals[dim].hi)) {
        return false;
    }
    left = item;
    right = item;
    left.node = static_cast<OracleNodeId>(2 * static_cast<std::uint64_t>(item.node) + 1u);
    right.node = static_cast<OracleNodeId>(2 * static_cast<std::uint64_t>(item.node) + 2u);
    left.changed_dim = split_dim;
    right.changed_dim = split_dim;
    left.intervals[dim].hi = split_value;
    right.intervals[dim].lo = split_value;
    left.free_seed_hits = 0;
    right.free_seed_hits = 0;
    left.overlap_depth = 0.0;
    right.overlap_depth = 0.0;
    left.overlap_ratio = 0.0;
    right.overlap_ratio = 0.0;
    left.score = 0.0;
    right.score = 0.0;
    return true;
}

BoxNode adaptive_make_box_from_intervals(const std::vector<Interval>& intervals,
                                         OracleNodeId node,
                                         int id,
                                         BoxSafetyStatus status,
                                         bool strict_audit_required) {
    BoxNode box;
    box.id = id;
    box.joint_intervals = intervals;
    box.seed_config = adaptive_center_of_intervals(intervals);
    box.tree_id = node;
    box.parent_box_id = -1;
    box.root_id = id;
    box.safety_status = status;
    box.strict_audit_required = strict_audit_required;
    box.compute_volume();
    return box;
}

std::unordered_set<int> adaptive_largest_island_ids(const AdjacencyGraph& graph) {
    std::unordered_set<int> ids;
    const auto islands = find_islands(graph);
    if (islands.empty()) {
        return ids;
    }
    const auto* largest = &islands.front();
    for (const auto& island : islands) {
        if (island.size() > largest->size()) {
            largest = &island;
        }
    }
    ids.insert(largest->begin(), largest->end());
    return ids;
}

bool adaptive_has_adjacency_to_any(const std::vector<BoxNode>& boxes,
                                   const BoxNode& candidate,
                                   const std::unordered_set<int>* allowed_ids,
                                   double tolerance) {
    for (const auto& existing : boxes) {
        if (allowed_ids != nullptr && allowed_ids->find(existing.id) == allowed_ids->end()) {
            continue;
        }
        if (boxes_connected(existing, candidate, tolerance)) {
            return true;
        }
    }
    return false;
}

struct AdaptiveConnectivityDominance {
    int adjacent_free = 0;
    int adjacent_main = 0;
    int adjacent_other = 0;
    bool has_free_context = false;
    bool connector_candidate = false;
    bool single_component = false;
    bool isolated = true;
    double priority_delta = 0.0;
};

AdaptiveConnectivityDominance adaptive_connectivity_dominance(
    const std::vector<BoxNode>& boxes,
    const AdaptiveFrontierItem& item,
    const std::unordered_set<int>& main_ids,
    double tolerance) {
    AdaptiveConnectivityDominance out;
    out.has_free_context = !boxes.empty();
    if (!out.has_free_context) {
        return out;
    }
    const BoxNode candidate = adaptive_make_box_from_intervals(item.intervals,
                                                              item.node,
                                                              -1,
                                                              BoxSafetyStatus::Unknown,
                                                              false);
    for (const auto& box : boxes) {
        if (!boxes_connected(box, candidate, tolerance)) {
            continue;
        }
        out.adjacent_free += 1;
        if (main_ids.find(box.id) != main_ids.end()) {
            out.adjacent_main += 1;
        } else {
            out.adjacent_other += 1;
        }
    }
    out.connector_candidate = out.adjacent_main > 0 && out.adjacent_other > 0;
    out.single_component = out.adjacent_free > 0 && !out.connector_candidate;
    out.isolated = out.adjacent_free == 0;
    if (out.connector_candidate) {
        out.priority_delta = 60.0;
    } else if (out.adjacent_main > 0) {
        out.priority_delta = 18.0;
    } else if (out.single_component) {
        out.priority_delta = -4.0;
    } else {
        out.priority_delta = -12.0;
    }
    return out;
}

double adaptive_frontier_score(const std::vector<BoxNode>& boxes,
                               const AdaptiveFrontierItem& item,
                               const std::unordered_set<int>& main_ids,
                               double overlap_depth_threshold,
                               double tolerance) {
    const double volume = std::max(adaptive_interval_volume(item.intervals), 1e-300);
    const AdaptiveConnectivityDominance dominance =
        adaptive_connectivity_dominance(boxes, item, main_ids, tolerance);
    const double normalized_overlap_depth =
        overlap_depth_threshold > 1e-12
            ? std::max(0.0, item.overlap_depth / overlap_depth_threshold)
            : 0.0;
    return std::log(volume) +
           75.0 * static_cast<double>(item.free_seed_hits) -
           (item.free_seed_hits > 0 ? 0.0 : (dominance.has_free_context ? 0.0 : 6.0)) +
           dominance.priority_delta -
           3.0 * std::max(0.0, item.overlap_ratio) -
           1.5 * normalized_overlap_depth -
           0.10 * static_cast<double>(adaptive_virtual_depth(item.node));
}

std::vector<Eigen::VectorXd> adaptive_generate_free_probes(DatabaseBoxOracle& oracle,
                                                           const std::vector<Interval>& domain,
                                                           int probe_count,
                                                           int rng_seed,
                                                           int& attempted) {
    attempted = std::max(0, probe_count);
    std::vector<Eigen::VectorXd> free_points;
    if (probe_count <= 0 || domain.empty()) {
        return free_points;
    }
    std::mt19937 rng(static_cast<std::uint32_t>(rng_seed));
    std::vector<std::uniform_real_distribution<double>> distributions;
    distributions.reserve(domain.size());
    for (const auto& interval : domain) {
        distributions.emplace_back(interval.lo, interval.hi);
    }
    for (int index = 0; index < probe_count; ++index) {
        Eigen::VectorXd point(static_cast<int>(domain.size()));
        for (int dim = 0; dim < point.size(); ++dim) {
            point[dim] = distributions[static_cast<std::size_t>(dim)](rng);
        }
        if (!oracle.point_in_collision(point)) {
            free_points.push_back(std::move(point));
        }
    }
    return free_points;
}

int adaptive_count_seed_hits(const AdaptiveFrontierItem& item,
                             const std::vector<Eigen::VectorXd>& free_probes) {
    int hits = 0;
    for (const auto& point : free_probes) {
        if (intervals_contain_point_strict_local(item.intervals, point, 0.0)) {
            ++hits;
        }
    }
    return hits;
}

void adaptive_add_depth_counter(std::unordered_map<std::string, double>& diagnostics,
                                const std::string& prefix,
                                int depth) {
    diagnostics[prefix + std::to_string(depth)] += 1.0;
}

class ScopedAdaptiveFullOverlapStats {
public:
    ScopedAdaptiveFullOverlapStats(DatabaseBoxOracle& oracle, bool enabled)
        : oracle_(oracle), previous_(oracle.validation_config().collect_full_overlap_stats) {
        if (enabled) {
            oracle_.set_collect_full_overlap_stats(true);
        }
    }

    ~ScopedAdaptiveFullOverlapStats() {
        oracle_.set_collect_full_overlap_stats(previous_);
    }

    ScopedAdaptiveFullOverlapStats(const ScopedAdaptiveFullOverlapStats&) = delete;
    ScopedAdaptiveFullOverlapStats& operator=(const ScopedAdaptiveFullOverlapStats&) = delete;

private:
    DatabaseBoxOracle& oracle_;
    bool previous_ = false;
};

std::unordered_set<int> collect_local_adjacency_ids(const std::vector<BoxNode>& live_boxes,
                                                    const std::vector<BoxNode>& local_domains,
                                                    double tolerance) {
    std::unordered_set<int> ids;
    if (local_domains.empty()) {
        return ids;
    }
    for (const auto& box : live_boxes) {
        for (const auto& domain : local_domains) {
            if (intervals_overlap_local(box.joint_intervals, domain.joint_intervals, tolerance) ||
                interval_bounds_gap_squared_local(box.joint_intervals, domain.joint_intervals) <= tolerance * tolerance) {
                ids.insert(box.id);
                break;
            }
        }
    }
    return ids;
}

void rebuild_local_adjacency(AdjacencyGraph& graph,
                             const std::vector<BoxNode>& boxes,
                             const std::unordered_set<int>& local_ids,
                             double tolerance) {
    if (local_ids.empty()) {
        return;
    }
    for (int id : local_ids) {
        graph.erase(id);
    }
    for (auto& [_, neighbors] : graph) {
        neighbors.erase(std::remove_if(neighbors.begin(), neighbors.end(), [&](int id) {
            return local_ids.find(id) != local_ids.end();
        }), neighbors.end());
    }
    std::vector<BoxNode> local_boxes;
    local_boxes.reserve(local_ids.size());
    for (const auto& box : boxes) {
        if (local_ids.find(box.id) != local_ids.end()) {
            local_boxes.push_back(box);
        }
    }
    const AdjacencyGraph local_graph = compute_adjacency(local_boxes, tolerance);
    for (const auto& [id, neighbors] : local_graph) {
        graph[id];
        for (int neighbor : neighbors) {
            append_local_edge(graph, id, neighbor);
        }
    }
}

bool allow_dynamic_commit(BoxOracle& oracle,
                          FindFreeBoxResult& result,
                          BoxCommitPolicy policy) {
    if (result.validation_detail.safety_status == BoxSafetyStatus::CertifiedFree) {
        return true;
    }
    if (result.validation_detail.safety_status != BoxSafetyStatus::ProvisionalFree) {
        return false;
    }
    if (policy == BoxCommitPolicy::CommitCertifiedOnly) {
        return false;
    }
    if (policy == BoxCommitPolicy::CommitProvisionalAllowed) {
        return true;
    }
    if (policy == BoxCommitPolicy::AuditBeforeCommit) {
        if (oracle.validate_intervals(result.intervals)) {
            result.validation_detail.safety_status = BoxSafetyStatus::CertifiedFree;
            result.validation_detail.strict_audit_required = false;
            return true;
        }
        return false;
    }
    return false;
}

bool intervals_touch_or_overlap_local(const Interval& lhs, const Interval& rhs, double tolerance) {
    return lhs.lo <= rhs.hi + tolerance && rhs.lo <= lhs.hi + tolerance;
}

std::string exact_face_merge_signature(const BoxNode& box, int merge_dim) {
    std::ostringstream oss;
    oss << std::setprecision(17);
    oss << static_cast<int>(box.safety_status) << '|'
        << (box.strict_audit_required ? 1 : 0) << '|';
    for (int dim = 0; dim < box.n_dims(); ++dim) {
        if (dim == merge_dim) {
            continue;
        }
        const auto& interval = box.joint_intervals[static_cast<std::size_t>(dim)];
        oss << dim << ':' << interval.lo << ',' << interval.hi << ';';
    }
    return oss.str();
}

bool fast_exact_face_merge_one_dim(BoxOracle& oracle,
                                   std::vector<BoxNode>& boxes,
                                   int merge_dim,
                                   double tolerance,
                                   int& exact_merges,
                                   const std::chrono::steady_clock::time_point* deadline = nullptr,
                                   bool* timed_out = nullptr) {
    if (boxes.empty() || merge_dim < 0 || merge_dim >= boxes.front().n_dims()) {
        return false;
    }
    std::unordered_map<std::string, std::vector<BoxNode>> groups;
    groups.reserve(boxes.size() * 2);
    for (const auto& box : boxes) {
        if (box.n_dims() != boxes.front().n_dims()) {
            continue;
        }
        groups[exact_face_merge_signature(box, merge_dim)].push_back(box);
    }

    bool changed = false;
    std::vector<BoxNode> merged_boxes;
    merged_boxes.reserve(boxes.size());
    std::vector<std::string> keys;
    keys.reserve(groups.size());
    for (const auto& item : groups) {
        keys.push_back(item.first);
    }
    std::sort(keys.begin(), keys.end());
    for (std::size_t key_index = 0; key_index < keys.size(); ++key_index) {
        const auto& key = keys[key_index];
        if (deadline != nullptr && std::chrono::steady_clock::now() >= *deadline) {
            if (timed_out != nullptr) {
                *timed_out = true;
            }
            for (std::size_t remaining = key_index; remaining < keys.size(); ++remaining) {
                auto& remaining_group = groups[keys[remaining]];
                merged_boxes.insert(merged_boxes.end(),
                                    std::make_move_iterator(remaining_group.begin()),
                                    std::make_move_iterator(remaining_group.end()));
            }
            break;
        }
        auto& group = groups[key];
        std::sort(group.begin(), group.end(), [merge_dim](const BoxNode& lhs, const BoxNode& rhs) {
            const auto& li = lhs.joint_intervals[static_cast<std::size_t>(merge_dim)];
            const auto& ri = rhs.joint_intervals[static_cast<std::size_t>(merge_dim)];
            if (std::abs(li.lo - ri.lo) > 1e-18) {
                return li.lo < ri.lo;
            }
            if (std::abs(li.hi - ri.hi) > 1e-18) {
                return li.hi < ri.hi;
            }
            return lhs.id < rhs.id;
        });
        BoxNode current = group.front();
        for (std::size_t index = 1; index < group.size(); ++index) {
            const BoxNode& next = group[index];
            auto& current_interval = current.joint_intervals[static_cast<std::size_t>(merge_dim)];
            const auto& next_interval = next.joint_intervals[static_cast<std::size_t>(merge_dim)];
            if (intervals_touch_or_overlap_local(current_interval, next_interval, tolerance)) {
                current_interval = current_interval.hull(next_interval);
                current.compute_volume();
                current.tree_id = -1;
                current.parent_box_id = -1;
                current.seed_config = current.center();
                oracle.release_box(next.id);
                exact_merges += 1;
                changed = true;
            } else {
                merged_boxes.push_back(current);
                current = next;
            }
        }
        merged_boxes.push_back(current);
    }
    boxes = std::move(merged_boxes);
    return changed;
}

MergerResult fast_exact_face_merge_leaf(BoxOracle& oracle,
                                        std::vector<BoxNode>& boxes,
                                        const MergerConfig& config,
                                        const std::chrono::steady_clock::time_point* deadline = nullptr,
                                        bool* timed_out = nullptr) {
    MergerResult result;
    result.boxes_before = static_cast<int>(boxes.size());
    if (boxes.empty()) {
        result.boxes_after = 0;
        return result;
    }
    const int nd = boxes.front().n_dims();
    const int max_rounds = std::max(1, config.max_rounds);
    for (int round = 0; round < max_rounds; ++round) {
        bool changed = false;
        for (int dim = 0; dim < nd; ++dim) {
            if (deadline != nullptr && std::chrono::steady_clock::now() >= *deadline) {
                if (timed_out != nullptr) {
                    *timed_out = true;
                }
                result.boxes_after = static_cast<int>(boxes.size());
                return result;
            }
            changed = fast_exact_face_merge_one_dim(oracle,
                                                    boxes,
                                                    dim,
                                                    config.adjacency_tolerance,
                                                    result.exact_merges,
                                                    deadline,
                                                    timed_out) || changed;
            if (timed_out != nullptr && *timed_out) {
                result.boxes_after = static_cast<int>(boxes.size());
                return result;
            }
        }
        result.rounds += 1;
        if (!changed) {
            break;
        }
    }
    result.boxes_after = static_cast<int>(boxes.size());
    return result;
}

struct BuildDisjointSet {
    std::unordered_map<int, int> parent;
    std::unordered_map<int, int> rank;

    void add(int id) {
        if (parent.find(id) == parent.end()) {
            parent[id] = id;
            rank[id] = 0;
        }
    }

    int find(int id) {
        add(id);
        int p = parent[id];
        if (p != id) {
            p = find(p);
            parent[id] = p;
        }
        return p;
    }

    void unite(int lhs, int rhs) {
        int left = find(lhs);
        int right = find(rhs);
        if (left == right) {
            return;
        }
        if (rank[left] < rank[right]) {
            std::swap(left, right);
        }
        parent[right] = left;
        if (rank[left] == rank[right]) {
            rank[left] += 1;
        }
    }

    bool connected(int lhs, int rhs) {
        return find(lhs) == find(rhs);
    }

    int island_count() {
        std::unordered_set<int> roots;
        roots.reserve(parent.size());
        for (const auto& [id, _] : parent) {
            roots.insert(find(id));
        }
        return static_cast<int>(roots.size());
    }
};

struct BoxSpatialIndex {
    int index_dim = -1;
    double origin = 0.0;
    double bin_width = 1.0;
    std::unordered_map<long long, std::vector<int>> bins;

    static int choose_dim(const std::vector<BoxNode>& boxes) {
        if (boxes.empty()) {
            return -1;
        }
        const int nd = boxes.front().n_dims();
        int best_dim = -1;
        double best_span = -1.0;
        for (int dim = 0; dim < nd; ++dim) {
            double lo = std::numeric_limits<double>::infinity();
            double hi = -std::numeric_limits<double>::infinity();
            for (const auto& box : boxes) {
                if (box.n_dims() != nd) {
                    continue;
                }
                lo = std::min(lo, box.joint_intervals[dim].lo);
                hi = std::max(hi, box.joint_intervals[dim].hi);
            }
            const double span = hi - lo;
            if (std::isfinite(span) && span > best_span) {
                best_span = span;
                best_dim = dim;
            }
        }
        return best_dim;
    }

    static long long bin_of(double value, double origin_value, double width) {
        return static_cast<long long>(std::floor((value - origin_value) / std::max(width, 1e-12)));
    }

    void rebuild(const std::vector<BoxNode>& boxes, double tolerance) {
        bins.clear();
        index_dim = choose_dim(boxes);
        if (index_dim < 0 || boxes.empty()) {
            return;
        }
        double lo = std::numeric_limits<double>::infinity();
        double hi = -std::numeric_limits<double>::infinity();
        double width_sum = 0.0;
        int width_count = 0;
        for (const auto& box : boxes) {
            if (box.n_dims() <= index_dim) {
                continue;
            }
            const auto& interval = box.joint_intervals[static_cast<std::size_t>(index_dim)];
            lo = std::min(lo, interval.lo);
            hi = std::max(hi, interval.hi);
            width_sum += std::max(0.0, interval.width());
            width_count += 1;
        }
        origin = std::isfinite(lo) ? lo - tolerance : 0.0;
        const double span = std::max(hi - lo, 1e-9);
        const double avg_width = width_count > 0 ? width_sum / static_cast<double>(width_count) : span;
        bin_width = std::max({span / 128.0, avg_width, tolerance * 4.0, 1e-9});
        bins.reserve(std::max<std::size_t>(1, boxes.size() * 2));
        for (int index = 0; index < static_cast<int>(boxes.size()); ++index) {
            add_box(boxes[static_cast<std::size_t>(index)], index, tolerance);
        }
    }

    void add_box(const BoxNode& box, int index, double tolerance) {
        if (index_dim < 0 || box.n_dims() <= index_dim) {
            return;
        }
        const auto& interval = box.joint_intervals[static_cast<std::size_t>(index_dim)];
        const long long lo_bin = bin_of(interval.lo - tolerance, origin, bin_width);
        const long long hi_bin = bin_of(interval.hi + tolerance, origin, bin_width);
        for (long long bin = lo_bin; bin <= hi_bin; ++bin) {
            bins[bin].push_back(index);
        }
    }

    std::vector<int> interval_candidates(const std::vector<Interval>& intervals, double tolerance) const {
        std::vector<int> out;
        if (index_dim < 0 || index_dim >= static_cast<int>(intervals.size())) {
            return out;
        }
        std::unordered_set<int> seen;
        const auto& interval = intervals[static_cast<std::size_t>(index_dim)];
        const long long lo_bin = bin_of(interval.lo - tolerance, origin, bin_width);
        const long long hi_bin = bin_of(interval.hi + tolerance, origin, bin_width);
        for (long long bin = lo_bin; bin <= hi_bin; ++bin) {
            auto it = bins.find(bin);
            if (it == bins.end()) {
                continue;
            }
            for (int index : it->second) {
                if (seen.insert(index).second) {
                    out.push_back(index);
                }
            }
        }
        return out;
    }

    std::vector<int> point_candidates(const Eigen::Ref<const Eigen::VectorXd>& point) const {
        std::vector<int> out;
        if (index_dim < 0 || index_dim >= point.size()) {
            return out;
        }
        auto it = bins.find(bin_of(point[index_dim], origin, bin_width));
        if (it != bins.end()) {
            out = it->second;
        }
        return out;
    }

    int covering_box(const std::vector<BoxNode>& boxes,
                     const Eigen::Ref<const Eigen::VectorXd>& point,
                     double tolerance) const {
        int best = -1;
        double best_volume = std::numeric_limits<double>::infinity();
        auto candidates = point_candidates(point);
        if (candidates.empty()) {
            candidates.reserve(boxes.size());
            for (int index = 0; index < static_cast<int>(boxes.size()); ++index) {
                candidates.push_back(index);
            }
        }
        for (int index : candidates) {
            if (index < 0 || index >= static_cast<int>(boxes.size())) {
                continue;
            }
            const auto& box = boxes[static_cast<std::size_t>(index)];
            if (intervals_contain_point_local(box.joint_intervals, point, tolerance) &&
                box.volume < best_volume) {
                best = index;
                best_volume = box.volume;
            }
        }
        return best;
    }
};

struct BudgetedMergeStats {
    int input_boxes = 0;
    int output_boxes = 0;
    int grid_merges = 0;
    int grid_rounds = 0;
    int tree_merges = 0;
    int tree_rounds = 0;
    int containment_pruned = 0;
    int exact_merges = 0;
    int rounds = 0;
    int stop_reason = 1;  // 0 skipped, 1 complete, 2 time_budget, 3 input_cap
    double containment_ms = 0.0;
    double grid_ms = 0.0;
    double tree_ms = 0.0;
    double exact_ms = 0.0;
    double total_ms = 0.0;
};

struct GridCellKey {
    bool valid = false;
    int depth = 0;
    std::vector<int> split_counts;
    std::vector<std::uint64_t> coords;
};

int heap_node_depth(OracleNodeId node) {
    if (node < 0) {
        return -1;
    }
    std::uint64_t value = static_cast<std::uint64_t>(node) + 1u;
    int depth = -1;
    while (value != 0u) {
        value >>= 1u;
        depth += 1;
    }
    return depth;
}

bool heap_path_bit(OracleNodeId node, int depth, int bit_index) {
    const std::uint64_t first_at_depth = (std::uint64_t{1} << static_cast<unsigned>(depth)) - 1u;
    const std::uint64_t local = static_cast<std::uint64_t>(node) - first_at_depth;
    const int shift = depth - 1 - bit_index;
    return ((local >> static_cast<unsigned>(shift)) & std::uint64_t{1}) != 0u;
}

GridCellKey grid_key_for_heap_box(const DatabaseBoxOracle& oracle,
                                  const BoxNode& box,
                                  const std::vector<int>& depth_dimensions,
                                  double tolerance) {
    GridCellKey key;
    const int nd = box.n_dims();
    const int depth = heap_node_depth(box.tree_id);
    if (depth < 0 || nd <= 0 || static_cast<int>(depth_dimensions.size()) < depth) {
        return key;
    }
    const auto& root = oracle.root_intervals();
    if (static_cast<int>(root.size()) != nd) {
        return key;
    }
    key.depth = depth;
    key.split_counts.assign(static_cast<std::size_t>(nd), 0);
    key.coords.assign(static_cast<std::size_t>(nd), 0u);
    for (int level = 0; level < depth; ++level) {
        const int dim = depth_dimensions[static_cast<std::size_t>(level)];
        if (dim < 0 || dim >= nd) {
            key.valid = false;
            return key;
        }
        key.coords[static_cast<std::size_t>(dim)] =
            (key.coords[static_cast<std::size_t>(dim)] << 1u) |
            (heap_path_bit(box.tree_id, depth, level) ? std::uint64_t{1} : std::uint64_t{0});
        key.split_counts[static_cast<std::size_t>(dim)] += 1;
    }
    std::vector<Interval> expected = root;
    for (int dim = 0; dim < nd; ++dim) {
        const int count = key.split_counts[static_cast<std::size_t>(dim)];
        const double cell_width = std::ldexp(root[static_cast<std::size_t>(dim)].width(), -count);
        expected[static_cast<std::size_t>(dim)].lo =
            root[static_cast<std::size_t>(dim)].lo +
            static_cast<double>(key.coords[static_cast<std::size_t>(dim)]) * cell_width;
        expected[static_cast<std::size_t>(dim)].hi =
            expected[static_cast<std::size_t>(dim)].lo + cell_width;
    }
    if (!intervals_equal_local(expected, box.joint_intervals, std::max(tolerance, 1e-10))) {
        return key;
    }
    key.valid = true;
    return key;
}

std::string grid_line_key(const GridCellKey& key, int merge_dim) {
    std::ostringstream oss;
    oss << "d" << key.depth << "|m" << merge_dim << '|';
    for (std::size_t dim = 0; dim < key.coords.size(); ++dim) {
        oss << key.split_counts[dim] << ':';
        if (static_cast<int>(dim) == merge_dim) {
            oss << '*';
        } else {
            oss << key.coords[dim];
        }
        oss << ';';
    }
    return oss.str();
}

int grid_line_merge_leaf(DatabaseBoxOracle& oracle,
                         std::vector<BoxNode>& boxes,
                         double tolerance,
                         int max_rounds,
                         int& rounds) {
    rounds = 0;
    if (boxes.empty()) {
        return 0;
    }
    const auto descriptor = oracle.database().split_policy_descriptor();
    if (descriptor.depth_dimensions.empty()) {
        return 0;
    }
    int total_merges = 0;
    for (int round = 0; round < std::max(1, max_rounds); ++round) {
        bool changed = false;
        const int nd = boxes.front().n_dims();
        for (int merge_dim = 0; merge_dim < nd; ++merge_dim) {
            std::vector<GridCellKey> keys;
            keys.reserve(boxes.size());
            std::unordered_map<std::string, std::vector<int>> groups;
            groups.reserve(boxes.size() * 2);
            for (int index = 0; index < static_cast<int>(boxes.size()); ++index) {
                keys.push_back(grid_key_for_heap_box(oracle, boxes[static_cast<std::size_t>(index)], descriptor.depth_dimensions, tolerance));
                const auto& key = keys.back();
                if (!key.valid) {
                    continue;
                }
                groups[grid_line_key(key, merge_dim)].push_back(index);
            }
            std::vector<unsigned char> removed(boxes.size(), 0);
            std::vector<BoxNode> additions;
            for (auto& [_, members] : groups) {
                if (members.size() <= 1) {
                    continue;
                }
                std::sort(members.begin(), members.end(), [&](int lhs, int rhs) {
                    return keys[static_cast<std::size_t>(lhs)].coords[static_cast<std::size_t>(merge_dim)] <
                           keys[static_cast<std::size_t>(rhs)].coords[static_cast<std::size_t>(merge_dim)];
                });
                std::size_t run_begin = 0;
                while (run_begin < members.size()) {
                    std::size_t run_end = run_begin + 1;
                    while (run_end < members.size()) {
                        const auto prev_coord = keys[static_cast<std::size_t>(members[run_end - 1])].coords[static_cast<std::size_t>(merge_dim)];
                        const auto next_coord = keys[static_cast<std::size_t>(members[run_end])].coords[static_cast<std::size_t>(merge_dim)];
                        if (next_coord != prev_coord + 1u) {
                            break;
                        }
                        ++run_end;
                    }
                    if (run_end - run_begin > 1) {
                        BoxNode merged = boxes[static_cast<std::size_t>(members[run_begin])];
                        for (std::size_t pos = run_begin + 1; pos < run_end; ++pos) {
                            const BoxNode& next = boxes[static_cast<std::size_t>(members[pos])];
                            for (std::size_t dim = 0; dim < merged.joint_intervals.size(); ++dim) {
                                merged.joint_intervals[dim] = merged.joint_intervals[dim].hull(next.joint_intervals[dim]);
                            }
                            removed[static_cast<std::size_t>(members[pos])] = 1;
                            oracle.release_box(next.id);
                            total_merges += 1;
                        }
                        removed[static_cast<std::size_t>(members[run_begin])] = 1;
                        merged.tree_id = -1;
                        merged.parent_box_id = -1;
                        merged.root_id = merged.id;
                        merged.seed_config = merged.center();
                        merged.compute_volume();
                        additions.push_back(std::move(merged));
                        changed = true;
                    }
                    run_begin = run_end;
                }
            }
            if (changed) {
                std::vector<BoxNode> kept;
                kept.reserve(boxes.size() + additions.size());
                for (int index = 0; index < static_cast<int>(boxes.size()); ++index) {
                    if (removed[static_cast<std::size_t>(index)] == 0) {
                        kept.push_back(std::move(boxes[static_cast<std::size_t>(index)]));
                    }
                }
                kept.insert(kept.end(),
                            std::make_move_iterator(additions.begin()),
                            std::make_move_iterator(additions.end()));
                boxes = std::move(kept);
            }
        }
        rounds += 1;
        if (!changed) {
            break;
        }
    }
    return total_merges;
}

bool try_tree_sibling_merge_round(BoxOracle& oracle,
                                  std::vector<BoxNode>& boxes,
                                  double tolerance,
                                  int& merges) {
    std::unordered_map<OracleNodeId, int> index_by_node;
    index_by_node.reserve(boxes.size() * 2);
    for (int index = 0; index < static_cast<int>(boxes.size()); ++index) {
        const OracleNodeId node = boxes[static_cast<std::size_t>(index)].tree_id;
        if (node <= 0) {
            continue;
        }
        if (index_by_node.find(node) == index_by_node.end()) {
            index_by_node[node] = index;
        }
    }
    std::vector<unsigned char> removed(boxes.size(), 0);
    bool changed = false;
    const auto planning_domain = oracle.planning_intervals();
    for (int left_index = 0; left_index < static_cast<int>(boxes.size()); ++left_index) {
        if (removed[static_cast<std::size_t>(left_index)] != 0) {
            continue;
        }
        const OracleNodeId left_node = boxes[static_cast<std::size_t>(left_index)].tree_id;
        if (left_node <= 0 || left_node % 2 == 0) {
            continue;
        }
        const OracleNodeId right_node = left_node + 1;
        auto right_it = index_by_node.find(right_node);
        if (right_it == index_by_node.end()) {
            continue;
        }
        const int right_index = right_it->second;
        if (right_index < 0 || right_index >= static_cast<int>(boxes.size()) ||
            removed[static_cast<std::size_t>(right_index)] != 0) {
            continue;
        }
        const OracleNodeId parent_node = (left_node - 1) / 2;
        BoxNode& left_box = boxes[static_cast<std::size_t>(left_index)];
        BoxNode& right_box = boxes[static_cast<std::size_t>(right_index)];
        if (left_box.n_dims() != right_box.n_dims() ||
            !boxes_connected(left_box, right_box, tolerance)) {
            continue;
        }
        std::vector<Interval> hull = left_box.joint_intervals;
        for (std::size_t dim = 0; dim < hull.size(); ++dim) {
            hull[dim] = hull[dim].hull(right_box.joint_intervals[dim]);
        }
        std::vector<Interval> parent_intervals;
        try {
            parent_intervals = oracle.node_intervals(parent_node);
        } catch (...) {
            continue;
        }
        if (!clip_intervals_to_domain_local(parent_intervals, planning_domain)) {
            continue;
        }
        if (!intervals_equal_local(hull, parent_intervals, std::max(tolerance, 1e-12))) {
            continue;
        }
        left_box.joint_intervals = std::move(hull);
        left_box.tree_id = parent_node;
        left_box.parent_box_id = -1;
        left_box.root_id = left_box.id;
        left_box.seed_config = left_box.center();
        left_box.compute_volume();
        oracle.release_box(right_box.id);
        removed[static_cast<std::size_t>(right_index)] = 1;
        merges += 1;
        changed = true;
    }
    if (!changed) {
        return false;
    }
    std::vector<BoxNode> kept;
    kept.reserve(boxes.size());
    for (int index = 0; index < static_cast<int>(boxes.size()); ++index) {
        if (removed[static_cast<std::size_t>(index)] == 0) {
            kept.push_back(std::move(boxes[static_cast<std::size_t>(index)]));
        }
    }
    boxes = std::move(kept);
    return true;
}

int tree_sibling_merge_leaf(BoxOracle& oracle,
                            std::vector<BoxNode>& boxes,
                            double tolerance,
                            int max_rounds,
                            const std::chrono::steady_clock::time_point* deadline,
                            bool& timed_out,
                            int& rounds) {
    int merges = 0;
    rounds = 0;
    for (int round = 0; round < std::max(1, max_rounds); ++round) {
        if (deadline != nullptr && std::chrono::steady_clock::now() >= *deadline) {
            timed_out = true;
            break;
        }
        int round_merges = 0;
        const bool changed = try_tree_sibling_merge_round(oracle, boxes, tolerance, round_merges);
        merges += round_merges;
        rounds += 1;
        if (!changed) {
            break;
        }
    }
    return merges;
}

int indexed_containment_prune_leaf(BoxOracle& oracle,
                                   std::vector<BoxNode>& boxes,
                                   double tolerance,
                                   const std::chrono::steady_clock::time_point* deadline,
                                   bool& timed_out) {
    if (boxes.size() <= 1) {
        return 0;
    }
    BoxSpatialIndex index;
    index.rebuild(boxes, tolerance);
    std::vector<unsigned char> remove(boxes.size(), 0);
    int pruned = 0;
    for (int i = 0; i < static_cast<int>(boxes.size()); ++i) {
        if (deadline != nullptr && std::chrono::steady_clock::now() >= *deadline) {
            timed_out = true;
            break;
        }
        if (remove[static_cast<std::size_t>(i)] != 0) {
            continue;
        }
        const BoxNode& inner = boxes[static_cast<std::size_t>(i)];
        const auto candidates = index.point_candidates(inner.center());
        for (int j : candidates) {
            if (j == i || j < 0 || j >= static_cast<int>(boxes.size()) ||
                remove[static_cast<std::size_t>(j)] != 0) {
                continue;
            }
            const BoxNode& outer = boxes[static_cast<std::size_t>(j)];
            if (outer.volume + 1e-18 < inner.volume) {
                continue;
            }
            if (std::abs(outer.volume - inner.volume) <= 1e-18 && outer.id > inner.id) {
                continue;
            }
            if (intervals_subset_local(inner.joint_intervals, outer.joint_intervals, tolerance)) {
                remove[static_cast<std::size_t>(i)] = 1;
                pruned += 1;
                oracle.release_box(inner.id);
                break;
            }
        }
    }
    if (pruned <= 0) {
        return 0;
    }
    std::vector<BoxNode> kept;
    kept.reserve(boxes.size() - static_cast<std::size_t>(pruned));
    for (int i = 0; i < static_cast<int>(boxes.size()); ++i) {
        if (remove[static_cast<std::size_t>(i)] == 0) {
            kept.push_back(std::move(boxes[static_cast<std::size_t>(i)]));
        }
    }
    boxes = std::move(kept);
    return pruned;
}

BudgetedMergeStats budgeted_leaf_merge(DatabaseBoxOracle& oracle,
                                       std::vector<BoxNode>& boxes,
                                       MergerConfig config,
                                       double max_merge_ms,
                                       int max_merge_rounds,
                                       int max_merge_input_boxes,
                                       double adjacency_tolerance) {
    using Clock = std::chrono::steady_clock;
    const auto start = Clock::now();
    BudgetedMergeStats stats;
    stats.input_boxes = static_cast<int>(boxes.size());
    stats.output_boxes = stats.input_boxes;
    if (boxes.empty()) {
        stats.stop_reason = 0;
        return stats;
    }
    const bool has_budget = max_merge_ms > 0.0;
    const auto deadline = has_budget
        ? start + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double, std::milli>(max_merge_ms))
        : Clock::time_point::max();
    bool timed_out = false;
    const auto grid_start = Clock::now();
    stats.grid_merges = grid_line_merge_leaf(oracle,
                                             boxes,
                                             adjacency_tolerance,
                                             std::max(1, max_merge_rounds),
                                             stats.grid_rounds);
    stats.grid_ms = std::chrono::duration<double, std::milli>(Clock::now() - grid_start).count();
    if (has_budget && Clock::now() >= deadline) {
        stats.stop_reason = 2;
        stats.output_boxes = static_cast<int>(boxes.size());
        stats.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - start).count();
        return stats;
    }
    const auto tree_start = Clock::now();
    stats.tree_merges = tree_sibling_merge_leaf(oracle,
                                                boxes,
                                                adjacency_tolerance,
                                                std::max(1, max_merge_rounds),
                                                has_budget ? &deadline : nullptr,
                                                timed_out,
                                                stats.tree_rounds);
    stats.tree_ms = std::chrono::duration<double, std::milli>(Clock::now() - tree_start).count();
    if (timed_out) {
        stats.stop_reason = 2;
        stats.output_boxes = static_cast<int>(boxes.size());
        stats.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - start).count();
        return stats;
    }
    const auto containment_start = Clock::now();
    if (config.containment_prune) {
        const double containment_budget_ms = has_budget
            ? std::min(max_merge_ms * 0.15, 100.0)
            : 0.0;
        const auto containment_deadline = has_budget
            ? start + std::chrono::duration_cast<Clock::duration>(
                          std::chrono::duration<double, std::milli>(containment_budget_ms))
            : Clock::time_point::max();
        stats.containment_pruned = indexed_containment_prune_leaf(oracle,
                                                                  boxes,
                                                                  adjacency_tolerance,
                                                                  has_budget ? &containment_deadline : nullptr,
                                                                  timed_out);
    }
    stats.containment_ms = std::chrono::duration<double, std::milli>(Clock::now() - containment_start).count();
    if (timed_out) {
        timed_out = false;
    }
    const int input_cap = std::max(0, max_merge_input_boxes);
    if (input_cap > 0 && static_cast<int>(boxes.size()) > input_cap) {
        stats.stop_reason = 3;
        stats.output_boxes = static_cast<int>(boxes.size());
        stats.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - start).count();
        return stats;
    }
    config.exact_face_merge = true;
    config.greedy_hull_merge = false;
    config.containment_prune = false;
    config.adjacency_tolerance = adjacency_tolerance;
    config.max_rounds = std::max(1, max_merge_rounds);
    const auto exact_start = Clock::now();
    bool exact_timed_out = false;
    MergerResult exact_result = fast_exact_face_merge_leaf(oracle,
                                                           boxes,
                                                           config,
                                                           has_budget ? &deadline : nullptr,
                                                           &exact_timed_out);
    stats.exact_ms = std::chrono::duration<double, std::milli>(Clock::now() - exact_start).count();
    stats.exact_merges = exact_result.exact_merges;
    stats.rounds = exact_result.rounds;
    if (exact_timed_out) {
        stats.stop_reason = 2;
    }
    stats.output_boxes = static_cast<int>(boxes.size());
    stats.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - start).count();
    return stats;
}

struct QueryRootPair {
    Eigen::VectorXd start;
    Eigen::VectorXd goal;
    int start_box_id = -1;
    int goal_box_id = -1;
    int start_frontier_box_id = -1;
    int goal_frontier_box_id = -1;
    int attempts = 0;
    bool grow_from_start = true;
};

struct QueryRootGrowResult {
	int boxes_added = 0;
	int endpoint_anchors_added = 0;
	int endpoint_root_fallbacks = 0;
	int uncovered_endpoints = 0;
    int ffb_success = 0;
    int ffb_fail = 0;
    int contained_rejects = 0;
    int domain_rejects = 0;
    int adjacency_rejects = 0;
    int adjacency_edges_added = 0;
    int commit_rejects = 0;
    int adjacency_candidates_tested = 0;
    int pair_attempts = 0;
    int pairs_total = 0;
    int pairs_connected_before = 0;
    int pairs_connected_after = 0;
    int islands_before = 0;
    int islands_after = 0;
    double index_rebuild_ms = 0.0;
    double index_query_ms = 0.0;
    double total_ms = 0.0;
};

std::vector<QueryRootPair> make_query_root_pairs(const std::vector<Eigen::VectorXd>& priority_points) {
    std::vector<QueryRootPair> pairs;
    if (priority_points.size() >= 5) {
        for (std::size_t index = 0; index + 4 < priority_points.size(); index += 5) {
            QueryRootPair pair;
            pair.start = priority_points[index];
            pair.goal = priority_points[index + 4];
            pairs.push_back(std::move(pair));
        }
        return pairs;
    }
    for (std::size_t index = 0; index + 1 < priority_points.size(); index += 2) {
        QueryRootPair pair;
        pair.start = priority_points[index];
        pair.goal = priority_points[index + 1];
        pairs.push_back(std::move(pair));
    }
    return pairs;
}

BuildDisjointSet make_dsu_from_graph(const std::vector<BoxNode>& boxes, const AdjacencyGraph& graph) {
    BuildDisjointSet dsu;
    for (const auto& box : boxes) {
        dsu.add(box.id);
    }
    for (const auto& [id, neighbors] : graph) {
        dsu.add(id);
        for (int neighbor : neighbors) {
            dsu.unite(id, neighbor);
        }
    }
    return dsu;
}

int find_containing_domain_index(const std::vector<BoxNode>& domains,
                                 const BoxSpatialIndex& domain_index,
                                 const Eigen::Ref<const Eigen::VectorXd>& point,
                                 double tolerance) {
    auto candidates = domain_index.point_candidates(point);
    if (candidates.empty()) {
        candidates.reserve(domains.size());
        for (int index = 0; index < static_cast<int>(domains.size()); ++index) {
            candidates.push_back(index);
        }
    }
    int best = -1;
    double best_volume = std::numeric_limits<double>::infinity();
    for (int index : candidates) {
        if (index < 0 || index >= static_cast<int>(domains.size())) {
            continue;
        }
        const auto& domain = domains[static_cast<std::size_t>(index)];
        if (intervals_contain_point_strict_local(domain.joint_intervals, point, tolerance) &&
            domain.volume < best_volume) {
            best = index;
            best_volume = domain.volume;
        }
    }
    return best;
}

bool make_directed_face_seed(const BoxNode& source,
                             const Eigen::Ref<const Eigen::VectorXd>& target,
                             const std::vector<Interval>& root,
                             double epsilon,
                             int rank,
                             Eigen::VectorXd& seed) {
    if (source.n_dims() != target.size() || target.size() != static_cast<int>(root.size())) {
        return false;
    }
    struct Candidate {
        int dim = -1;
        int side = 0;
        double score = 0.0;
    };
    std::vector<Candidate> candidates;
    const Eigen::VectorXd center = source.center();
    for (int dim = 0; dim < source.n_dims(); ++dim) {
        const double delta = target[dim] - center[dim];
        if (std::abs(delta) <= 1e-12) {
            continue;
        }
        const int side = delta > 0.0 ? 1 : 0;
        const double value = side > 0
            ? source.joint_intervals[dim].hi + epsilon
            : source.joint_intervals[dim].lo - epsilon;
        if (!root[static_cast<std::size_t>(dim)].contains(value, 0.0)) {
            continue;
        }
        candidates.push_back({dim, side, std::abs(delta)});
    }
    std::sort(candidates.begin(), candidates.end(), [](const Candidate& lhs, const Candidate& rhs) {
        if (std::abs(lhs.score - rhs.score) > 1e-18) {
            return lhs.score > rhs.score;
        }
        return lhs.dim < rhs.dim;
    });
    // For each outward face, try several points on the same face.  A pure
    // target-clamped corner seed often falls into the narrow obstacle side of a
    // collision domain; face-center and half-target variants give the local
    // FFB a chance to grow around that obstruction while still remaining
    // adjacent to the source box.
    constexpr int variants_per_face = 3;
    const int candidate_rank = rank / variants_per_face;
    const int variant = rank % variants_per_face;
    if (rank < 0 || candidate_rank < 0 || candidate_rank >= static_cast<int>(candidates.size())) {
        return false;
    }
    const Candidate& candidate = candidates[static_cast<std::size_t>(candidate_rank)];
    seed = source.center();
    for (int dim = 0; dim < source.n_dims(); ++dim) {
        if (dim == candidate.dim) {
            seed[dim] = candidate.side > 0
                ? source.joint_intervals[dim].hi + epsilon
                : source.joint_intervals[dim].lo - epsilon;
        } else {
            const double clamped_target = std::clamp(target[dim],
                                                     source.joint_intervals[dim].lo,
                                                     source.joint_intervals[dim].hi);
            if (variant == 0) {
                seed[dim] = center[dim];
            } else if (variant == 1) {
                seed[dim] = 0.5 * (center[dim] + clamped_target);
            } else {
                seed[dim] = clamped_target;
            }
        }
        seed[dim] = std::clamp(seed[dim],
                               root[static_cast<std::size_t>(dim)].lo,
                               root[static_cast<std::size_t>(dim)].hi);
    }
    return true;
}

int nearest_box_in_component(const std::vector<BoxNode>& boxes,
                             BuildDisjointSet& dsu,
                             int component_box_id,
                             const Eigen::Ref<const Eigen::VectorXd>& point) {
    if (component_box_id < 0) {
        return -1;
    }
    const int component = dsu.find(component_box_id);
    int best_id = -1;
    double best_gap = std::numeric_limits<double>::infinity();
    double best_center = std::numeric_limits<double>::infinity();
    for (const auto& box : boxes) {
        if (dsu.find(box.id) != component || box.n_dims() != point.size()) {
            continue;
        }
        const double gap = intervals_point_gap_local(box.joint_intervals, point);
        const double center_dist = (box.center() - point).squaredNorm();
        if (gap < best_gap - 1e-18 ||
            (std::abs(gap - best_gap) <= 1e-18 && center_dist < best_center)) {
            best_id = box.id;
            best_gap = gap;
            best_center = center_dist;
        }
    }
    return best_id;
}

int nearest_box_outside_component(const std::vector<BoxNode>& boxes,
                                  BuildDisjointSet& dsu,
                                  int component_box_id,
                                  const Eigen::Ref<const Eigen::VectorXd>& point) {
    if (component_box_id < 0) {
        return -1;
    }
    const int component = dsu.find(component_box_id);
    int best_id = -1;
    double best_gap = std::numeric_limits<double>::infinity();
    double best_center = std::numeric_limits<double>::infinity();
    for (const auto& box : boxes) {
        if (dsu.find(box.id) == component || box.n_dims() != point.size()) {
            continue;
        }
        const double gap = intervals_point_gap_local(box.joint_intervals, point);
        const double center_dist = (box.center() - point).squaredNorm();
        if (gap < best_gap - 1e-18 ||
            (std::abs(gap - best_gap) <= 1e-18 && center_dist < best_center)) {
            best_id = box.id;
            best_gap = gap;
            best_center = center_dist;
        }
    }
    return best_id;
}

int commit_query_root_box(BoxOracle& oracle,
                          const FindFreeBoxOptions& options,
                          BoxCommitPolicy commit_policy,
                          const std::function<FindFreeBoxResult(const Eigen::VectorXd&,
                                                                 const std::vector<Interval>&,
                                                                 StageContext&,
                                                                 const FindFreeBoxOptions&)>& find_in_domain,
                          const Eigen::Ref<const Eigen::VectorXd>& seed,
                          const BoxNode& domain,
                          int parent_box_id,
                          int root_id,
                          std::vector<BoxNode>& boxes,
                          std::vector<BoxNode>& raw_boxes,
                          AdjacencyGraph& graph,
                          BoxSpatialIndex& box_index,
                          BuildDisjointSet& dsu,
                          int& next_id,
                          StageContext& context,
                          QueryRootGrowResult& stats,
                          double adjacency_tolerance) {
    using Clock = std::chrono::steady_clock;
    const auto query_start = Clock::now();
    if (box_index.covering_box(boxes, seed, 0.0) >= 0) {
        stats.contained_rejects += 1;
        stats.index_query_ms += std::chrono::duration<double, std::milli>(Clock::now() - query_start).count();
        return -1;
    }
    stats.index_query_ms += std::chrono::duration<double, std::milli>(Clock::now() - query_start).count();

    auto result = find_in_domain(seed, domain.joint_intervals, context, options);
    if (!result.found) {
        stats.ffb_fail += 1;
        return -1;
    }
    stats.ffb_success += 1;
    if (!intervals_subset_local(result.intervals, domain.joint_intervals, 1e-12) ||
        !intervals_contain_point_with_boundary_tolerance(result.intervals, seed, adjacency_tolerance)) {
        stats.domain_rejects += 1;
        return -1;
    }
    if (!allow_dynamic_commit(oracle, result, commit_policy)) {
        stats.commit_rejects += 1;
        return -1;
    }

    BoxNode box;
    box.id = next_id++;
    box.joint_intervals = result.intervals;
    const bool boundary_expanded =
        expand_intervals_to_contain_boundary_seed(box.joint_intervals, seed, adjacency_tolerance);
    if (!intervals_subset_local(box.joint_intervals,
                                domain.joint_intervals,
                                kQueryRootBoundaryContainmentTolerance)) {
        stats.domain_rejects += 1;
        return -1;
    }
    box.seed_config = seed;
    box.tree_id = result.node;
    box.parent_box_id = parent_box_id;
    box.root_id = root_id >= 0 ? root_id : box.id;
    box.safety_status = result.validation_detail.safety_status;
    box.strict_audit_required = result.validation_detail.strict_audit_required || boundary_expanded;
    box.compute_volume();

    const auto contained_start = Clock::now();
    auto containing_candidates = box_index.interval_candidates(box.joint_intervals, 0.0);
    if (containing_candidates.empty()) {
        containing_candidates.reserve(boxes.size());
        for (int index = 0; index < static_cast<int>(boxes.size()); ++index) {
            containing_candidates.push_back(index);
        }
    }
    for (int index : containing_candidates) {
        if (index >= 0 && index < static_cast<int>(boxes.size()) &&
            box_contains_box_exact_local(boxes[static_cast<std::size_t>(index)], box)) {
            stats.contained_rejects += 1;
            stats.index_query_ms += std::chrono::duration<double, std::milli>(Clock::now() - contained_start).count();
            return -1;
        }
    }
    stats.index_query_ms += std::chrono::duration<double, std::milli>(Clock::now() - contained_start).count();

    if (parent_box_id >= 0) {
        const BoxNode* parent = find_box_by_id(boxes, parent_box_id);
        if (parent == nullptr || !boxes_connected(*parent, box, adjacency_tolerance)) {
            stats.adjacency_rejects += 1;
            return -1;
        }
        box.root_id = parent->root_id >= 0 ? parent->root_id : parent->id;
    }

    oracle.reserve_node(box.tree_id, box.id);
    const int new_index = static_cast<int>(boxes.size());
    boxes.push_back(box);
    raw_boxes.push_back(box);
    graph[box.id];
    dsu.add(box.id);
    if (parent_box_id >= 0) {
        append_local_edge(graph, parent_box_id, box.id);
        dsu.unite(parent_box_id, box.id);
    }
    auto adjacency_candidates = box_index.interval_candidates(box.joint_intervals, adjacency_tolerance);
    stats.adjacency_candidates_tested += static_cast<int>(adjacency_candidates.size());
    for (int index : adjacency_candidates) {
        if (index < 0 || index >= new_index) {
            continue;
        }
        const BoxNode& existing = boxes[static_cast<std::size_t>(index)];
        if (boxes_connected(existing, box, adjacency_tolerance)) {
            append_local_edge(graph, existing.id, box.id);
            dsu.unite(existing.id, box.id);
        }
    }
    box_index.add_box(boxes.back(), new_index, adjacency_tolerance);
    stats.boxes_added += 1;
    return box.id;
}

struct OfflineAnchorGrowResult {
    int candidates_total = 0;
    int candidates_covered = 0;
    int boxes_added = 0;
    int ffb_success = 0;
    int ffb_fail = 0;
    int contained_rejects = 0;
    int domain_rejects = 0;
    int adjacency_rejects = 0;
    int commit_rejects = 0;
    int adjacency_candidates_tested = 0;
    int adjacency_edges_added = 0;
    int islands_before = 0;
    int islands_after = 0;
    double box_volume_sum = 0.0;
    double box_volume_max = 0.0;
    double index_rebuild_ms = 0.0;
    double index_query_ms = 0.0;
    double total_ms = 0.0;
};

OfflineAnchorGrowResult run_offline_anchor_grower(
    BoxOracle& oracle,
    const LeafSweepRefineConfig& refine_config,
    const std::vector<BoxNode>& collision_domains,
    const std::vector<Eigen::VectorXd>& offline_anchor_points,
    const std::function<FindFreeBoxResult(const Eigen::VectorXd&,
                                         const std::vector<Interval>&,
                                         StageContext&,
                                         const FindFreeBoxOptions&)>& find_in_domain,
    BoxCommitPolicy commit_policy,
    std::vector<BoxNode>& boxes,
    std::vector<BoxNode>& raw_boxes,
    AdjacencyGraph& graph,
    int& next_id,
    StageContext& context,
    const FindFreeBoxOptions& base_options,
    double adjacency_tolerance) {
    using Clock = std::chrono::steady_clock;
    const auto total_start = Clock::now();
    OfflineAnchorGrowResult stats;
    stats.candidates_total = static_cast<int>(offline_anchor_points.size());
    if (offline_anchor_points.empty()) {
        return stats;
    }

    BuildDisjointSet dsu = make_dsu_from_graph(boxes, graph);
    stats.islands_before = dsu.island_count();

    const auto index_start = Clock::now();
    BoxSpatialIndex box_index;
    box_index.rebuild(boxes, adjacency_tolerance);
    BoxSpatialIndex domain_index;
    domain_index.rebuild(collision_domains, adjacency_tolerance);
    stats.index_rebuild_ms += std::chrono::duration<double, std::milli>(Clock::now() - index_start).count();

    QueryRootGrowResult commit_stats;
    FindFreeBoxOptions options = base_options;
    options.max_depth = refine_config.deep_ffb_depth;
    options.reject_seed_collision = false;
    const int max_boxes = std::max(0, refine_config.deep_max_boxes);

    for (const auto& point : offline_anchor_points) {
        if (context.should_stop() || commit_stats.boxes_added >= max_boxes) {
            break;
        }
        const auto cover_start = Clock::now();
        const int owner_index = box_index.covering_box(boxes, point, adjacency_tolerance);
        commit_stats.index_query_ms +=
            std::chrono::duration<double, std::milli>(Clock::now() - cover_start).count();
        if (owner_index >= 0) {
            stats.candidates_covered += 1;
            continue;
        }
        const int domain_idx = find_containing_domain_index(collision_domains,
                                                            domain_index,
                                                            point,
                                                            adjacency_tolerance);
        if (domain_idx < 0) {
            commit_stats.domain_rejects += 1;
            continue;
        }
        const int new_id = commit_query_root_box(oracle,
                                                 options,
                                                 commit_policy,
                                                 find_in_domain,
                                                 point,
                                                 collision_domains[static_cast<std::size_t>(domain_idx)],
                                                 -1,
                                                 -1,
                                                 boxes,
                                                 raw_boxes,
                                                 graph,
                                                 box_index,
                                                 dsu,
                                                 next_id,
                                                 context,
                                                 commit_stats,
                                                 adjacency_tolerance);
        if (new_id >= 0) {
            if (const BoxNode* box = find_box_by_id(boxes, new_id)) {
                stats.box_volume_sum += box->volume;
                stats.box_volume_max = std::max(stats.box_volume_max, box->volume);
            }
        }
    }

    stats.boxes_added = commit_stats.boxes_added;
    stats.ffb_success = commit_stats.ffb_success;
    stats.ffb_fail = commit_stats.ffb_fail;
    stats.contained_rejects = commit_stats.contained_rejects;
    stats.domain_rejects = commit_stats.domain_rejects;
    stats.adjacency_rejects = commit_stats.adjacency_rejects;
    stats.commit_rejects = commit_stats.commit_rejects;
    stats.adjacency_candidates_tested = commit_stats.adjacency_candidates_tested;
    stats.adjacency_edges_added = commit_stats.adjacency_edges_added;
    stats.index_query_ms = commit_stats.index_query_ms;
    stats.islands_after = dsu.island_count();
    stats.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - total_start).count();
    return stats;
}

QueryRootGrowResult run_query_root_box_grower(BoxOracle& oracle,
                                              const LeafSweepRefineConfig& refine_config,
                                              const std::vector<BoxNode>& collision_domains,
                                              const std::vector<Eigen::VectorXd>& priority_points,
                                              const std::function<FindFreeBoxResult(const Eigen::VectorXd&,
                                                                                   const std::vector<Interval>&,
                                                                                   StageContext&,
                                                                                   const FindFreeBoxOptions&)>& find_in_domain,
                                              BoxCommitPolicy commit_policy,
                                              std::vector<BoxNode>& boxes,
                                              std::vector<BoxNode>& raw_boxes,
                                              AdjacencyGraph& graph,
                                              int& next_id,
                                              StageContext& context,
                                              const FindFreeBoxOptions& base_options,
                                              double adjacency_tolerance) {
    using Clock = std::chrono::steady_clock;
    const auto total_start = Clock::now();
    QueryRootGrowResult stats;
    auto pairs = make_query_root_pairs(priority_points);
    stats.pairs_total = static_cast<int>(pairs.size());
    BuildDisjointSet dsu = make_dsu_from_graph(boxes, graph);
    stats.islands_before = dsu.island_count();

    const auto index_start = Clock::now();
    BoxSpatialIndex box_index;
    box_index.rebuild(boxes, adjacency_tolerance);
    BoxSpatialIndex domain_index;
    domain_index.rebuild(collision_domains, adjacency_tolerance);
    stats.index_rebuild_ms += std::chrono::duration<double, std::milli>(Clock::now() - index_start).count();

	FindFreeBoxOptions options = base_options;
	options.max_depth = refine_config.deep_ffb_depth;
	options.reject_seed_collision = false;
	const auto root = oracle.planning_intervals();
	BoxNode root_domain;
	root_domain.id = -1;
	root_domain.joint_intervals = root;
	root_domain.compute_volume();
	const double epsilon = std::max(1e-10, 0.25 * adjacency_tolerance);

    auto owner_for_point = [&](const Eigen::VectorXd& point, double tolerance) -> int {
        const auto t0 = Clock::now();
        const int owner_index = box_index.covering_box(boxes, point, tolerance);
        stats.index_query_ms += std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
        return owner_index >= 0 ? boxes[static_cast<std::size_t>(owner_index)].id : -1;
    };

    auto ensure_endpoint_box = [&](const Eigen::VectorXd& point) -> int {
        int owner = owner_for_point(point, adjacency_tolerance);
        if (owner >= 0) {
            return owner;
        }
        stats.uncovered_endpoints += 1;
		const int domain_idx = find_containing_domain_index(collision_domains, domain_index, point, adjacency_tolerance);
		if (stats.boxes_added >= std::max(0, refine_config.deep_max_boxes)) {
			stats.domain_rejects += 1;
			return -1;
		}
		int box_id = -1;
		if (domain_idx >= 0) {
			const BoxNode& domain = collision_domains[static_cast<std::size_t>(domain_idx)];
			box_id = commit_query_root_box(oracle,
									 options,
									 commit_policy,
									 find_in_domain,
									 point,
									 domain,
									 -1,
									 -1,
									 boxes,
									 raw_boxes,
									 graph,
									 box_index,
									 dsu,
									 next_id,
									 context,
									 stats,
									 adjacency_tolerance);
		} else {
			stats.domain_rejects += 1;
		}
		if (box_id < 0 && stats.boxes_added < std::max(0, refine_config.deep_max_boxes)) {
			stats.endpoint_root_fallbacks += 1;
			box_id = commit_query_root_box(oracle,
										 options,
										 commit_policy,
										 find_in_domain,
										 point,
										 root_domain,
										 -1,
										 -1,
										 boxes,
										 raw_boxes,
										 graph,
										 box_index,
										 dsu,
										 next_id,
										 context,
										 stats,
										 adjacency_tolerance);
		}
		if (box_id >= 0) {
			stats.endpoint_anchors_added += 1;
		}
        return box_id;
    };

    for (auto& pair : pairs) {
        pair.start_box_id = ensure_endpoint_box(pair.start);
        pair.goal_box_id = ensure_endpoint_box(pair.goal);
        pair.start_frontier_box_id = pair.start_box_id;
        pair.goal_frontier_box_id = pair.goal_box_id;
        if (pair.start_box_id >= 0 && pair.goal_box_id >= 0 && dsu.connected(pair.start_box_id, pair.goal_box_id)) {
            stats.pairs_connected_before += 1;
        }
    }

    const int max_boxes = std::max(0, refine_config.deep_max_boxes);
    const int per_pair_attempt_cap = std::max(1, refine_config.domain_attempt_cap);
    const int max_attempts = static_cast<int>(pairs.size()) * per_pair_attempt_cap;
    bool progressed = true;
    while (!context.should_stop() &&
           progressed &&
           stats.boxes_added < max_boxes &&
           stats.pair_attempts < max_attempts) {
        progressed = false;
        for (auto& pair : pairs) {
            if (context.should_stop() ||
                stats.boxes_added >= max_boxes ||
                stats.pair_attempts >= max_attempts) {
                break;
            }
            if (pair.start_box_id < 0 || pair.goal_box_id < 0 ||
                dsu.connected(pair.start_box_id, pair.goal_box_id)) {
                continue;
            }
            if (pair.attempts >= per_pair_attempt_cap) {
                continue;
            }
            const int source_id = pair.grow_from_start ? pair.start_frontier_box_id : pair.goal_frontier_box_id;
            const int target_anchor_id = pair.grow_from_start ? pair.goal_box_id : pair.start_box_id;
            const Eigen::VectorXd& target_point = pair.grow_from_start ? pair.goal : pair.start;
            const BoxNode* source = find_box_by_id(boxes, source_id);
            if (source == nullptr) {
                continue;
            }
            int target_box_id = nearest_box_outside_component(boxes, dsu, source->id, source->center());
            if (target_box_id < 0) {
                target_box_id = nearest_box_in_component(boxes, dsu, target_anchor_id, source->center());
            }
            const BoxNode* target_box = target_box_id >= 0 ? find_box_by_id(boxes, target_box_id) : nullptr;
            const Eigen::VectorXd target = target_box != nullptr ? target_box->center() : target_point;
            bool added = false;
            const int remaining_pair_attempts = per_pair_attempt_cap - pair.attempts;
            const int local_seed_cap = std::min(std::max(1, refine_config.domain_seed_cap),
                                                std::max(1, remaining_pair_attempts));
            for (int face_rank = 0;
                 face_rank < local_seed_cap && pair.attempts < per_pair_attempt_cap;
                 ++face_rank) {
                Eigen::VectorXd seed;
                if (!make_directed_face_seed(*source, target, root, epsilon, face_rank, seed)) {
                    break;
                }
                stats.pair_attempts += 1;
                pair.attempts += 1;
                const int covered_owner = owner_for_point(seed, 0.0);
                if (covered_owner >= 0) {
                    const BoxNode* owner_box = find_box_by_id(boxes, covered_owner);
                    if (owner_box != nullptr &&
                        owner_box->id != source->id &&
                        boxes_connected(*source, *owner_box, adjacency_tolerance)) {
                        append_local_edge(graph, source->id, owner_box->id);
                        dsu.unite(source->id, owner_box->id);
                        stats.adjacency_edges_added += 1;
                        if (pair.grow_from_start) {
                            pair.start_frontier_box_id = owner_box->id;
                        } else {
                            pair.goal_frontier_box_id = owner_box->id;
                        }
                        progressed = true;
                        added = true;
                        break;
                    }
                    stats.contained_rejects += 1;
                    continue;
                }
                const int domain_idx = find_containing_domain_index(collision_domains, domain_index, seed, adjacency_tolerance);
                if (domain_idx < 0) {
                    stats.domain_rejects += 1;
                    continue;
                }
                const int new_id = commit_query_root_box(oracle,
                                                         options,
                                                         commit_policy,
                                                         find_in_domain,
                                                         seed,
                                                         collision_domains[static_cast<std::size_t>(domain_idx)],
                                                         source->id,
                                                         source->root_id >= 0 ? source->root_id : source->id,
                                                         boxes,
                                                         raw_boxes,
                                                         graph,
                                                         box_index,
                                                         dsu,
                                                         next_id,
                                                         context,
                                                         stats,
                                                         adjacency_tolerance);
                if (new_id >= 0) {
                    if (pair.grow_from_start) {
                        pair.start_frontier_box_id = new_id;
                    } else {
                        pair.goal_frontier_box_id = new_id;
                    }
                    added = true;
                    progressed = true;
                    break;
                }
                if (stats.pair_attempts >= max_attempts ||
                    stats.boxes_added >= max_boxes ||
                    context.should_stop()) {
                    break;
                }
            }
            pair.grow_from_start = !pair.grow_from_start;
            if (!added && pair.attempts >= refine_config.domain_attempt_cap * 2) {
                pair.grow_from_start = !pair.grow_from_start;
            }
        }
    }

    for (const auto& pair : pairs) {
        if (pair.start_box_id >= 0 && pair.goal_box_id >= 0 && dsu.connected(pair.start_box_id, pair.goal_box_id)) {
            stats.pairs_connected_after += 1;
        }
    }
    stats.islands_after = dsu.island_count();
    stats.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - total_start).count();
    return stats;
}

bool segment_edge_survives_scene(const SegmentEdge& edge,
                                 const CollisionChecker& checker,
                                 int audit_resolution,
                                 double audit_segment_step) {
    if (edge.waypoints.size() < 2) {
        return false;
    }
    const int resolution = std::max({1, audit_resolution, edge.segment_resolution});
    return audit_waypoint_path(edge.waypoints, checker, resolution, audit_segment_step).passed;
}

std::vector<int> spatial_dirty_all_box_indices(const Robot& robot,
                                               const std::vector<BoxNode>& boxes,
                                               const Obstacle& obstacle,
                                               const DynamicUpdateConfig& config,
                                               int& dirty_count) {
    dirty_count = 0;
    std::vector<int> dirty_indices;
    if (boxes.empty()) {
        return dirty_indices;
    }
    CollisionChecker dirty_checker(robot, Scene(std::vector<Obstacle>{inflate_obstacle(obstacle, config.dirty_region_padding)}));
    dirty_indices.reserve(boxes.size());
    for (int index = 0; index < static_cast<int>(boxes.size()); ++index) {
        if (dirty_checker.check_box(boxes[static_cast<std::size_t>(index)].joint_intervals)) {
            dirty_count += 1;
            dirty_indices.push_back(index);
        }
    }
    return dirty_indices;
}

void summarize_query_path(QueryResult& result,
                          const std::vector<BoxNode>& boxes,
                          const SegmentEdgeList& segment_edges) {
    if (result.raw_path_length <= 0.0 && result.path_length > 0.0) {
        result.raw_path_length = result.path_length;
    }
    result.segment_edge_length = 0.0;
    result.segment_edges_used = 0;
    for (int edge_id : result.segment_edge_sequence) {
        if (edge_id < 0) {
            continue;
        }
        if (const SegmentEdge* edge = find_segment_edge_by_id(segment_edges, edge_id)) {
            if (counts_as_segment_edge(edge->type)) {
                result.segment_edges_used += 1;
                result.segment_edge_length +=
                    uncovered_segment_edge_length(*edge, boxes);
            }
        }
    }
    int provisional_or_unknown_boxes = 0;
    for (int box_id : result.box_sequence) {
        const BoxNode* box = find_box_by_id(boxes, box_id);
        if (box == nullptr || box->safety_status != BoxSafetyStatus::CertifiedFree || box->strict_audit_required) {
            provisional_or_unknown_boxes += 1;
        }
    }
    const double box_path_length = std::max(0.0, result.path_length - result.segment_edge_length);
    if (provisional_or_unknown_boxes == 0) {
        result.certified_box_length = box_path_length;
        result.provisional_audited_length = 0.0;
    } else {
        result.certified_box_length = 0.0;
        result.provisional_audited_length = box_path_length;
    }
    result.remaining_unsafe_assumptions = provisional_or_unknown_boxes;
}

bool try_local_birrt_repair(QueryResult& result,
                            const PathAuditCheck& audit,
                            const CollisionChecker& checker,
                            const Robot& robot,
                            const QueryConfig& query_config,
                            const RRTConnectConfig& base_repair_config,
                            int planner_seed_base) {
    if (audit.failed_segment_index < 0 || audit.failed_segment_index + 1 >= static_cast<int>(result.path.size())) {
        return false;
    }
    Eigen::VectorXd repair_start;
    Eigen::VectorXd repair_goal;
    if (!collision_bracket(result.path[static_cast<std::size_t>(audit.failed_segment_index)],
                           result.path[static_cast<std::size_t>(audit.failed_segment_index + 1)],
                           checker,
                           query_config.audit_resolution,
                           repair_start,
                           repair_goal)) {
        return false;
    }

    RRTConnectConfig repair_config = base_repair_config;
    repair_config.max_iters = std::max(repair_config.max_iters, query_config.repair_rrt_max_iters);
    if (query_config.repair_timeout_ms > 0.0) {
        repair_config.timeout_ms = query_config.repair_timeout_ms;
    }
    repair_config.segment_resolution = std::max(repair_config.segment_resolution, query_config.audit_resolution);

    const int attempts = std::max(1, query_config.repair_max_attempts);
    std::vector<Eigen::VectorXd> best_repaired;
    double best_length = std::numeric_limits<double>::infinity();
    for (int attempt = 0; attempt < attempts; ++attempt) {
        RRTConnectConfig attempt_config = repair_config;
        if (query_config.repair_local_sampling_radius > 0.0 && attempt + 1 < attempts) {
            const double growth = std::max(1.0, query_config.repair_local_sampling_growth);
            attempt_config.local_sampling_radius = query_config.repair_local_sampling_radius * std::pow(growth, attempt);
        }
        auto repair_path = rrt_connect(repair_start,
                                       repair_goal,
                                       checker,
                                       robot,
                                       attempt_config,
                                       derived_planner_seed(planner_seed_base,
                                                            kSeedRepairLocalOffset,
                                                            attempt,
                                                            0,
                                                            audit.failed_segment_index));
        if (repair_path.empty()) {
            continue;
        }
        std::vector<Eigen::VectorXd> repaired;
        repaired.reserve(result.path.size() + repair_path.size() + 2);
        for (int index = 0; index <= audit.failed_segment_index; ++index) {
            append_waypoint_unique(repaired, result.path[static_cast<std::size_t>(index)]);
        }
        append_waypoint_unique(repaired, repair_start);
        for (const auto& waypoint : repair_path) {
            append_waypoint_unique(repaired, waypoint);
        }
        append_waypoint_unique(repaired, repair_goal);
        for (std::size_t index = static_cast<std::size_t>(audit.failed_segment_index + 1); index < result.path.size(); ++index) {
            append_waypoint_unique(repaired, result.path[index]);
        }
        if (audit_waypoint_path(repaired,
                                checker,
                                query_config.audit_resolution,
                                query_config.audit_segment_step)
                .passed) {
            if (query_config.collision_shortcut && repaired.size() > 2) {
                std::vector<Eigen::VectorXd> shortened = collision_shortcut_path(
                    repaired,
                    checker,
                    collision_shortcut_resolution(query_config));
                if (audit_waypoint_path(shortened,
                                        checker,
                                        query_config.audit_resolution,
                                        query_config.audit_segment_step)
                        .passed &&
                    path_length(shortened) <= path_length(repaired) + 1e-12) {
                    repaired = std::move(shortened);
                }
            }
            const double repaired_length = path_length(repaired);
            if (repaired_length < best_length) {
                best_length = repaired_length;
                best_repaired = std::move(repaired);
            }
        }
    }
    if (!best_repaired.empty()) {
        result.path = std::move(best_repaired);
        result.path_length = best_length;
        result.repair_count += 1;
        return true;
    }
    return false;
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
			std::abs(requested.lo - (primary_symmetry->canonical_lo - 2.0 * primary_symmetry->period)) <= 1e-9 &&
			std::abs(requested.hi - (primary_symmetry->canonical_lo + 2.0 * primary_symmetry->period)) <= 1e-9;
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
    if (root.lo + 1e-12 < symmetry.canonical_lo || root.hi - 1e-12 > symmetry.canonical_hi) {
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

RBFPlanningForest::RBFPlanningForest(Robot robot, RBFPlanningConfig config)
    : robot_(std::move(robot)), audit_robot_(make_sbf_audit_robot(robot_)),
      config_(std::move(config)) {
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
            // so we skip loading all node pages / evidence / indices (which can
            // dominate forest construction for large warm caches).
            external_config.open.metadata_only = true;
            if (!verifier.open(external_config, &verify_reason)) {
                throw std::runtime_error("failed to verify external LECTDatabase evidence source: " + verify_reason);
            }
            const auto snapshot_path = config_.database.external_evidence_snapshot_path.empty()
                ? lect_database::LectReadSnapshot::default_snapshot_path(config_.database.external_evidence_path)
                : config_.database.external_evidence_snapshot_path;
            if (config_.database.external_evidence_auto_build_snapshot && !std::filesystem::exists(snapshot_path)) {
                std::string build_reason;
                if (!lect_database::LectReadSnapshot::build_from_legacy(config_.database.external_evidence_path,
                                                                        snapshot_path,
                                                                        &build_reason)) {
                    throw std::runtime_error("failed to build external LECT snapshot: " + build_reason);
                }
            }
            external_evidence_snapshot_ = std::make_unique<lect_database::LectReadSnapshot>();
            std::string snapshot_reason;
            if (!external_evidence_snapshot_->open(snapshot_path, &snapshot_reason)) {
                throw std::runtime_error("failed to open external LECT snapshot evidence source: " + snapshot_reason);
            }
            external_evidence_snapshot_source_ = std::make_unique<lect_database::LectSnapshotEvidenceSource>(*external_evidence_snapshot_);
            external_evidence_source_ = external_evidence_snapshot_source_.get();
        } else {
            external_evidence_database_ = std::make_unique<lect_database::LectDatabase>();
            std::string external_reason;
            if (!external_evidence_database_->open(external_config, &external_reason)) {
                throw std::runtime_error("failed to open external LECTDatabase evidence source: " + external_reason);
            }
            external_evidence_database_source_ = std::make_unique<lect_database::LectDatabaseEvidenceSource>(*external_evidence_database_);
            external_evidence_source_ = external_evidence_database_source_.get();
            direct_external_evidence_database_ = external_evidence_database_.get();
        }
    }
    online_cache_ = std::make_unique<lect_database::OnlineEnvelopeCacheTree>(*database_, config_.database.online_cache);
    reset_oracle(Scene{});
}

BuildProfile RBFPlanningForest::build(const Eigen::Ref<const Eigen::VectorXd>& start,
                                  const Eigen::Ref<const Eigen::VectorXd>& goal,
                                  const std::vector<Obstacle>& obstacles) {
    StageContext context = StageContext::from_runtime(config_.runtime);
    return build(start, goal, obstacles, context);
}

BuildProfile RBFPlanningForest::build(const Eigen::Ref<const Eigen::VectorXd>& start,
                                  const Eigen::Ref<const Eigen::VectorXd>& goal,
                                  const std::vector<Obstacle>& obstacles,
                                  StageContext& context) {
    return build_coverage(obstacles, {start, goal}, context);
}

BuildProfile RBFPlanningForest::build_coverage(const std::vector<Obstacle>& obstacles,
                                           const std::vector<Eigen::VectorXd>& seeds) {
    StageContext context = StageContext::from_runtime(config_.runtime);
    return build_coverage(obstacles, seeds, context);
}

BuildProfile RBFPlanningForest::build_coverage(const std::vector<Obstacle>& obstacles,
                                           const std::vector<Eigen::VectorXd>& seeds,
                                           StageContext& context) {
    using Clock = std::chrono::steady_clock;
    ScopedStageTimer function_timer(context.diagnostics(), "forest.build_coverage");
    const auto t0 = Clock::now();
    last_build_seeds_ = seeds;
    scene_.set_obstacles(obstacles);
    reset_oracle(scene_);
    boxes_.clear();
    raw_boxes_.clear();
    adjacency_.clear();
    segment_edges_.clear();
    adaptive_partition_.reset();
    adaptive_partition_query_enabled_ = false;
    has_adaptive_partition_config_ = false;
    dynamic_collision_box_cache_.clear();
    invalidate_query_cache();

    const auto grow_t0 = Clock::now();
    auto grower = make_grower(*oracle_, config_.grower);
    auto grow = grower->grow(seeds, context);
    context.diagnostics().record_timing(
        "forest.grow_stage",
        std::chrono::duration<double, std::milli>(Clock::now() - grow_t0).count());
    boxes_ = std::move(grow.boxes);
    raw_boxes_ = boxes_;
    adjacency_ = std::move(grow.adjacency);
    last_build_ = {};
    last_build_.grow_ms = grow.build_time_ms;
    last_build_.raw_boxes = static_cast<int>(raw_boxes_.size());
    last_build_.grow_adjacency_islands = grow.adjacency_islands;
    last_build_.grow_largest_island = grow.adjacency_largest_island;

    const auto merge_t0 = Clock::now();
    if (config_.enable_merger && !boxes_.empty()) {
        Consolidator consolidator(*oracle_, config_.merger);
        consolidator.run(boxes_, context);
    }
    last_build_.merge_ms = std::chrono::duration<double, std::milli>(Clock::now() - merge_t0).count();
    context.diagnostics().record_timing("forest.merge_stage", last_build_.merge_ms);

    const auto connector_t0 = Clock::now();
    bool connector_ran = false;
    if (config_.enable_connector && !boxes_.empty() && !context.should_stop()) {
        rebuild_adjacency();
        CollisionChecker checker(robot_, scene_);
        IslandConnectorConfig connector_config = config_.connector;
        if (connector_config.n_threads <= 1 && config_.runtime.n_threads > 1) {
            connector_config.n_threads = config_.runtime.n_threads;
        }
        IslandConnector connector(*oracle_, robot_, checker, connector_config);
        int next_id = next_box_id();
        const auto connector_result = connector.connect_all(boxes_, adjacency_, segment_edges_, next_id, context);
        last_build_.bridge_boxes_added = connector_result.bridge_boxes_added;
        last_build_.segment_edges_added = connector_result.segment_edges_added;
        last_build_.rrt_segment_edges_added = connector_result.rrt_segment_edges_added;
        last_build_.point_gap_segment_edges_added = connector_result.point_gap_segment_edges_added;
        last_build_.connector_attempted_pairs = connector_result.attempted_pairs;
        last_build_.connector_connected = connector_result.connected;
        connector_ran = true;
    }
    last_build_.connector_ms = std::chrono::duration<double, std::milli>(Clock::now() - connector_t0).count();
    context.diagnostics().record_timing("forest.connector_stage", last_build_.connector_ms);

    const auto adj_t0 = Clock::now();
    last_build_.segment_edges = static_cast<int>(segment_edges_.size());
    if (!connector_ran) {
        rebuild_adjacency();
    }
    last_build_.adjacency_ms = std::chrono::duration<double, std::milli>(Clock::now() - adj_t0).count();
    context.diagnostics().record_timing("forest.adjacency_stage", last_build_.adjacency_ms);
    last_build_.final_boxes = static_cast<int>(boxes_.size());
    last_build_.adjacency_islands = static_cast<int>(find_islands(adjacency_).size());
    context.diagnostics().set_value("grower.adjacency_islands", static_cast<double>(last_build_.grow_adjacency_islands));
    context.diagnostics().set_value("grower.adjacency_largest_island", static_cast<double>(last_build_.grow_largest_island));
    const OracleCounters oracle_counters = oracle_->counters();
    context.diagnostics().set_value("oracle.node_validations", static_cast<double>(oracle_counters.node_validations));
    context.diagnostics().set_value("oracle.interval_validations", static_cast<double>(oracle_counters.interval_validations));
    context.diagnostics().set_value("oracle.materializations", static_cast<double>(oracle_counters.materializations));
    context.diagnostics().set_value("oracle.materialization_stored_endpoint", static_cast<double>(oracle_counters.materialization_stored_endpoint));
    context.diagnostics().set_value("oracle.materialization_skipped_endpoint_cache", static_cast<double>(oracle_counters.materialization_skipped_endpoint_cache));
    context.diagnostics().set_value("oracle.materialization_incremental_fk", static_cast<double>(oracle_counters.materialization_incremental_fk));
    context.diagnostics().set_value("oracle.materialization_source_incremental_state", static_cast<double>(oracle_counters.materialization_source_incremental_state));
    context.diagnostics().set_value("oracle.materialization_reused_fk", static_cast<double>(oracle_counters.materialization_reused_fk));
    context.diagnostics().set_value("oracle.materialization_reused_endpoint_cache", static_cast<double>(oracle_counters.materialization_reused_endpoint_cache));
    context.diagnostics().set_value("oracle.materialization_reused_external_evidence", static_cast<double>(oracle_counters.materialization_reused_external_evidence));
    context.diagnostics().set_value("oracle.materialization_external_exact_hits", static_cast<double>(oracle_counters.materialization_external_exact_hits));
    context.diagnostics().set_value("oracle.materialization_external_exact_misses", static_cast<double>(oracle_counters.materialization_external_exact_misses));
    context.diagnostics().set_value("oracle.materialization_external_live_fallbacks", static_cast<double>(oracle_counters.materialization_external_live_fallbacks));
    context.diagnostics().set_value("oracle.materialization_external_maybe_live_retries", static_cast<double>(oracle_counters.materialization_external_maybe_live_retries));
    context.diagnostics().set_value("oracle.materialization_external_maybe_live_retry_free", static_cast<double>(oracle_counters.materialization_external_maybe_live_retry_free));
    context.diagnostics().set_value("oracle.interval_replay_compatibility_checks", static_cast<double>(oracle_counters.interval_replay_compatibility_checks));
    context.diagnostics().set_value("oracle.interval_replay_compatible", static_cast<double>(oracle_counters.interval_replay_compatible));
    context.diagnostics().set_value("oracle.interval_replay_incompatible", static_cast<double>(oracle_counters.interval_replay_incompatible));
    context.diagnostics().set_value("oracle.interval_replay_direct_exact_hits", static_cast<double>(oracle_counters.interval_replay_direct_exact_hits));
    context.diagnostics().set_value("oracle.interval_replay_key_only_blocked", static_cast<double>(oracle_counters.interval_replay_key_only_blocked));
    context.diagnostics().set_value("oracle.canonical_frame_invalid", static_cast<double>(oracle_counters.canonical_frame_invalid));
    context.diagnostics().set_value("oracle.canonical_reflected_seed_misses", static_cast<double>(oracle_counters.canonical_reflected_seed_misses));
    context.diagnostics().set_value("oracle.materialization_reused_shared_endpoint_cache", static_cast<double>(oracle_counters.materialization_reused_shared_endpoint_cache));
    context.diagnostics().set_value("oracle.materialization_stored_shared_endpoint_cache", static_cast<double>(oracle_counters.materialization_stored_shared_endpoint_cache));
    if (const auto* shared_cache = oracle_->shared_endpoint_cache_peek()) {
        context.diagnostics().set_value("oracle.shared_endpoint_cache_size", static_cast<double>(shared_cache->size()));
        context.diagnostics().set_value("oracle.shared_endpoint_cache_bytes", static_cast<double>(shared_cache->bytes()));
        context.diagnostics().set_value("oracle.shared_endpoint_cache_evictions", static_cast<double>(shared_cache->evictions()));
    }
    context.diagnostics().set_value("oracle.materialization_endpoint_time_us", oracle_counters.materialization_endpoint_time_us);
    context.diagnostics().set_value("oracle.materialization_endpoint_wall_time_us", oracle_counters.materialization_endpoint_wall_time_us);
    context.diagnostics().set_value("oracle.validate_node_total_time_us", oracle_counters.validate_node_total_time_us);
    context.diagnostics().set_value("oracle.validate_node_preamble_time_us", oracle_counters.validate_node_preamble_time_us);
    context.diagnostics().set_value("oracle.validate_node_endpoint_path_time_us", oracle_counters.validate_node_endpoint_path_time_us);
    context.diagnostics().set_value("oracle.validate_node_classify_time_us", oracle_counters.validate_node_classify_time_us);
    context.diagnostics().set_value("oracle.validate_node_overhead_time_us", oracle_counters.validate_node_overhead_time_us);
    context.diagnostics().set_value("oracle.materialization_envelope_time_us", oracle_counters.materialization_envelope_time_us);
    context.diagnostics().set_value("oracle.materialization_cache_lookup_time_us", oracle_counters.materialization_cache_lookup_time_us);
    context.diagnostics().set_value("oracle.materialization_cache_read_time_us", oracle_counters.materialization_cache_read_time_us);
    context.diagnostics().set_value("oracle.materialization_external_lookup_time_us", oracle_counters.materialization_external_lookup_time_us);
    context.diagnostics().set_value("oracle.materialization_external_read_time_us", oracle_counters.materialization_external_read_time_us);
    context.diagnostics().set_value("oracle.materialization_envelope_compute_time_us", oracle_counters.materialization_envelope_compute_time_us);
    context.diagnostics().set_value("oracle.materialization_envelope_read_time_us", oracle_counters.materialization_envelope_read_time_us);
    context.diagnostics().set_value("oracle.materialization_envelope_collision_time_us", oracle_counters.materialization_envelope_collision_time_us);
    context.diagnostics().set_value("oracle.materialization_candidate_dirty_count", static_cast<double>(oracle_counters.materialization_candidate_dirty_count));
    context.diagnostics().set_value("oracle.materialization_predh_rebuild_count", static_cast<double>(oracle_counters.materialization_predh_rebuild_count));
    context.diagnostics().set_value("oracle.scoring_evaluations", static_cast<double>(oracle_counters.scoring_evaluations));
    context.diagnostics().set_value("oracle.scoring_changed_dim_inferred", static_cast<double>(oracle_counters.scoring_changed_dim_inferred));
    context.diagnostics().set_value("oracle.scoring_incremental_fk", static_cast<double>(oracle_counters.scoring_incremental_fk));
    context.diagnostics().set_value("oracle.scoring_source_incremental_state", static_cast<double>(oracle_counters.scoring_source_incremental_state));
    context.diagnostics().set_value("oracle.scoring_reused_fk", static_cast<double>(oracle_counters.scoring_reused_fk));
    context.diagnostics().set_value("oracle.scoring_reused_endpoint_cache", static_cast<double>(oracle_counters.scoring_reused_endpoint_cache));
    context.diagnostics().set_value("oracle.scoring_reused_external_evidence", static_cast<double>(oracle_counters.scoring_reused_external_evidence));
    context.diagnostics().set_value("oracle.scoring_endpoint_time_us", oracle_counters.scoring_endpoint_time_us);
    context.diagnostics().set_value("oracle.scoring_envelope_time_us", oracle_counters.scoring_envelope_time_us);
    context.diagnostics().set_value("oracle.scoring_candidate_dirty_count", static_cast<double>(oracle_counters.scoring_candidate_dirty_count));
    context.diagnostics().set_value("oracle.scoring_predh_rebuild_count", static_cast<double>(oracle_counters.scoring_predh_rebuild_count));
    context.diagnostics().set_value("oracle.certified_free", static_cast<double>(oracle_counters.certified_free));
    context.diagnostics().set_value("oracle.certified_occupied", static_cast<double>(oracle_counters.certified_occupied));
    context.diagnostics().set_value("oracle.provisional_free", static_cast<double>(oracle_counters.provisional_free));
    context.diagnostics().set_value("oracle.collision_possible", static_cast<double>(oracle_counters.collision_possible));
    context.diagnostics().set_value("oracle.validation_cache_hits", static_cast<double>(oracle_counters.validation_cache_hits));
    context.diagnostics().set_value("oracle.validation_cache_misses", static_cast<double>(oracle_counters.validation_cache_misses));
    context.diagnostics().set_value("oracle.unsafe_free_rejected", static_cast<double>(oracle_counters.unsafe_free_rejected));
    context.diagnostics().set_value("oracle.envelope_collision_queries", static_cast<double>(oracle_counters.envelope_collision_queries));
    context.diagnostics().set_value("oracle.envelope_collision_free", static_cast<double>(oracle_counters.envelope_collision_free));
    context.diagnostics().set_value("oracle.envelope_collision_maybe", static_cast<double>(oracle_counters.envelope_collision_maybe));
    context.diagnostics().set_value("oracle.envelope_collision_envelope_aabb_tests", static_cast<double>(oracle_counters.envelope_collision_envelope_aabb_tests));
    context.diagnostics().set_value("oracle.envelope_collision_envelope_aabb_rejects", static_cast<double>(oracle_counters.envelope_collision_envelope_aabb_rejects));
    context.diagnostics().set_value("oracle.envelope_collision_link_union_aabb_tests", static_cast<double>(oracle_counters.envelope_collision_link_union_aabb_tests));
    context.diagnostics().set_value("oracle.envelope_collision_link_union_aabb_rejects", static_cast<double>(oracle_counters.envelope_collision_link_union_aabb_rejects));
    context.diagnostics().set_value("oracle.envelope_collision_link_aabb_tests", static_cast<double>(oracle_counters.envelope_collision_link_aabb_tests));
    context.diagnostics().set_value("oracle.envelope_collision_link_aabb_rejects", static_cast<double>(oracle_counters.envelope_collision_link_aabb_rejects));
    context.diagnostics().set_value("oracle.envelope_collision_kdop_tests", static_cast<double>(oracle_counters.envelope_collision_kdop_tests));
    context.diagnostics().set_value("oracle.envelope_collision_kdop_rejects", static_cast<double>(oracle_counters.envelope_collision_kdop_rejects));
    context.diagnostics().set_value("oracle.envelope_collision_kdop_axes_tested", static_cast<double>(oracle_counters.envelope_collision_kdop_axes_tested));
    context.diagnostics().set_value("oracle.envelope_collision_gjk_tests", static_cast<double>(oracle_counters.envelope_collision_gjk_tests));
    context.diagnostics().set_value("oracle.envelope_collision_gjk_rejects", static_cast<double>(oracle_counters.envelope_collision_gjk_rejects));
    context.diagnostics().set_value("oracle.envelope_collision_gjk_iterations", static_cast<double>(oracle_counters.envelope_collision_gjk_iterations));
    record_portal_membership_policy(context.diagnostics(), config_.portal_membership_policy);
    last_build_.diagnostics = context.diagnostics().snapshot();
    last_build_.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
    invalidate_query_cache();

    if (config_.database.checkpoint_after_build && database_) {
        database_->checkpoint();
    }
    return last_build_;
}

LeafSweepResult RBFPlanningForest::build_leaf_sweep(const std::vector<Obstacle>& obstacles,
                                                    int start_depth,
                                                    int max_depth,
                                                    const LeafSweepConfig& leaf_sweep_config) {
    LeafSweepConfig active_config = leaf_sweep_config;
    const int configured_threads = active_config.n_threads > 0
        ? active_config.n_threads
        : std::max(1, config_.runtime.n_threads);
    active_config.n_threads = configured_threads;
    RuntimeConfig runtime = config_.runtime;
    runtime.mode = configured_threads > 1 ? ExecutionMode::Parallel : ExecutionMode::Inline;
    runtime.n_threads = configured_threads;
    runtime.batch_size = std::max(1, active_config.validation_batch_size);
    StageContext context(runtime, Deadline::after_ms(active_config.timeout_ms));
    return build_leaf_sweep(obstacles, start_depth, max_depth, active_config, context);
}

LeafSweepResult RBFPlanningForest::build_leaf_sweep(const std::vector<Obstacle>& obstacles,
                                                    int start_depth,
                                                    int max_depth,
                                                    const LeafSweepConfig& leaf_sweep_config,
                                                    StageContext& context) {
    ScopedStageTimer function_timer(context.diagnostics(), "forest.build_leaf_sweep");
    last_build_seeds_.clear();
    scene_.set_obstacles(obstacles);
    const bool previous_stateless_materialization = config_.validation.stateless_materialization_context;
    if (leaf_sweep_config.use_virtual_topology) {
        config_.validation.stateless_materialization_context = true;
    }
    reset_oracle(scene_);
    boxes_.clear();
    raw_boxes_.clear();
    adjacency_.clear();
    segment_edges_.clear();
    dynamic_collision_box_cache_.clear();
    invalidate_query_cache();

    LeafSweepConfig active_config = leaf_sweep_config;
    if (active_config.n_threads <= 0) {
        active_config.n_threads = std::max(1, context.executor().n_threads());
    }
    if (!active_config.use_virtual_topology && oracle_ &&
        oracle_->native_root_interval_copies().size() > 1) {
        active_config.use_virtual_topology = true;
        config_.validation.stateless_materialization_context = true;
        reset_oracle(scene_);
        context.diagnostics().add_counter("forest.leaf_sweep_forced_virtual_for_native_sectors");
    }
    LeafSweepGrower grower(*oracle_, active_config, config_.grower.find_free_box.split);
    LeafSweepResult result = grower.sweep(obstacles, start_depth, max_depth, context);

    scene_.set_obstacles(obstacles);
    config_.validation.stateless_materialization_context = previous_stateless_materialization;
    oracle_->set_scene(scene_);
    boxes_ = result.free_boxes;
    raw_boxes_ = boxes_;
    populate_dynamic_collision_cache(result, static_cast<int>(obstacles.size()));
    reserve_existing_boxes();
    adjacency_.clear();
    segment_edges_.clear();
    invalidate_query_cache();

    last_build_ = {};
    last_build_.grow_ms = result.total_ms;
    last_build_.raw_boxes = static_cast<int>(raw_boxes_.size());
    last_build_.final_boxes = static_cast<int>(boxes_.size());
    last_build_.total_ms = result.total_ms;
    last_build_.diagnostics = result.diagnostics;
    if (config_.database.checkpoint_after_build && database_) {
        database_->checkpoint();
    }
    return result;
}

AdaptiveLeafSweepResult RBFPlanningForest::build_adaptive_deep_leaf_sweep_cover(
    const std::vector<Obstacle>& obstacles,
    const AdaptiveLeafSweepConfig& adaptive_config) {
    using Clock = std::chrono::steady_clock;
    const auto total_start = Clock::now();
    AdaptiveLeafSweepResult out;
    out.diagnostics["adaptive.offline_query_agnostic_build"] = 1.0;
    out.diagnostics["adaptive.qroot_pairs_total"] = 0.0;
    out.diagnostics["adaptive.qroot_uncovered_endpoints"] = 0.0;
    out.diagnostics["adaptive.fast_virtual_checkpoint_mode"] =
        adaptive_config.fast_virtual_checkpoint_mode ? 1.0 : 0.0;
    out.diagnostics["adaptive.terminal_controller_enabled"] =
        adaptive_config.fast_virtual_checkpoint_mode ? 0.0 : 1.0;

    const bool adaptive_depth_enabled = adaptive_config.adaptive_depth_enabled;
    const int adaptive_depth_min = std::max(
        adaptive_config.shallow_start_depth,
        adaptive_config.adaptive_depth_min > 0
            ? adaptive_config.adaptive_depth_min
            : adaptive_config.shallow_max_depth);
    const int adaptive_depth_max = std::max(
        adaptive_depth_min,
        adaptive_config.adaptive_depth_max > 0
            ? adaptive_config.adaptive_depth_max
            : adaptive_config.target_max_depth);
    const int initial_leaf_depth = adaptive_depth_enabled
        ? adaptive_depth_min
        : adaptive_config.shallow_max_depth;
    const int target_leaf_depth = adaptive_depth_enabled
        ? adaptive_depth_max
        : adaptive_config.target_max_depth;

    LeafSweepConfig leaf_config;
    leaf_config.obstacle_cluster_gap = adaptive_config.obstacle_cluster_gap;
    leaf_config.n_threads = std::max(1, adaptive_config.threads);
    leaf_config.validation_batch_size = std::max(1, adaptive_config.validation_batch_size);
    leaf_config.timeout_ms = adaptive_config.time_budget_ms > 0.0
        ? adaptive_config.time_budget_ms
        : 0.0;
    leaf_config.store_group_results = adaptive_config.store_group_results;
    leaf_config.use_virtual_topology = adaptive_config.use_virtual_topology;
    leaf_config.parallel_virtual_validation = adaptive_config.parallel_virtual_validation;
    leaf_config.max_free_boxes = std::max(0, adaptive_config.max_free_boxes);
    leaf_config.max_collision_boxes = std::max(0, adaptive_config.max_unresolved_domains);
    leaf_config.collision_overlap_prune_min_depth = -1;
    leaf_config.collision_overlap_prune_threshold = 0.0;
    leaf_config.collision_overlap_prune_min_threshold = 0.0;
    leaf_config.collision_overlap_prune_decay_per_depth = 0.0;
    leaf_config.collision_overlap_prune_ratio_threshold = 0.0;

    AdaptiveLeafSweepConfig partition_config = adaptive_config;
    partition_config.shallow_max_depth = initial_leaf_depth;
    partition_config.target_max_depth = target_leaf_depth;
    if (adaptive_depth_enabled || partition_config.grid_target_depth <= 0) {
        partition_config.grid_target_depth = target_leaf_depth;
    }

    auto next_fast_depth_checkpoint = [&](int depth) {
        const int step = depth < 16 ? 1 : 2;
        return std::min(target_leaf_depth, depth + step);
    };
    auto snapshot_readiness_met = [&](const AdaptiveDepthSnapshot& snapshot) {
        const int min_covered_probes = std::max(0, adaptive_config.adaptive_depth_min_covered_probes);
        const int min_main_probes = std::max(0, adaptive_config.adaptive_depth_min_main_probes);
        const int min_cells = std::max(0, adaptive_config.adaptive_depth_min_cells);
        const int min_main_cells = std::max(0, adaptive_config.adaptive_depth_min_main_cells);
        const bool probe_gate =
            snapshot.covered_count >= min_covered_probes &&
            snapshot.main_accessible_count >= min_main_probes &&
            (min_covered_probes <= 0 ||
             snapshot.main_connected_ratio >= adaptive_config.adaptive_depth_min_main_ratio);
        const bool cell_gate =
            snapshot.cell_count >= min_cells &&
            snapshot.main_island_cell_count >= min_main_cells;
        return probe_gate &&
               cell_gate &&
               (adaptive_config.adaptive_depth_max_online_cells <= 0 ||
                snapshot.cell_count <= adaptive_config.adaptive_depth_max_online_cells);
    };
    auto snapshot_from_fast_candidate = [&](const AdaptiveLeafSweepResult& candidate,
                                            int depth) {
        AdaptiveDepthSnapshot snapshot;
        snapshot.depth = depth;
        snapshot.free_probe_count = candidate.seed_probe_free_count;
        snapshot.covered_count = candidate.seed_probe_box_covered;
        snapshot.main_accessible_count =
            std::min(candidate.seed_probe_main_accessible, candidate.seed_probe_box_covered);
        snapshot.anchor_success_count = candidate.seed_probe_anchor_success;
        snapshot.anchor_to_main_count =
            std::max(0, candidate.seed_probe_main_accessible - snapshot.main_accessible_count);
        const auto attempts_it = candidate.profile.diagnostics.find("adaptive.seed_anchor_probe_attempts");
        if (attempts_it != candidate.profile.diagnostics.end()) {
            snapshot.anchor_probe_attempts = static_cast<int>(std::llround(attempts_it->second));
        }
        snapshot.cell_count = candidate.partition_cell_count > 0
            ? candidate.partition_cell_count
            : candidate.profile.final_boxes;
        snapshot.collision_count = candidate.shallow_collision_count;
        snapshot.island_count = candidate.partition_islands > 0
            ? candidate.partition_islands
            : candidate.profile.adjacency_islands;
        snapshot.main_island_cell_count = candidate.partition_largest_island > 0
            ? candidate.partition_largest_island
            : candidate.profile.grow_largest_island;
        const double free_den = static_cast<double>(std::max(1, snapshot.free_probe_count));
        snapshot.p_box_covered = static_cast<double>(snapshot.covered_count) / free_den;
        snapshot.p_main_accessible = static_cast<double>(snapshot.main_accessible_count) / free_den;
        snapshot.main_connected_ratio =
            static_cast<double>(snapshot.main_accessible_count) /
            static_cast<double>(std::max(1, snapshot.covered_count));
        snapshot.p_anchor_to_main_uncovered =
            static_cast<double>(snapshot.anchor_to_main_count) /
            static_cast<double>(std::max(1, snapshot.anchor_probe_attempts));
        snapshot.probe_ms = candidate.coverage_probe_ms;
        snapshot.readiness_met = snapshot_readiness_met(snapshot);
        return snapshot;
    };
    if (adaptive_depth_enabled && adaptive_config.fast_virtual_checkpoint_mode) {
        std::vector<AdaptiveDepthSnapshot> depth_snapshots;
        AdaptiveLeafSweepResult selected;
        bool have_selected = false;
        double accumulated_leaf_sweep_ms = 0.0;
        auto materialize_fast_checkpoint_candidate = [&](const LeafSweepResult& leaf_result,
                                                         int depth,
                                                         int sweep_count) {
            AdaptiveLeafSweepResult candidate;
            candidate.leaf_sweep = leaf_result;
            candidate.leaf_sweep_ms = accumulated_leaf_sweep_ms;
            candidate.selected_leaf_depth = depth;
            candidate.shallow_free_count = static_cast<int>(leaf_result.free_boxes.size());
            candidate.shallow_collision_count = static_cast<int>(leaf_result.collision_boxes.size());
            candidate.adaptive_deferred = static_cast<int>(leaf_result.collision_boxes.size());
            candidate.unresolved_domains = static_cast<int>(leaf_result.collision_boxes.size());

            scene_.set_obstacles(obstacles);
            if (oracle_) {
                oracle_->set_scene(scene_);
            }
            boxes_ = leaf_result.free_boxes;
            raw_boxes_ = boxes_;
            adjacency_.clear();
            segment_edges_.clear();
            dynamic_collision_box_cache_.clear();
            invalidate_query_cache();
            populate_dynamic_collision_cache(leaf_result, static_cast<int>(obstacles.size()));
            reserve_existing_boxes();

            const double adjacency_tolerance = config_.query.adjacency_tolerance;
            const auto merge_start = Clock::now();
            BudgetedMergeStats merge_stats;
            if (config_.enable_merger && !boxes_.empty()) {
                bool merged_by_partition = false;
                if (adaptive_config.planning_backend == "partition_native") {
                    rebuild_adaptive_partition(partition_config, nullptr);
                    if (adaptive_partition_query_enabled_ && adaptive_partition_) {
                        AdaptiveGridPartitionMergeOptions options;
                        options.max_ms = adaptive_config.max_merge_ms;
                        options.max_rounds = adaptive_config.max_merge_rounds;
                        options.grid_line_merge = true;
                        options.containment_prune = false;
                        const auto partition_merge =
                            adaptive_partition_->merge_boxes(boxes_, options, adjacency_tolerance);
                        for (int released_id : partition_merge.released_box_ids) {
                            oracle_->release_box(released_id);
                        }
                        merge_stats.input_boxes = partition_merge.input_boxes;
                        merge_stats.output_boxes = partition_merge.output_boxes;
                        merge_stats.grid_merges = partition_merge.grid_merges;
                        merge_stats.grid_rounds = partition_merge.rounds;
                        merge_stats.containment_pruned = partition_merge.containment_pruned;
                        merge_stats.stop_reason =
                            partition_merge.stop_reason == 0 ? 1 : partition_merge.stop_reason;
                        merge_stats.total_ms = partition_merge.total_ms;
                        merge_stats.grid_ms = partition_merge.total_ms;
                        candidate.diagnostics["adaptive.partition_merge_enabled"] = 1.0;
                        candidate.diagnostics["adaptive.partition_merge_released_boxes"] =
                            static_cast<double>(partition_merge.released_box_ids.size());
                        merged_by_partition = true;
                    }
                }
                if (!merged_by_partition) {
                    MergerConfig leaf_merge_config = config_.merger;
                    leaf_merge_config.containment_prune = true;
                    merge_stats = budgeted_leaf_merge(*oracle_,
                                                      boxes_,
                                                      leaf_merge_config,
                                                      adaptive_config.max_merge_ms,
                                                      adaptive_config.max_merge_rounds,
                                                      adaptive_config.max_merge_input_boxes,
                                                      adjacency_tolerance);
                }
                raw_boxes_ = boxes_;
            } else {
                merge_stats.input_boxes = static_cast<int>(boxes_.size());
                merge_stats.output_boxes = static_cast<int>(boxes_.size());
                merge_stats.stop_reason = 0;
            }
            const double merge_ms =
                std::chrono::duration<double, std::milli>(Clock::now() - merge_start).count();

            const bool use_partition_backend = adaptive_config.planning_backend == "partition_native";
            AdjacencyBuildStats adjacency_stats;
            std::unordered_set<int> main_ids;
            int partition_island_count_for_profile = 0;
            int partition_largest_island_for_profile = 0;
            if (use_partition_backend) {
                if (!adaptive_partition_query_enabled_ || !adaptive_partition_) {
                    rebuild_adaptive_partition(partition_config, nullptr);
                }
                if (adaptive_partition_query_enabled_ && adaptive_partition_) {
                    const auto largest = adaptive_partition_->largest_component_box_ids_with_overlay();
                    main_ids.insert(largest.begin(), largest.end());
                    const auto& partition_stats = adaptive_partition_->stats();
                    partition_island_count_for_profile = partition_stats.islands;
                    partition_largest_island_for_profile = static_cast<int>(largest.size());
                    candidate.diagnostics["adaptive.partition_skipped_graph_adjacency"] = 1.0;
                }
            }
            if (!use_partition_backend) {
                rebuild_adjacency();
                adjacency_stats = last_adjacency_build_stats();
                main_ids = adaptive_largest_island_ids(adjacency_);
            } else if (main_ids.empty()) {
                candidate.diagnostics["adaptive.partition_missing_no_graph_fallback"] = 1.0;
            }

            const auto coverage_start = Clock::now();
            const auto planning_domain = oracle_ ? oracle_->planning_intervals() : std::vector<Interval>{};
            int probe_attempted = 0;
            std::vector<Eigen::VectorXd> free_probes =
                oracle_ ? adaptive_generate_free_probes(*oracle_,
                                                        planning_domain,
                                                        std::max(0, adaptive_config.adaptive_depth_probe_count),
                                                        adaptive_config.adaptive_depth_probe_seed,
                                                        probe_attempted)
                        : std::vector<Eigen::VectorXd>{};
            candidate.seed_probe_count = probe_attempted;
            candidate.seed_probe_free_count = static_cast<int>(free_probes.size());
            int uncovered_anchor_attempts = 0;
            StageContext probe_context = StageContext::from_runtime(config_.runtime);
            FindFreeBoxOptions probe_options = config_.grower.find_free_box;
            probe_options.max_depth = target_leaf_depth;
            probe_options.reject_seed_collision = false;
            probe_options.deadline_ms = std::max(1.0, adaptive_config.adaptive_depth_max_probe_ms);
            const int anchor_cap = std::max(0, adaptive_config.adaptive_depth_anchor_probe_cap);
            BoxSpatialIndex coverage_index;
            const bool use_partition_coverage =
                use_partition_backend && adaptive_partition_query_enabled_ && adaptive_partition_;
            if (!use_partition_coverage) {
                coverage_index.rebuild(boxes_, adjacency_tolerance);
            }
            for (const auto& point : free_probes) {
                const int owner = use_partition_coverage
                    ? adaptive_partition_->locate_containing_box(point, false, adjacency_tolerance)
                    : [&]() {
                          const int owner_index = coverage_index.covering_box(boxes_, point, adjacency_tolerance);
                          return owner_index >= 0 ? boxes_[static_cast<std::size_t>(owner_index)].id : -1;
                      }();
                if (owner >= 0) {
                    candidate.seed_probe_box_covered += 1;
                    if (main_ids.find(owner) != main_ids.end()) {
                        candidate.seed_probe_main_accessible += 1;
                    }
                    continue;
                }
                if (uncovered_anchor_attempts >= anchor_cap || planning_domain.empty()) {
                    continue;
                }
                ++uncovered_anchor_attempts;
                const auto ffb = find_free_box_in_domain(point, planning_domain, probe_context, probe_options);
                if (!ffb.found) {
                    continue;
                }
                candidate.seed_probe_anchor_success += 1;
                const BoxNode anchor = adaptive_make_box_from_intervals(
                    ffb.intervals,
                    ffb.node,
                    -1,
                    ffb.validation_detail.safety_status,
                    ffb.validation_detail.strict_audit_required);
                const bool anchor_main_accessible = use_partition_coverage
                    ? adaptive_partition_->box_adjacent_to_any(anchor, main_ids, adjacency_tolerance)
                    : (!use_partition_backend &&
                       adaptive_has_adjacency_to_any(boxes_, anchor, &main_ids, adjacency_tolerance));
                if (anchor_main_accessible) {
                    candidate.seed_probe_main_accessible += 1;
                }
            }
            candidate.coverage_probe_ms =
                std::chrono::duration<double, std::milli>(Clock::now() - coverage_start).count();
            const double free_den = static_cast<double>(std::max(1, candidate.seed_probe_free_count));
            candidate.p_box_covered = static_cast<double>(candidate.seed_probe_box_covered) / free_den;
            candidate.p_anchor_success = static_cast<double>(candidate.seed_probe_anchor_success) / free_den;
            candidate.p_main_accessible = static_cast<double>(candidate.seed_probe_main_accessible) / free_den;

            candidate.total_ms =
                std::chrono::duration<double, std::milli>(Clock::now() - total_start).count();
            candidate.profile = {};
            candidate.profile.raw_boxes = static_cast<int>(raw_boxes_.size());
            candidate.profile.final_boxes = static_cast<int>(boxes_.size());
            candidate.profile.segment_edges = static_cast<int>(segment_edges_.size());
            candidate.profile.grow_ms = candidate.total_ms;
            candidate.profile.total_ms = candidate.total_ms;
            if (use_partition_backend) {
                candidate.profile.grow_adjacency_islands = partition_island_count_for_profile;
                candidate.profile.adjacency_islands = partition_island_count_for_profile;
                candidate.profile.grow_largest_island = partition_largest_island_for_profile;
            } else {
                const auto graph_islands = find_islands(adjacency_);
                candidate.profile.grow_adjacency_islands = static_cast<int>(graph_islands.size());
                candidate.profile.adjacency_islands = candidate.profile.grow_adjacency_islands;
                for (const auto& island : graph_islands) {
                    candidate.profile.grow_largest_island =
                        std::max(candidate.profile.grow_largest_island, static_cast<int>(island.size()));
                }
            }
            candidate.profile.diagnostics = leaf_result.diagnostics;
            for (const auto& [key, value] : candidate.diagnostics) {
                candidate.profile.diagnostics[key] = value;
            }
            candidate.profile.diagnostics["adaptive.fast_checkpoint_mode"] = 1.0;
            candidate.profile.diagnostics["adaptive.fast_virtual_checkpoint_mode"] = 1.0;
            candidate.profile.diagnostics["adaptive.terminal_controller_enabled"] = 0.0;
            candidate.profile.diagnostics["adaptive.in_sweep_checkpoint_mode"] = 1.0;
            candidate.profile.diagnostics["adaptive.in_sweep_checkpoints"] =
                static_cast<double>(sweep_count);
            candidate.profile.diagnostics["adaptive.offline_query_agnostic_build"] = 1.0;
            candidate.profile.diagnostics["adaptive.qroot_pairs_total"] = 0.0;
            candidate.profile.diagnostics["adaptive.qroot_uncovered_endpoints"] = 0.0;
            candidate.profile.diagnostics["adaptive.leaf_sweep_ms"] = candidate.leaf_sweep_ms;
            candidate.profile.diagnostics["adaptive.merge_ms"] = merge_ms;
            candidate.profile.diagnostics["adaptive.merge_input_boxes"] =
                static_cast<double>(merge_stats.input_boxes);
            candidate.profile.diagnostics["adaptive.merge_output_boxes"] =
                static_cast<double>(merge_stats.output_boxes);
            candidate.profile.diagnostics["adaptive.adjacency_ms"] = adjacency_stats.build_ms;
            candidate.profile.diagnostics["adaptive.adjacency_boxes"] =
                static_cast<double>(adjacency_stats.boxes);
            candidate.profile.diagnostics["adaptive.adjacency_selected_dims"] =
                static_cast<double>(adjacency_stats.selected_dims);
            candidate.profile.diagnostics["adaptive.adjacency_primary_dim"] =
                static_cast<double>(adjacency_stats.primary_dim);
            candidate.profile.diagnostics["adaptive.adjacency_candidates"] =
                static_cast<double>(adjacency_stats.candidate_pairs);
            candidate.profile.diagnostics["adaptive.adjacency_exact_tests"] =
                static_cast<double>(adjacency_stats.exact_tests);
            candidate.profile.diagnostics["adaptive.adjacency_edges"] =
                static_cast<double>(adjacency_stats.edges);
            candidate.profile.diagnostics["adaptive.coverage_probe_ms"] = candidate.coverage_probe_ms;
            candidate.profile.diagnostics["adaptive.total_ms"] = candidate.total_ms;
            candidate.profile.diagnostics["adaptive.shallow_free_count"] =
                static_cast<double>(candidate.shallow_free_count);
            candidate.profile.diagnostics["adaptive.shallow_collision_count"] =
                static_cast<double>(candidate.shallow_collision_count);
            candidate.profile.diagnostics["adaptive.seed_probe_count"] =
                static_cast<double>(candidate.seed_probe_count);
            candidate.profile.diagnostics["adaptive.seed_probe_free_count"] =
                static_cast<double>(candidate.seed_probe_free_count);
            candidate.profile.diagnostics["adaptive.seed_probe_box_covered"] =
                static_cast<double>(candidate.seed_probe_box_covered);
            candidate.profile.diagnostics["adaptive.seed_probe_anchor_success"] =
                static_cast<double>(candidate.seed_probe_anchor_success);
            candidate.profile.diagnostics["adaptive.seed_probe_main_accessible"] =
                static_cast<double>(candidate.seed_probe_main_accessible);
            candidate.profile.diagnostics["adaptive.seed_anchor_probe_cap"] =
                static_cast<double>(anchor_cap);
            candidate.profile.diagnostics["adaptive.seed_anchor_probe_attempts"] =
                static_cast<double>(uncovered_anchor_attempts);
            candidate.profile.diagnostics["adaptive.p_box_covered"] = candidate.p_box_covered;
            candidate.profile.diagnostics["adaptive.p_anchor_success"] = candidate.p_anchor_success;
            candidate.profile.diagnostics["adaptive.p_main_accessible"] = candidate.p_main_accessible;
            rebuild_adaptive_partition(partition_config, &candidate.profile);
            if (adaptive_partition_ && !adaptive_partition_->empty()) {
                const auto& partition_stats = adaptive_partition_->stats();
                candidate.partition_cell_count = partition_stats.cells;
                candidate.partition_grid_cell_count = partition_stats.grid_cells;
                candidate.partition_non_grid_cell_count = partition_stats.non_grid_cells;
                candidate.partition_face_index_entries = partition_stats.face_index_entries;
                candidate.partition_islands = partition_stats.islands;
                candidate.partition_largest_island = candidate.profile.grow_largest_island;
            }
            candidate.diagnostics = candidate.profile.diagnostics;
            return candidate;
        };
        int depth = initial_leaf_depth;
        int sweep_count = 0;
        std::vector<int> checkpoint_depths;
        for (int checkpoint = initial_leaf_depth;
             checkpoint <= target_leaf_depth;
             checkpoint = next_fast_depth_checkpoint(checkpoint)) {
            checkpoint_depths.push_back(checkpoint);
            if (checkpoint >= target_leaf_depth) {
                break;
            }
        }
        const auto adaptive_sweep_start = Clock::now();
        LeafSweepConfig checkpoint_leaf_config = leaf_config;
        checkpoint_leaf_config.checkpoint_depths = checkpoint_depths;
        checkpoint_leaf_config.checkpoint_callback = [&](const LeafSweepResult& checkpoint_leaf,
                                                         int checkpoint_depth) {
            depth = checkpoint_depth;
            ++sweep_count;
            accumulated_leaf_sweep_ms =
                std::chrono::duration<double, std::milli>(Clock::now() - adaptive_sweep_start).count();
            AdaptiveLeafSweepResult candidate =
                materialize_fast_checkpoint_candidate(checkpoint_leaf, depth, sweep_count);
            auto snapshot = snapshot_from_fast_candidate(candidate, depth);
            if (snapshot.readiness_met) {
                snapshot.stop_reason = "coverage_ready";
            } else if (depth >= target_leaf_depth) {
                snapshot.stop_reason = "max_depth";
            } else {
                snapshot.stop_reason = "checkpoint";
            }
            depth_snapshots.push_back(snapshot);

            selected = std::move(candidate);
            selected.selected_leaf_depth = depth;
            have_selected = true;
            return snapshot.readiness_met || depth >= target_leaf_depth;
        };
        out.leaf_sweep = build_leaf_sweep(obstacles,
                                          adaptive_config.shallow_start_depth,
                                          target_leaf_depth,
                                          checkpoint_leaf_config);
        if (!have_selected) {
            ++sweep_count;
            accumulated_leaf_sweep_ms =
                std::chrono::duration<double, std::milli>(Clock::now() - adaptive_sweep_start).count();
            selected = materialize_fast_checkpoint_candidate(out.leaf_sweep,
                                                             target_leaf_depth,
                                                             sweep_count);
            auto snapshot = snapshot_from_fast_candidate(selected, target_leaf_depth);
            snapshot.stop_reason = snapshot.readiness_met ? "coverage_ready" : "max_depth";
            depth_snapshots.push_back(snapshot);
            selected.selected_leaf_depth = target_leaf_depth;
            selected.adaptive_depth_readiness_met = snapshot.readiness_met;
            selected.adaptive_depth_stop_reason = snapshot.stop_reason;
            have_selected = true;
        }
        if (have_selected) {
            for (const auto& [key, value] : out.leaf_sweep.diagnostics) {
                if (key.find("worker_oracle.") != std::string::npos ||
                    key.find("external") != std::string::npos ||
                    key.find("canonical_frame") != std::string::npos) {
                    set_diagnostic_max(selected.profile.diagnostics, key, value);
                }
            }
            const auto& final_snapshot = depth_snapshots.back();
            selected.selected_leaf_depth = final_snapshot.depth;
            selected.adaptive_depth_readiness_met = final_snapshot.readiness_met;
            selected.adaptive_depth_stop_reason = final_snapshot.stop_reason;
            selected.adaptive_depth_snapshots_json =
                adaptive_depth_snapshots_to_json(depth_snapshots);
            selected.seed_probe_box_covered = final_snapshot.covered_count;
            selected.seed_probe_main_accessible =
                final_snapshot.main_accessible_count + final_snapshot.anchor_to_main_count;
            selected.p_box_covered = final_snapshot.p_box_covered;
            selected.p_main_accessible =
                static_cast<double>(selected.seed_probe_main_accessible) /
                static_cast<double>(std::max(1, final_snapshot.free_probe_count));
            selected.p_anchor_to_main_uncovered = final_snapshot.p_anchor_to_main_uncovered;
            selected.total_ms =
                std::chrono::duration<double, std::milli>(Clock::now() - total_start).count();
            selected.profile.total_ms = selected.total_ms;
            selected.profile.grow_ms = selected.total_ms;
            selected.profile.diagnostics["adaptive.in_sweep_checkpoints"] =
                static_cast<double>(sweep_count);
            selected.profile.diagnostics["adaptive.fast_checkpoint_mode"] = 1.0;
            selected.profile.diagnostics["adaptive.fast_virtual_checkpoint_mode"] = 1.0;
            selected.profile.diagnostics["adaptive.terminal_controller_enabled"] = 0.0;
            selected.profile.diagnostics["adaptive.in_sweep_checkpoint_mode"] = 1.0;
            selected.profile.diagnostics["adaptive.selected_leaf_depth"] =
                static_cast<double>(selected.selected_leaf_depth);
            selected.profile.diagnostics["adaptive.depth_readiness_met"] =
                selected.adaptive_depth_readiness_met ? 1.0 : 0.0;
            selected.profile.diagnostics["adaptive.depth_enabled"] = 1.0;
            selected.profile.diagnostics["adaptive.depth_min"] = static_cast<double>(adaptive_depth_min);
            selected.profile.diagnostics["adaptive.depth_max"] = static_cast<double>(target_leaf_depth);
            record_depth_semantics_diagnostics(selected.profile.diagnostics,
                                               "adaptive.",
                                               adaptive_config.shallow_start_depth,
                                               initial_leaf_depth,
                                               target_leaf_depth,
                                               config_.grower.find_free_box,
                                               target_leaf_depth);
            if (oracle_) {
                const OracleCounters counters = oracle_->counters();
                normalize_external_evidence_diagnostics(selected.profile.diagnostics, &counters);
            } else {
                normalize_external_evidence_diagnostics(selected.profile.diagnostics);
            }
            record_portal_membership_policy(selected.profile.diagnostics, config_.portal_membership_policy);
            selected.diagnostics = selected.profile.diagnostics;
            last_build_ = selected.profile;
            if (config_.database.checkpoint_after_build && database_) {
                database_->checkpoint();
            }
            return selected;
        }
    }

    if (adaptive_config.node_budget <= 0 && !adaptive_depth_enabled) {
        out.diagnostics["adaptive.fixed_virtual_layer_mode"] = 1.0;
        out.diagnostics["adaptive.terminal_controller_enabled"] = 0.0;
        leaf_config.collision_overlap_prune_min_depth = adaptive_config.defer_min_depth;
        leaf_config.collision_overlap_prune_threshold = adaptive_config.overlap_depth_threshold;
        leaf_config.collision_overlap_prune_min_threshold = adaptive_config.overlap_depth_min_threshold;
        leaf_config.collision_overlap_prune_decay_per_depth = adaptive_config.overlap_depth_decay_per_depth;
        leaf_config.collision_overlap_prune_ratio_threshold = adaptive_config.overlap_ratio_threshold;
        out.leaf_sweep = build_leaf_sweep(obstacles,
                                          adaptive_config.shallow_start_depth,
                                          target_leaf_depth,
                                          leaf_config);
        out.selected_leaf_depth = target_leaf_depth;
        out.adaptive_depth_readiness_met = false;
        out.adaptive_depth_stop_reason = "fixed_depth";
        out.leaf_sweep_ms = out.leaf_sweep.total_ms;
        out.shallow_free_count = static_cast<int>(out.leaf_sweep.free_boxes.size());
        out.shallow_collision_count = static_cast<int>(out.leaf_sweep.collision_boxes.size());
        out.adaptive_deferred = static_cast<int>(out.leaf_sweep.collision_boxes.size());
        out.unresolved_domains = static_cast<int>(out.leaf_sweep.collision_boxes.size());

        const double adjacency_tolerance = config_.query.adjacency_tolerance;
        const auto merge_start = Clock::now();
        BudgetedMergeStats merge_stats;
        if (config_.enable_merger && !boxes_.empty()) {
            bool merged_by_partition = false;
            if (adaptive_config.planning_backend == "partition_native") {
                rebuild_adaptive_partition(partition_config, nullptr);
                if (adaptive_partition_query_enabled_ && adaptive_partition_) {
                    AdaptiveGridPartitionMergeOptions options;
                    options.max_ms = adaptive_config.max_merge_ms;
                    options.max_rounds = adaptive_config.max_merge_rounds;
                    options.grid_line_merge = true;
                    options.containment_prune = false;
                    const auto partition_merge =
                        adaptive_partition_->merge_boxes(boxes_, options, adjacency_tolerance);
                    for (int released_id : partition_merge.released_box_ids) {
                        oracle_->release_box(released_id);
                    }
                    merge_stats.input_boxes = partition_merge.input_boxes;
                    merge_stats.output_boxes = partition_merge.output_boxes;
                    merge_stats.grid_merges = partition_merge.grid_merges;
                    merge_stats.grid_rounds = partition_merge.rounds;
                    merge_stats.containment_pruned = partition_merge.containment_pruned;
                    merge_stats.stop_reason = partition_merge.stop_reason == 0 ? 1 : partition_merge.stop_reason;
                    merge_stats.total_ms = partition_merge.total_ms;
                    merge_stats.grid_ms = partition_merge.total_ms;
                    out.diagnostics["adaptive.partition_merge_enabled"] = 1.0;
                    out.diagnostics["adaptive.partition_merge_released_boxes"] =
                        static_cast<double>(partition_merge.released_box_ids.size());
                    out.diagnostics["adaptive.partition_merge_containment_skipped"] =
                        static_cast<double>(partition_merge.containment_skipped);
                    out.diagnostics["adaptive.partition_merge_containment_bucket_entries"] =
                        static_cast<double>(partition_merge.containment_bucket_entries);
                    out.diagnostics["adaptive.partition_merge_containment_candidates"] =
                        static_cast<double>(partition_merge.containment_candidates);
                    out.diagnostics["adaptive.partition_merge_containment_tests"] =
                        static_cast<double>(partition_merge.containment_tests);
                    out.diagnostics["adaptive.partition_merge_containment_overflow"] =
                        static_cast<double>(partition_merge.containment_overflow);
                    out.diagnostics["adaptive.partition_merge_containment_ms"] =
                        partition_merge.containment_ms;
                    out.diagnostics["adaptive.partition_merge_line_ms"] =
                        partition_merge.line_merge_ms;
                    merged_by_partition = true;
                }
            }
            if (!merged_by_partition) {
                MergerConfig leaf_merge_config = config_.merger;
                leaf_merge_config.containment_prune = true;
                merge_stats = budgeted_leaf_merge(*oracle_,
                                                  boxes_,
                                                  leaf_merge_config,
                                                  adaptive_config.max_merge_ms,
                                                  adaptive_config.max_merge_rounds,
                                                  adaptive_config.max_merge_input_boxes,
                                                  adjacency_tolerance);
            }
            raw_boxes_ = boxes_;
        } else {
            merge_stats.input_boxes = static_cast<int>(boxes_.size());
            merge_stats.output_boxes = static_cast<int>(boxes_.size());
            merge_stats.stop_reason = 0;
        }
        const double merge_ms =
            std::chrono::duration<double, std::milli>(Clock::now() - merge_start).count();
        const bool use_partition_backend = adaptive_config.planning_backend == "partition_native";
        AdjacencyBuildStats adjacency_stats;
        std::unordered_set<int> main_ids;
        int partition_island_count_for_profile = 0;
        int partition_largest_island_for_profile = 0;
        if (use_partition_backend) {
            if (!adaptive_partition_query_enabled_ || !adaptive_partition_) {
                rebuild_adaptive_partition(partition_config, nullptr);
            }
            if (adaptive_partition_query_enabled_ && adaptive_partition_) {
                const auto largest = adaptive_partition_->largest_component_box_ids_with_overlay();
                main_ids.insert(largest.begin(), largest.end());
                const auto& partition_stats = adaptive_partition_->stats();
                partition_island_count_for_profile = partition_stats.islands;
                partition_largest_island_for_profile = partition_stats.largest_island;
                out.diagnostics["adaptive.partition_skipped_graph_adjacency"] = 1.0;
            }
        }
        if (!use_partition_backend) {
            rebuild_adjacency();
            adjacency_stats = last_adjacency_build_stats();
            main_ids = adaptive_largest_island_ids(adjacency_);
        } else if (main_ids.empty()) {
            out.diagnostics["adaptive.partition_missing_no_graph_fallback"] = 1.0;
        }

        const auto coverage_start = Clock::now();
        const auto planning_domain = oracle_ ? oracle_->planning_intervals() : std::vector<Interval>{};
        int probe_attempted = 0;
        std::vector<Eigen::VectorXd> free_probes =
            oracle_ ? adaptive_generate_free_probes(*oracle_,
                                                    planning_domain,
                                                    adaptive_config.seed_probe_count,
                                                    adaptive_config.seed_probe_rng_seed,
                                                    probe_attempted)
                    : std::vector<Eigen::VectorXd>{};
        out.seed_probe_count = probe_attempted;
        out.seed_probe_free_count = static_cast<int>(free_probes.size());
        int uncovered_anchor_attempts = 0;
        StageContext probe_context = StageContext::from_runtime(config_.runtime);
        FindFreeBoxOptions probe_options = config_.grower.find_free_box;
        probe_options.max_depth = target_leaf_depth;
        probe_options.reject_seed_collision = false;
        probe_options.deadline_ms = 5.0;
        const int anchor_cap = std::max(0, adaptive_config.seed_anchor_probe_cap);
	        BoxSpatialIndex coverage_index;
	        const bool use_partition_coverage =
	            use_partition_backend && adaptive_partition_query_enabled_ && adaptive_partition_;
	        if (!use_partition_coverage) {
	            coverage_index.rebuild(boxes_, adjacency_tolerance);
	        }
	        for (const auto& point : free_probes) {
	            const int owner = use_partition_coverage
	                ? adaptive_partition_->locate_containing_box(point, false, adjacency_tolerance)
	                : [&]() {
	                      const int owner_index = coverage_index.covering_box(boxes_, point, adjacency_tolerance);
	                      return owner_index >= 0 ? boxes_[static_cast<std::size_t>(owner_index)].id : -1;
	                  }();
	            if (owner >= 0) {
	                out.seed_probe_box_covered += 1;
	                if (main_ids.find(owner) != main_ids.end()) {
	                    out.seed_probe_main_accessible += 1;
                }
                continue;
            }
            if (uncovered_anchor_attempts >= anchor_cap || planning_domain.empty()) {
                continue;
            }
            ++uncovered_anchor_attempts;
            const auto ffb = find_free_box_in_domain(point, planning_domain, probe_context, probe_options);
            if (!ffb.found) {
                continue;
            }
            out.seed_probe_anchor_success += 1;
	            const BoxNode anchor = adaptive_make_box_from_intervals(ffb.intervals,
	                                                                    ffb.node,
	                                                                    -1,
	                                                                    ffb.validation_detail.safety_status,
	                                                                    ffb.validation_detail.strict_audit_required);
            const bool anchor_main_accessible = use_partition_coverage
                ? adaptive_partition_->box_adjacent_to_any(anchor, main_ids, adjacency_tolerance)
                : (!use_partition_backend &&
                   adaptive_has_adjacency_to_any(boxes_, anchor, &main_ids, adjacency_tolerance));
	            if (anchor_main_accessible) {
	                out.seed_probe_main_accessible += 1;
	            }
	        }
        out.coverage_probe_ms =
            std::chrono::duration<double, std::milli>(Clock::now() - coverage_start).count();
        const double free_den = static_cast<double>(std::max(1, out.seed_probe_free_count));
        out.p_box_covered = static_cast<double>(out.seed_probe_box_covered) / free_den;
        out.p_anchor_success = static_cast<double>(out.seed_probe_anchor_success) / free_den;
        out.p_main_accessible = static_cast<double>(out.seed_probe_main_accessible) / free_den;

        out.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - total_start).count();
        out.profile = {};
        out.profile.raw_boxes = static_cast<int>(raw_boxes_.size());
        out.profile.final_boxes = static_cast<int>(boxes_.size());
        out.profile.segment_edges = static_cast<int>(segment_edges_.size());
        out.profile.grow_ms = out.leaf_sweep_ms;
        out.profile.total_ms = out.total_ms;
        if (use_partition_backend) {
            out.profile.grow_adjacency_islands = partition_island_count_for_profile;
            out.profile.adjacency_islands = partition_island_count_for_profile;
            out.profile.grow_largest_island = partition_largest_island_for_profile;
        } else {
            const auto graph_islands = find_islands(adjacency_);
            out.profile.grow_adjacency_islands = static_cast<int>(graph_islands.size());
            out.profile.adjacency_islands = out.profile.grow_adjacency_islands;
            for (const auto& island : graph_islands) {
                out.profile.grow_largest_island =
                    std::max(out.profile.grow_largest_island, static_cast<int>(island.size()));
            }
        }
        out.profile.diagnostics = out.leaf_sweep.diagnostics;
        out.profile.diagnostics["adaptive.fast_leaf_sweep"] = 1.0;
        out.profile.diagnostics["adaptive.offline_query_agnostic_build"] = 1.0;
        out.profile.diagnostics["adaptive.qroot_pairs_total"] = 0.0;
        out.profile.diagnostics["adaptive.qroot_uncovered_endpoints"] = 0.0;
        out.profile.diagnostics["adaptive.leaf_sweep_ms"] = out.leaf_sweep_ms;
        record_depth_semantics_diagnostics(out.profile.diagnostics,
                                           "adaptive.",
                                           adaptive_config.shallow_start_depth,
                                           initial_leaf_depth,
                                           target_leaf_depth,
                                           config_.grower.find_free_box,
                                           target_leaf_depth);
        out.profile.diagnostics["adaptive.overlap_depth_threshold"] = adaptive_config.overlap_depth_threshold;
        out.profile.diagnostics["adaptive.overlap_depth_min_threshold"] = adaptive_config.overlap_depth_min_threshold;
        out.profile.diagnostics["adaptive.overlap_depth_decay_per_depth"] = adaptive_config.overlap_depth_decay_per_depth;
        out.profile.diagnostics["adaptive.merge_ms"] = merge_ms;
        out.profile.diagnostics["adaptive.merge_input_boxes"] = static_cast<double>(merge_stats.input_boxes);
        out.profile.diagnostics["adaptive.merge_output_boxes"] = static_cast<double>(merge_stats.output_boxes);
        out.profile.diagnostics["adaptive.merge_grid_ms"] = merge_stats.grid_ms;
        out.profile.diagnostics["adaptive.merge_grid_merges"] = static_cast<double>(merge_stats.grid_merges);
        out.profile.diagnostics["adaptive.merge_grid_rounds"] = static_cast<double>(merge_stats.grid_rounds);
        out.profile.diagnostics["adaptive.merge_tree_ms"] = merge_stats.tree_ms;
        out.profile.diagnostics["adaptive.merge_tree_merges"] = static_cast<double>(merge_stats.tree_merges);
        out.profile.diagnostics["adaptive.merge_tree_rounds"] = static_cast<double>(merge_stats.tree_rounds);
        out.profile.diagnostics["adaptive.merge_containment_ms"] = merge_stats.containment_ms;
        out.profile.diagnostics["adaptive.merge_exact_ms"] = merge_stats.exact_ms;
        out.profile.diagnostics["adaptive.merge_containment_pruned"] = static_cast<double>(merge_stats.containment_pruned);
        out.profile.diagnostics["adaptive.merge_exact_merges"] = static_cast<double>(merge_stats.exact_merges);
        out.profile.diagnostics["adaptive.merge_rounds"] = static_cast<double>(merge_stats.rounds);
        out.profile.diagnostics["adaptive.merge_stop_reason"] = static_cast<double>(merge_stats.stop_reason);
        out.profile.diagnostics["adaptive.partition_merge_enabled"] =
            out.diagnostics.find("adaptive.partition_merge_enabled") != out.diagnostics.end() ? 1.0 : 0.0;
        out.profile.diagnostics["adaptive.partition_merge_released_boxes"] =
            out.diagnostics.find("adaptive.partition_merge_released_boxes") != out.diagnostics.end()
                ? out.diagnostics["adaptive.partition_merge_released_boxes"]
                : 0.0;
        out.profile.diagnostics["adaptive.partition_merge_containment_skipped"] =
            out.diagnostics.find("adaptive.partition_merge_containment_skipped") != out.diagnostics.end()
                ? out.diagnostics["adaptive.partition_merge_containment_skipped"]
                : 0.0;
        out.profile.diagnostics["adaptive.partition_merge_containment_bucket_entries"] =
            out.diagnostics.find("adaptive.partition_merge_containment_bucket_entries") != out.diagnostics.end()
                ? out.diagnostics["adaptive.partition_merge_containment_bucket_entries"]
                : 0.0;
        out.profile.diagnostics["adaptive.partition_merge_containment_candidates"] =
            out.diagnostics.find("adaptive.partition_merge_containment_candidates") != out.diagnostics.end()
                ? out.diagnostics["adaptive.partition_merge_containment_candidates"]
                : 0.0;
        out.profile.diagnostics["adaptive.partition_merge_containment_tests"] =
            out.diagnostics.find("adaptive.partition_merge_containment_tests") != out.diagnostics.end()
                ? out.diagnostics["adaptive.partition_merge_containment_tests"]
                : 0.0;
        out.profile.diagnostics["adaptive.partition_merge_containment_overflow"] =
            out.diagnostics.find("adaptive.partition_merge_containment_overflow") != out.diagnostics.end()
                ? out.diagnostics["adaptive.partition_merge_containment_overflow"]
                : 0.0;
        out.profile.diagnostics["adaptive.partition_merge_containment_ms"] =
            out.diagnostics.find("adaptive.partition_merge_containment_ms") != out.diagnostics.end()
                ? out.diagnostics["adaptive.partition_merge_containment_ms"]
                : 0.0;
        out.profile.diagnostics["adaptive.partition_merge_line_ms"] =
            out.diagnostics.find("adaptive.partition_merge_line_ms") != out.diagnostics.end()
                ? out.diagnostics["adaptive.partition_merge_line_ms"]
                : 0.0;
        out.profile.diagnostics["adaptive.partition_skipped_graph_adjacency"] =
            out.diagnostics.find("adaptive.partition_skipped_graph_adjacency") != out.diagnostics.end()
                ? out.diagnostics["adaptive.partition_skipped_graph_adjacency"]
                : 0.0;
        out.profile.diagnostics["adaptive.adjacency_ms"] = adjacency_stats.build_ms;
        out.profile.diagnostics["adaptive.adjacency_boxes"] = static_cast<double>(adjacency_stats.boxes);
        out.profile.diagnostics["adaptive.adjacency_selected_dims"] = static_cast<double>(adjacency_stats.selected_dims);
        out.profile.diagnostics["adaptive.adjacency_primary_dim"] = static_cast<double>(adjacency_stats.primary_dim);
        out.profile.diagnostics["adaptive.adjacency_candidates"] = static_cast<double>(adjacency_stats.candidate_pairs);
        out.profile.diagnostics["adaptive.adjacency_exact_tests"] = static_cast<double>(adjacency_stats.exact_tests);
        out.profile.diagnostics["adaptive.adjacency_edges"] = static_cast<double>(adjacency_stats.edges);
        out.profile.diagnostics["adaptive.adaptive_ms"] = 0.0;
	    out.profile.diagnostics["adaptive.coverage_probe_ms"] = out.coverage_probe_ms;
	    out.profile.diagnostics["adaptive.total_ms"] = out.total_ms;
    out.profile.diagnostics["adaptive.depth_enabled"] = adaptive_depth_enabled ? 1.0 : 0.0;
    out.profile.diagnostics["adaptive.depth_min"] = static_cast<double>(adaptive_depth_min);
    out.profile.diagnostics["adaptive.depth_max"] = static_cast<double>(target_leaf_depth);
    out.profile.diagnostics["adaptive.depth_min_covered_probes"] =
        static_cast<double>(adaptive_config.adaptive_depth_min_covered_probes);
    out.profile.diagnostics["adaptive.depth_min_main_probes"] =
        static_cast<double>(adaptive_config.adaptive_depth_min_main_probes);
    out.profile.diagnostics["adaptive.depth_min_main_ratio"] =
        adaptive_config.adaptive_depth_min_main_ratio;
    out.profile.diagnostics["adaptive.depth_min_cells"] =
        static_cast<double>(adaptive_config.adaptive_depth_min_cells);
    out.profile.diagnostics["adaptive.depth_min_main_cells"] =
        static_cast<double>(adaptive_config.adaptive_depth_min_main_cells);
    out.profile.diagnostics["adaptive.depth_max_online_cells"] =
        static_cast<double>(adaptive_config.adaptive_depth_max_online_cells);
	    out.profile.diagnostics["adaptive.shallow_free_count"] = static_cast<double>(out.shallow_free_count);
        out.profile.diagnostics["adaptive.shallow_collision_count"] = static_cast<double>(out.shallow_collision_count);
        out.profile.diagnostics["adaptive.deferred"] = static_cast<double>(out.adaptive_deferred);
        out.profile.diagnostics["adaptive.unresolved_domains"] = static_cast<double>(out.unresolved_domains);
        out.profile.diagnostics["adaptive.seed_probe_count"] = static_cast<double>(out.seed_probe_count);
        out.profile.diagnostics["adaptive.seed_probe_free_count"] = static_cast<double>(out.seed_probe_free_count);
        out.profile.diagnostics["adaptive.seed_probe_box_covered"] = static_cast<double>(out.seed_probe_box_covered);
        out.profile.diagnostics["adaptive.seed_probe_anchor_success"] = static_cast<double>(out.seed_probe_anchor_success);
        out.profile.diagnostics["adaptive.seed_probe_main_accessible"] = static_cast<double>(out.seed_probe_main_accessible);
        out.profile.diagnostics["adaptive.seed_anchor_probe_cap"] = static_cast<double>(anchor_cap);
        out.profile.diagnostics["adaptive.seed_anchor_probe_attempts"] = static_cast<double>(uncovered_anchor_attempts);
	        out.profile.diagnostics["adaptive.p_box_covered"] = out.p_box_covered;
	        out.profile.diagnostics["adaptive.p_anchor_success"] = out.p_anchor_success;
	        out.profile.diagnostics["adaptive.p_main_accessible"] = out.p_main_accessible;
	        out.profile.diagnostics["adaptive.p_anchor_to_main_uncovered"] = out.p_anchor_to_main_uncovered;
	        out.profile.diagnostics["adaptive.selected_leaf_depth"] =
	            static_cast<double>(out.selected_leaf_depth);
	        out.profile.diagnostics["adaptive.depth_readiness_met"] =
	            out.adaptive_depth_readiness_met ? 1.0 : 0.0;
	        rebuild_adaptive_partition(partition_config, &out.profile);
        if (adaptive_partition_ && !adaptive_partition_->empty()) {
            const auto& partition_stats = adaptive_partition_->stats();
            out.partition_cell_count = partition_stats.cells;
            out.partition_grid_cell_count = partition_stats.grid_cells;
            out.partition_non_grid_cell_count = partition_stats.non_grid_cells;
            out.partition_face_index_entries = partition_stats.face_index_entries;
            out.partition_islands = partition_stats.islands;
            out.partition_largest_island = partition_stats.largest_island;
        }
        if (oracle_) {
            const OracleCounters counters = oracle_->counters();
            normalize_external_evidence_diagnostics(out.profile.diagnostics, &counters);
        } else {
            normalize_external_evidence_diagnostics(out.profile.diagnostics);
        }
        record_portal_membership_policy(out.profile.diagnostics, config_.portal_membership_policy);
        out.diagnostics = out.profile.diagnostics;
        last_build_ = out.profile;
        if (config_.database.checkpoint_after_build && database_) {
            database_->checkpoint();
        }
        return out;
    }

    out.leaf_sweep = build_leaf_sweep(obstacles,
                                      adaptive_config.shallow_start_depth,
                                      initial_leaf_depth,
                                      leaf_config);
    out.leaf_sweep_ms = out.leaf_sweep.total_ms;
    out.shallow_free_count = static_cast<int>(out.leaf_sweep.free_boxes.size());
    out.shallow_collision_count = static_cast<int>(out.leaf_sweep.collision_boxes.size());

    const double adjacency_tolerance = config_.query.adjacency_tolerance;
    const auto merge_start = Clock::now();
    BudgetedMergeStats merge_stats;
    if (config_.enable_merger && !boxes_.empty()) {
        bool merged_by_partition = false;
        if (adaptive_config.planning_backend == "partition_native") {
            rebuild_adaptive_partition(partition_config, nullptr);
            if (adaptive_partition_query_enabled_ && adaptive_partition_) {
                AdaptiveGridPartitionMergeOptions options;
                options.max_ms = adaptive_config.max_merge_ms;
                options.max_rounds = adaptive_config.max_merge_rounds;
                options.grid_line_merge = true;
                options.containment_prune = false;
                const auto partition_merge =
                    adaptive_partition_->merge_boxes(boxes_, options, adjacency_tolerance);
                for (int released_id : partition_merge.released_box_ids) {
                    oracle_->release_box(released_id);
                }
                merge_stats.input_boxes = partition_merge.input_boxes;
                merge_stats.output_boxes = partition_merge.output_boxes;
                merge_stats.grid_merges = partition_merge.grid_merges;
                merge_stats.grid_rounds = partition_merge.rounds;
                merge_stats.containment_pruned = partition_merge.containment_pruned;
                merge_stats.stop_reason = partition_merge.stop_reason == 0 ? 1 : partition_merge.stop_reason;
                merge_stats.total_ms = partition_merge.total_ms;
                merge_stats.grid_ms = partition_merge.total_ms;
                out.diagnostics["adaptive.partition_merge_enabled"] = 1.0;
                out.diagnostics["adaptive.partition_merge_released_boxes"] =
                    static_cast<double>(partition_merge.released_box_ids.size());
                out.diagnostics["adaptive.partition_merge_containment_skipped"] =
                    static_cast<double>(partition_merge.containment_skipped);
                out.diagnostics["adaptive.partition_merge_containment_bucket_entries"] =
                    static_cast<double>(partition_merge.containment_bucket_entries);
                out.diagnostics["adaptive.partition_merge_containment_candidates"] =
                    static_cast<double>(partition_merge.containment_candidates);
                out.diagnostics["adaptive.partition_merge_containment_tests"] =
                    static_cast<double>(partition_merge.containment_tests);
                out.diagnostics["adaptive.partition_merge_containment_overflow"] =
                    static_cast<double>(partition_merge.containment_overflow);
                out.diagnostics["adaptive.partition_merge_containment_ms"] =
                    partition_merge.containment_ms;
                out.diagnostics["adaptive.partition_merge_line_ms"] =
                    partition_merge.line_merge_ms;
                merged_by_partition = true;
            }
        }
        if (!merged_by_partition) {
            MergerConfig leaf_merge_config = config_.merger;
            leaf_merge_config.containment_prune = true;
            merge_stats = budgeted_leaf_merge(*oracle_,
                                              boxes_,
                                              leaf_merge_config,
                                              adaptive_config.max_merge_ms,
                                              adaptive_config.max_merge_rounds,
                                              adaptive_config.max_merge_input_boxes,
                                              adjacency_tolerance);
        }
        raw_boxes_ = boxes_;
    } else {
        merge_stats.input_boxes = static_cast<int>(boxes_.size());
        merge_stats.output_boxes = static_cast<int>(boxes_.size());
        merge_stats.stop_reason = 0;
    }
	    const double merge_ms = std::chrono::duration<double, std::milli>(Clock::now() - merge_start).count();
	    const bool use_partition_backend = adaptive_config.planning_backend == "partition_native";
	    AdjacencyBuildStats initial_adjacency_stats;
	    std::unordered_set<int> main_ids;
	    auto refresh_main_from_partition = [&]() -> bool {
	        if (!adaptive_partition_query_enabled_ || !adaptive_partition_) {
	            return false;
	        }
	        const auto largest = adaptive_partition_->largest_component_box_ids_with_overlay();
	        main_ids.clear();
	        main_ids.insert(largest.begin(), largest.end());
	        return !main_ids.empty();
	    };
	    if (use_partition_backend) {
	        if (!adaptive_partition_query_enabled_ || !adaptive_partition_) {
	            rebuild_adaptive_partition(partition_config, nullptr);
	        }
	        if (refresh_main_from_partition()) {
	            out.diagnostics["adaptive.partition_skipped_graph_adjacency"] = 1.0;
	        }
	    }
	    if (!use_partition_backend) {
	        rebuild_adjacency();
	        initial_adjacency_stats = last_adjacency_build_stats();
	        main_ids = adaptive_largest_island_ids(adjacency_);
	    } else if (main_ids.empty()) {
	        out.diagnostics["adaptive.partition_missing_no_graph_fallback"] = 1.0;
	    }
    std::vector<BoxNode> scoring_boxes = boxes_;
    std::vector<AdaptiveFrontierItem> deferred;
    deferred.reserve(out.leaf_sweep.collision_boxes.size());
    const auto planning_domain = oracle_ ? oracle_->planning_intervals() : std::vector<Interval>{};
    int probe_attempted = 0;
    const int requested_probe_count = adaptive_depth_enabled
        ? std::max(0, adaptive_config.adaptive_depth_probe_count)
        : std::max(0, adaptive_config.seed_probe_count);
    const int requested_probe_seed = adaptive_depth_enabled
        ? adaptive_config.adaptive_depth_probe_seed
        : adaptive_config.seed_probe_rng_seed;
    const auto probe_seed_start = Clock::now();
    std::vector<Eigen::VectorXd> free_probes =
        oracle_ ? adaptive_generate_free_probes(*oracle_,
                                                planning_domain,
                                                requested_probe_count,
                                                requested_probe_seed,
                                                probe_attempted)
                : std::vector<Eigen::VectorXd>{};
    if (adaptive_depth_enabled && oracle_ &&
        adaptive_config.adaptive_depth_min_free_probes > 0 &&
        static_cast<int>(free_probes.size()) < adaptive_config.adaptive_depth_min_free_probes) {
        const int supplement_limit = std::max(
            requested_probe_count,
            std::min(8192, std::max(requested_probe_count * 4,
                                    adaptive_config.adaptive_depth_min_free_probes * 64)));
        int supplement_seed_offset = 1;
        while (probe_attempted < supplement_limit &&
               static_cast<int>(free_probes.size()) < adaptive_config.adaptive_depth_min_free_probes) {
            const int batch = std::min(std::max(128, requested_probe_count), supplement_limit - probe_attempted);
            int extra_attempted = 0;
            auto extra = adaptive_generate_free_probes(*oracle_,
                                                       planning_domain,
                                                       batch,
                                                       requested_probe_seed + supplement_seed_offset,
                                                       extra_attempted);
            probe_attempted += extra_attempted;
            free_probes.insert(free_probes.end(),
                               std::make_move_iterator(extra.begin()),
                               std::make_move_iterator(extra.end()));
            ++supplement_seed_offset;
            if (batch <= 0 || extra_attempted <= 0) {
                break;
            }
        }
    }
    const double initial_probe_ms =
        std::chrono::duration<double, std::milli>(Clock::now() - probe_seed_start).count();
    out.seed_probe_count = probe_attempted;
    out.seed_probe_free_count = static_cast<int>(free_probes.size());

    auto item_less = [](const AdaptiveFrontierItem& lhs, const AdaptiveFrontierItem& rhs) {
        return lhs.score < rhs.score;
    };
    std::priority_queue<AdaptiveFrontierItem,
                        std::vector<AdaptiveFrontierItem>,
                        decltype(item_less)>
        frontier(item_less);

    auto refresh_score = [&](AdaptiveFrontierItem& item) {
        item.score = adaptive_frontier_score(scoring_boxes,
                                             item,
                                             main_ids,
                                             adaptive_config.overlap_depth_threshold,
                                             adjacency_tolerance);
    };
    auto push_frontier = [&](AdaptiveFrontierItem item) {
        item.free_seed_hits = adaptive_count_seed_hits(item, free_probes);
        if (item.free_seed_hits > 0) {
            out.diagnostics["adaptive.frontier_seed_hit_pushes"] += 1.0;
            out.diagnostics["adaptive.frontier_seed_hits_total"] += static_cast<double>(item.free_seed_hits);
        }
        refresh_score(item);
        frontier.push(std::move(item));
    };

    for (const auto& collision_box : out.leaf_sweep.collision_boxes) {
        AdaptiveFrontierItem item;
        item.node = collision_box.tree_id >= 0 ? collision_box.tree_id : collision_box.id;
        item.intervals = collision_box.joint_intervals;
        item.changed_dim = -1;
        if (!planning_domain.empty() && !intervals_overlap_local(item.intervals, planning_domain, 0.0)) {
            out.diagnostics["adaptive.initial_frontier_outside_domain"] += 1.0;
            continue;
        }
        push_frontier(std::move(item));
    }

    const auto adaptive_start = Clock::now();
    auto elapsed_total_ms = [&]() {
        return std::chrono::duration<double, std::milli>(Clock::now() - total_start).count();
    };
    auto budget_exhausted = [&]() {
        if (adaptive_config.time_budget_ms > 0.0 &&
            elapsed_total_ms() >= adaptive_config.time_budget_ms) {
            return true;
        }
        return adaptive_config.node_budget > 0 &&
               out.adaptive_validated >= adaptive_config.node_budget;
    };
    auto promote_deferred = [&]() {
        if (!adaptive_config.seed_promote_uncovered || deferred.empty() || free_probes.empty()) {
            return;
        }
        std::vector<AdaptiveFrontierItem> keep;
        keep.reserve(deferred.size());
        for (auto& item : deferred) {
            const int hits = adaptive_count_seed_hits(item, free_probes);
            if (hits > 0) {
                item.free_seed_hits = hits;
                refresh_score(item);
                frontier.push(std::move(item));
                out.adaptive_promoted += 1;
                out.diagnostics["adaptive.promoted_by_seed_probe"] += 1.0;
            } else {
                keep.push_back(std::move(item));
            }
        }
        deferred = std::move(keep);
    };

    const auto& split_descriptor = oracle_->database().split_policy_descriptor();
    std::size_t first_unconnected_new_index = boxes_.size();
    int pending_adjacency_boxes = 0;
    constexpr int kAdaptiveAdjacencyBatchSize = 512;
    std::vector<AdaptiveDepthSnapshot> depth_snapshots;
    double checkpoint_probe_ms_total = 0.0;
    auto evaluate_depth_snapshot = [&](int depth, bool allow_anchor_probe) {
        const auto snapshot_start = Clock::now();
        if (use_partition_backend && adaptive_partition_query_enabled_ && adaptive_partition_) {
            refresh_main_from_partition();
        } else if (!use_partition_backend && pending_adjacency_boxes > 0) {
            connect_incremental_boxes(adjacency_,
                                      boxes_,
                                      first_unconnected_new_index,
                                      adjacency_tolerance);
            first_unconnected_new_index = boxes_.size();
            pending_adjacency_boxes = 0;
            main_ids = adaptive_largest_island_ids(adjacency_);
        }
        AdaptiveDepthSnapshot snapshot;
        snapshot.depth = depth;
        snapshot.free_probe_count = static_cast<int>(free_probes.size());
        snapshot.collision_count = static_cast<int>(out.leaf_sweep.collision_boxes.size());
        if (use_partition_backend && adaptive_partition_query_enabled_ && adaptive_partition_) {
            const auto& stats = adaptive_partition_->stats();
            snapshot.cell_count = stats.cells;
            snapshot.island_count = stats.islands;
            snapshot.main_island_cell_count = static_cast<int>(main_ids.size());
        } else {
            snapshot.cell_count = static_cast<int>(boxes_.size());
            snapshot.island_count = static_cast<int>(find_islands(adjacency_).size());
            snapshot.main_island_cell_count = static_cast<int>(main_ids.size());
        }
        BoxSpatialIndex coverage_index;
        const bool use_partition_coverage =
            use_partition_backend && adaptive_partition_query_enabled_ && adaptive_partition_;
        if (!use_partition_coverage) {
            coverage_index.rebuild(boxes_, adjacency_tolerance);
        }
        std::vector<const Eigen::VectorXd*> uncovered;
        uncovered.reserve(free_probes.size());
        for (const auto& point : free_probes) {
            const int owner = use_partition_coverage
                ? adaptive_partition_->locate_containing_box(point, false, adjacency_tolerance)
                : [&]() {
                      const int owner_index = coverage_index.covering_box(boxes_, point, adjacency_tolerance);
                      return owner_index >= 0 ? boxes_[static_cast<std::size_t>(owner_index)].id : -1;
                  }();
            if (owner >= 0) {
                snapshot.covered_count += 1;
                if (main_ids.find(owner) != main_ids.end()) {
                    snapshot.main_accessible_count += 1;
                }
            } else {
                uncovered.push_back(&point);
            }
        }
        const int anchor_cap = allow_anchor_probe
            ? std::max(0, adaptive_depth_enabled
                           ? adaptive_config.adaptive_depth_anchor_probe_cap
                           : adaptive_config.seed_anchor_probe_cap)
            : 0;
        if (anchor_cap > 0 && !planning_domain.empty() && !uncovered.empty()) {
            StageContext probe_context = StageContext::from_runtime(config_.runtime);
            FindFreeBoxOptions probe_options = config_.grower.find_free_box;
            probe_options.max_depth = target_leaf_depth;
            probe_options.reject_seed_collision = false;
            probe_options.deadline_ms = adaptive_depth_enabled ? 3.0 : 5.0;
            const double max_probe_ms = adaptive_depth_enabled
                ? std::max(0.0, adaptive_config.adaptive_depth_max_probe_ms)
                : 0.0;
            for (const Eigen::VectorXd* point : uncovered) {
                if (snapshot.anchor_probe_attempts >= anchor_cap) {
                    break;
                }
                const double elapsed_ms =
                    std::chrono::duration<double, std::milli>(Clock::now() - snapshot_start).count();
                if (max_probe_ms > 0.0 && elapsed_ms >= max_probe_ms) {
                    break;
                }
                ++snapshot.anchor_probe_attempts;
                const auto ffb = find_free_box_in_domain(*point, planning_domain, probe_context, probe_options);
                if (!ffb.found) {
                    continue;
                }
                snapshot.anchor_success_count += 1;
                const BoxNode anchor = adaptive_make_box_from_intervals(
                    ffb.intervals,
                    ffb.node,
                    -1,
                    ffb.validation_detail.safety_status,
                    ffb.validation_detail.strict_audit_required);
                const bool anchor_main_accessible = use_partition_coverage
                    ? adaptive_partition_->box_adjacent_to_any(anchor, main_ids, adjacency_tolerance)
                    : (!use_partition_backend &&
                       adaptive_has_adjacency_to_any(boxes_, anchor, &main_ids, adjacency_tolerance));
                if (anchor_main_accessible) {
                    snapshot.anchor_to_main_count += 1;
                }
            }
        }
        const double free_den = static_cast<double>(std::max(1, snapshot.free_probe_count));
        snapshot.p_box_covered = static_cast<double>(snapshot.covered_count) / free_den;
        snapshot.p_main_accessible = static_cast<double>(snapshot.main_accessible_count) / free_den;
        snapshot.main_connected_ratio =
            static_cast<double>(snapshot.main_accessible_count) /
            static_cast<double>(std::max(1, snapshot.covered_count));
        snapshot.p_anchor_to_main_uncovered =
            static_cast<double>(snapshot.anchor_to_main_count) /
            static_cast<double>(std::max(1, snapshot.anchor_probe_attempts));
        snapshot.probe_ms =
            std::chrono::duration<double, std::milli>(Clock::now() - snapshot_start).count();
        const int min_covered_probes = std::max(0, adaptive_config.adaptive_depth_min_covered_probes);
        const int min_main_probes = std::max(0, adaptive_config.adaptive_depth_min_main_probes);
        const int min_cells = std::max(0, adaptive_config.adaptive_depth_min_cells);
        const int min_main_cells = std::max(0, adaptive_config.adaptive_depth_min_main_cells);
        const bool probe_gate =
            snapshot.covered_count >= min_covered_probes &&
            snapshot.main_accessible_count >= min_main_probes &&
            (min_covered_probes <= 0 ||
             snapshot.main_connected_ratio >= adaptive_config.adaptive_depth_min_main_ratio);
        const bool cell_gate =
            snapshot.cell_count >= min_cells &&
            snapshot.main_island_cell_count >= min_main_cells;
        snapshot.readiness_met =
            adaptive_depth_enabled &&
            probe_gate &&
            cell_gate &&
            (adaptive_config.adaptive_depth_max_online_cells <= 0 ||
             snapshot.cell_count <= adaptive_config.adaptive_depth_max_online_cells);
        return snapshot;
    };
    auto apply_final_depth_snapshot = [&](const AdaptiveDepthSnapshot& snapshot) {
        out.selected_leaf_depth = snapshot.depth;
        out.adaptive_depth_readiness_met = snapshot.readiness_met;
        out.adaptive_depth_stop_reason = snapshot.stop_reason;
        out.seed_probe_box_covered = snapshot.covered_count;
        out.seed_probe_anchor_success = snapshot.anchor_success_count;
        out.seed_probe_main_accessible = snapshot.main_accessible_count + snapshot.anchor_to_main_count;
        out.p_box_covered = snapshot.p_box_covered;
        const double free_den = static_cast<double>(std::max(1, snapshot.free_probe_count));
        out.p_anchor_success = static_cast<double>(snapshot.anchor_success_count) / free_den;
        out.p_main_accessible = static_cast<double>(out.seed_probe_main_accessible) / free_den;
        out.p_anchor_to_main_uncovered = snapshot.p_anchor_to_main_uncovered;
    };
    auto record_depth_snapshot = [&](AdaptiveDepthSnapshot snapshot) {
        checkpoint_probe_ms_total += snapshot.probe_ms;
        depth_snapshots.push_back(std::move(snapshot));
    };
    auto next_depth_checkpoint = [&](int depth) {
        const int step = depth < 16 ? 1 : 2;
        return std::min(target_leaf_depth, depth + step);
    };
    bool adaptive_depth_stop = false;
    int next_checkpoint_depth = initial_leaf_depth;
    if (adaptive_depth_enabled) {
        auto initial_snapshot = evaluate_depth_snapshot(initial_leaf_depth, true);
        if (initial_snapshot.readiness_met) {
            initial_snapshot.stop_reason = "coverage_ready";
            adaptive_depth_stop = true;
        } else if (initial_leaf_depth >= target_leaf_depth) {
            initial_snapshot.stop_reason = "max_depth";
            adaptive_depth_stop = true;
        } else {
            initial_snapshot.stop_reason = "checkpoint";
            next_checkpoint_depth = next_depth_checkpoint(initial_leaf_depth);
        }
        record_depth_snapshot(std::move(initial_snapshot));
    }
    std::vector<AdaptiveFrontierItem> checkpoint_hold;
    auto restore_checkpoint_hold = [&]() {
        for (auto& held : checkpoint_hold) {
            frontier.push(std::move(held));
        }
        checkpoint_hold.clear();
    };
    {
        const bool collect_overlap_ratio =
            adaptive_config.overlap_ratio_threshold > 0.0 &&
            adaptive_config.defer_min_depth >= 0;
        ScopedAdaptiveFullOverlapStats overlap_stats(*oracle_, collect_overlap_ratio);
        while (!frontier.empty() && !budget_exhausted() && !adaptive_depth_stop) {
            AdaptiveFrontierItem item = frontier.top();
            frontier.pop();
            if (item.intervals.empty()) {
                out.diagnostics["adaptive.empty_frontier_items"] += 1.0;
                continue;
            }
            const int depth = adaptive_virtual_depth(item.node);
            if (adaptive_depth_enabled && depth > next_checkpoint_depth) {
                checkpoint_hold.push_back(std::move(item));
                if (frontier.empty()) {
                    restore_checkpoint_hold();
                    auto snapshot = evaluate_depth_snapshot(next_checkpoint_depth, true);
                    if (snapshot.readiness_met) {
                        snapshot.stop_reason = "coverage_ready";
                        adaptive_depth_stop = true;
                    } else if (next_checkpoint_depth >= target_leaf_depth) {
                        snapshot.stop_reason = "max_depth";
                        adaptive_depth_stop = true;
                    } else {
                        snapshot.stop_reason = "checkpoint";
                        next_checkpoint_depth = next_depth_checkpoint(next_checkpoint_depth);
                    }
                    record_depth_snapshot(std::move(snapshot));
                }
                continue;
            }
            adaptive_add_depth_counter(out.diagnostics, "adaptive.depth.validated.", depth);
            BoxValidation validation = BoxValidation::Unknown;
            OracleValidationDetail detail;
            try {
                validation = oracle_->validate_node(item.node, item.intervals, item.changed_dim);
                detail = oracle_->last_validation_detail();
            } catch (const std::exception&) {
                out.diagnostics["adaptive.validation_exceptions"] += 1.0;
                validation = BoxValidation::Unknown;
            }
            out.adaptive_validated += 1;
            item.overlap_depth = detail.aabb_overlap_depth;
            item.overlap_ratio = detail.aabb_overlap_volume_ratio;
            const bool item_has_seed_hit = item.free_seed_hits > 0;
            if (item_has_seed_hit) {
                out.diagnostics["adaptive.seed_hit_validated"] += 1.0;
            }

            if (validation == BoxValidation::Free) {
                BoxNode candidate = adaptive_make_box_from_intervals(item.intervals,
                                                                     item.node,
                                                                     next_box_id(),
                                                                     detail.safety_status,
                                                                     detail.strict_audit_required);
                bool contained = false;
                for (const auto& existing : boxes_) {
                    if (intervals_subset_local(candidate.joint_intervals,
                                               existing.joint_intervals,
                                               1e-12)) {
                        contained = true;
                        break;
                    }
                }
                if (contained) {
                    out.diagnostics["adaptive.free_contained_rejects"] += 1.0;
                    continue;
                }
                const std::size_t new_index = boxes_.size();
                (void)new_index;
                boxes_.push_back(candidate);
                raw_boxes_.push_back(candidate);
                scoring_boxes.push_back(candidate);
	                oracle_->reserve_node(candidate.tree_id, candidate.id);
	                out.adaptive_free_added += 1;
	                pending_adjacency_boxes += 1;
	                adaptive_add_depth_counter(out.diagnostics, "adaptive.depth.free.", depth);
	                if (item_has_seed_hit) {
	                    out.diagnostics["adaptive.seed_hit_free"] += 1.0;
	                }
	                if (use_partition_backend && adaptive_partition_query_enabled_ && adaptive_partition_) {
	                    const int appended =
	                        adaptive_partition_->append_boxes(boxes_, new_index, adjacency_tolerance);
	                    out.diagnostics["adaptive.partition_incremental_boxes_appended"] +=
	                        static_cast<double>(std::max(0, appended));
	                    if (pending_adjacency_boxes >= kAdaptiveAdjacencyBatchSize) {
	                        refresh_main_from_partition();
	                        pending_adjacency_boxes = 0;
	                        out.diagnostics["adaptive.partition_main_refreshes"] += 1.0;
	                    }
	                } else {
	                    adjacency_[candidate.id];
	                    if (pending_adjacency_boxes >= kAdaptiveAdjacencyBatchSize) {
	                    connect_incremental_boxes(adjacency_,
	                                              boxes_,
	                                              first_unconnected_new_index,
	                                              adjacency_tolerance);
	                    first_unconnected_new_index = boxes_.size();
	                    pending_adjacency_boxes = 0;
	                    main_ids = adaptive_largest_island_ids(adjacency_);
	                    out.diagnostics["adaptive.adjacency_batch_updates"] += 1.0;
	                    }
	                }
	                continue;
	            }

            double active_overlap_depth_threshold = adaptive_config.overlap_depth_threshold;
            if (adaptive_config.overlap_depth_decay_per_depth > 0.0 &&
                depth > adaptive_config.defer_min_depth) {
                active_overlap_depth_threshold =
                    active_overlap_depth_threshold /
                    (1.0 + adaptive_config.overlap_depth_decay_per_depth *
                               static_cast<double>(depth - adaptive_config.defer_min_depth));
            }
            if (adaptive_config.overlap_depth_min_threshold > 0.0) {
                active_overlap_depth_threshold =
                    std::max(adaptive_config.overlap_depth_min_threshold,
                             active_overlap_depth_threshold);
            }
            const bool high_overlap =
                depth >= adaptive_config.defer_min_depth &&
                ((adaptive_config.overlap_depth_threshold > 0.0 &&
                  item.overlap_depth >= active_overlap_depth_threshold) ||
                 (adaptive_config.overlap_ratio_threshold > 0.0 &&
                  item.overlap_ratio >= adaptive_config.overlap_ratio_threshold));
            const bool protected_by_seed = item_has_seed_hit;
            const AdaptiveConnectivityDominance connectivity =
                adaptive_connectivity_dominance(scoring_boxes, item, main_ids, adjacency_tolerance);
            const bool protected_by_adjacency =
                high_overlap && !protected_by_seed &&
                (connectivity.connector_candidate || connectivity.adjacent_main > 0);
            if (depth >= target_leaf_depth) {
                deferred.push_back(std::move(item));
                out.adaptive_deferred += 1;
                out.diagnostics["adaptive.deferred_depth_cap"] += 1.0;
                adaptive_add_depth_counter(out.diagnostics, "adaptive.depth.deferred.", depth);
                if (item_has_seed_hit) {
                    out.diagnostics["adaptive.seed_hit_deferred"] += 1.0;
                }
                continue;
            }
            if (high_overlap && !protected_by_seed && !protected_by_adjacency) {
                deferred.push_back(std::move(item));
                out.adaptive_deferred += 1;
                out.diagnostics["adaptive.deferred_high_overlap"] += 1.0;
                adaptive_add_depth_counter(out.diagnostics, "adaptive.depth.deferred.", depth);
                continue;
            }
            if (depth >= adaptive_config.defer_min_depth &&
                !protected_by_seed &&
                connectivity.has_free_context &&
                connectivity.isolated) {
                deferred.push_back(std::move(item));
                out.adaptive_deferred += 1;
                out.diagnostics["adaptive.deferred_connectivity_isolated"] += 1.0;
                adaptive_add_depth_counter(out.diagnostics, "adaptive.depth.deferred.", depth);
                continue;
            }
            if (depth >= adaptive_config.defer_min_depth &&
                !protected_by_seed &&
                connectivity.has_free_context &&
                connectivity.single_component &&
                connectivity.adjacent_main == 0) {
                deferred.push_back(std::move(item));
                out.adaptive_deferred += 1;
                out.diagnostics["adaptive.deferred_connectivity_single_component"] += 1.0;
                adaptive_add_depth_counter(out.diagnostics, "adaptive.depth.deferred.", depth);
                continue;
            }

            AdaptiveFrontierItem left;
            AdaptiveFrontierItem right;
            if (!adaptive_virtual_split_node(split_descriptor, item, left, right)) {
                deferred.push_back(std::move(item));
                out.adaptive_deferred += 1;
                out.diagnostics["adaptive.deferred_split_failure"] += 1.0;
                continue;
            }
            out.adaptive_splits += 1;
            if (item_has_seed_hit) {
                out.diagnostics["adaptive.seed_hit_splits"] += 1.0;
            }
            adaptive_add_depth_counter(out.diagnostics, "adaptive.depth.split.", depth);
            if (planning_domain.empty() || intervals_overlap_local(left.intervals, planning_domain, 0.0)) {
                push_frontier(std::move(left));
            } else {
                out.diagnostics["adaptive.split_child_outside_domain"] += 1.0;
            }
            if (planning_domain.empty() || intervals_overlap_local(right.intervals, planning_domain, 0.0)) {
                push_frontier(std::move(right));
            } else {
                out.diagnostics["adaptive.split_child_outside_domain"] += 1.0;
            }
            if (adaptive_config.promotion_interval > 0 &&
                out.adaptive_validated % adaptive_config.promotion_interval == 0) {
                promote_deferred();
            }
        }
    }
    restore_checkpoint_hold();
    if (adaptive_depth_enabled && !adaptive_depth_stop && budget_exhausted()) {
        auto snapshot = evaluate_depth_snapshot(depth_snapshots.empty()
                                                    ? initial_leaf_depth
                                                    : depth_snapshots.back().depth,
                                                true);
        snapshot.stop_reason = "budget";
        adaptive_depth_stop = true;
        record_depth_snapshot(std::move(snapshot));
    } else if (adaptive_depth_enabled && !adaptive_depth_stop && frontier.empty()) {
        auto snapshot = evaluate_depth_snapshot(depth_snapshots.empty()
                                                    ? initial_leaf_depth
                                                    : depth_snapshots.back().depth,
                                                true);
        snapshot.stop_reason = "frontier_empty";
        adaptive_depth_stop = true;
        record_depth_snapshot(std::move(snapshot));
    }
    promote_deferred();
	    while (!frontier.empty()) {
	        deferred.push_back(frontier.top());
	        frontier.pop();
	    }
	    out.unresolved_domains = static_cast<int>(deferred.size());
	    out.adaptive_ms = std::chrono::duration<double, std::milli>(Clock::now() - adaptive_start).count();

		    AdjacencyBuildStats final_adjacency_stats;
		    if (use_partition_backend && adaptive_partition_query_enabled_ && adaptive_partition_) {
		        refresh_main_from_partition();
		    } else if (!use_partition_backend) {
		        rebuild_adjacency();
	        final_adjacency_stats = last_adjacency_build_stats();
	        main_ids = adaptive_largest_island_ids(adjacency_);
		    } else {
		        out.diagnostics["adaptive.partition_missing_no_graph_fallback"] = 1.0;
		    }
    if (depth_snapshots.empty()) {
        auto snapshot = evaluate_depth_snapshot(adaptive_depth_enabled ? initial_leaf_depth : target_leaf_depth,
                                                true);
        snapshot.stop_reason = adaptive_depth_enabled ? "max_depth" : "fixed_depth";
        record_depth_snapshot(std::move(snapshot));
    } else if (!adaptive_depth_enabled || depth_snapshots.back().stop_reason == "checkpoint") {
        auto snapshot = evaluate_depth_snapshot(adaptive_depth_enabled
                                                    ? depth_snapshots.back().depth
                                                    : target_leaf_depth,
                                                true);
        snapshot.stop_reason = adaptive_depth_enabled ? "max_depth" : "fixed_depth";
        record_depth_snapshot(std::move(snapshot));
    }
    apply_final_depth_snapshot(depth_snapshots.back());
    out.adaptive_depth_snapshots_json = adaptive_depth_snapshots_to_json(depth_snapshots);
    out.coverage_probe_ms = initial_probe_ms + checkpoint_probe_ms_total;

    out.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - total_start).count();
    out.profile = {};
    out.profile.raw_boxes = static_cast<int>(raw_boxes_.size());
    out.profile.final_boxes = static_cast<int>(boxes_.size());
	    out.profile.segment_edges = static_cast<int>(segment_edges_.size());
	    out.profile.grow_ms = out.leaf_sweep_ms + out.adaptive_ms;
	    out.profile.total_ms = out.total_ms;
	    out.profile.grow_largest_island = 0;
	    if (use_partition_backend) {
	        if (adaptive_partition_query_enabled_ && adaptive_partition_) {
	            const auto& partition_stats = adaptive_partition_->stats();
	            out.profile.grow_adjacency_islands = partition_stats.islands;
	            out.profile.adjacency_islands = partition_stats.islands;
	            out.profile.grow_largest_island = partition_stats.largest_island;
	        }
	    } else {
	        const auto graph_islands = find_islands(adjacency_);
	        out.profile.grow_adjacency_islands = static_cast<int>(graph_islands.size());
	        out.profile.adjacency_islands = out.profile.grow_adjacency_islands;
	        for (const auto& island : graph_islands) {
	            out.profile.grow_largest_island =
	                std::max(out.profile.grow_largest_island, static_cast<int>(island.size()));
	        }
	    }
    out.profile.connector_ms = 0.0;
    out.profile.diagnostics = out.leaf_sweep.diagnostics;
    merge_diagnostic_snapshot(out.profile.diagnostics, out.diagnostics);
    out.profile.diagnostics["adaptive.leaf_sweep_ms"] = out.leaf_sweep_ms;
    record_depth_semantics_diagnostics(out.profile.diagnostics,
                                       "adaptive.",
                                       adaptive_config.shallow_start_depth,
                                       initial_leaf_depth,
                                       target_leaf_depth,
                                       config_.grower.find_free_box,
                                       target_leaf_depth);
    out.profile.diagnostics["adaptive.overlap_depth_threshold"] = adaptive_config.overlap_depth_threshold;
    out.profile.diagnostics["adaptive.overlap_depth_min_threshold"] = adaptive_config.overlap_depth_min_threshold;
    out.profile.diagnostics["adaptive.overlap_depth_decay_per_depth"] = adaptive_config.overlap_depth_decay_per_depth;
    out.profile.diagnostics["adaptive.merge_ms"] = merge_ms;
    out.profile.diagnostics["adaptive.merge_input_boxes"] = static_cast<double>(merge_stats.input_boxes);
    out.profile.diagnostics["adaptive.merge_output_boxes"] = static_cast<double>(merge_stats.output_boxes);
    out.profile.diagnostics["adaptive.merge_grid_ms"] = merge_stats.grid_ms;
    out.profile.diagnostics["adaptive.merge_grid_merges"] = static_cast<double>(merge_stats.grid_merges);
    out.profile.diagnostics["adaptive.merge_grid_rounds"] = static_cast<double>(merge_stats.grid_rounds);
    out.profile.diagnostics["adaptive.merge_tree_ms"] = merge_stats.tree_ms;
    out.profile.diagnostics["adaptive.merge_tree_merges"] = static_cast<double>(merge_stats.tree_merges);
    out.profile.diagnostics["adaptive.merge_tree_rounds"] = static_cast<double>(merge_stats.tree_rounds);
    out.profile.diagnostics["adaptive.merge_containment_ms"] = merge_stats.containment_ms;
    out.profile.diagnostics["adaptive.merge_exact_ms"] = merge_stats.exact_ms;
    out.profile.diagnostics["adaptive.merge_containment_pruned"] = static_cast<double>(merge_stats.containment_pruned);
    out.profile.diagnostics["adaptive.merge_exact_merges"] = static_cast<double>(merge_stats.exact_merges);
    out.profile.diagnostics["adaptive.merge_rounds"] = static_cast<double>(merge_stats.rounds);
    out.profile.diagnostics["adaptive.merge_stop_reason"] = static_cast<double>(merge_stats.stop_reason);
    out.profile.diagnostics["adaptive.partition_merge_enabled"] =
        out.diagnostics.find("adaptive.partition_merge_enabled") != out.diagnostics.end() ? 1.0 : 0.0;
    out.profile.diagnostics["adaptive.partition_merge_released_boxes"] =
        out.diagnostics.find("adaptive.partition_merge_released_boxes") != out.diagnostics.end()
            ? out.diagnostics["adaptive.partition_merge_released_boxes"]
            : 0.0;
    out.profile.diagnostics["adaptive.partition_merge_containment_skipped"] =
        out.diagnostics.find("adaptive.partition_merge_containment_skipped") != out.diagnostics.end()
            ? out.diagnostics["adaptive.partition_merge_containment_skipped"]
            : 0.0;
    out.profile.diagnostics["adaptive.partition_merge_containment_bucket_entries"] =
        out.diagnostics.find("adaptive.partition_merge_containment_bucket_entries") != out.diagnostics.end()
            ? out.diagnostics["adaptive.partition_merge_containment_bucket_entries"]
            : 0.0;
    out.profile.diagnostics["adaptive.partition_merge_containment_candidates"] =
        out.diagnostics.find("adaptive.partition_merge_containment_candidates") != out.diagnostics.end()
            ? out.diagnostics["adaptive.partition_merge_containment_candidates"]
            : 0.0;
    out.profile.diagnostics["adaptive.partition_merge_containment_tests"] =
        out.diagnostics.find("adaptive.partition_merge_containment_tests") != out.diagnostics.end()
            ? out.diagnostics["adaptive.partition_merge_containment_tests"]
            : 0.0;
    out.profile.diagnostics["adaptive.partition_merge_containment_overflow"] =
        out.diagnostics.find("adaptive.partition_merge_containment_overflow") != out.diagnostics.end()
            ? out.diagnostics["adaptive.partition_merge_containment_overflow"]
            : 0.0;
    out.profile.diagnostics["adaptive.partition_merge_containment_ms"] =
        out.diagnostics.find("adaptive.partition_merge_containment_ms") != out.diagnostics.end()
            ? out.diagnostics["adaptive.partition_merge_containment_ms"]
            : 0.0;
    out.profile.diagnostics["adaptive.partition_merge_line_ms"] =
        out.diagnostics.find("adaptive.partition_merge_line_ms") != out.diagnostics.end()
            ? out.diagnostics["adaptive.partition_merge_line_ms"]
            : 0.0;
    out.profile.diagnostics["adaptive.initial_adjacency_ms"] = initial_adjacency_stats.build_ms;
    out.profile.diagnostics["adaptive.initial_adjacency_candidates"] = static_cast<double>(initial_adjacency_stats.candidate_pairs);
    out.profile.diagnostics["adaptive.initial_adjacency_exact_tests"] = static_cast<double>(initial_adjacency_stats.exact_tests);
    out.profile.diagnostics["adaptive.adjacency_ms"] = final_adjacency_stats.build_ms;
    out.profile.diagnostics["adaptive.adjacency_boxes"] = static_cast<double>(final_adjacency_stats.boxes);
    out.profile.diagnostics["adaptive.adjacency_selected_dims"] = static_cast<double>(final_adjacency_stats.selected_dims);
    out.profile.diagnostics["adaptive.adjacency_primary_dim"] = static_cast<double>(final_adjacency_stats.primary_dim);
    out.profile.diagnostics["adaptive.adjacency_candidates"] = static_cast<double>(final_adjacency_stats.candidate_pairs);
    out.profile.diagnostics["adaptive.adjacency_exact_tests"] = static_cast<double>(final_adjacency_stats.exact_tests);
    out.profile.diagnostics["adaptive.adjacency_edges"] = static_cast<double>(final_adjacency_stats.edges);
    out.profile.diagnostics["adaptive.adaptive_ms"] = out.adaptive_ms;
    out.profile.diagnostics["adaptive.coverage_probe_ms"] = out.coverage_probe_ms;
    out.profile.diagnostics["adaptive.total_ms"] = out.total_ms;
    out.profile.diagnostics["adaptive.depth_enabled"] = adaptive_depth_enabled ? 1.0 : 0.0;
    out.profile.diagnostics["adaptive.depth_min"] = static_cast<double>(adaptive_depth_min);
    out.profile.diagnostics["adaptive.depth_max"] = static_cast<double>(target_leaf_depth);
    out.profile.diagnostics["adaptive.depth_min_covered_probes"] =
        static_cast<double>(adaptive_config.adaptive_depth_min_covered_probes);
    out.profile.diagnostics["adaptive.depth_min_main_probes"] =
        static_cast<double>(adaptive_config.adaptive_depth_min_main_probes);
    out.profile.diagnostics["adaptive.depth_min_main_ratio"] =
        adaptive_config.adaptive_depth_min_main_ratio;
    out.profile.diagnostics["adaptive.depth_min_cells"] =
        static_cast<double>(adaptive_config.adaptive_depth_min_cells);
    out.profile.diagnostics["adaptive.depth_min_main_cells"] =
        static_cast<double>(adaptive_config.adaptive_depth_min_main_cells);
    out.profile.diagnostics["adaptive.depth_max_online_cells"] =
        static_cast<double>(adaptive_config.adaptive_depth_max_online_cells);
    out.profile.diagnostics["adaptive.shallow_free_count"] = static_cast<double>(out.shallow_free_count);
    out.profile.diagnostics["adaptive.shallow_collision_count"] = static_cast<double>(out.shallow_collision_count);
    out.profile.diagnostics["adaptive.free_added"] = static_cast<double>(out.adaptive_free_added);
    out.profile.diagnostics["adaptive.validated"] = static_cast<double>(out.adaptive_validated);
    out.profile.diagnostics["adaptive.splits"] = static_cast<double>(out.adaptive_splits);
	    out.profile.diagnostics["adaptive.deferred"] = static_cast<double>(out.adaptive_deferred);
	    out.profile.diagnostics["adaptive.promoted"] = static_cast<double>(out.adaptive_promoted);
	    out.profile.diagnostics["adaptive.unresolved_domains"] = static_cast<double>(out.unresolved_domains);
    const int final_anchor_cap = adaptive_depth_enabled
        ? std::max(0, adaptive_config.adaptive_depth_anchor_probe_cap)
        : std::max(0, adaptive_config.seed_anchor_probe_cap);
    const int final_anchor_attempts = depth_snapshots.empty()
        ? 0
        : depth_snapshots.back().anchor_probe_attempts;
	    out.profile.diagnostics["adaptive.seed_probe_count"] = static_cast<double>(out.seed_probe_count);
    out.profile.diagnostics["adaptive.seed_probe_free_count"] = static_cast<double>(out.seed_probe_free_count);
    out.profile.diagnostics["adaptive.seed_probe_box_covered"] = static_cast<double>(out.seed_probe_box_covered);
    out.profile.diagnostics["adaptive.seed_probe_anchor_success"] = static_cast<double>(out.seed_probe_anchor_success);
    out.profile.diagnostics["adaptive.seed_probe_main_accessible"] = static_cast<double>(out.seed_probe_main_accessible);
	    out.profile.diagnostics["adaptive.seed_anchor_probe_cap"] = static_cast<double>(final_anchor_cap);
	    out.profile.diagnostics["adaptive.seed_anchor_probe_attempts"] = static_cast<double>(final_anchor_attempts);
    out.profile.diagnostics["adaptive.p_box_covered"] = out.p_box_covered;
    out.profile.diagnostics["adaptive.p_anchor_success"] = out.p_anchor_success;
    out.profile.diagnostics["adaptive.p_main_accessible"] = out.p_main_accessible;
    out.profile.diagnostics["adaptive.p_anchor_to_main_uncovered"] = out.p_anchor_to_main_uncovered;
    out.profile.diagnostics["adaptive.selected_leaf_depth"] = static_cast<double>(out.selected_leaf_depth);
    out.profile.diagnostics["adaptive.depth_readiness_met"] =
        out.adaptive_depth_readiness_met ? 1.0 : 0.0;
	    if (use_partition_backend && adaptive_partition_query_enabled_ && adaptive_partition_) {
	        refresh_adaptive_partition_diagnostics(&out.profile);
	    } else {
	        rebuild_adaptive_partition(partition_config, &out.profile);
	    }
    if (adaptive_partition_ && !adaptive_partition_->empty()) {
        const auto& partition_stats = adaptive_partition_->stats();
        out.partition_cell_count = partition_stats.cells;
        out.partition_grid_cell_count = partition_stats.grid_cells;
        out.partition_non_grid_cell_count = partition_stats.non_grid_cells;
        out.partition_face_index_entries = partition_stats.face_index_entries;
        out.partition_islands = partition_stats.islands;
        out.partition_largest_island = partition_stats.largest_island;
    }
    if (oracle_) {
        const OracleCounters counters = oracle_->counters();
        normalize_external_evidence_diagnostics(out.profile.diagnostics, &counters);
    } else {
        normalize_external_evidence_diagnostics(out.profile.diagnostics);
    }
    record_portal_membership_policy(out.profile.diagnostics, config_.portal_membership_policy);
    out.diagnostics = out.profile.diagnostics;
    last_build_ = out.profile;
    if (config_.database.checkpoint_after_build && database_) {
        database_->checkpoint();
    }
    return out;
}

LeafSweepRefineResult RBFPlanningForest::build_leaf_sweep_refined(
    const std::vector<Obstacle>& obstacles,
    const LeafSweepRefineConfig& refine_config,
    const std::vector<Eigen::VectorXd>& priority_points,
    const std::vector<Eigen::VectorXd>& offline_anchor_points) {
    using Clock = std::chrono::steady_clock;
    const auto total_start = Clock::now();

    LeafSweepRefineResult out;
    LeafSweepConfig leaf_config;
    leaf_config.obstacle_cluster_gap = refine_config.obstacle_cluster_gap;
    leaf_config.n_threads = refine_config.leaf_threads;
    leaf_config.validation_batch_size = refine_config.validation_batch_size;
    leaf_config.timeout_ms = refine_config.leaf_timeout_ms;
    leaf_config.store_group_results = refine_config.store_group_results;
    leaf_config.use_virtual_topology = refine_config.use_virtual_topology;
    leaf_config.parallel_virtual_validation = refine_config.parallel_virtual_validation;
    leaf_config.collision_overlap_prune_min_depth = refine_config.collision_overlap_prune_min_depth;
    leaf_config.collision_overlap_prune_threshold = refine_config.collision_overlap_prune_threshold;
    leaf_config.collision_overlap_prune_ratio_threshold =
        refine_config.collision_overlap_prune_ratio_threshold;

    out.leaf_sweep = build_leaf_sweep(obstacles,
                                      refine_config.leaf_start_depth,
                                      refine_config.leaf_max_depth,
                                      leaf_config);
    const auto priority_prune = prune_leaf_sweep_to_priority(out.leaf_sweep,
                                                             boxes_,
                                                             raw_boxes_,
                                                             priority_points,
                                                             refine_config.priority_prune_radius);
    if (refine_config.priority_prune_radius > 0.0 && !priority_points.empty()) {
        dynamic_collision_box_cache_.clear();
        populate_dynamic_collision_cache(out.leaf_sweep, static_cast<int>(obstacles.size()));
        reserve_existing_boxes();
        adjacency_.clear();
        segment_edges_.clear();
        invalidate_query_cache();
    }
    out.leaf_sweep_ms = out.leaf_sweep.total_ms;
    out.leaf_free_count = static_cast<int>(out.leaf_sweep.free_boxes.size());
    out.leaf_collision_count = static_cast<int>(out.leaf_sweep.collision_boxes.size());

    const double adjacency_tolerance = config_.query.adjacency_tolerance;
    MergerResult leaf_merge_result;
    const auto leaf_merge_start = Clock::now();
    if (config_.enable_merger && !boxes_.empty()) {
        MergerConfig leaf_merge_config = config_.merger;
        leaf_merge_config.exact_face_merge = true;
        leaf_merge_config.greedy_hull_merge = false;
        leaf_merge_config.containment_prune = true;
        leaf_merge_config.adjacency_tolerance = adjacency_tolerance;
        leaf_merge_config.max_rounds = std::max(1, leaf_merge_config.max_rounds);
        leaf_merge_result = fast_exact_face_merge_leaf(*oracle_, boxes_, leaf_merge_config);
        raw_boxes_ = boxes_;
    }
    const double leaf_merge_ms =
        std::chrono::duration<double, std::milli>(Clock::now() - leaf_merge_start).count();
    rebuild_adjacency();
    const auto refine_start = Clock::now();
    Deadline refine_deadline = refine_config.refine_timeout_ms > 0.0
        ? Deadline::after_ms(refine_config.refine_timeout_ms)
        : Deadline{};
    StageContext refine_context = StageContext::from_runtime(config_.runtime, refine_deadline);
    FindFreeBoxOptions refine_options = config_.grower.find_free_box;
    refine_options.max_depth = refine_config.deep_ffb_depth;
    refine_options.reject_seed_collision = false;
    int next_id = next_box_id();
    auto find_in_domain = [this](const Eigen::VectorXd& seed,
                                 const std::vector<Interval>& domain,
                                 StageContext& context,
                                 const FindFreeBoxOptions& options) {
        return this->find_free_box_in_domain(seed, domain, context, options);
    };
    const auto offline_anchors = run_offline_anchor_grower(*oracle_,
                                                           refine_config,
                                                           out.leaf_sweep.collision_boxes,
                                                           offline_anchor_points,
                                                           find_in_domain,
                                                           config_.grower.commit_policy,
                                                           boxes_,
                                                           raw_boxes_,
                                                           adjacency_,
                                                           next_id,
                                                           refine_context,
                                                           refine_options,
                                                           adjacency_tolerance);
    const auto qroot = run_query_root_box_grower(*oracle_,
                                                 refine_config,
                                                 out.leaf_sweep.collision_boxes,
                                                 priority_points,
                                                 find_in_domain,
                                                 config_.grower.commit_policy,
                                                 boxes_,
                                                 raw_boxes_,
                                                 adjacency_,
                                                 next_id,
                                                 refine_context,
                                                 refine_options,
                                                 adjacency_tolerance);
    out.deep_boxes_added = qroot.boxes_added;
    out.deep_domain_attempts = qroot.pair_attempts;
    out.deep_ffb_success = qroot.ffb_success;
    out.deep_ffb_fail = qroot.ffb_fail;
    out.deep_commit_rejects = qroot.commit_rejects;
    out.deep_domain_rejects = qroot.domain_rejects;
    out.deep_contained_rejects = qroot.contained_rejects;
    out.deep_adjacency_rejects = qroot.adjacency_rejects;
    out.deep_anchor_roots_added = qroot.endpoint_anchors_added;
    out.deep_refine_ms = std::chrono::duration<double, std::milli>(Clock::now() - refine_start).count();
    out.rrt_grower_ms = 0.0;
    out.rrt_grower_boxes_added = 0;
    out.rrt_grower_ffb_success = 0;
    out.rrt_grower_ffb_fail = 0;

    const auto connector_start = Clock::now();
    bool connector_ran = false;
    std::unordered_map<std::string, double> connector_diagnostics;
    if (config_.enable_connector && !boxes_.empty()) {
        StageContext connector_context = StageContext::from_runtime(config_.runtime);
        CollisionChecker checker(robot_, scene_);
        IslandConnectorConfig connector_config = config_.connector;
        if (connector_config.n_threads <= 1 && config_.runtime.n_threads > 1) {
            connector_config.n_threads = config_.runtime.n_threads;
        }
        int connector_next_id = next_id;
        IslandConnectorConfig box_only_config = connector_config;
        box_only_config.segment_edges_fallback_only = true;
        {
            IslandConnector connector(*oracle_, robot_, checker, box_only_config);
            const auto connector_result = connector.connect_all(boxes_,
                                                                adjacency_,
                                                                segment_edges_,
                                                                connector_next_id,
                                                                connector_context);
            out.profile.bridge_boxes_added += connector_result.bridge_boxes_added;
            out.profile.connector_attempted_pairs += connector_result.attempted_pairs;
            out.profile.connector_connected = connector_result.connected;
        }
        if (find_islands(adjacency_).size() > 1 &&
            connector_config.segment_edges_enabled &&
            (connector_config.rrt_segment_edges || connector_config.point_gap_segment_edges)) {
            IslandConnectorConfig fallback_config = connector_config;
            fallback_config.segment_edges_fallback_only = false;
            fallback_config.max_total_bridge_boxes = 0;
            fallback_config.max_pairs_per_gap = std::max(fallback_config.max_pairs_per_gap, 4);
            IslandConnector fallback_connector(*oracle_, robot_, checker, fallback_config);
            const auto fallback_result = fallback_connector.connect_all(boxes_,
                                                                        adjacency_,
                                                                        segment_edges_,
                                                                        connector_next_id,
                                                                        connector_context);
            out.profile.bridge_boxes_added += fallback_result.bridge_boxes_added;
            out.profile.segment_edges_added += fallback_result.segment_edges_added;
            out.profile.rrt_segment_edges_added += fallback_result.rrt_segment_edges_added;
            out.profile.point_gap_segment_edges_added += fallback_result.point_gap_segment_edges_added;
            out.profile.connector_attempted_pairs += fallback_result.attempted_pairs;
            out.profile.connector_connected = fallback_result.connected;
        }
        next_id = connector_next_id;
        connector_diagnostics = connector_context.diagnostics().snapshot();
        connector_ran = true;
    }
    out.connector_ms = std::chrono::duration<double, std::milli>(Clock::now() - connector_start).count();
    out.profile.connector_ms = out.connector_ms;

    const auto adjacency_start = Clock::now();
    out.profile.segment_edges = static_cast<int>(segment_edges_.size());
    if (!connector_ran) {
        rebuild_adjacency();
    }
    out.profile.adjacency_ms = std::chrono::duration<double, std::milli>(Clock::now() - adjacency_start).count();
    out.profile.raw_boxes = static_cast<int>(raw_boxes_.size());
    out.profile.final_boxes = static_cast<int>(boxes_.size());
    out.profile.grow_ms = out.leaf_sweep_ms + out.deep_refine_ms + out.rrt_grower_ms;
    out.profile.grow_adjacency_islands = static_cast<int>(find_islands(adjacency_).size());
    out.profile.grow_largest_island = 0;
    for (const auto& island : find_islands(adjacency_)) {
        out.profile.grow_largest_island =
            std::max(out.profile.grow_largest_island, static_cast<int>(island.size()));
    }
    out.profile.adjacency_islands = out.profile.grow_adjacency_islands;
    out.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - total_start).count();
    out.profile.total_ms = out.total_ms;
    out.profile.diagnostics = out.leaf_sweep.diagnostics;
    out.profile.diagnostics["leaf_refine.leaf_sweep_ms"] = out.leaf_sweep_ms;
    record_depth_semantics_diagnostics(out.profile.diagnostics,
                                       "leaf_refine.",
                                       refine_config.leaf_start_depth,
                                       refine_config.leaf_max_depth,
                                       refine_config.deep_ffb_depth,
                                       config_.grower.find_free_box,
                                       refine_config.deep_ffb_depth);
    out.profile.diagnostics["leaf_refine.leaf_free_count"] = static_cast<double>(out.leaf_free_count);
    out.profile.diagnostics["leaf_refine.leaf_collision_count"] = static_cast<double>(out.leaf_collision_count);
    out.profile.diagnostics["leaf_refine.priority_prune_radius"] = refine_config.priority_prune_radius;
    out.profile.diagnostics["leaf_refine.collision_overlap_prune_min_depth"] =
        static_cast<double>(refine_config.collision_overlap_prune_min_depth);
    out.profile.diagnostics["leaf_refine.collision_overlap_prune_threshold"] =
        refine_config.collision_overlap_prune_threshold;
    out.profile.diagnostics["leaf_refine.collision_overlap_prune_ratio_threshold"] =
        refine_config.collision_overlap_prune_ratio_threshold;
    out.profile.diagnostics["leaf_refine.priority_prune_free_before"] =
        static_cast<double>(priority_prune.free_before);
    out.profile.diagnostics["leaf_refine.priority_prune_free_after"] =
        static_cast<double>(priority_prune.free_after);
    out.profile.diagnostics["leaf_refine.priority_prune_collision_before"] =
        static_cast<double>(priority_prune.collision_before);
    out.profile.diagnostics["leaf_refine.priority_prune_collision_after"] =
        static_cast<double>(priority_prune.collision_after);
    out.profile.diagnostics["leaf_refine.leaf_merge_ms"] = leaf_merge_ms;
    out.profile.diagnostics["leaf_refine.leaf_merge_boxes_before"] =
        static_cast<double>(leaf_merge_result.boxes_before);
    out.profile.diagnostics["leaf_refine.leaf_merge_boxes_after"] =
        static_cast<double>(leaf_merge_result.boxes_after);
    out.profile.diagnostics["leaf_refine.leaf_merge_exact"] =
        static_cast<double>(leaf_merge_result.exact_merges);
    out.profile.diagnostics["leaf_refine.leaf_merge_pruned"] =
        static_cast<double>(leaf_merge_result.pruned_boxes);
    out.profile.diagnostics["leaf_refine.collision_cache_boxes"] =
        static_cast<double>(dynamic_collision_box_cache_.size());
    out.profile.diagnostics["leaf_refine.deep_refine_ms"] = out.deep_refine_ms;
    out.profile.diagnostics["leaf_refine.offline_anchor_ms"] = offline_anchors.total_ms;
    out.profile.diagnostics["leaf_refine.offline_anchor_candidates"] =
        static_cast<double>(offline_anchors.candidates_total);
    out.profile.diagnostics["leaf_refine.offline_anchor_candidates_covered"] =
        static_cast<double>(offline_anchors.candidates_covered);
    out.profile.diagnostics["leaf_refine.offline_anchor_roots_added"] =
        static_cast<double>(offline_anchors.boxes_added);
    out.profile.diagnostics["leaf_refine.offline_anchor_ffb_success"] =
        static_cast<double>(offline_anchors.ffb_success);
    out.profile.diagnostics["leaf_refine.offline_anchor_ffb_fail"] =
        static_cast<double>(offline_anchors.ffb_fail);
    out.profile.diagnostics["leaf_refine.offline_anchor_commit_rejects"] =
        static_cast<double>(offline_anchors.commit_rejects);
    out.profile.diagnostics["leaf_refine.offline_anchor_domain_rejects"] =
        static_cast<double>(offline_anchors.domain_rejects);
    out.profile.diagnostics["leaf_refine.offline_anchor_contained_rejects"] =
        static_cast<double>(offline_anchors.contained_rejects);
    out.profile.diagnostics["leaf_refine.offline_anchor_adjacency_rejects"] =
        static_cast<double>(offline_anchors.adjacency_rejects);
    out.profile.diagnostics["leaf_refine.offline_anchor_adjacency_candidates_tested"] =
        static_cast<double>(offline_anchors.adjacency_candidates_tested);
    out.profile.diagnostics["leaf_refine.offline_anchor_adjacency_edges_added"] =
        static_cast<double>(offline_anchors.adjacency_edges_added);
    out.profile.diagnostics["leaf_refine.offline_anchor_islands_before"] =
        static_cast<double>(offline_anchors.islands_before);
    out.profile.diagnostics["leaf_refine.offline_anchor_islands_after"] =
        static_cast<double>(offline_anchors.islands_after);
    out.profile.diagnostics["leaf_refine.offline_anchor_box_volume_mean"] =
        offline_anchors.boxes_added > 0
            ? offline_anchors.box_volume_sum / static_cast<double>(offline_anchors.boxes_added)
            : 0.0;
    out.profile.diagnostics["leaf_refine.offline_anchor_box_volume_max"] =
        offline_anchors.box_volume_max;
    out.profile.diagnostics["leaf_refine.offline_anchor_index_rebuild_ms"] =
        offline_anchors.index_rebuild_ms;
    out.profile.diagnostics["leaf_refine.offline_anchor_index_query_ms"] =
        offline_anchors.index_query_ms;
    out.profile.diagnostics["leaf_refine.deep_boxes_added"] = static_cast<double>(out.deep_boxes_added);
    out.profile.diagnostics["leaf_refine.deep_domain_attempts"] = static_cast<double>(out.deep_domain_attempts);
    out.profile.diagnostics["leaf_refine.deep_ffb_success"] = static_cast<double>(out.deep_ffb_success);
    out.profile.diagnostics["leaf_refine.deep_ffb_fail"] = static_cast<double>(out.deep_ffb_fail);
    out.profile.diagnostics["leaf_refine.deep_commit_rejects"] = static_cast<double>(out.deep_commit_rejects);
    out.profile.diagnostics["leaf_refine.deep_domain_rejects"] = static_cast<double>(out.deep_domain_rejects);
    out.profile.diagnostics["leaf_refine.deep_contained_rejects"] = static_cast<double>(out.deep_contained_rejects);
    out.profile.diagnostics["leaf_refine.deep_adjacency_rejects"] = static_cast<double>(out.deep_adjacency_rejects);
    out.profile.diagnostics["leaf_refine.deep_anchor_roots_added"] = static_cast<double>(out.deep_anchor_roots_added);
    out.profile.diagnostics["leaf_refine.qroot_ms"] = qroot.total_ms;
    out.profile.diagnostics["leaf_refine.qroot_pairs_total"] = static_cast<double>(qroot.pairs_total);
    out.profile.diagnostics["leaf_refine.qroot_pairs_connected_before"] =
        static_cast<double>(qroot.pairs_connected_before);
    out.profile.diagnostics["leaf_refine.qroot_pairs_connected_after"] =
        static_cast<double>(qroot.pairs_connected_after);
    out.profile.diagnostics["leaf_refine.qroot_uncovered_endpoints"] =
        static_cast<double>(qroot.uncovered_endpoints);
	out.profile.diagnostics["leaf_refine.qroot_endpoint_anchors_added"] =
		static_cast<double>(qroot.endpoint_anchors_added);
	out.profile.diagnostics["leaf_refine.qroot_endpoint_root_fallbacks"] =
		static_cast<double>(qroot.endpoint_root_fallbacks);
	out.profile.diagnostics["leaf_refine.qroot_boxes_added"] = static_cast<double>(qroot.boxes_added);
    out.profile.diagnostics["leaf_refine.qroot_ffb_success"] = static_cast<double>(qroot.ffb_success);
    out.profile.diagnostics["leaf_refine.qroot_ffb_fail"] = static_cast<double>(qroot.ffb_fail);
    out.profile.diagnostics["leaf_refine.qroot_adjacency_candidates_tested"] =
        static_cast<double>(qroot.adjacency_candidates_tested);
    out.profile.diagnostics["leaf_refine.qroot_adjacency_edges_added"] =
        static_cast<double>(qroot.adjacency_edges_added);
    out.profile.diagnostics["leaf_refine.qroot_index_rebuild_ms"] = qroot.index_rebuild_ms;
    out.profile.diagnostics["leaf_refine.qroot_index_query_ms"] = qroot.index_query_ms;
    out.profile.diagnostics["leaf_refine.qroot_islands_before"] = static_cast<double>(qroot.islands_before);
    out.profile.diagnostics["leaf_refine.qroot_islands_after"] = static_cast<double>(qroot.islands_after);
    out.profile.diagnostics["leaf_refine.rrt_grower_ms"] = out.rrt_grower_ms;
    out.profile.diagnostics["leaf_refine.rrt_grower_initial_boxes"] = 0.0;
    out.profile.diagnostics["leaf_refine.rrt_grower_boxes_added"] = static_cast<double>(out.rrt_grower_boxes_added);
    out.profile.diagnostics["leaf_refine.rrt_grower_ffb_success"] = static_cast<double>(out.rrt_grower_ffb_success);
    out.profile.diagnostics["leaf_refine.rrt_grower_ffb_fail"] = static_cast<double>(out.rrt_grower_ffb_fail);
    out.profile.diagnostics["leaf_refine.rrt_grower_deadline_reached"] = 0.0;
    out.profile.diagnostics["leaf_refine.connector_ms"] = out.connector_ms;
    for (const auto& [key, value] : connector_diagnostics) {
        out.profile.diagnostics[std::string("leaf_refine.") + key] = value;
    }
    out.profile.diagnostics["leaf_refine.total_ms"] = out.total_ms;
    record_portal_membership_policy(out.profile.diagnostics, config_.portal_membership_policy);
    out.diagnostics = out.profile.diagnostics;
    last_build_ = out.profile;
    invalidate_query_cache();
    if (config_.database.checkpoint_after_build && database_) {
        database_->checkpoint();
    }
    return out;
}

FindFreeBoxResult RBFPlanningForest::find_free_box_in_domain(const Eigen::Ref<const Eigen::VectorXd>& seed,
                                                         const std::vector<Interval>& domain,
                                                         StageContext& context,
                                                         const FindFreeBoxOptions& options) {
    using Clock = std::chrono::steady_clock;
    const auto start = Clock::now();
    FindFreeBoxResult result;
    if (!oracle_ || !database_ || seed.size() != oracle_->n_dims() || domain.size() != static_cast<std::size_t>(oracle_->n_dims())) {
        result.fail_code = 5;
        return result;
    }
    if (context.should_stop()) {
        result.deadline_reached = context.deadline().expired();
        result.fail_code = 4;
        return result;
    }
    if (!oracle_->contains_point(oracle_->root_node(), seed) ||
        !intervals_contain_point_strict_local(domain, seed, 1e-12)) {
        result.fail_code = 5;
        return result;
    }
    if (!options.skip_existing_cover_check && point_covered_by_existing_box_local(boxes_, seed)) {
        context.diagnostics().add_counter("forest.find_free_box_in_domain_seed_already_covered");
        result.fail_code = 7;
        return result;
    }
    if (options.reject_seed_collision && oracle_->point_in_collision(seed)) {
        result.seed_collision = true;
        result.fail_code = 1;
        return result;
    }

    auto elapsed_ms = [&]() {
        return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
    };
    const int effective_max_depth = std::max(0, std::min(options.max_depth, oracle_->max_tree_depth() - 1));
    // Seed-independent: canonical split depends only on (robot, domain). No
    // query-seed coupling is applied to the split values.
    OracleSplitOptions split_options = options.split;
    OracleNodeId node = oracle_->root_node();
    OracleNodeTopology node_topology = oracle_->node_topology(node);
    if (!node_topology.valid) {
        result.fail_code = 5;
        return result;
    }
    int changed_dim = -1;
    const Eigen::VectorXd tree_seed = oracle_->tree_configuration_for_query(seed);
    std::vector<Interval> tree_intervals = oracle_->node_intervals(node);
    auto descend_to_seed_child = [&](const OracleNodeTopology& parent_topology) -> bool {
        const int split_dim = parent_topology.split_dim;
        if (split_dim < 0 || split_dim >= tree_seed.size()) {
            return false;
        }
        const double split_value = parent_topology.split_value;
        const OracleNodeId left_child = parent_topology.left;
        const OracleNodeId right_child = parent_topology.right;
        const OracleNodeId child =
            tree_seed[split_dim] <= split_value ? left_child : right_child;
        if (child == kInvalidOracleNodeId) {
            return false;
        }
        if (split_dim >= 0 && split_dim < static_cast<int>(tree_intervals.size())) {
            auto& interval = tree_intervals[static_cast<std::size_t>(split_dim)];
            if (child == left_child) {
                interval.hi = split_value;
            } else if (child == right_child) {
                interval.lo = split_value;
            } else {
                tree_intervals = oracle_->node_intervals(child);
            }
        } else {
            tree_intervals = oracle_->node_intervals(child);
        }
        changed_dim = split_dim;
        node = child;
        node_topology = oracle_->node_topology(node);
        if (!node_topology.valid) {
            return false;
        }
        return true;
    };
    if (options.search_mode == FindFreeBoxSearchMode::BinaryDepth &&
        options.adaptive_depths.empty()) {
        const int virtual_start_depth =
            std::max(0,
                     std::min(effective_max_depth,
                              std::max(options.start_depth, options.skip_to_depth)));
        if (detail::split_policy_supports_virtual_cells(oracle_->split_policy_descriptor(),
                                                        effective_max_depth)) {
            if (options.record_diagnostics) {
                context.diagnostics().add_counter("ffb.virtual_sparse_binary_attempts");
            }
            auto validate_virtual_depth = [&](int depth, FindFreeBoxResult& candidate) {
                if (depth < options.skip_to_depth) {
                    candidate.hit_unknown_depth_cap = true;
                    candidate.fail_code = 2;
                    return BoxValidation::Unknown;
                }
                const auto cell = detail::virtual_seed_cell_at_depth(*oracle_, seed, depth);
                if (!cell) {
                    candidate.fail_code = 6;
                    return BoxValidation::Unknown;
                }
                candidate.node = oracle_->root_node();
                candidate.changed_dim = cell->changed_dim;
                candidate.intervals = cell->query_intervals;
                if (!intervals_overlap_local(candidate.intervals, domain, 0.0)) {
                    candidate.fail_code = 5;
                    return BoxValidation::Unknown;
                }
                if (!intervals_subset_local(candidate.intervals, domain, 1e-12)) {
                    candidate.hit_unknown_depth_cap = depth >= effective_max_depth;
                    candidate.fail_code = 2;
                    return BoxValidation::Unknown;
                }
                const auto validation = oracle_->validate_node(oracle_->root_node(),
                                                               candidate.intervals,
                                                               candidate.changed_dim);
                candidate.validation_detail = oracle_->last_validation_detail();
                candidate.decisions += 1;
                if (options.record_diagnostics) {
                    context.diagnostics().add_counter("ffb.virtual_sparse_binary_probes");
                    context.diagnostics().add_counter("ffb.virtual_sparse_binary_probe_depth_sum",
                                                      static_cast<double>(depth));
                    context.diagnostics().set_value(
                        "ffb.virtual_sparse_binary_probe_depth_max",
                        std::max(context.diagnostics().value("ffb.virtual_sparse_binary_probe_depth_max"),
                                 static_cast<double>(depth)));
                }
                if (validation == BoxValidation::Free) {
                    candidate.found = true;
                    candidate.fail_code = 0;
                } else if (validation == BoxValidation::Occupied) {
                    candidate.fail_code = 3;
                } else {
                    candidate.hit_unknown_depth_cap = depth >= effective_max_depth;
                    candidate.fail_code = 2;
                }
                return validation;
            };

            int lo = virtual_start_depth;
            int hi = effective_max_depth;
            int best_depth = -1;
            FindFreeBoxResult best;
            const int probe_depth = env_int_or_default("RBF_FFB_BINARY_PROBE_DEPTH", -1);
            if (probe_depth >= virtual_start_depth && probe_depth < effective_max_depth) {
                FindFreeBoxResult probe_candidate;
                const BoxValidation probe_validation = validate_virtual_depth(probe_depth,
                                                                              probe_candidate);
                result.decisions += probe_candidate.decisions;
                if (probe_validation == BoxValidation::Free) {
                    best = std::move(probe_candidate);
                    best_depth = probe_depth;
                    hi = probe_depth;
                    if (options.record_diagnostics) {
                        context.diagnostics().add_counter("ffb.binary_probe_free");
                    }
                } else if (options.record_diagnostics) {
                    context.diagnostics().add_counter("ffb.binary_probe_not_free");
                }
            }
            if (!best.found) {
                FindFreeBoxResult high_candidate;
                const BoxValidation high_validation = validate_virtual_depth(effective_max_depth,
                                                                             high_candidate);
                result.decisions += high_candidate.decisions;
                if (high_validation != BoxValidation::Free) {
                    high_candidate.decisions = result.decisions;
                    high_candidate.splits = 0;
                    high_candidate.total_ms = elapsed_ms();
                    return high_candidate;
                }
                best = high_candidate;
                best_depth = effective_max_depth;
            }
            while (lo < hi) {
                if (context.should_stop() ||
                    (options.deadline_ms > 0.0 && elapsed_ms() > options.deadline_ms)) {
                    best.deadline_reached =
                        context.deadline().expired() || options.deadline_ms > 0.0;
                    best.fail_code = 4;
                    break;
                }
                const int mid = lo + (hi - lo) / 2;
                FindFreeBoxResult candidate;
                const BoxValidation validation = validate_virtual_depth(mid, candidate);
                result.decisions += candidate.decisions;
                if (validation == BoxValidation::Free) {
                    best = std::move(candidate);
                    best_depth = mid;
                    hi = mid;
                } else {
                    lo = mid + 1;
                }
            }
            best.decisions = result.decisions;
            if (best.found && best_depth >= 0) {
                const auto materialized = detail::materialize_seed_path_to_depth(
                    *oracle_,
                    seed,
                    best_depth,
                    split_options,
                    [&](const SplitNodeResult&,
                        const std::vector<Interval>&,
                        int,
                        double split_ms) {
                        if (options.record_diagnostics) {
                            context.diagnostics().record_timing("oracle.split_node", split_ms);
                        }
                    });
                if (materialized) {
                    result.splits += materialized->splits;
                    best.splits = result.splits;
                    best.node = materialized->node;
                    best.changed_dim = materialized->changed_dim;
                    if (!detail::intervals_equal_with_tolerance(best.intervals,
                                                                materialized->query_intervals,
                                                                1e-10)) {
                        if (options.record_diagnostics) {
                            context.diagnostics().add_counter(
                                "ffb.virtual_sparse_binary_materialize_mismatch");
                        }
                        best.intervals = materialized->query_intervals;
                        if (!intervals_overlap_local(best.intervals, domain, 0.0)) {
                            best.found = false;
                            best.fail_code = 5;
                            best.total_ms = elapsed_ms();
                            return best;
                        }
                        if (!intervals_subset_local(best.intervals, domain, 1e-12)) {
                            best.found = false;
                            best.hit_unknown_depth_cap = best_depth >= effective_max_depth;
                            best.fail_code = 2;
                            best.total_ms = elapsed_ms();
                            return best;
                        }
                        const auto validation = oracle_->validate_node(materialized->node,
                                                                       best.intervals,
                                                                       materialized->changed_dim);
                        best.validation_detail = oracle_->last_validation_detail();
                        best.decisions += 1;
                        if (validation != BoxValidation::Free) {
                            best.found = false;
                            best.fail_code = validation == BoxValidation::Occupied ? 3 : 2;
                            best.hit_unknown_depth_cap = validation == BoxValidation::Unknown;
                            best.total_ms = elapsed_ms();
                            return best;
                        }
                    } else {
                        best.intervals = materialized->query_intervals;
                    }
                    if (oracle_->is_reserved(best.node) && !options.split_reserved_leaf) {
                        best.found = false;
                        best.hit_reserved_depth_cap = true;
                        best.fail_code = 2;
                        best.total_ms = elapsed_ms();
                        return best;
                    }
                    best.total_ms = elapsed_ms();
                    if (options.record_diagnostics) {
                        context.diagnostics().add_counter("ffb.virtual_sparse_binary_successes");
                        context.diagnostics().add_counter("ffb.free_ancestor_hits");
                        context.diagnostics().add_counter("ffb.free_ancestor_depth_sum",
                                                          static_cast<double>(best_depth));
                        context.diagnostics().set_value(
                            "ffb.free_ancestor_depth_max",
                            std::max(context.diagnostics().value("ffb.free_ancestor_depth_max"),
                                     static_cast<double>(best_depth)));
                        double free_log_volume = 0.0;
                        for (const auto& interval : best.intervals) {
                            const double width = std::max(0.0, interval.width());
                            if (width > 0.0) {
                                free_log_volume += std::log(width);
                            }
                        }
                        context.diagnostics().add_counter("ffb.free_ancestor_log_volume_sum",
                                                          free_log_volume);
                    }
                    return best;
                }
                if (options.record_diagnostics) {
                    context.diagnostics().add_counter(
                        "ffb.virtual_sparse_binary_materialize_failures");
                }
                node = oracle_->root_node();
                node_topology = oracle_->node_topology(node);
                changed_dim = -1;
                tree_intervals = oracle_->node_intervals(node);
            }
        }
        struct PathEntry {
            OracleNodeId node = kInvalidOracleNodeId;
            int changed_dim = -1;
            std::vector<Interval> tree_intervals;
        };
        std::vector<PathEntry> path;
        path.reserve(static_cast<std::size_t>(effective_max_depth + 1));
        while (true) {
            if (context.should_stop() ||
                (options.deadline_ms > 0.0 && elapsed_ms() > options.deadline_ms)) {
                result.deadline_reached =
                    context.deadline().expired() || options.deadline_ms > 0.0;
                result.fail_code = 4;
                result.total_ms = elapsed_ms();
                return result;
            }
            if (node_topology.depth < 0) {
                result.fail_code = 5;
                result.total_ms = elapsed_ms();
                return result;
            }
            if (static_cast<int>(path.size()) <= node_topology.depth) {
                path.resize(static_cast<std::size_t>(node_topology.depth + 1));
            }
            path[static_cast<std::size_t>(node_topology.depth)] =
                {node, changed_dim, tree_intervals};
            if (node_topology.depth >= effective_max_depth) {
                break;
            }
            if (node_topology.leaf) {
                if (oracle_->is_reserved(node) && !options.split_reserved_leaf) {
                    result.hit_reserved_depth_cap = true;
                    result.node = node;
                    result.intervals = oracle_->query_intervals_for_node(node,
                                                                         tree_intervals,
                                                                         seed);
                    result.fail_code = 2;
                    result.total_ms = elapsed_ms();
                    return result;
                }
                if (!options.split_unknown_leaf) {
                    result.hit_unknown_depth_cap = true;
                    result.node = node;
                    result.intervals = oracle_->query_intervals_for_node(node,
                                                                         tree_intervals,
                                                                         seed);
                    result.fail_code = 2;
                    result.total_ms = elapsed_ms();
                    return result;
                }
                const auto split = oracle_->split_node(node,
                                                       tree_intervals,
                                                       changed_dim,
                                                       split_options);
                if (!split.split) {
                    result.fail_code = 6;
                    result.total_ms = elapsed_ms();
                    return result;
                }
                result.splits += 1;
                OracleNodeTopology split_topology = node_topology;
                split_topology.leaf = false;
                split_topology.split_dim = split.split_dim;
                split_topology.split_value = split.split_value;
                split_topology.left = split.left;
                split_topology.right = split.right;
                if (!descend_to_seed_child(split_topology)) {
                    result.fail_code = 5;
                    result.total_ms = elapsed_ms();
                    return result;
                }
                continue;
            }
            if (!descend_to_seed_child(node_topology)) {
                result.fail_code = 5;
                result.total_ms = elapsed_ms();
                return result;
            }
        }

        const int start_depth =
            std::max(0,
                     std::min(effective_max_depth,
                              std::max(options.start_depth, options.skip_to_depth)));
        auto validate_depth = [&](int depth, FindFreeBoxResult& candidate) {
            if (depth < 0 ||
                depth >= static_cast<int>(path.size()) ||
                path[static_cast<std::size_t>(depth)].node == kInvalidOracleNodeId) {
                candidate.fail_code = 5;
                return BoxValidation::Unknown;
            }
            const auto& entry = path[static_cast<std::size_t>(depth)];
            auto native_intervals = oracle_->query_intervals_for_node(entry.node,
                                                                      entry.tree_intervals,
                                                                      seed);
            candidate.node = entry.node;
            candidate.changed_dim = entry.changed_dim;
            candidate.intervals = native_intervals;
            if (!intervals_overlap_local(native_intervals, domain, 0.0)) {
                candidate.fail_code = 5;
                return BoxValidation::Unknown;
            }
            if (!intervals_subset_local(native_intervals, domain, 1e-12)) {
                candidate.hit_unknown_depth_cap = depth >= effective_max_depth;
                candidate.fail_code = 2;
                return BoxValidation::Unknown;
            }
            if (oracle_->is_reserved(entry.node)) {
                candidate.hit_reserved_depth_cap = true;
                candidate.fail_code = 2;
                return BoxValidation::Unknown;
            }
            if (depth < options.skip_to_depth) {
                candidate.hit_unknown_depth_cap = true;
                candidate.fail_code = 2;
                return BoxValidation::Unknown;
            }
            const auto validation = oracle_->validate_node(entry.node,
                                                           native_intervals,
                                                           entry.changed_dim);
            candidate.validation_detail = oracle_->last_validation_detail();
            candidate.decisions += 1;
            if (validation == BoxValidation::Free) {
                candidate.found = true;
                candidate.fail_code = 0;
            } else if (validation == BoxValidation::Occupied) {
                candidate.fail_code = 3;
            } else {
                candidate.hit_unknown_depth_cap = depth >= effective_max_depth;
                candidate.fail_code = 2;
            }
            return validation;
        };

        int lo = start_depth;
        int hi = effective_max_depth;
        FindFreeBoxResult best;
        FindFreeBoxResult high_candidate;
        const BoxValidation high_validation = validate_depth(effective_max_depth,
                                                             high_candidate);
        result.decisions += high_candidate.decisions;
        if (high_validation != BoxValidation::Free) {
            high_candidate.splits = result.splits;
            high_candidate.total_ms = elapsed_ms();
            return high_candidate;
        }
        best = high_candidate;
        while (lo < hi) {
            if (context.should_stop() ||
                (options.deadline_ms > 0.0 && elapsed_ms() > options.deadline_ms)) {
                best.deadline_reached =
                    context.deadline().expired() || options.deadline_ms > 0.0;
                best.fail_code = 4;
                break;
            }
            const int mid = lo + (hi - lo) / 2;
            FindFreeBoxResult candidate;
            const BoxValidation validation = validate_depth(mid, candidate);
            result.decisions += candidate.decisions;
            if (validation == BoxValidation::Free) {
                best = std::move(candidate);
                hi = mid;
            } else {
                lo = mid + 1;
            }
        }
        best.decisions = result.decisions;
        best.splits = result.splits;
        best.total_ms = elapsed_ms();
        if (best.found && options.record_diagnostics) {
            const double free_depth =
                static_cast<double>(oracle_->node_topology(best.node).depth);
            context.diagnostics().add_counter("ffb.free_ancestor_hits");
            context.diagnostics().add_counter("ffb.free_ancestor_depth_sum",
                                              free_depth);
            context.diagnostics().set_value(
                "ffb.free_ancestor_depth_max",
                std::max(context.diagnostics().value("ffb.free_ancestor_depth_max"),
                         free_depth));
            double free_log_volume = 0.0;
            for (const auto& interval : best.intervals) {
                const double width = std::max(0.0, interval.width());
                if (width > 0.0) {
                    free_log_volume += std::log(width);
                }
            }
            context.diagnostics().add_counter("ffb.free_ancestor_log_volume_sum",
                                              free_log_volume);
        }
        return best;
    }
    while (true) {
        if (context.should_stop() || (options.deadline_ms > 0.0 && elapsed_ms() > options.deadline_ms)) {
            result.deadline_reached = context.deadline().expired() || options.deadline_ms > 0.0;
            result.fail_code = 4;
            break;
        }

        if (!node_topology.leaf) {
            if (!descend_to_seed_child(node_topology)) {
                result.fail_code = 5;
                break;
            }
            continue;
        }

        auto native_intervals = oracle_->query_intervals_for_node(node, tree_intervals, seed);
        if (!intervals_overlap_local(native_intervals, domain, 0.0)) {
            result.node = node;
            result.intervals = std::move(native_intervals);
            result.fail_code = 5;
            break;
        }

        if (!intervals_subset_local(native_intervals, domain, 1e-12)) {
            if (node_topology.depth >= effective_max_depth) {
                result.hit_unknown_depth_cap = true;
                result.node = node;
                result.intervals = std::move(native_intervals);
                result.fail_code = 2;
                break;
            }
            const auto split = oracle_->split_node(node, tree_intervals, changed_dim, split_options);
            if (!split.split) {
                result.node = node;
                result.intervals = std::move(native_intervals);
                result.fail_code = 6;
                break;
            }
            result.splits += 1;
            OracleNodeTopology split_topology = node_topology;
            split_topology.leaf = false;
            split_topology.split_dim = split.split_dim;
            split_topology.split_value = split.split_value;
            split_topology.left = split.left;
            split_topology.right = split.right;
            if (!descend_to_seed_child(split_topology)) {
                result.fail_code = 5;
                break;
            }
            continue;
        }

        if (oracle_->is_reserved(node)) {
            if (node_topology.depth >= effective_max_depth || !options.split_reserved_leaf) {
                result.hit_reserved_depth_cap = true;
                result.node = node;
                result.intervals = std::move(native_intervals);
                result.fail_code = 2;
                break;
            }
            const auto split = oracle_->split_node(node, tree_intervals, changed_dim, split_options);
            if (!split.split) {
                result.fail_code = 6;
                break;
            }
            result.splits += 1;
            OracleNodeTopology split_topology = node_topology;
            split_topology.leaf = false;
            split_topology.split_dim = split.split_dim;
            split_topology.split_value = split.split_value;
            split_topology.left = split.left;
            split_topology.right = split.right;
            if (!descend_to_seed_child(split_topology)) {
                result.fail_code = 5;
                break;
            }
            continue;
        }

        if (node_topology.depth < std::max(options.start_depth, options.skip_to_depth)) {
            if (node_topology.depth >= effective_max_depth || !options.split_unknown_leaf) {
                result.hit_unknown_depth_cap = true;
                result.node = node;
                result.intervals = std::move(native_intervals);
                result.fail_code = 2;
                break;
            }
            const auto split = oracle_->split_node(node, tree_intervals, changed_dim, split_options);
            if (!split.split) {
                result.fail_code = 6;
                break;
            }
            result.splits += 1;
            OracleNodeTopology split_topology = node_topology;
            split_topology.leaf = false;
            split_topology.split_dim = split.split_dim;
            split_topology.split_value = split.split_value;
            split_topology.left = split.left;
            split_topology.right = split.right;
            if (!descend_to_seed_child(split_topology)) {
                result.fail_code = 5;
                break;
            }
            continue;
        }

        const auto validation = oracle_->validate_node(node, native_intervals, changed_dim);
        result.validation_detail = oracle_->last_validation_detail();
        result.decisions += 1;
        if (validation == BoxValidation::Free) {
            result.found = true;
            result.node = node;
            result.changed_dim = changed_dim;
            result.intervals = std::move(native_intervals);
            result.fail_code = 0;
            break;
        }
        if (validation == BoxValidation::Occupied) {
            result.node = node;
            result.intervals = std::move(native_intervals);
            result.fail_code = 3;
            break;
        }
        if (node_topology.depth >= effective_max_depth || !options.split_unknown_leaf) {
            result.hit_unknown_depth_cap = true;
            result.node = node;
            result.intervals = std::move(native_intervals);
            result.fail_code = 2;
            break;
        }
        const auto split = oracle_->split_node(node, tree_intervals, changed_dim, split_options);
        if (!split.split) {
            result.fail_code = 6;
            break;
        }
        result.splits += 1;
        OracleNodeTopology split_topology = node_topology;
        split_topology.leaf = false;
        split_topology.split_dim = split.split_dim;
        split_topology.split_value = split.split_value;
        split_topology.left = split.left;
        split_topology.right = split.right;
        if (!descend_to_seed_child(split_topology)) {
            result.fail_code = 5;
            break;
        }
    }
    result.total_ms = elapsed_ms();
    return result;
}

void RBFPlanningForest::populate_dynamic_collision_cache(const LeafSweepResult& result,
                                                         int obstacle_count) {
    dynamic_collision_box_cache_.clear();
    dynamic_collision_box_cache_.reserve(result.collision_boxes.size());
    std::vector<int> all_obstacles;
    all_obstacles.reserve(static_cast<std::size_t>(std::max(0, obstacle_count)));
    for (int index = 0; index < obstacle_count; ++index) {
        all_obstacles.push_back(index);
    }
    for (std::size_t index = 0; index < result.collision_boxes.size(); ++index) {
        std::vector<int> blockers = all_obstacles;
        if (index < result.collision_box_obstacle_indices.size() &&
            !result.collision_box_obstacle_indices[index].empty()) {
            blockers = result.collision_box_obstacle_indices[index];
        }
        add_dynamic_collision_cache_box(result.collision_boxes[index], std::move(blockers));
    }
}

void RBFPlanningForest::add_dynamic_collision_cache_box(const BoxNode& box,
                                                        std::vector<int> blocking_obstacle_indices) {
    blocking_obstacle_indices.erase(
        std::remove_if(blocking_obstacle_indices.begin(),
                       blocking_obstacle_indices.end(),
                       [](int index) { return index < 0; }),
        blocking_obstacle_indices.end());
    std::sort(blocking_obstacle_indices.begin(), blocking_obstacle_indices.end());
    blocking_obstacle_indices.erase(
        std::unique(blocking_obstacle_indices.begin(), blocking_obstacle_indices.end()),
        blocking_obstacle_indices.end());
    if (blocking_obstacle_indices.empty()) {
        return;
    }
    CachedCollisionBox cached;
    cached.box = box;
    cached.blocking_obstacle_indices = std::move(blocking_obstacle_indices);
    dynamic_collision_box_cache_.push_back(std::move(cached));
}

int RBFPlanningForest::promote_unblocked_collision_cache(const std::unordered_set<int>& removed_obstacle_indices,
                                                         RebuildProfile& profile) {
    profile.collision_cache_boxes_before = static_cast<int>(dynamic_collision_box_cache_.size());
    if (removed_obstacle_indices.empty() || dynamic_collision_box_cache_.empty()) {
        profile.collision_cache_boxes_after = static_cast<int>(dynamic_collision_box_cache_.size());
        return 0;
    }

    auto remap_obstacle_index = [&](int old_index, int& new_index) {
        if (removed_obstacle_indices.find(old_index) != removed_obstacle_indices.end()) {
            return false;
        }
        int shift = 0;
        for (int removed : removed_obstacle_indices) {
            if (removed < old_index) {
                shift += 1;
            }
        }
        new_index = old_index - shift;
        return new_index >= 0 && new_index < scene_.n_obstacles();
    };

    std::vector<CachedCollisionBox> retained;
    retained.reserve(dynamic_collision_box_cache_.size());
    int promoted = 0;
    int next_id = next_box_id();
    const double adjacency_tolerance = config_.query.adjacency_tolerance;
    const auto cache_scan_t0 = std::chrono::steady_clock::now();
    for (auto& cached : dynamic_collision_box_cache_) {
        profile.diagnostics["delete.cache_entries_scanned"] += 1.0;
        bool touched = false;
        std::vector<int> remaining_blockers;
        remaining_blockers.reserve(cached.blocking_obstacle_indices.size());
        for (int old_index : cached.blocking_obstacle_indices) {
            profile.diagnostics["delete.blocker_index_checks"] += 1.0;
            if (removed_obstacle_indices.find(old_index) != removed_obstacle_indices.end()) {
                touched = true;
                continue;
            }
            int new_index = -1;
            if (remap_obstacle_index(old_index, new_index)) {
                remaining_blockers.push_back(new_index);
            }
        }
        std::sort(remaining_blockers.begin(), remaining_blockers.end());
        remaining_blockers.erase(std::unique(remaining_blockers.begin(), remaining_blockers.end()),
                                 remaining_blockers.end());

        if (!touched) {
            cached.blocking_obstacle_indices = std::move(remaining_blockers);
            retained.push_back(std::move(cached));
            continue;
        }
        profile.collision_cache_candidates += 1;
        if (!remaining_blockers.empty()) {
            cached.blocking_obstacle_indices = std::move(remaining_blockers);
            retained.push_back(std::move(cached));
            continue;
        }

        BoxNode box = cached.box;
        box.id = next_id;
        bool contained = false;
        const auto contained_t0 = std::chrono::steady_clock::now();
        for (const auto& existing : boxes_) {
            profile.diagnostics["delete.containment_checks"] += 1.0;
            if (box_contains_box_exact_local(existing, box)) {
                contained = true;
                break;
            }
        }
        profile.diagnostics["delete.containment_check_ms"] +=
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - contained_t0).count();
        if (contained) {
            profile.collision_cache_rejected_contained += 1;
            continue;
        }
        int adjacent_parent = -1;
        const auto adjacency_t0 = std::chrono::steady_clock::now();
        if (!boxes_.empty() && !leaf_refine_has_adjacency(boxes_, box, adjacency_tolerance, &adjacent_parent)) {
            profile.diagnostics["delete.adjacency_check_ms"] +=
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - adjacency_t0).count();
            profile.collision_cache_rejected_disconnected += 1;
            cached.blocking_obstacle_indices.clear();
            retained.push_back(std::move(cached));
            continue;
        }
        profile.diagnostics["delete.adjacency_check_ms"] +=
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - adjacency_t0).count();
        profile.diagnostics["delete.adjacency_checks"] += 1.0;
        box.parent_box_id = adjacent_parent;
        const BoxNode* parent = find_box_by_id(boxes_, adjacent_parent);
        box.root_id = parent != nullptr && parent->root_id >= 0 ? parent->root_id : box.id;
        box.safety_status = BoxSafetyStatus::CertifiedFree;
        box.strict_audit_required = false;
        box.compute_volume();
        if (oracle_ && box.tree_id >= 0) {
            oracle_->reserve_node(box.tree_id, box.id);
        }
        boxes_.push_back(box);
        raw_boxes_.push_back(box);
        next_id += 1;
        promoted += 1;
        profile.collision_cache_promoted += 1;
        profile.boxes_added += 1;
        profile.raw_boxes_added += 1;
    }
    profile.diagnostics["delete.cache_scan_ms"] +=
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - cache_scan_t0).count();
    dynamic_collision_box_cache_ = std::move(retained);
    profile.collision_cache_boxes_after = static_cast<int>(dynamic_collision_box_cache_.size());
    return promoted;
}

int RBFPlanningForest::refill_removed_box_with_leaf_sweep(const BoxNode& removed_box,
                                                          int new_obstacle_index,
                                                          int max_depth,
                                                          int& next_id,
                                                          RebuildProfile& profile) {
    if (!oracle_ || removed_box.tree_id < 0 || removed_box.joint_intervals.empty()) {
        return 0;
    }
    const int effective_max_depth = std::max(0, std::min(max_depth, oracle_->max_tree_depth() - 1));
    struct Item {
        OracleNodeId node = kInvalidOracleNodeId;
        int changed_dim = -1;
    };
    std::vector<Item> stack;
    stack.push_back(Item{removed_box.tree_id, -1});
    int added = 0;
    const OracleSplitOptions split_options = config_.grower.find_free_box.split;
    const Eigen::VectorXd removed_reference = removed_box.center();
    const int max_stack_pops = std::max(64, 4 * (1 << std::min(effective_max_depth + 1, 12)));
    int stack_pops = 0;

    auto cache_collision_leaf = [&](OracleNodeId node, const std::vector<Interval>& intervals) {
        profile.diagnostics["insert.refill_cached_collision_leaves"] += 1.0;
        BoxNode cached_box;
        cached_box.id = -1;
        cached_box.joint_intervals = intervals;
        cached_box.seed_config = cached_box.center();
        cached_box.tree_id = node;
        cached_box.parent_box_id = removed_box.parent_box_id;
        cached_box.root_id = removed_box.root_id;
        cached_box.safety_status = BoxSafetyStatus::Unknown;
        cached_box.strict_audit_required = true;
        cached_box.compute_volume();
        add_dynamic_collision_cache_box(cached_box, {new_obstacle_index});
    };

    while (!stack.empty()) {
        profile.diagnostics["insert.refill_stack_pops"] += 1.0;
        if (++stack_pops > max_stack_pops) {
            profile.diagnostics["insert.refill_stack_pop_cap_hits"] += 1.0;
            break;
        }
        const Item item = stack.back();
        stack.pop_back();
        if (item.node < 0) {
            continue;
        }
        std::vector<Interval> tree_intervals = oracle_->node_intervals(item.node);
        std::vector<Interval> intervals;
        bool found_matching_native_copy = false;
        for (auto candidate : oracle_->native_interval_copies_for_node(item.node, tree_intervals)) {
            if (intervals_subset_local(candidate, removed_box.joint_intervals, 1e-12)) {
                intervals = std::move(candidate);
                found_matching_native_copy = true;
                break;
            }
        }
        if (!found_matching_native_copy) {
            continue;
        }
        if (oracle_->is_reserved(item.node)) {
            profile.diagnostics["insert.refill_reserved_skips"] += 1.0;
            continue;
        }
        profile.regrow_attempts += 1;
        const auto validate_t0 = std::chrono::steady_clock::now();
        const BoxValidation validation = oracle_->validate_node(item.node, intervals, item.changed_dim);
        profile.diagnostics["insert.refill_validate_node_calls"] += 1.0;
        profile.diagnostics["insert.refill_validate_node_ms"] +=
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - validate_t0).count();
        OracleValidationDetail detail = oracle_->last_validation_detail();
        if (validation == BoxValidation::Free) {
            profile.diagnostics["insert.refill_free_leaves"] += 1.0;
            BoxNode box;
            box.id = next_id++;
            box.joint_intervals = std::move(intervals);
            box.seed_config = box.center();
            box.tree_id = item.node;
            box.parent_box_id = removed_box.parent_box_id;
            box.root_id = removed_box.root_id >= 0 ? removed_box.root_id : box.id;
            box.safety_status = detail.safety_status;
            box.strict_audit_required = detail.strict_audit_required;
            box.compute_volume();

            FindFreeBoxResult commit_probe;
            commit_probe.found = true;
            commit_probe.node = item.node;
            commit_probe.intervals = box.joint_intervals;
            commit_probe.validation_detail = detail;
            if (!allow_dynamic_commit(*oracle_, commit_probe, config_.grower.commit_policy)) {
                profile.diagnostics["insert.refill_commit_rejects"] += 1.0;
                continue;
            }
            bool contained = false;
            const auto contained_t0 = std::chrono::steady_clock::now();
            for (const auto& existing : boxes_) {
                profile.diagnostics["insert.refill_containment_checks"] += 1.0;
                if (box_contains_box_exact_local(existing, box)) {
                    contained = true;
                    break;
                }
            }
            profile.diagnostics["insert.refill_containment_check_ms"] +=
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - contained_t0).count();
            if (contained) {
                profile.collision_cache_rejected_contained += 1;
                profile.diagnostics["insert.refill_contained_rejects"] += 1.0;
                continue;
            }
            oracle_->reserve_node(box.tree_id, box.id);
            boxes_.push_back(box);
            raw_boxes_.push_back(std::move(box));
            profile.boxes_added += 1;
            profile.raw_boxes_added += 1;
            added += 1;
            continue;
        }

        if (oracle_->depth(item.node) >= effective_max_depth) {
            profile.diagnostics["insert.refill_depth_cap_hits"] += 1.0;
            cache_collision_leaf(item.node, intervals);
            continue;
        }
        const auto split_t0 = std::chrono::steady_clock::now();
        const auto split = oracle_->split_node(item.node, tree_intervals, item.changed_dim, split_options);
        profile.diagnostics["insert.refill_split_node_calls"] += 1.0;
        profile.diagnostics["insert.refill_split_node_ms"] +=
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - split_t0).count();
        if (!split.split) {
            profile.diagnostics["insert.refill_split_failures"] += 1.0;
            cache_collision_leaf(item.node, intervals);
            continue;
        }
        profile.diagnostics["insert.refill_split_success"] += 1.0;
        stack.push_back(Item{split.right, split.split_dim});
        stack.push_back(Item{split.left, split.split_dim});
    }
    return added;
}

BuildProfile RBFPlanningForest::build_subtractive(
    const std::vector<SubtractiveObstacleGroup>& obstacle_groups,
    const std::vector<Eigen::VectorXd>& seeds,
    const SubtractiveBuildOptions& options) {
    using Clock = std::chrono::steady_clock;
    const auto t0 = Clock::now();

    BuildProfile profile;
    const auto bootstrap_t0 = Clock::now();
    if (!seeds.empty()) {
        StageContext bootstrap_context = StageContext::from_runtime(config_.runtime);
        const BuildProfile bootstrap = build_coverage({}, seeds, bootstrap_context);
        profile.diagnostics["subtractive.bootstrap_boxes"] = static_cast<double>(bootstrap.final_boxes);
        profile.diagnostics["subtractive.bootstrap_raw_boxes"] = static_cast<double>(bootstrap.raw_boxes);
        profile.diagnostics["subtractive.bootstrap_segment_edges"] = static_cast<double>(bootstrap.segment_edges);
    } else {
        last_build_seeds_ = seeds;
        scene_.clear();
        boxes_.clear();
        raw_boxes_.clear();
        adjacency_.clear();
        segment_edges_.clear();
        dynamic_collision_box_cache_.clear();
        invalidate_query_cache();
        reset_oracle(Scene{});
        profile.diagnostics["subtractive.bootstrap_boxes"] = 0.0;
        profile.diagnostics["subtractive.bootstrap_raw_boxes"] = 0.0;
        profile.diagnostics["subtractive.bootstrap_segment_edges"] = 0.0;
    }
    const double bootstrap_ms = std::chrono::duration<double, std::milli>(Clock::now() - bootstrap_t0).count();
    profile.diagnostics["subtractive.bootstrap_ms"] = bootstrap_ms;
    profile.diagnostics["subtractive.initial_leaf_boxes"] = static_cast<double>(boxes_.size());
    rebuild_adjacency();

    std::vector<Obstacle> validation_obstacles;
    std::vector<Obstacle> carving_obstacles;
    int groups_with_validation = 0;
    int carving_insertions = 0;
    int groups_with_collisions = 0;
    int boxes_removed = 0;
    int boxes_added = 0;
    int regrow_seeds = 0;
    int regrow_found_failures = 0;
    int regrow_commit_rejects = 0;
    int regrow_domain_rejects = 0;
    double carve_collision_ms = 0.0;
    double carve_regrow_ms = 0.0;
    double carve_local_adjacency_ms = 0.0;
    double carve_global_adjacency_ms = 0.0;
    int next_subtractive_id = next_box_id();
    const double adjacency_tolerance = config_.query.adjacency_tolerance;
    const double boundary_epsilon = std::max(1e-10, 2.0 * adjacency_tolerance);

    for (const auto& group : obstacle_groups) {
        const auto& group_validation = group.validation_obstacles.empty()
            ? group.carving_obstacles
            : group.validation_obstacles;
        if (!group_validation.empty()) {
            groups_with_validation += 1;
        }
        validation_obstacles.insert(validation_obstacles.end(),
                                    group_validation.begin(),
                                    group_validation.end());
        carving_obstacles.insert(carving_obstacles.end(),
                                 group.carving_obstacles.begin(),
                                 group.carving_obstacles.end());
        carving_insertions += static_cast<int>(group.carving_obstacles.size());
        if (group.carving_obstacles.empty()) {
            continue;
        }

        scene_.set_obstacles(carving_obstacles);
        reset_oracle(scene_);
        reserve_existing_boxes();
        CollisionChecker carving_checker(robot_, scene_);
        const AdjacencyGraph previous_adjacency = adjacency_;

        const auto collision_t0 = Clock::now();
        std::vector<BoxNode> removed_boxes;
        removed_boxes.reserve(boxes_.size());
        std::unordered_set<int> removed_box_ids;
        for (const auto& box : boxes_) {
            if (carving_checker.check_box(box.joint_intervals)) {
                removed_box_ids.insert(box.id);
                removed_boxes.push_back(box);
            }
        }
        carve_collision_ms += std::chrono::duration<double, std::milli>(Clock::now() - collision_t0).count();
        if (removed_boxes.empty()) {
            continue;
        }
        groups_with_collisions += 1;
        boxes_removed += static_cast<int>(removed_boxes.size());

        for (const auto& box : removed_boxes) {
            if (oracle_) {
                oracle_->release_box(box.id);
            }
        }
        boxes_.erase(std::remove_if(boxes_.begin(), boxes_.end(), [&](const BoxNode& box) {
            return removed_box_ids.find(box.id) != removed_box_ids.end();
        }), boxes_.end());
        raw_boxes_.erase(std::remove_if(raw_boxes_.begin(), raw_boxes_.end(), [&](const BoxNode& box) {
            return removed_box_ids.find(box.id) != removed_box_ids.end();
        }), raw_boxes_.end());
        segment_edges_.erase(std::remove_if(segment_edges_.begin(), segment_edges_.end(), [&](const SegmentEdge& edge) {
            if (removed_box_ids.find(edge.source_box_id) != removed_box_ids.end() ||
                removed_box_ids.find(edge.target_box_id) != removed_box_ids.end()) {
                return true;
            }
            return !segment_edge_survives_scene(
                edge, carving_checker, config_.query.audit_resolution, config_.query.audit_segment_step);
        }), segment_edges_.end());

        const auto local_adj_t0 = Clock::now();
        std::unordered_set<int> local_adjacency_ids = collect_local_adjacency_ids(boxes_, removed_boxes, boundary_epsilon);
        local_adjacency_ids.insert(removed_box_ids.begin(), removed_box_ids.end());
        rebuild_local_adjacency(adjacency_, boxes_, local_adjacency_ids, adjacency_tolerance);
        carve_local_adjacency_ms += std::chrono::duration<double, std::milli>(Clock::now() - local_adj_t0).count();

        const auto regrow_t0 = Clock::now();
        std::vector<Eigen::VectorXd> anchors = seeds;
        anchors.insert(anchors.end(), last_build_seeds_.begin(), last_build_seeds_.end());
        const auto local_seeds = make_subtractive_regrow_seeds(boxes_,
                                                               removed_boxes,
                                                               removed_box_ids,
                                                               previous_adjacency,
                                                               anchors,
                                                               config_.dynamic_update,
                                                               adjacency_tolerance,
                                                               boundary_epsilon);
        regrow_seeds += static_cast<int>(local_seeds.size());
        StageContext regrow_context = StageContext::from_runtime(config_.runtime);
        FindFreeBoxOptions regrow_options = config_.grower.find_free_box;
        regrow_options.reject_seed_collision = true;
        for (const auto& candidate : local_seeds) {
            if (candidate.domain_index < 0 || candidate.domain_index >= static_cast<int>(removed_boxes.size())) {
                regrow_domain_rejects += 1;
                continue;
            }
            const BoxNode& domain = removed_boxes[static_cast<std::size_t>(candidate.domain_index)];
            auto result = find_free_box_in_domain(candidate.seed,
                                                  domain.joint_intervals,
                                                  regrow_context,
                                                  regrow_options);
            if (!result.found) {
                regrow_found_failures += 1;
                continue;
            }
            if (!intervals_subset_local(result.intervals, domain.joint_intervals, 1e-12) ||
                containing_domain_index(removed_boxes, result.intervals, 1e-12) < 0) {
                regrow_domain_rejects += 1;
                continue;
            }
            if (carving_checker.check_box(result.intervals)) {
                regrow_commit_rejects += 1;
                continue;
            }
            if (!allow_dynamic_commit(*oracle_, result, config_.grower.commit_policy)) {
                regrow_commit_rejects += 1;
                continue;
            }
            BoxNode box;
            box.id = next_subtractive_id++;
            box.joint_intervals = result.intervals;
            box.seed_config = candidate.seed;
            box.tree_id = result.node;
            box.parent_box_id = candidate.parent_box_id;
            box.root_id = candidate.root_id >= 0 ? candidate.root_id : box.id;
            box.safety_status = result.validation_detail.safety_status;
            box.strict_audit_required = result.validation_detail.strict_audit_required;
            box.compute_volume();
            bool contained_by_existing = false;
            for (const auto& existing : boxes_) {
                if (box_contains_box_exact_local(existing, box)) {
                    contained_by_existing = true;
                    break;
                }
            }
            if (contained_by_existing) {
                regrow_commit_rejects += 1;
                continue;
            }
            oracle_->reserve_node(box.tree_id, box.id);
            local_adjacency_ids.insert(box.id);
            boxes_.push_back(box);
            raw_boxes_.push_back(std::move(box));
            boxes_added += 1;
        }
        carve_regrow_ms += std::chrono::duration<double, std::milli>(Clock::now() - regrow_t0).count();

        const auto local_after_t0 = Clock::now();
        const auto expanded_local_ids = collect_local_adjacency_ids(boxes_, removed_boxes, boundary_epsilon);
        local_adjacency_ids.insert(expanded_local_ids.begin(), expanded_local_ids.end());
        rebuild_local_adjacency(adjacency_, boxes_, local_adjacency_ids, adjacency_tolerance);
        carve_local_adjacency_ms += std::chrono::duration<double, std::milli>(Clock::now() - local_after_t0).count();

        const auto group_global_adj_t0 = Clock::now();
        rebuild_adjacency();
        carve_global_adjacency_ms += std::chrono::duration<double, std::milli>(Clock::now() - group_global_adj_t0).count();
    }

    profile.diagnostics["subtractive.groups"] = static_cast<double>(obstacle_groups.size());
    profile.diagnostics["subtractive.groups_with_validation_obstacles"] = static_cast<double>(groups_with_validation);
    profile.diagnostics["subtractive.groups_with_collisions"] = static_cast<double>(groups_with_collisions);
    profile.diagnostics["subtractive.carving_obstacles"] = static_cast<double>(carving_obstacles.size());
    profile.diagnostics["subtractive.carving_insertions"] = static_cast<double>(carving_insertions);
    profile.diagnostics["subtractive.carve_boxes_removed"] = static_cast<double>(boxes_removed);
    profile.diagnostics["subtractive.carve_boxes_added"] = static_cast<double>(boxes_added);
    profile.diagnostics["subtractive.regrow_seeds"] = static_cast<double>(regrow_seeds);
    profile.diagnostics["subtractive.regrow_found_failures"] = static_cast<double>(regrow_found_failures);
    profile.diagnostics["subtractive.regrow_commit_rejects"] = static_cast<double>(regrow_commit_rejects);
    profile.diagnostics["subtractive.regrow_domain_rejects"] = static_cast<double>(regrow_domain_rejects);
    profile.diagnostics["subtractive.carve_collision_ms"] = carve_collision_ms;
    profile.diagnostics["subtractive.carve_regrow_ms"] = carve_regrow_ms;
    profile.diagnostics["subtractive.carve_local_adjacency_ms"] = carve_local_adjacency_ms;
    profile.diagnostics["subtractive.carve_global_adjacency_ms"] = carve_global_adjacency_ms;

    std::vector<Obstacle> final_obstacles = options.use_validation_obstacles_for_final_scene
        ? validation_obstacles
        : carving_obstacles;
    if (final_obstacles.empty() && !scene_.empty()) {
        final_obstacles = scene_.obstacles();
    }

    int final_pruned_boxes = 0;
    if (options.use_validation_obstacles_for_final_scene) {
        Scene validation_scene(final_obstacles);
        CollisionChecker validation_checker(robot_, validation_scene);
        std::unordered_set<int> removed_box_ids;
        for (const auto& box : boxes_) {
            if (validation_checker.check_box(box.joint_intervals)) {
                removed_box_ids.insert(box.id);
            }
        }
        if (!removed_box_ids.empty()) {
            for (std::size_t i = 0; i < boxes_.size();) {
                if (removed_box_ids.find(boxes_[i].id) != removed_box_ids.end()) {
                    if (oracle_) {
                        oracle_->release_box(boxes_[i].id);
                    }
                    boxes_.erase(boxes_.begin() + static_cast<std::ptrdiff_t>(i));
                    final_pruned_boxes += 1;
                } else {
                    ++i;
                }
            }
            for (std::size_t i = 0; i < raw_boxes_.size();) {
                if (removed_box_ids.find(raw_boxes_[i].id) != removed_box_ids.end() ||
                    validation_checker.check_box(raw_boxes_[i].joint_intervals)) {
                    raw_boxes_.erase(raw_boxes_.begin() + static_cast<std::ptrdiff_t>(i));
                } else {
                    ++i;
                }
            }
        }
        std::unordered_set<int> live_box_ids;
        live_box_ids.reserve(boxes_.size());
        for (const auto& box : boxes_) {
            live_box_ids.insert(box.id);
        }
        segment_edges_.erase(std::remove_if(segment_edges_.begin(), segment_edges_.end(), [&](const SegmentEdge& edge) {
            if (live_box_ids.find(edge.source_box_id) == live_box_ids.end() ||
                live_box_ids.find(edge.target_box_id) == live_box_ids.end()) {
                return true;
            }
            return !segment_edge_survives_scene(
                edge, validation_checker, config_.query.audit_resolution, config_.query.audit_segment_step);
        }), segment_edges_.end());
        scene_.set_obstacles(std::move(final_obstacles));
        reset_oracle(scene_);
        reserve_existing_boxes();
        rebuild_adjacency();
    }
    profile.diagnostics["subtractive.final_validation_obstacles"] = static_cast<double>(scene_.n_obstacles());
    profile.diagnostics["subtractive.final_pruned_boxes"] = static_cast<double>(final_pruned_boxes);

    const auto connector_t0 = Clock::now();
    bool connector_ran = false;
    if (options.run_connector && config_.enable_connector && !boxes_.empty()) {
        StageContext context = StageContext::from_runtime(config_.runtime);
        CollisionChecker checker(robot_, scene_);
        IslandConnectorConfig connector_config = config_.connector;
        if (connector_config.n_threads <= 1 && config_.runtime.n_threads > 1) {
            connector_config.n_threads = config_.runtime.n_threads;
        }
        IslandConnector connector(*oracle_, robot_, checker, connector_config);
        int next_id = next_box_id();
        const auto connector_result = connector.connect_all(boxes_, adjacency_, segment_edges_, next_id, context);
        profile.bridge_boxes_added = connector_result.bridge_boxes_added;
        profile.segment_edges_added = connector_result.segment_edges_added;
        profile.rrt_segment_edges_added = connector_result.rrt_segment_edges_added;
        profile.point_gap_segment_edges_added = connector_result.point_gap_segment_edges_added;
        profile.connector_attempted_pairs = connector_result.attempted_pairs;
        profile.connector_connected = connector_result.connected;
        connector_ran = true;
    }
    profile.connector_ms = std::chrono::duration<double, std::milli>(Clock::now() - connector_t0).count();
    profile.diagnostics["subtractive.connector_ran"] = connector_ran ? 1.0 : 0.0;

    const auto adj_t0 = Clock::now();
    rebuild_adjacency();
    profile.adjacency_ms = carve_local_adjacency_ms + carve_global_adjacency_ms +
        std::chrono::duration<double, std::milli>(Clock::now() - adj_t0).count();
    profile.grow_ms = bootstrap_ms + carve_collision_ms + carve_regrow_ms;
    profile.raw_boxes = static_cast<int>(raw_boxes_.size());
    profile.final_boxes = static_cast<int>(boxes_.size());
    profile.segment_edges = static_cast<int>(segment_edges_.size());
    profile.adjacency_islands = static_cast<int>(find_islands(adjacency_).size());
    profile.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
    record_portal_membership_policy(profile.diagnostics, config_.portal_membership_policy);
    last_build_ = profile;
    invalidate_query_cache();

    if (config_.database.checkpoint_after_build && database_) {
        database_->checkpoint();
    }
    return last_build_;
}

QueryResult RBFPlanningForest::query(const Eigen::Ref<const Eigen::VectorXd>& start,
                                 const Eigen::Ref<const Eigen::VectorXd>& goal) const {
    return run_query_internal(start, goal, true);
}

QueryResult RBFPlanningForest::run_query_internal(const Eigen::Ref<const Eigen::VectorXd>& start,
                                              const Eigen::Ref<const Eigen::VectorXd>& goal,
                                              bool allow_collision_shortcut) const {
    using Clock = std::chrono::steady_clock;
    QueryConfig query_config = config_.query;
    if (!allow_collision_shortcut) {
        query_config.collision_shortcut = false;
    }
    const bool do_collision_shortcut = query_config.collision_shortcut;
    const int active_query_index = env_int_or_default("RBF_ACTIVE_QUERY_INDEX", -1);
    const bool partition_last_query_cache_enabled =
        partition_native_mode() &&
        adaptive_partition_query_enabled_ &&
        adaptive_partition_ &&
        env_int_or_default("RBF_PARTITION_LAST_QUERY_CACHE", 0) != 0;
    auto same_vector = [](const Eigen::VectorXd& lhs,
                          const Eigen::Ref<const Eigen::VectorXd>& rhs) {
        return lhs.size() == rhs.size() &&
               (lhs.size() == 0 || (lhs - rhs).cwiseAbs().maxCoeff() <= 0.0);
    };
    if (partition_last_query_cache_enabled &&
        partition_last_query_cache_.valid &&
        partition_last_query_cache_.allow_collision_shortcut == allow_collision_shortcut &&
        partition_last_query_cache_.active_query_index == active_query_index &&
        same_vector(partition_last_query_cache_.start, start) &&
        same_vector(partition_last_query_cache_.goal, goal)) {
        QueryResult cached = partition_last_query_cache_.result;
        cached.query_time_ms = 0.0;
        cached.partition_search_ms = 0.0;
        cached.audit_time_ms = 0.0;
        cached.final_simplify_time_ms = 0.0;
        return cached;
    }
    QueryResult result;
    QueryResult partition_attempt;
    if (adaptive_partition_query_enabled_ &&
        adaptive_partition_ &&
        !adaptive_partition_->empty()) {
        AdaptiveGridPartitionQueryOptions partition_options;
        partition_options.nearest_if_outside = query_config.nearest_if_outside;
        partition_options.shortcut_boxes = query_config.shortcut_boxes;
        partition_options.max_expansions = last_build_.diagnostics.count("adaptive.grid_planning_max_expansions") > 0
            ? static_cast<int>(last_build_.diagnostics.at("adaptive.grid_planning_max_expansions"))
            : 0;
        partition_options.adjacency_tolerance = query_config.adjacency_tolerance;
        const auto partition_result = adaptive_partition_->query(start, goal, partition_options);
        result.start_box_id = partition_result.start_box_id;
        result.goal_box_id = partition_result.goal_box_id;
        result.partition_search_ms = partition_result.search_ms;
        result.query_time_ms = partition_result.search_ms;
        result.non_grid_cells_used = partition_result.non_grid_cells_used;
        if (partition_result.found) {
            result.success = true;
            result.box_sequence = partition_result.box_sequence;
            result.segment_edge_sequence = partition_result.segment_edge_sequence;
            if (result.segment_edge_sequence.size() + 1 != result.box_sequence.size()) {
                result.segment_edge_sequence.assign(
                    result.box_sequence.size() > 0 ? result.box_sequence.size() - 1 : 0,
                    -1);
            }
            result.partition_cells_used = static_cast<int>(result.box_sequence.size());
            result.path = partition_result.path;
            if (result.path.empty()) {
                if (partition_native_mode()) {
                    throw std::runtime_error(
                        "partition_native query returned a box sequence without partition waypoints");
                }
                result.path = extract_partition_waypoints_local(result.box_sequence,
                                                                result.segment_edge_sequence,
                                                                boxes_,
                                                                segment_edges_,
                                                                start,
                                                                goal,
                                                                query_config.adjacency_tolerance);
            }
            result.path_length = path_length(result.path);
            result.raw_path_length = result.path_length;
        }
        partition_attempt = result;
    }
    if (!result.success && !partition_native_mode()) {
        CorridorQuery query_engine(query_config);
        QueryResult graph_result = query_engine.run(query_cache(), start, goal);
        graph_result.partition_search_ms = partition_attempt.partition_search_ms;
        graph_result.partition_repair_ms = partition_attempt.partition_repair_ms;
        graph_result.partition_cells_used = partition_attempt.partition_cells_used;
        graph_result.non_grid_cells_used = partition_attempt.non_grid_cells_used;
        result = std::move(graph_result);
    }
    if (result.success && do_collision_shortcut && !query_config.strict_path_audit && result.path.size() > 2) {
        CollisionChecker checker = make_audit_checker(audit_robot_, scene_, query_config);
        result.path = collision_shortcut_path(result.path,
                                             checker,
                                             collision_shortcut_resolution(query_config));
        result.path_length = path_length(result.path);
    }
    summarize_query_path(result, boxes_, segment_edges_);
    if (!result.success && query_config.strict_path_audit && query_config.repair_on_audit_failure) {
        CollisionChecker checker = make_audit_checker(audit_robot_, scene_, query_config);
        const auto repair_t0 = Clock::now();
        RRTConnectConfig repair_config = config_.connector.rrt;
        repair_config.max_iters = std::max(repair_config.max_iters, query_config.repair_rrt_max_iters);
        if (query_config.repair_timeout_ms > 0.0) {
            repair_config.timeout_ms = query_config.repair_timeout_ms;
        }
        repair_config.segment_resolution = std::max(repair_config.segment_resolution, query_config.audit_resolution);
        std::vector<Eigen::VectorXd> repair_path = rrt_connect(
            start,
            goal,
            checker,
            audit_robot_,
            repair_config,
            derived_planner_seed(config_.grower.rng_seed, kSeedRepairGlobalOffset));
        if (!repair_path.empty()) {
            PathAuditCheck repair_audit = audit_waypoint_path(repair_path,
                                                             checker,
                                                             query_config.audit_resolution,
                                                             query_config.audit_segment_step);
            if (repair_audit.passed && do_collision_shortcut && repair_path.size() > 2) {
                std::vector<Eigen::VectorXd> shortened = collision_shortcut_path(
                    repair_path,
                    checker,
                    collision_shortcut_resolution(query_config));
                PathAuditCheck shortened_audit = audit_waypoint_path(shortened,
                                                                     checker,
                                                                     query_config.audit_resolution,
                                                                     query_config.audit_segment_step);
                if (shortened_audit.passed && path_length(shortened) <= path_length(repair_path) + 1e-12) {
                    repair_path = std::move(shortened);
                    repair_audit = shortened_audit;
                }
            }
            if (repair_audit.passed) {
                result.success = true;
                result.path = std::move(repair_path);
                result.path_length = path_length(result.path);
                result.raw_path_length = result.path_length;
                result.repair_count += 1;
                result.audit_status = PathAuditStatus::Repaired;
                result.audit_passed = true;
                result.failed_segment_index = repair_audit.failed_segment_index;
                result.remaining_unsafe_assumptions = 0;
            }
        }
        result.repair_time_ms += std::chrono::duration<double, std::milli>(Clock::now() - repair_t0).count();
    }
    if (result.success && query_config.strict_path_audit) {
        CollisionChecker checker = make_audit_checker(audit_robot_, scene_, query_config);
        auto try_final_simplify = [&]() {
            if (!query_config.final_rrt_simplify ||
                !(query_config.final_rrt_simplify_timeout_ms > 0.0) ||
                result.path_length <= 0.0) {
                return;
            }
            const auto simplify_t0 = Clock::now();
            auto simplify_elapsed_ms = [&]() {
                return std::chrono::duration<double, std::milli>(Clock::now() - simplify_t0).count();
            };
            RRTConnectConfig simplify_config = config_.connector.rrt;
            // Final OMPL-style simplification is audited in native C-space and is
            // intentionally not restricted to one active LECT root sector. The
            // raw box/segment statistics are computed before this replacement.
            simplify_config.max_iters = std::max(1, query_config.final_rrt_simplify_max_iters);
            simplify_config.segment_resolution = std::max(simplify_config.segment_resolution,
                                                          query_config.audit_resolution);
            simplify_config.segment_step = query_config.audit_segment_step;
            simplify_config.shortcut_path = true;
            const int attempts = std::max(1, query_config.final_rrt_simplify_attempts);
            for (int attempt = 0; attempt < attempts; ++attempt) {
                const double remaining_ms = query_config.final_rrt_simplify_timeout_ms - simplify_elapsed_ms();
                if (remaining_ms <= 0.0) {
                    break;
                }
                const int attempts_left = attempts - attempt;
                simplify_config.timeout_ms = std::max(1.0, remaining_ms / static_cast<double>(attempts_left));
                std::vector<Eigen::VectorXd> simplified = rrt_connect(start,
                                                                      goal,
                                                                      checker,
                                                                      audit_robot_,
                                                                      simplify_config,
                                                                      derived_planner_seed(config_.grower.rng_seed,
                                                                                           kSeedFinalSimplifyOffset,
                                                                                           attempt));
                if (!simplified.empty()) {
                    PathAuditCheck simplified_audit = audit_waypoint_path(simplified,
                                                                          checker,
                                                                          query_config.audit_resolution,
                                                                          query_config.audit_segment_step);
                    const double simplified_length = path_length(simplified);
                    if (simplified_audit.passed &&
                        simplified_length + 1e-12 < result.path_length) {
                        result.path = std::move(simplified);
                        result.path_length = simplified_length;
                        result.failed_segment_index = simplified_audit.failed_segment_index;
                        result.audit_passed = true;
                        result.audit_status = result.repair_count > 0 ? PathAuditStatus::Repaired : PathAuditStatus::Passed;
                    }
                }
            }
            const double simplify_ms =
                std::chrono::duration<double, std::milli>(Clock::now() - simplify_t0).count();
            result.final_simplify_time_ms += simplify_ms;
        };
        const auto audit_t0 = Clock::now();
        PathAuditCheck audit = audit_waypoint_path(result.path,
                                                   checker,
                                                   query_config.audit_resolution,
                                                   query_config.audit_segment_step);
        result.failed_segment_index = audit.failed_segment_index;
        if (audit.passed) {
            if (do_collision_shortcut && result.path.size() > 2) {
                std::vector<Eigen::VectorXd> shortened = collision_shortcut_path(
                    result.path,
                    checker,
                    collision_shortcut_resolution(query_config));
                PathAuditCheck shortened_audit = audit_waypoint_path(shortened,
                                                                     checker,
                                                                     query_config.audit_resolution,
                                                                     query_config.audit_segment_step);
                if (shortened_audit.passed && path_length(shortened) <= result.path_length + 1e-12) {
                    result.path = std::move(shortened);
                    result.path_length = path_length(result.path);
                    audit = shortened_audit;
                    result.failed_segment_index = audit.failed_segment_index;
                }
            }
            result.audit_status = result.repair_count > 0 ? PathAuditStatus::Repaired : PathAuditStatus::Passed;
            result.audit_passed = true;
            result.remaining_unsafe_assumptions = 0;
            try_final_simplify();
        } else if (query_config.repair_on_audit_failure) {
            const auto repair_t0 = Clock::now();
            const RRTConnectConfig repair_domain_config = oracle_
                ? with_query_root_hull_domain(config_.connector.rrt, *oracle_, start, goal)
                : config_.connector.rrt;
            const bool repaired = try_local_birrt_repair(result,
                                                         audit,
                                                         checker,
                                                         audit_robot_,
                                                         query_config,
                                                         repair_domain_config,
                                                         config_.grower.rng_seed);
            result.repair_time_ms = std::chrono::duration<double, std::milli>(Clock::now() - repair_t0).count();
            if (repaired) {
                PathAuditCheck repaired_audit = audit_waypoint_path(result.path,
                                                                    checker,
                                                                    query_config.audit_resolution,
                                                                    query_config.audit_segment_step);
                if (repaired_audit.passed && do_collision_shortcut && result.path.size() > 2) {
                    std::vector<Eigen::VectorXd> shortened = collision_shortcut_path(
                        result.path,
                        checker,
                        collision_shortcut_resolution(query_config));
                    PathAuditCheck shortened_audit = audit_waypoint_path(shortened,
                                                                         checker,
                                                                         query_config.audit_resolution,
                                                                         query_config.audit_segment_step);
                    if (shortened_audit.passed && path_length(shortened) <= result.path_length + 1e-12) {
                        result.path = std::move(shortened);
                        result.path_length = path_length(result.path);
                        repaired_audit = shortened_audit;
                    }
                }
                result.failed_segment_index = repaired_audit.failed_segment_index;
                result.audit_status = repaired_audit.passed ? PathAuditStatus::Repaired : PathAuditStatus::Failed;
                result.audit_passed = repaired_audit.passed;
                result.success = repaired_audit.passed;
                if (repaired_audit.passed) {
                    result.remaining_unsafe_assumptions = 0;
                    try_final_simplify();
                }
            } else {
                result.audit_status = PathAuditStatus::Failed;
                result.audit_passed = false;
                result.success = false;
            }
        } else {
            result.audit_status = PathAuditStatus::Failed;
            result.audit_passed = false;
            result.success = false;
        }
        result.audit_time_ms = std::chrono::duration<double, std::milli>(Clock::now() - audit_t0).count();
        summarize_query_path(result, boxes_, segment_edges_);
        if (result.audit_passed) {
            result.remaining_unsafe_assumptions = 0;
        }
    }
    if (partition_last_query_cache_enabled) {
        partition_last_query_cache_.valid = true;
        partition_last_query_cache_.allow_collision_shortcut = allow_collision_shortcut;
        partition_last_query_cache_.active_query_index = active_query_index;
        partition_last_query_cache_.start = start;
        partition_last_query_cache_.goal = goal;
        partition_last_query_cache_.result = result;
    }
    return result;
}

int RBFPlanningForest::anchor_query_endpoint(const Eigen::Ref<const Eigen::VectorXd>& point) {
    StageContext context = StageContext::from_runtime(config_.runtime);
    const int box_id = anchor_query_endpoint_box(point, context);
    merge_diagnostic_snapshot(last_build_.diagnostics, context.diagnostics().snapshot());
    return box_id;
}

int RBFPlanningForest::connect_query_endpoint_to_main_island(
    const Eigen::Ref<const Eigen::VectorXd>& point,
    double max_segment_length) {
    if (boxes_.empty() || !oracle_) {
        return 0;
    }
    record_portal_membership_policy(last_build_.diagnostics, config_.portal_membership_policy);
    const std::size_t boxes_before_endpoint_main = boxes_.size();
    std::vector<int> pre_anchor_main_island_storage;
    if (adaptive_partition_query_enabled_ && adaptive_partition_ && !adaptive_partition_->empty()) {
        pre_anchor_main_island_storage = adaptive_partition_->largest_component_box_ids_with_overlay();
    }
    int source_box_id = locate_box_partition_first(point, config_.query.nearest_if_outside);

    std::vector<int> main_island_storage;
    if (!pre_anchor_main_island_storage.empty()) {
        main_island_storage = pre_anchor_main_island_storage;
    } else if (adaptive_partition_query_enabled_ && adaptive_partition_ && !adaptive_partition_->empty()) {
        main_island_storage = adaptive_partition_->largest_component_box_ids_with_overlay();
    }
    if (main_island_storage.empty()) {
        if (partition_native_mode()) {
            last_build_.diagnostics["query_bridge.endpoint_to_main_direct_partition_missing_no_graph_fallback"] += 1.0;
            return 0;
        }
        auto islands = find_islands(adjacency_);
        if (islands.empty()) {
            return 0;
        }
        std::sort(islands.begin(), islands.end(), [](const auto& lhs, const auto& rhs) {
            return lhs.size() > rhs.size();
        });
        main_island_storage = islands.front();
    }
    const auto& main_island = main_island_storage;
    if (source_box_id >= 0 &&
        std::find(main_island.begin(), main_island.end(), source_box_id) != main_island.end()) {
        last_build_.diagnostics["query_bridge.endpoint_to_main_direct_already_main"] += 1.0;
        return 0;
    }

    int target_box_id = -1;
    Eigen::VectorXd target_point = point;
    double best_dist2 = std::numeric_limits<double>::infinity();
    if (adaptive_partition_query_enabled_ && adaptive_partition_ && !adaptive_partition_->empty()) {
        const auto nearest = adaptive_partition_->nearest_boxes(point, main_island, 1);
        if (!nearest.empty()) {
            target_box_id = nearest.front().box_id;
            target_point = nearest.front().closest_point;
            best_dist2 = nearest.front().distance_sq;
            last_build_.diagnostics["query_bridge.endpoint_to_main_direct_partition_nearest"] += 1.0;
        }
    } else {
        for (int box_id : main_island) {
            const BoxNode* box = find_box_by_id(boxes_, box_id);
            if (box == nullptr || box->n_dims() != point.size()) {
                continue;
            }
            const Eigen::VectorXd candidate = closest_point_in_box(*box, point);
            const double dist2 = (candidate - point).squaredNorm();
            if (dist2 < best_dist2) {
                best_dist2 = dist2;
                target_box_id = box_id;
                target_point = candidate;
            }
        }
    }
    if (target_box_id < 0 || best_dist2 <= 1e-18) {
        last_build_.diagnostics["query_bridge.endpoint_to_main_direct_missing_target"] += 1.0;
        return 0;
    }
    const double length = std::sqrt(best_dist2);
    if (max_segment_length > 0.0 && length > max_segment_length) {
        last_build_.diagnostics["query_bridge.endpoint_to_main_direct_too_long"] += 1.0;
        last_build_.diagnostics["query_bridge.endpoint_to_main_direct_too_long_length"] += length;
        if (source_box_id < 0) {
            last_build_.diagnostics["query_bridge.endpoint_to_main_direct_too_long_anchor_skipped"] += 1.0;
        }
        return 0;
    }

    if (source_box_id < 0) {
        StageContext context = StageContext::from_runtime(config_.runtime);
        source_box_id = anchor_query_endpoint_box(point, context);
        merge_diagnostic_snapshot(last_build_.diagnostics, context.diagnostics().snapshot());
        if (partition_native_mode() &&
            adaptive_partition_query_enabled_ &&
            adaptive_partition_ &&
            boxes_.size() > boxes_before_endpoint_main) {
            append_adaptive_partition_boxes(boxes_before_endpoint_main,
                                            &last_build_,
                                            "query_bridge.endpoint_to_main_direct_anchor");
            source_box_id = locate_box_partition_first(point, false);
        }
    }
    if (source_box_id < 0) {
        last_build_.diagnostics["query_bridge.endpoint_to_main_direct_missing_source"] += 1.0;
        return 0;
    }

    CollisionChecker checker = make_audit_checker(audit_robot_, scene_, config_.query);
    std::vector<Eigen::VectorXd> waypoints{point, target_point};
    const PathAuditCheck audit = audit_waypoint_path(waypoints,
                                                     checker,
                                                     config_.query.audit_resolution,
                                                     config_.query.audit_segment_step);
    last_build_.diagnostics["query_bridge.endpoint_to_main_direct_attempts"] += 1.0;
    if (!audit.passed) {
        last_build_.diagnostics["query_bridge.endpoint_to_main_direct_audit_fail"] += 1.0;
        return 0;
    }
    const int edge_id = add_segment_edge_partition_first(                                         source_box_id,
                                         target_box_id,
                                         std::move(waypoints),
                                         SegmentEdgeType::QueryBridge,
                                         config_.query.audit_resolution,
                                         SegmentEdgeValidation::CollisionChecked,
                                         true,
                                         -1);
    if (edge_id < 0) {
        last_build_.diagnostics["query_bridge.endpoint_to_main_direct_add_fail"] += 1.0;
        return 0;
    }
    last_build_.diagnostics["query_bridge.endpoint_to_main_direct_success"] += 1.0;
    last_build_.diagnostics["query_bridge.endpoint_to_main_direct_length"] += length;
    sync_adaptive_partition_segment_edges(&last_build_, "query_bridge.endpoint_to_main_direct");
    invalidate_query_cache();
    return 1;
}

int RBFPlanningForest::connect_query_endpoint_to_main_box_corridor(
    const Eigen::Ref<const Eigen::VectorXd>& point,
    const EndpointMainBoxCorridorConfig& corridor_config) {
    using Clock = std::chrono::steady_clock;
    const auto t0 = Clock::now();
    auto add_diag = [&](const std::string& key, double value = 1.0) {
        last_build_.diagnostics["endpoint_main." + key] += value;
    };
    auto set_diag = [&](const std::string& key, double value) {
        last_build_.diagnostics["endpoint_main." + key] = value;
    };
    record_portal_membership_policy(last_build_.diagnostics, config_.portal_membership_policy);
    struct TimingFlush {
        BuildProfile& profile;
        Clock::time_point start;
        ~TimingFlush() {
            const double ms = std::chrono::duration<double, std::milli>(
                                  Clock::now() - start)
                                  .count();
            profile.diagnostics["endpoint_main.ms"] += ms;
        }
    } timing_flush{last_build_, t0};

    if (boxes_.empty() || !oracle_ || point.size() != oracle_->n_dims()) {
        add_diag("fallback_to_e2e");
        return 0;
    }

    const std::size_t boxes_before_endpoint_main = boxes_.size();
    std::vector<int> pre_anchor_main_island_storage;
    if (adaptive_partition_query_enabled_ && adaptive_partition_ && !adaptive_partition_->empty()) {
        pre_anchor_main_island_storage = adaptive_partition_->largest_component_box_ids_with_overlay();
    }
    StageContext context = StageContext::from_runtime(config_.runtime);
    int source_box_id = locate_box_partition_first(point, config_.query.nearest_if_outside);
    if (source_box_id < 0) {
        source_box_id = anchor_query_endpoint_box(point, context);
        merge_diagnostic_snapshot(last_build_.diagnostics, context.diagnostics().snapshot());
        if (partition_native_mode() &&
            adaptive_partition_query_enabled_ &&
            adaptive_partition_ &&
            boxes_.size() > boxes_before_endpoint_main) {
            append_adaptive_partition_boxes(boxes_before_endpoint_main,
                                            &last_build_,
                                            "endpoint_main.anchor");
            source_box_id = locate_box_partition_first(point, false);
        }
    }
    if (source_box_id < 0) {
        add_diag("anchor_fail");
        add_diag("fallback_to_e2e");
        return 0;
    }

    std::vector<int> main_island_storage;
    if (!pre_anchor_main_island_storage.empty()) {
        main_island_storage = pre_anchor_main_island_storage;
    } else if (adaptive_partition_query_enabled_ && adaptive_partition_ && !adaptive_partition_->empty()) {
        main_island_storage = adaptive_partition_->largest_component_box_ids_with_overlay();
    }
    if (main_island_storage.empty()) {
        if (partition_native_mode()) {
            add_diag("partition_missing_no_graph_fallback");
            add_diag("fallback_to_e2e");
            return 0;
        }
        auto islands = find_islands(adjacency_);
        if (islands.empty()) {
            add_diag("fallback_to_e2e");
            return 0;
        }
        std::sort(islands.begin(), islands.end(), [](const auto& lhs, const auto& rhs) {
            return lhs.size() > rhs.size();
        });
        main_island_storage = islands.front();
    }
    const auto& main_island = main_island_storage;
    std::unordered_set<int> main_ids(main_island.begin(), main_island.end());
    if (main_ids.find(source_box_id) != main_ids.end()) {
        add_diag("already_main");
        return 0;
    }

    std::unordered_map<int, std::size_t> box_index_by_id;
    box_index_by_id.reserve(boxes_.size() * 2);
    for (std::size_t index = 0; index < boxes_.size(); ++index) {
        box_index_by_id[boxes_[index].id] = index;
    }
    const bool graphless_endpoint_main = partition_native_mode();
    const bool use_partition_endpoint_index =
        graphless_endpoint_main &&
        adaptive_partition_query_enabled_ &&
        adaptive_partition_;

    struct TargetCandidate {
        int box_id = -1;
        Eigen::VectorXd point;
        double dist2 = 0.0;
    };
    std::vector<TargetCandidate> targets;
    if (use_partition_endpoint_index) {
        const auto nearest = adaptive_partition_->nearest_boxes(
            point,
            main_island,
            std::max(1, corridor_config.target_k));
        targets.reserve(nearest.size());
        for (const auto& item : nearest) {
            Eigen::VectorXd target_point = item.closest_point;
            double dist2 = item.distance_sq;
            if (dist2 <= 1e-18) {
                Eigen::VectorXd center;
                if (adaptive_partition_->center_for_box(item.box_id, center) &&
                    center.size() == point.size()) {
                    target_point = std::move(center);
                    dist2 = (target_point - point).squaredNorm();
                }
            }
            targets.push_back({item.box_id, std::move(target_point), dist2});
        }
        add_diag("partition_nearest_target_queries");
    } else {
        targets.reserve(main_island.size());
        for (int box_id : main_island) {
            const auto box_it = box_index_by_id.find(box_id);
            if (box_it == box_index_by_id.end() ||
                box_it->second >= boxes_.size()) {
                continue;
            }
            const BoxNode& box = boxes_[box_it->second];
            if (box.n_dims() != point.size()) {
                continue;
            }
            Eigen::VectorXd target_point = closest_point_in_box(box, point);
            double dist2 = (target_point - point).squaredNorm();
            if (dist2 <= 1e-18) {
                target_point = box.center();
                dist2 = (target_point - point).squaredNorm();
            }
            targets.push_back({box_id, std::move(target_point), dist2});
        }
    }
    if (targets.empty()) {
        add_diag("missing_target");
        add_diag("fallback_to_e2e");
        return 0;
    }
    std::sort(targets.begin(), targets.end(), [](const TargetCandidate& lhs,
                                                 const TargetCandidate& rhs) {
        return lhs.dist2 < rhs.dist2;
    });
    const int target_limit = std::min<int>(
        std::max(1, corridor_config.target_k),
        static_cast<int>(targets.size()));

    std::unordered_map<int, int> node_owner;
    node_owner.reserve(boxes_.size());
    for (const auto& box : boxes_) {
        if (node_owner.find(box.tree_id) == node_owner.end()) {
            node_owner[box.tree_id] = box.id;
        }
    }
    BoxSpatialIndex all_box_index;

    std::vector<int> main_box_ids;
    std::vector<BoxNode> main_boxes;
    BoxSpatialIndex main_box_index;
    if (!use_partition_endpoint_index) {
        all_box_index.rebuild(boxes_, config_.query.adjacency_tolerance);
        main_box_ids.reserve(main_island.size());
        main_boxes.reserve(main_island.size());
        for (int box_id : main_island) {
            auto it = box_index_by_id.find(box_id);
            if (it == box_index_by_id.end()) {
                continue;
            }
            const BoxNode& box = boxes_[it->second];
            if (box.n_dims() != point.size()) {
                continue;
            }
            main_box_ids.push_back(box_id);
            main_boxes.push_back(box);
        }
        main_box_index.rebuild(main_boxes, config_.query.adjacency_tolerance);
    }

	int next_id = next_box_id();
	auto finish_endpoint_main = [&](int value) {
		if (boxes_.size() > boxes_before_endpoint_main) {
			append_adaptive_partition_boxes(boxes_before_endpoint_main,
											&last_build_,
											"endpoint_main");
		}
		sync_adaptive_partition_segment_edges(&last_build_, "endpoint_main");
		return value;
	};
	int added_total = 0;
    int boxes_added = 0;
    int ffb_calls = 0;
    int local_adj_checks = 0;
    bool max_depth_ffb_failed = false;
    const int requested_final_depth = config_.query_bridge_pave_depth > 0
        ? config_.query_bridge_pave_depth
        : config_.connector.pave.find_free_box.max_depth;
    const int final_ffb_depth = std::min(std::max(1, config_.database.max_tree_depth),
                                         std::max(1, requested_final_depth));

    auto box_by_id = [&](int box_id) -> BoxNode* {
        auto it = box_index_by_id.find(box_id);
        if (it == box_index_by_id.end() || it->second >= boxes_.size()) {
            return nullptr;
        }
        return &boxes_[it->second];
    };
    auto contains_point = [&](int box_id, const Eigen::VectorXd& q) {
        if (use_partition_endpoint_index &&
            adaptive_partition_->box_contains_point(box_id,
                                                    q,
                                                    config_.query.adjacency_tolerance)) {
            return true;
        }
        const BoxNode* box = box_by_id(box_id);
        return box != nullptr && box->contains(q, config_.query.adjacency_tolerance);
    };
    auto append_edge_if_connected = [&](int lhs, int rhs) {
        if (lhs == rhs) {
            return true;
        }
        if (use_partition_endpoint_index &&
            adaptive_partition_->contains_box_id(lhs) &&
            adaptive_partition_->contains_box_id(rhs)) {
            local_adj_checks += 1;
            return adaptive_partition_->boxes_are_neighbors(lhs, rhs);
        }
        BoxNode* lhs_box = box_by_id(lhs);
        BoxNode* rhs_box = box_by_id(rhs);
        if (lhs_box == nullptr || rhs_box == nullptr) {
            return false;
        }
        local_adj_checks += 1;
        if (!boxes_connected(*lhs_box, *rhs_box, config_.query.adjacency_tolerance)) {
            return false;
        }
        if (!graphless_endpoint_main) {
            append_local_edge(adjacency_, lhs, rhs);
        }
        return true;
    };
    auto main_owner = [&](const Eigen::VectorXd& q) {
        if (use_partition_endpoint_index) {
            const auto ids = adaptive_partition_->covering_box_ids(
                q,
                config_.query.adjacency_tolerance);
            for (int box_id : ids) {
                if (main_ids.find(box_id) != main_ids.end()) {
                    return box_id;
                }
            }
            return -1;
        }
        auto candidates = main_box_index.point_candidates(q);
        if (candidates.empty()) {
            candidates.reserve(main_boxes.size());
            for (int index = 0; index < static_cast<int>(main_boxes.size()); ++index) {
                candidates.push_back(index);
            }
        }
        for (int index : candidates) {
            if (index < 0 || index >= static_cast<int>(main_boxes.size())) {
                continue;
            }
            const BoxNode& box = main_boxes[static_cast<std::size_t>(index)];
            if (box.contains(q, config_.query.adjacency_tolerance)) {
                return main_box_ids[static_cast<std::size_t>(index)];
            }
        }
        return -1;
    };
    auto first_existing_cover = [&](const Eigen::VectorXd& q) {
        if (use_partition_endpoint_index) {
            const auto ids = adaptive_partition_->covering_box_ids(
                q,
                config_.query.adjacency_tolerance);
            if (!ids.empty()) {
                return ids.front();
            }
            for (std::size_t index = boxes_before_endpoint_main; index < boxes_.size(); ++index) {
                if (intervals_contain_point_local(boxes_[index].joint_intervals,
                                                  q,
                                                  config_.query.adjacency_tolerance)) {
                    return boxes_[index].id;
                }
            }
            return -1;
        }
        const int index = all_box_index.covering_box(
            boxes_,
            q,
            config_.query.adjacency_tolerance);
        if (index >= 0 && index < static_cast<int>(boxes_.size())) {
            return boxes_[static_cast<std::size_t>(index)].id;
        }
        return -1;
    };
    auto segment_exit_param = [&](const BoxNode& box,
                                  const Eigen::VectorXd& from,
                                  const Eigen::VectorXd& to) {
        if (box.n_dims() != from.size() || to.size() != from.size()) {
            return 0.0;
        }
        const Eigen::VectorXd delta = to - from;
        double exit_param = 1.0;
        for (int dim = 0; dim < from.size(); ++dim) {
            const double d = delta[dim];
            if (std::abs(d) < 1e-15) {
                continue;
            }
            const auto& interval = box.joint_intervals[static_cast<std::size_t>(dim)];
            const double boundary = d > 0.0 ? interval.hi : interval.lo;
            const double t = (boundary - from[dim]) / d;
            if (t > 1e-12 && t < exit_param) {
                exit_param = t;
            }
        }
        return std::clamp(exit_param, 0.0, 1.0);
    };
    auto segment_exit_param_intervals = [&](const std::vector<Interval>& intervals,
                                            const Eigen::VectorXd& from,
                                            const Eigen::VectorXd& to) {
        if (intervals.size() != static_cast<std::size_t>(from.size()) ||
            to.size() != from.size()) {
            return 0.0;
        }
        const Eigen::VectorXd delta = to - from;
        double exit_param = 1.0;
        for (int dim = 0; dim < from.size(); ++dim) {
            const double d = delta[dim];
            if (std::abs(d) < 1e-15) {
                continue;
            }
            const auto& interval = intervals[static_cast<std::size_t>(dim)];
            const double boundary = d > 0.0 ? interval.hi : interval.lo;
            const double t = (boundary - from[dim]) / d;
            if (t > 1e-12 && t < exit_param) {
                exit_param = t;
            }
        }
        return std::clamp(exit_param, 0.0, 1.0);
    };
    auto make_seed_from_face = [&](int box_id,
                                   const Eigen::VectorXd& from,
                                   const Eigen::VectorXd& to) {
        const Eigen::VectorXd delta = to - from;
        const double norm = delta.norm();
        if (norm <= 1e-12) {
            return from;
        }
        double u = 0.0;
        std::vector<Interval> intervals;
        if (use_partition_endpoint_index &&
            adaptive_partition_->intervals_for_box(box_id, intervals)) {
            u = segment_exit_param_intervals(intervals, from, to);
        } else {
            const BoxNode* box = find_box_by_id(boxes_, box_id);
            if (box == nullptr) {
                return from;
            }
            u = segment_exit_param(*box, from, to);
        }
        Eigen::VectorXd seed = from + u * delta + corridor_config.face_epsilon * (delta / norm);
        const auto domain = oracle_->planning_intervals();
        for (int dim = 0; dim < seed.size() &&
                          dim < static_cast<int>(domain.size()); ++dim) {
            seed[dim] = std::min(domain[static_cast<std::size_t>(dim)].hi,
                                 std::max(domain[static_cast<std::size_t>(dim)].lo,
                                          seed[dim]));
        }
        return seed;
    };
    auto furthest_sample = [&](int box_id,
                               const std::vector<Eigen::VectorXd>& samples,
                               int start_index,
                               int target_index) {
        int best = std::max(0, start_index);
        for (int index = best; index <= target_index; ++index) {
            if (contains_point(box_id, samples[static_cast<std::size_t>(index)])) {
                best = index;
            }
        }
        return best;
    };
    auto attempt_seed = [&](const Eigen::VectorXd& seed,
                            int parent_box_id,
                            const std::vector<int>& local_candidates,
                            const std::vector<int>& target_box_ids,
                            const std::vector<int>& depths) {
        int reached_box_id = -1;
        bool reached_main = false;
        const int existing_cover = first_existing_cover(seed);
        if (existing_cover >= 0 &&
            append_edge_if_connected(parent_box_id, existing_cover)) {
            reached_box_id = existing_cover;
            reached_main = main_ids.find(existing_cover) != main_ids.end();
            return std::pair<int, bool>{reached_box_id, reached_main};
        }
        for (int depth : depths) {
            FindFreeBoxOptions options = config_.connector.pave.find_free_box;
            options.reject_seed_collision = false;
            options.max_depth = std::min(std::max(1, config_.database.max_tree_depth),
                                         std::max(1, depth));
            const bool is_max_depth_attempt = options.max_depth >= final_ffb_depth;
            ffb_calls += 1;
            add_diag("ffb_calls");
            FindFreeBoxResult result = find_free_box_in_domain(
                seed,
                oracle_->planning_intervals(),
                context,
                options);
            if (!result.found ||
                !intervals_contain_point_local(result.intervals,
                                               seed,
                                               config_.query.adjacency_tolerance)) {
                if (is_max_depth_attempt) {
                    max_depth_ffb_failed = true;
                }
                if (ffb_calls >= std::max(1, corridor_config.max_ffb_calls)) {
                    break;
                }
                continue;
            }
            if (!allow_dynamic_commit(*oracle_, result, config_.connector.pave.commit_policy)) {
                if (is_max_depth_attempt) {
                    max_depth_ffb_failed = true;
                }
                if (ffb_calls >= std::max(1, corridor_config.max_ffb_calls)) {
                    break;
                }
                continue;
            }
            BoxNode candidate;
            candidate.joint_intervals = result.intervals;
            candidate.seed_config = seed;
            candidate.tree_id = result.node;
            candidate.parent_box_id = parent_box_id;
            candidate.safety_status = result.validation_detail.safety_status;
            candidate.strict_audit_required = result.validation_detail.strict_audit_required;
            candidate.compute_volume();
            BoxNode* parent_box = box_by_id(parent_box_id);
            bool parent_adjacent = false;
            if (use_partition_endpoint_index &&
                adaptive_partition_->contains_box_id(parent_box_id)) {
                local_adj_checks += 1;
                parent_adjacent = adaptive_partition_->box_adjacent_to_box(
                    parent_box_id,
                    candidate,
                    config_.query.adjacency_tolerance);
            } else if (parent_box != nullptr) {
                local_adj_checks += 1;
                parent_adjacent = boxes_connected(*parent_box,
                                                  candidate,
                                                  config_.query.adjacency_tolerance);
            }
            if (!parent_adjacent) {
                if (is_max_depth_attempt) {
                    max_depth_ffb_failed = true;
                }
                if (ffb_calls >= std::max(1, corridor_config.max_ffb_calls)) {
                    break;
                }
                continue;
            }
            candidate.root_id =
                parent_box != nullptr && parent_box->root_id >= 0 ? parent_box->root_id : parent_box_id;
            candidate.id = next_id++;
            const int new_id = candidate.id;
            if (node_owner.find(candidate.tree_id) == node_owner.end()) {
                oracle_->reserve_node(candidate.tree_id, new_id);
                node_owner[candidate.tree_id] = new_id;
            }
            boxes_.push_back(candidate);
            raw_boxes_.push_back(candidate);
            const std::size_t new_index = boxes_.size() - 1;
            box_index_by_id[new_id] = new_index;
            if (!use_partition_endpoint_index) {
                all_box_index.add_box(boxes_.back(),
                                      static_cast<int>(new_index),
                                      config_.query.adjacency_tolerance);
            } else {
                adaptive_partition_->append_box(boxes_.back(),
                                                config_.query.adjacency_tolerance);
            }
            if (!graphless_endpoint_main) {
                adjacency_[new_id] = {};
                append_local_edge(adjacency_, parent_box_id, new_id);
            }
            boxes_added += 1;
            added_total += 1;
            add_diag("boxes_added");

            for (int candidate_id : local_candidates) {
                append_edge_if_connected(new_id, candidate_id);
            }
            for (int target_id : target_box_ids) {
                if (append_edge_if_connected(new_id, target_id)) {
                    reached_main = true;
                }
            }
            reached_box_id = new_id;
            return std::pair<int, bool>{reached_box_id, reached_main};
        }
        return std::pair<int, bool>{-1, false};
    };
    auto lateral_seeds = [&](const Eigen::VectorXd& seed,
                             const Eigen::VectorXd& direction) {
        std::vector<int> dims;
        dims.reserve(static_cast<std::size_t>(seed.size()));
        for (int dim = 0; dim < seed.size(); ++dim) {
            dims.push_back(dim);
        }
        std::sort(dims.begin(), dims.end(), [&](int lhs, int rhs) {
            return std::abs(direction[lhs]) < std::abs(direction[rhs]);
        });
        std::vector<Eigen::VectorXd> out;
        const auto domain = oracle_->planning_intervals();
        const int dim_limit = std::min<int>(std::max(0, corridor_config.lateral_rounds),
                                            static_cast<int>(dims.size()));
        for (int item = 0; item < dim_limit; ++item) {
            const int dim = dims[static_cast<std::size_t>(item)];
            for (double sign : {1.0, -1.0}) {
                Eigen::VectorXd candidate = seed;
                candidate[dim] += sign * corridor_config.lateral_offset;
                if (dim < static_cast<int>(domain.size())) {
                    candidate[dim] = std::min(domain[static_cast<std::size_t>(dim)].hi,
                                              std::max(domain[static_cast<std::size_t>(dim)].lo,
                                                       candidate[dim]));
                }
                out.push_back(std::move(candidate));
            }
        }
        return out;
    };
    auto try_residual_segment = [&](int front_box_id, int target_box_id, const Eigen::VectorXd& target_point) {
        if (!max_depth_ffb_failed || corridor_config.residual_segment_max_length <= 0.0) {
            return false;
        }
        Eigen::VectorXd front_point;
        Eigen::VectorXd main_point;
        if (use_partition_endpoint_index &&
            adaptive_partition_->closest_point_for_box(front_box_id,
                                                       target_point,
                                                       front_point) &&
            adaptive_partition_->closest_point_for_box(target_box_id,
                                                       front_point,
                                                       main_point)) {
        } else {
            const BoxNode* front_box = box_by_id(front_box_id);
            const BoxNode* target_box = box_by_id(target_box_id);
            if (front_box == nullptr || target_box == nullptr) {
                return false;
            }
            front_point = closest_point_in_box(*front_box, target_point);
            main_point = closest_point_in_box(*target_box, front_point);
        }
        const double length = (main_point - front_point).norm();
        if (length > corridor_config.residual_segment_max_length) {
            return false;
        }
        CollisionChecker checker = make_audit_checker(audit_robot_, scene_, config_.query);
        std::vector<Eigen::VectorXd> waypoints{front_point, main_point};
        const PathAuditCheck audit = audit_waypoint_path(waypoints,
                                                         checker,
                                                         config_.query.audit_resolution,
                                                         config_.query.audit_segment_step);
        if (!audit.passed) {
            return false;
        }
        const int edge_id = add_segment_edge_partition_first(                                             front_box_id,
                                             target_box_id,
                                             std::move(waypoints),
                                             SegmentEdgeType::QueryBridge,
                                             config_.query.audit_resolution,
                                             SegmentEdgeValidation::CollisionChecked,
                                             true,
                                             -1);
        if (edge_id < 0) {
            return false;
        }
        add_diag("residual_segment_edges");
        added_total += 1;
        return true;
    };

    std::vector<int> depth_schedule = corridor_config.adaptive_ffb_depths;
    depth_schedule.push_back(final_ffb_depth);
    for (int& depth : depth_schedule) {
        depth = std::min(std::max(1, config_.database.max_tree_depth),
                         std::max(1, depth));
    }
    std::sort(depth_schedule.begin(), depth_schedule.end());
    depth_schedule.erase(std::unique(depth_schedule.begin(), depth_schedule.end()),
                         depth_schedule.end());

    for (int target_index = 0; target_index < target_limit; ++target_index) {
        if (ffb_calls >= std::max(1, corridor_config.max_ffb_calls) ||
            boxes_added >= std::max(1, corridor_config.max_boxes)) {
            break;
        }
        add_diag("targets_tested");
        const TargetCandidate& target = targets[static_cast<std::size_t>(target_index)];
        std::vector<Eigen::VectorXd> samples =
            densify_waypoint_path_local({point, target.point},
                                        std::max(1e-4, corridor_config.coarse_step));
        if (samples.size() < 2) {
            continue;
        }
        int target_sample_index = static_cast<int>(samples.size()) - 1;
        int target_owner = target.box_id;
        for (int sample_index = 1; sample_index < static_cast<int>(samples.size()); ++sample_index) {
            const int owner = main_owner(samples[static_cast<std::size_t>(sample_index)]);
            if (owner >= 0) {
                target_sample_index = sample_index;
                target_owner = owner;
                break;
            }
        }
        if (target_sample_index > 1 && corridor_config.fine_step > 0.0) {
            std::vector<Eigen::VectorXd> fine =
                densify_waypoint_path_local({samples.front(),
                                             samples[static_cast<std::size_t>(target_sample_index)]},
                                            std::max(1e-4, corridor_config.fine_step));
            if (fine.size() >= 2) {
                samples = std::move(fine);
                target_sample_index = static_cast<int>(samples.size()) - 1;
            }
        }
        std::vector<int> target_box_ids;
        target_box_ids.reserve(static_cast<std::size_t>(target_limit));
        for (int item = 0; item < target_limit; ++item) {
            target_box_ids.push_back(targets[static_cast<std::size_t>(item)].box_id);
        }
        std::vector<int> chain_ids{source_box_id};
        int current_box_id = source_box_id;
        int current_sample_index = furthest_sample(current_box_id, samples, 0, target_sample_index);
        int stall_count = 0;
        while (current_sample_index < target_sample_index &&
               ffb_calls < std::max(1, corridor_config.max_ffb_calls) &&
               boxes_added < std::max(1, corridor_config.max_boxes)) {
		if (append_edge_if_connected(current_box_id, target_owner)) {
			add_diag("main_contact_success");
			set_diag("local_adj_checks", static_cast<double>(local_adj_checks));
			invalidate_query_cache();
			return finish_endpoint_main(std::max(1, added_total));
		}
            const Eigen::VectorXd current_sample =
                samples[static_cast<std::size_t>(current_sample_index)];
            Eigen::VectorXd from = current_sample;
            if (!contains_point(current_box_id, current_sample)) {
                if (use_partition_endpoint_index) {
                    if (!adaptive_partition_->closest_point_for_box(current_box_id,
                                                                    current_sample,
                                                                    from)) {
                        break;
                    }
                } else {
                    const BoxNode* current_box = find_box_by_id(boxes_, current_box_id);
                    if (current_box == nullptr) {
                        break;
                    }
                    from = closest_point_in_box(*current_box, current_sample);
                }
            }
            const Eigen::VectorXd seed = make_seed_from_face(
                current_box_id,
                from,
                samples[static_cast<std::size_t>(target_sample_index)]);
            std::vector<int> local_candidates = chain_ids;
            local_candidates.push_back(target_owner);
            const auto [new_box_id, reached_main] = attempt_seed(
                seed,
                current_box_id,
                local_candidates,
                target_box_ids,
                depth_schedule);
			if (reached_main) {
				add_diag("main_contact_success");
				set_diag("local_adj_checks", static_cast<double>(local_adj_checks));
				invalidate_query_cache();
				return finish_endpoint_main(std::max(1, added_total));
			}
            if (new_box_id >= 0) {
                const int next_sample_index =
                    furthest_sample(new_box_id, samples, current_sample_index, target_sample_index);
                current_box_id = new_box_id;
                chain_ids.push_back(new_box_id);
                if (next_sample_index > current_sample_index) {
                    current_sample_index = next_sample_index;
                    stall_count = 0;
                    continue;
                }
                stall_count += 1;
            } else {
                stall_count += 1;
            }
            if (stall_count <= std::max(0, corridor_config.lateral_rounds)) {
                const Eigen::VectorXd direction =
                    samples[static_cast<std::size_t>(target_sample_index)] - seed;
                bool lateral_progress = false;
                for (const auto& lateral_seed : lateral_seeds(seed, direction)) {
                    if (ffb_calls >= std::max(1, corridor_config.max_ffb_calls) ||
                        boxes_added >= std::max(1, corridor_config.max_boxes)) {
                        break;
                    }
                    const auto [lat_box_id, lat_main] = attempt_seed(
                        lateral_seed,
                        current_box_id,
                        local_candidates,
                        target_box_ids,
                        depth_schedule);
					if (lat_main) {
						add_diag("main_contact_success");
						set_diag("local_adj_checks", static_cast<double>(local_adj_checks));
						invalidate_query_cache();
						return finish_endpoint_main(std::max(1, added_total));
					}
                    if (lat_box_id >= 0) {
                        const int next_sample_index =
                            furthest_sample(lat_box_id, samples, current_sample_index, target_sample_index);
                        current_box_id = lat_box_id;
                        chain_ids.push_back(lat_box_id);
                        if (next_sample_index > current_sample_index) {
                            current_sample_index = next_sample_index;
                            lateral_progress = true;
                            stall_count = 0;
                            break;
                        }
                    }
                }
                if (lateral_progress) {
                    continue;
                }
            }
			if (try_residual_segment(current_box_id, target_owner, target.point)) {
				add_diag("main_contact_success");
				set_diag("local_adj_checks", static_cast<double>(local_adj_checks));
				invalidate_query_cache();
				return finish_endpoint_main(added_total);
			}
            break;
        }
		if (append_edge_if_connected(current_box_id, target_owner) ||
			box_only_path_connected_partition_first(current_box_id, target_owner)) {
			add_diag("main_contact_success");
			set_diag("local_adj_checks", static_cast<double>(local_adj_checks));
			invalidate_query_cache();
			return finish_endpoint_main(std::max(1, added_total));
		}
		if (try_residual_segment(current_box_id, target_owner, target.point)) {
			add_diag("main_contact_success");
			set_diag("local_adj_checks", static_cast<double>(local_adj_checks));
			invalidate_query_cache();
			return finish_endpoint_main(added_total);
		}
    }

    merge_diagnostic_snapshot(last_build_.diagnostics, context.diagnostics().snapshot());
	set_diag("local_adj_checks", static_cast<double>(local_adj_checks));
	add_diag("fallback_to_e2e");
	invalidate_query_cache();
	return finish_endpoint_main(0);
}

int RBFPlanningForest::add_offline_shortcut_edges(int max_edges,
                                                  int candidate_limit,
                                                  double min_gain_ratio,
                                                  double max_segment_length) {
    if (max_edges <= 0 || candidate_limit < 2 || boxes_.size() < 2) {
        return 0;
    }
    if (partition_native_mode()) {
        if (!adaptive_partition_query_enabled_ || !adaptive_partition_ || adaptive_partition_->empty()) {
            last_build_.diagnostics["offline_shortcut.partition_native_missing_partition"] += 1.0;
            return 0;
        }
        const int limit = std::min<int>(candidate_limit, static_cast<int>(boxes_.size()));
        auto landmarks = adaptive_partition_->landmarks(true, limit);
        if (static_cast<int>(landmarks.size()) < 2) {
            landmarks = adaptive_partition_->landmarks(false, limit);
        }
        if (static_cast<int>(landmarks.size()) < 2) {
            last_build_.diagnostics["offline_shortcut.partition_native_insufficient_landmarks"] += 1.0;
            return 0;
        }
        auto existing_segment_pairs = partition_segment_pair_set_local(segment_edges_);

        struct ShortcutCandidate {
            int source = -1;
            int target = -1;
            Eigen::VectorXd source_center;
            Eigen::VectorXd target_center;
            double direct = 0.0;
            double partition_cost = 0.0;
            double score = 0.0;
        };
        std::vector<ShortcutCandidate> candidates;
        const double safe_min_gain = std::max(1.0, min_gain_ratio);
        const double safe_max_segment_length = std::max(0.0, max_segment_length);
        AdaptiveGridPartitionQueryOptions query_options;
        query_options.nearest_if_outside = false;
        query_options.shortcut_boxes = false;
        query_options.max_expansions = std::max(0, last_adaptive_partition_config_.grid_planning_max_expansions);
        query_options.adjacency_tolerance = config_.query.adjacency_tolerance;
        int tested_pairs = 0;
        for (std::size_t outer = 0; outer < landmarks.size(); ++outer) {
            const auto& lhs = landmarks[outer];
            for (std::size_t inner = outer + 1; inner < landmarks.size(); ++inner) {
                const auto& rhs = landmarks[inner];
                if (lhs.center.size() == 0 || lhs.center.size() != rhs.center.size()) {
                    continue;
                }
                ++tested_pairs;
                const double direct = (lhs.center - rhs.center).norm();
                if (direct <= 1e-9 ||
                    (safe_max_segment_length > 0.0 && direct > safe_max_segment_length) ||
                    existing_segment_pairs.count(
                        partition_segment_pair_key_local(lhs.box_id, rhs.box_id)) > 0) {
                    continue;
                }
                const auto route = adaptive_partition_->query(lhs.center, rhs.center, query_options);
                if (!route.found ||
                    !std::isfinite(route.total_cost) ||
                    route.total_cost <= direct * safe_min_gain) {
                    continue;
                }
                candidates.push_back({lhs.box_id,
                                      rhs.box_id,
                                      lhs.center,
                                      rhs.center,
                                      direct,
                                      route.total_cost,
                                      route.total_cost - direct});
            }
        }
        std::sort(candidates.begin(), candidates.end(), [](const ShortcutCandidate& lhs,
                                                           const ShortcutCandidate& rhs) {
            return lhs.score > rhs.score;
        });

        int added = 0;
        int portal_corridor_edges = 0;
        int portal_corridor_fail = 0;
        int box_corridor_edges = 0;
        int segment_edges = 0;
        int pave_added_total = 0;
        int pave_fail = 0;
        int audit_fail = 0;
        for (const auto& candidate : candidates) {
            if (added >= max_edges) {
                break;
            }
            if (candidate.source_center.size() == 0 ||
                candidate.source_center.size() != candidate.target_center.size()) {
                continue;
            }
            std::vector<Eigen::VectorXd> waypoints{candidate.source_center,
                                                   candidate.target_center};
            if (last_adaptive_partition_config_.hipac_portal_connectivity) {
                const int portal_added = add_partition_portal_corridor_overlay(candidate.source_center,
                                                                               candidate.target_center,
                                                                               waypoints,
                                                                               "offline_shortcut",
                                                                               false,
                                                                               false,
                                                                               -1,
                                                                               &last_build_);
                if (portal_added > 0) {
                    existing_segment_pairs.insert(
                        partition_segment_pair_key_local(candidate.source, candidate.target));
                    ++portal_corridor_edges;
                    ++added;
                    continue;
                }
                ++portal_corridor_fail;
            }
            const double audit_fail_before =
                last_build_.diagnostics["offline_shortcut.partition_box_corridor_overlay_audit_fail"];
            const int overlay_added = add_partition_box_corridor_overlay(candidate.source_center,
                                                                         candidate.target_center,
                                                                         waypoints,
                                                                         "offline_shortcut",
                                                                         false,
                                                                         false,
                                                                         -1,
                                                                         &last_build_);
            const double audit_fail_after =
                last_build_.diagnostics["offline_shortcut.partition_box_corridor_overlay_audit_fail"];
            if (audit_fail_after > audit_fail_before) {
                ++audit_fail;
            }
            if (overlay_added > 0) {
                existing_segment_pairs.insert(
                    partition_segment_pair_key_local(candidate.source, candidate.target));
                ++box_corridor_edges;
                ++added;
                continue;
            }
            ++pave_fail;
        }
        last_build_.diagnostics["offline_shortcut.partition_native"] += 1.0;
        last_build_.diagnostics["offline_shortcut.partition_native_direct_overlay"] += 1.0;
        last_build_.diagnostics["offline_shortcut.partition_native_query_bridge_skipped"] += 1.0;
        last_build_.diagnostics["offline_shortcut.tested_pairs"] += static_cast<double>(tested_pairs);
        last_build_.diagnostics["offline_shortcut.candidates"] += static_cast<double>(candidates.size());
        last_build_.diagnostics["offline_shortcut.audit_fail"] += static_cast<double>(audit_fail);
        last_build_.diagnostics["offline_shortcut.edges_added"] += static_cast<double>(added);
        last_build_.diagnostics["offline_shortcut.segment_edges_added"] += static_cast<double>(segment_edges);
        last_build_.diagnostics["offline_shortcut.portal_corridor_edges_added"] +=
            static_cast<double>(portal_corridor_edges);
        last_build_.diagnostics["offline_shortcut.portal_corridor_fail"] +=
            static_cast<double>(portal_corridor_fail);
        last_build_.diagnostics["offline_shortcut.box_corridor_edges_added"] += static_cast<double>(box_corridor_edges);
        last_build_.diagnostics["offline_shortcut.pave_boxes_added"] += static_cast<double>(pave_added_total);
        last_build_.diagnostics["offline_shortcut.pave_fail"] += static_cast<double>(pave_fail);
        sync_adaptive_partition_segment_edges(&last_build_, "offline_shortcut.partition_native");
        return added;
    }

    const int limit = std::min<int>(candidate_limit, static_cast<int>(boxes_.size()));
    std::vector<int> landmarks;
    landmarks.reserve(boxes_.size());
    for (int index = 0; index < static_cast<int>(boxes_.size()); ++index) {
        const auto graph_it = adjacency_.find(boxes_[static_cast<std::size_t>(index)].id);
        if (graph_it != adjacency_.end() && !graph_it->second.empty()) {
            landmarks.push_back(index);
        }
    }
    if (static_cast<int>(landmarks.size()) < 2) {
        landmarks.clear();
        for (int index = 0; index < static_cast<int>(boxes_.size()); ++index) {
            landmarks.push_back(index);
        }
    }
    std::sort(landmarks.begin(), landmarks.end(), [&](int lhs, int rhs) {
        return boxes_[static_cast<std::size_t>(lhs)].volume >
               boxes_[static_cast<std::size_t>(rhs)].volume;
    });
    if (static_cast<int>(landmarks.size()) > limit) {
        landmarks.resize(static_cast<std::size_t>(limit));
    }

    struct ShortcutCandidate {
        int source = -1;
        int target = -1;
        double direct = 0.0;
        double graph_cost = 0.0;
        double score = 0.0;
    };
    std::vector<ShortcutCandidate> candidates;
    const QueryGraphCache cache = build_query_graph_cache(boxes_, adjacency_, segment_edges_);
    const double safe_min_gain = std::max(1.0, min_gain_ratio);
    const double safe_max_segment_length = std::max(0.0, max_segment_length);
    int tested_pairs = 0;
    for (std::size_t outer = 0; outer < landmarks.size(); ++outer) {
        const BoxNode& lhs = boxes_[static_cast<std::size_t>(landmarks[outer])];
        for (std::size_t inner = outer + 1; inner < landmarks.size(); ++inner) {
            const BoxNode& rhs = boxes_[static_cast<std::size_t>(landmarks[inner])];
            if (lhs.n_dims() != rhs.n_dims()) {
                continue;
            }
            ++tested_pairs;
            const double direct = (lhs.center() - rhs.center()).norm();
            if (direct <= 1e-9 ||
                (safe_max_segment_length > 0.0 && direct > safe_max_segment_length) ||
                find_segment_edge(segment_edges_, lhs.id, rhs.id) != nullptr) {
                continue;
            }
            const auto route = dijkstra_search(cache, lhs.id, rhs.id, rhs.center());
            if (!route.found || !std::isfinite(route.total_cost) ||
                route.total_cost <= direct * safe_min_gain) {
                continue;
            }
            candidates.push_back({lhs.id, rhs.id, direct, route.total_cost, route.total_cost - direct});
        }
    }
    std::sort(candidates.begin(), candidates.end(), [](const ShortcutCandidate& lhs,
                                                       const ShortcutCandidate& rhs) {
        return lhs.score > rhs.score;
    });

    CollisionChecker checker = make_audit_checker(audit_robot_, scene_, config_.query);
    int added = 0;
    int box_corridor_edges = 0;
    int segment_edges = 0;
    int pave_added_total = 0;
    int pave_fail = 0;
    int audit_fail = 0;
    int next_id = next_box_id();
    StageContext context = StageContext::from_runtime(config_.runtime);
    for (const auto& candidate : candidates) {
        if (added >= max_edges) {
            break;
        }
        if (find_segment_edge(segment_edges_, candidate.source, candidate.target) != nullptr) {
            continue;
        }
        const BoxNode* source = find_box_by_id(boxes_, candidate.source);
        const BoxNode* target = find_box_by_id(boxes_, candidate.target);
        if (source == nullptr || target == nullptr) {
            continue;
        }
        std::vector<Eigen::VectorXd> waypoints{source->center(), target->center()};
        const auto audit = audit_waypoint_path(waypoints,
                                               checker,
                                               config_.query.audit_resolution,
                                               config_.query.audit_segment_step);
        if (!audit.passed) {
            ++audit_fail;
            continue;
        }
        int pave_added = 0;
        if (!partition_native_mode()) {
            ChainPaveConfig pave_config = config_.connector.pave;
            pave_config.fill_gaps = true;
            pave_config.require_connected_chain = true;
            pave_config.max_chain = std::max(pave_config.max_chain, 64);
            pave_config.max_steps_per_waypoint = std::max(pave_config.max_steps_per_waypoint, 16);
            pave_config.find_free_box.max_depth = std::min(
                std::max(1, config_.database.max_tree_depth),
                std::max(1,
                         config_.query_bridge_pave_depth > 0
                             ? config_.query_bridge_pave_depth
                             : pave_config.find_free_box.max_depth));
            pave_config.adaptive_ffb_depths =
                config_.query_bridge_adaptive_ffb_depths.empty()
                    ? pave_config.adaptive_ffb_depths
                    : config_.query_bridge_adaptive_ffb_depths;
            pave_config.gap_fill_time_budget_ms = std::max(pave_config.gap_fill_time_budget_ms, 75.0);
            pave_config.gap_fill_max_ffb_calls = std::max(pave_config.gap_fill_max_ffb_calls, 192);
            const std::size_t boxes_before_pave = boxes_.size();
            pave_added = chain_pave_along_path(waypoints,
                                               candidate.source,
                                               boxes_,
                                               *oracle_,
                                               adjacency_,
                                               next_id,
                                               context,
                                               pave_config);
            if (pave_added > 0) {
                append_adaptive_partition_boxes(boxes_before_pave,
                                                &last_build_,
                                                "offline_shortcut.pave");
            }
        } else {
            last_build_.diagnostics["offline_shortcut.partition_legacy_chain_pave_skipped"] += 1.0;
        }
        pave_added_total += pave_added;
        int edge_id = -1;
        if (pave_added > 0 &&
            box_only_path_connected_partition_first(candidate.source, candidate.target)) {
            edge_id = add_segment_edge_partition_first(                                       candidate.source,
                                       candidate.target,
                                       waypoints,
                                       SegmentEdgeType::BoxCorridor,
                                       config_.query.audit_resolution,
                                       SegmentEdgeValidation::CollisionChecked,
                                       false,
                                       -1);
            if (edge_id >= 0) {
                ++box_corridor_edges;
            }
        } else {
            ++pave_fail;
            edge_id = add_segment_edge_partition_first(                                       candidate.source,
                                       candidate.target,
                                       std::move(waypoints),
                                       SegmentEdgeType::QueryBridge,
                                       config_.query.audit_resolution,
                                       SegmentEdgeValidation::CollisionChecked,
                                       true,
                                       -1);
            if (edge_id >= 0) {
                ++segment_edges;
            }
        }
        if (edge_id >= 0) {
            ++added;
        }
    }
    if (added > 0) {
        invalidate_query_cache();
    }
    last_build_.diagnostics["offline_shortcut.tested_pairs"] += static_cast<double>(tested_pairs);
    last_build_.diagnostics["offline_shortcut.candidates"] += static_cast<double>(candidates.size());
    last_build_.diagnostics["offline_shortcut.audit_fail"] += static_cast<double>(audit_fail);
    last_build_.diagnostics["offline_shortcut.edges_added"] += static_cast<double>(added);
    last_build_.diagnostics["offline_shortcut.box_corridor_edges_added"] += static_cast<double>(box_corridor_edges);
    last_build_.diagnostics["offline_shortcut.segment_edges_added"] += static_cast<double>(segment_edges);
    last_build_.diagnostics["offline_shortcut.pave_boxes_added"] += static_cast<double>(pave_added_total);
    last_build_.diagnostics["offline_shortcut.pave_fail"] += static_cast<double>(pave_fail);
    sync_adaptive_partition_segment_edges(&last_build_, "offline_shortcut");
    return added;
}

int RBFPlanningForest::anchor_query_endpoint_box(const Eigen::Ref<const Eigen::VectorXd>& point,
                                                 StageContext& context) {
    if (!oracle_) {
        return -1;
    }
    record_portal_membership_policy(context.diagnostics(), config_.portal_membership_policy);
    context.diagnostics().add_counter("portal_membership.global_forest_lookup");
    int existing = -1;
    for (const auto& box : boxes_) {
        if (box.contains(point, config_.query.adjacency_tolerance)) {
            existing = box.id;
            break;
        }
    }
    if (existing < 0 && config_.query.nearest_if_outside) {
        existing = locate_box_partition_first(point, config_.query.nearest_if_outside);
    }
    if (existing >= 0) {
        context.diagnostics().add_counter("query_bridge.endpoint_anchor_already_covered");
        return existing;
    }

    FindFreeBoxOptions options = config_.connector.pave.find_free_box;
    const int requested_anchor_depth =
        config_.query_endpoint_anchor_ffb_depth > 0
            ? config_.query_endpoint_anchor_ffb_depth
            : config_.query_bridge_pave_depth;
    if (requested_anchor_depth > 0) {
        options.max_depth = std::min(std::max(1, config_.database.max_tree_depth),
                                     std::max(1, requested_anchor_depth));
    }
    options.reject_seed_collision = false;
    context.diagnostics().set_value("query_bridge.endpoint_anchor_ffb_depth",
                                    static_cast<double>(options.max_depth));

    BoxNode root_domain;
    root_domain.id = -1;
    root_domain.joint_intervals = oracle_->planning_intervals();
    root_domain.compute_volume();

    if (partition_native_mode()) {
        const std::size_t boxes_before_anchor = boxes_.size();
        StageContext local_context = context;
        auto result = find_free_box_in_domain(point,
                                              root_domain.joint_intervals,
                                              local_context,
                                              options);
        merge_diagnostic_snapshot(context.diagnostics(), local_context.diagnostics().snapshot());
        context.diagnostics().add_counter("query_bridge.endpoint_anchor_calls");
        if (!result.found ||
            !intervals_contain_point_local(result.intervals,
                                           point,
                                           config_.query.adjacency_tolerance)) {
            context.diagnostics().add_counter("query_bridge.endpoint_anchor_ffb_fail");
            return -1;
        }
        context.diagnostics().add_counter("query_bridge.endpoint_anchor_ffb_success");
        auto same_intervals = [](const std::vector<Interval>& lhs,
                                 const std::vector<Interval>& rhs) {
            if (lhs.size() != rhs.size()) {
                return false;
            }
            for (std::size_t dim = 0; dim < lhs.size(); ++dim) {
                if (std::abs(lhs[dim].lo - rhs[dim].lo) > 1e-12 ||
                    std::abs(lhs[dim].hi - rhs[dim].hi) > 1e-12) {
                    return false;
                }
            }
            return true;
        };
        for (const auto& existing_box : boxes_) {
            if (existing_box.tree_id == result.node ||
                same_intervals(existing_box.joint_intervals, result.intervals)) {
                context.diagnostics().add_counter("query_bridge.endpoint_anchor_duplicate_reuse");
                return existing_box.id;
            }
        }
        if (!allow_dynamic_commit(*oracle_, result, config_.grower.commit_policy)) {
            context.diagnostics().add_counter("query_bridge.endpoint_anchor_commit_rejects");
            return -1;
        }
        BoxNode box;
        box.id = next_box_id();
        box.joint_intervals = result.intervals;
        box.seed_config = point;
        box.tree_id = result.node;
        box.parent_box_id = -1;
        box.root_id = box.id;
        box.safety_status = result.validation_detail.safety_status;
        box.strict_audit_required = result.validation_detail.strict_audit_required;
        box.compute_volume();
        oracle_->reserve_node(box.tree_id, box.id);
        const int new_id = box.id;
        boxes_.push_back(box);
        raw_boxes_.push_back(box);
        context.diagnostics().add_counter("query_bridge.endpoint_anchor_boxes_added");
        append_adaptive_partition_boxes(boxes_before_anchor,
                                        &last_build_,
                                        "query_bridge.endpoint_anchor");
        return new_id;
    }

    BoxSpatialIndex box_index;
    box_index.rebuild(boxes_, config_.query.adjacency_tolerance);
    BuildDisjointSet dsu = make_dsu_from_graph(boxes_, adjacency_);
    int next_id = next_box_id();
    QueryRootGrowResult stats;
    auto find_in_domain = [this](const Eigen::VectorXd& seed,
                                 const std::vector<Interval>& domain,
                                 StageContext& local_context,
                                 const FindFreeBoxOptions& local_options) {
        return this->find_free_box_in_domain(seed, domain, local_context, local_options);
    };
    const std::size_t boxes_before_anchor = boxes_.size();
    const int new_id = commit_query_root_box(*oracle_,
                                             options,
                                             config_.grower.commit_policy,
                                             find_in_domain,
                                             point,
                                             root_domain,
                                             -1,
                                             -1,
                                             boxes_,
                                             raw_boxes_,
                                             adjacency_,
                                             box_index,
                                             dsu,
                                             next_id,
                                             context,
                                             stats,
                                             config_.query.adjacency_tolerance);
    context.diagnostics().add_counter("query_bridge.endpoint_anchor_calls");
    context.diagnostics().add_counter("query_bridge.endpoint_anchor_ffb_success", stats.ffb_success);
    context.diagnostics().add_counter("query_bridge.endpoint_anchor_ffb_fail", stats.ffb_fail);
    context.diagnostics().add_counter("query_bridge.endpoint_anchor_commit_rejects", stats.commit_rejects);
    context.diagnostics().add_counter("query_bridge.endpoint_anchor_domain_rejects", stats.domain_rejects);
    context.diagnostics().add_counter("query_bridge.endpoint_anchor_contained_rejects", stats.contained_rejects);
    context.diagnostics().add_counter("query_bridge.endpoint_anchor_adjacency_rejects", stats.adjacency_rejects);
    context.diagnostics().add_counter("query_bridge.endpoint_anchor_boxes_added", stats.boxes_added);
    if (new_id >= 0) {
        append_adaptive_partition_boxes(boxes_before_anchor,
                                        &last_build_,
                                        "query_bridge.endpoint_anchor");
        const BoxNode* anchor_box = find_box_by_id(boxes_, new_id);
        int best_target_id = -1;
        Eigen::VectorXd best_target_point = point;
        double best_dist2 = std::numeric_limits<double>::infinity();
        auto consider_target = [&](bool require_graph_degree) {
            for (const auto& candidate : boxes_) {
                if (candidate.id == new_id || candidate.n_dims() != point.size()) {
                    continue;
                }
                if (require_graph_degree) {
                    const auto graph_it = adjacency_.find(candidate.id);
                    if (graph_it == adjacency_.end() || graph_it->second.empty()) {
                        continue;
                    }
                }
                const Eigen::VectorXd target = closest_point_in_box(candidate, point);
                const double dist2 = (target - point).squaredNorm();
                if (dist2 < best_dist2) {
                    best_dist2 = dist2;
                    best_target_id = candidate.id;
                    best_target_point = target;
                }
            }
        };
        consider_target(true);
        if (best_target_id < 0) {
            consider_target(false);
        }
        const double max_shortlink_length =
            std::max(0.0, env_double_or_default("RBF_ENDPOINT_SHORTLINK_MAX_LENGTH", 0.25));
        if (anchor_box != nullptr &&
            best_target_id >= 0 &&
            best_dist2 > 1e-18 &&
            std::sqrt(best_dist2) <= max_shortlink_length) {
            context.diagnostics().add_counter("query_bridge.endpoint_anchor_shortlink_attempts");
            CollisionChecker checker = make_audit_checker(audit_robot_, scene_, config_.query);
            std::vector<Eigen::VectorXd> waypoints{point, best_target_point};
            const PathAuditCheck audit = audit_waypoint_path(waypoints,
                                                             checker,
                                                             config_.query.audit_resolution,
                                                             config_.query.audit_segment_step);
            if (audit.passed) {
                const int edge_id = add_segment_edge_partition_first(                                                     new_id,
                                                     best_target_id,
                                                     std::move(waypoints),
                                                     SegmentEdgeType::QueryBridge,
                                                     config_.query.audit_resolution,
                                                     SegmentEdgeValidation::CollisionChecked,
                                                     true,
                                                     -1);
                if (edge_id >= 0) {
                    context.diagnostics().add_counter("query_bridge.endpoint_anchor_shortlink_success");
                    context.diagnostics().add_counter("query_bridge.endpoint_anchor_shortlink_length",
                                                      std::sqrt(best_dist2));
                }
            } else {
                context.diagnostics().add_counter("query_bridge.endpoint_anchor_shortlink_audit_fail");
            }
        }
        invalidate_query_cache();
    }
    return new_id;
}

int RBFPlanningForest::locate_box_partition_first(const Eigen::Ref<const Eigen::VectorXd>& point,
                                                  bool nearest_if_outside) const {
    if (adaptive_partition_query_enabled_ && adaptive_partition_ && !adaptive_partition_->empty()) {
        const int partition_box = adaptive_partition_->locate_containing_box(point,
                                                                             nearest_if_outside,
                                                                             config_.query.adjacency_tolerance);
        if (partition_box >= 0) {
            return partition_box;
        }
    }
    if (partition_native_mode()) {
        return -1;
    }
    return locate_containing_box(query_cache(), point, nearest_if_outside);
}

bool RBFPlanningForest::box_only_path_connected_partition_first(int source_box_id,
                                                                int target_box_id) const {
    if (source_box_id < 0 || target_box_id < 0) {
        return false;
    }
    if (adaptive_partition_query_enabled_ &&
        adaptive_partition_ &&
        !adaptive_partition_->empty() &&
        adaptive_partition_->same_island(source_box_id, target_box_id)) {
        return true;
    }
    if (partition_native_mode()) {
        return false;
    }
    return graph_has_certified_box_path_local(boxes_,
                                              adjacency_,
                                              source_box_id,
                                              target_box_id,
                                              config_.query.adjacency_tolerance);
}

bool RBFPlanningForest::overlay_path_connected_partition_first(int source_box_id,
                                                               int target_box_id) const {
    if (source_box_id < 0 || target_box_id < 0) {
        return false;
    }
    if (adaptive_partition_query_enabled_ &&
        adaptive_partition_ &&
        !adaptive_partition_->empty() &&
        adaptive_partition_->same_component_with_overlay(source_box_id, target_box_id)) {
        return true;
    }
    if (partition_native_mode()) {
        return false;
    }
    return graph_has_box_path_local(adjacency_, source_box_id, target_box_id);
}

bool RBFPlanningForest::partition_native_mode() const {
    return has_adaptive_partition_config_ &&
           last_adaptive_partition_config_.planning_backend == "partition_native";
}

int RBFPlanningForest::island_count_partition_first() const {
    if (partition_native_mode() &&
        adaptive_partition_query_enabled_ &&
        adaptive_partition_) {
        return adaptive_partition_->component_count_with_overlay();
    }
    return static_cast<int>(find_islands(adjacency_).size());
}

int RBFPlanningForest::bridge_query(const Eigen::Ref<const Eigen::VectorXd>& start,
                                const Eigen::Ref<const Eigen::VectorXd>& goal) {
    if (boxes_.empty() || !oracle_) {
        return 0;
    }
    QueryResult current = query(start, goal);
    if (current.success && current.repair_count == 0) {
        const double direct = (goal - start).norm();
        const bool graph_only = current.segment_edges_used == 0;
        const bool short_enough =
            direct <= 1e-9 ||
            current.path_length <= std::max(direct * 1.35, direct + 0.35);
        if (graph_only && short_enough) {
            return 0;
        }
    }
    return bridge_query_known_needed(start, goal);
}

int RBFPlanningForest::bridge_query_known_needed(const Eigen::Ref<const Eigen::VectorXd>& start,
                                             const Eigen::Ref<const Eigen::VectorXd>& goal) {
    if (boxes_.empty() || !oracle_) {
        return 0;
    }
    StageContext context = StageContext::from_runtime(config_.runtime);
    struct QueryBridgeDiagnosticsFlush {
        BuildProfile& profile;
        StageContext& context;
        ~QueryBridgeDiagnosticsFlush() {
            for (const auto& [key, value] : context.diagnostics().snapshot()) {
                profile.diagnostics[key] = value;
            }
        }
    } diagnostics_flush{last_build_, context};
    int start_box_id = locate_box_partition_first(start, config_.query.nearest_if_outside);
    if (start_box_id < 0) {
        start_box_id = anchor_query_endpoint_box(start, context);
    }
    if (start_box_id < 0) {
        return 0;
    }
    int goal_box_id = locate_box_partition_first(goal, config_.query.nearest_if_outside);
    if (goal_box_id < 0) {
        goal_box_id = anchor_query_endpoint_box(goal, context);
    }
    if (goal_box_id < 0 || goal_box_id == start_box_id) {
        return 0;
    }
    CollisionChecker checker = make_audit_checker(audit_robot_, scene_, config_.query);
    RRTConnectConfig bridge_rrt = with_query_root_hull_domain(config_.connector.rrt, *oracle_, start, goal);
    bridge_rrt.segment_resolution = std::max(bridge_rrt.segment_resolution, config_.query.audit_resolution);
    const double bridge_distance = (goal - start).norm();
    const bool short_local_bridge = bridge_distance > 0.55 && bridge_distance < 0.85;
    std::vector<RRTConnectConfig> short_local_profiles;
    if (short_local_bridge) {
        bridge_rrt.step_size = std::min(bridge_rrt.step_size, 0.25);
        bridge_rrt.goal_bias = 0.08;
        bridge_rrt.local_sampling_radius =
            bridge_rrt.local_sampling_radius > 0.0
                ? std::min(bridge_rrt.local_sampling_radius, 0.85)
                : 0.85;
        auto add_profile = [&](double step_size, double goal_bias, double radius) {
            RRTConnectConfig profile = bridge_rrt;
            profile.step_size = step_size;
            profile.goal_bias = goal_bias;
            profile.local_sampling_radius = radius;
            profile.shortcut_path = true;
            short_local_profiles.push_back(std::move(profile));
        };
        add_profile(0.25, 0.08, 0.90);
        add_profile(0.50, 0.20, 1.00);
        add_profile(0.35, 0.10, 1.00);
        add_profile(0.25, 0.08, 0.45);
        context.diagnostics().add_counter("query_bridge.short_local_profile");
        context.diagnostics().set_value("query_bridge.short_local_step_size",
                                        bridge_rrt.step_size);
        context.diagnostics().set_value("query_bridge.short_local_goal_bias",
                                        bridge_rrt.goal_bias);
        context.diagnostics().set_value("query_bridge.short_local_radius",
                                        bridge_rrt.local_sampling_radius);
        context.diagnostics().set_value("query_bridge.short_local_profiles",
                                        static_cast<double>(short_local_profiles.size()));
    }
    const int bridge_attempts =
        std::max(1, config_.connector.max_pairs_per_gap);
    const int run_seed = config_.grower.rng_seed;
    const int bridge_seed_base = derived_planner_seed(run_seed, kSeedQueryBridgeOffset);
    context.diagnostics().set_value("query_bridge.run_seed", static_cast<double>(run_seed));
    context.diagnostics().set_value("query_bridge.seed_base", static_cast<double>(bridge_seed_base));
    auto waypoint_path = best_audited_rrt_bridge_path(start,
                                                      goal,
                                                      checker,
                                                      audit_robot_,
                                                      context,
                                                      bridge_rrt,
                                                      bridge_attempts,
                                                      config_.connector.per_pair_timeout_ms * bridge_attempts,
                                                      bridge_seed_base,
                                                      config_.query.audit_resolution,
                                                      config_.query.audit_segment_step,
                                                      short_local_profiles.empty() ? nullptr : &short_local_profiles,
                                                      short_local_bridge ? 1 : 7919);
    if (waypoint_path.empty()) {
        return 0;
    }
    return bridge_query_with_waypoint_path(start,
                                           goal,
                                           waypoint_path,
                                           short_local_bridge,
                                           bridge_rrt);
}

int RBFPlanningForest::bridge_query_with_waypoint_path(
    const Eigen::Ref<const Eigen::VectorXd>& start,
    const Eigen::Ref<const Eigen::VectorXd>& goal,
    const std::vector<Eigen::VectorXd>& waypoint_path,
    bool short_local_bridge,
    const RRTConnectConfig& bridge_rrt,
    int query_index,
    bool allow_residual_segments) {
    if (waypoint_path.empty() || boxes_.empty() || !oracle_) {
        return 0;
    }
    auto finish_bridge = [&](int added_total) {
        sync_adaptive_partition_segment_edges(&last_build_, "query_bridge");
        if (added_total > 0) {
            refresh_adaptive_partition_diagnostics(&last_build_);
        }
        return added_total;
    };
    const int start_box_id = locate_box_partition_first(start, config_.query.nearest_if_outside);
    if (start_box_id < 0) {
        return 0;
    }
    const int goal_box_id = locate_box_partition_first(goal, config_.query.nearest_if_outside);
    if (goal_box_id < 0 || goal_box_id == start_box_id) {
        return 0;
    }
    StageContext context = StageContext::from_runtime(config_.runtime);
    const bool scene_reusable_query_bridge_edges =
        env_int_or_default("RBF_QUERY_BRIDGE_SCENE_REUSABLE_EDGES", 0) != 0;
    const int bridge_edge_query_index =
        scene_reusable_query_bridge_edges ? -1 : query_index;
    context.diagnostics().set_value("query_bridge.scene_reusable_edges",
                                    scene_reusable_query_bridge_edges ? 1.0 : 0.0);
    struct QueryBridgePaveDiagnosticsFlush {
        BuildProfile& profile;
        StageContext& context;
        ~QueryBridgePaveDiagnosticsFlush() {
            for (const auto& [key, value] : context.diagnostics().snapshot()) {
                profile.diagnostics[key] = value;
            }
        }
    } pave_diagnostics_flush{last_build_, context};
    CollisionChecker checker = make_audit_checker(audit_robot_, scene_, config_.query);
    int direct_segment_edges_added = 0;
    int box_corridor_edges_added = 0;
    const bool defer_query_segment_edge = true;
    const double query_bridge_depth_failures_before =
        boundary_max_depth_failure_count_local(context);
    int next_id = next_box_id();
    auto append_partition_after_pave = [&](std::size_t boxes_before, const char* prefix) {
        if (boxes_.size() > boxes_before) {
            append_adaptive_partition_boxes(boxes_before, &last_build_, prefix);
        }
    };
    auto waypoint_length = [](const std::vector<Eigen::VectorXd>& path) {
        double total = 0.0;
        for (std::size_t i = 1; i < path.size(); ++i) {
            total += (path[i] - path[i - 1]).norm();
        }
        return total;
    };
    auto capped_ffb_depth = [&](int requested_depth) {
        const int max_tree_depth = std::max(1, config_.database.max_tree_depth);
        return std::min(max_tree_depth, std::max(1, requested_depth));
    };
    const int query_bridge_ffb_depth = capped_ffb_depth(
        config_.query_bridge_pave_depth > 0
            ? config_.query_bridge_pave_depth
            : config_.connector.pave.find_free_box.max_depth);
    context.diagnostics().set_value("query_bridge.pave_ffb_depth",
                                    static_cast<double>(query_bridge_ffb_depth));
    auto set_query_bridge_task_value = [&](const std::string& suffix, double value) {
        if (query_index < 0) {
            return;
        }
        context.diagnostics().set_value(
            "query_bridge.batch_task." + std::to_string(query_index) + "." + suffix,
            value);
    };
    std::vector<Eigen::VectorXd> corridor_path = waypoint_path;
    auto locate_box_linear = [&](const Eigen::Ref<const Eigen::VectorXd>& point) {
        for (const auto& box : boxes_) {
            if (intervals_contain_point_local(box.joint_intervals,
                                              point,
                                              config_.query.adjacency_tolerance)) {
                return box.id;
            }
        }
        return -1;
    };
    auto locate_query_boxes = [&]() {
        using Clock = std::chrono::steady_clock;
        const auto locate_t0 = Clock::now();
        int source_box_id = -1;
        int target_box_id = -1;
        if (partition_native_mode()) {
            source_box_id =
                locate_box_partition_first(start, config_.query.nearest_if_outside);
            target_box_id =
                locate_box_partition_first(goal, config_.query.nearest_if_outside);
            context.diagnostics().add_counter(
                "query_bridge.locate_query_boxes_partition_first");
        } else {
            source_box_id = locate_box_linear(start);
            target_box_id = locate_box_linear(goal);
            if ((source_box_id < 0 || target_box_id < 0) && config_.query.nearest_if_outside) {
                context.diagnostics().add_counter(
                    "query_bridge.locate_query_boxes_cache_fallbacks");
                invalidate_query_cache();
                source_box_id =
                    locate_box_partition_first(start, config_.query.nearest_if_outside);
                target_box_id =
                    locate_box_partition_first(goal, config_.query.nearest_if_outside);
            }
        }
        context.diagnostics().record_timing(
            "query_bridge.locate_query_boxes_ms",
            std::chrono::duration<double, std::milli>(Clock::now() -
                                                      locate_t0).count());
        return std::pair<int, int>{source_box_id, target_box_id};
    };
    auto query_boxes_connected = [&](int source_box_id, int target_box_id) {
        if (source_box_id < 0 || target_box_id < 0) {
            return false;
        }
        return box_only_path_connected_partition_first(source_box_id, target_box_id);
    };
    auto try_reverse_boundary_pave =
        [&](const ChainPaveConfig& forward_config,
            int forward_added,
            int& accumulated_added) -> std::pair<int, int> {
        auto [source_box_id, target_box_id] = locate_query_boxes();
        if (query_boxes_connected(source_box_id, target_box_id)) {
            return {source_box_id, target_box_id};
        }
        if (partition_native_mode()) {
            context.diagnostics().add_counter(
                "query_bridge.partition_legacy_reverse_chain_pave_skipped");
            return {source_box_id, target_box_id};
        }
        const int remaining_chain = forward_config.max_chain - std::max(0, forward_added);
        if (target_box_id < 0 || remaining_chain <= 0) {
            return {source_box_id, target_box_id};
        }
        ChainPaveConfig reverse_config = forward_config;
        reverse_config.max_chain = remaining_chain;
        std::vector<Eigen::VectorXd> reverse_path(corridor_path.rbegin(),
                                                  corridor_path.rend());
        context.diagnostics().add_counter("query_bridge.reverse_boundary_pave_attempts");
        const std::size_t boxes_before_reverse = boxes_.size();
        const int reverse_added = chain_pave_along_path(reverse_path,
                                                        target_box_id,
                                                        boxes_,
                                                        *oracle_,
                                                        adjacency_,
                                                        next_id,
                                                        context,
                                                        reverse_config);
        if (reverse_added > 0) {
            append_partition_after_pave(boxes_before_reverse,
                                        "query_bridge.reverse_boundary_pave");
            accumulated_added += reverse_added;
            context.diagnostics().add_counter("query_bridge.reverse_boundary_pave_added",
                                              static_cast<double>(reverse_added));
            context.diagnostics().add_counter(
                "query_bridge.full_adjacency_rebuilds_avoided");
            invalidate_query_cache();
        }
        return locate_query_boxes();
    };
    const bool bridge_internal_simplify =
        env_int_or_default("RBF_QUERY_BRIDGE_INTERNAL_SIMPLIFY", 0) != 0;
    context.diagnostics().set_value("query_bridge.internal_simplify_enabled",
                                    bridge_internal_simplify ? 1.0 : 0.0);
    if (bridge_internal_simplify &&
        config_.query.final_rrt_simplify &&
        config_.query.final_rrt_simplify_timeout_ms > 0.0 &&
        corridor_path.size() >= 2) {
        using Clock = std::chrono::steady_clock;
        const auto simplify_t0 = Clock::now();
        auto elapsed_ms = [&]() {
            return std::chrono::duration<double, std::milli>(Clock::now() -
                                                             simplify_t0)
                .count();
        };
        RRTConnectConfig simplify_config = config_.connector.rrt;
        simplify_config.max_iters =
            std::max(1, config_.query.final_rrt_simplify_max_iters);
        simplify_config.segment_resolution =
            std::max(simplify_config.segment_resolution,
                     config_.query.audit_resolution);
        simplify_config.segment_step = config_.query.audit_segment_step;
        simplify_config.shortcut_path = true;
        const int attempts = std::max(1, config_.query.final_rrt_simplify_attempts);
        double best_length = waypoint_length(corridor_path);
        for (int attempt = 0; attempt < attempts; ++attempt) {
            const double remaining_ms =
                config_.query.final_rrt_simplify_timeout_ms - elapsed_ms();
            if (remaining_ms <= 0.0) {
                break;
            }
            const int attempts_left = attempts - attempt;
            simplify_config.timeout_ms =
                std::max(1.0, remaining_ms / static_cast<double>(attempts_left));
            const int simplify_seed =
                derived_planner_seed(config_.grower.rng_seed,
                                     kSeedBridgeSimplifyOffset,
                                     attempt);
            auto candidate = rrt_connect(start,
                                         goal,
                                         checker,
                                         audit_robot_,
                                         simplify_config,
                                         simplify_seed);
            if (candidate.empty()) {
                continue;
            }
            const double candidate_length = waypoint_length(candidate);
            if (candidate_length + 1e-12 >= best_length) {
                continue;
            }
            if (audit_waypoint_path(candidate,
                                    checker,
                                    config_.query.audit_resolution,
                                    config_.query.audit_segment_step)
                    .passed) {
                best_length = candidate_length;
                corridor_path = std::move(candidate);
            }
        }
    }
    int dense_repair_added = 0;
    bool dense_repair_attempted = false;
    const double audited_bridge_length = waypoint_length(corridor_path);
    const double dense_box_corridor_max_length =
        std::max(0.0, env_double_or_default("RBF_QUERY_BRIDGE_DIRECT_MAX_LENGTH", 6.5));
    const bool dense_box_corridor_candidate =
        defer_query_segment_edge &&
        audited_bridge_length > 0.0 &&
        audited_bridge_length <= dense_box_corridor_max_length;
    auto try_direct_ffb_corridor = [&]() -> int {
        using Clock = std::chrono::steady_clock;
        const auto t0 = Clock::now();
        const bool graphless_direct_corridor = partition_native_mode();
        const std::size_t boxes_before_direct_corridor = boxes_.size();
        auto finish_direct_corridor = [&](int value) {
            if (boxes_.size() > boxes_before_direct_corridor) {
                append_adaptive_partition_boxes(boxes_before_direct_corridor,
                                                &last_build_,
                                                "query_bridge.direct_corridor");
            }
            sync_adaptive_partition_segment_edges(&last_build_, "query_bridge.direct_corridor");
            return value;
        };
        auto refresh_direct_corridor_partition = [&]() {
            if (boxes_.size() > boxes_before_direct_corridor) {
                append_adaptive_partition_boxes(boxes_before_direct_corridor,
                                                &last_build_,
                                                "query_bridge.direct_corridor");
            }
            sync_adaptive_partition_segment_edges(&last_build_,
                                                  "query_bridge.direct_corridor");
        };
        const double audit_step = config_.query.audit_segment_step > 0.0
            ? config_.query.audit_segment_step
            : 0.01;
        const double base_sample_step =
            env_double_or_default("RBF_QUERY_BRIDGE_DIRECT_SAMPLE_STEP",
                                  audit_step);
        const double sample_step =
            std::max(1e-4,
                     env_indexed_double_or_default("RBF_QUERY_BRIDGE_DIRECT_SAMPLE_STEP",
                                                   query_index,
                                                   base_sample_step));
        context.diagnostics().set_value("query_bridge.direct_corridor_sample_step",
                                        sample_step);
        const std::vector<Eigen::VectorXd> samples =
            densify_waypoint_path_local(corridor_path, sample_step);
        if (samples.size() < 2) {
            return 0;
        }

        const bool use_partition_cover_index =
            graphless_direct_corridor &&
            adaptive_partition_query_enabled_ &&
            adaptive_partition_;
        const bool use_partition_neighbor_candidates =
            use_partition_cover_index &&
            env_int_or_default("RBF_QUERY_BRIDGE_PARTITION_NEIGHBOR_CANDIDATES", 0) != 0;
        const bool immediate_partition_append =
            use_partition_cover_index &&
            env_int_or_default("RBF_QUERY_BRIDGE_DIRECT_APPEND_PARTITION_IMMEDIATE", 0) != 0;
        const bool detailed_direct_timing =
            env_int_or_default("RBF_QUERY_BRIDGE_DETAILED_TIMING", 0) != 0;
        context.diagnostics().set_value(
            "query_bridge.direct_corridor_partition_neighbor_candidates_enabled",
            use_partition_neighbor_candidates ? 1.0 : 0.0);
        context.diagnostics().set_value(
            "query_bridge.direct_corridor_immediate_partition_append_enabled",
            immediate_partition_append ? 1.0 : 0.0);
        context.diagnostics().set_value(
            "query_bridge.direct_corridor_detailed_timing_enabled",
            detailed_direct_timing ? 1.0 : 0.0);
        BoxSpatialIndex direct_box_index;
        if (!use_partition_cover_index) {
            direct_box_index.rebuild(boxes_, config_.query.adjacency_tolerance);
        }
        std::unordered_map<int, int> box_id_to_index;
        if (use_partition_neighbor_candidates) {
            box_id_to_index.reserve(boxes_.size() * 2);
            for (std::size_t box_index = 0; box_index < boxes_.size(); ++box_index) {
                box_id_to_index.emplace(boxes_[box_index].id, static_cast<int>(box_index));
            }
        }
        std::vector<int> corridor_new_box_indices;
        std::vector<std::vector<int>> sample_layers(samples.size());
        std::vector<bool> covered(samples.size(), false);
        auto mark_from_index = [&](std::size_t from_index) {
            const auto mark_t0 = Clock::now();
            int changed = 0;
            for (std::size_t sample_index = 0; sample_index < samples.size(); ++sample_index) {
                if (from_index == 0) {
                    std::vector<int> candidates;
                    if (use_partition_cover_index) {
                        candidates = adaptive_partition_->covering_box_indices(
                            samples[sample_index],
                            config_.query.adjacency_tolerance);
                    } else {
                        candidates = direct_box_index.point_candidates(samples[sample_index]);
                    }
                    for (int box_index : candidates) {
                        if (box_index < 0 || box_index >= static_cast<int>(boxes_.size())) {
                            continue;
                        }
                        if (!intervals_contain_point_local(
                                boxes_[static_cast<std::size_t>(box_index)].joint_intervals,
                                samples[sample_index],
                                config_.query.adjacency_tolerance)) {
                            continue;
                        }
                        auto& layer = sample_layers[sample_index];
                        if (std::find(layer.begin(), layer.end(), box_index) == layer.end()) {
                            layer.push_back(box_index);
                        }
                        if (!covered[sample_index]) {
                            covered[sample_index] = true;
                            changed += 1;
                        }
                    }
                    continue;
                }
                for (std::size_t box_index = from_index; box_index < boxes_.size(); ++box_index) {
                    if (!intervals_contain_point_local(boxes_[box_index].joint_intervals,
                                                       samples[sample_index],
                                                       config_.query.adjacency_tolerance)) {
                        continue;
                    }
                    auto& layer = sample_layers[sample_index];
                    const int index_value = static_cast<int>(box_index);
                    if (std::find(layer.begin(), layer.end(), index_value) == layer.end()) {
                        layer.push_back(index_value);
                    }
                    if (!covered[sample_index]) {
                        covered[sample_index] = true;
                        changed += 1;
                    }
                    if (from_index == 0) {
                        break;
                    }
                }
            }
            context.diagnostics().record_timing(
                from_index == 0
                    ? "query_bridge.direct_corridor_mark_initial_ms"
                    : "query_bridge.direct_corridor_mark_incremental_ms",
                std::chrono::duration<double, std::milli>(Clock::now() - mark_t0).count());
            return changed;
        };
        mark_from_index(0);

        struct LocalDsu {
            std::vector<int> parent;
            explicit LocalDsu(std::size_t count = 0) : parent(count) {
                for (std::size_t index = 0; index < parent.size(); ++index) {
                    parent[index] = static_cast<int>(index);
                }
            }
            int add() {
                const int id = static_cast<int>(parent.size());
                parent.push_back(id);
                return id;
            }
            int find(int value) {
                int root = value;
                while (parent[static_cast<std::size_t>(root)] != root) {
                    root = parent[static_cast<std::size_t>(root)];
                }
                while (parent[static_cast<std::size_t>(value)] != value) {
                    const int next = parent[static_cast<std::size_t>(value)];
                    parent[static_cast<std::size_t>(value)] = root;
                    value = next;
                }
                return root;
            }
            void unite(int lhs, int rhs) {
                if (lhs < 0 || rhs < 0 ||
                    lhs >= static_cast<int>(parent.size()) ||
                    rhs >= static_cast<int>(parent.size())) {
                    return;
                }
                const int left = find(lhs);
                const int right = find(rhs);
                if (left != right) {
                    parent[static_cast<std::size_t>(right)] = left;
                }
            }
        };
        LocalDsu dsu(boxes_.size());
        double transition_connected_ms = 0.0;
        double bad_transitions_ms = 0.0;
        double current_cover_ms = 0.0;
        double current_cover_partition_ms = 0.0;
        double current_cover_corridor_scan_ms = 0.0;
        double current_cover_direct_index_ms = 0.0;
        double duplicate_lookup_ms = 0.0;
        double commit_total_ms = 0.0;
        double commit_dynamic_policy_ms = 0.0;
        double commit_partition_append_ms = 0.0;
        double assimilate_sample_scan_ms = 0.0;
        double assimilate_candidate_build_ms = 0.0;
        double assimilate_adjacency_ms = 0.0;
        double segment_insert_ms = 0.0;
        double direct_task_build_ms = 0.0;
        double direct_loop_ms = 0.0;
        double repair_loop_ms = 0.0;
        double adaptive_loop_ms = 0.0;
        double lateral_loop_ms = 0.0;
        double residual_segment_loop_ms = 0.0;
        int transition_connected_calls = 0;
        int bad_transitions_calls = 0;
        int current_cover_calls = 0;
        int duplicate_lookup_calls = 0;
        int commit_calls = 0;
        int assimilate_calls = 0;
        int segment_insert_calls = 0;
        auto transition_connected = [&](int transition) {
            const auto timing_t0 = detailed_direct_timing ? Clock::now() : Clock::time_point{};
            auto finish = [&](bool value) {
                if (detailed_direct_timing) {
                    transition_connected_ms +=
                        std::chrono::duration<double, std::milli>(Clock::now() - timing_t0).count();
                    transition_connected_calls += 1;
                }
                return value;
            };
            if (transition < 0 || transition + 1 >= static_cast<int>(sample_layers.size())) {
                return finish(false);
            }
            const auto& lhs_layer = sample_layers[static_cast<std::size_t>(transition)];
            const auto& rhs_layer = sample_layers[static_cast<std::size_t>(transition + 1)];
            if (lhs_layer.empty() || rhs_layer.empty()) {
                return finish(false);
            }
            for (int lhs : lhs_layer) {
                const int root = dsu.find(lhs);
                for (int rhs : rhs_layer) {
                    if (root == dsu.find(rhs)) {
                        return finish(true);
                    }
                }
            }
            return finish(false);
        };
        auto bad_transitions = [&]() {
            const auto timing_t0 = detailed_direct_timing ? Clock::now() : Clock::time_point{};
            std::vector<int> bad;
            for (std::size_t sample_index = 0; sample_index + 1 < sample_layers.size(); ++sample_index) {
                if (!transition_connected(static_cast<int>(sample_index))) {
                    bad.push_back(static_cast<int>(sample_index));
                }
            }
            if (detailed_direct_timing) {
                bad_transitions_ms +=
                    std::chrono::duration<double, std::milli>(Clock::now() - timing_t0).count();
                bad_transitions_calls += 1;
            }
            return bad;
        };
        auto endpoint_layers_connected = [&]() {
            if (sample_layers.empty() ||
                sample_layers.front().empty() ||
                sample_layers.back().empty()) {
                return false;
            }
            const int root = dsu.find(sample_layers.front().front());
            for (int index : sample_layers.back()) {
                if (root == dsu.find(index)) {
                    return true;
                }
            }
            return false;
        };
        auto direct_boxes_adjacent = [&](int lhs, int rhs) {
            if (lhs < 0 || rhs < 0 ||
                lhs >= static_cast<int>(boxes_.size()) ||
                rhs >= static_cast<int>(boxes_.size())) {
                return false;
            }
            const int lhs_box_id = boxes_[static_cast<std::size_t>(lhs)].id;
            const int rhs_box_id = boxes_[static_cast<std::size_t>(rhs)].id;
            if (graphless_direct_corridor &&
                use_partition_cover_index &&
                adaptive_partition_ &&
                adaptive_partition_->contains_box_id(lhs_box_id) &&
                adaptive_partition_->contains_box_id(rhs_box_id)) {
                context.diagnostics().add_counter(
                    "query_bridge.direct_corridor_partition_neighbor_tests");
                const bool adjacent = adaptive_partition_->boxes_are_neighbors(lhs_box_id,
                                                                               rhs_box_id);
                if (adjacent) {
                    context.diagnostics().add_counter(
                        "query_bridge.direct_corridor_partition_neighbor_hits");
                }
                return adjacent;
            }
            return boxes_connected(boxes_[static_cast<std::size_t>(lhs)],
                                   boxes_[static_cast<std::size_t>(rhs)],
                                   config_.query.adjacency_tolerance);
        };
        auto initialize_dsu = [&]() {
            const auto dsu_t0 = Clock::now();
            for (const auto& layer : sample_layers) {
                if (layer.empty()) {
                    continue;
                }
                const int root = layer.front();
                for (int index : layer) {
                    dsu.unite(root, index);
                }
            }
            for (std::size_t sample_index = 0; sample_index + 1 < sample_layers.size(); ++sample_index) {
                for (int lhs : sample_layers[sample_index]) {
                    for (int rhs : sample_layers[sample_index + 1]) {
                        if (direct_boxes_adjacent(lhs, rhs)) {
                            dsu.unite(lhs, rhs);
                            if (!graphless_direct_corridor) {
                                append_local_edge(adjacency_,
                                                  boxes_[static_cast<std::size_t>(lhs)].id,
                                                  boxes_[static_cast<std::size_t>(rhs)].id);
                            }
                        }
                    }
                }
            }
            context.diagnostics().record_timing(
                "query_bridge.direct_corridor_initialize_dsu_ms",
                std::chrono::duration<double, std::milli>(Clock::now() - dsu_t0).count());
        };
        initialize_dsu();

        std::unordered_map<OracleNodeId, int> node_to_box_index;
        node_to_box_index.reserve(boxes_.size() + samples.size());
        for (std::size_t box_index = 0; box_index < boxes_.size(); ++box_index) {
            const auto node = static_cast<OracleNodeId>(boxes_[box_index].tree_id);
            if (node != kInvalidOracleNodeId &&
                node_to_box_index.find(node) == node_to_box_index.end()) {
                node_to_box_index[node] = static_cast<int>(box_index);
            }
        }
        auto find_duplicate_box_index = [&](OracleNodeId node,
                                            const std::vector<Interval>& intervals) {
            const auto timing_t0 = detailed_direct_timing ? Clock::now() : Clock::time_point{};
            auto finish = [&](int value) {
                if (detailed_direct_timing) {
                    duplicate_lookup_ms +=
                        std::chrono::duration<double, std::milli>(Clock::now() - timing_t0).count();
                    duplicate_lookup_calls += 1;
                }
                return value;
            };
            if (node != kInvalidOracleNodeId) {
                const auto node_it = node_to_box_index.find(node);
                if (node_it != node_to_box_index.end()) {
                    return finish(node_it->second);
                }
                return finish(-1);
            }
            for (std::size_t box_index = 0; box_index < boxes_.size(); ++box_index) {
                const auto& box = boxes_[box_index];
                if (box.joint_intervals.size() != intervals.size()) {
                    continue;
                }
                bool same = true;
                for (std::size_t dim = 0; dim < intervals.size(); ++dim) {
                    if (std::abs(box.joint_intervals[dim].lo - intervals[dim].lo) > 1e-12 ||
                        std::abs(box.joint_intervals[dim].hi - intervals[dim].hi) > 1e-12) {
                        same = false;
                        break;
                    }
                }
                if (same) {
                    return finish(static_cast<int>(box_index));
                }
            }
            return finish(-1);
        };
        std::vector<int> repair_indices;
        auto assimilate_box = [&](int box_index, int transition_hint) {
            if (detailed_direct_timing) {
                assimilate_calls += 1;
            }
            const auto assimilate_t0 = Clock::now();
            const int box_id = boxes_[static_cast<std::size_t>(box_index)].id;
            if (!graphless_direct_corridor) {
                adjacency_[box_id];
            }
            const auto sample_scan_t0 = detailed_direct_timing ? Clock::now() : Clock::time_point{};
            for (std::size_t sample_index = 0; sample_index < samples.size(); ++sample_index) {
                if (!intervals_contain_point_local(boxes_[static_cast<std::size_t>(box_index)].joint_intervals,
                                                   samples[sample_index],
                                                   config_.query.adjacency_tolerance)) {
                    continue;
                }
                auto& layer = sample_layers[sample_index];
                if (!layer.empty()) {
                    dsu.unite(box_index, layer.front());
                }
                if (std::find(layer.begin(), layer.end(), box_index) == layer.end()) {
                    layer.push_back(box_index);
                }
                covered[sample_index] = true;
            }
            if (detailed_direct_timing) {
                assimilate_sample_scan_ms +=
                    std::chrono::duration<double, std::milli>(Clock::now() - sample_scan_t0).count();
            }
            const auto candidate_build_t0 = detailed_direct_timing ? Clock::now() : Clock::time_point{};
            std::vector<int> candidates;
            auto add_layer = [&](int layer_index) {
                if (layer_index < 0 || layer_index >= static_cast<int>(sample_layers.size())) {
                    return;
                }
                const auto& layer = sample_layers[static_cast<std::size_t>(layer_index)];
                candidates.insert(candidates.end(), layer.begin(), layer.end());
            };
            add_layer(transition_hint - 1);
            add_layer(transition_hint);
            add_layer(transition_hint + 1);
            add_layer(transition_hint + 2);
            candidates.insert(candidates.end(), repair_indices.begin(), repair_indices.end());
            if (use_partition_neighbor_candidates && adaptive_partition_) {
                const auto partition_neighbor_ids =
                    adaptive_partition_->adjacent_box_ids(
                        boxes_[static_cast<std::size_t>(box_index)],
                        config_.query.adjacency_tolerance);
                context.diagnostics().add_counter(
                    "query_bridge.direct_corridor_partition_neighbor_candidates",
                    static_cast<double>(partition_neighbor_ids.size()));
                for (int neighbor_box_id : partition_neighbor_ids) {
                    const auto index_it = box_id_to_index.find(neighbor_box_id);
                    if (index_it != box_id_to_index.end()) {
                        candidates.push_back(index_it->second);
                    }
                }
            }
            std::sort(candidates.begin(), candidates.end());
            candidates.erase(std::unique(candidates.begin(), candidates.end()), candidates.end());
            if (detailed_direct_timing) {
                assimilate_candidate_build_ms +=
                    std::chrono::duration<double, std::milli>(Clock::now() - candidate_build_t0).count();
            }
            const auto adjacency_t0 = detailed_direct_timing ? Clock::now() : Clock::time_point{};
            int local_edges = 0;
            for (int candidate : candidates) {
                if (candidate == box_index ||
                    candidate < 0 ||
                    candidate >= static_cast<int>(boxes_.size())) {
                    continue;
                }
                if (direct_boxes_adjacent(box_index, candidate)) {
                    dsu.unite(box_index, candidate);
                    bool edge_counted = true;
                    if (!graphless_direct_corridor) {
                        const std::size_t before = adjacency_[box_id].size();
                        append_local_edge(adjacency_,
                                          box_id,
                                          boxes_[static_cast<std::size_t>(candidate)].id);
                        edge_counted = adjacency_[box_id].size() > before;
                    }
                    if (edge_counted) {
                        local_edges += 1;
                    }
                }
            }
            if (detailed_direct_timing) {
                assimilate_adjacency_ms +=
                    std::chrono::duration<double, std::milli>(Clock::now() - adjacency_t0).count();
            }
            context.diagnostics().add_counter(
                "query_bridge.direct_corridor_incremental_adjacency_checks",
                static_cast<double>(candidates.size()));
            context.diagnostics().add_counter(
                "query_bridge.direct_corridor_incremental_adjacency_edges",
                static_cast<double>(local_edges));
            context.diagnostics().add_counter(
                "query_bridge.direct_corridor_full_adjacency_scans_avoided");
            context.diagnostics().record_timing(
                "query_bridge.direct_corridor_assimilate_ms",
                std::chrono::duration<double, std::milli>(Clock::now() - assimilate_t0).count());
        };
        bool adopt_certified_subchain_attempted = false;
        auto try_adopt_certified_subchain = [&](int source_box_id,
                                                int target_box_id,
                                                const char* reason) -> int {
            const auto adopt_t0 = Clock::now();
            auto& diagnostics = context.diagnostics();
            auto finish_adopt = [&](int value) {
                const double elapsed =
                    std::chrono::duration<double, std::milli>(Clock::now() - adopt_t0).count();
                diagnostics.record_timing("query_bridge.hipac_promote_transition.ms_total",
                                          elapsed);
                diagnostics.add_counter("query_bridge.hipac_promote_transition.ms_total",
                                        elapsed);
                set_query_bridge_task_value("hipac_promote_transition_ms", elapsed);
                if (value > 0) {
                    set_query_bridge_task_value("hipac_promote_transition_added",
                                                static_cast<double>(value));
                }
                if (reason != nullptr) {
                    diagnostics.set_value(std::string("query_bridge.hipac_promote_transition.reason.") +
                                              reason,
                                          1.0);
                }
                return value;
            };
            if (adopt_certified_subchain_attempted) {
                diagnostics.add_counter("query_bridge.hipac_promote_transition.skipped_repeat");
                return finish_adopt(0);
            }
            adopt_certified_subchain_attempted = true;
            if (!last_adaptive_partition_config_.hipac_online_connectivity ||
                !last_adaptive_partition_config_.hipac_promote_transition_slices ||
                last_adaptive_partition_config_.hipac_promote_transition_max_attempts_per_query <= 0 ||
                !partition_native_mode() ||
                source_box_id < 0 ||
                target_box_id < 0 ||
                source_box_id == target_box_id) {
                diagnostics.add_counter("query_bridge.hipac_promote_transition.disabled");
                return finish_adopt(0);
            }
            const bool target_index =
                csv_index_list_contains(
                    last_adaptive_partition_config_.hipac_promote_transition_target_query_indices,
                    query_index);
            if (!target_index) {
                diagnostics.add_counter("query_bridge.hipac_promote_transition.target_rejects");
                return finish_adopt(0);
            }
            diagnostics.add_counter("query_bridge.hipac_promote_transition.attempts");

            const int min_boxes =
                std::max(1, last_adaptive_partition_config_.hipac_promote_transition_min_boxes);
            const int max_boxes =
                std::max(min_boxes, last_adaptive_partition_config_.hipac_promote_transition_max_boxes);
            const BoxNode* source_box_ptr = find_box_by_id(boxes_, source_box_id);
            const BoxNode* target_box_ptr = find_box_by_id(boxes_, target_box_id);
            if (source_box_ptr == nullptr || target_box_ptr == nullptr) {
                diagnostics.add_counter("query_bridge.hipac_promote_transition.missing_endpoint_box");
                return finish_adopt(0);
            }
            const auto source_it =
                std::find_if(boxes_.begin(), boxes_.end(), [&](const BoxNode& box) {
                    return box.id == source_box_id;
                });
            const auto target_it =
                std::find_if(boxes_.begin(), boxes_.end(), [&](const BoxNode& box) {
                    return box.id == target_box_id;
                });
            if (source_it == boxes_.end() || target_it == boxes_.end()) {
                diagnostics.add_counter("query_bridge.hipac_promote_transition.missing_endpoint_index");
                return finish_adopt(0);
            }
            const int source_index = static_cast<int>(std::distance(boxes_.begin(), source_it));
            const int target_index_box = static_cast<int>(std::distance(boxes_.begin(), target_it));

            std::unordered_map<int, int> first_sample_by_box;
            first_sample_by_box.reserve(boxes_.size() - boxes_before_direct_corridor + 8);
            for (std::size_t sample_index = 0; sample_index < sample_layers.size(); ++sample_index) {
                for (int box_index : sample_layers[sample_index]) {
                    if (box_index < static_cast<int>(boxes_before_direct_corridor) ||
                        box_index < 0 ||
                        box_index >= static_cast<int>(boxes_.size())) {
                        continue;
                    }
                    first_sample_by_box.emplace(box_index, static_cast<int>(sample_index));
                }
            }
            std::vector<int> candidate_indices;
            candidate_indices.reserve(first_sample_by_box.size());
            for (const auto& [box_index, sample_index] : first_sample_by_box) {
                (void)sample_index;
                if (box_index == source_index || box_index == target_index_box) {
                    continue;
                }
                const auto& box = boxes_[static_cast<std::size_t>(box_index)];
                if (box.safety_status != BoxSafetyStatus::CertifiedFree ||
                    box.strict_audit_required) {
                    diagnostics.add_counter(
                        "query_bridge.hipac_promote_transition.reject_non_certified");
                    continue;
                }
                candidate_indices.push_back(box_index);
            }
            std::sort(candidate_indices.begin(),
                      candidate_indices.end(),
                      [&](int lhs, int rhs) {
                const int lhs_sample = first_sample_by_box.count(lhs) > 0
                    ? first_sample_by_box[lhs]
                    : std::numeric_limits<int>::max();
                const int rhs_sample = first_sample_by_box.count(rhs) > 0
                    ? first_sample_by_box[rhs]
                    : std::numeric_limits<int>::max();
                if (lhs_sample != rhs_sample) {
                    return lhs_sample < rhs_sample;
                }
                return lhs < rhs;
            });
            if (static_cast<int>(candidate_indices.size()) < min_boxes) {
                diagnostics.add_counter("query_bridge.hipac_promote_transition.too_few_boxes");
                diagnostics.add_counter("query_bridge.hipac_promote_transition.candidate_boxes",
                                        static_cast<double>(candidate_indices.size()));
                return finish_adopt(0);
            }
            if (static_cast<int>(candidate_indices.size()) > max_boxes) {
                candidate_indices.resize(static_cast<std::size_t>(max_boxes));
            }
            diagnostics.add_counter("query_bridge.hipac_promote_transition.candidate_boxes",
                                    static_cast<double>(candidate_indices.size()));

            std::vector<int> local_indices;
            local_indices.reserve(candidate_indices.size() + 2);
            local_indices.push_back(source_index);
            local_indices.insert(local_indices.end(),
                                 candidate_indices.begin(),
                                 candidate_indices.end());
            local_indices.push_back(target_index_box);
            const int local_source = 0;
            const int local_target = static_cast<int>(local_indices.size()) - 1;
            std::vector<std::vector<int>> local_adj(local_indices.size());
            int exact_tests = 0;
            int exact_edges = 0;
            for (std::size_t lhs = 0; lhs < local_indices.size(); ++lhs) {
                for (std::size_t rhs = lhs + 1; rhs < local_indices.size(); ++rhs) {
                    ++exact_tests;
                    if (boxes_connected(boxes_[static_cast<std::size_t>(local_indices[lhs])],
                                        boxes_[static_cast<std::size_t>(local_indices[rhs])],
                                        config_.query.adjacency_tolerance)) {
                        local_adj[lhs].push_back(static_cast<int>(rhs));
                        local_adj[rhs].push_back(static_cast<int>(lhs));
                        ++exact_edges;
                    }
                }
            }
            diagnostics.add_counter("query_bridge.hipac_promote_transition.local_adj_tests",
                                    static_cast<double>(exact_tests));
            diagnostics.add_counter("query_bridge.hipac_promote_transition.local_adj_edges",
                                    static_cast<double>(exact_edges));
            auto shortest_local_path = [&](int source_node, int target_node) {
                std::vector<int> parent(local_indices.size(), -1);
                std::queue<int> queue;
                parent[static_cast<std::size_t>(source_node)] = source_node;
                queue.push(source_node);
                while (!queue.empty()) {
                    const int current = queue.front();
                    queue.pop();
                    if (current == target_node) {
                        break;
                    }
                    for (int neighbor : local_adj[static_cast<std::size_t>(current)]) {
                        if (parent[static_cast<std::size_t>(neighbor)] >= 0) {
                            continue;
                        }
                        parent[static_cast<std::size_t>(neighbor)] = current;
                        queue.push(neighbor);
                    }
                }
                std::vector<int> path;
                if (parent[static_cast<std::size_t>(target_node)] < 0) {
                    return path;
                }
                for (int current = target_node;
                     current != source_node;
                     current = parent[static_cast<std::size_t>(current)]) {
                    path.push_back(current);
                }
                path.push_back(source_node);
                std::reverse(path.begin(), path.end());
                return path;
            };
            auto promote_local_path = [&](const std::vector<int>& local_path,
                                          const char* mode) {
                if (local_path.size() < 3) {
                    diagnostics.add_counter("query_bridge.hipac_promote_transition.short_chain");
                    return 0;
                }
                const int source_local = local_path.front();
                const int target_local = local_path.back();
                const BoxNode& portal_source =
                    boxes_[static_cast<std::size_t>(local_indices[static_cast<std::size_t>(source_local)])];
                const BoxNode& portal_target =
                    boxes_[static_cast<std::size_t>(local_indices[static_cast<std::size_t>(target_local)])];
                std::vector<BoxNode> internal_boxes;
                internal_boxes.reserve(local_path.size());
                for (int local_node : local_path) {
                    if (local_node == source_local || local_node == target_local) {
                        continue;
                    }
                    internal_boxes.push_back(
                        boxes_[static_cast<std::size_t>(local_indices[static_cast<std::size_t>(local_node)])]);
                }
                if (static_cast<int>(internal_boxes.size()) < min_boxes) {
                    diagnostics.add_counter("query_bridge.hipac_promote_transition.short_chain");
                    return 0;
                }
                const int edge_id = append_portal_corridor_edge(segment_edges_,
                                                                portal_source,
                                                                portal_target,
                                                                std::move(internal_boxes),
                                                                -1,
                                                                config_.query.adjacency_tolerance,
                                                                bridge_edge_query_index);
                if (edge_id < 0) {
                    diagnostics.add_counter("query_bridge.hipac_promote_transition.edge_fail");
                    return 0;
                }
                sync_adaptive_partition_segment_edges(&last_build_,
                                                      "query_bridge.hipac_promote_transition");
                diagnostics.add_counter("query_bridge.hipac_promote_transition.added");
                diagnostics.add_counter(std::string("query_bridge.hipac_promote_transition.added_") +
                                            mode);
                diagnostics.add_counter("query_bridge.hipac_promote_transition.internal_boxes",
                                        static_cast<double>(local_path.size() - 2));
                set_query_bridge_task_value("hipac_promote_transition_internal_boxes",
                                            static_cast<double>(local_path.size() - 2));
                invalidate_query_cache();
                return 1;
            };
            std::vector<int> full_local_path = shortest_local_path(local_source, local_target);
            if (!full_local_path.empty()) {
                const int promoted = promote_local_path(full_local_path, "full");
                if (promoted > 0) {
                    return finish_adopt(promoted);
                }
            } else {
                diagnostics.add_counter("query_bridge.hipac_promote_transition.chain_fail");
            }

            std::vector<int> component_id(local_indices.size(), -1);
            int component_count = 0;
            for (int node = 1; node + 1 < static_cast<int>(local_indices.size()); ++node) {
                if (component_id[static_cast<std::size_t>(node)] >= 0) {
                    continue;
                }
                std::queue<int> component_queue;
                component_id[static_cast<std::size_t>(node)] = component_count;
                component_queue.push(node);
                while (!component_queue.empty()) {
                    const int current = component_queue.front();
                    component_queue.pop();
                    for (int neighbor : local_adj[static_cast<std::size_t>(current)]) {
                        if (neighbor <= local_source || neighbor >= local_target ||
                            component_id[static_cast<std::size_t>(neighbor)] >= 0) {
                            continue;
                        }
                        component_id[static_cast<std::size_t>(neighbor)] = component_count;
                        component_queue.push(neighbor);
                    }
                }
                ++component_count;
            }
            diagnostics.add_counter("query_bridge.hipac_promote_transition.slice_components",
                                    static_cast<double>(component_count));
            std::vector<std::vector<int>> nodes_by_component(static_cast<std::size_t>(component_count));
            for (int node = 1; node + 1 < static_cast<int>(local_indices.size()); ++node) {
                const int component = component_id[static_cast<std::size_t>(node)];
                if (component >= 0) {
                    nodes_by_component[static_cast<std::size_t>(component)].push_back(node);
                }
            }
            struct SliceCandidate {
                int first = -1;
                int last = -1;
                int count = 0;
                int span = 0;
            };
            std::vector<SliceCandidate> slices;
            slices.reserve(nodes_by_component.size());
            auto sample_rank = [&](int local_node) {
                const int box_index =
                    local_indices[static_cast<std::size_t>(local_node)];
                const auto it = first_sample_by_box.find(box_index);
                return it == first_sample_by_box.end()
                    ? std::numeric_limits<int>::max()
                    : it->second;
            };
            for (auto& nodes : nodes_by_component) {
                if (static_cast<int>(nodes.size()) < min_boxes + 2) {
                    continue;
                }
                std::sort(nodes.begin(), nodes.end(), [&](int lhs, int rhs) {
                    const int lhs_rank = sample_rank(lhs);
                    const int rhs_rank = sample_rank(rhs);
                    if (lhs_rank != rhs_rank) {
                        return lhs_rank < rhs_rank;
                    }
                    return lhs < rhs;
                });
                SliceCandidate slice;
                slice.first = nodes.front();
                slice.last = nodes.back();
                slice.count = static_cast<int>(nodes.size());
                slice.span = std::max(0, sample_rank(slice.last) - sample_rank(slice.first));
                slices.push_back(slice);
            }
            std::sort(slices.begin(), slices.end(), [](const SliceCandidate& lhs,
                                                       const SliceCandidate& rhs) {
                if (lhs.count != rhs.count) {
                    return lhs.count > rhs.count;
                }
                if (lhs.span != rhs.span) {
                    return lhs.span > rhs.span;
                }
                return lhs.first < rhs.first;
            });
            for (const auto& slice : slices) {
                std::vector<int> slice_path = shortest_local_path(slice.first, slice.last);
                if (slice_path.empty()) {
                    diagnostics.add_counter("query_bridge.hipac_promote_transition.slice_chain_fail");
                    continue;
                }
                const int promoted = promote_local_path(slice_path, "slice");
                if (promoted > 0) {
                    return finish_adopt(promoted);
                }
            }
            diagnostics.add_counter("query_bridge.hipac_promote_transition.failures");
            return finish_adopt(0);
        };
        auto commit_result = [&](FindFreeBoxResult result,
                                 const Eigen::VectorXd& seed,
                                 int transition_hint) -> int {
            const auto commit_t0 = detailed_direct_timing ? Clock::now() : Clock::time_point{};
            auto finish = [&](int value) {
                if (detailed_direct_timing) {
                    commit_total_ms +=
                        std::chrono::duration<double, std::milli>(Clock::now() - commit_t0).count();
                    commit_calls += 1;
                }
                return value;
            };
            if (!result.found ||
                !intervals_contain_point_local(result.intervals,
                                               seed,
                                               config_.query.adjacency_tolerance)) {
                return finish(-1);
            }
            const int duplicate_index = find_duplicate_box_index(result.node,
                                                                 result.intervals);
            if (duplicate_index >= 0) {
                assimilate_box(duplicate_index, transition_hint);
                return finish(duplicate_index);
            }
            const auto dynamic_policy_t0 = detailed_direct_timing ? Clock::now() : Clock::time_point{};
            if (!allow_dynamic_commit(*oracle_, result, config_.connector.pave.commit_policy)) {
                if (detailed_direct_timing) {
                    commit_dynamic_policy_ms +=
                        std::chrono::duration<double, std::milli>(Clock::now() - dynamic_policy_t0).count();
                }
                return finish(-1);
            }
            if (detailed_direct_timing) {
                commit_dynamic_policy_ms +=
                    std::chrono::duration<double, std::milli>(Clock::now() - dynamic_policy_t0).count();
            }
            BoxNode box;
            box.id = next_id++;
            box.joint_intervals = result.intervals;
            box.seed_config = seed;
            box.tree_id = result.node;
            box.parent_box_id = -1;
            box.root_id = box.id;
            box.safety_status = result.validation_detail.safety_status;
            box.strict_audit_required = result.validation_detail.strict_audit_required;
            box.compute_volume();
            oracle_->reserve_node(box.tree_id, box.id);
            const int box_index = static_cast<int>(boxes_.size());
            boxes_.push_back(box);
            raw_boxes_.push_back(box);
            if (result.node != kInvalidOracleNodeId) {
                node_to_box_index.emplace(result.node, box_index);
            }
            if (use_partition_cover_index) {
                if (immediate_partition_append && adaptive_partition_) {
                    const auto partition_append_t0 =
                        detailed_direct_timing ? Clock::now() : Clock::time_point{};
                    const bool appended = adaptive_partition_->append_box(
                        boxes_.back(),
                        config_.query.adjacency_tolerance);
                    if (detailed_direct_timing) {
                        commit_partition_append_ms +=
                            std::chrono::duration<double, std::milli>(Clock::now() -
                                                                      partition_append_t0).count();
                    }
                    context.diagnostics().add_counter(
                        appended
                            ? "query_bridge.direct_corridor_immediate_partition_appends"
                            : "query_bridge.direct_corridor_immediate_partition_append_rejects");
                    if (!appended) {
                        corridor_new_box_indices.push_back(box_index);
                    }
                } else {
                    corridor_new_box_indices.push_back(box_index);
                }
            } else {
                direct_box_index.add_box(boxes_.back(),
                                         box_index,
                                         config_.query.adjacency_tolerance);
            }
            if (use_partition_neighbor_candidates) {
                box_id_to_index[box.id] = box_index;
            }
            dsu.add();
            assimilate_box(box_index, transition_hint);
            return finish(box_index);
        };
        auto current_boxes_cover_point = [&](const Eigen::VectorXd& point) {
            const auto timing_t0 = detailed_direct_timing ? Clock::now() : Clock::time_point{};
            auto finish = [&](bool value) {
                if (detailed_direct_timing) {
                    current_cover_ms +=
                        std::chrono::duration<double, std::milli>(Clock::now() - timing_t0).count();
                    current_cover_calls += 1;
                }
                return value;
            };
            if (use_partition_cover_index) {
                const auto partition_cover_t0 =
                    detailed_direct_timing ? Clock::now() : Clock::time_point{};
                const bool partition_covered =
                    !adaptive_partition_->covering_box_ids(point,
                                                           config_.query.adjacency_tolerance).empty();
                if (detailed_direct_timing) {
                    current_cover_partition_ms +=
                        std::chrono::duration<double, std::milli>(Clock::now() -
                                                                  partition_cover_t0).count();
                }
                if (partition_covered) {
                    return finish(true);
                }
            }
            if (use_partition_cover_index) {
                const auto corridor_scan_t0 =
                    detailed_direct_timing ? Clock::now() : Clock::time_point{};
                for (int box_index : corridor_new_box_indices) {
                    if (box_index >= 0 &&
                        box_index < static_cast<int>(boxes_.size()) &&
                        intervals_contain_point_local(boxes_[static_cast<std::size_t>(box_index)].joint_intervals,
                                                       point,
                                                       config_.query.adjacency_tolerance)) {
                        if (detailed_direct_timing) {
                            current_cover_corridor_scan_ms +=
                                std::chrono::duration<double, std::milli>(Clock::now() -
                                                                          corridor_scan_t0).count();
                        }
                        return finish(true);
                    }
                }
                if (detailed_direct_timing) {
                    current_cover_corridor_scan_ms +=
                        std::chrono::duration<double, std::milli>(Clock::now() -
                                                                  corridor_scan_t0).count();
                }
                return finish(false);
            }
            const auto direct_index_t0 = detailed_direct_timing ? Clock::now() : Clock::time_point{};
            const bool covered_by_direct_index =
                direct_box_index.covering_box(boxes_,
                                              point,
                                              config_.query.adjacency_tolerance) >= 0;
            if (detailed_direct_timing) {
                current_cover_direct_index_ms +=
                    std::chrono::duration<double, std::milli>(Clock::now() - direct_index_t0).count();
            }
            return finish(covered_by_direct_index);
        };
        FindFreeBoxOptions direct_options = config_.connector.pave.find_free_box;
        direct_options.max_depth = query_bridge_ffb_depth;
        if (config_.query_bridge_ffb_start_depth >= 0) {
            direct_options.start_depth = config_.query_bridge_ffb_start_depth;
            direct_options.skip_to_depth = config_.query_bridge_ffb_start_depth;
        }
        direct_options.reject_seed_collision = false;
        direct_options.skip_existing_cover_check = true;
        direct_options.record_diagnostics =
            env_int_or_default("RBF_QUERY_BRIDGE_FFB_DIAGNOSTICS", 0) != 0;
        const std::vector<Interval> direct_planning_domain =
            oracle_ ? oracle_->planning_intervals() : std::vector<Interval>{};
        struct DirectFfbTask {
            Eigen::VectorXd seed;
            std::size_t sample_index = 0;
            int transition_hint = 0;
        };
        context.diagnostics().set_value(
            "query_bridge.direct_corridor_ffb_diagnostics_enabled",
            direct_options.record_diagnostics ? 1.0 : 0.0);
        int direct_calls = 0;
        int direct_added = 0;
        double direct_ffb_ms = 0.0;
        double repair_ffb_ms = 0.0;
        double adaptive_repair_ffb_ms = 0.0;
        double lateral_repair_ffb_ms = 0.0;
        double residual_segment_audit_ms = 0.0;
        std::vector<DirectFfbTask> direct_tasks;
        direct_tasks.reserve(samples.size());
        const int max_transition_hint =
            std::max(0, static_cast<int>(samples.size()) - 2);
        const bool grouped_direct_seeds =
            env_int_or_default("RBF_QUERY_BRIDGE_DIRECT_GROUPED_SEEDS", 0) != 0;
        const int max_group_seeds =
            std::max(1, env_int_or_default("RBF_QUERY_BRIDGE_DIRECT_MAX_SEEDS_PER_GAP", 3));
        const bool coverage_order_direct_tasks =
            env_int_or_default("RBF_QUERY_BRIDGE_COVERAGE_ORDER_DIRECT_TASKS", 1) != 0;
        const bool center_out_direct_tasks =
            !coverage_order_direct_tasks &&
            env_int_or_default("RBF_QUERY_BRIDGE_CENTER_OUT_DIRECT_TASKS", 1) != 0;
        int uncovered_gap_groups = 0;
        auto append_direct_task_for_sample = [&](std::size_t sample_index) {
            direct_tasks.push_back(
                {samples[sample_index],
                 sample_index,
                 std::min(static_cast<int>(sample_index), max_transition_hint)});
        };
        const auto direct_task_build_t0 =
            detailed_direct_timing ? Clock::now() : Clock::time_point{};
        if (grouped_direct_seeds) {
            std::size_t sample_index = 0;
            while (sample_index < samples.size()) {
                while (sample_index < samples.size() && covered[sample_index]) {
                    ++sample_index;
                }
                if (sample_index >= samples.size()) {
                    break;
                }
                const std::size_t begin = sample_index;
                while (sample_index < samples.size() && !covered[sample_index]) {
                    ++sample_index;
                }
                const std::size_t end = sample_index - 1;
                uncovered_gap_groups += 1;
                std::vector<std::size_t> chosen;
                chosen.reserve(static_cast<std::size_t>(
                    std::min(max_group_seeds, static_cast<int>(end - begin + 1))));
                auto push_unique_index = [&](std::size_t index) {
                    index = std::min(end, std::max(begin, index));
                    if (std::find(chosen.begin(), chosen.end(), index) == chosen.end()) {
                        chosen.push_back(index);
                    }
                };
                const std::size_t group_count = end - begin + 1;
                if (group_count <= static_cast<std::size_t>(max_group_seeds)) {
                    for (std::size_t index = begin; index <= end; ++index) {
                        push_unique_index(index);
                    }
                } else {
                    push_unique_index((begin + end) / 2);
                    if (static_cast<int>(chosen.size()) < max_group_seeds) {
                        push_unique_index(begin);
                    }
                    if (static_cast<int>(chosen.size()) < max_group_seeds) {
                        push_unique_index(end);
                    }
                    if (static_cast<int>(chosen.size()) < max_group_seeds && end > begin + 1) {
                        push_unique_index(begin + (end - begin) / 4);
                    }
                    if (static_cast<int>(chosen.size()) < max_group_seeds && end > begin + 1) {
                        push_unique_index(begin + (3 * (end - begin)) / 4);
                    }
                    for (int rank = 1;
                         static_cast<int>(chosen.size()) < max_group_seeds &&
                         rank < max_group_seeds - 1;
                         ++rank) {
                        const double alpha = static_cast<double>(rank) /
                                             static_cast<double>(max_group_seeds - 1);
                        const auto offset = static_cast<std::size_t>(
                            std::llround(alpha * static_cast<double>(end - begin)));
                        push_unique_index(begin + offset);
                    }
                }
                for (std::size_t index : chosen) {
                    append_direct_task_for_sample(index);
                }
            }
        } else if (center_out_direct_tasks) {
            std::size_t sample_index = 0;
            while (sample_index < samples.size()) {
                while (sample_index < samples.size() && covered[sample_index]) {
                    ++sample_index;
                }
                if (sample_index >= samples.size()) {
                    break;
                }
                const std::size_t begin = sample_index;
                while (sample_index < samples.size() && !covered[sample_index]) {
                    ++sample_index;
                }
                const std::size_t end = sample_index - 1;
                uncovered_gap_groups += 1;
                const std::size_t center = (begin + end) / 2;
                append_direct_task_for_sample(center);
                for (std::size_t radius = 1;
                     center >= begin + radius || center + radius <= end;
                     ++radius) {
                    if (center >= begin + radius) {
                        append_direct_task_for_sample(center - radius);
                    }
                    if (center + radius <= end) {
                        append_direct_task_for_sample(center + radius);
                    }
                }
            }
        } else {
            for (std::size_t sample_index = 0; sample_index < samples.size(); ++sample_index) {
                if (!covered[sample_index]) {
                    append_direct_task_for_sample(sample_index);
                }
            }
        }
        if (detailed_direct_timing) {
            direct_task_build_ms =
                std::chrono::duration<double, std::milli>(Clock::now() -
                                                          direct_task_build_t0).count();
        }
        context.diagnostics().set_value("query_bridge.direct_corridor_direct_grouped_seeds",
                                        grouped_direct_seeds ? 1.0 : 0.0);
        context.diagnostics().set_value("query_bridge.direct_corridor_coverage_order_direct_tasks",
                                        coverage_order_direct_tasks ? 1.0 : 0.0);
        context.diagnostics().set_value("query_bridge.direct_corridor_center_out_direct_tasks",
                                        center_out_direct_tasks ? 1.0 : 0.0);
        context.diagnostics().set_value("query_bridge.direct_corridor_ffb_start_depth",
                                        static_cast<double>(std::max(direct_options.start_depth,
                                                                    direct_options.skip_to_depth)));
        context.diagnostics().set_value("query_bridge.direct_corridor_uncovered_gap_groups",
                                        static_cast<double>(uncovered_gap_groups));
        context.diagnostics().set_value("query_bridge.direct_corridor_direct_max_seeds_per_gap",
                                        static_cast<double>(max_group_seeds));
        context.diagnostics().set_value("query_bridge.direct_corridor_direct_tasks",
                                        static_cast<double>(direct_tasks.size()));
        const auto direct_loop_t0 =
            detailed_direct_timing ? Clock::now() : Clock::time_point{};
        for (const auto& task : direct_tasks) {
            if (task.sample_index < covered.size() && covered[task.sample_index]) {
                context.diagnostics().add_counter(
                    "query_bridge.direct_corridor_direct_skip_covered");
                continue;
            }
            const auto direct_ffb_t0 = Clock::now();
            const FindFreeBoxResult result = find_free_box_in_domain(
                task.seed,
                direct_planning_domain,
                context,
                direct_options);
            direct_ffb_ms +=
                std::chrono::duration<double, std::milli>(Clock::now() - direct_ffb_t0).count();
            direct_calls += 1;
            const std::size_t before_boxes = boxes_.size();
            const int box_index = commit_result(std::move(result),
                                                task.seed,
                                                task.transition_hint);
            if (box_index >= 0 && boxes_.size() > before_boxes) {
                direct_added += 1;
            }
        }
        if (detailed_direct_timing) {
            direct_loop_ms =
                std::chrono::duration<double, std::milli>(Clock::now() -
                                                          direct_loop_t0).count();
        }
        std::vector<double> fractions;
        const int base_subdivisions =
            env_int_or_default("RBF_QUERY_BRIDGE_REPAIR_SUBDIVISIONS", 6);
        const int subdivisions =
            std::max(0,
                     env_indexed_int_or_default("RBF_QUERY_BRIDGE_REPAIR_SUBDIVISIONS",
                                                query_index,
                                                base_subdivisions));
        for (int item = 1; item < subdivisions; ++item) {
            fractions.push_back(static_cast<double>(item) / static_cast<double>(subdivisions));
        }
        std::stable_sort(fractions.begin(), fractions.end(), [](double lhs, double rhs) {
            return std::abs(lhs - 0.5) < std::abs(rhs - 0.5);
        });
        int repair_calls = 0;
        int repair_added = 0;
        const auto initial_bad = bad_transitions();
        const auto repair_loop_t0 =
            detailed_direct_timing ? Clock::now() : Clock::time_point{};
        if (subdivisions > 1) {
            for (int transition : initial_bad) {
                if (transition_connected(transition)) {
                    continue;
                }
                const Eigen::VectorXd& a = samples[static_cast<std::size_t>(transition)];
                const Eigen::VectorXd& b = samples[static_cast<std::size_t>(transition + 1)];
                for (double u : fractions) {
                    if (transition_connected(transition)) {
                        break;
                    }
                    const Eigen::VectorXd seed = (1.0 - u) * a + u * b;
                    if (current_boxes_cover_point(seed)) {
                        context.diagnostics().add_counter(
                            "query_bridge.direct_corridor_repair_skip_covered");
                        continue;
                    }
                    const auto repair_ffb_t0 = Clock::now();
                    const FindFreeBoxResult result = find_free_box_in_domain(
                        seed,
                        direct_planning_domain,
                        context,
                        direct_options);
                    repair_ffb_ms +=
                        std::chrono::duration<double, std::milli>(Clock::now() -
                                                                  repair_ffb_t0).count();
                    repair_calls += 1;
                    const std::size_t before_boxes = boxes_.size();
                    const int box_index = commit_result(std::move(result), seed, transition);
                    if (box_index >= 0) {
                        if (std::find(repair_indices.begin(), repair_indices.end(), box_index) ==
                            repair_indices.end()) {
                            repair_indices.push_back(box_index);
                        }
                        if (boxes_.size() > before_boxes) {
                            repair_added += 1;
                        }
                        if (transition_connected(transition)) {
                            break;
                        }
                    }
                }
            }
        }
        if (detailed_direct_timing) {
            repair_loop_ms =
                std::chrono::duration<double, std::milli>(Clock::now() -
                                                          repair_loop_t0).count();
        }
        std::vector<int> final_bad = bad_transitions();
        auto transition_length = [&](int transition) {
            if (transition < 0 ||
                transition + 1 >= static_cast<int>(samples.size())) {
                return 0.0;
            }
            return (samples[static_cast<std::size_t>(transition + 1)] -
                    samples[static_cast<std::size_t>(transition)]).norm();
        };
        auto bad_transition_length_sum = [&](const std::vector<int>& transitions) {
            double total = 0.0;
            for (int transition : transitions) {
                total += transition_length(transition);
            }
            return total;
        };
        auto bad_transition_fraction = [&](const std::vector<int>& transitions) {
            const double denominator =
                audited_bridge_length > 1e-12
                    ? audited_bridge_length
                    : std::max(1e-12, waypoint_length(samples));
            return bad_transition_length_sum(transitions) / denominator;
        };
        const int adaptive_repair_priority_mode =
            env_int_or_default("RBF_QUERY_BRIDGE_ADAPTIVE_REPAIR_PRIORITY", 1);
        auto order_adaptive_repair_transitions =
            [&](const std::vector<int>& transitions) {
                if (adaptive_repair_priority_mode <= 0 || transitions.size() < 2) {
                    return transitions;
                }
                struct GapGroup {
                    int begin = 0;
                    int end = 0;
                    double length = 0.0;
                };
                std::vector<GapGroup> groups;
                groups.reserve(transitions.size());
                for (int transition : transitions) {
                    if (groups.empty() ||
                        transition > groups.back().end + 1) {
                        groups.push_back({transition,
                                          transition,
                                          transition_length(transition)});
                    } else {
                        groups.back().end = transition;
                        groups.back().length += transition_length(transition);
                    }
                }
                std::stable_sort(groups.begin(), groups.end(), [](const GapGroup& lhs,
                                                                   const GapGroup& rhs) {
                    if (std::abs(lhs.length - rhs.length) > 1e-12) {
                        return lhs.length > rhs.length;
                    }
                    return lhs.begin < rhs.begin;
                });
                std::vector<int> ordered;
                ordered.reserve(transitions.size());
                for (const auto& group : groups) {
                    const int center = (group.begin + group.end) / 2;
                    ordered.push_back(center);
                    for (int radius = 1;
                         center - radius >= group.begin || center + radius <= group.end;
                         ++radius) {
                        if (center - radius >= group.begin) {
                            ordered.push_back(center - radius);
                        }
                        if (center + radius <= group.end) {
                            ordered.push_back(center + radius);
                        }
                    }
                }
                return ordered;
            };
        int adaptive_repair_calls = 0;
        int adaptive_repair_added = 0;
        int adaptive_repair_max_subdivisions_used = subdivisions;
        const bool adaptive_step_repair =
            env_int_or_default("RBF_QUERY_BRIDGE_ADAPTIVE_STEP_REPAIR", 1) != 0;
        const double adaptive_target_segment_fraction =
            std::max(0.0,
                     env_double_or_default(
                         "RBF_QUERY_BRIDGE_ADAPTIVE_REPAIR_TARGET_SEGMENT_FRACTION",
                         0.0));
        const double adaptive_initial_bad_fraction =
            bad_transition_fraction(final_bad);
        context.diagnostics().set_value(
            "query_bridge.direct_corridor_adaptive_repair_priority",
            static_cast<double>(adaptive_repair_priority_mode));
        context.diagnostics().set_value(
            "query_bridge.direct_corridor_adaptive_repair_target_segment_fraction",
            adaptive_target_segment_fraction);
        context.diagnostics().set_value(
            "query_bridge.direct_corridor_adaptive_initial_bad_fraction",
            adaptive_initial_bad_fraction);
        if (adaptive_step_repair && !final_bad.empty()) {
            const auto adaptive_loop_t0 =
                detailed_direct_timing ? Clock::now() : Clock::time_point{};
            const int adaptive_max_subdivisions = std::max(
                subdivisions + 1,
                env_int_or_default("RBF_QUERY_BRIDGE_ADAPTIVE_MAX_REPAIR_SUBDIVISIONS",
                                   std::max(2, subdivisions * 2)));
            const double adaptive_fine_step = std::max(
                1e-4,
                env_double_or_default("RBF_QUERY_BRIDGE_ADAPTIVE_FINE_STEP",
                                      std::max(audit_step, sample_step * 0.5)));
            const int adaptive_max_calls = std::max(
                0,
                env_indexed_int_or_default(
                    "RBF_QUERY_BRIDGE_ADAPTIVE_MAX_REPAIR_CALLS",
                    query_index,
                    env_int_or_default("RBF_QUERY_BRIDGE_ADAPTIVE_MAX_REPAIR_CALLS",
                                       std::numeric_limits<int>::max())));
            std::vector<int> ordered_final_bad =
                order_adaptive_repair_transitions(final_bad);
            for (int transition : ordered_final_bad) {
                if (adaptive_repair_calls >= adaptive_max_calls) {
                    break;
                }
                if (adaptive_target_segment_fraction > 0.0 &&
                    bad_transition_fraction(final_bad) <= adaptive_target_segment_fraction) {
                    context.diagnostics().add_counter(
                        "query_bridge.direct_corridor_adaptive_repair_target_stops");
                    break;
                }
                if (transition_connected(transition) ||
                    transition < 0 ||
                    transition + 1 >= static_cast<int>(samples.size())) {
                    continue;
                }
                const Eigen::VectorXd& a = samples[static_cast<std::size_t>(transition)];
                const Eigen::VectorXd& b = samples[static_cast<std::size_t>(transition + 1)];
                const double gap_length = (b - a).norm();
                const int target_subdivisions = std::min(
                    adaptive_max_subdivisions,
                    std::max(subdivisions + 1,
                             static_cast<int>(std::ceil(gap_length / adaptive_fine_step))));
                adaptive_repair_max_subdivisions_used =
                    std::max(adaptive_repair_max_subdivisions_used, target_subdivisions);
                std::vector<double> adaptive_fractions;
                adaptive_fractions.reserve(static_cast<std::size_t>(
                    std::max(0, target_subdivisions - 1)));
                for (int item = 1; item < target_subdivisions; ++item) {
                    adaptive_fractions.push_back(
                        static_cast<double>(item) /
                        static_cast<double>(target_subdivisions));
                }
                std::stable_sort(adaptive_fractions.begin(),
                                 adaptive_fractions.end(),
                                 [](double lhs, double rhs) {
                                     return std::abs(lhs - 0.5) < std::abs(rhs - 0.5);
                                 });
                for (double u : adaptive_fractions) {
                    if (adaptive_repair_calls >= adaptive_max_calls) {
                        break;
                    }
                    if (transition_connected(transition)) {
                        break;
                    }
                    const Eigen::VectorXd seed = (1.0 - u) * a + u * b;
                    if (current_boxes_cover_point(seed)) {
                        context.diagnostics().add_counter(
                            "query_bridge.direct_corridor_adaptive_repair_skip_covered");
                        continue;
                    }
                    const auto adaptive_ffb_t0 = Clock::now();
                    const FindFreeBoxResult result = find_free_box_in_domain(
                        seed,
                        direct_planning_domain,
                        context,
                        direct_options);
                    adaptive_repair_ffb_ms +=
                        std::chrono::duration<double, std::milli>(Clock::now() -
                                                                  adaptive_ffb_t0).count();
                    adaptive_repair_calls += 1;
                    const std::size_t before_boxes = boxes_.size();
                    const int box_index = commit_result(std::move(result), seed, transition);
                    if (box_index >= 0) {
                        if (std::find(repair_indices.begin(), repair_indices.end(), box_index) ==
                            repair_indices.end()) {
                            repair_indices.push_back(box_index);
                        }
                        if (boxes_.size() > before_boxes) {
                            adaptive_repair_added += 1;
                        }
                        if (adaptive_target_segment_fraction > 0.0) {
                            final_bad = bad_transitions();
                        }
                        if (transition_connected(transition)) {
                            break;
                        }
                    }
                }
            }
            final_bad = bad_transitions();
            if (detailed_direct_timing) {
                adaptive_loop_ms =
                    std::chrono::duration<double, std::milli>(Clock::now() -
                                                              adaptive_loop_t0).count();
            }
        }
        context.diagnostics().set_value(
            "query_bridge.direct_corridor_adaptive_final_bad_fraction",
            bad_transition_fraction(final_bad));
        int lateral_repair_calls = 0;
        int lateral_repair_added = 0;
        const bool lateral_repair =
            env_int_or_default("RBF_QUERY_BRIDGE_LATERAL_REPAIR", 0) != 0;
        if (lateral_repair && !final_bad.empty()) {
            const auto lateral_loop_t0 =
                detailed_direct_timing ? Clock::now() : Clock::time_point{};
            const int lateral_dims = std::max(
                0,
                env_int_or_default("RBF_QUERY_BRIDGE_LATERAL_REPAIR_DIMS", 2));
            const int lateral_rounds = std::max(
                1,
                env_int_or_default("RBF_QUERY_BRIDGE_LATERAL_REPAIR_ROUNDS", 1));
            const int lateral_max_calls = std::max(
                0,
                env_int_or_default("RBF_QUERY_BRIDGE_LATERAL_REPAIR_MAX_CALLS", 24));
            const double lateral_offset = std::max(
                1e-6,
                env_double_or_default("RBF_QUERY_BRIDGE_LATERAL_REPAIR_OFFSET",
                                      std::max(0.01, sample_step * 0.25)));
            const auto domain = oracle_->planning_intervals();
            auto lateral_candidates = [&](const Eigen::VectorXd& seed,
                                          const Eigen::VectorXd& direction) {
                std::vector<int> dims;
                dims.reserve(static_cast<std::size_t>(seed.size()));
                for (int dim = 0; dim < seed.size(); ++dim) {
                    dims.push_back(dim);
                }
                std::sort(dims.begin(), dims.end(), [&](int lhs, int rhs) {
                    const double lhs_abs = std::abs(direction[lhs]);
                    const double rhs_abs = std::abs(direction[rhs]);
                    if (std::abs(lhs_abs - rhs_abs) > 1e-12) {
                        return lhs_abs < rhs_abs;
                    }
                    return lhs < rhs;
                });
                std::vector<Eigen::VectorXd> out;
                const int dim_limit = std::min<int>(lateral_dims,
                                                    static_cast<int>(dims.size()));
                out.reserve(static_cast<std::size_t>(std::max(0, dim_limit) * lateral_rounds * 2));
                for (int item = 0; item < dim_limit; ++item) {
                    const int dim = dims[static_cast<std::size_t>(item)];
                    for (int round = 1; round <= lateral_rounds; ++round) {
                        for (double sign : {1.0, -1.0}) {
                            Eigen::VectorXd candidate = seed;
                            candidate[dim] += sign * lateral_offset * static_cast<double>(round);
                            if (dim < static_cast<int>(domain.size())) {
                                candidate[dim] = std::min(domain[static_cast<std::size_t>(dim)].hi,
                                                          std::max(domain[static_cast<std::size_t>(dim)].lo,
                                                                   candidate[dim]));
                            }
                            if ((candidate - seed).norm() > 1e-12) {
                                out.push_back(std::move(candidate));
                            }
                        }
                    }
                }
                return out;
            };
            for (int transition : final_bad) {
                if (lateral_repair_calls >= lateral_max_calls) {
                    break;
                }
                if (transition_connected(transition) ||
                    transition < 0 ||
                    transition + 1 >= static_cast<int>(samples.size())) {
                    continue;
                }
                const Eigen::VectorXd& a = samples[static_cast<std::size_t>(transition)];
                const Eigen::VectorXd& b = samples[static_cast<std::size_t>(transition + 1)];
                const Eigen::VectorXd seed = 0.5 * (a + b);
                const Eigen::VectorXd direction = b - a;
                for (const Eigen::VectorXd& lateral_seed : lateral_candidates(seed, direction)) {
                    if (lateral_repair_calls >= lateral_max_calls) {
                        break;
                    }
                    if (transition_connected(transition)) {
                        break;
                    }
                    if (current_boxes_cover_point(lateral_seed)) {
                        context.diagnostics().add_counter(
                            "query_bridge.direct_corridor_lateral_repair_skip_covered");
                        continue;
                    }
                    const auto lateral_ffb_t0 = Clock::now();
                    const FindFreeBoxResult result = find_free_box_in_domain(
                        lateral_seed,
                        direct_planning_domain,
                        context,
                        direct_options);
                    lateral_repair_ffb_ms +=
                        std::chrono::duration<double, std::milli>(Clock::now() -
                                                                  lateral_ffb_t0).count();
                    lateral_repair_calls += 1;
                    const std::size_t before_boxes = boxes_.size();
                    const int box_index = commit_result(std::move(result),
                                                        lateral_seed,
                                                        transition);
                    if (box_index >= 0) {
                        if (std::find(repair_indices.begin(), repair_indices.end(), box_index) ==
                            repair_indices.end()) {
                            repair_indices.push_back(box_index);
                        }
                        if (boxes_.size() > before_boxes) {
                            lateral_repair_added += 1;
                        }
                        if (transition_connected(transition)) {
                            break;
                        }
                    }
                }
            }
            final_bad = bad_transitions();
            if (detailed_direct_timing) {
                lateral_loop_ms =
                    std::chrono::duration<double, std::milli>(Clock::now() -
                                                              lateral_loop_t0).count();
            }
        }
        int local_segment_edges_added = 0;
        int local_segment_gap_samples_max = 0;
        auto nearest_nonempty_layer = [&](int start_index, int direction) {
            int index = start_index;
            while (index >= 0 && index < static_cast<int>(sample_layers.size())) {
                if (!sample_layers[static_cast<std::size_t>(index)].empty()) {
                    return index;
                }
                index += direction;
            }
            return -1;
        };
        if (!final_bad.empty() &&
            allow_residual_segments &&
            config_.connector.segment_edges_enabled &&
            config_.connector.rrt_segment_edges) {
            const auto residual_segment_loop_t0 =
                detailed_direct_timing ? Clock::now() : Clock::time_point{};
            const bool group_residual_gaps =
                env_int_or_default("RBF_QUERY_BRIDGE_GROUP_RESIDUAL_GAPS", 0) != 0;
            std::vector<std::pair<int, int>> gap_groups;
            gap_groups.reserve(final_bad.size());
            for (int transition : final_bad) {
                if (transition < 0 || transition + 1 >= static_cast<int>(sample_layers.size())) {
                    continue;
                }
                if (!group_residual_gaps ||
                    gap_groups.empty() ||
                    transition > gap_groups.back().second + 1) {
                    gap_groups.emplace_back(transition, transition);
                } else {
                    gap_groups.back().second = transition;
                }
            }
            context.diagnostics().set_value(
                "query_bridge.direct_corridor_segment_gap_groups",
                static_cast<double>(gap_groups.size()));
            std::vector<std::pair<int, int>> pending_gap_groups;
            pending_gap_groups.reserve(gap_groups.size());
            for (auto it = gap_groups.rbegin(); it != gap_groups.rend(); ++it) {
                pending_gap_groups.push_back(*it);
            }
            while (!pending_gap_groups.empty()) {
                const auto gap_group = pending_gap_groups.back();
                pending_gap_groups.pop_back();
                const int lhs_sample =
                    nearest_nonempty_layer(gap_group.first, -1);
                const int rhs_sample =
                    nearest_nonempty_layer(gap_group.second + 1, 1);
                if (lhs_sample < 0 || rhs_sample < 0 || lhs_sample >= rhs_sample) {
                    continue;
                }
                const auto& lhs_layer = sample_layers[static_cast<std::size_t>(lhs_sample)];
                const auto& rhs_layer = sample_layers[static_cast<std::size_t>(rhs_sample)];
                if (lhs_layer.empty() || rhs_layer.empty()) {
                    continue;
                }
                const int lhs_index = lhs_layer.front();
                const int rhs_index = rhs_layer.front();
                if (lhs_index < 0 || rhs_index < 0 ||
                    lhs_index >= static_cast<int>(boxes_.size()) ||
                    rhs_index >= static_cast<int>(boxes_.size())) {
                    continue;
                }
                std::vector<Eigen::VectorXd> gap_path{
                    samples[static_cast<std::size_t>(lhs_sample)],
                    samples[static_cast<std::size_t>(rhs_sample)]};
                const auto segment_audit_t0 = Clock::now();
                const PathAuditCheck gap_audit =
                    audit_waypoint_path(gap_path,
                                        checker,
                                        config_.query.audit_resolution,
                                        config_.query.audit_segment_step);
                residual_segment_audit_ms +=
                    std::chrono::duration<double, std::milli>(Clock::now() -
                                                              segment_audit_t0).count();
                if (!gap_audit.passed) {
                    context.diagnostics().add_counter(
                        "query_bridge.direct_corridor_segment_audit_rejects");
                    if (group_residual_gaps && gap_group.first < gap_group.second) {
                        const int mid = (gap_group.first + gap_group.second) / 2;
                        pending_gap_groups.emplace_back(mid + 1, gap_group.second);
                        pending_gap_groups.emplace_back(gap_group.first, mid);
                        context.diagnostics().add_counter(
                            "query_bridge.direct_corridor_segment_group_splits");
                    }
                    continue;
                }
                const auto segment_insert_t0 =
                    detailed_direct_timing ? Clock::now() : Clock::time_point{};
                const int edge_id = add_segment_edge_partition_first(                                                     boxes_[static_cast<std::size_t>(lhs_index)].id,
                                                     boxes_[static_cast<std::size_t>(rhs_index)].id,
                                                     std::move(gap_path),
                                                     SegmentEdgeType::QueryBridge,
                                                     bridge_rrt.segment_resolution,
                                                     SegmentEdgeValidation::CollisionChecked,
                                                     true,
                                                     bridge_edge_query_index);
                if (detailed_direct_timing) {
                    segment_insert_ms +=
                        std::chrono::duration<double, std::milli>(Clock::now() - segment_insert_t0).count();
                    segment_insert_calls += 1;
                }
                if (edge_id >= 0) {
                    local_segment_edges_added += 1;
                    local_segment_gap_samples_max =
                        std::max(local_segment_gap_samples_max,
                                 rhs_sample - lhs_sample);
                }
            }
            if (detailed_direct_timing) {
                residual_segment_loop_ms =
                    std::chrono::duration<double, std::milli>(Clock::now() -
                                                              residual_segment_loop_t0).count();
            }
        }
        const double direct_corridor_elapsed_ms =
            std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
        context.diagnostics().set_value("query_bridge.direct_corridor_ms",
                                        direct_corridor_elapsed_ms);
        context.diagnostics().add_counter("query_bridge.direct_corridor_ms_total",
                                          direct_corridor_elapsed_ms);
        context.diagnostics().set_value("query_bridge.direct_corridor_samples",
                                        static_cast<double>(samples.size()));
        context.diagnostics().add_counter("query_bridge.direct_corridor_samples_total",
                                          static_cast<double>(samples.size()));
        context.diagnostics().set_value("query_bridge.direct_corridor_ffb_calls",
                                        static_cast<double>(direct_calls));
        context.diagnostics().add_counter("query_bridge.direct_corridor_ffb_calls_total",
                                          static_cast<double>(direct_calls));
        context.diagnostics().set_value("query_bridge.direct_corridor_direct_ffb_ms",
                                        direct_ffb_ms);
        context.diagnostics().set_value("query_bridge.direct_corridor_repair_ffb_ms",
                                        repair_ffb_ms);
        context.diagnostics().set_value("query_bridge.direct_corridor_adaptive_repair_ffb_ms",
                                        adaptive_repair_ffb_ms);
        context.diagnostics().set_value("query_bridge.direct_corridor_lateral_repair_ffb_ms",
                                        lateral_repair_ffb_ms);
        context.diagnostics().set_value("query_bridge.direct_corridor_segment_audit_ms",
                                        residual_segment_audit_ms);
        const int all_ffb_calls =
            direct_calls + repair_calls + adaptive_repair_calls + lateral_repair_calls;
        context.diagnostics().set_value("query_bridge.direct_corridor_all_ffb_calls",
                                        static_cast<double>(all_ffb_calls));
        context.diagnostics().add_counter("query_bridge.direct_corridor_all_ffb_calls_total",
                                          static_cast<double>(all_ffb_calls));
        context.diagnostics().set_value("query_bridge.direct_corridor_added",
                                        static_cast<double>(direct_added));
        context.diagnostics().add_counter("query_bridge.direct_corridor_added_total",
                                          static_cast<double>(direct_added));
        context.diagnostics().set_value("query_bridge.direct_corridor_repair_calls",
                                        static_cast<double>(repair_calls));
        context.diagnostics().add_counter("query_bridge.direct_corridor_repair_calls_total",
                                          static_cast<double>(repair_calls));
        context.diagnostics().set_value("query_bridge.direct_corridor_repair_added",
                                        static_cast<double>(repair_added));
        context.diagnostics().add_counter("query_bridge.direct_corridor_repair_added_total",
                                          static_cast<double>(repair_added));
        context.diagnostics().set_value("query_bridge.direct_corridor_adaptive_repair_calls",
                                        static_cast<double>(adaptive_repair_calls));
        context.diagnostics().add_counter(
            "query_bridge.direct_corridor_adaptive_repair_calls_total",
            static_cast<double>(adaptive_repair_calls));
        context.diagnostics().set_value("query_bridge.direct_corridor_adaptive_repair_added",
                                        static_cast<double>(adaptive_repair_added));
        context.diagnostics().add_counter(
            "query_bridge.direct_corridor_adaptive_repair_added_total",
            static_cast<double>(adaptive_repair_added));
        context.diagnostics().set_value("query_bridge.direct_corridor_lateral_repair_enabled",
                                        lateral_repair ? 1.0 : 0.0);
        context.diagnostics().set_value("query_bridge.direct_corridor_lateral_repair_calls",
                                        static_cast<double>(lateral_repair_calls));
        context.diagnostics().add_counter(
            "query_bridge.direct_corridor_lateral_repair_calls_total",
            static_cast<double>(lateral_repair_calls));
        context.diagnostics().set_value("query_bridge.direct_corridor_lateral_repair_added",
                                        static_cast<double>(lateral_repair_added));
        context.diagnostics().add_counter(
            "query_bridge.direct_corridor_lateral_repair_added_total",
            static_cast<double>(lateral_repair_added));
        context.diagnostics().set_value("query_bridge.direct_corridor_adaptive_repair_max_subdivisions",
                                        static_cast<double>(adaptive_repair_max_subdivisions_used));
        context.diagnostics().set_value("query_bridge.direct_corridor_repair_subdivisions",
                                        static_cast<double>(subdivisions));
        context.diagnostics().set_value("query_bridge.direct_corridor_bad_initial",
                                        static_cast<double>(initial_bad.size()));
        context.diagnostics().add_counter("query_bridge.direct_corridor_bad_initial_total",
                                          static_cast<double>(initial_bad.size()));
        context.diagnostics().set_value("query_bridge.direct_corridor_bad_final",
                                        static_cast<double>(final_bad.size()));
        context.diagnostics().add_counter("query_bridge.direct_corridor_bad_final_total",
                                          static_cast<double>(final_bad.size()));
        context.diagnostics().set_value("query_bridge.direct_corridor_segment_edges",
                                        static_cast<double>(local_segment_edges_added));
        context.diagnostics().add_counter("query_bridge.direct_corridor_segment_edges_total",
                                          static_cast<double>(local_segment_edges_added));
        context.diagnostics().set_value("query_bridge.direct_corridor_segment_gap_samples_max",
                                        static_cast<double>(local_segment_gap_samples_max));
        if (detailed_direct_timing) {
            context.diagnostics().add_counter("query_bridge.direct_corridor_transition_connected_ms",
                                              transition_connected_ms);
            context.diagnostics().add_counter("query_bridge.direct_corridor_transition_connected_calls",
                                              static_cast<double>(transition_connected_calls));
            context.diagnostics().add_counter("query_bridge.direct_corridor_bad_transitions_ms",
                                              bad_transitions_ms);
            context.diagnostics().add_counter("query_bridge.direct_corridor_bad_transitions_calls",
                                              static_cast<double>(bad_transitions_calls));
            context.diagnostics().add_counter("query_bridge.direct_corridor_current_cover_ms",
                                              current_cover_ms);
            context.diagnostics().add_counter("query_bridge.direct_corridor_current_cover_calls",
                                              static_cast<double>(current_cover_calls));
            context.diagnostics().add_counter("query_bridge.direct_corridor_current_cover_partition_ms",
                                              current_cover_partition_ms);
            context.diagnostics().add_counter("query_bridge.direct_corridor_current_cover_corridor_scan_ms",
                                              current_cover_corridor_scan_ms);
            context.diagnostics().add_counter("query_bridge.direct_corridor_current_cover_direct_index_ms",
                                              current_cover_direct_index_ms);
            context.diagnostics().add_counter("query_bridge.direct_corridor_duplicate_lookup_ms",
                                              duplicate_lookup_ms);
            context.diagnostics().add_counter("query_bridge.direct_corridor_duplicate_lookup_calls",
                                              static_cast<double>(duplicate_lookup_calls));
            context.diagnostics().add_counter("query_bridge.direct_corridor_commit_total_ms",
                                              commit_total_ms);
            context.diagnostics().add_counter("query_bridge.direct_corridor_commit_calls",
                                              static_cast<double>(commit_calls));
            context.diagnostics().add_counter("query_bridge.direct_corridor_commit_dynamic_policy_ms",
                                              commit_dynamic_policy_ms);
            context.diagnostics().add_counter("query_bridge.direct_corridor_commit_partition_append_ms",
                                              commit_partition_append_ms);
            context.diagnostics().add_counter("query_bridge.direct_corridor_assimilate_calls",
                                              static_cast<double>(assimilate_calls));
            context.diagnostics().add_counter("query_bridge.direct_corridor_assimilate_sample_scan_ms",
                                              assimilate_sample_scan_ms);
            context.diagnostics().add_counter("query_bridge.direct_corridor_assimilate_candidate_build_ms",
                                              assimilate_candidate_build_ms);
            context.diagnostics().add_counter("query_bridge.direct_corridor_assimilate_adjacency_ms",
                                              assimilate_adjacency_ms);
            context.diagnostics().add_counter("query_bridge.direct_corridor_segment_insert_ms",
                                              segment_insert_ms);
            context.diagnostics().add_counter("query_bridge.direct_corridor_segment_insert_calls",
                                              static_cast<double>(segment_insert_calls));
            context.diagnostics().add_counter("query_bridge.direct_corridor_direct_task_build_ms",
                                              direct_task_build_ms);
            context.diagnostics().add_counter("query_bridge.direct_corridor_direct_loop_ms",
                                              direct_loop_ms);
            context.diagnostics().add_counter("query_bridge.direct_corridor_repair_loop_ms",
                                              repair_loop_ms);
            context.diagnostics().add_counter("query_bridge.direct_corridor_adaptive_loop_ms",
                                              adaptive_loop_ms);
            context.diagnostics().add_counter("query_bridge.direct_corridor_lateral_loop_ms",
                                              lateral_loop_ms);
            context.diagnostics().add_counter("query_bridge.direct_corridor_residual_segment_loop_ms",
                                              residual_segment_loop_ms);
            set_query_bridge_task_value("direct_corridor_transition_connected_ms",
                                        transition_connected_ms);
            set_query_bridge_task_value("direct_corridor_transition_connected_calls",
                                        static_cast<double>(transition_connected_calls));
            set_query_bridge_task_value("direct_corridor_bad_transitions_ms",
                                        bad_transitions_ms);
            set_query_bridge_task_value("direct_corridor_bad_transitions_calls",
                                        static_cast<double>(bad_transitions_calls));
            set_query_bridge_task_value("direct_corridor_current_cover_ms",
                                        current_cover_ms);
            set_query_bridge_task_value("direct_corridor_current_cover_calls",
                                        static_cast<double>(current_cover_calls));
            set_query_bridge_task_value("direct_corridor_current_cover_partition_ms",
                                        current_cover_partition_ms);
            set_query_bridge_task_value("direct_corridor_current_cover_corridor_scan_ms",
                                        current_cover_corridor_scan_ms);
            set_query_bridge_task_value("direct_corridor_current_cover_direct_index_ms",
                                        current_cover_direct_index_ms);
            set_query_bridge_task_value("direct_corridor_duplicate_lookup_ms",
                                        duplicate_lookup_ms);
            set_query_bridge_task_value("direct_corridor_duplicate_lookup_calls",
                                        static_cast<double>(duplicate_lookup_calls));
            set_query_bridge_task_value("direct_corridor_commit_total_ms",
                                        commit_total_ms);
            set_query_bridge_task_value("direct_corridor_commit_calls",
                                        static_cast<double>(commit_calls));
            set_query_bridge_task_value("direct_corridor_commit_dynamic_policy_ms",
                                        commit_dynamic_policy_ms);
            set_query_bridge_task_value("direct_corridor_commit_partition_append_ms",
                                        commit_partition_append_ms);
            set_query_bridge_task_value("direct_corridor_assimilate_calls",
                                        static_cast<double>(assimilate_calls));
            set_query_bridge_task_value("direct_corridor_assimilate_sample_scan_ms",
                                        assimilate_sample_scan_ms);
            set_query_bridge_task_value("direct_corridor_assimilate_candidate_build_ms",
                                        assimilate_candidate_build_ms);
            set_query_bridge_task_value("direct_corridor_assimilate_adjacency_ms",
                                        assimilate_adjacency_ms);
            set_query_bridge_task_value("direct_corridor_segment_insert_ms",
                                        segment_insert_ms);
            set_query_bridge_task_value("direct_corridor_segment_insert_calls",
                                        static_cast<double>(segment_insert_calls));
            set_query_bridge_task_value("direct_corridor_direct_task_build_ms",
                                        direct_task_build_ms);
            set_query_bridge_task_value("direct_corridor_direct_loop_ms",
                                        direct_loop_ms);
            set_query_bridge_task_value("direct_corridor_repair_loop_ms",
                                        repair_loop_ms);
            set_query_bridge_task_value("direct_corridor_adaptive_loop_ms",
                                        adaptive_loop_ms);
            set_query_bridge_task_value("direct_corridor_lateral_loop_ms",
                                        lateral_loop_ms);
            set_query_bridge_task_value("direct_corridor_residual_segment_loop_ms",
                                        residual_segment_loop_ms);
        }
        auto [source_box_id, target_box_id] = locate_query_boxes();
        const bool local_corridor_connected =
            final_bad.empty() && endpoint_layers_connected();
        context.diagnostics().set_value("query_bridge.direct_corridor_local_connected",
                                        local_corridor_connected ? 1.0 : 0.0);
        set_query_bridge_task_value("direct_corridor_ms",
                                    direct_corridor_elapsed_ms);
        set_query_bridge_task_value("direct_corridor_samples",
                                    static_cast<double>(samples.size()));
        set_query_bridge_task_value("direct_corridor_ffb_calls",
                                    static_cast<double>(direct_calls));
        set_query_bridge_task_value("direct_corridor_direct_ffb_ms",
                                    direct_ffb_ms);
        set_query_bridge_task_value("direct_corridor_repair_ffb_ms",
                                    repair_ffb_ms);
        set_query_bridge_task_value("direct_corridor_adaptive_repair_ffb_ms",
                                    adaptive_repair_ffb_ms);
        set_query_bridge_task_value("direct_corridor_lateral_repair_ffb_ms",
                                    lateral_repair_ffb_ms);
        set_query_bridge_task_value("direct_corridor_segment_audit_ms",
                                    residual_segment_audit_ms);
        set_query_bridge_task_value("direct_corridor_all_ffb_calls",
                                    static_cast<double>(all_ffb_calls));
        set_query_bridge_task_value("direct_corridor_added",
                                    static_cast<double>(direct_added));
        set_query_bridge_task_value("direct_corridor_repair_calls",
                                    static_cast<double>(repair_calls));
        set_query_bridge_task_value("direct_corridor_repair_added",
                                    static_cast<double>(repair_added));
        set_query_bridge_task_value("direct_corridor_adaptive_repair_calls",
                                    static_cast<double>(adaptive_repair_calls));
        set_query_bridge_task_value("direct_corridor_adaptive_repair_added",
                                    static_cast<double>(adaptive_repair_added));
        set_query_bridge_task_value("direct_corridor_lateral_repair_calls",
                                    static_cast<double>(lateral_repair_calls));
        set_query_bridge_task_value("direct_corridor_lateral_repair_added",
                                    static_cast<double>(lateral_repair_added));
        set_query_bridge_task_value("direct_corridor_bad_initial",
                                    static_cast<double>(initial_bad.size()));
        set_query_bridge_task_value("direct_corridor_bad_final",
                                    static_cast<double>(final_bad.size()));
        set_query_bridge_task_value("direct_corridor_segment_edges",
                                    static_cast<double>(local_segment_edges_added));
        set_query_bridge_task_value("direct_corridor_local_connected",
                                    local_corridor_connected ? 1.0 : 0.0);
        if (final_bad.empty() &&
            source_box_id >= 0 &&
            target_box_id >= 0 &&
            (local_corridor_connected ||
             box_only_path_connected_partition_first(source_box_id, target_box_id))) {
            try_adopt_certified_subchain(source_box_id,
                                         target_box_id,
                                         "box_connected");
            const int edge_id = add_segment_edge_partition_first(                                                 source_box_id,
                                                 target_box_id,
                                                 corridor_path,
                                                 SegmentEdgeType::BoxCorridor,
                                                 bridge_rrt.segment_resolution,
                                                 SegmentEdgeValidation::CollisionChecked,
                                                 false,
                                                 bridge_edge_query_index);
            if (edge_id >= 0) {
                invalidate_query_cache();
                return finish_direct_corridor(direct_added + repair_added + local_segment_edges_added + 1);
            }
            return finish_direct_corridor(direct_added + repair_added + local_segment_edges_added);
        }
        if (!final_bad.empty() &&
            allow_residual_segments &&
            local_segment_edges_added > 0 &&
            source_box_id >= 0 &&
            target_box_id >= 0) {
            refresh_direct_corridor_partition();
            const bool locally_overlay_connected =
                overlay_path_connected_partition_first(source_box_id, target_box_id);
            context.diagnostics().set_value(
                "query_bridge.direct_corridor_local_residual_overlay_connected",
                locally_overlay_connected ? 1.0 : 0.0);
            set_query_bridge_task_value("direct_corridor_local_residual_overlay_connected",
                                        locally_overlay_connected ? 1.0 : 0.0);
            if (locally_overlay_connected) {
                try_adopt_certified_subchain(source_box_id,
                                             target_box_id,
                                             "local_residual_overlay");
                invalidate_query_cache();
                return finish_direct_corridor(direct_added + repair_added + local_segment_edges_added);
            }
            context.diagnostics().add_counter(
                "query_bridge.direct_corridor_full_residual_without_local_overlay");
            const PathAuditCheck full_residual_audit =
                audit_waypoint_path(corridor_path,
                                    checker,
                                    config_.query.audit_resolution,
                                    config_.query.audit_segment_step);
            if (!full_residual_audit.passed) {
                context.diagnostics().add_counter(
                    "query_bridge.direct_corridor_full_residual_audit_rejects");
                invalidate_query_cache();
                return finish_direct_corridor(direct_added + repair_added + local_segment_edges_added);
            }
            const int edge_id = add_segment_edge_partition_first(                                                 source_box_id,
                                                 target_box_id,
                                                 corridor_path,
                                                 SegmentEdgeType::QueryBridge,
                                                 bridge_rrt.segment_resolution,
                                                 SegmentEdgeValidation::CollisionChecked,
                                                 true,
                                                 bridge_edge_query_index);
            if (edge_id >= 0) {
                context.diagnostics().add_counter(
                    "query_bridge.direct_corridor_full_residual_edges");
                if (!locally_overlay_connected) {
                    context.diagnostics().add_counter(
                        "query_bridge.direct_corridor_full_residual_edges_without_local_overlay");
                }
                set_query_bridge_task_value("direct_corridor_full_residual_edge",
                                            1.0);
            }
            try_adopt_certified_subchain(source_box_id,
                                         target_box_id,
                                         "full_residual");
            invalidate_query_cache();
            return finish_direct_corridor(direct_added + repair_added + local_segment_edges_added +
                                          (edge_id >= 0 ? 1 : 0));
        }
        return finish_direct_corridor(0);
    };
    if (dense_box_corridor_candidate) {
        const int direct_corridor_added = try_direct_ffb_corridor();
        if (direct_corridor_added > 0) {
            return direct_corridor_added;
        }
        if (partition_native_mode()) {
            context.diagnostics().add_counter(
                "query_bridge.partition_legacy_dense_chain_pave_skipped");
            dense_repair_attempted = true;
        }
    }
    if (dense_box_corridor_candidate && !partition_native_mode()) {
        dense_repair_attempted = true;
        ChainPaveConfig dense_config = config_.connector.pave;
        dense_config.max_chain = std::max(dense_config.max_chain, 256);
        dense_config.refine_covered_waypoints = true;
        dense_config.fill_gaps = true;
        dense_config.find_free_box.max_depth = query_bridge_ffb_depth;
        dense_config.adaptive_ffb_depths =
            config_.query_bridge_adaptive_ffb_depths.empty()
                ? dense_config.adaptive_ffb_depths
                : config_.query_bridge_adaptive_ffb_depths;
        dense_config.gap_fill_sample_step = 0.0025;
        dense_config.gap_fill_time_budget_ms = 0.0;
        dense_config.gap_fill_max_ffb_calls = -1;
        dense_config.gap_fill_min_arc_gain = 0.0;
        dense_config.require_connected_chain = true;
        const std::size_t boxes_before_dense = boxes_.size();
        dense_repair_added = chain_pave_along_path(
            corridor_path,
            start_box_id,
            boxes_,
            *oracle_,
            adjacency_,
            next_id,
            context,
            dense_config);
        if (dense_repair_added > 0) {
            append_partition_after_pave(boxes_before_dense,
                                        "query_bridge.dense_boundary_pave");
            context.diagnostics().add_counter(
                "query_bridge.full_adjacency_rebuilds_avoided");
            invalidate_query_cache();
        }
        auto [source_box_id, target_box_id] =
            try_reverse_boundary_pave(dense_config,
                                      dense_repair_added,
                                      dense_repair_added);
        if (source_box_id >= 0 &&
            target_box_id >= 0 &&
            box_only_path_connected_partition_first(source_box_id, target_box_id)) {
            const int edge_id = add_segment_edge_partition_first(	                                 source_box_id,
	                                 target_box_id,
	                                 corridor_path,
                                                 SegmentEdgeType::BoxCorridor,
                                                 bridge_rrt.segment_resolution,
                                                 SegmentEdgeValidation::CollisionChecked,
                                                 false,
                                                 bridge_edge_query_index);
            box_corridor_edges_added = edge_id >= 0 ? 1 : 0;
            if (box_corridor_edges_added > 0) {
                invalidate_query_cache();
            }
            return finish_bridge(dense_repair_added + box_corridor_edges_added);
        }
    }
    ChainPaveConfig pave_config = config_.connector.pave;
    if (defer_query_segment_edge) {
        pave_config.max_chain = std::max(pave_config.max_chain, 256);
        pave_config.refine_covered_waypoints = true;
        pave_config.fill_gaps = true;
        pave_config.find_free_box.max_depth = query_bridge_ffb_depth;
        pave_config.adaptive_ffb_depths =
            config_.query_bridge_adaptive_ffb_depths.empty()
                ? pave_config.adaptive_ffb_depths
                : config_.query_bridge_adaptive_ffb_depths;
        pave_config.gap_fill_sample_step = std::min(pave_config.gap_fill_sample_step, 0.02);
        pave_config.gap_fill_time_budget_ms =
            std::max(pave_config.gap_fill_time_budget_ms, short_local_bridge ? 350.0 : 200.0);
        pave_config.gap_fill_max_ffb_calls =
            std::max(pave_config.gap_fill_max_ffb_calls, short_local_bridge ? 768 : 512);
        pave_config.gap_fill_min_arc_gain = 0.0;
        pave_config.require_connected_chain = true;
    }
    int added = 0;
    if (!partition_native_mode()) {
        const std::size_t boxes_before_forward = boxes_.size();
        added = chain_pave_along_path(
            corridor_path,
            start_box_id,
            boxes_,
            *oracle_,
            adjacency_,
            next_id,
            context,
            pave_config);
        if (added > 0) {
            append_partition_after_pave(boxes_before_forward,
                                        "query_bridge.forward_boundary_pave");
            context.diagnostics().add_counter(
                "query_bridge.full_adjacency_rebuilds_avoided");
            invalidate_query_cache();
        }
    } else {
        context.diagnostics().add_counter(
            "query_bridge.partition_legacy_forward_chain_pave_skipped");
    }
    auto [source_box_id, target_box_id] =
        try_reverse_boundary_pave(pave_config, added, added);
    if (added > 0 &&
        source_box_id >= 0 &&
        target_box_id >= 0 &&
        (legacy_query_boxcorridor_enabled() ||
         box_only_path_connected_partition_first(source_box_id, target_box_id))) {
        const int edge_id = add_segment_edge_partition_first(                                             source_box_id,
                                             target_box_id,
                                             corridor_path,
                                             SegmentEdgeType::BoxCorridor,
                                             bridge_rrt.segment_resolution,
                                             SegmentEdgeValidation::CollisionChecked,
                                             false,
                                             bridge_edge_query_index);
        box_corridor_edges_added = edge_id >= 0 ? 1 : 0;
        if (box_corridor_edges_added > 0) {
            invalidate_query_cache();
        }
        return finish_bridge(added + box_corridor_edges_added);
    }
    if (dense_box_corridor_candidate && !dense_repair_attempted && !partition_native_mode()) {
        ChainPaveConfig dense_config = config_.connector.pave;
        dense_config.max_chain = std::max(dense_config.max_chain, 256);
        dense_config.refine_covered_waypoints = true;
        dense_config.fill_gaps = true;
        dense_config.find_free_box.max_depth = query_bridge_ffb_depth;
        dense_config.adaptive_ffb_depths =
            config_.query_bridge_adaptive_ffb_depths.empty()
                ? dense_config.adaptive_ffb_depths
                : config_.query_bridge_adaptive_ffb_depths;
        dense_config.gap_fill_sample_step = 0.0025;
        dense_config.gap_fill_time_budget_ms = 0.0;
        dense_config.gap_fill_max_ffb_calls = -1;
        dense_config.gap_fill_min_arc_gain = 0.0;
        dense_config.require_connected_chain = true;
        const std::size_t boxes_before_dense_retry = boxes_.size();
        dense_repair_added = chain_pave_along_path(
            corridor_path,
            start_box_id,
            boxes_,
            *oracle_,
            adjacency_,
            next_id,
            context,
            dense_config);
        if (dense_repair_added > 0) {
            append_partition_after_pave(boxes_before_dense_retry,
                                        "query_bridge.dense_boundary_retry");
            context.diagnostics().add_counter(
                "query_bridge.full_adjacency_rebuilds_avoided");
            invalidate_query_cache();
        }
        std::tie(source_box_id, target_box_id) =
            try_reverse_boundary_pave(dense_config,
                                      dense_repair_added,
                                      dense_repair_added);
        if (source_box_id >= 0 &&
            target_box_id >= 0 &&
            box_only_path_connected_partition_first(source_box_id, target_box_id)) {
            const int edge_id = add_segment_edge_partition_first(                                                 source_box_id,
                                                 target_box_id,
                                                 corridor_path,
                                                 SegmentEdgeType::BoxCorridor,
                                                 bridge_rrt.segment_resolution,
                                                 SegmentEdgeValidation::CollisionChecked,
                                                 false,
                                                 bridge_edge_query_index);
            box_corridor_edges_added = edge_id >= 0 ? 1 : 0;
            if (box_corridor_edges_added > 0) {
                invalidate_query_cache();
            }
            return finish_bridge(added + dense_repair_added + box_corridor_edges_added);
        }
    }
    if (!partition_native_mode()) {
        IslandConnectorConfig gap_config = config_.connector;
        gap_config.max_total_bridge_boxes = 0;
        IslandConnector gap_connector(*oracle_, robot_, checker, gap_config);
        const auto gap_result = gap_connector.connect_all(boxes_, adjacency_, segment_edges_, next_id, context);
        (void)gap_result;
        invalidate_query_cache();
    } else {
        context.diagnostics().add_counter(
            "query_bridge.partition_legacy_gap_connector_skipped");
    }
    source_box_id = locate_box_partition_first(start, config_.query.nearest_if_outside);
    target_box_id = locate_box_partition_first(goal, config_.query.nearest_if_outside);
    if (added > 0 &&
        source_box_id >= 0 &&
        target_box_id >= 0 &&
        (legacy_query_boxcorridor_enabled() ||
         box_only_path_connected_partition_first(source_box_id, target_box_id))) {
        const int edge_id = add_segment_edge_partition_first(	                                             source_box_id,
	                                             target_box_id,
	                                             corridor_path,
                                             SegmentEdgeType::BoxCorridor,
                                             bridge_rrt.segment_resolution,
                                             SegmentEdgeValidation::CollisionChecked,
                                             false,
                                             bridge_edge_query_index);
        box_corridor_edges_added = edge_id >= 0 ? 1 : 0;
        if (box_corridor_edges_added > 0) {
            invalidate_query_cache();
        }
        return finish_bridge(added + box_corridor_edges_added);
    }
    if (defer_query_segment_edge) {
        const bool max_depth_ffb_failed =
            boundary_max_depth_failure_count_local(context) >
            query_bridge_depth_failures_before + 0.5;
        if (config_.connector.segment_edges_enabled && config_.connector.rrt_segment_edges &&
            max_depth_ffb_failed) {
            if (source_box_id >= 0 && target_box_id >= 0) {
                const PathAuditCheck segment_audit =
                    audit_waypoint_path(corridor_path,
                                        checker,
                                        config_.query.audit_resolution,
                                        config_.query.audit_segment_step);
                if (!segment_audit.passed) {
                    context.diagnostics().add_counter(
                        "query_bridge.segment_edge_audit_rejects");
                    return finish_bridge(added + dense_repair_added + box_corridor_edges_added);
                }
                const int edge_id = add_segment_edge_partition_first(                                                     source_box_id,
                                                     target_box_id,
                                                     corridor_path,
                                                     SegmentEdgeType::QueryBridge,
                                                     bridge_rrt.segment_resolution,
                                                     SegmentEdgeValidation::CollisionChecked,
                                                     true,
                                                     bridge_edge_query_index);
                direct_segment_edges_added = edge_id >= 0 ? 1 : 0;
                if (direct_segment_edges_added > 0) {
                    invalidate_query_cache();
                }
            }
        } else if (config_.connector.segment_edges_enabled &&
                   config_.connector.rrt_segment_edges &&
                   !max_depth_ffb_failed) {
            context.diagnostics().add_counter(
                "query_bridge.segment_edge_blocked_no_max_depth_ffb_failure");
        }
    }
    return finish_bridge(added + dense_repair_added + box_corridor_edges_added + direct_segment_edges_added);
}

std::vector<int> RBFPlanningForest::bridge_queries(const std::vector<Eigen::VectorXd>& starts,
                                                   const std::vector<Eigen::VectorXd>& goals) {
    if (starts.size() != goals.size()) {
        throw std::invalid_argument("bridge_queries requires starts/goals with matching sizes");
    }
    std::vector<int> added_by_query(starts.size(), 0);
    std::size_t partition_refresh_base = boxes_.size();
    const std::size_t segment_edges_before_partition_refresh = segment_edges_.size();
    OracleCounters oracle_counters_before;
    bool oracle_counters_before_valid = false;
    auto finish_batch_bridge = [&]() {
        if (oracle_counters_before_valid && oracle_) {
            const auto after = oracle_->counters();
            auto add_counter_delta = [&](const std::string& key, auto after_value, auto before_value) {
                last_build_.diagnostics[key] +=
                    static_cast<double>(after_value - before_value);
            };
            add_counter_delta("query_bridge.oracle_node_validations",
                              after.node_validations,
                              oracle_counters_before.node_validations);
            add_counter_delta("query_bridge.oracle_validation_cache_hits",
                              after.validation_cache_hits,
                              oracle_counters_before.validation_cache_hits);
            add_counter_delta("query_bridge.oracle_validation_cache_misses",
                              after.validation_cache_misses,
                              oracle_counters_before.validation_cache_misses);
            add_counter_delta("query_bridge.oracle_materializations",
                              after.materializations,
                              oracle_counters_before.materializations);
            add_counter_delta("query_bridge.oracle_external_exact_hits",
                              after.materialization_external_exact_hits,
                              oracle_counters_before.materialization_external_exact_hits);
            add_counter_delta("query_bridge.oracle_external_exact_misses",
                              after.materialization_external_exact_misses,
                              oracle_counters_before.materialization_external_exact_misses);
            add_counter_delta("query_bridge.oracle_interval_replay_compatibility_checks",
                              after.interval_replay_compatibility_checks,
                              oracle_counters_before.interval_replay_compatibility_checks);
            add_counter_delta("query_bridge.oracle_interval_replay_compatible",
                              after.interval_replay_compatible,
                              oracle_counters_before.interval_replay_compatible);
            add_counter_delta("query_bridge.oracle_interval_replay_incompatible",
                              after.interval_replay_incompatible,
                              oracle_counters_before.interval_replay_incompatible);
            add_counter_delta("query_bridge.oracle_interval_replay_direct_exact_hits",
                              after.interval_replay_direct_exact_hits,
                              oracle_counters_before.interval_replay_direct_exact_hits);
            add_counter_delta("query_bridge.oracle_interval_replay_key_only_blocked",
                              after.interval_replay_key_only_blocked,
                              oracle_counters_before.interval_replay_key_only_blocked);
            add_counter_delta("query_bridge.oracle_shared_endpoint_cache_hits",
                              after.materialization_reused_shared_endpoint_cache,
                              oracle_counters_before.materialization_reused_shared_endpoint_cache);
            add_counter_delta("query_bridge.oracle_endpoint_path_ms",
                              after.validate_node_endpoint_path_time_us * 1.0e-3,
                              oracle_counters_before.validate_node_endpoint_path_time_us * 1.0e-3);
            add_counter_delta("query_bridge.oracle_classify_ms",
                              after.validate_node_classify_time_us * 1.0e-3,
                              oracle_counters_before.validate_node_classify_time_us * 1.0e-3);
            add_counter_delta("query_bridge.oracle_validate_total_ms",
                              after.validate_node_total_time_us * 1.0e-3,
                              oracle_counters_before.validate_node_total_time_us * 1.0e-3);
            add_counter_delta("query_bridge.oracle_materialization_endpoint_ms",
                              after.materialization_endpoint_time_us * 1.0e-3,
                              oracle_counters_before.materialization_endpoint_time_us * 1.0e-3);
            add_counter_delta("query_bridge.oracle_materialization_envelope_ms",
                              after.materialization_envelope_time_us * 1.0e-3,
                              oracle_counters_before.materialization_envelope_time_us * 1.0e-3);
            add_counter_delta("query_bridge.oracle_envelope_collision_queries",
                              after.envelope_collision_queries,
                              oracle_counters_before.envelope_collision_queries);
            add_counter_delta("query_bridge.oracle_envelope_gjk_tests",
                              after.envelope_collision_gjk_tests,
                              oracle_counters_before.envelope_collision_gjk_tests);
        }
        const bool changed = boxes_.size() != partition_refresh_base ||
                             segment_edges_.size() != segment_edges_before_partition_refresh ||
                             std::any_of(added_by_query.begin(),
                                         added_by_query.end(),
                                         [](int added) { return added > 0; });
        if (boxes_.size() > partition_refresh_base) {
            append_adaptive_partition_boxes(partition_refresh_base,
                                            &last_build_,
                                            "query_bridge.batch");
            partition_refresh_base = boxes_.size();
        } else if (changed) {
            sync_adaptive_partition_segment_edges(&last_build_, "query_bridge.batch");
            refresh_adaptive_partition_diagnostics(&last_build_);
        }
        return added_by_query;
    };
    if (starts.empty() || !oracle_) {
        return added_by_query;
    }
    oracle_counters_before = oracle_->counters();
    oracle_counters_before_valid = true;

    struct BridgeSearchTask {
        std::size_t index = 0;
        int query_index = 0;
        Eigen::VectorXd start;
        Eigen::VectorXd goal;
        bool short_local_bridge = false;
        RRTConnectConfig bridge_rrt;
        std::vector<RRTConnectConfig> short_local_profiles;
        int attempts = 1;
        std::vector<Eigen::VectorXd> waypoint_path;
        bool waypoint_path_from_partition_query = false;
        std::vector<Eigen::VectorXd> hipac_candidate_path;
        bool hipac_online_satisfied = false;
        int hipac_prebridge_resolves_used = 0;
        int hipac_transition_resolves_used = 0;
        int hipac_online_resolves_used = 0;
    };
    struct BridgeSearchJob {
        std::size_t task_index = 0;
        int attempt = 0;
    };

    const double bridge_accept_segment_fraction =
        std::max(0.0, env_double_or_default("RBF_QUERY_BRIDGE_ACCEPT_SEGMENT_FRACTION", 0.25));
    const double bridge_accept_path_ratio =
        std::max(0.0, env_double_or_default("RBF_QUERY_BRIDGE_ACCEPT_PATH_RATIO", 1.50));
    const double bridge_accept_path_additive =
        std::max(0.0, env_double_or_default("RBF_QUERY_BRIDGE_ACCEPT_PATH_ADDITIVE", 0.75));
    const double bridge_accept_max_path_length =
        std::max(0.0, env_double_or_default("RBF_QUERY_BRIDGE_ADAPTIVE_MAX_PATH_LENGTH", 4.5));
    auto query_result_good = [&](const QueryResult& current,
                                 const Eigen::VectorXd& start,
                                 const Eigen::VectorXd& goal) {
        if (!current.success || !current.audit_passed) {
            return false;
        }
        const double raw_length =
            current.raw_path_length > 1e-12 ? current.raw_path_length : current.path_length;
        const double segment_fraction =
            raw_length > 1e-12 ? current.segment_edge_length / raw_length
                               : std::numeric_limits<double>::infinity();
        if (!(segment_fraction <= bridge_accept_segment_fraction)) {
            return false;
        }
        const double direct = (goal - start).norm();
        return direct <= 1e-9 ||
               current.path_length <= std::max(direct * bridge_accept_path_ratio,
                                                direct + bridge_accept_path_additive) ||
               current.path_length <= bridge_accept_max_path_length;
    };
    auto locate_existing_box_for_query_bridge = [&](const Eigen::Ref<const Eigen::VectorXd>& point) {
        if (partition_native_mode()) {
            return locate_box_partition_first(point, config_.query.nearest_if_outside);
        }
        for (const auto& box : boxes_) {
            if (box.contains(point, config_.query.adjacency_tolerance)) {
                return box.id;
            }
        }
        if (!config_.query.nearest_if_outside) {
            return -1;
        }
        return locate_box_partition_first(point, config_.query.nearest_if_outside);
    };
    auto catch_up_query_bridge_partition = [&](const char* diagnostic_prefix) {
        if (partition_native_mode() &&
            adaptive_partition_query_enabled_ &&
            adaptive_partition_ &&
            boxes_.size() > partition_refresh_base) {
            append_adaptive_partition_boxes(partition_refresh_base,
                                            &last_build_,
                                            diagnostic_prefix);
            partition_refresh_base = boxes_.size();
        }
    };
    auto query_bridge_forced_index = [](std::size_t index) {
        return env_index_list_contains("RBF_QUERY_BRIDGE_FORCE_INDICES", index);
    };
    const bool partition_path_first =
        partition_native_mode() &&
        env_int_or_default("RBF_QUERY_BRIDGE_PARTITION_PATH_FIRST", 0) != 0;
    const bool partition_path_first_allow_long =
        env_int_or_default("RBF_QUERY_BRIDGE_PARTITION_PATH_FIRST_ALLOW_LONG", 0) != 0;
    const double partition_path_first_max_segment_fraction =
        std::max(0.0,
                 env_double_or_default("RBF_QUERY_BRIDGE_PARTITION_PATH_FIRST_MAX_SEGMENT_FRACTION",
                                       0.95));

    std::vector<BridgeSearchTask> tasks;
    tasks.reserve(starts.size());
    for (std::size_t index = 0; index < starts.size(); ++index) {
        if (starts[index].size() != goals[index].size()) {
            throw std::invalid_argument("bridge_queries received a start/goal dimension mismatch");
        }
        const bool forced_task = query_bridge_forced_index(index);
        auto mark_task_skip = [&](double code, const char* reason) {
            last_build_.diagnostics["query_bridge.batch_task." +
                                    std::to_string(index) +
                                    ".skip_reason_code"] = code;
            if (reason != nullptr && reason[0] != '\0') {
                last_build_.diagnostics[std::string("query_bridge.batch_task_skip.") + reason] += 1.0;
            }
        };
        QueryResult initial_query;
        bool has_initial_query = false;
	        if (!forced_task || partition_path_first) {
	            initial_query = query(starts[index], goals[index]);
	            has_initial_query = true;
	            if (!forced_task && query_result_good(initial_query, starts[index], goals[index])) {
	                mark_task_skip(1.0, "initial_good");
	                continue;
            }
        }
        int start_box_id = locate_existing_box_for_query_bridge(starts[index]);
        if (start_box_id < 0) {
            StageContext anchor_context = StageContext::from_runtime(config_.runtime);
            start_box_id = anchor_query_endpoint_box(starts[index], anchor_context);
            merge_diagnostic_snapshot(last_build_.diagnostics, anchor_context.diagnostics().snapshot());
        }
        if (start_box_id < 0) {
            mark_task_skip(2.0, "start_anchor_failed");
            continue;
        }
        int goal_box_id = locate_existing_box_for_query_bridge(goals[index]);
        if (goal_box_id < 0) {
            StageContext anchor_context = StageContext::from_runtime(config_.runtime);
            goal_box_id = anchor_query_endpoint_box(goals[index], anchor_context);
            merge_diagnostic_snapshot(last_build_.diagnostics, anchor_context.diagnostics().snapshot());
        }
        catch_up_query_bridge_partition("query_bridge.endpoint_anchor");
        if (start_box_id >= 0) {
            start_box_id = locate_existing_box_for_query_bridge(starts[index]);
        }
        if (goal_box_id >= 0) {
            goal_box_id = locate_existing_box_for_query_bridge(goals[index]);
        }
        if (goal_box_id < 0 || goal_box_id == start_box_id) {
            mark_task_skip(goal_box_id < 0 ? 3.0 : 4.0,
                           goal_box_id < 0 ? "goal_anchor_failed" : "same_box");
            continue;
        }

        BridgeSearchTask task;
        task.index = index;
        task.query_index =
            env_index_list_value_or_default("RBF_QUERY_BRIDGE_GLOBAL_INDICES",
                                            index,
                                            static_cast<int>(index));
        last_build_.diagnostics["query_bridge.batch_task." +
                                std::to_string(index) +
                                ".global_index"] = static_cast<double>(task.query_index);
	        task.start = starts[index];
	        task.goal = goals[index];
	        if (last_adaptive_partition_config_.hipac_online_connectivity &&
	            has_initial_query &&
	            initial_query.success &&
	            initial_query.audit_passed &&
	            !initial_query.path.empty()) {
	            task.hipac_candidate_path = initial_query.path;
	        }
	        task.bridge_rrt = with_query_root_hull_domain(config_.connector.rrt, *oracle_, task.start, task.goal);
        task.bridge_rrt.segment_resolution =
            std::max(task.bridge_rrt.segment_resolution, config_.query.audit_resolution);
        if (partition_path_first &&
            has_initial_query &&
            initial_query.success &&
            initial_query.audit_passed &&
            !initial_query.path.empty()) {
            last_build_.diagnostics["query_bridge.partition_path_first_initial_success"] += 1.0;
            const double direct = (task.goal - task.start).norm();
            const double raw_length = initial_query.raw_path_length > 1e-12
                ? initial_query.raw_path_length
                : initial_query.path_length;
            const double segment_fraction =
                raw_length > 1e-12
                    ? initial_query.segment_edge_length / raw_length
                    : std::numeric_limits<double>::infinity();
            const bool segment_reasonable =
                std::isfinite(segment_fraction) &&
                segment_fraction <= partition_path_first_max_segment_fraction;
            const bool length_reasonable =
                direct <= 1e-9 ||
                initial_query.path_length <= std::max(direct * bridge_accept_path_ratio,
                                                      direct + bridge_accept_path_additive) ||
                initial_query.path_length <= bridge_accept_max_path_length;
            if (!segment_reasonable) {
                last_build_.diagnostics["query_bridge.partition_path_first_reject_segment"] += 1.0;
            }
            if (!length_reasonable) {
                last_build_.diagnostics["query_bridge.partition_path_first_reject_length"] += 1.0;
            }
	            if (segment_reasonable && (length_reasonable || partition_path_first_allow_long)) {
	                task.waypoint_path = initial_query.path;
	                task.waypoint_path_from_partition_query = true;
	                if (task.hipac_candidate_path.empty()) {
	                    task.hipac_candidate_path = initial_query.path;
	                }
	                last_build_.diagnostics["query_bridge.partition_path_first_accepted"] += 1.0;
	            }
        }
        const double bridge_distance = (task.goal - task.start).norm();
        task.short_local_bridge = bridge_distance > 0.55 && bridge_distance < 0.85;
        if (task.short_local_bridge) {
            task.bridge_rrt.step_size = std::min(task.bridge_rrt.step_size, 0.25);
            task.bridge_rrt.goal_bias = 0.08;
            task.bridge_rrt.local_sampling_radius =
                task.bridge_rrt.local_sampling_radius > 0.0
                    ? std::min(task.bridge_rrt.local_sampling_radius, 0.85)
                    : 0.85;
            auto add_profile = [&](double step_size, double goal_bias, double radius) {
                RRTConnectConfig profile = task.bridge_rrt;
                profile.step_size = step_size;
                profile.goal_bias = goal_bias;
                profile.local_sampling_radius = radius;
                profile.shortcut_path = true;
                task.short_local_profiles.push_back(std::move(profile));
            };
            add_profile(0.25, 0.08, 0.90);
            add_profile(0.50, 0.20, 1.00);
            add_profile(0.35, 0.10, 1.00);
            add_profile(0.25, 0.08, 0.45);
        }
        task.attempts = std::max(1, config_.connector.max_pairs_per_gap);
        tasks.push_back(std::move(task));
    }

    std::stable_sort(tasks.begin(), tasks.end(), [](const BridgeSearchTask& lhs,
                                                    const BridgeSearchTask& rhs) {
        const bool lhs_short = lhs.short_local_bridge;
        const bool rhs_short = rhs.short_local_bridge;
        if (lhs_short != rhs_short) {
            return !lhs_short && rhs_short;
        }
        return lhs.index < rhs.index;
    });

    if (tasks.empty()) {
        return finish_batch_bridge();
    }
    using Clock = std::chrono::steady_clock;
    const auto batch_t0 = Clock::now();
    StageContext batch_context = StageContext::from_runtime(config_.runtime);
    const bool scene_reusable_query_bridge_edges =
        env_int_or_default("RBF_QUERY_BRIDGE_SCENE_REUSABLE_EDGES", 0) != 0;
    batch_context.diagnostics().set_value("query_bridge.scene_reusable_edges",
                                          scene_reusable_query_bridge_edges ? 1.0 : 0.0);
    struct BatchBridgeDiagnosticsFlush {
        BuildProfile& profile;
        StageContext& context;
        ~BatchBridgeDiagnosticsFlush() {
            for (const auto& [key, value] : context.diagnostics().snapshot()) {
                profile.diagnostics[key] = value;
            }
        }
    } batch_diagnostics_flush{last_build_, batch_context};
    auto elapsed_ms_since = [](Clock::time_point t0) {
        return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
    };
    auto task_key = [](std::size_t index, const std::string& suffix) {
        return "query_bridge.batch_task." + std::to_string(index) + "." + suffix;
    };
    auto edge_query_index_for = [&](const BridgeSearchTask& task) {
        return scene_reusable_query_bridge_edges ? -1 : task.query_index;
    };
	    auto accumulate_task_direct_corridor_totals = [&](std::size_t index) {
	        auto add = [&](const std::string& suffix, const std::string& total_key) {
	            const auto it = last_build_.diagnostics.find(task_key(index, suffix));
            if (it != last_build_.diagnostics.end()) {
                batch_context.diagnostics().add_counter(total_key, it->second);
            }
        };
        add("direct_corridor_ms", "query_bridge.direct_corridor_ms_total");
        add("direct_corridor_samples", "query_bridge.direct_corridor_samples_total");
        add("direct_corridor_ffb_calls", "query_bridge.direct_corridor_ffb_calls_total");
        add("direct_corridor_all_ffb_calls", "query_bridge.direct_corridor_all_ffb_calls_total");
        add("direct_corridor_direct_ffb_ms", "query_bridge.direct_corridor_direct_ffb_ms");
        add("direct_corridor_repair_ffb_ms", "query_bridge.direct_corridor_repair_ffb_ms");
        add("direct_corridor_adaptive_repair_ffb_ms",
            "query_bridge.direct_corridor_adaptive_repair_ffb_ms");
        add("direct_corridor_lateral_repair_ffb_ms",
            "query_bridge.direct_corridor_lateral_repair_ffb_ms");
        add("direct_corridor_segment_audit_ms",
            "query_bridge.direct_corridor_segment_audit_ms");
        add("direct_corridor_added", "query_bridge.direct_corridor_added_total");
        add("direct_corridor_repair_calls", "query_bridge.direct_corridor_repair_calls_total");
        add("direct_corridor_repair_added", "query_bridge.direct_corridor_repair_added_total");
        add("direct_corridor_adaptive_repair_calls",
            "query_bridge.direct_corridor_adaptive_repair_calls_total");
        add("direct_corridor_adaptive_repair_added",
            "query_bridge.direct_corridor_adaptive_repair_added_total");
        add("direct_corridor_lateral_repair_calls",
            "query_bridge.direct_corridor_lateral_repair_calls_total");
        add("direct_corridor_lateral_repair_added",
            "query_bridge.direct_corridor_lateral_repair_added_total");
        add("direct_corridor_bad_initial", "query_bridge.direct_corridor_bad_initial_total");
        add("direct_corridor_bad_final", "query_bridge.direct_corridor_bad_final_total");
        add("direct_corridor_segment_edges", "query_bridge.direct_corridor_segment_edges_total");
        add("direct_corridor_transition_connected_ms",
            "query_bridge.direct_corridor_transition_connected_ms");
        add("direct_corridor_transition_connected_calls",
            "query_bridge.direct_corridor_transition_connected_calls");
        add("direct_corridor_bad_transitions_ms",
            "query_bridge.direct_corridor_bad_transitions_ms");
        add("direct_corridor_bad_transitions_calls",
            "query_bridge.direct_corridor_bad_transitions_calls");
        add("direct_corridor_current_cover_ms",
            "query_bridge.direct_corridor_current_cover_ms");
        add("direct_corridor_current_cover_calls",
            "query_bridge.direct_corridor_current_cover_calls");
        add("direct_corridor_current_cover_partition_ms",
            "query_bridge.direct_corridor_current_cover_partition_ms");
        add("direct_corridor_current_cover_corridor_scan_ms",
            "query_bridge.direct_corridor_current_cover_corridor_scan_ms");
        add("direct_corridor_current_cover_direct_index_ms",
            "query_bridge.direct_corridor_current_cover_direct_index_ms");
        add("direct_corridor_duplicate_lookup_ms",
            "query_bridge.direct_corridor_duplicate_lookup_ms");
        add("direct_corridor_duplicate_lookup_calls",
            "query_bridge.direct_corridor_duplicate_lookup_calls");
        add("direct_corridor_commit_total_ms",
            "query_bridge.direct_corridor_commit_total_ms");
        add("direct_corridor_commit_calls",
            "query_bridge.direct_corridor_commit_calls");
        add("direct_corridor_commit_dynamic_policy_ms",
            "query_bridge.direct_corridor_commit_dynamic_policy_ms");
        add("direct_corridor_commit_partition_append_ms",
            "query_bridge.direct_corridor_commit_partition_append_ms");
        add("direct_corridor_assimilate_calls",
            "query_bridge.direct_corridor_assimilate_calls");
        add("direct_corridor_assimilate_sample_scan_ms",
            "query_bridge.direct_corridor_assimilate_sample_scan_ms");
        add("direct_corridor_assimilate_candidate_build_ms",
            "query_bridge.direct_corridor_assimilate_candidate_build_ms");
        add("direct_corridor_assimilate_adjacency_ms",
            "query_bridge.direct_corridor_assimilate_adjacency_ms");
        add("direct_corridor_segment_insert_ms",
            "query_bridge.direct_corridor_segment_insert_ms");
        add("direct_corridor_segment_insert_calls",
            "query_bridge.direct_corridor_segment_insert_calls");
        add("direct_corridor_direct_task_build_ms",
            "query_bridge.direct_corridor_direct_task_build_ms");
        add("direct_corridor_direct_loop_ms",
            "query_bridge.direct_corridor_direct_loop_ms");
        add("direct_corridor_repair_loop_ms",
            "query_bridge.direct_corridor_repair_loop_ms");
        add("direct_corridor_adaptive_loop_ms",
            "query_bridge.direct_corridor_adaptive_loop_ms");
        add("direct_corridor_lateral_loop_ms",
            "query_bridge.direct_corridor_lateral_loop_ms");
	        add("direct_corridor_residual_segment_loop_ms",
	            "query_bridge.direct_corridor_residual_segment_loop_ms");
	    };
	    auto point_segment_distance_sq = [](const Eigen::VectorXd& point,
	                                        const Eigen::VectorXd& a,
	                                        const Eigen::VectorXd& b) {
	        if (point.size() != a.size() || point.size() != b.size()) {
	            return std::numeric_limits<double>::infinity();
	        }
	        const Eigen::VectorXd ab = b - a;
	        const double denom = ab.squaredNorm();
	        if (denom <= 1e-18) {
	            return (point - a).squaredNorm();
	        }
	        const double t = std::clamp((point - a).dot(ab) / denom, 0.0, 1.0);
	        return (point - (a + t * ab)).squaredNorm();
	    };
	    auto point_polyline_distance_sq = [&](const Eigen::VectorXd& point,
	                                          const std::vector<Eigen::VectorXd>& path) {
	        if (path.empty()) {
	            return std::numeric_limits<double>::infinity();
	        }
	        double best = std::numeric_limits<double>::infinity();
	        for (std::size_t index = 1; index < path.size(); ++index) {
	            best = std::min(best,
	                            point_segment_distance_sq(point,
	                                                      path[index - 1],
	                                                      path[index]));
	        }
	        if (path.size() == 1) {
	            best = (point - path.front()).squaredNorm();
	        }
	        return best;
	    };
	    auto try_hipac_prebridge_portal = [&](BridgeSearchTask& task) -> int {
	        if (!last_adaptive_partition_config_.hipac_online_connectivity ||
	            !last_adaptive_partition_config_.hipac_online_before_query_bridge ||
	            !last_adaptive_partition_config_.hipac_online_prebridge_portal ||
	            !partition_native_mode() ||
	            !adaptive_partition_query_enabled_ ||
	            !adaptive_partition_ ||
		            adaptive_partition_->empty() ||
		            task.hipac_prebridge_resolves_used >=
		                std::max(0, last_adaptive_partition_config_.hipac_online_max_resolves_per_query)) {
		            return 0;
		        }
		        std::vector<Eigen::VectorXd> coarse_route = task.hipac_candidate_path;
		        if (coarse_route.size() < 2) {
		            coarse_route = {task.start, task.goal};
		            batch_context.diagnostics().add_counter(
		                "query_bridge.hipac_prebridge_direct_query_route");
		        }
		        if (coarse_route.size() < 2) {
		            return 0;
		        }
		        const auto prebridge_t0 = Clock::now();
	        batch_context.diagnostics().add_counter("query_bridge.hipac_prebridge_attempts");
	        batch_context.diagnostics().set_value(task_key(task.index, "hipac_prebridge_attempt"),
	                                              1.0);
	        const int candidate_limit =
	            std::max(1, last_adaptive_partition_config_.hipac_online_prebridge_candidate_limit);
	        const auto candidate_pairs =
	            adaptive_partition_->nearest_component_pairs_to_largest(1, candidate_limit);
	        batch_context.diagnostics().add_counter("query_bridge.hipac_prebridge_candidates",
	                                                static_cast<double>(candidate_pairs.size()));
	        if (candidate_pairs.empty()) {
	            batch_context.diagnostics().add_counter("query_bridge.hipac_prebridge_no_candidates");
	            return 0;
	        }

	        std::unordered_map<int, int> component_by_box;
	        const auto components = adaptive_partition_->component_box_ids_with_overlay();
	        for (std::size_t component_index = 0; component_index < components.size(); ++component_index) {
	            for (int box_id : components[component_index]) {
	                component_by_box.emplace(box_id, static_cast<int>(component_index));
	            }
	        }
	        const int start_box_id = locate_box_partition_first(task.start, false);
	        const int goal_box_id = locate_box_partition_first(task.goal, false);
	        const int start_component =
	            component_by_box.count(start_box_id) > 0 ? component_by_box[start_box_id] : -1;
	        const int goal_component =
	            component_by_box.count(goal_box_id) > 0 ? component_by_box[goal_box_id] : -1;
	        const bool has_endpoint_component_target =
	            start_component > 0 || goal_component > 0;

	        const double max_pair_distance =
	            std::max(0.0, last_adaptive_partition_config_.hipac_online_prebridge_max_pair_distance);
	        const double route_weight =
	            std::max(0.0,
	                     last_adaptive_partition_config_.hipac_online_prebridge_route_distance_weight);
	        const double pair_weight =
	            std::max(0.0,
	                     last_adaptive_partition_config_.hipac_online_prebridge_pair_distance_weight);
	        const AdaptiveGridPartitionComponentPair* best_pair = nullptr;
	        double best_score = std::numeric_limits<double>::infinity();
	        int considered = 0;
	        int distance_rejects = 0;
	        int endpoint_component_rejects = 0;
	        for (const auto& pair : candidate_pairs) {
	            if (pair.source_box_id < 0 ||
	                pair.target_box_id < 0 ||
	                pair.source_point.size() == 0 ||
	                pair.target_point.size() == 0 ||
	                pair.source_point.size() != pair.target_point.size()) {
	                continue;
	            }
	            const bool matches_endpoint_component =
	                (start_component > 0 && pair.source_component_index == start_component) ||
	                (goal_component > 0 && pair.source_component_index == goal_component);
	            if (has_endpoint_component_target && !matches_endpoint_component) {
	                ++endpoint_component_rejects;
	                continue;
	            }
	            const double pair_distance = std::sqrt(std::max(0.0, pair.distance_sq));
	            if (max_pair_distance > 0.0 &&
	                pair_distance > max_pair_distance + 1e-12) {
	                ++distance_rejects;
	                continue;
	            }
	            const Eigen::VectorXd midpoint = 0.5 * (pair.source_point + pair.target_point);
		            const double route_distance =
		                std::sqrt(std::max(0.0,
		                                   point_polyline_distance_sq(midpoint,
		                                                              coarse_route)));
	            const bool touches_start =
	                start_component >= 0 &&
	                (pair.source_component_index == start_component ||
	                 pair.target_component_index == start_component);
	            const bool touches_goal =
	                goal_component >= 0 &&
	                (pair.source_component_index == goal_component ||
	                 pair.target_component_index == goal_component);
	            const double endpoint_bonus = (touches_start ? 0.50 : 0.0) +
	                                          (touches_goal ? 0.50 : 0.0);
	            const double component_size_bonus =
	                0.02 * std::log1p(static_cast<double>(
	                    std::max(0, pair.source_component_size)));
	            const double score = route_weight * route_distance +
	                                 pair_weight * pair_distance -
	                                 endpoint_bonus -
	                                 component_size_bonus;
	            ++considered;
	            if (score < best_score) {
	                best_score = score;
	                best_pair = &pair;
	            }
	        }
	        batch_context.diagnostics().add_counter("query_bridge.hipac_prebridge_considered",
	                                                static_cast<double>(considered));
	        batch_context.diagnostics().add_counter("query_bridge.hipac_prebridge_distance_rejects",
	                                                static_cast<double>(distance_rejects));
	        batch_context.diagnostics().add_counter(
	            "query_bridge.hipac_prebridge_endpoint_component_rejects",
	            static_cast<double>(endpoint_component_rejects));
	        if (best_pair == nullptr) {
	            batch_context.diagnostics().add_counter("query_bridge.hipac_prebridge_no_candidate_after_filter");
	            return 0;
	        }

	        task.hipac_prebridge_resolves_used += 1;
	        batch_context.diagnostics().add_counter("query_bridge.hipac_prebridge_portal_attempts");
	        batch_context.diagnostics().set_value(task_key(task.index, "hipac_prebridge_score"),
	                                              best_score);
	        batch_context.diagnostics().set_value(task_key(task.index, "hipac_prebridge_pair_distance"),
	                                              std::sqrt(std::max(0.0, best_pair->distance_sq)));
	        std::vector<Eigen::VectorXd> local_path{best_pair->source_point, best_pair->target_point};
	        const int added = add_partition_portal_corridor_overlay(best_pair->source_point,
	                                                                best_pair->target_point,
	                                                                local_path,
	                                                                "query_bridge.hipac_online_prebridge",
	                                                                false,
	                                                                true,
	                                                                edge_query_index_for(task),
	                                                                &last_build_);
	        const double prebridge_ms = elapsed_ms_since(prebridge_t0);
	        batch_context.diagnostics().record_timing("query_bridge.hipac_prebridge_ms_total",
	                                                  prebridge_ms);
	        batch_context.diagnostics().add_counter("query_bridge.hipac_prebridge_ms_total",
	                                                prebridge_ms);
	        batch_context.diagnostics().set_value(task_key(task.index, "hipac_prebridge_ms"),
	                                              prebridge_ms);
	        if (added <= 0) {
	            batch_context.diagnostics().add_counter("query_bridge.hipac_prebridge_failures");
	            return 0;
	        }
	        batch_context.diagnostics().add_counter("query_bridge.hipac_prebridge_added",
	                                                static_cast<double>(added));
	        batch_context.diagnostics().set_value(task_key(task.index, "hipac_prebridge_added"),
	                                              static_cast<double>(added));
	        added_by_query[task.index] += added;
	        const QueryResult probe_after_prebridge = query(task.start, task.goal);
	        if (query_result_good(probe_after_prebridge, task.start, task.goal)) {
	            task.hipac_online_satisfied = true;
	            batch_context.diagnostics().add_counter("query_bridge.hipac_prebridge_satisfied");
	            batch_context.diagnostics().set_value(task_key(task.index, "hipac_prebridge_satisfied"),
	                                                  1.0);
	        } else {
	            batch_context.diagnostics().add_counter("query_bridge.hipac_prebridge_not_sufficient");
	        }
	        return added;
	    };
	    auto try_hipac_online_bridge = [&](BridgeSearchTask& task) -> int {
	        if (!last_adaptive_partition_config_.hipac_online_connectivity ||
	            !last_adaptive_partition_config_.hipac_online_before_query_bridge ||
	            !partition_native_mode() ||
	            task.hipac_candidate_path.size() < 2 ||
	            task.hipac_online_resolves_used >=
	                std::max(0, last_adaptive_partition_config_.hipac_online_max_resolves_per_query)) {
	            return 0;
	        }
	        task.hipac_online_resolves_used += 1;
	        const auto hipac_t0 = Clock::now();
	        batch_context.diagnostics().add_counter("query_bridge.hipac_online_attempts");
	        batch_context.diagnostics().set_value(task_key(task.index, "hipac_online_attempt"),
	                                              1.0);
	        std::vector<Eigen::VectorXd> hipac_path = task.hipac_candidate_path;
	        if (hipac_path.size() > 2) {
	            CollisionChecker checker = make_audit_checker(audit_robot_, scene_, config_.query);
	            const double before_length = path_length(hipac_path);
	            std::vector<Eigen::VectorXd> shortened =
	                collision_shortcut_path(hipac_path,
	                                        checker,
	                                        collision_shortcut_resolution(config_.query));
	            if (shortened.size() >= 2 &&
	                path_length(shortened) <= before_length + 1e-12) {
	                const PathAuditCheck audit =
	                    audit_waypoint_path(shortened,
	                                        checker,
	                                        config_.query.audit_resolution,
	                                        config_.query.audit_segment_step);
	                if (audit.passed) {
	                    batch_context.diagnostics().add_counter(
	                        "query_bridge.hipac_online_shortcut_accepts");
	                    batch_context.diagnostics().add_counter(
	                        "query_bridge.hipac_online_shortcut_delta",
	                        std::max(0.0, before_length - path_length(shortened)));
	                    hipac_path = std::move(shortened);
	                } else {
	                    batch_context.diagnostics().add_counter(
	                        "query_bridge.hipac_online_shortcut_audit_rejects");
	                }
	            }
	        }
	        const double hipac_candidate_length = path_length(hipac_path);
	        const double hipac_candidate_max_length =
	            std::max(0.0, last_adaptive_partition_config_.hipac_online_candidate_max_length);
	        batch_context.diagnostics().add_counter("query_bridge.hipac_online_candidate_length",
	                                                hipac_candidate_length);
	        if (hipac_candidate_max_length > 0.0 &&
	            hipac_candidate_length > hipac_candidate_max_length + 1e-12) {
	            batch_context.diagnostics().add_counter(
	                "query_bridge.hipac_online_candidate_length_rejects");
	            batch_context.diagnostics().set_value(task_key(task.index, "hipac_online_length_reject"),
	                                                  1.0);
	            const double hipac_ms = elapsed_ms_since(hipac_t0);
	            batch_context.diagnostics().record_timing("query_bridge.hipac_online_ms_total",
	                                                      hipac_ms);
	            batch_context.diagnostics().add_counter("query_bridge.hipac_online_ms_total",
	                                                    hipac_ms);
	            batch_context.diagnostics().set_value(task_key(task.index, "hipac_online_ms"),
	                                                  hipac_ms);
	            batch_context.diagnostics().add_counter("query_bridge.hipac_online_failures");
	            return 0;
	        }
	        int added = add_partition_box_corridor_overlay(task.start,
	                                                       task.goal,
	                                                       hipac_path,
	                                                       "query_bridge.hipac_online",
	                                                       true,
	                                                       false,
	                                                       edge_query_index_for(task),
	                                                       &last_build_);
	        if (added <= 0 &&
	            last_adaptive_partition_config_.hipac_online_ffb_portal_fallback) {
	            added = add_partition_portal_corridor_overlay(task.start,
	                                                          task.goal,
	                                                          hipac_path,
	                                                          "query_bridge.hipac_online",
	                                                          true,
	                                                          false,
	                                                          edge_query_index_for(task),
	                                                          &last_build_);
	        }
	        const double hipac_ms = elapsed_ms_since(hipac_t0);
	        batch_context.diagnostics().record_timing("query_bridge.hipac_online_ms_total",
	                                                  hipac_ms);
	        batch_context.diagnostics().add_counter("query_bridge.hipac_online_ms_total",
	                                                hipac_ms);
	        batch_context.diagnostics().set_value(task_key(task.index, "hipac_online_ms"),
	                                              hipac_ms);
	        if (added <= 0) {
	            batch_context.diagnostics().add_counter("query_bridge.hipac_online_failures");
	            return 0;
	        }
	        batch_context.diagnostics().add_counter("query_bridge.hipac_online_added",
	                                                static_cast<double>(added));
	        batch_context.diagnostics().set_value(task_key(task.index, "hipac_online_added"),
	                                              static_cast<double>(added));
	        added_by_query[task.index] += added;
	        const QueryResult probe_after_hipac = query(task.start, task.goal);
	        if (query_result_good(probe_after_hipac, task.start, task.goal)) {
	            task.hipac_online_satisfied = true;
	            batch_context.diagnostics().add_counter("query_bridge.hipac_online_satisfied");
	            batch_context.diagnostics().set_value(task_key(task.index, "hipac_online_satisfied"),
	                                                  1.0);
	        } else {
	            batch_context.diagnostics().add_counter("query_bridge.hipac_online_not_sufficient");
	        }
	        return added;
	    };
	    auto try_hipac_transition_portal = [&](BridgeSearchTask& task) -> int {
	        if (!last_adaptive_partition_config_.hipac_online_connectivity ||
	            !last_adaptive_partition_config_.hipac_online_before_query_bridge ||
	            !last_adaptive_partition_config_.hipac_online_transition_portal ||
	            !partition_native_mode() ||
	            !adaptive_partition_query_enabled_ ||
	            !adaptive_partition_ ||
	            adaptive_partition_->empty() ||
	            task.waypoint_path.size() < 2 ||
	            task.hipac_transition_resolves_used >=
	                std::max(0, last_adaptive_partition_config_.hipac_transition_max_attempts_per_query)) {
	            return 0;
	        }
	        const bool target_index =
	            csv_index_list_contains(last_adaptive_partition_config_.hipac_transition_target_query_indices,
	                                    static_cast<int>(task.index)) ||
	            csv_index_list_contains(last_adaptive_partition_config_.hipac_transition_target_query_indices,
	                                    task.query_index);
	        if (!target_index) {
	            batch_context.diagnostics().add_counter("query_bridge.hipac_transition_target_rejects");
	            return 0;
	        }

	        const auto transition_t0 = Clock::now();
	        batch_context.diagnostics().add_counter("query_bridge.hipac_transition_attempts");
	        batch_context.diagnostics().set_value(task_key(task.index, "hipac_transition_attempt"),
	                                              1.0);
	        struct TransitionCandidate {
	            int source_box_id = -1;
	            int target_box_id = -1;
	            int source_component = -1;
	            int target_component = -1;
	            int first_waypoint = 0;
	            int last_waypoint = 0;
	            Eigen::VectorXd source_point;
	            Eigen::VectorXd target_point;
	            std::vector<Eigen::VectorXd> local_path;
	            double pair_distance = 0.0;
	            double local_length = 0.0;
	            int predicted_bridge_edges = 0;
	            double score = -std::numeric_limits<double>::infinity();
	        };
	        const int stride =
	            std::max(1, last_adaptive_partition_config_.hipac_transition_window_stride);
	        const int candidate_limit =
	            std::max(1, last_adaptive_partition_config_.hipac_transition_candidate_limit);
	        const int min_predicted_edges =
	            std::max(0, last_adaptive_partition_config_.hipac_transition_min_predicted_bridge_edges);
	        const double max_pair_distance =
	            std::max(0.0, last_adaptive_partition_config_.hipac_transition_max_pair_distance);
	        const double sample_step =
	            std::max(1e-4,
	                     env_indexed_double_or_default(
	                         "RBF_QUERY_BRIDGE_DIRECT_SAMPLE_STEP",
	                         task.query_index,
	                         env_double_or_default("RBF_QUERY_BRIDGE_DIRECT_SAMPLE_STEP",
	                                               config_.query.audit_segment_step > 0.0
	                                                   ? config_.query.audit_segment_step
	                                                   : 0.01)));
	        std::unordered_map<int, int> component_by_box;
	        const auto components = adaptive_partition_->component_box_ids_with_overlay();
	        for (std::size_t component_index = 0; component_index < components.size(); ++component_index) {
	            for (int box_id : components[component_index]) {
	                component_by_box.emplace(box_id, static_cast<int>(component_index));
	            }
	        }

	        std::vector<TransitionCandidate> candidates;
	        candidates.reserve(static_cast<std::size_t>(candidate_limit));
	        int gated = 0;
	        int same_component_gated = 0;
	        int distance_gated = 0;
	        int edge_gated = 0;
	        for (std::size_t begin = 0; begin + 1 < task.waypoint_path.size(); ++begin) {
	            const std::size_t end =
	                std::min(task.waypoint_path.size() - 1,
	                         begin + static_cast<std::size_t>(stride));
	            if (end <= begin) {
	                continue;
	            }
	            const auto source_nearest =
	                adaptive_partition_->nearest_boxes(task.waypoint_path[begin], {}, 1);
	            const auto target_nearest =
	                adaptive_partition_->nearest_boxes(task.waypoint_path[end], {}, 1);
	            if (source_nearest.empty() || target_nearest.empty()) {
	                ++gated;
	                continue;
	            }
	            const auto& source = source_nearest.front();
	            const auto& target = target_nearest.front();
	            if (source.box_id < 0 || target.box_id < 0 ||
	                source.box_id == target.box_id ||
	                source.closest_point.size() == 0 ||
	                target.closest_point.size() == 0 ||
	                source.closest_point.size() != target.closest_point.size()) {
	                ++gated;
	                continue;
	            }
	            const int source_component =
	                component_by_box.count(source.box_id) > 0 ? component_by_box[source.box_id] : -1;
	            const int target_component =
	                component_by_box.count(target.box_id) > 0 ? component_by_box[target.box_id] : -1;
	            if (!last_adaptive_partition_config_.hipac_transition_allow_same_component &&
	                source_component >= 0 &&
	                source_component == target_component) {
	                ++same_component_gated;
	                continue;
	            }
	            const double pair_distance =
	                (target.closest_point - source.closest_point).norm();
	            if (max_pair_distance > 0.0 &&
	                pair_distance > max_pair_distance + 1e-12) {
	                ++distance_gated;
	                continue;
	            }
	            double local_length = 0.0;
	            for (std::size_t index = begin + 1; index <= end; ++index) {
	                local_length += (task.waypoint_path[index] - task.waypoint_path[index - 1]).norm();
	            }
	            const int predicted_edges =
	                static_cast<int>(std::ceil(local_length / sample_step));
	            if (predicted_edges < min_predicted_edges) {
	                ++edge_gated;
	                continue;
	            }
	            TransitionCandidate candidate;
	            candidate.source_box_id = source.box_id;
	            candidate.target_box_id = target.box_id;
	            candidate.source_component = source_component;
	            candidate.target_component = target_component;
	            candidate.first_waypoint = static_cast<int>(begin);
	            candidate.last_waypoint = static_cast<int>(end);
	            candidate.source_point = source.closest_point;
	            candidate.target_point = target.closest_point;
	            candidate.pair_distance = pair_distance;
	            candidate.local_length = local_length;
	            candidate.predicted_bridge_edges = predicted_edges;
	            candidate.local_path.reserve(end - begin + 2);
	            candidate.local_path.push_back(candidate.source_point);
	            for (std::size_t index = begin + 1; index < end; ++index) {
	                candidate.local_path.push_back(task.waypoint_path[index]);
	            }
	            candidate.local_path.push_back(candidate.target_point);
	            candidate.score =
	                static_cast<double>(predicted_edges) -
	                0.25 * pair_distance -
	                0.05 * static_cast<double>(std::abs(target_component - source_component));
	            candidates.push_back(std::move(candidate));
	        }
	        batch_context.diagnostics().add_counter("query_bridge.hipac_transition_candidates",
	                                                static_cast<double>(candidates.size()));
	        batch_context.diagnostics().add_counter("query_bridge.hipac_transition_gated",
	                                                static_cast<double>(gated + same_component_gated +
	                                                                    distance_gated + edge_gated));
	        batch_context.diagnostics().add_counter("query_bridge.hipac_transition_gated_same_component",
	                                                static_cast<double>(same_component_gated));
	        batch_context.diagnostics().add_counter("query_bridge.hipac_transition_gated_distance",
	                                                static_cast<double>(distance_gated));
	        batch_context.diagnostics().add_counter("query_bridge.hipac_transition_gated_edges",
	                                                static_cast<double>(edge_gated));
	        if (candidates.empty()) {
	            return 0;
	        }
	        std::sort(candidates.begin(),
	                  candidates.end(),
	                  [](const TransitionCandidate& lhs, const TransitionCandidate& rhs) {
	            if (std::abs(lhs.score - rhs.score) > 1e-12) {
	                return lhs.score > rhs.score;
	            }
	            if (std::abs(lhs.local_length - rhs.local_length) > 1e-12) {
	                return lhs.local_length > rhs.local_length;
	            }
	            return lhs.first_waypoint < rhs.first_waypoint;
	        });
	        if (static_cast<int>(candidates.size()) > candidate_limit) {
	            candidates.resize(static_cast<std::size_t>(candidate_limit));
	        }

	        int total_added = 0;
	        int attempts = 0;
	        const int attempt_cap =
	            std::max(1, last_adaptive_partition_config_.hipac_transition_max_attempts_per_query);
	        for (const auto& candidate : candidates) {
	            if (attempts >= attempt_cap ||
	                task.hipac_transition_resolves_used >= attempt_cap) {
	                break;
	            }
	            ++attempts;
	            task.hipac_transition_resolves_used += 1;
	            batch_context.diagnostics().add_counter("query_bridge.hipac_transition_portal_attempts");
	            batch_context.diagnostics().set_value(task_key(task.index, "hipac_transition_predicted_edges"),
	                                                  static_cast<double>(candidate.predicted_bridge_edges));
	            batch_context.diagnostics().set_value(task_key(task.index, "hipac_transition_pair_distance"),
	                                                  candidate.pair_distance);
	            const int added = add_partition_portal_corridor_overlay(candidate.source_point,
	                                                                    candidate.target_point,
	                                                                    candidate.local_path,
	                                                                    "query_bridge.hipac_online_transition",
	                                                                    false,
	                                                                    false,
	                                                                    edge_query_index_for(task),
	                                                                    &last_build_);
	            if (added <= 0) {
	                batch_context.diagnostics().add_counter("query_bridge.hipac_transition_failures");
	                continue;
	            }
	            total_added += added;
	            batch_context.diagnostics().add_counter("query_bridge.hipac_transition_added",
	                                                    static_cast<double>(added));
	            batch_context.diagnostics().set_value(task_key(task.index, "hipac_transition_added"),
	                                                  static_cast<double>(added));
	            added_by_query[task.index] += added;
	            const QueryResult probe_after_transition = query(task.start, task.goal);
	            if (probe_after_transition.success &&
	                probe_after_transition.audit_passed &&
	                !probe_after_transition.path.empty()) {
	                task.waypoint_path = probe_after_transition.path;
	                task.hipac_candidate_path = probe_after_transition.path;
	                batch_context.diagnostics().add_counter(
	                    "query_bridge.hipac_transition_probe_path_adopted");
	                batch_context.diagnostics().set_value(
	                    task_key(task.index, "hipac_transition_probe_path_length"),
	                    probe_after_transition.path_length);
	            }
	            if (query_result_good(probe_after_transition, task.start, task.goal)) {
	                task.hipac_online_satisfied = true;
	                batch_context.diagnostics().add_counter("query_bridge.hipac_transition_satisfied");
	                batch_context.diagnostics().set_value(task_key(task.index,
	                                                               "hipac_transition_satisfied"),
	                                                      1.0);
	                break;
	            }
	            batch_context.diagnostics().add_counter("query_bridge.hipac_transition_not_sufficient");
	        }
	        const double transition_ms = elapsed_ms_since(transition_t0);
	        batch_context.diagnostics().record_timing("query_bridge.hipac_transition_ms_total",
	                                                  transition_ms);
	        batch_context.diagnostics().add_counter("query_bridge.hipac_transition_ms_total",
	                                                transition_ms);
	        batch_context.diagnostics().set_value(task_key(task.index, "hipac_transition_ms"),
	                                              transition_ms);
	        return total_added;
	    };
	    auto maybe_promote_query_repair = [&](const BridgeSearchTask& task,
	                                          int bridge_added) -> int {
	        if (!last_adaptive_partition_config_.hipac_online_connectivity ||
	            !last_adaptive_partition_config_.hipac_promote_query_repairs ||
	            !partition_native_mode() ||
	            bridge_added <= 0 ||
	            task.waypoint_path.size() < 2) {
	            return 0;
	        }
	        const auto promote_t0 = Clock::now();
	        batch_context.diagnostics().add_counter("query_bridge.hipac_promote_attempts");
	        const int promoted = add_partition_portal_corridor_overlay(task.start,
	                                                                   task.goal,
	                                                                   task.waypoint_path,
	                                                                   "query_bridge.hipac_promote",
	                                                                   true,
	                                                                   false,
	                                                                   edge_query_index_for(task),
	                                                                   &last_build_);
	        const double promote_ms = elapsed_ms_since(promote_t0);
	        batch_context.diagnostics().record_timing("query_bridge.hipac_promote_ms_total",
	                                                  promote_ms);
	        batch_context.diagnostics().add_counter("query_bridge.hipac_promote_ms_total",
	                                                promote_ms);
	        batch_context.diagnostics().set_value(task_key(task.index, "hipac_promote_ms"),
	                                              promote_ms);
	        if (promoted > 0) {
	            batch_context.diagnostics().add_counter("query_bridge.hipac_promote_added",
	                                                    static_cast<double>(promoted));
	            batch_context.diagnostics().set_value(task_key(task.index, "hipac_promote_added"),
	                                                  static_cast<double>(promoted));
	            added_by_query[task.index] += promoted;
	        } else {
	            batch_context.diagnostics().add_counter("query_bridge.hipac_promote_failures");
	        }
	        return promoted;
	    };
	    batch_context.diagnostics().set_value("query_bridge.batch_tasks_initial",
	                                          static_cast<double>(tasks.size()));
    const bool skip_deferred_short_edges =
        env_int_or_default("RBF_QUERY_BRIDGE_SKIP_DEFERRED_SHORT", 1) != 0;
    batch_context.diagnostics().set_value(
        "query_bridge.skip_deferred_short_edges",
        skip_deferred_short_edges ? 1.0 : 0.0);
    batch_context.diagnostics().set_value(
        "query_bridge.accept_segment_fraction",
        bridge_accept_segment_fraction);
    batch_context.diagnostics().set_value(
        "query_bridge.accept_path_ratio",
        bridge_accept_path_ratio);
    batch_context.diagnostics().set_value(
        "query_bridge.accept_path_additive",
        bridge_accept_path_additive);
    batch_context.diagnostics().set_value(
        "query_bridge.accept_max_path_length",
        bridge_accept_max_path_length);
    batch_context.diagnostics().set_value(
        "query_bridge.partition_path_first",
        partition_path_first ? 1.0 : 0.0);
    batch_context.diagnostics().set_value(
        "query_bridge.partition_path_first_allow_long",
        partition_path_first_allow_long ? 1.0 : 0.0);
    batch_context.diagnostics().set_value(
        "query_bridge.partition_path_first_max_segment_fraction",
        partition_path_first_max_segment_fraction);
    const int segment_only_retry_attempts =
        std::max(0, env_int_or_default("RBF_QUERY_BRIDGE_SEGMENT_ONLY_RETRY_ATTEMPTS", 1));
    batch_context.diagnostics().set_value(
        "query_bridge.segment_only_retry_attempts_default",
        static_cast<double>(segment_only_retry_attempts));
    const int no_path_retry_attempts =
        std::max(0, env_int_or_default("RBF_QUERY_BRIDGE_NO_PATH_RETRY_ATTEMPTS", 1));
    batch_context.diagnostics().set_value(
        "query_bridge.no_path_retry_attempts_default",
        static_cast<double>(no_path_retry_attempts));
    const bool no_path_retry_stop_on_first_success =
        env_int_or_default("RBF_QUERY_BRIDGE_NO_PATH_RETRY_STOP_ON_FIRST_SUCCESS", 0) != 0;
    batch_context.diagnostics().set_value(
        "query_bridge.no_path_retry_stop_on_first_success",
        no_path_retry_stop_on_first_success ? 1.0 : 0.0);
    const int forced_query_attempts =
        std::max(1, env_int_or_default("RBF_QUERY_BRIDGE_FORCED_ATTEMPTS", 1));
    const int query_bridge_attempt_offset =
        std::max(0, env_int_or_default("RBF_QUERY_BRIDGE_ATTEMPT_OFFSET", 0));
    const int query_bridge_rrt_fixed_iters =
        std::max(0, env_int_or_default("RBF_QUERY_BRIDGE_RRT_FIXED_ITERS", 0));
    const double query_bridge_rrt_fixed_timeout_ms =
        std::max(0.0, env_double_or_default("RBF_QUERY_BRIDGE_RRT_FIXED_TIMEOUT_MS", 0.0));
    batch_context.diagnostics().set_value(
        "query_bridge.rrt_fixed_iters",
        static_cast<double>(query_bridge_rrt_fixed_iters));
    batch_context.diagnostics().set_value(
        "query_bridge.rrt_fixed_timeout_ms",
        query_bridge_rrt_fixed_timeout_ms);
    const bool post_rrt_skip_forced =
        env_int_or_default("RBF_QUERY_BRIDGE_POST_RRT_SKIP_FORCED", 0) != 0;
    batch_context.diagnostics().set_value("query_bridge.post_rrt_skip_forced",
                                          post_rrt_skip_forced ? 1.0 : 0.0);
    auto query_bridge_forced = [&](const BridgeSearchTask& task) {
        return query_bridge_forced_index(task.index);
    };
    auto current_query_good = [&](const BridgeSearchTask& task, bool respect_forced) {
        if (env_index_list_contains("RBF_QUERY_BRIDGE_SEGMENT_ONLY_INDICES", task.index)) {
            return false;
        }
        if (respect_forced && query_bridge_forced(task)) {
            return false;
        }
        if (!skip_deferred_short_edges) {
            return false;
        }
        return query_result_good(query(task.start, task.goal), task.start, task.goal);
    };
    auto run_task_attempt = [&](const BridgeSearchTask& task, int attempt) {
        const int scheduled_attempt = attempt + query_bridge_attempt_offset;
        CollisionChecker checker = make_audit_checker(audit_robot_, scene_, config_.query);
        RRTConnectConfig config =
            task.short_local_profiles.empty()
                ? task.bridge_rrt
                : task.short_local_profiles[
                      static_cast<std::size_t>(scheduled_attempt) % task.short_local_profiles.size()];
        if (query_bridge_rrt_fixed_iters > 0) {
            config.max_iters = query_bridge_rrt_fixed_iters;
            config.timeout_ms = query_bridge_rrt_fixed_timeout_ms;
        } else {
            config.timeout_ms = std::max(1.0, config_.connector.per_pair_timeout_ms);
        }
        std::vector<Eigen::VectorXd> path = rrt_connect(
            task.start,
            task.goal,
            checker,
            audit_robot_,
            config,
            derived_planner_seed(config_.grower.rng_seed,
                                 kSeedBatchBridgeOffset,
                                 scheduled_attempt,
                                 task.query_index,
                                 task.short_local_bridge ? 0 : kSeedAttemptStride),
            batch_context.native_cancel_flag());
        if (path.empty()) {
            return std::vector<Eigen::VectorXd>{};
        }
        const PathAuditCheck audit =
            audit_waypoint_path(path, checker, config_.query.audit_resolution, config_.query.audit_segment_step);
        if (!audit.passed) {
            return std::vector<Eigen::VectorXd>{};
        }
        return path;
    };
    const bool direct_line_on_no_path =
        env_int_or_default("RBF_QUERY_BRIDGE_DIRECT_LINE_ON_NO_PATH", 0) != 0;
    batch_context.diagnostics().set_value(
        "query_bridge.direct_line_on_no_path",
        direct_line_on_no_path ? 1.0 : 0.0);
    auto direct_line_fallback_path = [&](const BridgeSearchTask& task) {
        if (!direct_line_on_no_path) {
            return std::vector<Eigen::VectorXd>{};
        }
        batch_context.diagnostics().add_counter(
            "query_bridge.direct_line_on_no_path_attempts");
        CollisionChecker checker = make_audit_checker(audit_robot_, scene_, config_.query);
        std::vector<Eigen::VectorXd> path{task.start, task.goal};
        const PathAuditCheck audit =
            audit_waypoint_path(path,
                                checker,
                                config_.query.audit_resolution,
                                config_.query.audit_segment_step);
        if (!audit.passed) {
            batch_context.diagnostics().add_counter(
                "query_bridge.direct_line_on_no_path_rejects");
            return std::vector<Eigen::VectorXd>{};
        }
        batch_context.diagnostics().add_counter(
            "query_bridge.direct_line_on_no_path_successes");
        return path;
    };
    const bool detour_on_no_path =
        env_int_or_default("RBF_QUERY_BRIDGE_DETOUR_ON_NO_PATH", 0) != 0;
    const bool detour_candidate =
        env_int_or_default("RBF_QUERY_BRIDGE_DETOUR_CANDIDATE", 0) != 0;
    const double detour_replace_factor =
        std::max(0.0, env_double_or_default("RBF_QUERY_BRIDGE_DETOUR_REPLACE_FACTOR", 1.0));
    batch_context.diagnostics().set_value(
        "query_bridge.detour_on_no_path",
        detour_on_no_path ? 1.0 : 0.0);
    batch_context.diagnostics().set_value(
        "query_bridge.detour_candidate",
        detour_candidate ? 1.0 : 0.0);
    auto deterministic_detour_fallback_path = [&](const BridgeSearchTask& task) {
        if (!detour_on_no_path ||
            task.start.size() != task.goal.size() ||
            task.start.size() <= 0) {
            return std::vector<Eigen::VectorXd>{};
        }
        const auto domain = oracle_->planning_intervals();
        if (static_cast<int>(domain.size()) != task.start.size()) {
            return std::vector<Eigen::VectorXd>{};
        }
        CollisionChecker checker = make_audit_checker(audit_robot_, scene_, config_.query);
        const Eigen::VectorXd delta = task.goal - task.start;
        const double direct_length = delta.norm();
        if (direct_length <= 1e-9) {
            return std::vector<Eigen::VectorXd>{};
        }
        std::vector<int> dims(static_cast<std::size_t>(task.start.size()));
        std::iota(dims.begin(), dims.end(), 0);
        std::sort(dims.begin(), dims.end(), [&](int lhs, int rhs) {
            const double lhs_width = std::max(1e-9, domain[static_cast<std::size_t>(lhs)].width());
            const double rhs_width = std::max(1e-9, domain[static_cast<std::size_t>(rhs)].width());
            const double lhs_along = std::abs(delta[lhs]) / lhs_width;
            const double rhs_along = std::abs(delta[rhs]) / rhs_width;
            if (std::abs(lhs_along - rhs_along) > 1e-12) {
                return lhs_along < rhs_along;
            }
            return lhs < rhs;
        });
        const int dim_limit = std::min<int>(
            std::max(0, env_int_or_default("RBF_QUERY_BRIDGE_DETOUR_DIMS", 4)),
            static_cast<int>(dims.size()));
        const int rounds = std::max(
            1,
            env_int_or_default("RBF_QUERY_BRIDGE_DETOUR_ROUNDS", 2));
        const int max_candidates = std::max(
            1,
            env_int_or_default("RBF_QUERY_BRIDGE_DETOUR_MAX_CANDIDATES", 32));
        const double base_offset = std::max(
            1e-4,
            env_double_or_default("RBF_QUERY_BRIDGE_DETOUR_OFFSET", 0.35));
        const double two_bend_alpha = std::min(
            0.45,
            std::max(0.15,
                     env_double_or_default("RBF_QUERY_BRIDGE_DETOUR_TWO_BEND_ALPHA", 0.35)));
        const Eigen::VectorXd mid = 0.5 * (task.start + task.goal);
        double best_length = std::numeric_limits<double>::infinity();
        std::vector<Eigen::VectorXd> best_path;
        int candidates = 0;
        auto clamp_to_domain = [&](Eigen::VectorXd point) {
            for (int dim = 0; dim < point.size(); ++dim) {
                point[dim] = std::min(domain[static_cast<std::size_t>(dim)].hi,
                                      std::max(domain[static_cast<std::size_t>(dim)].lo,
                                               point[dim]));
            }
            return point;
        };
        auto try_path = [&](std::vector<Eigen::VectorXd> path) {
            if (candidates >= max_candidates) {
                return;
            }
            ++candidates;
            batch_context.diagnostics().add_counter(
                "query_bridge.detour_on_no_path_candidates");
            double length = path_length(path);
            if (!std::isfinite(length) || length + 1e-12 >= best_length) {
                return;
            }
            const PathAuditCheck audit =
                audit_waypoint_path(path,
                                    checker,
                                    config_.query.audit_resolution,
                                    config_.query.audit_segment_step);
            if (!audit.passed) {
                batch_context.diagnostics().add_counter(
                    "query_bridge.detour_on_no_path_rejects");
                return;
            }
            best_length = length;
            best_path = std::move(path);
        };
        for (int item = 0; item < dim_limit && candidates < max_candidates; ++item) {
            const int dim = dims[static_cast<std::size_t>(item)];
            const double width = domain[static_cast<std::size_t>(dim)].width();
            for (int round = 1; round <= rounds && candidates < max_candidates; ++round) {
                const double magnitude = std::min(0.45 * std::max(0.0, width),
                                                  base_offset * static_cast<double>(round));
                if (magnitude <= 1e-9) {
                    continue;
                }
                for (double sign : {1.0, -1.0}) {
                    Eigen::VectorXd single = mid;
                    single[dim] += sign * magnitude;
                    single = clamp_to_domain(std::move(single));
                    if ((single - mid).norm() > 1e-9) {
                        try_path({task.start, single, task.goal});
                    }
                    if (candidates >= max_candidates) {
                        break;
                    }
                    Eigen::VectorXd first = task.start + two_bend_alpha * delta;
                    Eigen::VectorXd second = task.start + (1.0 - two_bend_alpha) * delta;
                    first[dim] += sign * magnitude;
                    second[dim] += sign * magnitude;
                    first = clamp_to_domain(std::move(first));
                    second = clamp_to_domain(std::move(second));
                    if ((first - (task.start + two_bend_alpha * delta)).norm() > 1e-9 ||
                        (second - (task.start + (1.0 - two_bend_alpha) * delta)).norm() > 1e-9) {
                        try_path({task.start, first, second, task.goal});
                    }
                    if (candidates >= max_candidates) {
                        break;
                    }
                }
            }
        }
        batch_context.diagnostics().add_counter(
            "query_bridge.detour_on_no_path_attempts");
        if (!best_path.empty()) {
            batch_context.diagnostics().add_counter(
                "query_bridge.detour_on_no_path_successes");
        }
        return best_path;
    };
    auto maybe_apply_detour_path = [&](const BridgeSearchTask& task,
                                       double& best_length,
                                       std::vector<Eigen::VectorXd>& waypoint_path) {
        if (!waypoint_path.empty() && !detour_candidate) {
            return false;
        }
        auto detour_path = deterministic_detour_fallback_path(task);
        if (detour_path.empty()) {
            return false;
        }
        const double detour_length = path_length(detour_path);
        if (!waypoint_path.empty() &&
            detour_length > best_length * detour_replace_factor + 1e-12) {
            batch_context.diagnostics().add_counter(
                "query_bridge.detour_candidate_not_shorter");
            return false;
        }
        best_length = detour_length;
        waypoint_path = std::move(detour_path);
        batch_context.diagnostics().add_counter(
            "query_bridge.detour_candidate_selected");
        return true;
    };
    const bool waypoint_quality_retry =
        env_int_or_default("RBF_QUERY_BRIDGE_WAYPOINT_QUALITY_RETRY", 0) != 0;
    const int waypoint_quality_retry_attempts = std::max(
        0,
        env_int_or_default("RBF_QUERY_BRIDGE_WAYPOINT_QUALITY_RETRY_ATTEMPTS", 4));
    const double waypoint_quality_max_ratio = std::max(
        1.0,
        env_double_or_default("RBF_QUERY_BRIDGE_WAYPOINT_QUALITY_MAX_RATIO", 2.0));
    const double waypoint_quality_max_additive = std::max(
        0.0,
        env_double_or_default("RBF_QUERY_BRIDGE_WAYPOINT_QUALITY_MAX_ADDITIVE", 0.75));
    batch_context.diagnostics().set_value(
        "query_bridge.waypoint_quality_retry",
        waypoint_quality_retry ? 1.0 : 0.0);
	    auto improve_waypoint_if_needed = [&](const BridgeSearchTask& task,
	                                          int attempts_already_used,
	                                          double& best_length,
	                                          std::vector<Eigen::VectorXd>& waypoint_path) {
        if (!waypoint_quality_retry ||
            waypoint_quality_retry_attempts <= 0 ||
            waypoint_path.empty()) {
            return;
        }
        const double direct = (task.goal - task.start).norm();
        const double limit = std::max(direct * waypoint_quality_max_ratio,
                                      direct + waypoint_quality_max_additive);
        if (!(best_length > limit)) {
            return;
        }
        batch_context.diagnostics().add_counter(
            "query_bridge.waypoint_quality_retry_tasks");
        const auto retry_t0 = Clock::now();
        int retry_successes = 0;
        for (int retry = 0; retry < waypoint_quality_retry_attempts; ++retry) {
            auto retry_path = run_task_attempt(task, attempts_already_used + retry);
            if (retry_path.empty()) {
                continue;
            }
            retry_successes += 1;
            const double length = path_length(retry_path);
            if (length < best_length) {
                best_length = length;
                waypoint_path = std::move(retry_path);
            }
            if (best_length <= limit) {
                break;
            }
        }
        batch_context.diagnostics().add_counter(
            "query_bridge.waypoint_quality_retry_attempts",
            static_cast<double>(waypoint_quality_retry_attempts));
        batch_context.diagnostics().add_counter(
            "query_bridge.waypoint_quality_retry_successes",
            static_cast<double>(retry_successes));
        if (best_length <= limit) {
            batch_context.diagnostics().add_counter(
                "query_bridge.waypoint_quality_retry_fixed");
        }
	        batch_context.diagnostics().record_timing(
	            "query_bridge.waypoint_quality_retry_ms_total",
	            elapsed_ms_since(retry_t0));
	    };
	    if (last_adaptive_partition_config_.hipac_online_connectivity &&
	        last_adaptive_partition_config_.hipac_online_before_query_bridge) {
	        for (auto& task : tasks) {
	            try_hipac_online_bridge(task);
	            if (!task.hipac_online_satisfied) {
	                try_hipac_transition_portal(task);
	            }
	            if (!task.hipac_online_satisfied) {
	                try_hipac_prebridge_portal(task);
	            }
	        }
	    }

	    const bool parallel_task_rrt =
	        env_int_or_default("RBF_QUERY_BRIDGE_PARALLEL_TASK_RRT", 1) != 0;
    batch_context.diagnostics().set_value("query_bridge.attempt_offset",
                                          static_cast<double>(query_bridge_attempt_offset));
    const bool has_segment_only_task =
        std::any_of(tasks.begin(), tasks.end(), [](const BridgeSearchTask& task) {
            return env_index_list_contains("RBF_QUERY_BRIDGE_SEGMENT_ONLY_INDICES",
                                           task.index);
        });
    if (parallel_task_rrt && !has_segment_only_task && no_path_retry_attempts == 0) {
        struct PreparedTask {
            bool skipped = false;
            bool forced = false;
            int attempts = 1;
            double task_start_ms = 0.0;
        };
        struct PreparedJob {
            std::size_t task_offset = 0;
            int attempt = 0;
        };
        std::vector<PreparedTask> prepared(tasks.size());
        std::vector<PreparedJob> jobs;
	        for (std::size_t task_offset = 0; task_offset < tasks.size(); ++task_offset) {
	            auto& task = tasks[task_offset];
	            prepared[task_offset].task_start_ms = elapsed_ms_since(batch_t0);
	            const auto probe_t0 = Clock::now();
	            if (task.hipac_online_satisfied || current_query_good(task, true)) {
	                prepared[task_offset].skipped = true;
	                batch_context.diagnostics().add_counter("query_bridge.batch_tasks_skipped");
	                batch_context.diagnostics().record_timing("query_bridge.batch_probe_ms_total",
	                                                          elapsed_ms_since(probe_t0));
	                batch_context.diagnostics().set_value(task_key(task.index, "skipped"),
	                                                      1.0);
	                if (task.hipac_online_satisfied) {
	                    batch_context.diagnostics().set_value(task_key(task.index, "skipped_by_hipac_online"),
	                                                          1.0);
	                }
	                continue;
	            }
            batch_context.diagnostics().record_timing("query_bridge.batch_probe_ms_total",
                                                      elapsed_ms_since(probe_t0));
            batch_context.diagnostics().add_counter("query_bridge.batch_tasks_attempted");
            prepared[task_offset].forced = query_bridge_forced(task);
            prepared[task_offset].attempts = prepared[task_offset].forced
                ? std::max(std::max(1, task.attempts), forced_query_attempts)
                : std::max(1, task.attempts);
            if (task.waypoint_path_from_partition_query && !task.waypoint_path.empty()) {
                prepared[task_offset].attempts = 0;
                batch_context.diagnostics().add_counter(
                    "query_bridge.partition_path_first_tasks");
                batch_context.diagnostics().set_value(
                    task_key(task.index, "partition_path_first"),
                    1.0);
            }
            if (prepared[task_offset].forced) {
                batch_context.diagnostics().set_value(task_key(task.index, "forced"),
                                                      1.0);
            }
            batch_context.diagnostics().set_value(task_key(task.index, "attempts"),
                                                  static_cast<double>(prepared[task_offset].attempts));
            for (int attempt = 0; attempt < prepared[task_offset].attempts; ++attempt) {
                jobs.push_back({task_offset, attempt});
            }
        }

        std::vector<std::vector<std::vector<Eigen::VectorXd>>> attempt_paths(tasks.size());
        for (std::size_t task_offset = 0; task_offset < tasks.size(); ++task_offset) {
            attempt_paths[task_offset].resize(
                static_cast<std::size_t>(std::max(0, prepared[task_offset].attempts)));
        }
        const auto rrt_t0 = Clock::now();
        if (batch_context.executor().n_threads() > 1 && jobs.size() > 1) {
            batch_context.executor().parallel_for(0,
                                                  static_cast<int>(jobs.size()),
                                                  [&](int job_index) {
                const PreparedJob& job = jobs[static_cast<std::size_t>(job_index)];
                attempt_paths[job.task_offset][static_cast<std::size_t>(job.attempt)] =
                    run_task_attempt(tasks[job.task_offset], job.attempt);
            });
        } else {
            for (const PreparedJob& job : jobs) {
                attempt_paths[job.task_offset][static_cast<std::size_t>(job.attempt)] =
                    run_task_attempt(tasks[job.task_offset], job.attempt);
            }
        }
        const double rrt_ms = elapsed_ms_since(rrt_t0);
        batch_context.diagnostics().record_timing("query_bridge.batch_rrt_ms_total",
                                                  rrt_ms);
        batch_context.diagnostics().set_value("query_bridge.parallel_task_rrt",
                                              1.0);
        batch_context.diagnostics().set_value("query_bridge.parallel_task_rrt_jobs",
                                              static_cast<double>(jobs.size()));

        for (std::size_t task_offset = 0; task_offset < tasks.size(); ++task_offset) {
            auto& task = tasks[task_offset];
            if (prepared[task_offset].skipped) {
                batch_context.diagnostics().set_value(
                    task_key(task.index, "total_ms"),
                    elapsed_ms_since(batch_t0) - prepared[task_offset].task_start_ms);
                continue;
            }
            batch_context.diagnostics().set_value(task_key(task.index, "rrt_ms"),
                                                  rrt_ms);
            double best_length = std::numeric_limits<double>::infinity();
            if (task.waypoint_path_from_partition_query && !task.waypoint_path.empty()) {
                best_length = path_length(task.waypoint_path);
                batch_context.diagnostics().add_counter(
                    "query_bridge.partition_path_first_rrt_skipped");
                batch_context.diagnostics().set_value(
                    task_key(task.index, "waypoint_from_partition_path"),
                    1.0);
            }
            for (auto& path : attempt_paths[task_offset]) {
                if (path.empty()) {
                    continue;
                }
                const double length = path_length(path);
                if (length < best_length) {
                    best_length = length;
                    task.waypoint_path = std::move(path);
                }
            }
            if (task.waypoint_path.empty()) {
                auto direct_path = direct_line_fallback_path(task);
                if (!direct_path.empty()) {
                    best_length = path_length(direct_path);
                    task.waypoint_path = std::move(direct_path);
                    batch_context.diagnostics().set_value(
                        task_key(task.index, "direct_line_on_no_path"),
                        1.0);
                }
            }
            if (maybe_apply_detour_path(task, best_length, task.waypoint_path)) {
                batch_context.diagnostics().set_value(
                    task_key(task.index, "detour_on_no_path"),
                    1.0);
            }
            improve_waypoint_if_needed(task,
                                       prepared[task_offset].attempts,
                                       best_length,
                                       task.waypoint_path);
            if (task.waypoint_path.empty()) {
                batch_context.diagnostics().add_counter("query_bridge.batch_tasks_no_path");
                batch_context.diagnostics().set_value(task_key(task.index, "no_path"),
                                                      1.0);
                batch_context.diagnostics().set_value(
                    task_key(task.index, "total_ms"),
                    elapsed_ms_since(batch_t0) - prepared[task_offset].task_start_ms);
                continue;
            }
            batch_context.diagnostics().set_value(task_key(task.index, "waypoint_length"),
                                                  best_length);
            const auto second_probe_t0 = Clock::now();
            if (current_query_good(task, !post_rrt_skip_forced)) {
                batch_context.diagnostics().add_counter("query_bridge.batch_tasks_skipped_after_rrt");
                if (prepared[task_offset].forced) {
                    batch_context.diagnostics().add_counter(
                        "query_bridge.batch_forced_tasks_skipped_after_rrt");
                }
                batch_context.diagnostics().record_timing("query_bridge.batch_probe_ms_total",
                                                          elapsed_ms_since(second_probe_t0));
                batch_context.diagnostics().set_value(task_key(task.index, "skipped_after_rrt"),
                                                      1.0);
                batch_context.diagnostics().set_value(
                    task_key(task.index, "total_ms"),
                    elapsed_ms_since(batch_t0) - prepared[task_offset].task_start_ms);
                continue;
            }
            batch_context.diagnostics().record_timing("query_bridge.batch_probe_ms_total",
                                                      elapsed_ms_since(second_probe_t0));
            const int hipac_resolve_cap =
                std::max(0, last_adaptive_partition_config_.hipac_online_max_resolves_per_query);
            const bool can_run_hipac_online =
                task.hipac_online_resolves_used < hipac_resolve_cap ||
                (last_adaptive_partition_config_.hipac_online_transition_portal &&
                 task.hipac_transition_resolves_used <
                     std::max(0, last_adaptive_partition_config_.hipac_transition_max_attempts_per_query)) ||
                (last_adaptive_partition_config_.hipac_online_prebridge_portal &&
                 task.hipac_prebridge_resolves_used < hipac_resolve_cap);
            if (last_adaptive_partition_config_.hipac_online_connectivity &&
                !task.waypoint_path.empty() &&
                can_run_hipac_online) {
                task.hipac_candidate_path = task.waypoint_path;
                try_hipac_online_bridge(task);
                if (!task.hipac_online_satisfied) {
                    try_hipac_transition_portal(task);
                }
                if (!task.hipac_online_satisfied) {
                    try_hipac_prebridge_portal(task);
                }
                if (task.hipac_online_satisfied) {
                    batch_context.diagnostics().add_counter(
                        "query_bridge.batch_tasks_skipped_by_hipac_after_rrt");
                    batch_context.diagnostics().set_value(
                        task_key(task.index, "skipped_by_hipac_after_rrt"),
                        1.0);
                    batch_context.diagnostics().set_value(
                        task_key(task.index, "total_ms"),
                        elapsed_ms_since(batch_t0) - prepared[task_offset].task_start_ms);
                    continue;
                }
            }
            const auto pave_t0 = Clock::now();
	            const int bridge_added =
	                bridge_query_with_waypoint_path(task.start,
	                                                task.goal,
	                                                task.waypoint_path,
	                                                task.short_local_bridge,
	                                                task.bridge_rrt,
	                                                task.query_index);
	            added_by_query[task.index] += bridge_added;
	            maybe_promote_query_repair(task, bridge_added);
	            accumulate_task_direct_corridor_totals(task.index);
	            const double pave_ms = elapsed_ms_since(pave_t0);
            batch_context.diagnostics().record_timing("query_bridge.batch_pave_ms_total",
                                                      pave_ms);
            batch_context.diagnostics().set_value(task_key(task.index, "pave_ms"),
                                                  pave_ms);
            batch_context.diagnostics().set_value(task_key(task.index, "added"),
                                                  static_cast<double>(added_by_query[task.index]));
            batch_context.diagnostics().set_value(
                task_key(task.index, "total_ms"),
                elapsed_ms_since(batch_t0) - prepared[task_offset].task_start_ms);
        }

        batch_context.diagnostics().set_value("query_bridge.batch_total_ms",
                                              elapsed_ms_since(batch_t0));
        return finish_batch_bridge();
    }

	    for (auto& task : tasks) {
	        const auto task_t0 = Clock::now();
	        const auto probe_t0 = Clock::now();
	        if (task.hipac_online_satisfied || current_query_good(task, true)) {
	            batch_context.diagnostics().add_counter("query_bridge.batch_tasks_skipped");
	            batch_context.diagnostics().record_timing("query_bridge.batch_probe_ms_total",
	                                                      elapsed_ms_since(probe_t0));
	            batch_context.diagnostics().set_value(task_key(task.index, "skipped"),
	                                                  1.0);
	            if (task.hipac_online_satisfied) {
	                batch_context.diagnostics().set_value(task_key(task.index, "skipped_by_hipac_online"),
	                                                      1.0);
	            }
	            batch_context.diagnostics().set_value(task_key(task.index, "total_ms"),
	                                                  elapsed_ms_since(task_t0));
	            continue;
        }
        batch_context.diagnostics().record_timing("query_bridge.batch_probe_ms_total",
                                                  elapsed_ms_since(probe_t0));
        batch_context.diagnostics().add_counter("query_bridge.batch_tasks_attempted");
        const bool forced_task = query_bridge_forced(task);
        const int attempts = forced_task
            ? std::max(std::max(1, task.attempts), forced_query_attempts)
            : std::max(1, task.attempts);
        const int effective_attempts =
            task.waypoint_path_from_partition_query && !task.waypoint_path.empty()
                ? 0
                : attempts;
        if (forced_task) {
            batch_context.diagnostics().set_value(task_key(task.index, "forced"),
                                                  1.0);
        }
        batch_context.diagnostics().set_value(task_key(task.index, "attempts"),
                                              static_cast<double>(effective_attempts));
        if (task.waypoint_path_from_partition_query && !task.waypoint_path.empty()) {
            batch_context.diagnostics().add_counter(
                "query_bridge.partition_path_first_tasks");
            batch_context.diagnostics().add_counter(
                "query_bridge.partition_path_first_rrt_skipped");
            batch_context.diagnostics().set_value(
                task_key(task.index, "partition_path_first"),
                1.0);
            batch_context.diagnostics().set_value(
                task_key(task.index, "waypoint_from_partition_path"),
                1.0);
        }
        std::vector<std::vector<Eigen::VectorXd>> attempt_paths(static_cast<std::size_t>(effective_attempts));
        const auto rrt_t0 = Clock::now();
        if (batch_context.executor().n_threads() > 1 && effective_attempts > 1) {
            batch_context.executor().parallel_for(0, effective_attempts, [&](int attempt) {
                attempt_paths[static_cast<std::size_t>(attempt)] = run_task_attempt(task, attempt);
            });
        } else {
            for (int attempt = 0; attempt < effective_attempts; ++attempt) {
                attempt_paths[static_cast<std::size_t>(attempt)] = run_task_attempt(task, attempt);
            }
        }
        const double rrt_ms = elapsed_ms_since(rrt_t0);
        batch_context.diagnostics().record_timing("query_bridge.batch_rrt_ms_total",
                                                  rrt_ms);
        batch_context.diagnostics().set_value(task_key(task.index, "rrt_ms"),
                                              rrt_ms);
        double best_length = std::numeric_limits<double>::infinity();
        if (task.waypoint_path_from_partition_query && !task.waypoint_path.empty()) {
            best_length = path_length(task.waypoint_path);
        }
        for (auto& path : attempt_paths) {
            if (path.empty()) {
                continue;
            }
            const double length = path_length(path);
            if (length < best_length) {
                best_length = length;
                task.waypoint_path = std::move(path);
            }
        }
        if (task.waypoint_path.empty()) {
            auto direct_path = direct_line_fallback_path(task);
            if (!direct_path.empty()) {
                best_length = path_length(direct_path);
                task.waypoint_path = std::move(direct_path);
                batch_context.diagnostics().set_value(
                    task_key(task.index, "direct_line_on_no_path"),
                    1.0);
            }
        }
        if (maybe_apply_detour_path(task, best_length, task.waypoint_path)) {
            batch_context.diagnostics().set_value(
                task_key(task.index, "detour_on_no_path"),
                1.0);
        }
        improve_waypoint_if_needed(task,
                                   attempts,
                                   best_length,
                                   task.waypoint_path);
        const bool segment_only_task =
            env_index_list_contains("RBF_QUERY_BRIDGE_SEGMENT_ONLY_INDICES", task.index);
        if (task.waypoint_path.empty() && segment_only_task &&
            segment_only_retry_attempts > 0) {
            const auto retry_t0 = Clock::now();
            int retry_successes = 0;
            for (int retry = 0; retry < segment_only_retry_attempts; ++retry) {
                auto retry_path = run_task_attempt(task, attempts + retry);
                if (retry_path.empty()) {
                    continue;
                }
                retry_successes += 1;
                const double length = path_length(retry_path);
                if (length < best_length) {
                    best_length = length;
                    task.waypoint_path = std::move(retry_path);
                }
                if (no_path_retry_stop_on_first_success) {
                    break;
                }
            }
            const double retry_ms = elapsed_ms_since(retry_t0);
            batch_context.diagnostics().record_timing(
                "query_bridge.batch_segment_only_retry_ms_total",
                retry_ms);
            batch_context.diagnostics().add_counter(
                "query_bridge.batch_segment_only_retry_attempts",
                static_cast<double>(segment_only_retry_attempts));
            batch_context.diagnostics().add_counter(
                "query_bridge.batch_segment_only_retry_successes",
                static_cast<double>(retry_successes));
            batch_context.diagnostics().set_value(
                task_key(task.index, "segment_only_retry_attempts"),
                static_cast<double>(segment_only_retry_attempts));
            batch_context.diagnostics().set_value(
                task_key(task.index, "segment_only_retry_ms"),
                retry_ms);
            batch_context.diagnostics().set_value(
                task_key(task.index, "segment_only_retry_successes"),
                static_cast<double>(retry_successes));
        }
        if (task.waypoint_path.empty() && !segment_only_task &&
            no_path_retry_attempts > 0) {
            const auto retry_t0 = Clock::now();
            int retry_successes = 0;
            for (int retry = 0; retry < no_path_retry_attempts; ++retry) {
                auto retry_path = run_task_attempt(task, attempts + retry);
                if (retry_path.empty()) {
                    continue;
                }
                retry_successes += 1;
                const double length = path_length(retry_path);
                if (length < best_length) {
                    best_length = length;
                    task.waypoint_path = std::move(retry_path);
                }
                if (no_path_retry_stop_on_first_success) {
                    break;
                }
            }
            const double retry_ms = elapsed_ms_since(retry_t0);
            batch_context.diagnostics().record_timing(
                "query_bridge.batch_no_path_retry_ms_total",
                retry_ms);
            batch_context.diagnostics().add_counter(
                "query_bridge.batch_no_path_retry_attempts",
                static_cast<double>(no_path_retry_attempts));
            batch_context.diagnostics().add_counter(
                "query_bridge.batch_no_path_retry_successes",
                static_cast<double>(retry_successes));
            batch_context.diagnostics().set_value(
                task_key(task.index, "no_path_retry_attempts"),
                static_cast<double>(no_path_retry_attempts));
            batch_context.diagnostics().set_value(
                task_key(task.index, "no_path_retry_ms"),
                retry_ms);
            batch_context.diagnostics().set_value(
                task_key(task.index, "no_path_retry_successes"),
                static_cast<double>(retry_successes));
        }
        if (task.waypoint_path.empty()) {
            batch_context.diagnostics().add_counter("query_bridge.batch_tasks_no_path");
            batch_context.diagnostics().set_value(task_key(task.index, "no_path"),
                                                  1.0);
            batch_context.diagnostics().set_value(task_key(task.index, "total_ms"),
                                                  elapsed_ms_since(task_t0));
            continue;
        }
        batch_context.diagnostics().set_value(task_key(task.index, "waypoint_length"),
                                              best_length);
        const auto second_probe_t0 = Clock::now();
            if (current_query_good(task, !post_rrt_skip_forced)) {
            batch_context.diagnostics().add_counter("query_bridge.batch_tasks_skipped_after_rrt");
            if (forced_task) {
                batch_context.diagnostics().add_counter(
                    "query_bridge.batch_forced_tasks_skipped_after_rrt");
            }
            batch_context.diagnostics().record_timing("query_bridge.batch_probe_ms_total",
                                                      elapsed_ms_since(second_probe_t0));
            batch_context.diagnostics().set_value(task_key(task.index, "skipped_after_rrt"),
                                                  1.0);
            batch_context.diagnostics().set_value(task_key(task.index, "total_ms"),
                                                  elapsed_ms_since(task_t0));
            continue;
        }
        batch_context.diagnostics().record_timing("query_bridge.batch_probe_ms_total",
                                                  elapsed_ms_since(second_probe_t0));
        const int hipac_resolve_cap =
            std::max(0, last_adaptive_partition_config_.hipac_online_max_resolves_per_query);
        const bool can_run_hipac_online =
            task.hipac_online_resolves_used < hipac_resolve_cap ||
            (last_adaptive_partition_config_.hipac_online_transition_portal &&
             task.hipac_transition_resolves_used <
                 std::max(0, last_adaptive_partition_config_.hipac_transition_max_attempts_per_query)) ||
            (last_adaptive_partition_config_.hipac_online_prebridge_portal &&
             task.hipac_prebridge_resolves_used < hipac_resolve_cap);
        if (last_adaptive_partition_config_.hipac_online_connectivity &&
            !task.waypoint_path.empty() &&
            can_run_hipac_online) {
            task.hipac_candidate_path = task.waypoint_path;
            try_hipac_online_bridge(task);
            if (!task.hipac_online_satisfied) {
                try_hipac_transition_portal(task);
            }
            if (!task.hipac_online_satisfied) {
                try_hipac_prebridge_portal(task);
            }
            if (task.hipac_online_satisfied) {
                batch_context.diagnostics().add_counter(
                    "query_bridge.batch_tasks_skipped_by_hipac_after_rrt");
                batch_context.diagnostics().set_value(
                    task_key(task.index, "skipped_by_hipac_after_rrt"),
                    1.0);
                batch_context.diagnostics().set_value(task_key(task.index, "total_ms"),
                                                      elapsed_ms_since(task_t0));
                continue;
            }
        }
        if (segment_only_task) {
            const int source_box_id = locate_box_partition_first(task.start,
                                                                 config_.query.nearest_if_outside);
            const int target_box_id = locate_box_partition_first(task.goal,
                                                                 config_.query.nearest_if_outside);
            int edge_id = -1;
            if (source_box_id >= 0 && target_box_id >= 0) {
                edge_id = add_segment_edge_partition_first(                                           source_box_id,
                                           target_box_id,
                                           task.waypoint_path,
                                           SegmentEdgeType::QueryBridge,
                                           task.bridge_rrt.segment_resolution,
                                           SegmentEdgeValidation::CollisionChecked,
                                           true,
                                           edge_query_index_for(task));
	            }
	            if (edge_id >= 0) {
	                added_by_query[task.index] += 1;
	                invalidate_query_cache();
	                batch_context.diagnostics().add_counter(
                    "query_bridge.batch_tasks_segment_only");
                batch_context.diagnostics().set_value(task_key(task.index, "segment_only"),
                                                      1.0);
                batch_context.diagnostics().set_value(task_key(task.index, "added"),
                                                      1.0);
            } else {
                batch_context.diagnostics().add_counter(
                    "query_bridge.batch_tasks_segment_only_failures");
                batch_context.diagnostics().set_value(task_key(task.index, "segment_only_failure"),
                                                      1.0);
            }
            batch_context.diagnostics().set_value(task_key(task.index, "total_ms"),
                                                  elapsed_ms_since(task_t0));
            continue;
        }
        const auto pave_t0 = Clock::now();
	        const int bridge_added =
	            bridge_query_with_waypoint_path(task.start,
	                                            task.goal,
	                                            task.waypoint_path,
	                                            task.short_local_bridge,
	                                            task.bridge_rrt,
	                                            task.query_index);
	        added_by_query[task.index] += bridge_added;
	        maybe_promote_query_repair(task, bridge_added);
	        accumulate_task_direct_corridor_totals(task.index);
        const double pave_ms = elapsed_ms_since(pave_t0);
        batch_context.diagnostics().record_timing("query_bridge.batch_pave_ms_total",
                                                  pave_ms);
        batch_context.diagnostics().set_value(task_key(task.index, "pave_ms"),
                                              pave_ms);
        batch_context.diagnostics().set_value(task_key(task.index, "added"),
                                              static_cast<double>(added_by_query[task.index]));
        batch_context.diagnostics().set_value(task_key(task.index, "total_ms"),
                                              elapsed_ms_since(task_t0));
    }

    batch_context.diagnostics().set_value("query_bridge.batch_total_ms",
                                          elapsed_ms_since(batch_t0));
    return finish_batch_bridge();
}

DebugChainPaveResult RBFPlanningForest::debug_chain_pave(const Eigen::Ref<const Eigen::VectorXd>& start,
                                                         const Eigen::Ref<const Eigen::VectorXd>& goal,
                                                         const ChainPaveConfig& pave) {
    DebugChainPaveResult out;
    if (boxes_.empty() || !oracle_) {
        return out;
    }
    const int start_box_id = locate_box_partition_first(start, config_.query.nearest_if_outside);
    if (start_box_id < 0) {
        return out;
    }
    const int goal_box_id = locate_box_partition_first(goal, config_.query.nearest_if_outside);
    if (goal_box_id < 0 || goal_box_id == start_box_id) {
        return out;
    }
    out.start_box_id = start_box_id;
    out.goal_box_id = goal_box_id;
    for (const auto& box : boxes_) {
        if (box.id == start_box_id) {
            out.start_box = box.joint_intervals;
        }
        if (box.id == goal_box_id) {
            out.goal_box = box.joint_intervals;
        }
    }
    StageContext context = StageContext::from_runtime(config_.runtime);
    CollisionChecker checker = make_audit_checker(audit_robot_, scene_, config_.query);
    RRTConnectConfig bridge_rrt = with_query_root_hull_domain(config_.connector.rrt, *oracle_, start, goal);
    bridge_rrt.segment_resolution = std::max(bridge_rrt.segment_resolution, config_.query.audit_resolution);
    auto waypoint_path = rrt_connect(
        start,
        goal,
        checker,
        audit_robot_,
        context,
        bridge_rrt,
        derived_planner_seed(config_.grower.rng_seed, kSeedDebugBridgeOffset));
    if (waypoint_path.empty()) {
        return out;
    }
    out.bridge_found = true;
    out.waypoints = waypoint_path;
    out.audit_passed = audit_waypoint_path(waypoint_path,
                                           checker,
                                           config_.query.audit_resolution,
                                           config_.query.audit_segment_step)
                           .passed;
    const std::size_t boxes_before = boxes_.size();
    if (partition_native_mode()) {
        out.added = add_partition_box_corridor_overlay(start,
                                                       goal,
                                                       waypoint_path,
                                                       "debug_chain_pave",
                                                       false,
                                                       false,
                                                       -1,
                                                       &last_build_);
        out.boundary_ffb_calls = static_cast<int>(
            last_build_.diagnostics["debug_chain_pave.partition_box_corridor_overlay_attempts"]);
        out.boundary_commits = out.added;
        for (std::size_t i = boxes_before; i < boxes_.size(); ++i) {
            out.committed_boxes.push_back(boxes_[i].joint_intervals);
        }
        out.all_boxes.reserve(boxes_.size());
        for (const auto& box : boxes_) {
            out.all_boxes.push_back(box.joint_intervals);
        }
        if (out.added > 0) {
            invalidate_query_cache();
        }
        invalidate_query_cache();
        return out;
    }
    int next_id = next_box_id();
    ChainPaveConfig debug_pave = pave;
    debug_pave.debug_boundary_failures = &out.boundary_failures;
    out.added = chain_pave_along_path(
        waypoint_path,
        start_box_id,
        boxes_,
        *oracle_,
        adjacency_,
        next_id,
        context,
        debug_pave);
    if (boxes_.size() > boxes_before) {
        append_adaptive_partition_boxes(boxes_before, &last_build_, "debug_chain_pave");
    }
    out.fast_gap_fill_ffb_calls = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_fast_ffb_calls", 0.0));
    out.fast_gap_fill_ms =
        context.diagnostics().value("connector.chain_pave_fast_ms", 0.0);
    out.boundary_ffb_calls = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_boundary_ffb_calls", 0.0));
    out.boundary_commits = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_boundary_commits", 0.0));
    out.boundary_reject_not_free = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_boundary_reject_not_free", 0.0));
    out.boundary_reject_non_adjacent = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_boundary_reject_non_adjacent", 0.0));
    out.boundary_fail_seed_collision = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_boundary_fail_seed_collision", 0.0));
    out.boundary_fail_depth_cap = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_boundary_fail_depth_cap", 0.0));
    out.boundary_fail_unknown_depth_cap = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_boundary_fail_unknown_depth_cap", 0.0));
    out.boundary_fail_reserved_depth_cap = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_boundary_fail_reserved_depth_cap", 0.0));
    out.boundary_fail_occupied = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_boundary_fail_occupied", 0.0));
    out.boundary_fail_deadline = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_boundary_fail_deadline", 0.0));
    out.boundary_fail_out_of_domain = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_boundary_fail_out_of_domain", 0.0));
    out.boundary_fail_split = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_boundary_fail_split", 0.0));
    out.boundary_failed_seed_memoized = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_boundary_failed_seed_memoized", 0.0));
    out.boundary_skip_failed_seed = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_boundary_skip_failed_seed", 0.0));
    out.boundary_stall = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_boundary_stall", 0.0));
    out.boundary_target_hits = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_boundary_target_hits", 0.0));
    for (std::size_t i = boxes_before; i < boxes_.size(); ++i) {
        out.committed_boxes.push_back(boxes_[i].joint_intervals);
    }
    // Export EVERY forest box so callers can measure the bridge's true coverage:
    // chain_pave may COVER a path point by reusing a pre-existing forest box
    // (committed during build), which would otherwise be invisible to a caller
    // inspecting only `committed_boxes` and thus look like an uncovered gap.
    out.all_boxes.reserve(boxes_.size());
    for (const auto& box : boxes_) {
        out.all_boxes.push_back(box.joint_intervals);
    }
    if (out.added > 0) {
        invalidate_query_cache();
    }
    invalidate_query_cache();
    return out;
}

DebugChainPaveResult RBFPlanningForest::debug_chain_pave_waypoints(
    const std::vector<Eigen::VectorXd>& waypoint_path,
    const ChainPaveConfig& pave) {
    DebugChainPaveResult out;
    if (waypoint_path.empty() || boxes_.empty() || !oracle_) {
        return out;
    }
    const Eigen::VectorXd& start = waypoint_path.front();
    const Eigen::VectorXd& goal = waypoint_path.back();
    const int start_box_id = locate_box_partition_first(start, config_.query.nearest_if_outside);
    if (start_box_id < 0) {
        return out;
    }
    const int goal_box_id = locate_box_partition_first(goal, config_.query.nearest_if_outside);
    if (goal_box_id < 0 || goal_box_id == start_box_id) {
        return out;
    }
    out.start_box_id = start_box_id;
    out.goal_box_id = goal_box_id;
    out.bridge_found = true;
    out.waypoints = waypoint_path;
    for (const auto& box : boxes_) {
        if (box.id == start_box_id) {
            out.start_box = box.joint_intervals;
        }
        if (box.id == goal_box_id) {
            out.goal_box = box.joint_intervals;
        }
    }
    CollisionChecker checker = make_audit_checker(audit_robot_, scene_, config_.query);
    out.audit_passed = audit_waypoint_path(waypoint_path,
                                           checker,
                                           config_.query.audit_resolution,
                                           config_.query.audit_segment_step)
                           .passed;
    StageContext context = StageContext::from_runtime(config_.runtime);
    const std::size_t boxes_before = boxes_.size();
    if (partition_native_mode()) {
        out.added = add_partition_box_corridor_overlay(start,
                                                       goal,
                                                       waypoint_path,
                                                       "debug_chain_pave_waypoints",
                                                       false,
                                                       false,
                                                       -1,
                                                       &last_build_);
        out.boundary_ffb_calls = static_cast<int>(
            last_build_.diagnostics[
                "debug_chain_pave_waypoints.partition_box_corridor_overlay_attempts"]);
        out.boundary_commits = out.added;
        for (std::size_t i = boxes_before; i < boxes_.size(); ++i) {
            out.committed_boxes.push_back(boxes_[i].joint_intervals);
        }
        out.all_boxes.reserve(boxes_.size());
        for (const auto& box : boxes_) {
            out.all_boxes.push_back(box.joint_intervals);
        }
        if (out.added > 0) {
            invalidate_query_cache();
        }
        invalidate_query_cache();
        return out;
    }
    int next_id = next_box_id();
    ChainPaveConfig debug_pave = pave;
    debug_pave.debug_boundary_failures = &out.boundary_failures;
    out.added = chain_pave_along_path(
        waypoint_path,
        start_box_id,
        boxes_,
        *oracle_,
        adjacency_,
        next_id,
        context,
        debug_pave);
    if (boxes_.size() > boxes_before) {
        append_adaptive_partition_boxes(boxes_before, &last_build_, "debug_chain_pave_waypoints");
    }
    out.fast_gap_fill_ffb_calls = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_fast_ffb_calls", 0.0));
    out.fast_gap_fill_ms =
        context.diagnostics().value("connector.chain_pave_fast_ms", 0.0);
    out.boundary_ffb_calls = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_boundary_ffb_calls", 0.0));
    out.boundary_commits = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_boundary_commits", 0.0));
    out.boundary_reject_not_free = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_boundary_reject_not_free", 0.0));
    out.boundary_reject_non_adjacent = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_boundary_reject_non_adjacent", 0.0));
    out.boundary_fail_seed_collision = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_boundary_fail_seed_collision", 0.0));
    out.boundary_fail_depth_cap = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_boundary_fail_depth_cap", 0.0));
    out.boundary_fail_unknown_depth_cap = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_boundary_fail_unknown_depth_cap", 0.0));
    out.boundary_fail_reserved_depth_cap = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_boundary_fail_reserved_depth_cap", 0.0));
    out.boundary_fail_occupied = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_boundary_fail_occupied", 0.0));
    out.boundary_fail_deadline = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_boundary_fail_deadline", 0.0));
    out.boundary_fail_out_of_domain = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_boundary_fail_out_of_domain", 0.0));
    out.boundary_fail_split = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_boundary_fail_split", 0.0));
    out.boundary_failed_seed_memoized = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_boundary_failed_seed_memoized", 0.0));
    out.boundary_skip_failed_seed = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_boundary_skip_failed_seed", 0.0));
    out.boundary_stall = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_boundary_stall", 0.0));
    out.boundary_target_hits = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_boundary_target_hits", 0.0));
    for (std::size_t i = boxes_before; i < boxes_.size(); ++i) {
        out.committed_boxes.push_back(boxes_[i].joint_intervals);
    }
    out.all_boxes.reserve(boxes_.size());
    for (const auto& box : boxes_) {
        out.all_boxes.push_back(box.joint_intervals);
    }
    if (out.added > 0) {
        invalidate_query_cache();
    }
    invalidate_query_cache();
    return out;
}

int RBFPlanningForest::refine_query_corridor(const Eigen::Ref<const Eigen::VectorXd>& start,
                                         const Eigen::Ref<const Eigen::VectorXd>& goal,
                                         int max_boxes_to_add) {
    return refine_query_corridor(start,
                                 goal,
                                 max_boxes_to_add,
                                 CorridorRefineMode::LegacyBridge,
                                 std::numeric_limits<double>::infinity(),
                                 std::numeric_limits<double>::infinity());
}

int RBFPlanningForest::refine_query_corridor(const Eigen::Ref<const Eigen::VectorXd>& start,
                                         const Eigen::Ref<const Eigen::VectorXd>& goal,
                                         int max_boxes_to_add,
                                         CorridorRefineMode mode,
                                         double long_path_ratio,
                                         double long_path_min_delta) {
    if (boxes_.empty() || !oracle_ || max_boxes_to_add <= 0) {
        return 0;
    }
    CollisionChecker checker = make_audit_checker(audit_robot_, scene_, config_.query);
    QueryResult probe = run_query_internal(start, goal, false);

    std::vector<Eigen::VectorXd> waypoint_path;
    bool add_query_segment_edge = false;
    const bool graph_only_success =
        probe.success && probe.audit_passed && probe.repair_count == 0 && probe.segment_edges_used == 0 && !probe.path.empty();
    if (graph_only_success) {
        if (mode != CorridorRefineMode::BoxOnlyLongPath) {
            return 0;
        }
        const double direct = (goal - start).norm();
        const double delta = probe.path_length - direct;
        const bool ratio_trigger =
            direct > 1e-9 && std::isfinite(long_path_ratio) && probe.path_length / direct >= long_path_ratio;
        const bool delta_trigger = std::isfinite(long_path_min_delta) && delta >= long_path_min_delta;
        if (!ratio_trigger && !delta_trigger) {
            return 0;
        }
        StageContext rrt_context = StageContext::serial();
        RRTConnectConfig refine_rrt = with_query_root_hull_domain(config_.connector.rrt, *oracle_, start, goal);
        refine_rrt.segment_resolution = std::max(refine_rrt.segment_resolution, config_.query.audit_resolution);
        const int refine_attempts = std::max(1, config_.connector.max_pairs_per_gap);
        waypoint_path = best_audited_rrt_bridge_path(start,
                                                     goal,
                                                     checker,
                                                     audit_robot_,
                                                     rrt_context,
                                                     refine_rrt,
                                                     refine_attempts,
                                                     config_.connector.per_pair_timeout_ms * refine_attempts,
                                                     derived_planner_seed(config_.grower.rng_seed,
                                                                          kSeedCorridorRefineOffset,
                                                                          0),
                                                     config_.query.audit_resolution,
                                                     config_.query.audit_segment_step);
        if (waypoint_path.empty()) {
            waypoint_path = probe.path;
        }
    } else if (probe.success && probe.audit_passed && !probe.path.empty()) {
        if (mode == CorridorRefineMode::BoxOnlyLongPath) {
            const double direct = (goal - start).norm();
            const double delta = probe.path_length - direct;
            const bool ratio_trigger =
                direct > 1e-9 && std::isfinite(long_path_ratio) && probe.path_length / direct >= long_path_ratio;
            const bool delta_trigger = std::isfinite(long_path_min_delta) && delta >= long_path_min_delta;
            if (!ratio_trigger && !delta_trigger) {
                return 0;
            }
            StageContext rrt_context = StageContext::serial();
            RRTConnectConfig refine_rrt = with_query_root_hull_domain(config_.connector.rrt, *oracle_, start, goal);
            refine_rrt.segment_resolution = std::max(refine_rrt.segment_resolution, config_.query.audit_resolution);
            const int refine_attempts = std::max(1, config_.connector.max_pairs_per_gap);
            waypoint_path = best_audited_rrt_bridge_path(start,
                                                         goal,
                                                         checker,
                                                         audit_robot_,
                                                         rrt_context,
                                                         refine_rrt,
                                                         refine_attempts,
                                                         config_.connector.per_pair_timeout_ms * refine_attempts,
                                                         derived_planner_seed(config_.grower.rng_seed,
                                                                              kSeedCorridorRefineOffset,
                                                                              1),
                                                         config_.query.audit_resolution,
                                                         config_.query.audit_segment_step);
            if (waypoint_path.empty()) {
                waypoint_path = probe.path;
            }
        } else {
            waypoint_path = probe.path;
            add_query_segment_edge = true;
        }
    } else {
        StageContext rrt_context = StageContext::serial();
        RRTConnectConfig refine_rrt = with_query_root_hull_domain(config_.connector.rrt, *oracle_, start, goal);
        refine_rrt.segment_resolution = std::max(refine_rrt.segment_resolution, config_.query.audit_resolution);
        const int refine_attempts = std::max(1, config_.connector.max_pairs_per_gap);
        waypoint_path = best_audited_rrt_bridge_path(start,
                                                     goal,
                                                     checker,
                                                     audit_robot_,
                                                     rrt_context,
                                                     refine_rrt,
                                                     refine_attempts,
                                                     config_.connector.per_pair_timeout_ms * refine_attempts,
                                                     derived_planner_seed(config_.grower.rng_seed,
                                                                          kSeedCorridorRefineOffset,
                                                                          2),
                                                     config_.query.audit_resolution,
                                                     config_.query.audit_segment_step);
        add_query_segment_edge = mode != CorridorRefineMode::BoxOnlyLongPath;
    }
    if (waypoint_path.empty()) {
        return 0;
    }
    const int anchor_box_id = locate_box_partition_first(waypoint_path.front(), config_.query.nearest_if_outside);
    if (anchor_box_id < 0) {
        return 0;
    }

    if (partition_native_mode()) {
        last_build_.diagnostics["refine_query_corridor.partition_native"] = 1.0;
        const double edge_count_before =
            last_build_.diagnostics["refine_query_corridor.partition_box_corridor_overlay_added"];
        const int added = add_partition_box_corridor_overlay(start,
                                                             goal,
                                                             waypoint_path,
                                                             "refine_query_corridor",
                                                             true,
                                                             true,
                                                             -1,
                                                             &last_build_);
        const double edge_count_after =
            last_build_.diagnostics["refine_query_corridor.partition_box_corridor_overlay_added"];
        if (edge_count_after > edge_count_before) {
            last_build_.diagnostics["refine_query_corridor.partition_native_direct_overlay"] += 1.0;
            last_build_.diagnostics["refine_query_corridor.partition_native_query_bridge_skipped"] += 1.0;
            last_build_.diagnostics["refine_query_corridor.partition_native_box_corridor_edges"] += 1.0;
        }
        return added;
    }

    ChainPaveConfig pave_config = config_.connector.pave;
    pave_config.max_chain = std::min(max_boxes_to_add, std::max(1, max_boxes_to_add));
    pave_config.max_steps_per_waypoint = std::max(1, pave_config.max_steps_per_waypoint);
    pave_config.refine_covered_waypoints = true;
    if (mode == CorridorRefineMode::BoxOnlyLongPath) {
        pave_config.fill_gaps = true;
        pave_config.find_free_box.max_depth = std::max(pave_config.find_free_box.max_depth, 64);
        pave_config.gap_fill_sample_step = std::min(pave_config.gap_fill_sample_step, 0.02);
        pave_config.gap_fill_time_budget_ms = std::max(pave_config.gap_fill_time_budget_ms, 200.0);
        pave_config.gap_fill_max_ffb_calls = std::max(pave_config.gap_fill_max_ffb_calls, 512);
        pave_config.gap_fill_min_arc_gain = 0.0;
        pave_config.require_connected_chain = true;
    }
    StageContext context = StageContext::serial();
    int next_id = next_box_id();
    const std::size_t boxes_before_refine = boxes_.size();
    const int added = chain_pave_along_path(
        waypoint_path,
        anchor_box_id,
        boxes_,
        *oracle_,
        adjacency_,
        next_id,
        context,
        pave_config);
    if (added > 0) {
        append_adaptive_partition_boxes(boxes_before_refine,
                                        &last_build_,
                                        "refine_query_corridor");
        invalidate_query_cache();
        if (add_query_segment_edge &&
            config_.connector.segment_edges_enabled &&
            config_.connector.rrt_segment_edges &&
            audit_waypoint_path(waypoint_path,
                                checker,
                                config_.query.audit_resolution,
                                config_.query.audit_segment_step)
                .passed) {
            const int source_box_id = locate_box_partition_first(start, config_.query.nearest_if_outside);
            const int target_box_id = locate_box_partition_first(goal, config_.query.nearest_if_outside);
            if (source_box_id >= 0 && target_box_id >= 0) {
                add_segment_edge_partition_first(                                 source_box_id,
                                 target_box_id,
                                 waypoint_path,
                                 SegmentEdgeType::QueryBridge,
                                 std::max(1, config_.query.audit_resolution),
                                 SegmentEdgeValidation::CollisionChecked,
                                 false);
            }
        }
        invalidate_query_cache();
    }
    return added;
}

RebuildProfile RBFPlanningForest::add_obstacle_and_rebuild(const Obstacle& obstacle) {
    using Clock = std::chrono::steady_clock;
    const auto t0 = Clock::now();
    RebuildProfile profile;
    profile.boxes_before = static_cast<int>(boxes_.size());
    profile.raw_boxes_before = static_cast<int>(raw_boxes_.size());
    profile.obstacles_before = scene_.n_obstacles();
    profile.collision_cache_boxes_before = static_cast<int>(dynamic_collision_box_cache_.size());

    Scene added_scene(std::vector<Obstacle>{obstacle});
    CollisionChecker added_checker(robot_, added_scene);
    Scene updated_scene = scene_;
    updated_scene.add_obstacle(obstacle);
    CollisionChecker updated_checker(robot_, updated_scene);
    std::vector<BoxNode> removed_boxes;
    std::unordered_set<int> removed_box_ids;
    const auto check_t0 = Clock::now();
    profile.used_spatial_dirty_region = config_.dynamic_update.enable_spatial_dirty_region;
    std::vector<int> dirty_indices;
    if (config_.dynamic_update.enable_spatial_dirty_region) {
        const auto dirty_t0 = Clock::now();
        dirty_indices = spatial_dirty_all_box_indices(robot_, boxes_, obstacle, config_.dynamic_update, profile.dirty_boxes);
        profile.dirty_region_ms = std::chrono::duration<double, std::milli>(Clock::now() - dirty_t0).count();
        profile.dirty_boxes_used = static_cast<int>(dirty_indices.size());
    } else {
        dirty_indices.reserve(boxes_.size());
        for (int index = 0; index < static_cast<int>(boxes_.size()); ++index) {
            dirty_indices.push_back(index);
        }
        profile.dirty_boxes = static_cast<int>(dirty_indices.size());
        profile.dirty_boxes_used = profile.dirty_boxes;
    }
    for (int index : dirty_indices) {
        if (index < 0 || index >= static_cast<int>(boxes_.size())) {
            continue;
        }
        const BoxNode& box = boxes_[static_cast<std::size_t>(index)];
        const auto box_check_t0 = Clock::now();
        const bool collides = added_checker.check_box(box.joint_intervals);
        profile.diagnostics["insert.free_box_check_box_calls"] += 1.0;
        profile.diagnostics["insert.free_box_check_box_ms"] +=
            std::chrono::duration<double, std::milli>(Clock::now() - box_check_t0).count();
        if (collides) {
            removed_box_ids.insert(box.id);
            removed_boxes.push_back(box);
        }
    }
    for (std::size_t i = 0; i < boxes_.size();) {
        if (removed_box_ids.find(boxes_[i].id) != removed_box_ids.end()) {
            if (oracle_) {
                oracle_->release_box(boxes_[i].id);
            }
            boxes_.erase(boxes_.begin() + static_cast<std::ptrdiff_t>(i));
            profile.boxes_removed += 1;
        } else {
            ++i;
        }
    }
    for (std::size_t i = 0; i < raw_boxes_.size();) {
        bool remove_raw = removed_box_ids.find(raw_boxes_[i].id) != removed_box_ids.end();
        if (!remove_raw) {
            const auto raw_check_t0 = Clock::now();
            remove_raw = added_checker.check_box(raw_boxes_[i].joint_intervals);
            profile.diagnostics["insert.raw_box_check_box_calls"] += 1.0;
            profile.diagnostics["insert.raw_box_check_box_ms"] +=
                std::chrono::duration<double, std::milli>(Clock::now() - raw_check_t0).count();
        }
        if (remove_raw) {
            raw_boxes_.erase(raw_boxes_.begin() + static_cast<std::ptrdiff_t>(i));
            profile.raw_boxes_removed += 1;
        } else {
            ++i;
        }
    }
    std::unordered_set<int> live_box_ids;
    live_box_ids.reserve(boxes_.size());
    for (const auto& box : boxes_) {
        live_box_ids.insert(box.id);
    }
    const int segment_edges_before = static_cast<int>(segment_edges_.size());
    int segment_edges_removed_dead_endpoint = 0;
    int segment_edges_audited = 0;
    int segment_edges_removed_audit = 0;
    double segment_edge_audit_ms = 0.0;
    segment_edges_.erase(std::remove_if(segment_edges_.begin(), segment_edges_.end(), [&](const SegmentEdge& edge) {
        if (live_box_ids.find(edge.source_box_id) == live_box_ids.end() ||
            live_box_ids.find(edge.target_box_id) == live_box_ids.end()) {
            segment_edges_removed_dead_endpoint += 1;
            return true;
        }
        const auto edge_t0 = Clock::now();
        segment_edges_audited += 1;
        const bool survives = segment_edge_survives_scene(
            edge, updated_checker, config_.query.audit_resolution, config_.query.audit_segment_step);
        segment_edge_audit_ms += std::chrono::duration<double, std::milli>(Clock::now() - edge_t0).count();
        if (!survives) {
            segment_edges_removed_audit += 1;
            if (!partition_native_mode()) {
                const BoxNode* source_box = find_box_by_id(boxes_, edge.source_box_id);
                const BoxNode* target_box = find_box_by_id(boxes_, edge.target_box_id);
                if (source_box == nullptr || target_box == nullptr ||
                    !boxes_connected(*source_box, *target_box, config_.query.adjacency_tolerance)) {
                    remove_local_edge(adjacency_, edge.source_box_id, edge.target_box_id);
                }
            }
        }
        return !survives;
    }), segment_edges_.end());
    profile.diagnostics["insert.segment_edges_before"] = static_cast<double>(segment_edges_before);
    profile.diagnostics["insert.segment_edges_removed_dead_endpoint"] = static_cast<double>(segment_edges_removed_dead_endpoint);
    profile.diagnostics["insert.segment_edges_audited"] = static_cast<double>(segment_edges_audited);
    profile.diagnostics["insert.segment_edges_removed_audit"] = static_cast<double>(segment_edges_removed_audit);
    profile.diagnostics["insert.segment_edge_audit_ms"] = segment_edge_audit_ms;
    profile.collision_check_ms = std::chrono::duration<double, std::milli>(Clock::now() - check_t0).count();

    scene_ = std::move(updated_scene);
    const auto reset_t0 = Clock::now();
    reset_oracle(scene_);
    profile.diagnostics["insert.reset_oracle_ms"] =
        std::chrono::duration<double, std::milli>(Clock::now() - reset_t0).count();
    const auto reserve_t0 = Clock::now();
    reserve_existing_boxes();
    profile.diagnostics["insert.reserve_existing_boxes_ms"] =
        std::chrono::duration<double, std::milli>(Clock::now() - reserve_t0).count();

    const auto regrow_t0 = Clock::now();
    profile.dirty_seed_count = static_cast<int>(removed_boxes.size());
    int next_id = next_box_id();
    const std::size_t first_new_box_index = boxes_.size();
    for (const auto& removed_box : removed_boxes) {
        profile.diagnostics["insert.refill_domains"] += 1.0;
        int sweep_max_depth = config_.dynamic_update.insertion_leaf_sweep_max_depth;
        if (oracle_ && config_.dynamic_update.insertion_leaf_sweep_relative_depth >= 0 && removed_box.tree_id >= 0) {
            sweep_max_depth = std::min(
                sweep_max_depth,
                oracle_->depth(removed_box.tree_id) + config_.dynamic_update.insertion_leaf_sweep_relative_depth);
        }
        refill_removed_box_with_leaf_sweep(removed_box,
                                           profile.obstacles_before,
                                           sweep_max_depth,
                                           next_id,
                                           profile);
    }
    profile.regrow_ms = std::chrono::duration<double, std::milli>(Clock::now() - regrow_t0).count();

    profile.boxes_after = static_cast<int>(boxes_.size());
    profile.raw_boxes_after = static_cast<int>(raw_boxes_.size());
    profile.obstacles_after = scene_.n_obstacles();
    const auto adj_t0 = Clock::now();
    if (partition_native_mode()) {
        refresh_dynamic_partition_after_remove_append(profile,
                                                      removed_box_ids,
                                                      first_new_box_index,
                                                      "insert.partition_delta");
    } else {
        remove_adjacency_nodes(adjacency_, removed_box_ids);
        connect_incremental_boxes(adjacency_, boxes_, first_new_box_index, config_.query.adjacency_tolerance);
        apply_segment_edges_to_adjacency(segment_edges_, adjacency_);
        invalidate_query_cache();
        profile.adjacency_islands = static_cast<int>(find_islands(adjacency_).size());
    }
    profile.adjacency_ms = std::chrono::duration<double, std::milli>(Clock::now() - adj_t0).count();
    profile.collision_cache_boxes_after = static_cast<int>(dynamic_collision_box_cache_.size());
    profile.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
    invalidate_query_cache();
    return profile;
}

RebuildProfile RBFPlanningForest::add_obstacles_and_rebuild(const std::vector<Obstacle>& obstacles) {
    using Clock = std::chrono::steady_clock;
    const auto t0 = Clock::now();
    RebuildProfile profile;
    profile.boxes_before = static_cast<int>(boxes_.size());
    profile.raw_boxes_before = static_cast<int>(raw_boxes_.size());
    profile.obstacles_before = scene_.n_obstacles();
    profile.collision_cache_boxes_before = static_cast<int>(dynamic_collision_box_cache_.size());
    if (obstacles.empty()) {
        profile.boxes_after = profile.boxes_before;
        profile.raw_boxes_after = profile.raw_boxes_before;
        profile.obstacles_after = profile.obstacles_before;
        profile.adjacency_islands = island_count_partition_first();
        profile.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
        return profile;
    }

    Scene added_scene(obstacles);
    CollisionChecker added_checker(robot_, added_scene);
    Scene updated_scene = scene_;
    for (const auto& obstacle : obstacles) {
        updated_scene.add_obstacle(obstacle);
    }
    CollisionChecker updated_checker(robot_, updated_scene);

    std::vector<BoxNode> removed_boxes;
    std::unordered_set<int> removed_box_ids;
    const auto check_t0 = Clock::now();
    profile.used_spatial_dirty_region = config_.dynamic_update.enable_spatial_dirty_region;
    std::vector<int> dirty_indices;
    if (config_.dynamic_update.enable_spatial_dirty_region) {
        const auto dirty_t0 = Clock::now();
        dirty_indices.reserve(boxes_.size());
        std::unordered_set<int> dirty_set;
        for (const auto& obstacle : obstacles) {
            int obstacle_dirty_count = 0;
            auto current = spatial_dirty_all_box_indices(robot_, boxes_, obstacle, config_.dynamic_update, obstacle_dirty_count);
            profile.dirty_boxes += obstacle_dirty_count;
            for (int index : current) {
                if (dirty_set.insert(index).second) {
                    dirty_indices.push_back(index);
                }
            }
        }
        profile.dirty_region_ms = std::chrono::duration<double, std::milli>(Clock::now() - dirty_t0).count();
        profile.dirty_boxes_used = static_cast<int>(dirty_indices.size());
    } else {
        dirty_indices.reserve(boxes_.size());
        for (int index = 0; index < static_cast<int>(boxes_.size()); ++index) {
            dirty_indices.push_back(index);
        }
        profile.dirty_boxes = static_cast<int>(dirty_indices.size());
        profile.dirty_boxes_used = profile.dirty_boxes;
    }

    for (int index : dirty_indices) {
        if (index < 0 || index >= static_cast<int>(boxes_.size())) {
            continue;
        }
        const BoxNode& box = boxes_[static_cast<std::size_t>(index)];
        const auto box_check_t0 = Clock::now();
        const bool collides = added_checker.check_box(box.joint_intervals);
        profile.diagnostics["insert.free_box_check_box_calls"] += 1.0;
        profile.diagnostics["insert.free_box_check_box_ms"] +=
            std::chrono::duration<double, std::milli>(Clock::now() - box_check_t0).count();
        if (collides) {
            removed_box_ids.insert(box.id);
            removed_boxes.push_back(box);
        }
    }
    for (std::size_t i = 0; i < boxes_.size();) {
        if (removed_box_ids.find(boxes_[i].id) != removed_box_ids.end()) {
            if (oracle_) {
                oracle_->release_box(boxes_[i].id);
            }
            boxes_.erase(boxes_.begin() + static_cast<std::ptrdiff_t>(i));
            profile.boxes_removed += 1;
        } else {
            ++i;
        }
    }
    for (std::size_t i = 0; i < raw_boxes_.size();) {
        bool remove_raw = removed_box_ids.find(raw_boxes_[i].id) != removed_box_ids.end();
        if (!remove_raw) {
            const auto raw_check_t0 = Clock::now();
            remove_raw = added_checker.check_box(raw_boxes_[i].joint_intervals);
            profile.diagnostics["insert.raw_box_check_box_calls"] += 1.0;
            profile.diagnostics["insert.raw_box_check_box_ms"] +=
                std::chrono::duration<double, std::milli>(Clock::now() - raw_check_t0).count();
        }
        if (remove_raw) {
            raw_boxes_.erase(raw_boxes_.begin() + static_cast<std::ptrdiff_t>(i));
            profile.raw_boxes_removed += 1;
        } else {
            ++i;
        }
    }

    std::unordered_set<int> live_box_ids;
    live_box_ids.reserve(boxes_.size());
    for (const auto& box : boxes_) {
        live_box_ids.insert(box.id);
    }
    const int segment_edges_before = static_cast<int>(segment_edges_.size());
    int segment_edges_removed_dead_endpoint = 0;
    int segment_edges_audited = 0;
    int segment_edges_removed_audit = 0;
    double segment_edge_audit_ms = 0.0;
    segment_edges_.erase(std::remove_if(segment_edges_.begin(), segment_edges_.end(), [&](const SegmentEdge& edge) {
        if (live_box_ids.find(edge.source_box_id) == live_box_ids.end() ||
            live_box_ids.find(edge.target_box_id) == live_box_ids.end()) {
            segment_edges_removed_dead_endpoint += 1;
            return true;
        }
        const auto edge_t0 = Clock::now();
        segment_edges_audited += 1;
        const bool survives = segment_edge_survives_scene(
            edge, updated_checker, config_.query.audit_resolution, config_.query.audit_segment_step);
        segment_edge_audit_ms += std::chrono::duration<double, std::milli>(Clock::now() - edge_t0).count();
        if (!survives) {
            segment_edges_removed_audit += 1;
            if (!partition_native_mode()) {
                const BoxNode* source_box = find_box_by_id(boxes_, edge.source_box_id);
                const BoxNode* target_box = find_box_by_id(boxes_, edge.target_box_id);
                if (source_box == nullptr || target_box == nullptr ||
                    !boxes_connected(*source_box, *target_box, config_.query.adjacency_tolerance)) {
                    remove_local_edge(adjacency_, edge.source_box_id, edge.target_box_id);
                }
            }
        }
        return !survives;
    }), segment_edges_.end());
    profile.diagnostics["insert.segment_edges_before"] = static_cast<double>(segment_edges_before);
    profile.diagnostics["insert.segment_edges_removed_dead_endpoint"] = static_cast<double>(segment_edges_removed_dead_endpoint);
    profile.diagnostics["insert.segment_edges_audited"] = static_cast<double>(segment_edges_audited);
    profile.diagnostics["insert.segment_edges_removed_audit"] = static_cast<double>(segment_edges_removed_audit);
    profile.diagnostics["insert.segment_edge_audit_ms"] = segment_edge_audit_ms;
    profile.collision_check_ms = std::chrono::duration<double, std::milli>(Clock::now() - check_t0).count();

    scene_ = std::move(updated_scene);
    const auto reset_t0 = Clock::now();
    reset_oracle(scene_);
    profile.diagnostics["insert.reset_oracle_ms"] =
        std::chrono::duration<double, std::milli>(Clock::now() - reset_t0).count();
    const auto reserve_t0 = Clock::now();
    reserve_existing_boxes();
    profile.diagnostics["insert.reserve_existing_boxes_ms"] =
        std::chrono::duration<double, std::milli>(Clock::now() - reserve_t0).count();

    const auto regrow_t0 = Clock::now();
    profile.dirty_seed_count = static_cast<int>(removed_boxes.size());
    int next_id = next_box_id();
    const std::size_t first_new_box_index = boxes_.size();
    for (const auto& removed_box : removed_boxes) {
        profile.diagnostics["insert.refill_domains"] += 1.0;
        int sweep_max_depth = config_.dynamic_update.insertion_leaf_sweep_max_depth;
        if (oracle_ && config_.dynamic_update.insertion_leaf_sweep_relative_depth >= 0 && removed_box.tree_id >= 0) {
            sweep_max_depth = std::min(
                sweep_max_depth,
                oracle_->depth(removed_box.tree_id) + config_.dynamic_update.insertion_leaf_sweep_relative_depth);
        }
        refill_removed_box_with_leaf_sweep(removed_box,
                                           profile.obstacles_before,
                                           sweep_max_depth,
                                           next_id,
                                           profile);
    }
    profile.regrow_ms = std::chrono::duration<double, std::milli>(Clock::now() - regrow_t0).count();

    profile.boxes_after = static_cast<int>(boxes_.size());
    profile.raw_boxes_after = static_cast<int>(raw_boxes_.size());
    profile.obstacles_after = scene_.n_obstacles();
    const auto adj_t0 = Clock::now();
    if (partition_native_mode()) {
        refresh_dynamic_partition_after_remove_append(profile,
                                                      removed_box_ids,
                                                      first_new_box_index,
                                                      "insert_batch.partition_delta");
    } else {
        remove_adjacency_nodes(adjacency_, removed_box_ids);
        connect_incremental_boxes(adjacency_, boxes_, first_new_box_index, config_.query.adjacency_tolerance);
        apply_segment_edges_to_adjacency(segment_edges_, adjacency_);
        invalidate_query_cache();
        profile.adjacency_islands = static_cast<int>(find_islands(adjacency_).size());
    }
    profile.adjacency_ms = std::chrono::duration<double, std::milli>(Clock::now() - adj_t0).count();
    profile.collision_cache_boxes_after = static_cast<int>(dynamic_collision_box_cache_.size());
    profile.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
    invalidate_query_cache();
    return profile;
}

RebuildProfile RBFPlanningForest::connect_update_segment_fallback() {
    using Clock = std::chrono::steady_clock;
    const auto t0 = Clock::now();
    RebuildProfile profile;
    profile.boxes_before = static_cast<int>(boxes_.size());
    profile.raw_boxes_before = static_cast<int>(raw_boxes_.size());
    profile.obstacles_before = scene_.n_obstacles();
	    profile.obstacles_after = profile.obstacles_before;
	    profile.collision_cache_boxes_before = static_cast<int>(dynamic_collision_box_cache_.size());
	    profile.collision_cache_boxes_after = profile.collision_cache_boxes_before;
	    profile.diagnostics["segment_fallback.segment_edges_before"] = static_cast<double>(segment_edges_.size());
	    const bool use_partition_backend =
	        partition_native_mode() && adaptive_partition_query_enabled_ && adaptive_partition_;
	    const int islands_before = use_partition_backend
	        ? adaptive_partition_->component_count_with_overlay()
	        : static_cast<int>(find_islands(adjacency_).size());
	    profile.diagnostics["segment_fallback.islands_before"] = static_cast<double>(islands_before);

	    if (!oracle_ || boxes_.empty()) {
	        profile.boxes_after = profile.boxes_before;
	        profile.raw_boxes_after = profile.raw_boxes_before;
	        profile.adjacency_islands = islands_before;
	        profile.fallback_reason = boxes_.empty() ? "empty_forest" : "missing_oracle";
	        profile.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
	        return profile;
	    }

	    if (use_partition_backend) {
	        const auto connector_t0 = Clock::now();
	        if (islands_before <= 1) {
	            profile.boxes_after = profile.boxes_before;
	            profile.raw_boxes_after = profile.raw_boxes_before;
	            profile.adjacency_islands = islands_before;
	            profile.diagnostics["segment_fallback.partition_native"] = 1.0;
	            profile.diagnostics["segment_fallback.connected"] = 1.0;
	            profile.diagnostics["segment_fallback.segment_edges_after"] =
	                static_cast<double>(segment_edges_.size());
	            profile.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
	            return profile;
	        }
	        CollisionChecker checker = make_audit_checker(audit_robot_, scene_, config_.query);
	        int attempted_pairs = 0;
	        int audit_fail = 0;
	        int added = 0;
	        const int pair_candidate_cap = std::max(
	            8,
	            env_int_or_default("RBF_PARTITION_SEGMENT_FALLBACK_PAIR_CANDIDATE_CAP", 128));
	        const auto candidate_pairs =
	            adaptive_partition_->nearest_component_pairs_to_largest(1, pair_candidate_cap);
	        for (const auto& pair : candidate_pairs) {
	            if (pair.source_box_id < 0 || pair.target_box_id < 0 ||
	                pair.source_point.size() == 0 || pair.target_point.size() == 0) {
	                continue;
	            }
	            ++attempted_pairs;
	            std::vector<Eigen::VectorXd> waypoints{pair.source_point, pair.target_point};
	            const auto audit = audit_waypoint_path(waypoints,
	                                                   checker,
	                                                   config_.query.audit_resolution,
	                                                   config_.query.audit_segment_step);
	            if (!audit.passed) {
	                ++audit_fail;
	                continue;
	            }
	            const int edge_id = add_segment_edge_partition_first(pair.source_box_id,
	                                                                 pair.target_box_id,
	                                                                 std::move(waypoints),
	                                                                 SegmentEdgeType::QueryBridge,
	                                                                 config_.query.audit_resolution,
	                                                                 SegmentEdgeValidation::CollisionChecked,
	                                                                 true,
	                                                                 -1,
	                                                                 nullptr,
	                                                                 "segment_fallback.partition_native");
	            if (edge_id >= 0) {
	                ++added;
	            }
	        }
	        profile.regrow_ms = std::chrono::duration<double, std::milli>(Clock::now() - connector_t0).count();
	        profile.segment_edges_added = added;
	        profile.rrt_segment_edges_added = added;
	        profile.point_gap_segment_edges_added = 0;
	        profile.boxes_added = 0;
	        profile.raw_boxes_added = 0;
	        profile.boxes_after = static_cast<int>(boxes_.size());
	        profile.raw_boxes_after = static_cast<int>(raw_boxes_.size());
	        sync_adaptive_partition_segment_edges(nullptr, "segment_fallback.partition_native");
	        profile.adjacency_islands = adaptive_partition_->component_count_with_overlay();
	        profile.diagnostics["segment_fallback.partition_native"] = 1.0;
	        profile.diagnostics["segment_fallback.attempted_pairs"] = static_cast<double>(attempted_pairs);
	        profile.diagnostics["segment_fallback.audit_fail"] = static_cast<double>(audit_fail);
	        profile.diagnostics["segment_fallback.partition_pair_candidates"] =
	            static_cast<double>(candidate_pairs.size());
	        profile.diagnostics["segment_fallback.connected"] = profile.adjacency_islands <= 1 ? 1.0 : 0.0;
	        profile.diagnostics["segment_fallback.segment_edges_after"] =
	            static_cast<double>(segment_edges_.size());
	        profile.diagnostics["segment_fallback.islands_after"] =
	            static_cast<double>(profile.adjacency_islands);
	        profile.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
	        const auto& partition_stats = adaptive_partition_->stats();
	        profile.diagnostics["adaptive.partition_cells"] = static_cast<double>(partition_stats.cells);
	        profile.diagnostics["adaptive.partition_islands"] = static_cast<double>(partition_stats.islands);
	        profile.diagnostics["adaptive.partition_overlay_edges"] =
	            static_cast<double>(partition_stats.overlay_edges);
	        return profile;
	    }

	    const auto connector_t0 = Clock::now();
    StageContext context = StageContext::from_runtime(config_.runtime);
    CollisionChecker checker(robot_, scene_);
    IslandConnectorConfig connector_config = config_.connector;
    connector_config.segment_edges_enabled = true;
    connector_config.rrt_segment_edges = true;
    connector_config.point_gap_segment_edges = true;
    if (connector_config.n_threads <= 1 && config_.runtime.n_threads > 1) {
        connector_config.n_threads = config_.runtime.n_threads;
    }
    IslandConnector connector(*oracle_, robot_, checker, connector_config);
    int connector_next_id = next_box_id();
    const auto connector_result =
        connector.connect_all(boxes_, adjacency_, segment_edges_, connector_next_id, context);
    profile.regrow_ms = std::chrono::duration<double, std::milli>(Clock::now() - connector_t0).count();

    profile.bridge_boxes_added = connector_result.bridge_boxes_added;
    profile.segment_edges_added = connector_result.segment_edges_added;
    profile.rrt_segment_edges_added = connector_result.rrt_segment_edges_added;
    profile.point_gap_segment_edges_added = connector_result.point_gap_segment_edges_added;
    profile.boxes_added = std::max(0, static_cast<int>(boxes_.size()) - profile.boxes_before);
    profile.raw_boxes_added = std::max(0, static_cast<int>(raw_boxes_.size()) - profile.raw_boxes_before);
    profile.diagnostics["segment_fallback.attempted_pairs"] = static_cast<double>(connector_result.attempted_pairs);
    profile.diagnostics["segment_fallback.connected"] = connector_result.connected ? 1.0 : 0.0;
    profile.diagnostics["segment_fallback.segment_edges_after"] = static_cast<double>(segment_edges_.size());

    const auto adj_t0 = Clock::now();
    apply_segment_edges_to_adjacency(segment_edges_, adjacency_);
    invalidate_query_cache();
    profile.adjacency_ms = std::chrono::duration<double, std::milli>(Clock::now() - adj_t0).count();
    profile.boxes_after = static_cast<int>(boxes_.size());
    profile.raw_boxes_after = static_cast<int>(raw_boxes_.size());
    profile.adjacency_islands = static_cast<int>(find_islands(adjacency_).size());
    profile.diagnostics["segment_fallback.islands_after"] = static_cast<double>(profile.adjacency_islands);
    profile.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
    return profile;
}

RebuildProfile RBFPlanningForest::connect_update_endpoint_segment_fallback(
    const Eigen::Ref<const Eigen::VectorXd>& start,
    const Eigen::Ref<const Eigen::VectorXd>& goal) {
    using Clock = std::chrono::steady_clock;
    const auto t0 = Clock::now();
    RebuildProfile profile;
    profile.boxes_before = static_cast<int>(boxes_.size());
    profile.raw_boxes_before = static_cast<int>(raw_boxes_.size());
    profile.obstacles_before = scene_.n_obstacles();
	    profile.obstacles_after = profile.obstacles_before;
	    profile.collision_cache_boxes_before = static_cast<int>(dynamic_collision_box_cache_.size());
	    profile.collision_cache_boxes_after = profile.collision_cache_boxes_before;
	    profile.diagnostics["segment_fallback.segment_edges_before"] = static_cast<double>(segment_edges_.size());
	    const bool use_partition_backend =
	        partition_native_mode() && adaptive_partition_query_enabled_ && adaptive_partition_;
	    const int islands_before = use_partition_backend
	        ? adaptive_partition_->component_count_with_overlay()
	        : static_cast<int>(find_islands(adjacency_).size());
	    profile.diagnostics["segment_fallback.islands_before"] = static_cast<double>(islands_before);

	    if (!oracle_ || boxes_.empty() || start.size() != oracle_->n_dims() || goal.size() != oracle_->n_dims()) {
	        profile.boxes_after = profile.boxes_before;
	        profile.raw_boxes_after = profile.raw_boxes_before;
	        profile.adjacency_islands = islands_before;
	        profile.fallback_reason = boxes_.empty() ? "empty_forest" : !oracle_ ? "missing_oracle" : "bad_endpoint_dimension";
	        profile.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
	        return profile;
	    }

	    if (use_partition_backend) {
	        const auto endpoint_t0 = Clock::now();
	        const std::size_t boxes_before_partition = boxes_.size();
	        const int edges_before = static_cast<int>(segment_edges_.size());
	        int start_box = locate_box_partition_first(start, config_.query.nearest_if_outside);
	        int goal_box = locate_box_partition_first(goal, config_.query.nearest_if_outside);
	        StageContext context = StageContext::from_runtime(config_.runtime);
	        if (start_box < 0) {
	            start_box = anchor_query_endpoint_box(start, context);
	            profile.regrow_attempts += 1;
	        } else {
	            profile.diagnostics["segment_fallback.start_already_covered"] += 1.0;
	        }
	        if (goal_box < 0) {
	            goal_box = anchor_query_endpoint_box(goal, context);
	            profile.regrow_attempts += 1;
	        } else {
	            profile.diagnostics["segment_fallback.goal_already_covered"] += 1.0;
	        }
	        merge_diagnostic_snapshot(profile.diagnostics, context.diagnostics().snapshot());
	        if (boxes_.size() > boxes_before_partition) {
	            append_adaptive_partition_boxes(boxes_before_partition,
	                                            &last_build_,
	                                            "segment_fallback.endpoint_partition");
	        }
	        start_box = locate_box_partition_first(start, config_.query.nearest_if_outside);
	        goal_box = locate_box_partition_first(goal, config_.query.nearest_if_outside);
	        if (start_box >= 0 &&
	            goal_box >= 0 &&
	            !overlay_path_connected_partition_first(start_box, goal_box)) {
	            EndpointMainBoxCorridorConfig corridor_config;
	            (void)connect_query_endpoint_to_main_box_corridor(start, corridor_config);
	            (void)connect_query_endpoint_to_main_box_corridor(goal, corridor_config);
	        }
	        start_box = locate_box_partition_first(start, config_.query.nearest_if_outside);
	        goal_box = locate_box_partition_first(goal, config_.query.nearest_if_outside);
	        if (start_box >= 0 &&
	            goal_box >= 0 &&
	            !overlay_path_connected_partition_first(start_box, goal_box)) {
	            CollisionChecker checker = make_audit_checker(audit_robot_, scene_, config_.query);
	            std::vector<Eigen::VectorXd> waypoints{start, goal};
	            const auto audit = audit_waypoint_path(waypoints,
	                                                   checker,
	                                                   config_.query.audit_resolution,
	                                                   config_.query.audit_segment_step);
	            profile.diagnostics["segment_fallback.endpoint_direct_attempts"] += 1.0;
	            if (audit.passed) {
	                const int edge_id = add_segment_edge_partition_first(start_box,
	                                                                     goal_box,
	                                                                     std::move(waypoints),
	                                                                     SegmentEdgeType::QueryBridge,
	                                                                     config_.query.audit_resolution,
	                                                                     SegmentEdgeValidation::CollisionChecked,
	                                                                     true,
	                                                                     -1,
	                                                                     nullptr,
	                                                                     "segment_fallback.endpoint_partition");
	                if (edge_id >= 0) {
	                    profile.diagnostics["segment_fallback.endpoint_direct_success"] += 1.0;
	                }
	            } else {
	                profile.diagnostics["segment_fallback.endpoint_direct_audit_fail"] += 1.0;
	            }
	        }
	        sync_adaptive_partition_segment_edges(nullptr, "segment_fallback.endpoint_partition");
	        profile.regrow_ms = std::chrono::duration<double, std::milli>(Clock::now() - endpoint_t0).count();
	        profile.boxes_added = std::max(0, static_cast<int>(boxes_.size()) - profile.boxes_before);
	        profile.raw_boxes_added = std::max(0, static_cast<int>(raw_boxes_.size()) - profile.raw_boxes_before);
	        profile.segment_edges_added = std::max(0, static_cast<int>(segment_edges_.size()) - edges_before);
	        profile.rrt_segment_edges_added = profile.segment_edges_added;
	        profile.point_gap_segment_edges_added = 0;
	        profile.boxes_after = static_cast<int>(boxes_.size());
	        profile.raw_boxes_after = static_cast<int>(raw_boxes_.size());
	        profile.adjacency_islands = adaptive_partition_->component_count_with_overlay();
	        profile.diagnostics["segment_fallback.endpoint_partition_native"] = 1.0;
	        profile.diagnostics["segment_fallback.connected"] =
	            (start_box >= 0 && goal_box >= 0 &&
	             overlay_path_connected_partition_first(start_box, goal_box)) ? 1.0 : 0.0;
	        profile.diagnostics["segment_fallback.segment_edges_after"] =
	            static_cast<double>(segment_edges_.size());
	        profile.diagnostics["segment_fallback.islands_after"] =
	            static_cast<double>(profile.adjacency_islands);
	        const auto& partition_stats = adaptive_partition_->stats();
	        profile.diagnostics["adaptive.partition_cells"] = static_cast<double>(partition_stats.cells);
	        profile.diagnostics["adaptive.partition_islands"] = static_cast<double>(partition_stats.islands);
	        profile.diagnostics["adaptive.partition_overlay_edges"] =
	            static_cast<double>(partition_stats.overlay_edges);
	        profile.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
	        return profile;
	    }

	    const auto endpoint_t0 = Clock::now();
    const std::size_t first_new_box_index = boxes_.size();
    StageContext context = StageContext::from_runtime(config_.runtime);
    FindFreeBoxService ffb(*oracle_);
    FindFreeBoxOptions endpoint_options = config_.grower.find_free_box;
    endpoint_options.reject_seed_collision = true;
    int next_id = next_box_id();
    auto try_endpoint = [&](const Eigen::Ref<const Eigen::VectorXd>& point, const char* label) {
        profile.diagnostics[std::string("segment_fallback.") + label + "_attempts"] += 1.0;
        if (point_covered_by_existing_box_local(boxes_, point)) {
            profile.diagnostics[std::string("segment_fallback.") + label + "_already_covered"] += 1.0;
            return;
        }
        auto result = ffb.find(point, context, endpoint_options);
        profile.regrow_attempts += 1;
        if (!result.found || !intervals_contain_point_local(result.intervals, point, config_.query.adjacency_tolerance)) {
            profile.diagnostics[std::string("segment_fallback.") + label + "_ffb_failed"] += 1.0;
            return;
        }
        if (!allow_dynamic_commit(*oracle_, result, config_.grower.commit_policy)) {
            profile.diagnostics[std::string("segment_fallback.") + label + "_commit_rejected"] += 1.0;
            return;
        }
        BoxNode box;
        box.id = next_id++;
        box.joint_intervals = std::move(result.intervals);
        box.seed_config = point;
        box.tree_id = result.node;
        box.parent_box_id = -1;
        box.root_id = box.id;
        box.safety_status = result.validation_detail.safety_status;
        box.strict_audit_required = result.validation_detail.strict_audit_required;
        box.compute_volume();
        for (const auto& existing : boxes_) {
            if (box_contains_box_exact_local(existing, box)) {
                profile.diagnostics[std::string("segment_fallback.") + label + "_contained_rejected"] += 1.0;
                return;
            }
        }
        int adjacent_parent = -1;
        if (!leaf_refine_has_adjacency(boxes_, box, config_.query.adjacency_tolerance, &adjacent_parent)) {
            profile.diagnostics[std::string("segment_fallback.") + label + "_disconnected_rejected"] += 1.0;
            return;
        }
        box.parent_box_id = adjacent_parent;
        const BoxNode* parent = find_box_by_id(boxes_, adjacent_parent);
        box.root_id = parent != nullptr && parent->root_id >= 0 ? parent->root_id : adjacent_parent;
        oracle_->reserve_node(box.tree_id, box.id);
        boxes_.push_back(box);
        raw_boxes_.push_back(box);
        profile.boxes_added += 1;
        profile.raw_boxes_added += 1;
        profile.diagnostics[std::string("segment_fallback.") + label + "_boxes_added"] += 1.0;
    };
    try_endpoint(start, "start");
    try_endpoint(goal, "goal");
    profile.regrow_ms = std::chrono::duration<double, std::milli>(Clock::now() - endpoint_t0).count();

    const auto pre_connector_adj_t0 = Clock::now();
    if (boxes_.size() > first_new_box_index) {
        connect_incremental_boxes(adjacency_, boxes_, first_new_box_index, config_.query.adjacency_tolerance);
    }
    profile.adjacency_ms += std::chrono::duration<double, std::milli>(Clock::now() - pre_connector_adj_t0).count();

    const auto connector_t0 = Clock::now();
    IslandConnectorConfig connector_config = config_.connector;
    connector_config.segment_edges_enabled = true;
    connector_config.rrt_segment_edges = true;
    connector_config.point_gap_segment_edges = true;
    if (connector_config.n_threads <= 1 && config_.runtime.n_threads > 1) {
        connector_config.n_threads = config_.runtime.n_threads;
    }
    CollisionChecker checker(robot_, scene_);
    IslandConnector connector(*oracle_, robot_, checker, connector_config);
    const auto connector_result =
        connector.connect_all(boxes_, adjacency_, segment_edges_, next_id, context);
    profile.diagnostics["segment_fallback.connector_ms"] =
        std::chrono::duration<double, std::milli>(Clock::now() - connector_t0).count();

    profile.bridge_boxes_added += connector_result.bridge_boxes_added;
    profile.segment_edges_added += connector_result.segment_edges_added;
    profile.rrt_segment_edges_added += connector_result.rrt_segment_edges_added;
    profile.point_gap_segment_edges_added += connector_result.point_gap_segment_edges_added;
    profile.boxes_added = std::max(0, static_cast<int>(boxes_.size()) - profile.boxes_before);
    profile.raw_boxes_added = std::max(0, static_cast<int>(raw_boxes_.size()) - profile.raw_boxes_before);
    profile.diagnostics["segment_fallback.attempted_pairs"] = static_cast<double>(connector_result.attempted_pairs);
    profile.diagnostics["segment_fallback.connected"] = connector_result.connected ? 1.0 : 0.0;
    profile.diagnostics["segment_fallback.segment_edges_after"] = static_cast<double>(segment_edges_.size());

    const auto adj_t0 = Clock::now();
    apply_segment_edges_to_adjacency(segment_edges_, adjacency_);
    invalidate_query_cache();
    profile.adjacency_ms += std::chrono::duration<double, std::milli>(Clock::now() - adj_t0).count();
    profile.boxes_after = static_cast<int>(boxes_.size());
    profile.raw_boxes_after = static_cast<int>(raw_boxes_.size());
    profile.adjacency_islands = static_cast<int>(find_islands(adjacency_).size());
    profile.diagnostics["segment_fallback.islands_after"] = static_cast<double>(profile.adjacency_islands);
    profile.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
    return profile;
}

RebuildProfile RBFPlanningForest::remove_obstacle_and_regrow(int obstacle_index) {
    using Clock = std::chrono::steady_clock;
    const auto t0 = Clock::now();
    RebuildProfile profile;
    profile.boxes_before = static_cast<int>(boxes_.size());
    profile.raw_boxes_before = static_cast<int>(raw_boxes_.size());
    profile.obstacles_before = scene_.n_obstacles();
    profile.removed_obstacle_index = obstacle_index;

    Obstacle removed_obstacle;
    if (!scene_.remove_obstacle_at(obstacle_index, &removed_obstacle)) {
        profile.boxes_after = profile.boxes_before;
        profile.raw_boxes_after = profile.raw_boxes_before;
        profile.obstacles_after = profile.obstacles_before;
        profile.adjacency_islands = island_count_partition_first();
        profile.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
        return profile;
    }
    profile.obstacles_after = scene_.n_obstacles();
    reset_oracle(scene_);
    reserve_existing_boxes();
    const std::unordered_set<int> removed_indices{obstacle_index};
    const std::size_t first_new_box_index = boxes_.size();
    promote_unblocked_collision_cache(removed_indices, profile);

    profile.boxes_after = static_cast<int>(boxes_.size());
    profile.raw_boxes_after = static_cast<int>(raw_boxes_.size());
    const auto adj_t0_delete = Clock::now();
    if (partition_native_mode()) {
        refresh_dynamic_partition_after_append(profile,
                                               first_new_box_index,
                                               "remove.partition_append");
    } else {
        if (profile.boxes_added > 0) {
            connect_incremental_boxes(adjacency_, boxes_, first_new_box_index, config_.query.adjacency_tolerance);
            apply_segment_edges_to_adjacency(segment_edges_, adjacency_);
            invalidate_query_cache();
        }
        profile.adjacency_islands = static_cast<int>(find_islands(adjacency_).size());
    }
    profile.adjacency_ms = std::chrono::duration<double, std::milli>(Clock::now() - adj_t0_delete).count();
    profile.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
    invalidate_query_cache();
    return profile;
}

RebuildProfile RBFPlanningForest::remove_obstacle_suffix_and_regrow(int target_obstacle_count) {
    using Clock = std::chrono::steady_clock;
    const auto t0 = Clock::now();
    RebuildProfile profile;
    profile.boxes_before = static_cast<int>(boxes_.size());
    profile.raw_boxes_before = static_cast<int>(raw_boxes_.size());
    profile.obstacles_before = scene_.n_obstacles();
    const int target_count = std::clamp(target_obstacle_count, 0, profile.obstacles_before);
    profile.removed_obstacle_index = target_count;

    if (target_count >= profile.obstacles_before) {
        profile.boxes_after = profile.boxes_before;
        profile.raw_boxes_after = profile.raw_boxes_before;
        profile.obstacles_after = profile.obstacles_before;
        profile.adjacency_islands = island_count_partition_first();
        profile.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
        return profile;
    }

    std::vector<Obstacle> removed_obstacles;
    removed_obstacles.reserve(static_cast<std::size_t>(profile.obstacles_before - target_count));
    while (scene_.n_obstacles() > target_count) {
        Obstacle removed_obstacle;
        scene_.remove_obstacle_at(scene_.n_obstacles() - 1, &removed_obstacle);
        removed_obstacles.push_back(removed_obstacle);
    }
    profile.obstacles_after = scene_.n_obstacles();
    reset_oracle(scene_);
    reserve_existing_boxes();
    std::unordered_set<int> removed_indices;
    for (int index = target_count; index < profile.obstacles_before; ++index) {
        removed_indices.insert(index);
    }
    const std::size_t first_new_box_index = boxes_.size();
    promote_unblocked_collision_cache(removed_indices, profile);

    profile.boxes_after = static_cast<int>(boxes_.size());
    profile.raw_boxes_after = static_cast<int>(raw_boxes_.size());
    const auto adj_t0_delete = Clock::now();
    if (partition_native_mode()) {
        refresh_dynamic_partition_after_append(profile,
                                               first_new_box_index,
                                               "remove_suffix.partition_append");
    } else {
        if (profile.boxes_added > 0) {
            connect_incremental_boxes(adjacency_, boxes_, first_new_box_index, config_.query.adjacency_tolerance);
            apply_segment_edges_to_adjacency(segment_edges_, adjacency_);
            invalidate_query_cache();
        }
        profile.adjacency_islands = static_cast<int>(find_islands(adjacency_).size());
    }
    profile.adjacency_ms = std::chrono::duration<double, std::milli>(Clock::now() - adj_t0_delete).count();
    profile.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
    invalidate_query_cache();
    return profile;
}

void RBFPlanningForest::clear_forest() {
    boxes_.clear();
    raw_boxes_.clear();
    adjacency_.clear();
    segment_edges_.clear();
    adaptive_partition_.reset();
    adaptive_partition_query_enabled_ = false;
    has_adaptive_partition_config_ = false;
    dynamic_collision_box_cache_.clear();
    if (oracle_) {
        oracle_->clear_reservations();
    }
    invalidate_query_cache();
}

void RBFPlanningForest::reset_oracle(Scene scene) {
    if (!online_cache_) {
        throw std::runtime_error("SBF online envelope cache is not initialised");
    }
    online_cache_->clear_payloads();
    oracle_ = std::make_unique<DatabaseBoxOracle>(
        robot_, *online_cache_, std::move(scene), config_.endpoint_source, config_.envelope_type, config_.validation,
        external_evidence_source_, direct_external_evidence_database_);
    // Preserve the interval-keyed endpoint cache across oracle resets so it
    // persists across queries (endpoints are scene-independent). The cache is
    // memory-bounded by the validation config (OOM guard).
    if (shared_endpoint_cache_) {
        oracle_->set_shared_endpoint_cache(shared_endpoint_cache_);
    } else {
        shared_endpoint_cache_ = oracle_->shared_endpoint_cache();
    }
}

void RBFPlanningForest::reserve_existing_boxes() {
    if (!oracle_) {
        return;
    }
    oracle_->clear_reservations();
    for (const auto& box : boxes_) {
        if (box.tree_id >= 0) {
            oracle_->reserve_node(box.tree_id, box.id);
        }
    }
}

void RBFPlanningForest::rebuild_adjacency() {
    adjacency_ = compute_adjacency(boxes_, config_.query.adjacency_tolerance);
    apply_segment_edges_to_adjacency(segment_edges_, adjacency_);
    invalidate_query_cache();
}

void RBFPlanningForest::rebuild_adaptive_partition(const AdaptiveLeafSweepConfig& config,
                                                   BuildProfile* profile) {
    adaptive_partition_query_enabled_ = false;
    last_adaptive_partition_config_ = config;
    has_adaptive_partition_config_ = true;
    if (config.planning_backend != "partition_native" ||
        !config.grid_face_index_enabled ||
        !oracle_) {
        adaptive_partition_.reset();
        if (profile) {
            profile->diagnostics["adaptive.offline_backend_grid_partition"] = 0.0;
            profile->diagnostics["adaptive.online_backend_partition_native"] = 0.0;
        }
        return;
    }
    if (!adaptive_partition_) {
        adaptive_partition_ = std::make_unique<AdaptiveGridPartition>();
    }
    const int target_depth = config.grid_target_depth > 0
        ? config.grid_target_depth
        : config.target_max_depth;
    bool ok = false;
    try {
        auto root_copies = oracle_->native_root_interval_copies();
        if (root_copies.empty()) {
            root_copies.push_back(oracle_->planning_intervals());
        }
        ok = adaptive_partition_->rebuild(root_copies,
                                          oracle_->database().split_policy_descriptor(),
                                          oracle_->database().root_depth(),
                                          target_depth,
                                          boxes_,
                                          config_.query.adjacency_tolerance);
    } catch (const std::exception&) {
        ok = false;
    }
    adaptive_partition_query_enabled_ = ok;
    if (!adaptive_partition_query_enabled_) {
        adaptive_partition_.reset();
    }
    refresh_adaptive_partition_diagnostics(profile);
}

void RBFPlanningForest::refresh_adaptive_partition_diagnostics(BuildProfile* profile) const {
    if (profile == nullptr) {
        return;
    }
    profile->diagnostics["adaptive.offline_backend_grid_partition"] =
        adaptive_partition_query_enabled_ ? 1.0 : 0.0;
    profile->diagnostics["adaptive.online_backend_partition_native"] =
        adaptive_partition_query_enabled_ ? 1.0 : 0.0;
    if (has_adaptive_partition_config_) {
        const int target_depth = last_adaptive_partition_config_.grid_target_depth > 0
            ? last_adaptive_partition_config_.grid_target_depth
            : last_adaptive_partition_config_.target_max_depth;
        profile->diagnostics["adaptive.grid_target_depth"] = static_cast<double>(target_depth);
        profile->diagnostics["adaptive.grid_face_index_enabled"] =
            last_adaptive_partition_config_.grid_face_index_enabled ? 1.0 : 0.0;
        profile->diagnostics["adaptive.grid_planning_max_expansions"] =
            static_cast<double>(std::max(0, last_adaptive_partition_config_.grid_planning_max_expansions));
    }
    if (adaptive_partition_query_enabled_ && adaptive_partition_) {
        const auto& stats = adaptive_partition_->stats();
        profile->diagnostics["adaptive.partition_cells"] = static_cast<double>(stats.cells);
        profile->diagnostics["adaptive.partition_grid_cells"] = static_cast<double>(stats.grid_cells);
        profile->diagnostics["adaptive.partition_non_grid_cells"] = static_cast<double>(stats.non_grid_cells);
        profile->diagnostics["adaptive.partition_face_index_entries"] =
            static_cast<double>(stats.face_index_entries);
        profile->diagnostics["adaptive.partition_point_index_dims"] =
            static_cast<double>(stats.point_index_dims);
        profile->diagnostics["adaptive.partition_point_index_entries"] =
            static_cast<double>(stats.point_index_entries);
        profile->diagnostics["adaptive.partition_point_index_overflow_cells"] =
            static_cast<double>(stats.point_index_overflow_cells);
        profile->diagnostics["adaptive.partition_sparse_virtual_cells"] =
            static_cast<double>(stats.sparse_virtual_cells);
        profile->diagnostics["adaptive.partition_sparse_virtual_grid_cells"] =
            static_cast<double>(stats.sparse_virtual_grid_cells);
        profile->diagnostics["adaptive.partition_sparse_virtual_non_grid_cells"] =
            static_cast<double>(stats.sparse_virtual_non_grid_cells);
        profile->diagnostics["adaptive.partition_sparse_virtual_exact_index_entries"] =
            static_cast<double>(stats.sparse_virtual_exact_index_entries);
        profile->diagnostics["adaptive.partition_sparse_virtual_max_address_depth"] =
            static_cast<double>(stats.sparse_virtual_max_address_depth);
        profile->diagnostics["adaptive.partition_sparse_virtual_ancestor_refs_avoided"] =
            static_cast<double>(stats.sparse_virtual_ancestor_refs_avoided);
        profile->diagnostics["adaptive.partition_sparse_virtual_index_ms"] =
            stats.sparse_virtual_index_ms;
        profile->diagnostics["adaptive.partition_islands"] = static_cast<double>(stats.islands);
        profile->diagnostics["adaptive.partition_largest_island"] =
            static_cast<double>(stats.largest_island);
        profile->diagnostics["adaptive.partition_adjacency_candidates"] =
            static_cast<double>(stats.adjacency_candidates);
        profile->diagnostics["adaptive.partition_adjacency_tests"] =
            static_cast<double>(stats.adjacency_tests);
        profile->diagnostics["adaptive.partition_adjacency_edges"] =
            static_cast<double>(stats.adjacency_edges);
        profile->diagnostics["adaptive.partition_overlay_edges"] =
            static_cast<double>(stats.overlay_edges);
        profile->diagnostics["adaptive.partition_build_ms"] = stats.build_ms;
        profile->diagnostics["adaptive.partition_index_rebuild_ms"] = stats.index_rebuild_ms;
        profile->diagnostics["adaptive.partition_face_index_ms"] = stats.face_index_ms;
        profile->diagnostics["adaptive.partition_point_index_ms"] = stats.point_index_ms;
        profile->diagnostics["adaptive.partition_neighbor_cache_ms"] = stats.neighbor_cache_ms;
        profile->diagnostics["adaptive.partition_island_rebuild_ms"] = stats.island_rebuild_ms;
    }
}

void RBFPlanningForest::refresh_dynamic_partition_after_update(RebuildProfile& profile,
                                                               const char* diagnostic_prefix) {
    if (!partition_native_mode()) {
        return;
    }
    const auto t0 = std::chrono::steady_clock::now();
    rebuild_adaptive_partition(last_adaptive_partition_config_, nullptr);
    const int synced_edges = sync_adaptive_partition_segment_edges(nullptr, diagnostic_prefix);
    const double ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0).count();
    const std::string prefix = (diagnostic_prefix != nullptr && diagnostic_prefix[0] != '\0')
        ? std::string(diagnostic_prefix)
        : std::string("dynamic.partition");
    profile.diagnostics[prefix + ".rebuild_ms"] += ms;
    profile.diagnostics[prefix + ".edges_synced"] += static_cast<double>(synced_edges);
    profile.diagnostics["adaptive.offline_backend_grid_partition"] =
        adaptive_partition_query_enabled_ ? 1.0 : 0.0;
    profile.diagnostics["adaptive.online_backend_partition_native"] =
        adaptive_partition_query_enabled_ ? 1.0 : 0.0;
    if (adaptive_partition_query_enabled_ && adaptive_partition_) {
        const auto& stats = adaptive_partition_->stats();
        profile.adjacency_islands = adaptive_partition_->component_count_with_overlay();
        profile.diagnostics["adaptive.partition_cells"] = static_cast<double>(stats.cells);
        profile.diagnostics["adaptive.partition_grid_cells"] = static_cast<double>(stats.grid_cells);
        profile.diagnostics["adaptive.partition_non_grid_cells"] = static_cast<double>(stats.non_grid_cells);
        profile.diagnostics["adaptive.partition_face_index_entries"] =
            static_cast<double>(stats.face_index_entries);
        profile.diagnostics["adaptive.partition_point_index_dims"] =
            static_cast<double>(stats.point_index_dims);
        profile.diagnostics["adaptive.partition_point_index_entries"] =
            static_cast<double>(stats.point_index_entries);
        profile.diagnostics["adaptive.partition_point_index_overflow_cells"] =
            static_cast<double>(stats.point_index_overflow_cells);
        profile.diagnostics["adaptive.partition_sparse_virtual_cells"] =
            static_cast<double>(stats.sparse_virtual_cells);
        profile.diagnostics["adaptive.partition_sparse_virtual_grid_cells"] =
            static_cast<double>(stats.sparse_virtual_grid_cells);
        profile.diagnostics["adaptive.partition_sparse_virtual_non_grid_cells"] =
            static_cast<double>(stats.sparse_virtual_non_grid_cells);
        profile.diagnostics["adaptive.partition_sparse_virtual_exact_index_entries"] =
            static_cast<double>(stats.sparse_virtual_exact_index_entries);
        profile.diagnostics["adaptive.partition_sparse_virtual_max_address_depth"] =
            static_cast<double>(stats.sparse_virtual_max_address_depth);
        profile.diagnostics["adaptive.partition_sparse_virtual_ancestor_refs_avoided"] =
            static_cast<double>(stats.sparse_virtual_ancestor_refs_avoided);
        profile.diagnostics["adaptive.partition_sparse_virtual_index_ms"] =
            stats.sparse_virtual_index_ms;
        profile.diagnostics["adaptive.partition_islands"] = static_cast<double>(stats.islands);
        profile.diagnostics["adaptive.partition_largest_island"] =
            static_cast<double>(stats.largest_island);
        profile.diagnostics["adaptive.partition_overlay_edges"] =
            static_cast<double>(stats.overlay_edges);
    }
    invalidate_query_cache();
}

void RBFPlanningForest::refresh_dynamic_partition_after_append(RebuildProfile& profile,
                                                               std::size_t first_box_index,
                                                               const char* diagnostic_prefix) {
    if (!partition_native_mode()) {
        return;
    }
    const auto t0 = std::chrono::steady_clock::now();
    const std::string prefix = (diagnostic_prefix != nullptr && diagnostic_prefix[0] != '\0')
        ? std::string(diagnostic_prefix)
        : std::string("dynamic.partition_append");
    int appended = 0;
    if (first_box_index < boxes_.size()) {
        appended = adaptive_partition_->append_boxes(boxes_,
                                                     first_box_index,
                                                     config_.query.adjacency_tolerance);
    }
    const int synced_edges = sync_adaptive_partition_segment_edges(nullptr, prefix.c_str());
    if (first_box_index < boxes_.size() && appended <= 0) {
        profile.diagnostics[prefix + ".append_failed_rebuilds"] += 1.0;
        refresh_dynamic_partition_after_update(profile,
                                               (prefix + ".rebuild_after_append_failure").c_str());
        return;
    }
    const double ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0).count();
    profile.diagnostics[prefix + ".append_ms"] += ms;
    profile.diagnostics[prefix + ".boxes_appended"] += static_cast<double>(appended);
    profile.diagnostics[prefix + ".edges_synced"] += static_cast<double>(synced_edges);
    profile.diagnostics["adaptive.offline_backend_grid_partition"] =
        adaptive_partition_query_enabled_ ? 1.0 : 0.0;
    profile.diagnostics["adaptive.online_backend_partition_native"] =
        adaptive_partition_query_enabled_ ? 1.0 : 0.0;
    refresh_adaptive_partition_diagnostics(nullptr);
    if (adaptive_partition_query_enabled_ && adaptive_partition_) {
        const auto& stats = adaptive_partition_->stats();
        profile.adjacency_islands = adaptive_partition_->component_count_with_overlay();
        profile.diagnostics["adaptive.partition_cells"] = static_cast<double>(stats.cells);
        profile.diagnostics["adaptive.partition_grid_cells"] = static_cast<double>(stats.grid_cells);
        profile.diagnostics["adaptive.partition_non_grid_cells"] = static_cast<double>(stats.non_grid_cells);
        profile.diagnostics["adaptive.partition_face_index_entries"] =
            static_cast<double>(stats.face_index_entries);
        profile.diagnostics["adaptive.partition_point_index_dims"] =
            static_cast<double>(stats.point_index_dims);
        profile.diagnostics["adaptive.partition_point_index_entries"] =
            static_cast<double>(stats.point_index_entries);
        profile.diagnostics["adaptive.partition_point_index_overflow_cells"] =
            static_cast<double>(stats.point_index_overflow_cells);
        profile.diagnostics["adaptive.partition_sparse_virtual_cells"] =
            static_cast<double>(stats.sparse_virtual_cells);
        profile.diagnostics["adaptive.partition_sparse_virtual_grid_cells"] =
            static_cast<double>(stats.sparse_virtual_grid_cells);
        profile.diagnostics["adaptive.partition_sparse_virtual_non_grid_cells"] =
            static_cast<double>(stats.sparse_virtual_non_grid_cells);
        profile.diagnostics["adaptive.partition_sparse_virtual_exact_index_entries"] =
            static_cast<double>(stats.sparse_virtual_exact_index_entries);
        profile.diagnostics["adaptive.partition_sparse_virtual_max_address_depth"] =
            static_cast<double>(stats.sparse_virtual_max_address_depth);
        profile.diagnostics["adaptive.partition_sparse_virtual_ancestor_refs_avoided"] =
            static_cast<double>(stats.sparse_virtual_ancestor_refs_avoided);
        profile.diagnostics["adaptive.partition_sparse_virtual_index_ms"] =
            stats.sparse_virtual_index_ms;
        profile.diagnostics["adaptive.partition_islands"] = static_cast<double>(stats.islands);
        profile.diagnostics["adaptive.partition_largest_island"] =
            static_cast<double>(stats.largest_island);
        profile.diagnostics["adaptive.partition_overlay_edges"] =
            static_cast<double>(stats.overlay_edges);
    }
    invalidate_query_cache();
}

void RBFPlanningForest::refresh_dynamic_partition_after_remove_append(
    RebuildProfile& profile,
    const std::unordered_set<int>& removed_box_ids,
    std::size_t first_box_index,
    const char* diagnostic_prefix) {
    if (!partition_native_mode()) {
        return;
    }
    const auto t0 = std::chrono::steady_clock::now();
    const std::string prefix = (diagnostic_prefix != nullptr && diagnostic_prefix[0] != '\0')
        ? std::string(diagnostic_prefix)
        : std::string("dynamic.partition_delta");
    const std::size_t expected_appended =
        first_box_index < boxes_.size() ? boxes_.size() - first_box_index : 0u;
    AdaptiveGridPartitionDeltaResult delta;
    if (adaptive_partition_) {
        delta = adaptive_partition_->replace_box_ids_with_boxes(removed_box_ids,
                                                                boxes_,
                                                                first_box_index,
                                                                config_.query.adjacency_tolerance);
    }
    if (!removed_box_ids.empty() &&
        delta.boxes_removed != static_cast<int>(removed_box_ids.size())) {
        profile.diagnostics[prefix + ".remove_failed_rebuilds"] += 1.0;
        profile.diagnostics[prefix + ".boxes_removed_before_rebuild"] +=
            static_cast<double>(delta.boxes_removed);
        refresh_dynamic_partition_after_update(profile,
                                               (prefix + ".rebuild_after_remove_failure").c_str());
        return;
    }
    if (expected_appended > 0u &&
        delta.boxes_appended != static_cast<int>(expected_appended)) {
        profile.diagnostics[prefix + ".append_failed_rebuilds"] += 1.0;
        profile.diagnostics[prefix + ".boxes_appended_before_rebuild"] +=
            static_cast<double>(delta.boxes_appended);
        refresh_dynamic_partition_after_update(profile,
                                               (prefix + ".rebuild_after_append_failure").c_str());
        return;
    }
    const int synced_edges = sync_adaptive_partition_segment_edges(nullptr, prefix.c_str());
    const double ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0).count();
    profile.diagnostics[prefix + ".delta_ms"] += ms;
    profile.diagnostics[prefix + ".replace_ms"] += delta.update_ms;
    profile.diagnostics[prefix + ".boxes_removed"] += static_cast<double>(delta.boxes_removed);
    profile.diagnostics[prefix + ".boxes_appended"] += static_cast<double>(delta.boxes_appended);
    profile.diagnostics[prefix + ".edges_synced"] += static_cast<double>(synced_edges);
    profile.diagnostics["adaptive.offline_backend_grid_partition"] =
        adaptive_partition_query_enabled_ ? 1.0 : 0.0;
    profile.diagnostics["adaptive.online_backend_partition_native"] =
        adaptive_partition_query_enabled_ ? 1.0 : 0.0;
    if (adaptive_partition_query_enabled_ && adaptive_partition_) {
        const auto& stats = adaptive_partition_->stats();
        profile.adjacency_islands = adaptive_partition_->component_count_with_overlay();
        profile.diagnostics["adaptive.partition_cells"] = static_cast<double>(stats.cells);
        profile.diagnostics["adaptive.partition_grid_cells"] = static_cast<double>(stats.grid_cells);
        profile.diagnostics["adaptive.partition_non_grid_cells"] = static_cast<double>(stats.non_grid_cells);
        profile.diagnostics["adaptive.partition_face_index_entries"] =
            static_cast<double>(stats.face_index_entries);
        profile.diagnostics["adaptive.partition_point_index_dims"] =
            static_cast<double>(stats.point_index_dims);
        profile.diagnostics["adaptive.partition_point_index_entries"] =
            static_cast<double>(stats.point_index_entries);
        profile.diagnostics["adaptive.partition_point_index_overflow_cells"] =
            static_cast<double>(stats.point_index_overflow_cells);
        profile.diagnostics["adaptive.partition_sparse_virtual_cells"] =
            static_cast<double>(stats.sparse_virtual_cells);
        profile.diagnostics["adaptive.partition_sparse_virtual_grid_cells"] =
            static_cast<double>(stats.sparse_virtual_grid_cells);
        profile.diagnostics["adaptive.partition_sparse_virtual_non_grid_cells"] =
            static_cast<double>(stats.sparse_virtual_non_grid_cells);
        profile.diagnostics["adaptive.partition_sparse_virtual_exact_index_entries"] =
            static_cast<double>(stats.sparse_virtual_exact_index_entries);
        profile.diagnostics["adaptive.partition_sparse_virtual_max_address_depth"] =
            static_cast<double>(stats.sparse_virtual_max_address_depth);
        profile.diagnostics["adaptive.partition_sparse_virtual_ancestor_refs_avoided"] =
            static_cast<double>(stats.sparse_virtual_ancestor_refs_avoided);
        profile.diagnostics["adaptive.partition_sparse_virtual_index_ms"] =
            stats.sparse_virtual_index_ms;
        profile.diagnostics["adaptive.partition_islands"] = static_cast<double>(stats.islands);
        profile.diagnostics["adaptive.partition_largest_island"] =
            static_cast<double>(stats.largest_island);
        profile.diagnostics["adaptive.partition_overlay_edges"] =
            static_cast<double>(stats.overlay_edges);
    }
    invalidate_query_cache();
}

int RBFPlanningForest::append_adaptive_partition_boxes(std::size_t first_box_index,
                                                       BuildProfile* profile,
                                                       const char* diagnostic_prefix) {
    if (!adaptive_partition_query_enabled_ || !adaptive_partition_ || first_box_index >= boxes_.size()) {
        return 0;
    }
    const int appended = adaptive_partition_->append_boxes(boxes_,
                                                           first_box_index,
                                                           config_.query.adjacency_tolerance);
    if (appended <= 0) {
        return 0;
    }
    if (profile != nullptr && diagnostic_prefix != nullptr && diagnostic_prefix[0] != '\0') {
        profile->diagnostics[std::string(diagnostic_prefix) + ".partition_boxes_appended"] +=
            static_cast<double>(appended);
    }
    sync_adaptive_partition_segment_edges(profile, diagnostic_prefix);
    refresh_adaptive_partition_diagnostics(profile);
    return appended;
}

int RBFPlanningForest::sync_adaptive_partition_segment_edges(BuildProfile* profile,
                                                             const char* diagnostic_prefix) {
    if (!adaptive_partition_query_enabled_ || !adaptive_partition_) {
        return 0;
    }
    const int appended = adaptive_partition_->sync_segment_edges(segment_edges_);
    if (appended > 0 && profile != nullptr && diagnostic_prefix != nullptr && diagnostic_prefix[0] != '\0') {
        profile->diagnostics[std::string(diagnostic_prefix) + ".partition_edges_appended"] +=
            static_cast<double>(appended);
    }
    if (appended > 0) {
        refresh_adaptive_partition_diagnostics(profile);
    }
    return appended;
}

int RBFPlanningForest::add_partition_box_corridor_overlay(
    const Eigen::Ref<const Eigen::VectorXd>& start,
    const Eigen::Ref<const Eigen::VectorXd>& goal,
    const std::vector<Eigen::VectorXd>& waypoint_path,
    const char* diagnostic_prefix,
    bool anchor_endpoints,
    bool skip_if_connected,
    int query_index,
    BuildProfile* profile) {
    BuildProfile* out_profile = profile != nullptr ? profile : &last_build_;
    const std::string prefix =
        (diagnostic_prefix != nullptr && diagnostic_prefix[0] != '\0')
            ? std::string(diagnostic_prefix)
            : std::string("partition_box_corridor");
    auto& diagnostics = out_profile->diagnostics;
    diagnostics[prefix + ".partition_box_corridor_overlay_attempts"] += 1.0;

    if (!partition_native_mode()) {
        diagnostics[prefix + ".partition_box_corridor_overlay_not_partition_native"] += 1.0;
        return 0;
    }
    if (!adaptive_partition_query_enabled_ || !adaptive_partition_ || adaptive_partition_->empty()) {
        diagnostics[prefix + ".partition_box_corridor_overlay_missing_partition"] += 1.0;
        return 0;
    }
    if (waypoint_path.size() < 2) {
        diagnostics[prefix + ".partition_box_corridor_overlay_missing_waypoints"] += 1.0;
        return 0;
    }

    StageContext anchor_context = StageContext::from_runtime(config_.runtime);
    const std::size_t boxes_before_anchor = boxes_.size();
    int source_box_id = locate_box_partition_first(start, false);
    if (source_box_id < 0 && anchor_endpoints) {
        source_box_id = anchor_query_endpoint_box(start, anchor_context);
    }
    int target_box_id = locate_box_partition_first(goal, false);
    if (target_box_id < 0 && anchor_endpoints) {
        target_box_id = anchor_query_endpoint_box(goal, anchor_context);
    }
    for (const auto& [key, value] : anchor_context.diagnostics().snapshot()) {
        diagnostics[prefix + ".partition_box_corridor_anchor." + key] += value;
    }

    int anchors_added = 0;
    if (boxes_.size() > boxes_before_anchor) {
        const std::string anchor_prefix = prefix + ".partition_box_corridor_anchor";
        anchors_added = append_adaptive_partition_boxes(boxes_before_anchor,
                                                        out_profile,
                                                        anchor_prefix.c_str());
        source_box_id = locate_box_partition_first(start, false);
        target_box_id = locate_box_partition_first(goal, false);
    }
    if (anchors_added > 0) {
        diagnostics[prefix + ".partition_box_corridor_overlay_anchor_boxes_added"] +=
            static_cast<double>(anchors_added);
    }

    if (source_box_id < 0 || target_box_id < 0) {
        diagnostics[prefix + ".partition_box_corridor_overlay_missing_endpoint"] += 1.0;
        return anchors_added;
    }
    if (source_box_id == target_box_id ||
        (skip_if_connected && overlay_path_connected_partition_first(source_box_id, target_box_id))) {
        diagnostics[prefix + ".partition_box_corridor_overlay_already_connected"] += 1.0;
        return anchors_added;
    }

    CollisionChecker checker = make_audit_checker(audit_robot_, scene_, config_.query);
    const PathAuditCheck audit = audit_waypoint_path(waypoint_path,
                                                     checker,
                                                     config_.query.audit_resolution,
                                                     config_.query.audit_segment_step);
    if (!audit.passed) {
        diagnostics[prefix + ".partition_box_corridor_overlay_audit_fail"] += 1.0;
        return anchors_added;
    }

    const std::string edge_prefix = prefix + ".partition_native";
    const int edge_id = add_segment_edge_partition_first(source_box_id,
                                                         target_box_id,
                                                         waypoint_path,
                                                         SegmentEdgeType::BoxCorridor,
                                                         std::max(1, config_.query.audit_resolution),
                                                         SegmentEdgeValidation::CollisionChecked,
                                                         false,
                                                         query_index,
                                                         out_profile,
                                                         edge_prefix.c_str());
    if (edge_id < 0) {
        diagnostics[prefix + ".partition_box_corridor_overlay_edge_fail"] += 1.0;
        return anchors_added;
    }
    diagnostics[prefix + ".partition_box_corridor_overlay_added"] += 1.0;
    invalidate_query_cache();
    return anchors_added + 1;
}

int RBFPlanningForest::add_partition_portal_corridor_overlay(
    const Eigen::Ref<const Eigen::VectorXd>& start,
    const Eigen::Ref<const Eigen::VectorXd>& goal,
    const std::vector<Eigen::VectorXd>& waypoint_path,
    const char* diagnostic_prefix,
    bool anchor_endpoints,
    bool skip_if_connected,
    int query_index,
    BuildProfile* profile) {
    BuildProfile* out_profile = profile != nullptr ? profile : &last_build_;
    const std::string prefix =
        (diagnostic_prefix != nullptr && diagnostic_prefix[0] != '\0')
            ? std::string(diagnostic_prefix)
            : std::string("partition_portal_corridor");
    auto& diagnostics = out_profile->diagnostics;
    diagnostics[prefix + ".portal_corridor_attempts"] += 1.0;
    const bool online_portal_prefix =
        prefix.find("hipac_online") != std::string::npos ||
        prefix.find("hipac_promote") != std::string::npos;

    if (!last_adaptive_partition_config_.hipac_portal_connectivity &&
        !(online_portal_prefix && last_adaptive_partition_config_.hipac_online_connectivity)) {
        diagnostics[prefix + ".portal_corridor_disabled"] += 1.0;
        return 0;
    }
    if (!partition_native_mode()) {
        diagnostics[prefix + ".portal_corridor_not_partition_native"] += 1.0;
        return 0;
    }
    if (!adaptive_partition_query_enabled_ || !adaptive_partition_ || adaptive_partition_->empty()) {
        diagnostics[prefix + ".portal_corridor_missing_partition"] += 1.0;
        return 0;
    }
    if (!oracle_ || waypoint_path.size() < 2) {
        diagnostics[prefix + ".portal_corridor_missing_waypoints"] += 1.0;
        return 0;
    }

    StageContext anchor_context = StageContext::from_runtime(config_.runtime);
    const std::size_t boxes_before_anchor = boxes_.size();
    int source_box_id = locate_box_partition_first(start, false);
    if (source_box_id < 0 && anchor_endpoints) {
        source_box_id = anchor_query_endpoint_box(start, anchor_context);
    }
    int target_box_id = locate_box_partition_first(goal, false);
    if (target_box_id < 0 && anchor_endpoints) {
        target_box_id = anchor_query_endpoint_box(goal, anchor_context);
    }
    for (const auto& [key, value] : anchor_context.diagnostics().snapshot()) {
        diagnostics[prefix + ".portal_corridor_anchor." + key] += value;
    }

    int anchors_added = 0;
    if (boxes_.size() > boxes_before_anchor) {
        const std::string anchor_prefix = prefix + ".portal_corridor_anchor";
        anchors_added = append_adaptive_partition_boxes(boxes_before_anchor,
                                                        out_profile,
                                                        anchor_prefix.c_str());
        source_box_id = locate_box_partition_first(start, false);
        target_box_id = locate_box_partition_first(goal, false);
    }
    if (anchors_added > 0) {
        diagnostics[prefix + ".portal_corridor_anchor_boxes_added"] +=
            static_cast<double>(anchors_added);
    }
    if (source_box_id < 0 || target_box_id < 0) {
        diagnostics[prefix + ".portal_corridor_missing_endpoint"] += 1.0;
        return anchors_added;
    }
    if (source_box_id == target_box_id ||
        (skip_if_connected && overlay_path_connected_partition_first(source_box_id, target_box_id))) {
        diagnostics[prefix + ".portal_corridor_already_connected"] += 1.0;
        return anchors_added;
    }

    const BoxNode* source_ptr = find_box_by_id(boxes_, source_box_id);
    const BoxNode* target_ptr = find_box_by_id(boxes_, target_box_id);
    if (source_ptr == nullptr || target_ptr == nullptr) {
        diagnostics[prefix + ".portal_corridor_missing_box"] += 1.0;
        return anchors_added;
    }
    const BoxNode source_box = *source_ptr;
    const BoxNode target_box = *target_ptr;

    const bool online_portal = online_portal_prefix;
    const int max_internal_boxes = online_portal
        ? std::max(0, last_adaptive_partition_config_.hipac_online_max_hidden_boxes_per_portal)
        : std::max(0, last_adaptive_partition_config_.hipac_portal_max_internal_boxes);
    const int max_recursion_depth =
        std::max(0, last_adaptive_partition_config_.hipac_portal_max_recursion_depth);
    if (max_internal_boxes <= 0) {
        diagnostics[prefix + ".portal_corridor_internal_cap_zero"] += 1.0;
        return anchors_added;
    }
    const int requested_depth = last_adaptive_partition_config_.hipac_portal_ffb_depth > 0
        ? last_adaptive_partition_config_.hipac_portal_ffb_depth
        : std::max({config_.query_bridge_pave_depth,
                    config_.connector.pave.find_free_box.max_depth,
                    last_adaptive_partition_config_.target_max_depth});

    StageContext context = StageContext::from_runtime(config_.runtime);
    const auto domain = oracle_->planning_intervals();
    const double tol = config_.query.adjacency_tolerance;
    std::vector<BoxNode> internal_boxes;
    internal_boxes.reserve(static_cast<std::size_t>(std::min(max_internal_boxes, 32)));
    int next_internal_id = -1000000;
    bool chain_ok = false;

    auto append_portal_if_ready = [&]() -> int {
        if (!chain_ok || internal_boxes.empty()) {
            diagnostics[prefix + ".portal_corridor_chain_fail"] += 1.0;
            return anchors_added;
        }

        const int edge_id = append_portal_corridor_edge(segment_edges_,
                                                        source_box,
                                                        target_box,
                                                        std::move(internal_boxes),
                                                        -1,
                                                        tol,
                                                        query_index);
        if (edge_id < 0) {
            diagnostics[prefix + ".portal_corridor_edge_fail"] += 1.0;
            return anchors_added;
        }

        const std::string edge_prefix = prefix + ".partition_native_portal";
        sync_adaptive_partition_segment_edges(out_profile, edge_prefix.c_str());
        diagnostics[prefix + ".portal_corridor_added"] += 1.0;
        invalidate_query_cache();
        return anchors_added + 1;
    };

    auto build_cell_native_chain = [&]() -> bool {
        const auto& split_descriptor = oracle_->database().split_policy_descriptor();
        const int dims = oracle_->n_dims();
        const int max_cell_depth =
            std::max(1, std::min({requested_depth,
                                  config_.database.max_tree_depth,
                                  oracle_->max_tree_depth() - 1,
                                  60}));
        const int selected_build_depth =
            out_profile != nullptr
                ? static_cast<int>(std::round(
                      diagnostic_map_value(out_profile->diagnostics, "adaptive.selected_leaf_depth")))
                : 0;
        const int min_cell_depth = std::max(
            1,
            std::min(max_cell_depth,
                     selected_build_depth > 0
                         ? selected_build_depth
                         : std::max(1, last_adaptive_partition_config_.shallow_max_depth)));
        std::vector<int> candidate_depths;
        for (int depth = min_cell_depth; depth < max_cell_depth; depth += 4) {
            candidate_depths.push_back(depth);
        }
        if (candidate_depths.empty() || candidate_depths.back() != max_cell_depth) {
            candidate_depths.push_back(max_cell_depth);
        }
        struct NativeCellCacheEntry {
            bool free = false;
            OracleNodeId node = kInvalidOracleNodeId;
            std::vector<Interval> intervals;
            BoxSafetyStatus safety_status = BoxSafetyStatus::Unknown;
            bool strict_audit_required = false;
        };
        std::unordered_map<std::string, NativeCellCacheEntry> cell_cache;
        cell_cache.reserve(static_cast<std::size_t>(max_internal_boxes * 4 + 16));
        int cell_validations = 0;
        int cell_free = 0;
        int cell_not_free = 0;
        int cell_invalid = 0;
        int cell_cache_hits = 0;
        int non_adjacent = 0;
        int recursion_splits = 0;
        int internal_cap_hits = 0;

        auto cache_key = [](OracleNodeId node, const std::vector<Interval>& intervals) {
            return std::to_string(node) + ":" +
                   std::to_string(lect_database::fingerprint_intervals(intervals));
        };

        auto classify_cell_at_point_at_depth = [&](const Eigen::VectorXd& point,
                                                   int cell_depth,
                                                   BoxNode& candidate) -> bool {
            if (point.size() != dims ||
                !oracle_->contains_point(oracle_->root_node(), point) ||
                !intervals_contain_point_strict_local(domain, point, 1e-12)) {
                ++cell_invalid;
                return false;
            }
            Eigen::VectorXd tree_seed = oracle_->tree_configuration_for_query(point);
            if (tree_seed.size() != dims) {
                ++cell_invalid;
                return false;
            }
            OracleNodeId node = oracle_->root_node();
            std::vector<Interval> tree_intervals = oracle_->node_intervals(node);
            int changed_dim = -1;
            for (int level = 0; level < cell_depth; ++level) {
                int split_dim = -1;
                if (!split_descriptor.depth_dimensions.empty() &&
                    level < static_cast<int>(split_descriptor.depth_dimensions.size())) {
                    split_dim = split_descriptor.depth_dimensions[static_cast<std::size_t>(level)];
                } else {
                    split_dim = level % dims;
                }
                if (split_dim < 0 ||
                    split_dim >= dims ||
                    split_dim >= static_cast<int>(tree_intervals.size())) {
                    ++cell_invalid;
                    return false;
                }
                auto& interval = tree_intervals[static_cast<std::size_t>(split_dim)];
                const double split_value = interval.center();
                if (!(split_value > interval.lo && split_value < interval.hi)) {
                    ++cell_invalid;
                    return false;
                }
                const bool right_child = tree_seed[split_dim] > split_value;
                if (node > (std::numeric_limits<OracleNodeId>::max() - 2) / 2) {
                    ++cell_invalid;
                    return false;
                }
                node = static_cast<OracleNodeId>(2 * node + (right_child ? 2 : 1));
                if (right_child) {
                    interval.lo = split_value;
                } else {
                    interval.hi = split_value;
                }
                changed_dim = split_dim;
            }
            std::vector<Interval> native_intervals =
                oracle_->query_intervals_for_node(node, tree_intervals, point);
            if (native_intervals.size() != static_cast<std::size_t>(dims) ||
                !intervals_contain_point_strict_local(native_intervals, point, std::max(1e-12, tol))) {
                ++cell_invalid;
                return false;
            }

            const std::string key = cache_key(node, native_intervals);
            const auto cache_it = cell_cache.find(key);
            if (cache_it != cell_cache.end()) {
                ++cell_cache_hits;
                if (!cache_it->second.free) {
                    return false;
                }
                candidate = adaptive_make_box_from_intervals(cache_it->second.intervals,
                                                             cache_it->second.node,
                                                             next_internal_id--,
                                                             cache_it->second.safety_status,
                                                             cache_it->second.strict_audit_required);
                candidate.seed_config = point;
                return true;
            }

            ++cell_validations;
            const BoxValidation validation = oracle_->validate_node(node, native_intervals, changed_dim);
            const OracleValidationDetail detail = oracle_->last_validation_detail();
            NativeCellCacheEntry entry;
            entry.free = validation == BoxValidation::Free &&
                         detail.safety_status == BoxSafetyStatus::CertifiedFree &&
                         !detail.strict_audit_required;
            entry.node = node;
            entry.intervals = native_intervals;
            entry.safety_status = detail.safety_status;
            entry.strict_audit_required = detail.strict_audit_required;
            cell_cache.emplace(key, entry);
            if (!entry.free) {
                ++cell_not_free;
                return false;
            }
            ++cell_free;
            candidate = adaptive_make_box_from_intervals(native_intervals,
                                                         node,
                                                         next_internal_id--,
                                                         detail.safety_status,
                                                         detail.strict_audit_required);
            candidate.seed_config = point;
            return true;
        };
        auto classify_cell_at_point = [&](const Eigen::VectorXd& point,
                                          BoxNode& candidate) -> bool {
            for (int depth : candidate_depths) {
                if (classify_cell_at_point_at_depth(point, depth, candidate)) {
                    return true;
                }
            }
            return false;
        };

        auto boundary_seed_from_box = [&](const BoxNode& box,
                                          const Eigen::VectorXd& from,
                                          const Eigen::VectorXd& to) {
            if (box.n_dims() != from.size() || to.size() != from.size()) {
                return to;
            }
            const Eigen::VectorXd delta = to - from;
            const double norm = delta.norm();
            if (norm <= 1e-12) {
                return to;
            }
            double exit_param = 1.0;
            for (int dim = 0; dim < from.size(); ++dim) {
                const double d = delta[dim];
                if (std::abs(d) < 1e-15) {
                    continue;
                }
                const auto& interval = box.joint_intervals[static_cast<std::size_t>(dim)];
                const double boundary = d > 0.0 ? interval.hi : interval.lo;
                const double t = (boundary - from[dim]) / d;
                if (t > 1e-12 && t < exit_param) {
                    exit_param = t;
                }
            }
            const double face_epsilon = std::max(16.0 * std::max(0.0, tol), 1e-6);
            Eigen::VectorXd seed = from + std::clamp(exit_param, 0.0, 1.0) * delta +
                                   face_epsilon * (delta / norm);
            for (int dim = 0; dim < seed.size() &&
                              dim < static_cast<int>(domain.size()); ++dim) {
                seed[dim] = std::min(domain[static_cast<std::size_t>(dim)].hi,
                                     std::max(domain[static_cast<std::size_t>(dim)].lo,
                                              seed[dim]));
            }
            return seed;
        };

        std::function<bool(BoxNode&, const Eigen::VectorXd&, const Eigen::VectorXd&, int)> connect_to_point;
        connect_to_point = [&](BoxNode& current,
                               const Eigen::VectorXd& from,
                               const Eigen::VectorXd& to,
                               int depth) -> bool {
            if (current.contains(to, tol)) {
                return true;
            }
            if (target_box.contains(to, tol) && boxes_connected(current, target_box, tol)) {
                return true;
            }
            if (static_cast<int>(internal_boxes.size()) >= max_internal_boxes) {
                ++internal_cap_hits;
                return false;
            }
            const Eigen::VectorXd seed = boundary_seed_from_box(current, from, to);
            BoxNode candidate;
            if (classify_cell_at_point(seed, candidate)) {
                if (candidate.safety_status == BoxSafetyStatus::CertifiedFree &&
                    !candidate.strict_audit_required &&
                    boxes_connected(current, candidate, tol)) {
                    internal_boxes.push_back(candidate);
                    current = internal_boxes.back();
                    if (current.contains(to, tol) ||
                        (target_box.contains(to, tol) && boxes_connected(current, target_box, tol))) {
                        return true;
                    }
                    if ((seed - from).norm() <= 1e-12) {
                        return false;
                    }
                    return connect_to_point(current, seed, to, depth);
                }
                ++non_adjacent;
            }
            if (depth <= 0 || from.size() != to.size()) {
                return false;
            }
            ++recursion_splits;
            const Eigen::VectorXd midpoint = 0.5 * (from + to);
            if (!connect_to_point(current, from, midpoint, depth - 1)) {
                return false;
            }
            return connect_to_point(current, midpoint, to, depth - 1);
        };

        bool ok = true;
        BoxNode current = source_box;
        Eigen::VectorXd previous = source_box.center();
        for (const auto& waypoint : waypoint_path) {
            if (waypoint.size() != previous.size()) {
                ok = false;
                break;
            }
        }
        if (ok) {
            for (std::size_t index = 1; index < waypoint_path.size(); ++index) {
                if (!connect_to_point(current, previous, waypoint_path[index], max_recursion_depth)) {
                    ok = false;
                    break;
                }
                previous = waypoint_path[index];
            }
        }
        if (ok && !boxes_connected(current, target_box, tol)) {
            ok = connect_to_point(current, previous, target_box.center(), max_recursion_depth) &&
                 boxes_connected(current, target_box, tol);
        }

        diagnostics[prefix + ".portal_corridor_cell_native_min_depth"] =
            static_cast<double>(min_cell_depth);
        diagnostics[prefix + ".portal_corridor_cell_native_max_depth"] =
            static_cast<double>(max_cell_depth);
        diagnostics[prefix + ".portal_corridor_cell_native_depth_candidates"] =
            static_cast<double>(candidate_depths.size());
        diagnostics[prefix + ".portal_corridor_cell_native_validations"] +=
            static_cast<double>(cell_validations);
        diagnostics[prefix + ".portal_corridor_cell_native_free"] += static_cast<double>(cell_free);
        diagnostics[prefix + ".portal_corridor_cell_native_not_free"] +=
            static_cast<double>(cell_not_free);
        diagnostics[prefix + ".portal_corridor_cell_native_invalid"] +=
            static_cast<double>(cell_invalid);
        diagnostics[prefix + ".portal_corridor_cell_native_cache_hits"] +=
            static_cast<double>(cell_cache_hits);
        diagnostics[prefix + ".portal_corridor_cell_native_non_adjacent"] +=
            static_cast<double>(non_adjacent);
        diagnostics[prefix + ".portal_corridor_cell_native_recursion_splits"] +=
            static_cast<double>(recursion_splits);
        diagnostics[prefix + ".portal_corridor_cell_native_internal_cap_hit"] +=
            static_cast<double>(internal_cap_hits);
        diagnostics[prefix + ".portal_corridor_cell_native_internal_boxes"] +=
            static_cast<double>(internal_boxes.size());
        diagnostics[prefix + ".portal_corridor_internal_boxes"] +=
            static_cast<double>(internal_boxes.size());
        if (ok) {
            diagnostics[prefix + ".portal_corridor_cell_native_success"] += 1.0;
        } else {
            diagnostics[prefix + ".portal_corridor_cell_native_fail"] += 1.0;
        }
        return ok;
    };

    if (last_adaptive_partition_config_.hipac_portal_cell_native_validate) {
        chain_ok = build_cell_native_chain();
        if (chain_ok) {
            return append_portal_if_ready();
        }
        internal_boxes.clear();
    }

    const bool allow_ffb_resolver =
        !last_adaptive_partition_config_.hipac_portal_cell_native_validate ||
        (online_portal && last_adaptive_partition_config_.hipac_online_ffb_portal_fallback);
    if (!allow_ffb_resolver) {
        return anchors_added;
    }

    FindFreeBoxOptions ffb_options = config_.connector.pave.find_free_box;
    ffb_options.max_depth = std::max(1, std::min(requested_depth, config_.database.max_tree_depth));
    ffb_options.skip_existing_cover_check = true;
    ffb_options.reject_seed_collision = false;
    ffb_options.deadline_ms =
        std::max(0.0, last_adaptive_partition_config_.hipac_portal_ffb_deadline_ms);
    const int max_ffb_calls = online_portal
        ? std::max(0, last_adaptive_partition_config_.hipac_online_max_ffb_calls_per_portal)
        : -1;
    int ffb_calls = 0;
    int ffb_success = 0;
    int ffb_fail = 0;
    int non_adjacent = 0;
    int non_certified = 0;
    int recursion_splits = 0;

    std::function<bool(BoxNode&, const Eigen::VectorXd&, const Eigen::VectorXd&, int)> connect_to_point_ffb;
    connect_to_point_ffb = [&](BoxNode& current,
                               const Eigen::VectorXd& from,
                               const Eigen::VectorXd& to,
                               int depth) -> bool {
        if (current.contains(to, tol)) {
            return true;
        }
        if (target_box.contains(to, tol) && boxes_connected(current, target_box, tol)) {
            return true;
        }
        if (static_cast<int>(internal_boxes.size()) >= max_internal_boxes) {
            diagnostics[prefix + ".portal_corridor_internal_cap_hit"] += 1.0;
            return false;
        }
        if (max_ffb_calls >= 0 && ffb_calls >= max_ffb_calls) {
            diagnostics[prefix + ".portal_corridor_ffb_cap_hit"] += 1.0;
            return false;
        }
        ++ffb_calls;
        FindFreeBoxResult found = find_free_box_in_domain(to, domain, context, ffb_options);
        if (found.found) {
            BoxNode candidate = adaptive_make_box_from_intervals(found.intervals,
                                                                 found.node,
                                                                 next_internal_id--,
                                                                 BoxSafetyStatus::CertifiedFree,
                                                                 false);
            candidate.seed_config = to;
            if (candidate.safety_status != BoxSafetyStatus::CertifiedFree ||
                candidate.strict_audit_required) {
                ++non_certified;
            } else if (boxes_connected(current, candidate, tol)) {
                ++ffb_success;
                internal_boxes.push_back(candidate);
                current = internal_boxes.back();
                return true;
            } else {
                ++non_adjacent;
            }
        } else {
            ++ffb_fail;
        }
        if (depth <= 0 || from.size() != to.size()) {
            return false;
        }
        ++recursion_splits;
        const Eigen::VectorXd midpoint = 0.5 * (from + to);
        if (!connect_to_point_ffb(current, from, midpoint, depth - 1)) {
            return false;
        }
        return connect_to_point_ffb(current, midpoint, to, depth - 1);
    };

    chain_ok = true;
    {
        BoxNode current = source_box;
        Eigen::VectorXd previous = source_box.center();
        for (const auto& waypoint : waypoint_path) {
            if (waypoint.size() != previous.size()) {
                chain_ok = false;
                break;
            }
        }
        if (chain_ok) {
            for (std::size_t index = 1; index < waypoint_path.size(); ++index) {
                if (!connect_to_point_ffb(current, previous, waypoint_path[index], max_recursion_depth)) {
                    chain_ok = false;
                    break;
                }
                previous = waypoint_path[index];
            }
        }
        if (chain_ok && !boxes_connected(current, target_box, tol)) {
            chain_ok = connect_to_point_ffb(current, previous, target_box.center(), max_recursion_depth) &&
                       boxes_connected(current, target_box, tol);
        }
    }

    diagnostics[prefix + ".portal_corridor_ffb_calls"] += static_cast<double>(ffb_calls);
    diagnostics[prefix + ".portal_corridor_ffb_success"] += static_cast<double>(ffb_success);
    diagnostics[prefix + ".portal_corridor_ffb_fail"] += static_cast<double>(ffb_fail);
    diagnostics[prefix + ".portal_corridor_non_adjacent"] += static_cast<double>(non_adjacent);
    diagnostics[prefix + ".portal_corridor_non_certified"] += static_cast<double>(non_certified);
    diagnostics[prefix + ".portal_corridor_recursion_splits"] += static_cast<double>(recursion_splits);
    diagnostics[prefix + ".portal_corridor_internal_boxes"] +=
        static_cast<double>(internal_boxes.size());

    for (const auto& [key, value] : context.diagnostics().snapshot()) {
        diagnostics[prefix + ".portal_corridor_context." + key] += value;
    }

    return append_portal_if_ready();
}

int RBFPlanningForest::add_segment_edge_partition_first(
    int source_box_id,
    int target_box_id,
    std::vector<Eigen::VectorXd> waypoints,
    SegmentEdgeType type,
    int segment_resolution,
    SegmentEdgeValidation validation,
    bool strict_audit_required,
    int query_index,
    BuildProfile* profile,
    const char* diagnostic_prefix) {
    const bool use_partition_overlay = partition_native_mode();
    const int edge_id = use_partition_overlay
        ? append_segment_edge(segment_edges_,
                              source_box_id,
                              target_box_id,
                              std::move(waypoints),
                              type,
                              segment_resolution,
                              validation,
                              strict_audit_required,
                              query_index)
        : add_segment_edge(segment_edges_,
                           adjacency_,
                           source_box_id,
                           target_box_id,
                           std::move(waypoints),
                           type,
                           segment_resolution,
                           validation,
                           strict_audit_required,
                           query_index);
    if (edge_id < 0) {
        return -1;
    }
    if (use_partition_overlay) {
        sync_adaptive_partition_segment_edges(profile != nullptr ? profile : &last_build_,
                                              diagnostic_prefix != nullptr ? diagnostic_prefix
                                                                           : "partition_segment_edge");
    } else {
        invalidate_query_cache();
    }
    return edge_id;
}

void RBFPlanningForest::invalidate_query_cache() const {
    query_cache_dirty_ = true;
    partition_last_query_cache_.valid = false;
}

const QueryGraphCache& RBFPlanningForest::query_cache() const {
    if (partition_native_mode() &&
        env_int_or_default("RBF_PARTITION_NATIVE_ALLOW_GRAPH_QUERY_CACHE", 0) == 0) {
        throw std::logic_error(
            "partition_native mode forbids QueryGraphCache fallback; "
            "use AdaptiveGridPartition query/locate/connect APIs instead "
            "or set RBF_PARTITION_NATIVE_ALLOW_GRAPH_QUERY_CACHE=1 for legacy debugging");
    }
    if (query_cache_dirty_) {
        query_cache_ = build_query_graph_cache(boxes_, adjacency_, segment_edges_);
        query_cache_dirty_ = false;
    }
    return query_cache_;
}

int RBFPlanningForest::next_box_id() const {
    int next = 0;
    for (const auto& box : boxes_) {
        next = std::max(next, box.id + 1);
    }
    return next;
}

}  // namespace rbf
