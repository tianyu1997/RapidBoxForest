#include <SBF/connector.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace rbf {
namespace {

bool allow_connector_box_commit(BoxOracle& oracle,
                                FindFreeBoxResult& result,
                                BoxCommitPolicy policy,
                                StageContext& context) {
    if (policy == BoxCommitPolicy::AuditBeforeCommit &&
        (result.validation_detail.safety_status == BoxSafetyStatus::CertifiedFree ||
         result.validation_detail.safety_status == BoxSafetyStatus::ProvisionalFree)) {
        context.diagnostics().add_counter("connector.commit_audit_attempted");
        if (oracle.validate_intervals(result.intervals)) {
            result.validation_detail.safety_status = BoxSafetyStatus::CertifiedFree;
            result.validation_detail.strict_audit_required = false;
            context.diagnostics().add_counter("connector.commit_audit_success");
            return true;
        }
        context.diagnostics().add_counter("connector.commit_audit_failed");
        return false;
    }
    if (result.validation_detail.safety_status == BoxSafetyStatus::CertifiedFree) {
        return true;
    }
    if (result.validation_detail.safety_status != BoxSafetyStatus::ProvisionalFree) {
        context.diagnostics().add_counter("connector.commit_rejected_unknown_status");
        return false;
    }
    if (policy == BoxCommitPolicy::CommitCertifiedOnly) {
        context.diagnostics().add_counter("connector.commit_rejected_provisional");
        return false;
    }
    if (policy == BoxCommitPolicy::CommitProvisionalAllowed) {
        context.diagnostics().add_counter("connector.commit_provisional_allowed");
        return true;
    }
    return false;
}

double interval_point_gap(const Interval& interval, double value) {
    if (value < interval.lo) {
        return interval.lo - value;
    }
    if (value > interval.hi) {
        return value - interval.hi;
    }
    return 0.0;
}

double intervals_point_gap(const std::vector<Interval>& intervals,
                           const Eigen::Ref<const Eigen::VectorXd>& point) {
    if (point.size() != static_cast<int>(intervals.size())) {
        return std::numeric_limits<double>::infinity();
    }
    double gap = 0.0;
    for (int dim = 0; dim < point.size(); ++dim) {
        gap = std::max(gap, interval_point_gap(intervals[static_cast<std::size_t>(dim)], point[dim]));
    }
    return gap;
}

bool intervals_contain_point(const std::vector<Interval>& intervals,
                             const Eigen::Ref<const Eigen::VectorXd>& point,
                             double tolerance) {
    return intervals_point_gap(intervals, point) <= tolerance;
}

std::unordered_map<int, const BoxNode*> make_box_map(const std::vector<BoxNode>& boxes) {
    std::unordered_map<int, const BoxNode*> map;
    for (const auto& box : boxes) {
        map[box.id] = &box;
    }
    return map;
}

void record_chain_pave_boundary_ffb_failure(const FindFreeBoxResult& result,
                                            const Eigen::VectorXd& seed,
                                            BoxOracle& oracle,
                                            const ChainPaveConfig& config,
                                            StageContext& context) {
    context.diagnostics().add_counter(
        "connector.chain_pave_boundary_fail_code." + std::to_string(result.fail_code));
    if (result.seed_collision || result.fail_code == 1) {
        context.diagnostics().add_counter("connector.chain_pave_boundary_fail_seed_collision");
    }
    if (result.hit_unknown_depth_cap || result.hit_reserved_depth_cap ||
        result.fail_code == 2) {
        context.diagnostics().add_counter("connector.chain_pave_boundary_fail_depth_cap");
    }
    if (result.hit_unknown_depth_cap) {
        context.diagnostics().add_counter("connector.chain_pave_boundary_fail_unknown_depth_cap");
    }
    if (result.hit_reserved_depth_cap) {
        context.diagnostics().add_counter("connector.chain_pave_boundary_fail_reserved_depth_cap");
    }
    if (result.fail_code == 3) {
        context.diagnostics().add_counter("connector.chain_pave_boundary_fail_occupied");
    }
    if (result.deadline_reached || result.fail_code == 4) {
        context.diagnostics().add_counter("connector.chain_pave_boundary_fail_deadline");
    }
    if (result.fail_code == 5) {
        context.diagnostics().add_counter("connector.chain_pave_boundary_fail_out_of_domain");
    }
    if (result.fail_code == 6) {
        context.diagnostics().add_counter("connector.chain_pave_boundary_fail_split");
    }
    if (config.debug_boundary_failures != nullptr &&
        (result.hit_unknown_depth_cap || result.hit_reserved_depth_cap)) {
        DebugBoundaryFfbFailure failure;
        failure.seed.assign(seed.data(), seed.data() + seed.size());
        failure.intervals = result.intervals;
        failure.validation_detail = result.validation_detail;
        failure.node = result.node;
        failure.depth = result.node >= 0 ? oracle.depth(result.node) : -1;
        failure.changed_dim = result.changed_dim;
        failure.fail_code = result.fail_code;
        failure.hit_unknown_depth_cap = result.hit_unknown_depth_cap;
        failure.hit_reserved_depth_cap = result.hit_reserved_depth_cap;
        config.debug_boundary_failures->push_back(std::move(failure));
    }
}

double chain_pave_segment_exit_param(const BoxNode& box,
                                     const Eigen::VectorXd& a,
                                     const Eigen::VectorXd& b,
                                     double u0) {
    double u_hi = 1.0;
    const Eigen::VectorXd v = b - a;
    for (int d = 0; d < a.size(); ++d) {
        const double lo = box.joint_intervals[d].lo;
        const double hi = box.joint_intervals[d].hi;
        if (std::abs(v[d]) < 1e-15) {
            continue;
        }
        const double t1 = (lo - a[d]) / v[d];
        const double t2 = (hi - a[d]) / v[d];
        u_hi = std::min(u_hi, std::max(t1, t2));
    }
    return std::max(u0, std::min(1.0, u_hi));
}

Eigen::VectorXd chain_pave_boundary_seed_from_box(const BoxNode& box,
                                                  const Eigen::VectorXd& from,
                                                  const Eigen::VectorXd& target,
                                                  double adjacency_tolerance,
                                                  double gap_fill_min_step) {
    const double seg_len = (target - from).norm();
    if (seg_len < 1e-12) {
        return target;
    }
    const double u_exit = chain_pave_segment_exit_param(box, from, target, 0.0);
    // The FFB seed should step just outside the current box face; otherwise FFB
    // can certify a free box beyond a tiny gap and return a free but non-adjacent
    // result.
    const double face_epsilon =
        std::max(16.0 * std::max(0.0, adjacency_tolerance),
                 std::min(1e-6, std::max(gap_fill_min_step, 1e-12)));
    const double u_step =
        std::max(1e-12, face_epsilon / std::max(seg_len, 1e-12));
    const double u_seed = std::min(1.0, u_exit + u_step);
    return (from + u_seed * (target - from)).eval();
}

}  // namespace

int chain_pave_along_path(const std::vector<Eigen::VectorXd>& waypoint_path,
                          int anchor_box_id,
                          std::vector<BoxNode>& boxes,
                          BoxOracle& oracle,
                          AdjacencyGraph& graph,
                          int& next_box_id,
                          const ChainPaveConfig& config) {
    StageContext context = StageContext::serial();
    return chain_pave_along_path(waypoint_path, anchor_box_id, boxes, oracle, graph, next_box_id, context, config);
}

int chain_pave_along_path(const std::vector<Eigen::VectorXd>& waypoint_path,
                          int anchor_box_id,
                          std::vector<BoxNode>& boxes,
                          BoxOracle& oracle,
                          AdjacencyGraph& graph,
                          int& next_box_id,
                          StageContext& context,
                          const ChainPaveConfig& config) {
    if (waypoint_path.empty()) {
        return 0;
    }
    FindFreeBoxService ffb(oracle);
    auto find_at_depth = [&](const Eigen::VectorXd& seed,
                             StageContext& stage_context,
                             int depth) {
        FindFreeBoxOptions options = config.find_free_box;
        options.max_depth = std::max(1, depth);
        return ffb.find(seed, stage_context, options);
    };
    int current_box_id = anchor_box_id;
    int added = 0;

    std::unordered_map<int, std::size_t> box_index;
    std::unordered_map<std::int64_t, int> tree_owner;
    box_index.reserve(boxes.size() + 16);
    tree_owner.reserve(boxes.size() + 16);
    auto index_box = [&](std::size_t index) {
        const BoxNode& box = boxes[index];
        box_index[box.id] = index;
        if (tree_owner.find(box.tree_id) == tree_owner.end()) {
            tree_owner[box.tree_id] = box.id;
        }
    };
    for (std::size_t i = 0; i < boxes.size(); ++i) {
        index_box(i);
    }
    auto box_by_id = [&](int id) -> BoxNode* {
        auto it = box_index.find(id);
        return it == box_index.end() ? nullptr : &boxes[it->second];
    };
    auto append_graph_edge = [&](int lhs, int rhs) {
        if (lhs == rhs) {
            return;
        }
        auto append_one = [&](int from, int to) {
            auto& list = graph[from];
            if (std::find(list.begin(), list.end(), to) == list.end()) {
                list.push_back(to);
            }
        };
        append_one(lhs, rhs);
        append_one(rhs, lhs);
    };
    auto find_existing_cover = [&](const Eigen::VectorXd& p,
                                   int preferred_id = -1) -> int {
        if (preferred_id >= 0) {
            if (BoxNode* preferred = box_by_id(preferred_id)) {
                if (preferred->contains(p, config.adjacency_tolerance)) {
                    return preferred_id;
                }
            }
        }
        for (const auto& box : boxes) {
            if (box.contains(p, config.adjacency_tolerance)) {
                return box.id;
            }
        }
        return -1;
    };
    auto find_box_owning_node_covering = [&](int node, const Eigen::VectorXd& p) -> int {
        for (const auto& box : boxes) {
            if (box.tree_id == node && box.contains(p, config.adjacency_tolerance)) {
                return box.id;
            }
        }
        return -1;
    };
    // Commit a certified-free FFB result as a new box parented to `parent_id`.
    // Returns the new box id on success, or -1 if the result cannot be committed
    // (already-committed node, commit-policy rejection, or not adjacent to the
    // parent). Connector boxes are never allowed to appear as isolated coverage
    // boxes: every newly committed box must intersect the existing box graph.
    auto commit_box = [&](FindFreeBoxResult& result,
                          const Eigen::VectorXd& seed,
                          int parent_id,
                          bool allow_duplicate_node = false) -> int {
        BoxNode* parent_box = box_by_id(parent_id);
        if (parent_box == nullptr) {
            return -1;
        }
        if (result.node != kInvalidOracleNodeId &&
            result.node == parent_box->tree_id) {
            return -1;
        }
        // Normally a canonical tree node hosts a single committed box. As a
        // coverage last resort we allow a second box on an already-committed node:
        // FFB grows its certified slab from the *query seed*, so two samples that
        // share a canonical cell can be certified by different thin slabs, and the
        // first-committed slab need not contain a later sample. Permitting a
        // duplicate-node box (without clobbering the oracle's canonical owner) lets
        // that later free sample still be covered.
        const bool node_already_owned = find_box_owning_node_covering(result.node, seed) >= 0;
        if (!allow_duplicate_node && node_already_owned) {
            return -1;
        }
        if (!allow_connector_box_commit(oracle, result, config.commit_policy, context)) {
            return -1;
        }
        BoxNode box;
        box.joint_intervals = result.intervals;
        box.seed_config = seed;
        box.tree_id = result.node;
        box.parent_box_id = parent_id;
        box.root_id = parent_box->root_id;
        box.safety_status = result.validation_detail.safety_status;
        box.strict_audit_required = result.validation_detail.strict_audit_required;
        box.compute_volume();
        const bool adjacent = boxes_connected(*parent_box, box, config.adjacency_tolerance);
        if (!adjacent) {
            return -1;
        }
        box.id = next_box_id++;
        const int new_id = box.id;
        // Keep the oracle's canonical node->box reservation pointing at the first
        // owner; a duplicate-node coverage box is bookkeeping-only and must not
        // clobber it (the box still lives in `boxes` and the adjacency graph).
        if (!node_already_owned) {
            oracle.reserve_node(result.node, new_id);
        }
        graph[new_id] = {};
        if (adjacent) {
            graph[parent_id].push_back(new_id);
            graph[new_id].push_back(parent_id);
        }
        const std::size_t new_index = boxes.size();
        boxes.push_back(std::move(box));
        index_box(new_index);
        added += 1;
        return new_id;
    };

    // Find the first committed box indexed by canonical tree node `node`, or -1.
    auto find_box_owning_node = [&](int node) -> int {
        auto it = tree_owner.find(node);
        return it == tree_owner.end() ? -1 : it->second;
    };

    // Commit a duplicate-node coverage box from a *reserved-cap* FFB result. When
    // FFB caps out on a canonical leaf that an earlier box in this chain already
    // owns, `result.found` is false but `result.intervals` is that same certified-
    // free leaf cell mapped to THIS sample's symmetry sector (so it contains the
    // sample). The first box only covered a *different* sector of the leaf, so the
    // sample is still uncovered. The leaf was validated free when first committed,
    // hence every symmetry image of it is free too -- we copy the owner box's
    // certification and reuse the sector-mapped intervals. Returns the new box id,
    // or -1 if it cannot be committed (no owner / commit-policy rejection).
    auto commit_reserved_cap_box = [&](const FindFreeBoxResult& result,
                                       const Eigen::VectorXd& seed,
                                       int parent_id) -> int {
        const int owner = find_box_owning_node(result.node);
        if (owner < 0) {
            return -1;
        }
        BoxNode* parent_box = box_by_id(parent_id);
        BoxNode* owner_box = box_by_id(owner);
        if (parent_box == nullptr || owner_box == nullptr) {
            return -1;
        }
        BoxNode box;
        box.joint_intervals = result.intervals;
        box.seed_config = seed;
        box.tree_id = result.node;
        box.parent_box_id = parent_id;
        box.root_id = parent_box->root_id;
        // The certification belongs to the leaf, not the seed, so inherit it from
        // the box that already owns the canonical node.
        box.safety_status = owner_box->safety_status;
        box.strict_audit_required = owner_box->strict_audit_required;
        box.compute_volume();
        const bool adjacent =
            boxes_connected(*parent_box, box, config.adjacency_tolerance);
        if (!adjacent) {
            return -1;
        }
        box.id = next_box_id++;
        const int new_id = box.id;
        // Do not clobber the oracle's canonical node->box reservation: this is a
        // bookkeeping-only duplicate that only exists to cover the sample.
        graph[new_id] = {};
        if (adjacent) {
            graph[parent_id].push_back(new_id);
            graph[new_id].push_back(parent_id);
        }
        const std::size_t new_index = boxes.size();
        boxes.push_back(std::move(box));
        index_box(new_index);
        added += 1;
        return new_id;
    };

    // Cover the C-space segment [from_pt -> to_pt] with connected boxes, extending
    // the chain from box `from_id`. First try to commit a box certified at to_pt
    // directly; if that box exists but is not adjacent to the current chain box (a
    // residual gap), bisect the segment and recurse. Crucially the segment lies on
    // the connector's collision-free bridge polyline, so every midpoint is itself
    // collision-free and certifiable -- the recursion fills the gap with real
    // boxes instead of cutting a corner through a C-space obstacle. Returns the id
    // of the furthest box reached (== from_id when no progress was made). With
    // budget == 0 this reduces to a single direct-commit attempt at to_pt.
    std::function<int(int, const Eigen::VectorXd&, const Eigen::VectorXd&, int)> cover =
        [&](int from_id,
            const Eigen::VectorXd& from_pt,
            const Eigen::VectorXd& to_pt,
            int budget) -> int {
        if (added >= config.max_chain || context.should_stop()) {
            return from_id;
        }
        // Obtain a box covering to_pt: commit a fresh certified box, or reuse an
        // existing box that already owns that canonical cell (so the chain can
        // follow the bridge through previously paved regions).
        int to_id = from_id;
        bool saw_certifiable = false;
        bool rejection_counted = false;
        bool bridge_to_existing_cover = false;
        const int added_before = added;
        const int existing_cover = find_existing_cover(to_pt, from_id);
        if (existing_cover >= 0) {
            if (existing_cover == from_id) {
                context.diagnostics().add_counter("connector.chain_pave_existing_cover_current");
                to_id = from_id;
            } else {
                BoxNode* from_box = box_by_id(from_id);
                BoxNode* cover_box = box_by_id(existing_cover);
                if (from_box != nullptr && cover_box != nullptr &&
                    boxes_connected(*from_box, *cover_box, config.adjacency_tolerance)) {
                    append_graph_edge(from_id, existing_cover);
                    context.diagnostics().add_counter("connector.chain_pave_existing_cover_adjacent");
                    to_id = existing_cover;
                } else {
                    context.diagnostics().add_counter("connector.chain_pave_existing_cover_non_adjacent");
                    bridge_to_existing_cover = true;
                }
            }
        }
        auto consume_result = [&](FindFreeBoxResult& result) {
            saw_certifiable = true;
            const int committed = commit_box(result, to_pt, from_id);
            if (committed >= 0) {
                to_id = committed;
                if (added > added_before) {
                    context.diagnostics().add_counter("connector.chain_pave_boundary_commits");
                }
                return true;
            } else {
                const int owner = find_box_owning_node_covering(result.node, to_pt);
                if (owner >= 0) {
                    BoxNode* owner_box = box_by_id(owner);
                    BoxNode* from_box = box_by_id(from_id);
                    if (owner_box != nullptr && from_box != nullptr &&
                        boxes_connected(*from_box, *owner_box, config.adjacency_tolerance)) {
                        to_id = owner;
                        return true;
                    }
                }
                if (!result.found &&
                    result.hit_reserved_depth_cap &&
                    intervals_contain_point(result.intervals,
                                            to_pt,
                                            config.adjacency_tolerance)) {
                    const int duplicate = commit_reserved_cap_box(result,
                                                                  to_pt,
                                                                  from_id);
                    if (duplicate >= 0) {
                        to_id = duplicate;
                        if (added > added_before) {
                            context.diagnostics().add_counter("connector.chain_pave_boundary_commits");
                        }
                        return true;
                    }
                }
            }
            return false;
        };
        if (existing_cover < 0) {
            context.diagnostics().add_counter("connector.chain_pave_boundary_ffb_calls");
            auto result = find_at_depth(to_pt, context, config.find_free_box.max_depth);
            if (result.seed_collision) {
                record_chain_pave_boundary_ffb_failure(result, to_pt, oracle, config, context);
                context.diagnostics().add_counter("connector.chain_pave_boundary_reject_not_free");
                rejection_counted = true;
            } else if (result.found ||
                       (result.hit_reserved_depth_cap &&
                        intervals_contain_point(result.intervals,
                                                to_pt,
                                                config.adjacency_tolerance))) {
                consume_result(result);
            } else {
                record_chain_pave_boundary_ffb_failure(result, to_pt, oracle, config, context);
            }
        }
        if (bridge_to_existing_cover && budget <= 0 && config.fill_gaps) {
            budget = std::max(1, config.max_gap_fill_depth);
            context.diagnostics().add_counter("connector.chain_pave_existing_cover_bridge_attempts");
        }
        if (to_id == from_id && !rejection_counted && !bridge_to_existing_cover) {
            context.diagnostics().add_counter(
                saw_certifiable
                    ? "connector.chain_pave_boundary_reject_non_adjacent"
                    : "connector.chain_pave_boundary_reject_not_free");
        }
        if (budget <= 0) {
            return to_id;
        }
        // A committed box is a convex axis-aligned box: if a SINGLE box contains
        // BOTH endpoints, the whole segment between them is inside that box and is
        // therefore fully covered. This is the correct termination test -- checking
        // only the midpoint (as a weaker variant did) leaves the quarter/three-
        // quarter points of the segment uncovered, capping coverage well below
        // 100%. Recurse by bisection until each leaf sub-segment has both endpoints
        // inside one box (every midpoint lies on the collision-free bridge polyline
        // and is itself certifiable, so the recursion terminates with real boxes).
        {
            auto map = make_box_map(boxes);
            auto segment_in_one_box = [&](int id) {
                auto it = map.find(id);
                return it != map.end() && it->second->contains(from_pt) &&
                       it->second->contains(to_pt);
            };
            if (segment_in_one_box(from_id) || segment_in_one_box(to_id)) {
                return to_id;
            }
        }
        const Eigen::VectorXd mid = 0.5 * (from_pt + to_pt);
        if ((mid - to_pt).norm() < config.gap_fill_min_step ||
            (mid - from_pt).norm() < config.gap_fill_min_step) {
            return to_id;
        }
        const int via = cover(from_id, from_pt, mid, budget - 1);
        if (added >= config.max_chain || context.should_stop()) {
            return via;
        }
        return cover(via, mid, to_pt, budget - 1);
    };

    auto boundary_seed_candidates = [&](const BoxNode& box,
                                        const Eigen::VectorXd& from,
                                        const Eigen::VectorXd& target,
                                        double requested_step)
        -> std::vector<Eigen::VectorXd> {
        std::vector<Eigen::VectorXd> seeds;
        const Eigen::VectorXd forward_seed =
            chain_pave_boundary_seed_from_box(box,
                                              from,
                                              target,
                                              config.adjacency_tolerance,
                                              config.gap_fill_min_step);
        seeds.push_back(forward_seed);
        if (from.size() != target.size() || box.n_dims() != from.size()) {
            return seeds;
        }
        const Eigen::VectorXd delta = target - from;
        const double distance = delta.norm();
        if (distance < 1e-12) {
            return seeds;
        }

        struct LateralDim {
            int dim = -1;
            double score = 0.0;
            double width = 0.0;
        };
        std::vector<LateralDim> dims;
        dims.reserve(static_cast<std::size_t>(from.size()));
        for (int dim = 0; dim < from.size(); ++dim) {
            const auto& interval = box.joint_intervals[static_cast<std::size_t>(dim)];
            const double width = interval.width();
            if (width <= 2.0 * config.adjacency_tolerance) {
                continue;
            }
            const double alignment = std::abs(delta[dim]) / distance;
            dims.push_back({dim, width * (1.0 - alignment), width});
        }
        std::sort(dims.begin(), dims.end(), [](const LateralDim& lhs,
                                               const LateralDim& rhs) {
            return lhs.score > rhs.score;
        });

        const double base_radius =
            std::max(config.gap_fill_min_step,
                     0.35 * std::max(requested_step, config.gap_fill_min_step));
        const int max_lateral_dims = std::min<int>(2, static_cast<int>(dims.size()));
        for (int rank = 0; rank < max_lateral_dims; ++rank) {
            const int dim = dims[static_cast<std::size_t>(rank)].dim;
            const auto& interval = box.joint_intervals[static_cast<std::size_t>(dim)];
            const double radius =
                std::min(base_radius,
                         std::max(config.gap_fill_min_step,
                                  0.45 * dims[static_cast<std::size_t>(rank)].width));
            for (const double sign : {1.0, -1.0}) {
                Eigen::VectorXd candidate = seeds.front();
                candidate[dim] = std::clamp(candidate[dim] + sign * radius,
                                            interval.lo + config.adjacency_tolerance,
                                            interval.hi - config.adjacency_tolerance);
                if ((candidate - from).norm() >=
                    std::max(config.gap_fill_min_step, 1e-6) * 0.25 &&
                    (candidate - seeds.front()).norm() > 1e-12) {
                    seeds.push_back(std::move(candidate));
                }
            }
        }
        return seeds;
    };

    auto closest_point_in_box = [&](const BoxNode& box,
                                    const Eigen::VectorXd& point) -> Eigen::VectorXd {
        Eigen::VectorXd out(point.size());
        for (int dim = 0; dim < point.size(); ++dim) {
            const auto& interval = box.joint_intervals[static_cast<std::size_t>(dim)];
            out[dim] = std::clamp(point[dim], interval.lo, interval.hi);
        }
        return out;
    };
    std::unordered_set<std::uint64_t> failed_boundary_seed_keys;
    auto boundary_seed_key = [&](int parent_id,
                                 std::size_t segment_index,
                                 const BoxNode& parent_box,
                                 const Eigen::VectorXd& cursor,
                                 const Eigen::VectorXd& seed) {
        int face_dim = 0;
        int side = 1;
        double best_gap = -1.0;
        for (int dim = 0; dim < seed.size(); ++dim) {
            const auto& interval = parent_box.joint_intervals[static_cast<std::size_t>(dim)];
            double gap = 0.0;
            int candidate_side = seed[dim] >= cursor[dim] ? 1 : 0;
            if (seed[dim] > interval.hi + config.adjacency_tolerance) {
                gap = seed[dim] - interval.hi;
                candidate_side = 1;
            } else if (seed[dim] < interval.lo - config.adjacency_tolerance) {
                gap = interval.lo - seed[dim];
                candidate_side = 0;
            } else {
                gap = std::abs(seed[dim] - cursor[dim]) * 1e-3;
            }
            if (gap > best_gap) {
                best_gap = gap;
                face_dim = dim;
                side = candidate_side;
            }
        }
        auto mix = [](std::uint64_t value) {
            value ^= value >> 33;
            value *= 0xff51afd7ed558ccdULL;
            value ^= value >> 33;
            value *= 0xc4ceb9fe1a85ec53ULL;
            value ^= value >> 33;
            return value;
        };
        std::uint64_t key = mix(static_cast<std::uint64_t>(static_cast<std::uint32_t>(parent_id)));
        key ^= mix(static_cast<std::uint64_t>(segment_index + 0x9e3779b97f4a7c15ULL));
        key ^= mix(static_cast<std::uint64_t>((face_dim & 0xff) |
                                             ((side & 0x1) << 8)));
        const double bucket =
            std::max(1e-6, std::max(config.gap_fill_min_step, 1e-9) * 0.25);
        for (int dim = 0; dim < seed.size(); ++dim) {
            const auto quantized =
                static_cast<std::int64_t>(std::llround(seed[dim] / bucket));
            key ^= mix(static_cast<std::uint64_t>(
                quantized + 0x9e3779b97f4a7c15LL +
                static_cast<std::int64_t>(dim) * 0x100000001b3LL));
        }
        return key;
    };

    if (waypoint_path.size() >= 2) {
        int connected_segments = 0;
        int connected_steps = 0;
        int connected_reach_failures = 0;
        int connected_target_hits = 0;
        for (std::size_t seg = 1;
             seg < waypoint_path.size() && added < config.max_chain &&
             !context.should_stop();
             ++seg) {
            const Eigen::VectorXd& a = waypoint_path[seg - 1];
            const Eigen::VectorXd& b = waypoint_path[seg];
            const double seg_len = (b - a).norm();
            if (seg_len < 1e-12) {
                continue;
            }
            connected_segments += 1;
            BoxNode* current_box = box_by_id(current_box_id);
            if (current_box == nullptr) {
                break;
            }
            Eigen::VectorXd cursor =
                current_box->contains(a) ? a : closest_point_in_box(*current_box, a);
            const double front_step = std::max(
                config.gap_fill_sample_step > 0.0 ? config.gap_fill_sample_step
                                                  : config.gap_fill_min_step,
                1e-6);
            int guard = 0;
            const int guard_max = std::max(
                1,
                static_cast<int>(std::ceil(seg_len / front_step)) + 2);
            while (added < config.max_chain && !context.should_stop() &&
                   guard++ < guard_max) {
                current_box = box_by_id(current_box_id);
                if (current_box == nullptr) {
                    break;
                }
                if (current_box->contains(b)) {
                    connected_target_hits += 1;
                    break;
                }
                connected_steps += 1;
                if (!current_box->contains(cursor)) {
                    cursor = closest_point_in_box(*current_box, cursor);
                }
                int reached = current_box_id;
                double attempt_step = front_step;
                for (int attempt = 0; attempt < 8 && reached == current_box_id;
                     ++attempt) {
                    const auto seeds =
                        boundary_seed_candidates(*current_box, cursor, b, attempt_step);
                    const double min_seed_motion =
                        std::max(4.0 * std::max(0.0, config.adjacency_tolerance), 1e-12);
                    if (seeds.empty() ||
                        (seeds.front() - cursor).norm() < min_seed_motion) {
                        break;
                    }
                    for (std::size_t seed_rank = 0; seed_rank < seeds.size(); ++seed_rank) {
                        const auto& seed = seeds[seed_rank];
                        const auto key = boundary_seed_key(current_box_id,
                                                           seg,
                                                           *current_box,
                                                           cursor,
                                                           seed);
                        if (failed_boundary_seed_keys.find(key) != failed_boundary_seed_keys.end()) {
                            context.diagnostics().add_counter("connector.chain_pave_boundary_skip_failed_seed");
                            continue;
                        }
                        reached = cover(current_box_id, cursor, seed, 0);
                        if (reached == current_box_id) {
                            failed_boundary_seed_keys.insert(key);
                            context.diagnostics().add_counter("connector.chain_pave_boundary_failed_seed_memoized");
                        }
                        if (reached != current_box_id ||
                            added >= config.max_chain || context.should_stop()) {
                            break;
                        }
                    }
                    attempt_step *= 0.5;
                }
                if (reached == current_box_id) {
                    connected_reach_failures += 1;
                    context.diagnostics().add_counter("connector.chain_pave_boundary_stall");
                    break;
                }
                current_box_id = reached;
                if (BoxNode* reached_box = box_by_id(current_box_id)) {
                    cursor = closest_point_in_box(*reached_box, b);
                }
            }
        }
        context.diagnostics().set_value("connector.chain_pave_connected_added",
                                        static_cast<double>(added));
        context.diagnostics().set_value("connector.chain_pave_connected_segments",
                                        static_cast<double>(connected_segments));
        context.diagnostics().set_value("connector.chain_pave_connected_steps",
                                        static_cast<double>(connected_steps));
        context.diagnostics().set_value("connector.chain_pave_connected_reach_failures",
                                        static_cast<double>(connected_reach_failures));
        context.diagnostics().set_value("connector.chain_pave_connected_target_hits",
                                        static_cast<double>(connected_target_hits));
        context.diagnostics().set_value("connector.chain_pave_boundary_target_hits",
                                        static_cast<double>(connected_target_hits));
        if (added >= config.max_chain) {
            context.diagnostics().add_counter("connector.chain_pave_connected_max_chain_hits");
        }
        return added;
    }

    return added;
}

}  // namespace rbf
