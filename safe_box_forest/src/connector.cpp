#include <SBF/connector.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <random>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace rbf {
namespace {

int segment_resolution_for_step(const Eigen::Ref<const Eigen::VectorXd>& a,
                                const Eigen::Ref<const Eigen::VectorXd>& b,
                                int base_resolution,
                                double segment_step) {
    int resolution = std::max(1, base_resolution);
    if (segment_step > 0.0) {
        resolution = std::max(resolution, static_cast<int>(std::ceil((b - a).norm() / segment_step)));
    }
    return resolution;
}

bool check_segment_with_step(const CollisionChecker& checker,
                             const Eigen::Ref<const Eigen::VectorXd>& a,
                             const Eigen::Ref<const Eigen::VectorXd>& b,
                             int base_resolution,
                             double segment_step) {
    return checker.check_segment(a, b, segment_resolution_for_step(a, b, base_resolution, segment_step));
}

double waypoint_path_length(const std::vector<Eigen::VectorXd>& path) {
    double total = 0.0;
    for (std::size_t index = 1; index < path.size(); ++index) {
        total += (path[index] - path[index - 1]).norm();
    }
    return total;
}

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

RRTConnectConfig with_query_root_hull_domain(const RRTConnectConfig& config,
                                             BoxOracle& oracle,
                                             const Eigen::Ref<const Eigen::VectorXd>& start,
                                             const Eigen::Ref<const Eigen::VectorXd>& goal) {
    RRTConnectConfig out = config;
    auto lhs = oracle.planning_intervals();
    auto rhs = oracle.planning_intervals();
    (void)start;
    (void)goal;
    if (lhs.size() == rhs.size()) {
        for (std::size_t i = 0; i < lhs.size(); ++i) {
            lhs[i] = lhs[i].hull(rhs[i]);
        }
    }
    out.domain_intervals = std::move(lhs);
    return out;
}

struct RRTTree {
    std::vector<Eigen::VectorXd> nodes;
    std::vector<int> parents;
};

struct RRTConnectStats {
    bool success = false;
    bool direct = false;
    bool cancelled = false;
    bool timed_out = false;
    int iterations = 0;
    int merge_iteration = -1;
    int forward_tree_size = 0;
    int backward_tree_size = 0;
    int raw_waypoints = 0;
    int shortcut_waypoints = 0;
    double elapsed_ms = 0.0;
};

struct RRTConnectOutcome {
    std::vector<Eigen::VectorXd> path;
    RRTConnectStats stats;
};

int nearest_node(const RRTTree& tree, const Eigen::VectorXd& q) {
    int best = -1;
    double best_dist = std::numeric_limits<double>::max();
    for (int i = 0; i < static_cast<int>(tree.nodes.size()); ++i) {
        const double dist = (tree.nodes[static_cast<std::size_t>(i)] - q).squaredNorm();
        if (dist < best_dist) {
            best_dist = dist;
            best = i;
        }
    }
    return best;
}

bool steer(const Eigen::VectorXd& from, const Eigen::VectorXd& to, double step_size, Eigen::VectorXd& out) {
    const Eigen::VectorXd diff = to - from;
    const double norm = diff.norm();
    if (norm < 1e-10) {
        return false;
    }
    out = norm <= step_size ? to : from + (diff / norm) * step_size;
    return true;
}

bool graph_has_path(const AdjacencyGraph& graph, int source_box_id, int target_box_id) {
    if (source_box_id == target_box_id) {
        return true;
    }
    std::vector<int> stack{source_box_id};
    std::unordered_set<int> visited;
    visited.insert(source_box_id);
    while (!stack.empty()) {
        const int current = stack.back();
        stack.pop_back();
        const auto it = graph.find(current);
        if (it == graph.end()) {
            continue;
        }
        for (const int next : it->second) {
            if (next == target_box_id) {
                return true;
            }
            if (visited.insert(next).second) {
                stack.push_back(next);
            }
        }
    }
    return false;
}

void append_graph_edge_unique(AdjacencyGraph& graph, int lhs, int rhs) {
    if (lhs < 0 || rhs < 0 || lhs == rhs) {
        return;
    }
    auto append_one = [&](int from, int to) {
        auto& neighbors = graph[from];
        if (std::find(neighbors.begin(), neighbors.end(), to) == neighbors.end()) {
            neighbors.push_back(to);
        }
    };
    append_one(lhs, rhs);
    append_one(rhs, lhs);
}

int connect_new_boxes_to_island(std::vector<BoxNode>& boxes,
                                AdjacencyGraph& graph,
                                int first_new_box_id,
                                int next_box_id,
                                const std::vector<int>& target_island,
                                double tolerance) {
    if (first_new_box_id >= next_box_id || target_island.empty()) {
        return 0;
    }
    std::unordered_map<int, const BoxNode*> map;
    map.reserve(boxes.size());
    for (const auto& box : boxes) {
        map[box.id] = &box;
    }
    int added_edges = 0;
    for (int new_id = first_new_box_id; new_id < next_box_id; ++new_id) {
        const auto new_it = map.find(new_id);
        if (new_it == map.end()) {
            continue;
        }
        for (int target_id : target_island) {
            const auto target_it = map.find(target_id);
            if (target_it == map.end()) {
                continue;
            }
            if (boxes_connected(*new_it->second, *target_it->second, tolerance)) {
                const std::size_t before = graph[new_id].size();
                append_graph_edge_unique(graph, new_id, target_id);
                if (graph[new_id].size() > before) {
                    added_edges += 1;
                }
            }
        }
    }
    return added_edges;
}

int add_rrt_node(RRTTree& tree, const Eigen::VectorXd& q, int parent) {
    const int id = static_cast<int>(tree.nodes.size());
    tree.nodes.push_back(q);
    tree.parents.push_back(parent);
    return id;
}

int extend_tree(RRTTree& tree,
                const Eigen::VectorXd& target,
                const Eigen::VectorXd& lo,
                const Eigen::VectorXd& hi,
                const CollisionChecker& checker,
                const RRTConnectConfig& config) {
    const int near = nearest_node(tree, target);
    if (near < 0) {
        return -1;
    }
    Eigen::VectorXd q_new;
    if (!steer(tree.nodes[static_cast<std::size_t>(near)], target, config.step_size, q_new)) {
        return -1;
    }
    for (int dim = 0; dim < q_new.size(); ++dim) {
        q_new[dim] = std::clamp(q_new[dim], lo[dim], hi[dim]);
    }
    if (checker.check_config(q_new) ||
        check_segment_with_step(checker,
                                tree.nodes[static_cast<std::size_t>(near)],
                                q_new,
                                config.segment_resolution,
                                config.segment_step)) {
        return -1;
    }
    return add_rrt_node(tree, q_new, near);
}

int max_connect_steps(const Eigen::VectorXd& lo, const Eigen::VectorXd& hi, double step_size) {
    const double span = (hi - lo).norm();
    const double safe_step = std::max(step_size, 1e-9);
    return std::clamp(static_cast<int>(std::ceil(span / safe_step)) + 2, 1, 1000);
}

int connect_greedy(RRTTree& tree,
                   const Eigen::VectorXd& target,
                   const Eigen::VectorXd& lo,
                   const Eigen::VectorXd& hi,
                   const CollisionChecker& checker,
                   const RRTConnectConfig& config) {
    int current = nearest_node(tree, target);
    const int connect_steps = max_connect_steps(lo, hi, config.step_size);
    for (int step = 0; step < connect_steps && current >= 0; ++step) {
        if ((tree.nodes[static_cast<std::size_t>(current)] - target).norm() < 1e-8) {
            return current;
        }
        Eigen::VectorXd q_new;
        if (!steer(tree.nodes[static_cast<std::size_t>(current)], target, config.step_size, q_new)) {
            return -1;
        }
        for (int dim = 0; dim < q_new.size(); ++dim) {
            q_new[dim] = std::clamp(q_new[dim], lo[dim], hi[dim]);
        }
        if (checker.check_config(q_new) ||
            check_segment_with_step(checker,
                                    tree.nodes[static_cast<std::size_t>(current)],
                                    q_new,
                                    config.segment_resolution,
                                    config.segment_step)) {
            return -1;
        }
        const int id = add_rrt_node(tree, q_new, current);
        current = id;
        const double remaining = (q_new - target).norm();
        if (remaining <= 1e-8) {
            return current;
        }
        if (remaining <= config.step_size * 0.5 &&
            !check_segment_with_step(checker, q_new, target, config.segment_resolution, config.segment_step)) {
            return add_rrt_node(tree, target, current);
        }
    }
    return -1;
}

std::vector<Eigen::VectorXd> extract_rrt_path(const RRTTree& tree, int index) {
    std::vector<Eigen::VectorXd> path;
    while (index >= 0) {
        path.push_back(tree.nodes[static_cast<std::size_t>(index)]);
        index = tree.parents[static_cast<std::size_t>(index)];
    }
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<Eigen::VectorXd> merge_birrt_paths(const RRTTree& start_tree,
                                               int start_tree_index,
                                               const RRTTree& goal_tree,
                                               int goal_tree_index) {
    std::vector<Eigen::VectorXd> start_path = extract_rrt_path(start_tree, start_tree_index);
    std::vector<Eigen::VectorXd> goal_path = extract_rrt_path(goal_tree, goal_tree_index);
    std::reverse(goal_path.begin(), goal_path.end());
    if (!start_path.empty() && !goal_path.empty() && (start_path.back() - goal_path.front()).norm() <= 1e-8) {
        start_path.insert(start_path.end(), goal_path.begin() + 1, goal_path.end());
    } else {
        start_path.insert(start_path.end(), goal_path.begin(), goal_path.end());
    }
    return start_path;
}

std::vector<Eigen::VectorXd> shortcut_collision_free_path(const std::vector<Eigen::VectorXd>& path,
                                                          const CollisionChecker& checker,
                                                          int segment_resolution,
                                                          double segment_step = 0.0) {
    if (path.size() <= 2) {
        return path;
    }
    std::vector<Eigen::VectorXd> out;
    out.push_back(path.front());
    std::size_t current = 0;
    while (current + 1 < path.size()) {
        std::size_t next = path.size() - 1;
        while (next > current + 1 &&
               check_segment_with_step(checker, path[current], path[next], segment_resolution, segment_step)) {
            --next;
        }
        out.push_back(path[next]);
        current = next;
    }
    return out;
}

std::vector<Eigen::VectorXd> densify_path_by_step(const std::vector<Eigen::VectorXd>& path,
                                                  double max_step) {
    if (path.size() <= 1 || max_step <= 0.0) {
        return path;
    }
    std::vector<Eigen::VectorXd> out;
    out.push_back(path.front());
    for (std::size_t index = 1; index < path.size(); ++index) {
        const Eigen::VectorXd& a = path[index - 1];
        const Eigen::VectorXd& b = path[index];
        const double length = (b - a).norm();
        const int n = std::max(1, static_cast<int>(std::ceil(length / max_step)));
        for (int sample = 1; sample <= n; ++sample) {
            const double u = static_cast<double>(sample) / static_cast<double>(n);
            Eigen::VectorXd point = a + u * (b - a);
            if ((out.back() - point).norm() > 1e-12) {
                out.push_back(std::move(point));
            }
        }
    }
    return out;
}

void finish_rrt_stats(RRTConnectOutcome& outcome,
                      const RRTTree& start_tree,
                      const RRTTree& goal_tree,
                      std::chrono::steady_clock::time_point t0) {
    outcome.stats.forward_tree_size = static_cast<int>(start_tree.nodes.size());
    outcome.stats.backward_tree_size = static_cast<int>(goal_tree.nodes.size());
    outcome.stats.elapsed_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    outcome.stats.success = !outcome.path.empty();
}

void record_birrt_stats(StageDiagnostics& diagnostics, const RRTConnectStats& stats) {
    diagnostics.record_timing("connector.birrt", stats.elapsed_ms);
    diagnostics.set_value("connector.birrt_last_iterations", static_cast<double>(stats.iterations));
    diagnostics.set_value("connector.birrt_last_forward_tree_size", static_cast<double>(stats.forward_tree_size));
    diagnostics.set_value("connector.birrt_last_backward_tree_size", static_cast<double>(stats.backward_tree_size));
    diagnostics.set_value("connector.birrt_last_raw_waypoints", static_cast<double>(stats.raw_waypoints));
    diagnostics.set_value("connector.birrt_last_shortcut_waypoints", static_cast<double>(stats.shortcut_waypoints));
    if (stats.merge_iteration >= 0) {
        diagnostics.set_value("connector.birrt_last_merge_iteration", static_cast<double>(stats.merge_iteration));
    }
    if (stats.success) {
        diagnostics.add_counter("connector.birrt_successes");
        if (stats.direct) {
            diagnostics.add_counter("connector.birrt_direct_successes");
        }
    } else {
        diagnostics.add_counter("connector.birrt_failures");
        if (stats.cancelled) {
            diagnostics.add_counter("connector.birrt_cancelled");
        }
        if (stats.timed_out) {
            diagnostics.add_counter("connector.birrt_timeouts");
        }
    }
}

std::unordered_map<int, const BoxNode*> make_box_map(const std::vector<BoxNode>& boxes) {
    std::unordered_map<int, const BoxNode*> map;
    for (const auto& box : boxes) {
        map[box.id] = &box;
    }
    return map;
}

double interval_max_gap(const BoxNode& lhs, const BoxNode& rhs) {
    double max_gap = 0.0;
    for (int dim = 0; dim < lhs.n_dims(); ++dim) {
        const auto& a = lhs.joint_intervals[static_cast<std::size_t>(dim)];
        const auto& b = rhs.joint_intervals[static_cast<std::size_t>(dim)];
        max_gap = std::max(max_gap, std::max({a.lo - b.hi, b.lo - a.hi, 0.0}));
    }
    return max_gap;
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

std::vector<Eigen::VectorXd> closest_box_point_segment(const BoxNode& source,
                                                       const BoxNode& target,
                                                       const CollisionChecker& checker,
                                                       int segment_resolution,
                                                       double segment_step = 0.0) {
    if (source.n_dims() != target.n_dims()) {
        return {};
    }
    Eigen::VectorXd lhs(source.n_dims());
    Eigen::VectorXd rhs(source.n_dims());
    for (int dim = 0; dim < source.n_dims(); ++dim) {
        const auto& a = source.joint_intervals[static_cast<std::size_t>(dim)];
        const auto& b = target.joint_intervals[static_cast<std::size_t>(dim)];
        if (a.hi < b.lo) {
            lhs[dim] = a.hi;
            rhs[dim] = b.lo;
        } else if (b.hi < a.lo) {
            lhs[dim] = a.lo;
            rhs[dim] = b.hi;
        } else {
            const double lo = std::max(a.lo, b.lo);
            const double hi = std::min(a.hi, b.hi);
            lhs[dim] = rhs[dim] = 0.5 * (lo + hi);
        }
    }
    if (checker.check_config(lhs) ||
        checker.check_config(rhs) ||
        check_segment_with_step(checker, lhs, rhs, segment_resolution, segment_step)) {
        return {};
    }
    return {lhs, rhs};
}

bool intervals_contain_point(const std::vector<Interval>& intervals,
                             const Eigen::Ref<const Eigen::VectorXd>& point,
                             double tolerance) {
    return intervals_point_gap(intervals, point) <= tolerance;
}

double box_gap_squared(const BoxNode& lhs, const BoxNode& rhs) {
    if (lhs.n_dims() != rhs.n_dims()) {
        return std::numeric_limits<double>::infinity();
    }
    double gap_sq = 0.0;
    for (int dim = 0; dim < lhs.n_dims(); ++dim) {
        const auto& a = lhs.joint_intervals[static_cast<std::size_t>(dim)];
        const auto& b = rhs.joint_intervals[static_cast<std::size_t>(dim)];
        const double gap = std::max({a.lo - b.hi, b.lo - a.hi, 0.0});
        gap_sq += gap * gap;
    }
    return gap_sq;
}

double point_box_gap_squared(const Eigen::Ref<const Eigen::VectorXd>& point, const BoxNode& box) {
    if (point.size() != box.n_dims()) {
        return std::numeric_limits<double>::infinity();
    }
    double gap_sq = 0.0;
    for (int dim = 0; dim < point.size(); ++dim) {
        const double gap = interval_point_gap(box.joint_intervals[static_cast<std::size_t>(dim)], point[dim]);
        gap_sq += gap * gap;
    }
    return gap_sq;
}

bool box_contains_point_exact(const BoxNode& box, const Eigen::Ref<const Eigen::VectorXd>& point) {
    if (box.n_dims() != point.size()) {
        return false;
    }
    for (int dim = 0; dim < box.n_dims(); ++dim) {
        if (point[dim] < box.joint_intervals[static_cast<std::size_t>(dim)].lo ||
            point[dim] > box.joint_intervals[static_cast<std::size_t>(dim)].hi) {
            return false;
        }
    }
    return true;
}

bool point_covered_by_existing_box(const std::vector<BoxNode>& boxes,
                                   const Eigen::Ref<const Eigen::VectorXd>& point) {
    return std::any_of(boxes.begin(), boxes.end(), [&](const BoxNode& box) {
        return box_contains_point_exact(box, point);
    });
}

double boundary_max_depth_failure_count(const StageContext& context) {
    const auto& diagnostics = context.diagnostics();
    return diagnostics.value("connector.chain_pave_boundary_fail_depth_cap", 0.0) +
           diagnostics.value("connector.chain_pave_boundary_fail_unknown_depth_cap", 0.0) +
           diagnostics.value("connector.chain_pave_boundary_fail_reserved_depth_cap", 0.0);
}

bool can_step_outside_face(const BoxNode& box,
                           const std::vector<Interval>& root,
                           int dim,
                           int side,
                           double epsilon) {
    return side == 1
        ? box.joint_intervals[static_cast<std::size_t>(dim)].hi + epsilon <= root[static_cast<std::size_t>(dim)].hi
        : box.joint_intervals[static_cast<std::size_t>(dim)].lo - epsilon >= root[static_cast<std::size_t>(dim)].lo;
}

double face_seed_score(const BoxNode& box,
                       const std::vector<Interval>& root,
                       const Eigen::Ref<const Eigen::VectorXd>& target,
                       int face_dim,
                       int side,
                       double epsilon) {
    if (!can_step_outside_face(box, root, face_dim, side, epsilon)) {
        return std::numeric_limits<double>::infinity();
    }
    double score = 0.0;
    for (int dim = 0; dim < box.n_dims(); ++dim) {
        double value = target[dim];
        const auto& interval = box.joint_intervals[static_cast<std::size_t>(dim)];
        if (dim == face_dim) {
            value = side == 1 ? interval.hi + epsilon : interval.lo - epsilon;
        } else {
            const double safe_lo = interval.lo + epsilon;
            const double safe_hi = interval.hi - epsilon;
            value = safe_lo <= safe_hi
                ? std::clamp(target[dim], safe_lo, safe_hi)
                : std::clamp(target[dim], interval.lo, interval.hi);
        }
        const double delta = target[dim] - value;
        score += delta * delta;
    }
    return score;
}

Eigen::VectorXd make_face_seed(const BoxNode& box,
                               const std::vector<Interval>& root,
                               const Eigen::Ref<const Eigen::VectorXd>& target,
                               int face_dim,
                               int side,
                               double epsilon) {
    Eigen::VectorXd seed(box.n_dims());
    for (int dim = 0; dim < box.n_dims(); ++dim) {
        const auto& interval = box.joint_intervals[static_cast<std::size_t>(dim)];
        if (dim == face_dim) {
            seed[dim] = side == 1 ? interval.hi + epsilon : interval.lo - epsilon;
        } else {
            const double safe_lo = interval.lo + epsilon;
            const double safe_hi = interval.hi - epsilon;
            seed[dim] = safe_lo <= safe_hi
                ? std::clamp(target[dim], safe_lo, safe_hi)
                : std::clamp(target[dim], interval.lo, interval.hi);
        }
        seed[dim] = std::clamp(seed[dim],
                               root[static_cast<std::size_t>(dim)].lo,
                               root[static_cast<std::size_t>(dim)].hi);
    }
    return seed;
}

void set_max_diagnostic(StageContext& context, const std::string& key, double value) {
    context.diagnostics().set_value(key, std::max(context.diagnostics().value(key), value));
}

int center_broadphase_dimension(const std::unordered_map<int, const BoxNode*>& map,
                                const std::vector<int>& lhs_ids,
                                const std::vector<int>& rhs_ids) {
    if (lhs_ids.empty() && rhs_ids.empty()) {
        return -1;
    }
    const int first_id = lhs_ids.empty() ? rhs_ids.front() : lhs_ids.front();
    auto first_it = map.find(first_id);
    if (first_it == map.end()) {
        return -1;
    }
    const int nd = first_it->second->n_dims();
    int best_dim = 0;
    double best_span = -1.0;
    for (int dim = 0; dim < nd; ++dim) {
        double lo = std::numeric_limits<double>::infinity();
        double hi = -std::numeric_limits<double>::infinity();
        auto observe = [&](const std::vector<int>& ids) {
            for (int id : ids) {
                auto it = map.find(id);
                if (it == map.end()) {
                    continue;
                }
                const double value = it->second->center()[dim];
                lo = std::min(lo, value);
                hi = std::max(hi, value);
            }
        };
        observe(lhs_ids);
        observe(rhs_ids);
        const double span = hi - lo;
        if (span > best_span) {
            best_span = span;
            best_dim = dim;
        }
    }
    return best_dim;
}

std::vector<BridgePairTask> broadphase_bridge_pairs(const std::unordered_map<int, const BoxNode*>& map,
                                                    const std::vector<int>& lhs_ids,
                                                    const std::vector<int>& rhs_ids,
                                                    int result_limit,
                                                    int window_hint) {
    (void)window_hint;
    std::vector<BridgePairTask> out;
    if (lhs_ids.empty() || rhs_ids.empty()) {
        return out;
    }
    const int dim = center_broadphase_dimension(map, lhs_ids, rhs_ids);
    if (dim < 0) {
        return out;
    }
    std::vector<std::pair<double, int>> rhs_sorted;
    rhs_sorted.reserve(rhs_ids.size());
    for (int id : rhs_ids) {
        auto it = map.find(id);
        if (it != map.end()) {
            rhs_sorted.emplace_back(it->second->center()[dim], id);
        }
    }
    std::sort(rhs_sorted.begin(), rhs_sorted.end(), [](const auto& lhs, const auto& rhs) {
        return lhs.first < rhs.first;
    });
    if (rhs_sorted.empty()) {
        return out;
    }
    const int safe_limit = std::max(1, result_limit);
    out.reserve(static_cast<std::size_t>(safe_limit));
    std::unordered_set<std::uint64_t> seen;
    seen.reserve(static_cast<std::size_t>(safe_limit * 16));
    auto pair_key = [](int lhs, int rhs) {
        const std::uint32_t lo = static_cast<std::uint32_t>(std::min(lhs, rhs));
        const std::uint32_t hi = static_cast<std::uint32_t>(std::max(lhs, rhs));
        return (static_cast<std::uint64_t>(lo) << 32) | static_cast<std::uint64_t>(hi);
    };
    auto trim_to_limit = [&]() {
        if (static_cast<int>(out.size()) <= safe_limit) {
            return;
        }
        std::nth_element(out.begin(),
                         out.begin() + safe_limit,
                         out.end(),
                         [](const BridgePairTask& lhs, const BridgePairTask& rhs) {
                             return lhs.score < rhs.score;
                         });
        out.resize(static_cast<std::size_t>(safe_limit));
    };
    auto current_cutoff = [&]() {
        if (static_cast<int>(out.size()) < safe_limit) {
            return std::numeric_limits<double>::infinity();
        }
        double cutoff = 0.0;
        for (const auto& item : out) {
            cutoff = std::max(cutoff, item.score);
        }
        return cutoff;
    };
    for (int lhs_id : lhs_ids) {
        auto lhs_it = map.find(lhs_id);
        if (lhs_it == map.end()) {
            continue;
        }
        const double center_value = lhs_it->second->center()[dim];
        const double cutoff = current_cutoff();
        auto begin = rhs_sorted.begin();
        auto end = rhs_sorted.end();
        if (std::isfinite(cutoff)) {
            const double radius = std::sqrt(std::max(0.0, cutoff));
            begin = std::lower_bound(rhs_sorted.begin(), rhs_sorted.end(), std::make_pair(center_value - radius, std::numeric_limits<int>::min()),
                                     [](const auto& lhs, const auto& rhs) { return lhs.first < rhs.first; });
            end = std::upper_bound(rhs_sorted.begin(), rhs_sorted.end(), std::make_pair(center_value + radius, std::numeric_limits<int>::max()),
                                   [](const auto& lhs, const auto& rhs) { return lhs.first < rhs.first; });
        }
        for (auto it = begin; it != end; ++it) {
            const int rhs_id = it->second;
            const auto key = pair_key(lhs_id, rhs_id);
            if (!seen.insert(key).second) {
                continue;
            }
            const BoxNode& lhs = *lhs_it->second;
            const BoxNode& rhs = *map.at(rhs_id);
            out.push_back({static_cast<int>(out.size()), lhs_id, rhs_id, (lhs.center() - rhs.center()).squaredNorm()});
            trim_to_limit();
        }
    }
    std::sort(out.begin(), out.end(), [](const BridgePairTask& lhs, const BridgePairTask& rhs) {
        return lhs.score < rhs.score;
    });
    if (static_cast<int>(out.size()) > safe_limit) {
        out.resize(static_cast<std::size_t>(safe_limit));
    }
    for (int index = 0; index < static_cast<int>(out.size()); ++index) {
        out[static_cast<std::size_t>(index)].task_id = index;
    }
    return out;
}

int interval_broadphase_dimension(const std::unordered_map<int, const BoxNode*>& map,
                                  const std::vector<int>& lhs_ids,
                                  const std::vector<int>& rhs_ids) {
    if (lhs_ids.empty() && rhs_ids.empty()) {
        return -1;
    }
    const int first_id = lhs_ids.empty() ? rhs_ids.front() : lhs_ids.front();
    auto first_it = map.find(first_id);
    if (first_it == map.end()) {
        return -1;
    }
    const int nd = first_it->second->n_dims();
    int best_dim = 0;
    double best_span = -1.0;
    for (int dim = 0; dim < nd; ++dim) {
        double lo = std::numeric_limits<double>::infinity();
        double hi = -std::numeric_limits<double>::infinity();
        auto observe = [&](const std::vector<int>& ids) {
            for (int id : ids) {
                auto it = map.find(id);
                if (it == map.end()) {
                    continue;
                }
                const auto& interval = it->second->joint_intervals[static_cast<std::size_t>(dim)];
                lo = std::min(lo, interval.lo);
                hi = std::max(hi, interval.hi);
            }
        };
        observe(lhs_ids);
        observe(rhs_ids);
        const double span = hi - lo;
        if (span > best_span) {
            best_span = span;
            best_dim = dim;
        }
    }
    return best_dim;
}

std::vector<BridgePairTask> interval_gap_broadphase_pairs(const std::unordered_map<int, const BoxNode*>& map,
                                                          const std::vector<int>& lhs_ids,
                                                          const std::vector<int>& rhs_ids,
                                                          double gap_tolerance) {
    std::vector<BridgePairTask> out;
    if (lhs_ids.empty() || rhs_ids.empty()) {
        return out;
    }
    const int dim = interval_broadphase_dimension(map, lhs_ids, rhs_ids);
    if (dim < 0) {
        return out;
    }
    std::vector<int> rhs_sorted = rhs_ids;
    rhs_sorted.erase(std::remove_if(rhs_sorted.begin(), rhs_sorted.end(), [&](int id) {
        return map.find(id) == map.end();
    }), rhs_sorted.end());
    std::sort(rhs_sorted.begin(), rhs_sorted.end(), [&](int lhs, int rhs) {
        return map.at(lhs)->joint_intervals[static_cast<std::size_t>(dim)].lo <
               map.at(rhs)->joint_intervals[static_cast<std::size_t>(dim)].lo;
    });
    std::unordered_set<std::uint64_t> seen;
    seen.reserve(lhs_ids.size() * 4);
    auto pair_key = [](int lhs, int rhs) {
        const std::uint32_t lo = static_cast<std::uint32_t>(std::min(lhs, rhs));
        const std::uint32_t hi = static_cast<std::uint32_t>(std::max(lhs, rhs));
        return (static_cast<std::uint64_t>(lo) << 32) | static_cast<std::uint64_t>(hi);
    };
    for (int lhs_id : lhs_ids) {
        auto lhs_it = map.find(lhs_id);
        if (lhs_it == map.end()) {
            continue;
        }
        const auto& lhs_interval = lhs_it->second->joint_intervals[static_cast<std::size_t>(dim)];
        const double search_hi = lhs_interval.hi + gap_tolerance;
        for (int rhs_id : rhs_sorted) {
            const auto& rhs_interval = map.at(rhs_id)->joint_intervals[static_cast<std::size_t>(dim)];
            if (rhs_interval.lo > search_hi) {
                break;
            }
            if (rhs_interval.hi + gap_tolerance < lhs_interval.lo) {
                continue;
            }
            const auto key = pair_key(lhs_id, rhs_id);
            if (!seen.insert(key).second) {
                continue;
            }
            const BoxNode& lhs = *lhs_it->second;
            const BoxNode& rhs = *map.at(rhs_id);
            out.push_back({static_cast<int>(out.size()), lhs_id, rhs_id, interval_max_gap(lhs, rhs)});
        }
    }
    std::sort(out.begin(), out.end(), [](const BridgePairTask& lhs, const BridgePairTask& rhs) {
        if (lhs.score == rhs.score) {
            if (lhs.source_box_id == rhs.source_box_id) {
                return lhs.target_box_id < rhs.target_box_id;
            }
            return lhs.source_box_id < rhs.source_box_id;
        }
        return lhs.score < rhs.score;
    });
    for (int index = 0; index < static_cast<int>(out.size()); ++index) {
        out[static_cast<std::size_t>(index)].task_id = index;
    }
    return out;
}

struct FrontierBridgeCandidate {
    int source_box_id = -1;
    int target_box_id = -1;
    int source_root_id = -1;
    Eigen::VectorXd seed;
    double gap_sq = std::numeric_limits<double>::infinity();
    double projected_gap_sq = std::numeric_limits<double>::infinity();
    double face_score = std::numeric_limits<double>::infinity();
    int ffb_depth_override = 0;
};

bool frontier_bridge_better(const FrontierBridgeCandidate& lhs,
                            const FrontierBridgeCandidate& rhs) {
    if (std::abs(lhs.gap_sq - rhs.gap_sq) > 1e-18) {
        return lhs.gap_sq < rhs.gap_sq;
    }
    if (std::abs(lhs.projected_gap_sq - rhs.projected_gap_sq) > 1e-18) {
        return lhs.projected_gap_sq < rhs.projected_gap_sq;
    }
    if (std::abs(lhs.face_score - rhs.face_score) > 1e-18) {
        return lhs.face_score < rhs.face_score;
    }
    if (lhs.source_box_id != rhs.source_box_id) {
        return lhs.source_box_id < rhs.source_box_id;
    }
    return lhs.target_box_id < rhs.target_box_id;
}

bool select_frontier_bridge_candidate(const std::vector<BoxNode>& boxes,
                                      const AdjacencyGraph& graph,
                                      BoxOracle& oracle,
                                      const IslandConnectorConfig& config,
                                      std::unordered_map<int, double>& best_gap_by_root,
                                      std::unordered_map<int, int>& stale_by_root,
                                      StageContext& context,
                                      FrontierBridgeCandidate& out) {
    const auto islands = find_islands(graph);
    if (islands.size() <= 1) {
        return false;
    }
    const auto map = make_box_map(boxes);
    const auto root = oracle.planning_intervals();
    const double epsilon = std::max(config.frontier_bridge_boundary_epsilon, 0.25 * config.pave.adjacency_tolerance);
    std::vector<FrontierBridgeCandidate> candidates;
    candidates.reserve(static_cast<std::size_t>(std::max(1, config.frontier_bridge_candidate_limit)));

    auto add_candidate = [&](const BoxNode& source, const BoxNode& target, double gap_sq) {
        if (source.n_dims() != target.n_dims()) {
            return;
        }
        const Eigen::VectorXd target_center = target.center();
        for (int dim = 0; dim < source.n_dims(); ++dim) {
            for (int side = 0; side <= 1; ++side) {
                const auto& source_interval = source.joint_intervals[static_cast<std::size_t>(dim)];
                const auto& target_interval = target.joint_intervals[static_cast<std::size_t>(dim)];
                if (source_interval.hi < target_interval.lo && side != 1) {
                    continue;
                }
                if (target_interval.hi < source_interval.lo && side != 0) {
                    continue;
                }
                const double score = face_seed_score(source, root, target_center, dim, side, epsilon);
                if (!std::isfinite(score)) {
                    continue;
                }
                Eigen::VectorXd seed = make_face_seed(source, root, target_center, dim, side, epsilon);
                if (point_covered_by_existing_box(boxes, seed)) {
                    continue;
                }
                const double projected_gap_sq = point_box_gap_squared(seed, target);
                if (!(projected_gap_sq < gap_sq - 1e-18)) {
                    continue;
                }
                candidates.push_back({source.id,
                                      target.id,
                                      source.root_id,
                                      std::move(seed),
                                      gap_sq,
                                      projected_gap_sq,
                                      score,
                                      0});
            }
        }
    };

    for (std::size_t lhs_island = 0; lhs_island < islands.size(); ++lhs_island) {
        for (std::size_t rhs_island = lhs_island + 1; rhs_island < islands.size(); ++rhs_island) {
            const auto pair_tasks = broadphase_bridge_pairs(map,
                                                            islands[lhs_island],
                                                            islands[rhs_island],
                                                            std::max(1, config.frontier_bridge_candidate_limit * 4),
                                                            std::max(4, config.frontier_bridge_candidate_limit / 4));
            context.diagnostics().add_counter("connector.frontier_bridge_broadphase_pairs", static_cast<double>(pair_tasks.size()));
            for (const auto& task : pair_tasks) {
                const BoxNode& lhs = *map.at(task.source_box_id);
                const BoxNode& rhs = *map.at(task.target_box_id);
                const double gap_sq = box_gap_squared(lhs, rhs);
                if (!std::isfinite(gap_sq) || gap_sq <= 1e-24) {
                    continue;
                }
                add_candidate(lhs, rhs, gap_sq);
                add_candidate(rhs, lhs, gap_sq);
            }
        }
    }

    if (candidates.empty()) {
        context.diagnostics().add_counter("connector.frontier_bridge_no_expandable_face");
        return false;
    }
    std::sort(candidates.begin(), candidates.end(), frontier_bridge_better);
    if (static_cast<int>(candidates.size()) > std::max(1, config.frontier_bridge_candidate_limit)) {
        candidates.resize(static_cast<std::size_t>(std::max(1, config.frontier_bridge_candidate_limit)));
    }

    FrontierBridgeCandidate chosen = candidates.front();
    const double nearest_gap = std::sqrt(std::max(0.0, chosen.gap_sq));
    const int root_key = chosen.source_root_id;
    const auto best_it = best_gap_by_root.find(root_key);
    bool improved = best_it == best_gap_by_root.end();
    if (!improved) {
        const double min_delta = std::max(1e-9, 1e-3 * std::max(1.0, best_it->second));
        improved = nearest_gap < best_it->second - min_delta;
    }
    int stale_count = 0;
    if (improved) {
        best_gap_by_root[root_key] = nearest_gap;
        stale_by_root[root_key] = 0;
    } else {
        stale_count = ++stale_by_root[root_key];
        set_max_diagnostic(context, "connector.frontier_bridge_gap_stall_count_max", static_cast<double>(stale_count));
    }

    if (stale_count > 0 && candidates.size() > 1) {
        chosen = candidates[static_cast<std::size_t>(stale_count) % candidates.size()];
        context.diagnostics().add_counter("connector.frontier_bridge_face_rotation_tasks");
    }
    if (config.frontier_bridge_adaptive_ffb &&
        stale_count >= std::max(1, config.frontier_bridge_gap_stall_iterations)) {
        const int stall_window = std::max(1, config.frontier_bridge_gap_stall_iterations);
        const int multiplier = 1 + (stale_count - stall_window) / stall_window;
        int depth = config.pave.find_free_box.max_depth + std::max(1, config.frontier_bridge_ffb_depth_increment) * multiplier;
        if (config.frontier_bridge_ffb_max_depth > 0) {
            depth = std::min(depth, config.frontier_bridge_ffb_max_depth);
        }
        if (depth > config.pave.find_free_box.max_depth) {
            chosen.ffb_depth_override = depth;
            set_max_diagnostic(context, "connector.frontier_bridge_adaptive_ffb_depth_max", static_cast<double>(depth));
        }
    }

    context.diagnostics().add_counter("connector.frontier_bridge_face_candidates", static_cast<double>(candidates.size()));
    context.diagnostics().set_value("connector.frontier_bridge_gap_latest", nearest_gap);
    set_max_diagnostic(context, "connector.frontier_bridge_gap_max", nearest_gap);
    out = std::move(chosen);
    return true;
}

bool add_frontier_bridge_box(const FrontierBridgeCandidate& candidate,
                             std::vector<BoxNode>& boxes,
                             BoxOracle& oracle,
                             AdjacencyGraph& graph,
                             int& next_box_id,
                             const IslandConnectorConfig& config,
                             StageContext& context) {
    auto map = make_box_map(boxes);
    const auto source_it = map.find(candidate.source_box_id);
    if (source_it == map.end()) {
        return false;
    }
    if (point_covered_by_existing_box(boxes, candidate.seed)) {
        context.diagnostics().add_counter("connector.frontier_bridge_seed_already_covered");
        return false;
    }
    FindFreeBoxOptions options = config.pave.find_free_box;
    if (candidate.ffb_depth_override > options.max_depth) {
        options.max_depth = candidate.ffb_depth_override;
        context.diagnostics().add_counter("connector.frontier_bridge_adaptive_ffb_tasks");
    }
    FindFreeBoxService ffb(oracle);
    auto result = ffb.find(candidate.seed, context, options);
    if (!result.found) {
        context.diagnostics().add_counter("connector.frontier_bridge_ffb_failures");
        context.diagnostics().add_counter("connector.frontier_bridge_ffb_fail_code." + std::to_string(result.fail_code));
        if (result.hit_unknown_depth_cap) {
            context.diagnostics().add_counter("connector.frontier_bridge_ffb_unknown_depth_cap");
        }
        if (result.hit_reserved_depth_cap) {
            context.diagnostics().add_counter("connector.frontier_bridge_ffb_reserved_depth_cap");
        }
        return false;
    }
    if (!intervals_contain_point(result.intervals, candidate.seed, config.pave.adjacency_tolerance)) {
        context.diagnostics().add_counter("connector.frontier_bridge_seed_miss");
        return false;
    }
    if (!allow_connector_box_commit(oracle, result, config.pave.commit_policy, context)) {
        context.diagnostics().add_counter("connector.frontier_bridge_commit_rejected");
        return false;
    }

    BoxNode box;
    box.id = next_box_id++;
    box.joint_intervals = std::move(result.intervals);
    box.seed_config = candidate.seed;
    box.tree_id = result.node;
    box.parent_box_id = candidate.source_box_id;
    box.root_id = source_it->second->root_id;
    box.safety_status = result.validation_detail.safety_status;
    box.strict_audit_required = result.validation_detail.strict_audit_required;
    box.compute_volume();
    if (!boxes_connected(*source_it->second, box, config.pave.adjacency_tolerance)) {
        context.diagnostics().add_counter("connector.frontier_bridge_disconnected_child");
        return false;
    }
    oracle.reserve_node(result.node, box.id);
    const int new_box_id = box.id;
    const std::size_t new_box_index = boxes.size();
    boxes.push_back(std::move(box));
    graph[new_box_id] = {};
    int local_edges = 0;
    for (std::size_t index = 0; index < new_box_index; ++index) {
        if (boxes_connected(boxes[index], boxes.back(), config.pave.adjacency_tolerance)) {
            const std::size_t before = graph[new_box_id].size();
            append_graph_edge_unique(graph, boxes[index].id, new_box_id);
            if (graph[new_box_id].size() > before) {
                local_edges += 1;
            }
        }
    }
    context.diagnostics().add_counter("connector.frontier_bridge_incremental_adjacency_checks",
                                      static_cast<double>(new_box_index));
    context.diagnostics().add_counter("connector.frontier_bridge_incremental_edges",
                                      static_cast<double>(local_edges));
    context.diagnostics().add_counter("connector.frontier_bridge_successes");
    return true;
}

bool try_point_validated_gap_edge(const std::vector<BoxNode>& boxes,
                                  AdjacencyGraph& graph,
                                  SegmentEdgeList* segment_edges,
                                  const CollisionChecker& checker,
                                  const IslandConnectorConfig& config,
                                  StageContext& context) {
    if (config.point_validated_gap_tolerance <= 0.0) {
        return false;
    }
    auto islands = find_islands(graph);
    if (islands.size() <= 1) {
        return false;
    }
    std::sort(islands.begin(), islands.end(), [](const auto& lhs, const auto& rhs) {
        return lhs.size() > rhs.size();
    });
    const auto map = make_box_map(boxes);
    const auto& main_island = islands.front();
    const auto& target_island = islands.back();
    struct GapCandidate {
        int source = -1;
        int target = -1;
        double gap = 0.0;
        double distance = 0.0;
    };
    std::vector<GapCandidate> candidates;
    const auto pair_tasks = interval_gap_broadphase_pairs(map,
                                                          main_island,
                                                          target_island,
                                                          config.point_validated_gap_tolerance);
    context.diagnostics().add_counter("connector.point_gap_broadphase_pairs", static_cast<double>(pair_tasks.size()));
    for (const auto& task : pair_tasks) {
        const BoxNode& source = *map.at(task.source_box_id);
        const BoxNode& target = *map.at(task.target_box_id);
        candidates.push_back({task.source_box_id, task.target_box_id, interval_max_gap(source, target), (source.center() - target.center()).norm()});
    }
    std::sort(candidates.begin(), candidates.end(), [](const GapCandidate& lhs, const GapCandidate& rhs) {
        if (lhs.gap == rhs.gap) {
            return lhs.distance < rhs.distance;
        }
        return lhs.gap < rhs.gap;
    });
    if (!candidates.empty()) {
        context.diagnostics().set_value("connector.closest_gap", candidates.front().gap);
        context.diagnostics().set_value("connector.closest_gap_distance", candidates.front().distance);
    }
    for (const auto& candidate : candidates) {
        if (candidate.gap > config.point_validated_gap_tolerance) {
            break;
        }
        const BoxNode& source = *map.at(candidate.source);
        const BoxNode& target = *map.at(candidate.target);
        if (check_segment_with_step(checker,
                                    source.center(),
                                    target.center(),
                                    config.point_validated_gap_resolution,
                                    config.point_validated_gap_step)) {
            context.diagnostics().add_counter("connector.point_gap_collision_rejects");
            continue;
        }
        if (config.segment_edges_fallback_only) {
            continue;
        }
        std::vector<Eigen::VectorXd> waypoints{source.center(), target.center()};
        if (segment_edges != nullptr && config.segment_edges_enabled && config.point_gap_segment_edges) {
            add_segment_edge(*segment_edges,
                             graph,
                             candidate.source,
                             candidate.target,
                             std::move(waypoints),
                             SegmentEdgeType::PointValidatedGap,
                             config.point_validated_gap_resolution,
                             SegmentEdgeValidation::CollisionChecked,
                             false);
            context.diagnostics().add_counter("connector.segment_edges_added");
            context.diagnostics().add_counter("connector.point_gap_segment_edges_added");
        } else {
            graph[candidate.source].push_back(candidate.target);
            graph[candidate.target].push_back(candidate.source);
        }
        context.diagnostics().add_counter("connector.point_validated_gap_edges");
        context.diagnostics().set_value("connector.point_validated_gap", candidate.gap);
        return true;
    }
    return false;
}

RRTConnectOutcome birrt_connect_impl(const Eigen::Ref<const Eigen::VectorXd>& start,
                                     const Eigen::Ref<const Eigen::VectorXd>& goal,
                                     const CollisionChecker& checker,
                                     const Robot& robot,
                                     const RRTConnectConfig& config,
                                     int seed,
                                     std::shared_ptr<std::atomic<bool>> cancel) {
    RRTConnectOutcome outcome;
    const auto t0 = std::chrono::steady_clock::now();
    if (checker.check_config(start) || checker.check_config(goal)) {
        outcome.stats.iterations = 0;
        outcome.stats.raw_waypoints = 0;
        outcome.stats.shortcut_waypoints = 0;
        outcome.stats.elapsed_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        return outcome;
    }
    if (!config.domain_intervals.empty()) {
        if (config.domain_intervals.size() != static_cast<std::size_t>(start.size())) {
            outcome.stats.elapsed_ms =
                std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - t0)
                    .count();
            return outcome;
        }
        for (int dim = 0; dim < start.size(); ++dim) {
            const Interval& interval =
                config.domain_intervals[static_cast<std::size_t>(dim)];
            const double domain_tol = std::max(0.0, config.domain_tolerance);
            if (start[dim] < interval.lo - domain_tol ||
                start[dim] > interval.hi + domain_tol ||
                goal[dim] < interval.lo - domain_tol ||
                goal[dim] > interval.hi + domain_tol) {
                outcome.stats.elapsed_ms =
                    std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - t0)
                        .count();
                return outcome;
            }
        }
    }
    if (!check_segment_with_step(checker, start, goal, config.segment_resolution, config.segment_step)) {
        outcome.path = {start, goal};
        outcome.stats.direct = true;
        outcome.stats.merge_iteration = 0;
        outcome.stats.raw_waypoints = 2;
        outcome.stats.shortcut_waypoints = 2;
        RRTTree start_tree{{start}, {-1}};
        RRTTree goal_tree{{goal}, {-1}};
        finish_rrt_stats(outcome, start_tree, goal_tree, t0);
        return outcome;
    }
    const int nd = static_cast<int>(start.size());
    Eigen::VectorXd lo(nd), hi(nd);
    const auto& limits = robot.joint_limits().limits;
    for (int dim = 0; dim < nd; ++dim) {
        const bool use_domain =
            config.domain_intervals.size() == static_cast<std::size_t>(nd);
        const Interval& interval = use_domain
                                       ? config.domain_intervals[static_cast<std::size_t>(dim)]
                                       : limits[static_cast<std::size_t>(dim)];
        lo[dim] = interval.lo;
        hi[dim] = interval.hi;
        if (config.local_sampling_radius > 0.0) {
            const double local_lo = std::min(start[dim], goal[dim]) - config.local_sampling_radius;
            const double local_hi = std::max(start[dim], goal[dim]) + config.local_sampling_radius;
            lo[dim] = std::max(lo[dim], local_lo);
            hi[dim] = std::min(hi[dim], local_hi);
        }
        const double domain_tol = std::max(0.0, config.domain_tolerance);
        if (start[dim] < lo[dim] - domain_tol || start[dim] > hi[dim] + domain_tol ||
            goal[dim] < lo[dim] - domain_tol || goal[dim] > hi[dim] + domain_tol ||
            hi[dim] < lo[dim]) {
            outcome.stats.iterations = 0;
            outcome.stats.raw_waypoints = 0;
            outcome.stats.shortcut_waypoints = 0;
            outcome.stats.elapsed_ms =
                std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - t0)
                    .count();
            return outcome;
        }
    }

    RRTTree start_tree{{start}, {-1}};
    RRTTree goal_tree{{goal}, {-1}};
    std::mt19937 rng(static_cast<unsigned>(seed));
    std::uniform_real_distribution<double> u01(0.0, 1.0);
    const int optimize_after_first_iters = std::max(0, config.optimize_after_first_iters);
    int first_solution_iter = -1;
    double best_solution_length = std::numeric_limits<double>::infinity();

    for (int iter = 0; iter < config.max_iters; ++iter) {
        outcome.stats.iterations = iter + 1;
        if ((iter & 7) == 7) {
            if (cancel && cancel->load(std::memory_order_relaxed)) {
                outcome.stats.cancelled = true;
                finish_rrt_stats(outcome, start_tree, goal_tree, t0);
                return outcome;
            }
            const double elapsed = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
            if (config.timeout_ms > 0.0 && elapsed > config.timeout_ms) {
                outcome.stats.timed_out = true;
                finish_rrt_stats(outcome, start_tree, goal_tree, t0);
                return outcome;
            }
        }

        const bool grow_from_start = (iter % 2) == 0;
        RRTTree& active_tree = grow_from_start ? start_tree : goal_tree;
        RRTTree& passive_tree = grow_from_start ? goal_tree : start_tree;
        const Eigen::VectorXd& active_target_root = grow_from_start ? goal : start;

        Eigen::VectorXd q_rand(nd);
        if (u01(rng) < std::clamp(config.goal_bias, 0.0, 1.0)) {
            q_rand = active_target_root;
        } else {
            for (int dim = 0; dim < nd; ++dim) {
                std::uniform_real_distribution<double> ud(lo[dim], hi[dim]);
                q_rand[dim] = ud(rng);
            }
        }

        const int active_index = extend_tree(active_tree, q_rand, lo, hi, checker, config);
        if (active_index < 0) {
            continue;
        }
        const int passive_index = connect_greedy(passive_tree,
                                                 active_tree.nodes[static_cast<std::size_t>(active_index)],
                                                 lo,
                                                 hi,
                                                 checker,
                                                 config);
        if (passive_index < 0) {
            continue;
        }

        std::vector<Eigen::VectorXd> raw_path;
        if (grow_from_start) {
            raw_path = merge_birrt_paths(start_tree, active_index, goal_tree, passive_index);
        } else {
            raw_path = merge_birrt_paths(start_tree, passive_index, goal_tree, active_index);
        }
        outcome.stats.merge_iteration = iter + 1;
        outcome.stats.raw_waypoints = static_cast<int>(raw_path.size());
        std::vector<Eigen::VectorXd> candidate =
            config.shortcut_path
                ? shortcut_collision_free_path(raw_path, checker, config.segment_resolution, config.segment_step)
                : std::move(raw_path);
        const double candidate_length = waypoint_path_length(candidate);
        if (candidate_length < best_solution_length) {
            best_solution_length = candidate_length;
            outcome.path = std::move(candidate);
            outcome.stats.shortcut_waypoints = static_cast<int>(outcome.path.size());
        }
        if (first_solution_iter < 0) {
            first_solution_iter = iter;
        }
        if (optimize_after_first_iters <= 0 ||
            iter + 1 >= first_solution_iter + 1 + optimize_after_first_iters) {
            finish_rrt_stats(outcome, start_tree, goal_tree, t0);
            return outcome;
        }
    }
    finish_rrt_stats(outcome, start_tree, goal_tree, t0);
    return outcome;
}

}  // namespace

std::vector<Eigen::VectorXd> rrt_connect(const Eigen::Ref<const Eigen::VectorXd>& start,
                                         const Eigen::Ref<const Eigen::VectorXd>& goal,
                                         const CollisionChecker& checker,
                                         const Robot& robot,
                                         const RRTConnectConfig& config,
                                         int seed,
                                         std::shared_ptr<std::atomic<bool>> cancel) {
    return birrt_connect_impl(start, goal, checker, robot, config, seed, std::move(cancel)).path;
}

std::vector<Eigen::VectorXd> rrt_connect(const Eigen::Ref<const Eigen::VectorXd>& start,
                                         const Eigen::Ref<const Eigen::VectorXd>& goal,
                                         const CollisionChecker& checker,
                                         const Robot& robot,
                                         StageContext& context,
                                         const RRTConnectConfig& config,
                                         int seed) {
    if (context.should_stop()) {
        return {};
    }
    auto outcome = birrt_connect_impl(start, goal, checker, robot, config, seed, context.native_cancel_flag());
    record_birrt_stats(context.diagnostics(), outcome.stats);
    return outcome.path;
}

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
                          bool /*require_adjacency*/,
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
            const int committed = commit_box(result, to_pt, from_id, true);
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
        auto record_boundary_ffb_failure = [&](const FindFreeBoxResult& result) {
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
                failure.seed.assign(to_pt.data(), to_pt.data() + to_pt.size());
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
        };
	        if (existing_cover < 0) {
	            context.diagnostics().add_counter("connector.chain_pave_boundary_ffb_calls");
	            auto result = find_at_depth(to_pt, context, config.find_free_box.max_depth);
	            if (result.seed_collision) {
	                record_boundary_ffb_failure(result);
	                context.diagnostics().add_counter("connector.chain_pave_boundary_reject_not_free");
	                rejection_counted = true;
            } else if (result.found ||
                       (result.hit_reserved_depth_cap &&
                        intervals_contain_point(result.intervals,
                                                to_pt,
                                                config.adjacency_tolerance))) {
                consume_result(result);
            } else {
                record_boundary_ffb_failure(result);
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

    // Max parameter u in [u0, 1] such that a + u*(b - a) stays inside `box`.
    auto segment_exit_param = [](const BoxNode& box, const Eigen::VectorXd& a,
                                 const Eigen::VectorXd& b, double u0) -> double {
        double u_hi = 1.0;
        const Eigen::VectorXd v = b - a;
        for (int d = 0; d < a.size(); ++d) {
            const double lo = box.joint_intervals[d].lo;
            const double hi = box.joint_intervals[d].hi;
            if (std::abs(v[d]) < 1e-15) {
                continue;  // parallel to this slab; no constraint from it
            }
            const double t1 = (lo - a[d]) / v[d];
            const double t2 = (hi - a[d]) / v[d];
            u_hi = std::min(u_hi, std::max(t1, t2));
        }
        return std::max(u0, std::min(1.0, u_hi));
    };

    auto boundary_seed_from_box = [&](const BoxNode& box,
                                      const Eigen::VectorXd& from,
                                      const Eigen::VectorXd& target) -> Eigen::VectorXd {
        const double seg_len = (target - from).norm();
        if (seg_len < 1e-12) {
            return target;
        }
        const double u_exit = segment_exit_param(box, from, target, 0.0);
        // `requested_step` controls how far the front tries to advance along the
        // corridor. The FFB seed itself should only step just outside the current
        // box face; otherwise FFB often certifies a free box beyond a tiny gap and
        // the result is free but non-adjacent to `box`.
        const double face_epsilon =
            std::max(16.0 * std::max(0.0, config.adjacency_tolerance),
                     std::min(1e-6, std::max(config.gap_fill_min_step, 1e-12)));
        const double u_step =
            std::max(1e-12, face_epsilon / std::max(seg_len, 1e-12));
        const double u_seed = std::min(1.0, u_exit + u_step);
        return (from + u_seed * (target - from)).eval();
    };

    auto boundary_seed_candidates = [&](const BoxNode& box,
                                        const Eigen::VectorXd& from,
                                        const Eigen::VectorXd& target,
                                        double requested_step)
        -> std::vector<Eigen::VectorXd> {
        std::vector<Eigen::VectorXd> seeds;
        const Eigen::VectorXd forward_seed = boundary_seed_from_box(box, from, target);
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

IslandConnector::IslandConnector(BoxOracle& oracle,
                                 const Robot& robot,
                                 const CollisionChecker& checker,
                                 IslandConnectorConfig config)
    : oracle_(oracle), robot_(robot), checker_(checker), config_(std::move(config)) {}

IslandConnectorResult IslandConnector::connect_all(std::vector<BoxNode>& boxes,
                                                   AdjacencyGraph& graph,
                                                   int& next_box_id) {
    SegmentEdgeList transient_segment_edges;
    return connect_all(boxes, graph, transient_segment_edges, next_box_id);
}

IslandConnectorResult IslandConnector::connect_all(std::vector<BoxNode>& boxes,
                                                   AdjacencyGraph& graph,
                                                   SegmentEdgeList& segment_edges,
                                                   int& next_box_id) {
    RuntimeConfig runtime;
    runtime.mode = config_.n_threads > 1 ? ExecutionMode::Parallel : ExecutionMode::Inline;
    runtime.n_threads = config_.n_threads;
    runtime.batch_size = config_.pair_batch_size;
    runtime.parallel_threshold = config_.parallel_threshold;
    runtime.deterministic_reduce = config_.deterministic_reduce;
    StageContext context(runtime);
    return connect_all(boxes, graph, segment_edges, next_box_id, context);
}

IslandConnectorResult IslandConnector::connect_all(std::vector<BoxNode>& boxes,
                                                   AdjacencyGraph& graph,
                                                   int& next_box_id,
                                                   StageContext& context) {
    SegmentEdgeList transient_segment_edges;
    return connect_all(boxes, graph, transient_segment_edges, next_box_id, context);
}

IslandConnectorResult IslandConnector::connect_all(std::vector<BoxNode>& boxes,
                                                   AdjacencyGraph& graph,
                                                   SegmentEdgeList& segment_edges,
                                                   int& next_box_id,
                                                   StageContext& context) {
    IslandConnectorResult result;
    auto islands = find_islands(graph);
    context.diagnostics().add_counter("connector.islands_initial", static_cast<double>(islands.size()));
    if (islands.size() <= 1) {
        result.connected = true;
        return result;
    }

    std::unordered_map<int, double> frontier_best_gap_by_root;
    std::unordered_map<int, int> frontier_stale_by_root;
    while (config_.frontier_bridge && islands.size() > 1 &&
           result.bridge_boxes_added < config_.max_total_bridge_boxes) {
        if (context.should_stop()) {
            break;
        }
        FrontierBridgeCandidate candidate;
        if (!select_frontier_bridge_candidate(boxes,
                                              graph,
                                              oracle_,
                                              config_,
                                              frontier_best_gap_by_root,
                                              frontier_stale_by_root,
                                              context,
                                              candidate)) {
            break;
        }
        context.diagnostics().add_counter("connector.frontier_bridge_attempts");
        if (!add_frontier_bridge_box(candidate,
                                     boxes,
                                     oracle_,
                                     graph,
                                     next_box_id,
                                     config_,
                                     context)) {
            break;
        }
        result.bridge_boxes_added += 1;
        islands = find_islands(graph);
    }

    while (islands.size() > 1 &&
           (result.bridge_boxes_added < config_.max_total_bridge_boxes ||
            (config_.segment_edges_enabled && config_.rrt_segment_edges && !config_.segment_edges_fallback_only))) {
        if (context.should_stop()) {
            break;
        }
        auto map = make_box_map(boxes);
        std::sort(islands.begin(), islands.end(), [](const auto& lhs, const auto& rhs) {
            return lhs.size() > rhs.size();
        });
        // E5: gather candidates between every island pair in a single round so
        // the parallel_for fills all worker threads (the gaps are independent).
        // This is intentionally not largest-island-only: in shelf-like scenes,
        // two small query-anchor islands can be much closer to each other than
        // either is to the largest component, and connecting them first gives the
        // box connector a shorter, easier target.
        // box_id -> island index, used at commit time to merge distinct components.
        std::unordered_map<int, int> island_of;
        for (std::size_t isl = 0; isl < islands.size(); ++isl) {
            for (int box_id : islands[isl]) {
                island_of[box_id] = static_cast<int>(isl);
            }
        }
        std::vector<BridgePairTask> candidates;
        const int per_gap_limit = std::max(1, config_.max_pairs_per_gap);
        for (std::size_t lhs_isl = 0; lhs_isl < islands.size(); ++lhs_isl) {
            for (std::size_t rhs_isl = lhs_isl + 1; rhs_isl < islands.size(); ++rhs_isl) {
                std::vector<BridgePairTask> gap_candidates = broadphase_bridge_pairs(map,
                                                                                     islands[lhs_isl],
                                                                                     islands[rhs_isl],
                                                                                     per_gap_limit,
                                                                                     std::max(4, config_.max_pairs_per_gap));
                for (auto& task : gap_candidates) {
                    task.task_id = static_cast<int>(candidates.size());
                    candidates.push_back(std::move(task));
                }
            }
        }
        const int broadphase_pairs_before_prune = static_cast<int>(candidates.size());
        const int global_candidate_limit =
            std::min(per_gap_limit,
                     std::max(4, 4 * std::max(1, static_cast<int>(islands.size()) - 1)));
        if (static_cast<int>(candidates.size()) > global_candidate_limit) {
            std::nth_element(candidates.begin(),
                             candidates.begin() + global_candidate_limit,
                             candidates.end(),
                             [](const BridgePairTask& lhs, const BridgePairTask& rhs) {
                                 return lhs.score < rhs.score;
                             });
            candidates.resize(static_cast<std::size_t>(global_candidate_limit));
        }
        std::sort(candidates.begin(), candidates.end(), [](const BridgePairTask& lhs, const BridgePairTask& rhs) {
            if (lhs.score == rhs.score) {
                if (lhs.source_box_id == rhs.source_box_id) {
                    return lhs.target_box_id < rhs.target_box_id;
                }
                return lhs.source_box_id < rhs.source_box_id;
            }
            return lhs.score < rhs.score;
        });
        for (int index = 0; index < static_cast<int>(candidates.size()); ++index) {
            candidates[static_cast<std::size_t>(index)].task_id = index;
        }
        context.diagnostics().add_counter("connector.bridge_broadphase_pairs_raw",
                                          static_cast<double>(broadphase_pairs_before_prune));
        context.diagnostics().add_counter("connector.bridge_broadphase_pairs", static_cast<double>(candidates.size()));
        context.diagnostics().add_counter("connector.bridge_broadphase_pairs_pruned",
                                          static_cast<double>(std::max(0, broadphase_pairs_before_prune -
                                                                          static_cast<int>(candidates.size()))));
        bool progressed = false;
        std::vector<BridgePairResult> successful_pairs;
        RRTConnectConfig pair_rrt = config_.rrt;
        if (config_.per_pair_timeout_ms > 0.0 &&
            (pair_rrt.timeout_ms <= 0.0 || config_.per_pair_timeout_ms < pair_rrt.timeout_ms)) {
            pair_rrt.timeout_ms = config_.per_pair_timeout_ms;
        }

        const bool run_parallel = context.executor().n_threads() > 1 &&
            static_cast<int>(candidates.size()) >= config_.parallel_threshold;
        if (run_parallel) {
            std::vector<BridgePairResult> pair_results(candidates.size());
            auto local_cancel = std::make_shared<std::atomic<bool>>(false);
            context.executor().parallel_for(0, static_cast<int>(candidates.size()), [&](int idx) {
                if (context.should_stop() || local_cancel->load(std::memory_order_relaxed)) {
                    return;
                }
                const auto& candidate = candidates[static_cast<std::size_t>(idx)];
                const BoxNode& source_box = *map.at(candidate.source_box_id);
                const BoxNode& target_box = *map.at(candidate.target_box_id);
                const auto& source_center = source_box.center();
                const auto& target_center = target_box.center();
                const RRTConnectConfig domain_rrt =
                    with_query_root_hull_domain(pair_rrt, oracle_, source_center, target_center);
                RRTConnectConfig box_rrt = domain_rrt;
                if (config_.pave.require_connected_chain) {
                    box_rrt.shortcut_path = true;
                }
                auto path = closest_box_point_segment(source_box, target_box, checker_, box_rrt.segment_resolution, box_rrt.segment_step);
                if (!path.empty()) {
                    context.diagnostics().add_counter("connector.direct_box_segment_successes");
                    context.diagnostics().add_counter("connector.rrt_successes");
                    pair_results[static_cast<std::size_t>(idx)] = {
                        candidate.task_id,
                        candidate.source_box_id,
                        candidate.target_box_id,
                        std::move(path),
                        true};
                    if (!config_.deterministic_reduce) {
                        local_cancel->store(true, std::memory_order_relaxed);
                    }
                    return;
                }
                const bool source_colliding = checker_.check_config(source_center);
                const bool target_colliding = checker_.check_config(target_center);
                if (source_colliding || target_colliding) {
                    context.diagnostics().add_counter("connector.center_collision_candidates");
                    if (source_colliding) {
                        context.diagnostics().add_counter("connector.source_center_collisions");
                    }
                    if (target_colliding) {
                        context.diagnostics().add_counter("connector.target_center_collisions");
                    }
                    return;
                }
                if (!config_.enable_birrt) {
                    context.diagnostics().add_counter("connector.birrt_disabled_skips");
                    return;
                }
                context.diagnostics().add_counter("connector.birrt_invocations");
                auto outcome = birrt_connect_impl(
                    source_center,
                    target_center,
                    checker_,
                    robot_,
                    box_rrt,
                    candidate.source_box_id + candidate.target_box_id + candidate.task_id,
                    local_cancel);
                record_birrt_stats(context.diagnostics(), outcome.stats);
                path = std::move(outcome.path);
                if (!path.empty()) {
                    context.diagnostics().add_counter("connector.rrt_successes");
                    pair_results[static_cast<std::size_t>(idx)] = {
                        candidate.task_id,
                        candidate.source_box_id,
                        candidate.target_box_id,
                        std::move(path),
                        true};
                    if (!config_.deterministic_reduce) {
                        local_cancel->store(true, std::memory_order_relaxed);
                    }
                } else {
                    context.diagnostics().add_counter("connector.rrt_failures");
                }
            });
            result.attempted_pairs += static_cast<int>(candidates.size());
            for (auto& item : pair_results) {
                if (item.success) {
                    successful_pairs.push_back(std::move(item));
                }
            }
        } else {
            for (const auto& candidate : candidates) {
                if (context.should_stop()) {
                    break;
                }
                result.attempted_pairs += 1;
                const BoxNode& source_box = *map.at(candidate.source_box_id);
                const BoxNode& target_box = *map.at(candidate.target_box_id);
                const auto& source_center = source_box.center();
                const auto& target_center = target_box.center();
                const RRTConnectConfig domain_rrt =
                    with_query_root_hull_domain(pair_rrt, oracle_, source_center, target_center);
                RRTConnectConfig box_rrt = domain_rrt;
                if (config_.pave.require_connected_chain) {
                    box_rrt.shortcut_path = true;
                }
                auto path = closest_box_point_segment(source_box, target_box, checker_, box_rrt.segment_resolution, box_rrt.segment_step);
                if (!path.empty()) {
                    context.diagnostics().add_counter("connector.direct_box_segment_successes");
                    context.diagnostics().add_counter("connector.rrt_successes");
                    successful_pairs.push_back({
                        candidate.task_id,
                        candidate.source_box_id,
                        candidate.target_box_id,
                        std::move(path),
                        true});
                    continue;
                }
                const bool source_colliding = checker_.check_config(source_center);
                const bool target_colliding = checker_.check_config(target_center);
                if (source_colliding || target_colliding) {
                    context.diagnostics().add_counter("connector.center_collision_candidates");
                    if (source_colliding) {
                        context.diagnostics().add_counter("connector.source_center_collisions");
                    }
                    if (target_colliding) {
                        context.diagnostics().add_counter("connector.target_center_collisions");
                    }
                    continue;
                }
                if (!config_.enable_birrt) {
                    context.diagnostics().add_counter("connector.birrt_disabled_skips");
                    continue;
                }
                context.diagnostics().add_counter("connector.birrt_invocations");
                path = rrt_connect(
                    source_center,
                    target_center,
                    checker_,
                    robot_,
                    context,
                    box_rrt,
                    candidate.source_box_id + candidate.target_box_id + candidate.task_id);
                if (path.empty()) {
                    context.diagnostics().add_counter("connector.rrt_failures");
                    continue;
                }
                context.diagnostics().add_counter("connector.rrt_successes");
                successful_pairs.push_back({
                    candidate.task_id,
                    candidate.source_box_id,
                    candidate.target_box_id,
                    std::move(path),
                    true});
            }
        }

        // E5 deterministic commit: process successful pairs in a stable order (by
        // task_id) and commit every bridge whose two islands are still in distinct
        // components, tracked by a union-find over island indices. This merges
        // multiple independent gaps per round while keeping commit fully serial and
        // order-independent of thread completion. Adjacency is recomputed once after
        // all commits.
        std::sort(successful_pairs.begin(), successful_pairs.end(),
                  [](const BridgePairResult& lhs, const BridgePairResult& rhs) {
                      return lhs.task_id < rhs.task_id;
                  });
        std::vector<int> uf(islands.size());
        for (std::size_t i = 0; i < uf.size(); ++i) {
            uf[i] = static_cast<int>(i);
        }
        auto uf_find = [&](int x) {
            while (uf[x] != x) {
                uf[x] = uf[uf[x]];
                x = uf[x];
            }
            return x;
        };
        bool boxes_added_this_round = false;
        for (const auto& chosen : successful_pairs) {
            const auto src_isl_it = island_of.find(chosen.source_box_id);
            const auto tgt_isl_it = island_of.find(chosen.target_box_id);
            if (src_isl_it == island_of.end() || tgt_isl_it == island_of.end()) {
                continue;
            }
            const int src_root = uf_find(src_isl_it->second);
            const int tgt_root = uf_find(tgt_isl_it->second);
            if (src_root == tgt_root) {
                // These two islands were already bridged earlier this round.
                continue;
            }
            std::vector<Eigen::VectorXd> bridge_path = chosen.waypoint_path;
            if (config_.pave.require_connected_chain && bridge_path.size() > 2) {
                const double pave_step =
                    config_.pave.gap_fill_sample_step > 0.0
                        ? std::max(0.05, config_.pave.gap_fill_sample_step * 2.0)
                        : std::max(0.05, config_.rrt.step_size * 0.5);
                bridge_path = densify_path_by_step(bridge_path, pave_step);
                context.diagnostics().set_value("connector.box_shortcut_densified_last_waypoints",
                                                static_cast<double>(bridge_path.size()));
            }
            int added = 0;
            bool box_connected = false;
            double pair_depth_failures_before = boundary_max_depth_failure_count(context);
            if (result.bridge_boxes_added < config_.max_total_bridge_boxes) {
                context.diagnostics().add_counter("connector.chain_pave_attempts");
                const int first_new_box_id = next_box_id;
                added = chain_pave_along_path(
                    bridge_path,
                    chosen.source_box_id,
                    boxes,
                    oracle_,
                    graph,
                    next_box_id,
                    context,
                    config_.pave);
                if (added > 0) {
                    const int local_edges = connect_new_boxes_to_island(
                        boxes,
                        graph,
                        first_new_box_id,
                        next_box_id,
                        islands[static_cast<std::size_t>(tgt_isl_it->second)],
                        config_.pave.adjacency_tolerance);
                    if (local_edges > 0) {
                        context.diagnostics().add_counter("connector.chain_pave_local_target_edges",
                                                          static_cast<double>(local_edges));
                    }
                    box_connected = graph_has_path(graph, chosen.source_box_id, chosen.target_box_id);
                    if (!box_connected) {
                        context.diagnostics().add_counter("connector.chain_pave_full_adjacency_rebuilds_avoided");
                        context.diagnostics().add_counter("connector.chain_pave_incremental_path_misses");
                        box_connected = graph_has_path(graph, chosen.source_box_id, chosen.target_box_id);
                    }
                    if (box_connected) {
                        context.diagnostics().add_counter("connector.chain_pave_box_connected");
                    } else {
                        context.diagnostics().add_counter("connector.chain_pave_partial_added");
                    }
                }
            }
            const bool pair_had_max_depth_ffb_failure =
                boundary_max_depth_failure_count(context) > pair_depth_failures_before + 0.5;
            bool added_segment_edge = false;
            if (!box_connected &&
                config_.segment_edges_enabled && config_.rrt_segment_edges &&
                !config_.segment_edges_fallback_only &&
                pair_had_max_depth_ffb_failure) {
                const int edge_id = add_segment_edge(segment_edges,
                                                     graph,
                                                     chosen.source_box_id,
                                                     chosen.target_box_id,
                                                     bridge_path,
                                                     SegmentEdgeType::RRTConnector,
                                                     config_.rrt.segment_resolution,
                                                     SegmentEdgeValidation::CollisionChecked,
                                                     false);
                if (edge_id >= 0) {
                    added_segment_edge = true;
                    result.segment_edges_added += 1;
                    result.rrt_segment_edges_added += 1;
                    context.diagnostics().add_counter("connector.segment_edges_added");
                    context.diagnostics().add_counter("connector.rrt_segment_edges_added");
                }
            } else if (!box_connected &&
                       config_.segment_edges_enabled && config_.rrt_segment_edges &&
                       !config_.segment_edges_fallback_only &&
                       !pair_had_max_depth_ffb_failure) {
                context.diagnostics().add_counter("connector.segment_edge_blocked_no_max_depth_ffb_failure");
            }
            if (box_connected) {
                context.diagnostics().add_counter("connector.chain_pave_successes");
                result.bridge_boxes_added += added;
                boxes_added_this_round = true;
                uf[src_root] = tgt_root;
                progressed = true;
            } else if (added > 0) {
                result.bridge_boxes_added += added;
                boxes_added_this_round = true;
                if (added_segment_edge) {
                    uf[src_root] = tgt_root;
                    progressed = true;
                }
            } else if (added_segment_edge) {
                uf[src_root] = tgt_root;
                progressed = true;
            } else {
                context.diagnostics().add_counter("connector.chain_pave_zero_added");
            }
        }
        if (boxes_added_this_round) {
            context.diagnostics().add_counter("connector.round_full_adjacency_rebuilds_avoided");
        }
        apply_segment_edges_to_adjacency(segment_edges, graph);
        if (!progressed) {
            break;
        }
        islands = find_islands(graph);
    }
    while (find_islands(graph).size() > 1) {
        if (boundary_max_depth_failure_count(context) <= 0.5) {
            context.diagnostics().add_counter("connector.point_gap_segment_blocked_no_max_depth_ffb_failure");
            break;
        }
        const std::size_t edge_count_before = segment_edges.size();
        if (!try_point_validated_gap_edge(boxes, graph, &segment_edges, checker_, config_, context)) {
            break;
        }
        if (segment_edges.size() > edge_count_before) {
            const int added_edges = static_cast<int>(segment_edges.size() - edge_count_before);
            result.segment_edges_added += added_edges;
            result.point_gap_segment_edges_added += added_edges;
        }
    }
    result.connected = find_islands(graph).size() <= 1;
    return result;
}

}  // namespace rbf
