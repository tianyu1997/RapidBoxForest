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
    auto lhs = oracle.native_root_intervals_for_query(start);
    auto rhs = oracle.native_root_intervals_for_query(goal);
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
    const auto root = oracle.native_root_hull();
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
    boxes.push_back(std::move(box));
    graph = compute_adjacency(boxes, config.pave.adjacency_tolerance);
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
        outcome.path = config.shortcut_path
                           ? shortcut_collision_free_path(raw_path, checker, config.segment_resolution, config.segment_step)
                           : std::move(raw_path);
        outcome.stats.shortcut_waypoints = static_cast<int>(outcome.path.size());
        finish_rrt_stats(outcome, start_tree, goal_tree, t0);
        return outcome;
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
                          bool require_adjacency,
                          bool allow_duplicate_node = false) -> int {
        BoxNode* parent_box = box_by_id(parent_id);
        if (parent_box == nullptr) {
            return -1;
        }
        if (result.node == parent_box->tree_id) {
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
        (void)require_adjacency;
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
        [&](int from_id, const Eigen::VectorXd& from_pt, const Eigen::VectorXd& to_pt, int budget) -> int {
        if (added >= config.max_chain || context.should_stop()) {
            return from_id;
        }
        // Obtain a box covering to_pt: commit a fresh certified box, or reuse an
        // existing box that already owns that canonical cell (so the chain can
        // follow the bridge through previously paved regions).
        int to_id = from_id;
        auto result = ffb.find(to_pt, context, config.find_free_box);
        if (result.found) {
            const int committed = commit_box(result, to_pt, from_id, true);
            if (committed >= 0) {
                to_id = committed;
            } else {
                const int owner = find_box_owning_node_covering(result.node, to_pt);
                if (owner >= 0) {
                    BoxNode* owner_box = box_by_id(owner);
                    BoxNode* from_box = box_by_id(from_id);
                    if (owner_box != nullptr && from_box != nullptr &&
                        boxes_connected(*from_box, *owner_box, config.adjacency_tolerance)) {
                        to_id = owner;
                    }
                }
            }
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

    // When filling gaps we traverse each bridge segment under a fixed probe
    // budget. We still reuse the current box and opportunistically jump across
    // any long span it covers, but we only SEED at coarse arc-length samples
    // (`gap_fill_sample_step`, else max_steps_per_waypoint subdivisions). This
    // keeps runtime and box count near O(samples) instead of O(thin certified
    // slabs), which is the right trade-off when we want broad, cheap coverage
    // rather than exhaustive full coverage.

    // Seed a *fresh* certified-free box covering point `p` (no reuse of an
    // existing box). FFB returns the canonical cell containing `p`; because `p`
    // lies inside that cell, the cell extends forward along any travel direction,
    // guaranteeing progress for the sweep below. Returns the new box id or -1.
    auto seed_fresh = [&](const Eigen::VectorXd& p, int parent_id) -> int {
        auto result = ffb.find(p, context, config.find_free_box);
        if (!result.found) {
            // FFB can cap out on a reserved canonical leaf an earlier box already
            // owns; `result.intervals` is that certified-free cell mapped to this
            // sample's symmetry sector, so it contains the sample even though the
            // existing box covers a different sector.
            if (result.hit_reserved_depth_cap &&
                intervals_contain_point(result.intervals, p,
                                        config.adjacency_tolerance)) {
                return commit_reserved_cap_box(result, p, parent_id);
            }
            return -1;
        }
        int committed = commit_box(result, p, parent_id, true);
        if (committed < 0) {
            const int owner = find_box_owning_node_covering(result.node, p);
            if (owner >= 0) {
                BoxNode* owner_box = box_by_id(owner);
                BoxNode* parent_box = box_by_id(parent_id);
                const bool adjacent_owner =
                    owner_box != nullptr && parent_box != nullptr &&
                    boxes_connected(*parent_box, *owner_box,
                                    config.adjacency_tolerance);
                if (owner_box != nullptr && owner_box->contains(p) &&
                    (!config.require_connected_chain || adjacent_owner)) {
                    committed = owner;
                }
            }
        }
        if (committed < 0 && !config.require_connected_chain) {
            committed = commit_box(result, p, parent_id,
                                   /*require_adjacency=*/false,
                                   /*allow_duplicate_node=*/true);
        }
        return committed;
    };

    auto find_existing_cover = [&](const Eigen::VectorXd& p,
                                   int preferred_id) -> int {
        BoxNode* preferred_box = box_by_id(preferred_id);
        if (preferred_box != nullptr && preferred_box->contains(p)) {
            return preferred_id;
        }
        auto graph_it = graph.find(preferred_id);
        if (graph_it != graph.end()) {
            for (const int neighbor_id : graph_it->second) {
                BoxNode* neighbor = box_by_id(neighbor_id);
                if (neighbor != nullptr && neighbor->contains(p)) {
                    return neighbor_id;
                }
            }
        }
        for (const auto& box : boxes) {
            BoxNode* preferred_box_again = box_by_id(preferred_id);
            if (preferred_box_again != nullptr &&
                boxes_connected(*preferred_box_again, box, config.adjacency_tolerance) &&
                box.contains(p)) {
                return box.id;
            }
        }
        return -1;
    };

    // Return a box covering point `p`, reusing existing boxes first, else
    // seeding a fresh box. Returns -1 if none could be obtained.
    auto cover_point = [&](const Eigen::VectorXd& p, int parent_id) -> int {
        const int existing = find_existing_cover(p, parent_id);
        return existing >= 0 ? existing : seed_fresh(p, parent_id);
    };

    auto owned_node_covering_point = [&](const Eigen::VectorXd& p) -> int {
        if (p.size() != oracle.n_dims()) {
            return -1;
        }
        OracleNodeId node = oracle.root_node();
        if (!oracle.contains_point(node, p)) {
            return -1;
        }
        int best = -1;
        auto owner_it = tree_owner.find(node);
        if (owner_it != tree_owner.end()) {
            best = static_cast<int>(node);
        }
        int guard = 0;
        while (!oracle.is_leaf(node) && guard++ <= oracle.max_tree_depth() + 2) {
            const OracleNodeId child = oracle.child_containing_point(node, p);
            if (child == kInvalidOracleNodeId ||
                !oracle.contains_point(child, p)) {
                break;
            }
            node = child;
            owner_it = tree_owner.find(node);
            if (owner_it != tree_owner.end()) {
                best = static_cast<int>(node);
            }
        }
        return best;
    };

    auto clone_owned_node_cover = [&](const Eigen::VectorXd& p,
                                      int parent_id) -> int {
        const int existing = find_existing_cover(p, parent_id);
        if (existing >= 0) {
            return existing;
        }
        const int node = owned_node_covering_point(p);
        if (node < 0) {
            return -1;
        }
        const auto tree_intervals = oracle.node_intervals(node);
        auto query_intervals = oracle.query_intervals_for_node(
            node, tree_intervals, p);
        if (!intervals_contain_point(query_intervals, p,
                                     config.adjacency_tolerance)) {
            return -1;
        }
        FindFreeBoxResult result;
        result.node = node;
        result.intervals = std::move(query_intervals);
        result.hit_reserved_depth_cap = true;
        return commit_reserved_cap_box(result, p, parent_id);
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
                                      const Eigen::VectorXd& target,
                                      double requested_step) -> Eigen::VectorXd {
        const double seg_len = (target - from).norm();
        if (seg_len < 1e-12) {
            return target;
        }
        const double u_exit = segment_exit_param(box, from, target, 0.0);
        const double u_step =
            std::max(1e-9, std::max(requested_step, config.gap_fill_min_step) /
                                  std::max(seg_len, 1e-12));
        const double u_seed = std::min(1.0, u_exit + u_step);
        return (from + u_seed * (target - from)).eval();
    };

    auto boundary_seed_candidates = [&](const BoxNode& box,
                                        const Eigen::VectorXd& from,
                                        const Eigen::VectorXd& target,
                                        double requested_step)
        -> std::vector<Eigen::VectorXd> {
        std::vector<Eigen::VectorXd> seeds;
        seeds.push_back(boundary_seed_from_box(box, from, target, requested_step));
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

    struct ParamSpan {
        bool hit = false;
        double lo = 0.0;
        double hi = 0.0;
    };

    auto segment_interval_span = [&](const std::vector<Interval>& intervals,
                                     const Eigen::VectorXd& a,
                                     const Eigen::VectorXd& b) -> ParamSpan {
        double u_lo = 0.0;
        double u_hi = 1.0;
        const Eigen::VectorXd v = b - a;
        for (int d = 0; d < a.size(); ++d) {
            const double lo = intervals[static_cast<std::size_t>(d)].lo -
                              config.adjacency_tolerance;
            const double hi = intervals[static_cast<std::size_t>(d)].hi +
                              config.adjacency_tolerance;
            if (std::abs(v[d]) < 1e-15) {
                if (a[d] < lo || a[d] > hi) {
                    return {};
                }
                continue;
            }
            const double t1 = (lo - a[d]) / v[d];
            const double t2 = (hi - a[d]) / v[d];
            u_lo = std::max(u_lo, std::min(t1, t2));
            u_hi = std::min(u_hi, std::max(t1, t2));
            if (u_lo > u_hi) {
                return {};
            }
        }
        return {true, std::max(0.0, u_lo), std::min(1.0, u_hi)};
    };

    const bool fast_gap_fill =
        config.gap_fill_time_budget_ms > 0.0 ||
        config.gap_fill_max_ffb_calls >= 0;

    if (config.require_connected_chain && config.fill_gaps &&
        waypoint_path.size() >= 2) {
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
            Eigen::VectorXd cursor = current_box->contains(a) ? a : current_box->center();
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
                    cursor = current_box->center();
                }
                int reached = current_box_id;
                double attempt_step = front_step;
                for (int attempt = 0; attempt < 8 && reached == current_box_id;
                     ++attempt) {
                    const auto seeds =
                        boundary_seed_candidates(*current_box, cursor, b, attempt_step);
                    if (seeds.empty() ||
                        (seeds.front() - cursor).norm() <
                            std::max(config.gap_fill_min_step, 1e-6) * 0.25) {
                        break;
                    }
                    for (const auto& seed : seeds) {
                        reached = cover(current_box_id, cursor, seed, 0);
                        if (reached != current_box_id ||
                            added >= config.max_chain || context.should_stop()) {
                            break;
                        }
                    }
                    attempt_step *= 0.5;
                }
                if (reached == current_box_id) {
                    connected_reach_failures += 1;
                    break;
                }
                current_box_id = reached;
                if (BoxNode* reached_box = box_by_id(current_box_id)) {
                    cursor = reached_box->center();
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
        if (added >= config.max_chain) {
            context.diagnostics().add_counter("connector.chain_pave_connected_max_chain_hits");
        }
        return added;
    }

    if (config.fill_gaps && waypoint_path.size() >= 2 && fast_gap_fill) {
        using Clock = std::chrono::steady_clock;
        const auto fast_start = Clock::now();
        auto elapsed_ms = [&]() {
            return std::chrono::duration<double, std::milli>(Clock::now() -
                                                            fast_start)
                .count();
        };
        auto remaining_ms = [&]() {
            return config.gap_fill_time_budget_ms <= 0.0
                       ? std::numeric_limits<double>::infinity()
                       : std::max(0.0,
                                  config.gap_fill_time_budget_ms - elapsed_ms());
        };
        auto deadline_reached = [&]() {
            return context.should_stop() ||
                   (config.gap_fill_time_budget_ms > 0.0 &&
                    elapsed_ms() >= config.gap_fill_time_budget_ms);
        };

        struct GapCandidate {
            std::size_t seg = 0;
            double u = 0.0;
            double arc = 0.0;
            int rank = 0;
        };

        auto candidate_key = [](std::size_t seg, double u) -> std::size_t {
            const auto bin = static_cast<std::size_t>(
                std::max(0.0, std::min(1000000.0, std::round(u * 1000000.0))));
            return seg * 1000003ULL + bin;
        };

        std::unordered_set<std::size_t> rejected_candidates;
        const double probe_step = config.gap_fill_sample_step > 0.0
                                      ? config.gap_fill_sample_step
                                      : 0.05;

        auto clone_existing_leaf_coverage = [&]() {
            bool progressed = false;
            for (std::size_t seg = 1;
                 seg < waypoint_path.size() && !deadline_reached() &&
                 added < config.max_chain;
                 ++seg) {
                const Eigen::VectorXd& a = waypoint_path[seg - 1];
                const Eigen::VectorXd& b = waypoint_path[seg];
                const double seg_len = (b - a).norm();
                if (seg_len < 1e-12) {
                    continue;
                }
                const int n =
                    std::max(1, static_cast<int>(std::ceil(seg_len / probe_step)));
                for (int i = 0; i <= n && !deadline_reached() &&
                                added < config.max_chain;
                     ++i) {
                    const double u = static_cast<double>(i) / static_cast<double>(n);
                    const Eigen::VectorXd p = a + u * (b - a);
                    const int cover_id = clone_owned_node_cover(p, current_box_id);
                    if (cover_id >= 0) {
                        current_box_id = cover_id;
                        progressed = true;
                    }
                }
            }
            return progressed;
        };

        auto build_gap_candidates = [&]() {
            std::vector<GapCandidate> candidates;
            for (std::size_t seg = 1; seg < waypoint_path.size(); ++seg) {
                if (deadline_reached()) {
                    break;
                }
                const Eigen::VectorXd& a = waypoint_path[seg - 1];
                const Eigen::VectorXd& b = waypoint_path[seg];
                const double seg_len = (b - a).norm();
                if (seg_len < 1e-12) {
                    continue;
                }
                const int n =
                    std::max(1, static_cast<int>(std::ceil(seg_len / probe_step)));
                int run_begin = -1;
                int preferred_id = current_box_id;
                auto close_run = [&](int run_end) {
                    if (run_begin < 0 || run_end < run_begin) {
                        return;
                    }
                    const double arc =
                        (static_cast<double>(run_end - run_begin + 1) /
                         static_cast<double>(n)) *
                        seg_len;
                    const int mid = (run_begin + run_end) / 2;
                    for (int offset = 0; offset <= run_end - run_begin; ++offset) {
                        const int left = mid - offset;
                        const int right = mid + offset;
                        const int ids[2] = {left, right};
                        for (int side = 0; side < 2; ++side) {
                            const int sample_id = ids[side];
                            if (sample_id < run_begin || sample_id > run_end ||
                                (side == 1 && right == left)) {
                                continue;
                            }
                            const double u = static_cast<double>(sample_id) /
                                         static_cast<double>(n);
                            if (rejected_candidates.find(candidate_key(seg, u)) !=
                                rejected_candidates.end()) {
                                continue;
                            }
                            bool duplicate = false;
                            for (const auto& c : candidates) {
                                if (c.seg == seg && std::abs(c.u - u) < 1e-12) {
                                    duplicate = true;
                                    break;
                                }
                            }
                            if (!duplicate) {
                                candidates.push_back({seg, u, arc, offset});
                            }
                        }
                    }
                    run_begin = -1;
                };
                for (int i = 0; i <= n; ++i) {
                    const double u = static_cast<double>(i) / static_cast<double>(n);
                    const Eigen::VectorXd p = a + u * (b - a);
                    const int cover_id = find_existing_cover(p, preferred_id);
                    if (cover_id >= 0) {
                        preferred_id = cover_id;
                        close_run(i - 1);
                    } else if (run_begin < 0) {
                        run_begin = i;
                    }
                }
                close_run(n);
            }
            std::sort(candidates.begin(), candidates.end(),
                      [](const GapCandidate& lhs, const GapCandidate& rhs) {
                          if (lhs.rank != rhs.rank) {
                              return lhs.rank < rhs.rank;
                          }
                          return lhs.arc > rhs.arc;
                      });
            return candidates;
        };

        int ffb_calls = 0;
        const int max_ffb_calls = config.gap_fill_max_ffb_calls < 0
                                      ? std::numeric_limits<int>::max()
                                      : config.gap_fill_max_ffb_calls;

        clone_existing_leaf_coverage();

        struct EvaluatedCandidate {
            GapCandidate candidate;
            Eigen::VectorXd seed;
            FindFreeBoxResult result;
            double arc_gain = 0.0;
            bool reserved_cap_contains = false;
        };

        auto evaluate_candidate = [&](const GapCandidate& candidate,
                                      EvaluatedCandidate& evaluated) -> bool {
            const Eigen::VectorXd& a = waypoint_path[candidate.seg - 1];
            const Eigen::VectorXd& b = waypoint_path[candidate.seg];
            const double seg_len = (b - a).norm();
            if (seg_len < 1e-12) {
                return false;
            }
            const Eigen::VectorXd p = a + candidate.u * (b - a);
            const int existing = find_existing_cover(p, current_box_id);
            if (existing >= 0) {
                current_box_id = existing;
                return true;
            }

            FindFreeBoxOptions ffb_options = config.find_free_box;
            if (config.gap_fill_time_budget_ms > 0.0) {
                const double ms_left = remaining_ms();
                if (ms_left <= 0.0) {
                    return false;
                }
                ffb_options.deadline_ms = ms_left;
            }
            ++ffb_calls;
            auto result = ffb.find(p, context, ffb_options);
            const bool reserved_cap_contains =
                !result.found && result.hit_reserved_depth_cap &&
                intervals_contain_point(result.intervals, p,
                                        config.adjacency_tolerance);
            if (!result.found && !reserved_cap_contains) {
                return false;
            }

            const ParamSpan span = segment_interval_span(result.intervals, a, b);
            const double arc_gain = span.hit ? (span.hi - span.lo) * seg_len : 0.0;
            if (config.gap_fill_min_arc_gain > 0.0 &&
                arc_gain + 1e-12 < config.gap_fill_min_arc_gain) {
                return false;
            }

            evaluated.candidate = candidate;
            evaluated.seed = p;
            evaluated.result = std::move(result);
            evaluated.arc_gain = arc_gain;
            evaluated.reserved_cap_contains = reserved_cap_contains;
            return true;
        };

        auto commit_evaluated = [&](EvaluatedCandidate& evaluated) -> bool {
            if (deadline_reached() || added >= config.max_chain) {
                return false;
            }
            const int existing = find_existing_cover(evaluated.seed,
                                                     current_box_id);
            if (existing >= 0) {
                current_box_id = existing;
                return true;
            }

            int committed = -1;
            if (evaluated.result.found) {
                committed = commit_box(evaluated.result, evaluated.seed,
                                       current_box_id,
                                       /*require_adjacency=*/true);
                if (committed < 0) {
                    const int owner = find_box_owning_node_covering(evaluated.result.node, evaluated.seed);
                    BoxNode* owner_box = box_by_id(owner);
                    BoxNode* current_box = box_by_id(current_box_id);
                    const bool adjacent_owner =
                        owner_box != nullptr && current_box != nullptr &&
                        boxes_connected(*current_box, *owner_box,
                                        config.adjacency_tolerance);
                    if (owner_box != nullptr &&
                        owner_box->contains(evaluated.seed) &&
                        (!config.require_connected_chain || adjacent_owner)) {
                        committed = owner;
                    }
                }
                if (committed < 0 && !config.require_connected_chain) {
                    committed = commit_box(evaluated.result, evaluated.seed,
                                           current_box_id,
                                           /*require_adjacency=*/false,
                                           /*allow_duplicate_node=*/true);
                }
            } else {
                committed = commit_reserved_cap_box(evaluated.result,
                                                    evaluated.seed,
                                                    current_box_id);
            }
            if (committed >= 0) {
                current_box_id = committed;
                return true;
            }
            return false;
        };

        while (!deadline_reached() && added < config.max_chain &&
               ffb_calls < max_ffb_calls) {
            auto candidates = build_gap_candidates();
            if (candidates.empty()) {
                break;
            }
            std::vector<EvaluatedCandidate> evaluated;
            for (const auto& candidate : candidates) {
                if (deadline_reached() || added >= config.max_chain ||
                    ffb_calls >= max_ffb_calls) {
                    break;
                }
                if (config.gap_fill_time_budget_ms > 0.0 &&
                    remaining_ms() <= 0.25) {
                    break;
                }
                EvaluatedCandidate item;
                if (evaluate_candidate(candidate, item)) {
                    evaluated.push_back(std::move(item));
                }
                rejected_candidates.insert(candidate_key(candidate.seg,
                                                         candidate.u));
            }
            std::sort(evaluated.begin(), evaluated.end(),
                      [](const EvaluatedCandidate& lhs,
                         const EvaluatedCandidate& rhs) {
                          return lhs.arc_gain > rhs.arc_gain;
                      });
            bool progressed = false;
            for (auto& item : evaluated) {
                if (commit_evaluated(item)) {
                    progressed = true;
                }
            }
            if (!progressed) {
                break;
            }
        }
        context.diagnostics().set_value("connector.chain_pave_fast_ffb_calls",
                                        static_cast<double>(ffb_calls));
        context.diagnostics().set_value("connector.chain_pave_fast_ms",
                                        elapsed_ms());
        return added;
    }

    if (config.fill_gaps && waypoint_path.size() >= 2) {
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
            // Step in parameter space that corresponds to gap_fill_min_step in
            // C-space, so we never stall yet never overshoot the box face.
            const double param_eps =
                std::max(1e-9, config.gap_fill_min_step / seg_len);
            const double sample_param =
                config.gap_fill_sample_step > 0.0
                    ? std::min(1.0, std::max(param_eps,
                                             config.gap_fill_sample_step /
                                                 seg_len))
                    : std::min(1.0, std::max(param_eps,
                                             1.0 / std::max(
                                                       1,
                                                       config
                                                           .max_steps_per_waypoint)));
            double u = 0.0;
            int guard = 0;
            const int guard_max =
                std::max(1, static_cast<int>(std::ceil(1.0 / sample_param))) +
                1;
            while (u < 1.0 - param_eps && added < config.max_chain &&
                   !context.should_stop() && guard++ < guard_max) {
                const Eigen::VectorXd p = a + u * (b - a);
                int box_id = cover_point(p, current_box_id);
                if (box_id < 0) {
                    u = std::min(1.0, u + sample_param);
                    continue;
                }
                current_box_id = box_id;
                auto compute_exit = [&](int id) -> double {
                    auto map = make_box_map(boxes);
                    auto it = map.find(id);
                    return it != map.end()
                               ? segment_exit_param(*it->second, a, b, u)
                               : u;
                };
                const double u_exit = compute_exit(box_id);
                const double u_next_sample = std::min(1.0, u + sample_param);
                if (u_exit > u_next_sample + param_eps) {
                    // This box spans well beyond the next planned probe, so skip
                    // the already-covered region instead of spending budget there.
                    u = std::min(1.0, u_exit + param_eps);
                } else {
                    u = u_next_sample;
                }
            }
            // Ensure the segment endpoint itself is covered.
            if (added < config.max_chain && !context.should_stop()) {
                const int box_id = cover_point(b, current_box_id);
                if (box_id >= 0) {
                    current_box_id = box_id;
                }
            }
        }
        return added;
    }

    for (const auto& waypoint : waypoint_path) {
        if (context.should_stop()) {
            break;
        }
        if (added >= config.max_chain) {
            break;
        }
        auto map = make_box_map(boxes);
        auto current_it = map.find(current_box_id);
        if (current_it == map.end()) {
            break;
        }
        const bool covered_by_current = current_it->second->contains(waypoint);
        if (covered_by_current && !config.refine_covered_waypoints) {
            continue;
        }
        const Eigen::VectorXd start = current_it->second->center();
        const int step_count = covered_by_current ? 1 : config.max_steps_per_waypoint;
        for (int step = 1; step <= step_count && added < config.max_chain; ++step) {
            if (context.should_stop()) {
                break;
            }
            const double alpha = static_cast<double>(step) / static_cast<double>(step_count);
            const Eigen::VectorXd seed = covered_by_current ? waypoint : start + alpha * (waypoint - start);
            const int reached = cover(current_box_id, start, seed, 0);
            if (reached != current_box_id) {
                current_box_id = reached;
                break;
            }
        }
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
        context.diagnostics().add_counter("connector.bridge_broadphase_pairs", static_cast<double>(candidates.size()));
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
            if (result.bridge_boxes_added < config_.max_total_bridge_boxes) {
                context.diagnostics().add_counter("connector.chain_pave_attempts");
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
                    graph = compute_adjacency(boxes, config_.pave.adjacency_tolerance);
                    box_connected = graph_has_path(graph, chosen.source_box_id, chosen.target_box_id);
                    if (box_connected) {
                        context.diagnostics().add_counter("connector.chain_pave_box_connected");
                    } else {
                        context.diagnostics().add_counter("connector.chain_pave_partial_added");
                    }
                }
            }
            bool added_segment_edge = false;
            if (!box_connected &&
                config_.segment_edges_enabled && config_.rrt_segment_edges &&
                !config_.segment_edges_fallback_only) {
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
            graph = compute_adjacency(boxes, config_.pave.adjacency_tolerance);
        }
        apply_segment_edges_to_adjacency(segment_edges, graph);
        if (!progressed) {
            break;
        }
        islands = find_islands(graph);
    }
    while (find_islands(graph).size() > 1) {
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
