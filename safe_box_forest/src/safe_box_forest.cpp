#include <SBF/safe_box_forest.h>

#include <sbf/core/joint_symmetry.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace rbf {

namespace {

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
    auto lhs = oracle.query_intervals_for_node(
        oracle.root_node(),
        oracle.root_intervals(),
        start);
    auto rhs = oracle.query_intervals_for_node(
        oracle.root_node(),
        oracle.root_intervals(),
        goal);
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
    envelope_type.support_hull_config.keep_kdop = true;
    envelope_type.support_hull_config.safety_epsilon = 1e-9;

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

bool same_waypoint(const Eigen::VectorXd& lhs, const Eigen::VectorXd& rhs) {
    return lhs.size() == rhs.size() && (lhs - rhs).norm() <= 1e-10;
}

void append_waypoint_unique(std::vector<Eigen::VectorXd>& path, const Eigen::VectorXd& waypoint) {
    if (path.empty() || !same_waypoint(path.back(), waypoint)) {
        path.push_back(waypoint);
    }
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

struct DirtySeedCandidate {
    Eigen::VectorXd seed;
    int parent_box_id = -1;
    int root_id = -1;
};

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

bool seed_near_existing(const std::vector<DirtySeedCandidate>& candidates,
                        const Eigen::Ref<const Eigen::VectorXd>& seed,
                        double tolerance) {
    for (const auto& candidate : candidates) {
        if (candidate.seed.size() == seed.size() && (candidate.seed - seed).norm() <= tolerance) {
            return true;
        }
    }
    return false;
}

bool append_dirty_seed_candidate(std::vector<DirtySeedCandidate>& seeds,
                                 const std::vector<BoxNode>& live_boxes,
                                 const Eigen::Ref<const Eigen::VectorXd>& seed,
                                 int parent_box_id,
                                 int root_id,
                                 int limit,
                                 double dedup_tolerance) {
    if (static_cast<int>(seeds.size()) >= limit) {
        return false;
    }
    if (point_covered_by_existing_box_local(live_boxes, seed) || seed_near_existing(seeds, seed, dedup_tolerance)) {
        return false;
    }
    seeds.push_back(DirtySeedCandidate{seed, parent_box_id, root_id});
    return true;
}

struct SubtractiveSeedCandidate {
    Eigen::VectorXd seed;
    int parent_box_id = -1;
    int root_id = -1;
    int domain_index = -1;
};

struct LeafRefineSeedCandidate {
    Eigen::VectorXd seed;
    int parent_box_id = -1;
    int root_id = -1;
    int domain_index = -1;
    bool priority = false;
};

struct LeafRefineDomainRank {
    int index = -1;
    int adjacent_count = 0;
    double volume = 0.0;
    double priority_distance = std::numeric_limits<double>::infinity();
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

std::vector<LeafRefineDomainRank> rank_leaf_refine_domains(
    const std::vector<BoxNode>& domains,
    const std::vector<BoxNode>& live_boxes,
    const std::vector<Eigen::VectorXd>& priority_points,
    double tolerance) {
    (void)live_boxes;
    (void)tolerance;
    std::vector<LeafRefineDomainRank> ranks;
    ranks.reserve(domains.size());
    std::unordered_set<int> pinned;
    for (const auto& point : priority_points) {
        int best_index = -1;
        double best_volume = std::numeric_limits<double>::infinity();
        for (int index = 0; index < static_cast<int>(domains.size()); ++index) {
            if (pinned.find(index) != pinned.end()) {
                continue;
            }
            const auto& domain = domains[static_cast<std::size_t>(index)];
            if (point.size() != domain.n_dims() ||
                !intervals_contain_point_strict_local(domain.joint_intervals, point, 1e-9)) {
                continue;
            }
            if (domain.volume < best_volume) {
                best_volume = domain.volume;
                best_index = index;
            }
        }
        if (best_index >= 0) {
            const auto& domain = domains[static_cast<std::size_t>(best_index)];
            LeafRefineDomainRank rank;
            rank.index = best_index;
            rank.volume = domain.volume;
            rank.priority_distance = 0.0;
            ranks.push_back(rank);
            pinned.insert(best_index);
        }
    }
    std::vector<LeafRefineDomainRank> remaining;
    remaining.reserve(domains.size());
    for (int index = 0; index < static_cast<int>(domains.size()); ++index) {
        if (pinned.find(index) != pinned.end()) {
            continue;
        }
        const auto& domain = domains[static_cast<std::size_t>(index)];
        LeafRefineDomainRank rank;
        rank.index = index;
        rank.volume = domain.volume;
        rank.priority_distance = box_priority_point_distance(domain, priority_points);
        remaining.push_back(rank);
    }
    std::sort(remaining.begin(), remaining.end(), [](const auto& lhs, const auto& rhs) {
        if (lhs.adjacent_count != rhs.adjacent_count) {
            return lhs.adjacent_count > rhs.adjacent_count;
        }
        if (std::abs(lhs.priority_distance - rhs.priority_distance) > 1e-12) {
            return lhs.priority_distance < rhs.priority_distance;
        }
        if (std::abs(lhs.volume - rhs.volume) > 1e-18) {
            return lhs.volume > rhs.volume;
        }
        return lhs.index < rhs.index;
    });
    ranks.insert(ranks.end(), remaining.begin(), remaining.end());
    return ranks;
}

bool leaf_refine_seed_near_existing(const std::vector<LeafRefineSeedCandidate>& candidates,
                                    const Eigen::Ref<const Eigen::VectorXd>& seed,
                                    double tolerance) {
    for (const auto& candidate : candidates) {
        if (candidate.seed.size() == seed.size() && (candidate.seed - seed).norm() <= tolerance) {
            return true;
        }
    }
    return false;
}

bool append_leaf_refine_seed(std::vector<LeafRefineSeedCandidate>& seeds,
                             const std::vector<BoxNode>& live_boxes,
                             const BoxNode& domain,
                             int domain_index,
                             const Eigen::Ref<const Eigen::VectorXd>& raw_seed,
                             int parent_box_id,
                             int root_id,
                             int limit,
                             double epsilon,
                             double dedup_tolerance,
                             bool priority = false) {
    if (static_cast<int>(seeds.size()) >= limit) {
        return false;
    }
    Eigen::VectorXd seed = priority && raw_seed.size() == domain.n_dims() &&
            intervals_contain_point_strict_local(domain.joint_intervals, raw_seed, 1e-12)
        ? Eigen::VectorXd(raw_seed)
        : clamped_domain_seed(domain, raw_seed, epsilon);
    if (point_covered_by_existing_box_local(live_boxes, seed) ||
        leaf_refine_seed_near_existing(seeds, seed, dedup_tolerance)) {
        return false;
    }
    seeds.push_back(LeafRefineSeedCandidate{std::move(seed), parent_box_id, root_id, domain_index, priority});
    return true;
}

Eigen::VectorXd domain_face_seed_from_neighbor(const BoxNode& domain,
                                               const BoxNode& neighbor,
                                               double epsilon) {
    Eigen::VectorXd seed = domain.center();
    for (int dim = 0; dim < domain.n_dims(); ++dim) {
        const auto& di = domain.joint_intervals[static_cast<std::size_t>(dim)];
        const auto& ni = neighbor.joint_intervals[static_cast<std::size_t>(dim)];
        const double width = std::max(0.0, di.width());
        const double inset = std::min(std::max(epsilon, 1e-12), 0.25 * width);
        if (ni.hi <= di.lo + epsilon && width > 2.0 * inset) {
            seed[dim] = di.lo + inset;
        } else if (ni.lo >= di.hi - epsilon && width > 2.0 * inset) {
            seed[dim] = di.hi - inset;
        } else {
            const double lo = std::max(di.lo, ni.lo);
            const double hi = std::min(di.hi, ni.hi);
            seed[dim] = lo <= hi ? 0.5 * (lo + hi) : di.center();
        }
    }
    return clamped_domain_seed(domain, seed, epsilon);
}

std::vector<LeafRefineSeedCandidate> make_leaf_refine_domain_seeds(
    const BoxNode& domain,
    int domain_index,
    const std::vector<BoxNode>& live_boxes,
    const std::vector<Eigen::VectorXd>& priority_points,
    int limit,
    double adjacency_tolerance,
    double boundary_epsilon) {
    std::vector<LeafRefineSeedCandidate> seeds;
    if (limit <= 0) {
        return seeds;
    }
    seeds.reserve(static_cast<std::size_t>(std::min(limit, 16)));
    const double epsilon = std::max({boundary_epsilon, 2.0 * adjacency_tolerance, 1e-10});
    const double dedup_tolerance = std::max(1e-9, 4.0 * epsilon);
    for (const auto& point : priority_points) {
        if (static_cast<int>(seeds.size()) >= limit) {
            return seeds;
        }
        if (point.size() != domain.n_dims() ||
            !intervals_contain_point_strict_local(domain.joint_intervals, point, adjacency_tolerance)) {
            continue;
        }
        append_leaf_refine_seed(seeds,
                                live_boxes,
                                domain,
                                domain_index,
                                point,
                                -1,
                                domain.root_id >= 0 ? domain.root_id : domain.id,
                                limit,
                                epsilon,
                                dedup_tolerance,
                                true);
    }
    for (const auto& live : live_boxes) {
        if (!boxes_connected(domain, live, adjacency_tolerance)) {
            continue;
        }
        const Eigen::VectorXd seed = domain_face_seed_from_neighbor(domain, live, epsilon);
        append_leaf_refine_seed(seeds,
                                live_boxes,
                                domain,
                                domain_index,
                                seed,
                                live.id,
                                live.root_id >= 0 ? live.root_id : live.id,
                                limit,
                                epsilon,
                                dedup_tolerance);
        if (static_cast<int>(seeds.size()) >= limit) {
            return seeds;
        }
    }
    append_leaf_refine_seed(seeds,
                            live_boxes,
                            domain,
                            domain_index,
                            domain.center(),
                            -1,
                            domain.root_id >= 0 ? domain.root_id : domain.id,
                            limit,
                            epsilon,
                            dedup_tolerance);
    for (int dim = 0; dim < domain.n_dims() && static_cast<int>(seeds.size()) < limit; ++dim) {
        const auto& interval = domain.joint_intervals[static_cast<std::size_t>(dim)];
        const double width = std::max(0.0, interval.width());
        const double inset = std::min(std::max(epsilon, 1e-12), 0.25 * width);
        if (width <= 2.0 * inset) {
            continue;
        }
        Eigen::VectorXd lo_seed = domain.center();
        lo_seed[dim] = interval.lo + inset;
        append_leaf_refine_seed(seeds,
                                live_boxes,
                                domain,
                                domain_index,
                                lo_seed,
                                -1,
                                domain.root_id >= 0 ? domain.root_id : domain.id,
                                limit,
                                epsilon,
                                dedup_tolerance);
        if (static_cast<int>(seeds.size()) >= limit) {
            break;
        }
        Eigen::VectorXd hi_seed = domain.center();
        hi_seed[dim] = interval.hi - inset;
        append_leaf_refine_seed(seeds,
                                live_boxes,
                                domain,
                                domain_index,
                                hi_seed,
                                -1,
                                domain.root_id >= 0 ? domain.root_id : domain.id,
                                limit,
                                epsilon,
                                dedup_tolerance);
    }
    return seeds;
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

bool append_leaf_refine_target_step_seed(std::vector<LeafRefineSeedCandidate>& seeds,
                                         const std::vector<BoxNode>& live_boxes,
                                         const BoxNode& domain,
                                         int domain_index,
                                         const BoxNode& parent,
                                         const Eigen::VectorXd& target,
                                         int limit,
                                         double epsilon,
                                         double dedup_tolerance) {
    if (target.size() != domain.n_dims()) {
        return false;
    }
    Eigen::VectorXd seed = target;
    for (int dim = 0; dim < domain.n_dims(); ++dim) {
        const auto& interval = parent.joint_intervals[static_cast<std::size_t>(dim)];
        if (target[dim] < interval.lo) {
            seed[dim] = interval.lo - epsilon;
        } else if (target[dim] > interval.hi) {
            seed[dim] = interval.hi + epsilon;
        } else {
            seed[dim] = target[dim];
        }
    }
    return append_leaf_refine_seed(seeds,
                                   live_boxes,
                                   domain,
                                   domain_index,
                                   seed,
                                   parent.id,
                                   parent.root_id >= 0 ? parent.root_id : parent.id,
                                   limit,
                                   epsilon,
                                   dedup_tolerance,
                                   false);
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

bool make_dirty_face_seed(const BoxNode& box,
                          const std::vector<Interval>& root,
                          int dim,
                          int side,
                          double epsilon,
                          Eigen::VectorXd& seed) {
    if (dim < 0 || dim >= box.n_dims() || box.n_dims() != static_cast<int>(root.size())) {
        return false;
    }
    seed = box.center();
    const double value = side > 0
        ? box.joint_intervals[dim].hi + epsilon
        : box.joint_intervals[dim].lo - epsilon;
    if (!root[static_cast<std::size_t>(dim)].contains(value, 0.0)) {
        return false;
    }
    seed[dim] = value;
    for (int axis = 0; axis < box.n_dims(); ++axis) {
        seed[axis] = std::clamp(seed[axis],
                                root[static_cast<std::size_t>(axis)].lo,
                                root[static_cast<std::size_t>(axis)].hi);
    }
    return true;
}

void append_box_face_seeds(const BoxNode& source_box,
                           const std::vector<BoxNode>& live_boxes,
                           const std::vector<Interval>& root,
                           int parent_box_id,
                           int root_id,
                           int limit,
                           double epsilon,
                           double dedup_tolerance,
                           std::vector<DirtySeedCandidate>& seeds) {
    for (int dim = 0; dim < source_box.n_dims(); ++dim) {
        for (int side : {-1, 1}) {
            Eigen::VectorXd seed;
            if (!make_dirty_face_seed(source_box, root, dim, side, epsilon, seed)) {
                continue;
            }
            append_dirty_seed_candidate(seeds, live_boxes, seed, parent_box_id, root_id, limit, dedup_tolerance);
            if (static_cast<int>(seeds.size()) >= limit) {
                return;
            }
        }
    }
}

std::vector<DirtySeedCandidate> make_dirty_region_seeds(const std::vector<BoxNode>& boxes,
                                                        const std::vector<int>& dirty_indices,
                                                        const std::vector<Interval>& root,
                                                        const DynamicUpdateConfig& config,
                                                        double adjacency_tolerance,
                                                        double boundary_epsilon) {
    std::vector<DirtySeedCandidate> seeds;
    const int limit = std::max(0, config.dirty_seed_limit);
    if (limit == 0) {
        return seeds;
    }
    seeds.reserve(static_cast<std::size_t>(std::min(limit, 64)));
    const double epsilon = std::max({boundary_epsilon, 2.0 * adjacency_tolerance, 1e-10});
    const double dedup_tol = std::max(1e-9, 4.0 * epsilon);
    for (int index : dirty_indices) {
        if (index < 0 || index >= static_cast<int>(boxes.size())) {
            continue;
        }
        const BoxNode& box = boxes[static_cast<std::size_t>(index)];
        for (int dim = 0; dim < box.n_dims(); ++dim) {
            for (int side : {-1, 1}) {
                Eigen::VectorXd seed;
                if (!make_dirty_face_seed(box, root, dim, side, epsilon, seed)) {
                    continue;
                }
                if (point_covered_by_existing_box_local(boxes, seed) || seed_near_existing(seeds, seed, dedup_tol)) {
                    continue;
                }
                append_dirty_seed_candidate(seeds,
                                            boxes,
                                            seed,
                                            box.id,
                                            box.root_id >= 0 ? box.root_id : box.id,
                                            limit,
                                            dedup_tol);
                if (static_cast<int>(seeds.size()) >= limit) {
                    return seeds;
                }
            }
        }
    }
    return seeds;
}

std::vector<DirtySeedCandidate> make_insertion_regrow_seeds(const std::vector<BoxNode>& live_boxes,
                                                            const std::vector<BoxNode>& removed_boxes,
                                                            const std::unordered_set<int>& removed_box_ids,
                                                            const AdjacencyGraph& previous_adjacency,
                                                            const std::vector<Eigen::VectorXd>& anchor_points,
                                                            const std::vector<Interval>& root,
                                                            const DynamicUpdateConfig& config,
                                                            double adjacency_tolerance,
                                                            double boundary_epsilon) {
    std::vector<DirtySeedCandidate> seeds;
    const int limit = std::max(0, config.dirty_seed_limit);
    if (limit == 0) {
        return seeds;
    }
    seeds.reserve(static_cast<std::size_t>(std::min(limit, 64)));
    const double epsilon = std::max({boundary_epsilon, 2.0 * adjacency_tolerance, 1e-10});
    const double dedup_tol = std::max(1e-9, 4.0 * epsilon);
    std::unordered_map<int, const BoxNode*> live_by_id;
    live_by_id.reserve(live_boxes.size());
    for (const auto& box : live_boxes) {
        live_by_id[box.id] = &box;
    }
    for (const auto& removed : removed_boxes) {
        auto adjacency_it = previous_adjacency.find(removed.id);
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
                append_box_face_seeds(neighbor,
                                      live_boxes,
                                      root,
                                      neighbor.id,
                                      neighbor.root_id >= 0 ? neighbor.root_id : neighbor.id,
                                      limit,
                                      epsilon,
                                      dedup_tol,
                                      seeds);
                if (static_cast<int>(seeds.size()) >= limit) {
                    return seeds;
                }
            }
        }
        append_box_face_seeds(removed,
                              live_boxes,
                              root,
                              -1,
                              removed.root_id >= 0 ? removed.root_id : removed.id,
                              limit,
                              epsilon,
                              dedup_tol,
                              seeds);
        if (static_cast<int>(seeds.size()) >= limit) {
            return seeds;
        }
    }
    for (const auto& point : anchor_points) {
        if (point.size() != static_cast<int>(root.size())) {
            continue;
        }
        Eigen::VectorXd seed = point;
        for (int dim = 0; dim < seed.size(); ++dim) {
            seed[dim] = std::clamp(seed[dim], root[static_cast<std::size_t>(dim)].lo, root[static_cast<std::size_t>(dim)].hi);
        }
        append_dirty_seed_candidate(seeds, live_boxes, seed, -1, -1, limit, dedup_tol);
        if (static_cast<int>(seeds.size()) >= limit) {
            return seeds;
        }
    }
    const AdjacencyGraph live_adjacency = compute_adjacency(live_boxes, adjacency_tolerance);
    const auto islands = find_islands(live_adjacency);
    for (const auto& island : islands) {
        const BoxNode* best = nullptr;
        double best_distance = std::numeric_limits<double>::infinity();
        for (int box_id : island) {
            auto live_it = live_by_id.find(box_id);
            if (live_it == live_by_id.end()) {
                continue;
            }
            const Eigen::VectorXd center = live_it->second->center();
            for (const auto& removed : removed_boxes) {
                const double dist = (center - removed.center()).squaredNorm();
                if (dist < best_distance) {
                    best_distance = dist;
                    best = live_it->second;
                }
            }
        }
        if (best == nullptr) {
            continue;
        }
        append_box_face_seeds(*best,
                              live_boxes,
                              root,
                              best->id,
                              best->root_id >= 0 ? best->root_id : best->id,
                              limit,
                              epsilon,
                              dedup_tol,
                              seeds);
        if (static_cast<int>(seeds.size()) >= limit) {
            return seeds;
        }
    }
    return seeds;
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

std::vector<int> spatial_dirty_box_indices(const Robot& robot,
                                           const std::vector<BoxNode>& boxes,
                                           const std::vector<Obstacle>& removed_obstacles,
                                           const DynamicUpdateConfig& config,
                                           int& dirty_count);

std::vector<int> spatial_dirty_box_indices(const Robot& robot,
                                           const std::vector<BoxNode>& boxes,
                                           const Obstacle& removed_obstacle,
                                           const DynamicUpdateConfig& config,
                                           int& dirty_count) {
    return spatial_dirty_box_indices(robot,
                                     boxes,
                                     std::vector<Obstacle>{removed_obstacle},
                                     config,
                                     dirty_count);
}

std::vector<int> spatial_dirty_box_indices(const Robot& robot,
                                           const std::vector<BoxNode>& boxes,
                                           const std::vector<Obstacle>& removed_obstacles,
                                           const DynamicUpdateConfig& config,
                                           int& dirty_count) {
    dirty_count = 0;
    std::vector<int> dirty_indices;
    const int anchor_limit = std::max(0, config.dirty_anchor_limit);
    if (anchor_limit == 0 || boxes.empty() || removed_obstacles.empty()) {
        return dirty_indices;
    }
    std::vector<Obstacle> dirty_obstacles;
    dirty_obstacles.reserve(removed_obstacles.size());
    for (const auto& obstacle : removed_obstacles) {
        dirty_obstacles.push_back(inflate_obstacle(obstacle, config.dirty_region_padding));
    }
    CollisionChecker dirty_checker(robot, Scene(std::move(dirty_obstacles)));
    dirty_indices.reserve(static_cast<std::size_t>(std::min(anchor_limit, static_cast<int>(boxes.size()))));
    for (int index = 0; index < static_cast<int>(boxes.size()); ++index) {
        if (dirty_checker.check_box(boxes[static_cast<std::size_t>(index)].joint_intervals)) {
            dirty_count += 1;
            if (static_cast<int>(dirty_indices.size()) < anchor_limit) {
                dirty_indices.push_back(index);
            }
        }
    }
    return dirty_indices;
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
    result.segment_edge_length = 0.0;
    for (int edge_id : result.segment_edge_sequence) {
        if (edge_id < 0) {
            continue;
        }
        if (const SegmentEdge* edge = find_segment_edge_by_id(segment_edges, edge_id)) {
            result.segment_edge_length += edge->length;
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
                            const RRTConnectConfig& base_repair_config) {
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
                                       20260504 + 7919 * attempt + audit.failed_segment_index);
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
    for (std::size_t dim = 0; dim < override_intervals.size(); ++dim) {
        const Interval& allowed = (canonical_mode && !override_intervals.empty() &&
                                   dim < joint_limits.size() &&
                                   (override_intervals[dim].lo + 1e-12 < root_intervals[dim].lo ||
                                    override_intervals[dim].hi - 1e-12 > root_intervals[dim].hi))
            ? joint_limits[dim]
            : root_intervals[dim];
        const Interval& requested = override_intervals[dim];
        if (requested.lo > requested.hi) {
            std::ostringstream out;
            out << "database root_intervals_override[" << dim << "] is invalid: ["
                << requested.lo << ", " << requested.hi << "]";
            throw std::runtime_error(out.str());
        }
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

LeafSweepRefineResult RBFPlanningForest::build_leaf_sweep_refined(
    const std::vector<Obstacle>& obstacles,
    const LeafSweepRefineConfig& refine_config,
    const std::vector<Eigen::VectorXd>& priority_points) {
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

    out.leaf_sweep = build_leaf_sweep(obstacles,
                                      refine_config.leaf_start_depth,
                                      refine_config.leaf_max_depth,
                                      leaf_config);
    out.leaf_sweep_ms = out.leaf_sweep.total_ms;
    out.leaf_free_count = static_cast<int>(out.leaf_sweep.free_boxes.size());
    out.leaf_collision_count = static_cast<int>(out.leaf_sweep.collision_boxes.size());

    rebuild_adjacency();
    const auto refine_start = Clock::now();
    Deadline refine_deadline = refine_config.refine_timeout_ms > 0.0
        ? Deadline::after_ms(refine_config.refine_timeout_ms)
        : Deadline{};
    StageContext refine_context = StageContext::from_runtime(config_.runtime, refine_deadline);
    FindFreeBoxOptions refine_options = config_.grower.find_free_box;
    refine_options.max_depth = refine_config.deep_ffb_depth;
    refine_options.reject_seed_collision = false;

    const double adjacency_tolerance = config_.query.adjacency_tolerance;
    const double boundary_epsilon = std::max(1e-10, config_.grower.boundary_epsilon);
    const auto domain_ranks = rank_leaf_refine_domains(out.leaf_sweep.collision_boxes,
                                                       boxes_,
                                                       priority_points,
                                                       adjacency_tolerance);
    int next_id = next_box_id();
    std::unordered_map<int, int> domain_successes;
    domain_successes.reserve(std::min<std::size_t>(domain_ranks.size(), 4096));
    for (const auto& rank : domain_ranks) {
        if (refine_context.should_stop() ||
            out.deep_boxes_added >= std::max(0, refine_config.deep_max_boxes)) {
            break;
        }
        if (rank.index < 0 ||
            rank.index >= static_cast<int>(out.leaf_sweep.collision_boxes.size())) {
            continue;
        }
        const BoxNode& domain = out.leaf_sweep.collision_boxes[static_cast<std::size_t>(rank.index)];
        int attempts_in_domain = 0;
        while (!refine_context.should_stop() &&
               out.deep_boxes_added < std::max(0, refine_config.deep_max_boxes) &&
               attempts_in_domain < std::max(0, refine_config.domain_attempt_cap) &&
               domain_successes[rank.index] < std::max(0, refine_config.domain_success_cap)) {
            auto seeds = make_leaf_refine_domain_seeds(domain,
                                                       rank.index,
                                                       boxes_,
                                                       priority_points,
                                                       std::max(0, refine_config.domain_seed_cap),
                                                       adjacency_tolerance,
                                                       boundary_epsilon);
            const double epsilon = std::max({boundary_epsilon, 2.0 * adjacency_tolerance, 1e-10});
            const double dedup_tolerance = std::max(1e-9, 4.0 * epsilon);
            for (const auto& parent : boxes_) {
                if (!boxes_connected(domain, parent, adjacency_tolerance) &&
                    !intervals_overlap_local(parent.joint_intervals, domain.joint_intervals, adjacency_tolerance)) {
                    continue;
                }
                for (const auto& target : priority_points) {
                    if (static_cast<int>(seeds.size()) >= std::max(0, refine_config.domain_seed_cap)) {
                        break;
                    }
                    if (target.size() != domain.n_dims() ||
                        !intervals_contain_point_strict_local(domain.joint_intervals, target, adjacency_tolerance) ||
                        parent.contains(target, adjacency_tolerance)) {
                        continue;
                    }
                    append_leaf_refine_target_step_seed(seeds,
                                                        boxes_,
                                                        domain,
                                                        rank.index,
                                                        parent,
                                                        target,
                                                        std::max(0, refine_config.domain_seed_cap),
                                                        epsilon,
                                                        dedup_tolerance);
                }
                if (static_cast<int>(seeds.size()) >= std::max(0, refine_config.domain_seed_cap)) {
                    break;
                }
            }
            bool committed_in_round = false;
            for (const auto& candidate : seeds) {
            if (refine_context.should_stop() ||
                out.deep_boxes_added >= std::max(0, refine_config.deep_max_boxes) ||
                attempts_in_domain >= std::max(0, refine_config.domain_attempt_cap) ||
                domain_successes[rank.index] >= std::max(0, refine_config.domain_success_cap)) {
                break;
            }
            attempts_in_domain += 1;
            out.deep_domain_attempts += 1;
            if (point_covered_by_existing_box_local(boxes_, candidate.seed)) {
                out.deep_contained_rejects += 1;
                continue;
            }
            auto result = find_free_box_in_domain(candidate.seed,
                                                  domain.joint_intervals,
                                                  refine_context,
                                                  refine_options);
            if (!result.found) {
                out.deep_ffb_fail += 1;
                continue;
            }
            out.deep_ffb_success += 1;
            if (!intervals_subset_local(result.intervals, domain.joint_intervals, 1e-12)) {
                out.deep_domain_rejects += 1;
                continue;
            }
            if (!allow_dynamic_commit(*oracle_, result, config_.grower.commit_policy)) {
                out.deep_commit_rejects += 1;
                continue;
            }
            BoxNode box;
            box.id = next_id++;
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
                out.deep_contained_rejects += 1;
                continue;
            }

            int adjacent_parent = candidate.parent_box_id;
            if (adjacent_parent >= 0) {
                const BoxNode* parent = find_box_by_id(boxes_, adjacent_parent);
                if (parent == nullptr || !boxes_connected(*parent, box, adjacency_tolerance)) {
                    adjacent_parent = -1;
                }
            }
            const bool adjacent_to_forest = adjacent_parent >= 0 ||
                leaf_refine_has_adjacency(boxes_, box, adjacency_tolerance, &adjacent_parent);
            if (!adjacent_to_forest) {
                if (!candidate.priority || !refine_config.allow_anchor_roots) {
                    out.deep_adjacency_rejects += 1;
                    continue;
                }
                box.parent_box_id = -1;
                box.root_id = box.id;
                out.deep_anchor_roots_added += 1;
            } else {
                box.parent_box_id = adjacent_parent;
                const BoxNode* parent = find_box_by_id(boxes_, adjacent_parent);
                box.root_id = parent != nullptr && parent->root_id >= 0 ? parent->root_id : adjacent_parent;
            }
            oracle_->reserve_node(box.tree_id, box.id);
            boxes_.push_back(box);
            raw_boxes_.push_back(box);
            domain_successes[rank.index] += 1;
            out.deep_boxes_added += 1;
            committed_in_round = true;
            break;
            }
            if (!committed_in_round) {
                break;
            }
        }
    }
    out.deep_refine_ms = std::chrono::duration<double, std::milli>(Clock::now() - refine_start).count();

    const auto rrt_grower_start = Clock::now();
    bool rrt_grower_deadline_reached = false;
    int rrt_grower_initial_boxes = static_cast<int>(boxes_.size());
    if (refine_config.run_rrt_grower &&
        refine_config.rrt_grower_extra_boxes > 0 &&
        !boxes_.empty() &&
        !priority_points.empty()) {
        StageContext rrt_context = StageContext::from_runtime(
            config_.runtime,
            refine_config.rrt_grower_timeout_ms > 0.0
                ? Deadline::after_ms(refine_config.rrt_grower_timeout_ms)
                : Deadline{});
        GrowerConfig rrt_config = config_.grower;
        rrt_config.mode = GrowerConfig::Mode::RRT;
        rrt_config.max_boxes = static_cast<int>(boxes_.size()) +
            std::max(0, refine_config.rrt_grower_extra_boxes);
        if (refine_config.rrt_grower_timeout_ms > 0.0) {
            rrt_config.timeout_ms = refine_config.rrt_grower_timeout_ms;
        }
        rrt_config.stop_after_connect = false;
        rrt_config.root_seed_include_user_seeds = true;
        RrtGrower grower(*oracle_, rrt_config);
        const auto before_count = static_cast<int>(boxes_.size());
        rrt_grower_initial_boxes = before_count;
        auto grow = grower.grow_from_existing(boxes_, priority_points, rrt_context);
        if (!grow.boxes.empty()) {
            boxes_ = std::move(grow.boxes);
            raw_boxes_ = boxes_;
            adjacency_ = std::move(grow.adjacency);
        }
        out.rrt_grower_boxes_added =
            std::max(0, static_cast<int>(boxes_.size()) - before_count);
        out.rrt_grower_ffb_success = grow.n_ffb_success;
        out.rrt_grower_ffb_fail = grow.n_ffb_fail;
        rrt_grower_deadline_reached = grow.deadline_reached;
    }
    out.rrt_grower_ms =
        std::chrono::duration<double, std::milli>(Clock::now() - rrt_grower_start).count();

    const auto connector_start = Clock::now();
    bool connector_ran = false;
    rebuild_adjacency();
    if (config_.enable_connector && !boxes_.empty()) {
        StageContext connector_context = StageContext::from_runtime(config_.runtime);
        CollisionChecker checker(robot_, scene_);
        IslandConnectorConfig connector_config = config_.connector;
        if (connector_config.n_threads <= 1 && config_.runtime.n_threads > 1) {
            connector_config.n_threads = config_.runtime.n_threads;
        }
        IslandConnector connector(*oracle_, robot_, checker, connector_config);
        int connector_next_id = next_box_id();
        const auto connector_result = connector.connect_all(boxes_,
                                                            adjacency_,
                                                            segment_edges_,
                                                            connector_next_id,
                                                            connector_context);
        out.profile.bridge_boxes_added = connector_result.bridge_boxes_added;
        out.profile.segment_edges_added = connector_result.segment_edges_added;
        out.profile.rrt_segment_edges_added = connector_result.rrt_segment_edges_added;
        out.profile.point_gap_segment_edges_added = connector_result.point_gap_segment_edges_added;
        out.profile.connector_attempted_pairs = connector_result.attempted_pairs;
        out.profile.connector_connected = connector_result.connected;
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
    out.profile.diagnostics["leaf_refine.leaf_free_count"] = static_cast<double>(out.leaf_free_count);
    out.profile.diagnostics["leaf_refine.leaf_collision_count"] = static_cast<double>(out.leaf_collision_count);
    out.profile.diagnostics["leaf_refine.collision_cache_boxes"] =
        static_cast<double>(dynamic_collision_box_cache_.size());
    out.profile.diagnostics["leaf_refine.deep_refine_ms"] = out.deep_refine_ms;
    out.profile.diagnostics["leaf_refine.deep_boxes_added"] = static_cast<double>(out.deep_boxes_added);
    out.profile.diagnostics["leaf_refine.deep_domain_attempts"] = static_cast<double>(out.deep_domain_attempts);
    out.profile.diagnostics["leaf_refine.deep_ffb_success"] = static_cast<double>(out.deep_ffb_success);
    out.profile.diagnostics["leaf_refine.deep_ffb_fail"] = static_cast<double>(out.deep_ffb_fail);
    out.profile.diagnostics["leaf_refine.deep_commit_rejects"] = static_cast<double>(out.deep_commit_rejects);
    out.profile.diagnostics["leaf_refine.deep_domain_rejects"] = static_cast<double>(out.deep_domain_rejects);
    out.profile.diagnostics["leaf_refine.deep_contained_rejects"] = static_cast<double>(out.deep_contained_rejects);
    out.profile.diagnostics["leaf_refine.deep_adjacency_rejects"] = static_cast<double>(out.deep_adjacency_rejects);
    out.profile.diagnostics["leaf_refine.deep_anchor_roots_added"] = static_cast<double>(out.deep_anchor_roots_added);
    out.profile.diagnostics["leaf_refine.rrt_grower_ms"] = out.rrt_grower_ms;
    out.profile.diagnostics["leaf_refine.rrt_grower_initial_boxes"] =
        static_cast<double>(rrt_grower_initial_boxes);
    out.profile.diagnostics["leaf_refine.rrt_grower_boxes_added"] = static_cast<double>(out.rrt_grower_boxes_added);
    out.profile.diagnostics["leaf_refine.rrt_grower_ffb_success"] = static_cast<double>(out.rrt_grower_ffb_success);
    out.profile.diagnostics["leaf_refine.rrt_grower_ffb_fail"] = static_cast<double>(out.rrt_grower_ffb_fail);
    out.profile.diagnostics["leaf_refine.rrt_grower_deadline_reached"] =
        rrt_grower_deadline_reached ? 1.0 : 0.0;
    out.profile.diagnostics["leaf_refine.connector_ms"] = out.connector_ms;
    out.profile.diagnostics["leaf_refine.total_ms"] = out.total_ms;
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
    if (options.reject_seed_collision && oracle_->point_in_collision(seed)) {
        result.seed_collision = true;
        result.fail_code = 1;
        return result;
    }

    auto elapsed_ms = [&]() {
        return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
    };
    auto choose_domain_boundary_split = [&](const std::vector<Interval>& intervals,
                                            int& split_dim,
                                            double& split_value) {
        split_dim = -1;
        split_value = 0.0;
        double best_excess = 0.0;
        const double split_tol = 1e-12;
        for (int dim = 0; dim < static_cast<int>(intervals.size()); ++dim) {
            const auto& current = intervals[static_cast<std::size_t>(dim)];
            const auto& target = domain[static_cast<std::size_t>(dim)];
            const double low_excess = std::max(0.0, target.lo - current.lo);
            if (low_excess > best_excess && target.lo > current.lo + split_tol && target.lo < current.hi - split_tol) {
                best_excess = low_excess;
                split_dim = dim;
                split_value = target.lo;
            }
            const double high_excess = std::max(0.0, current.hi - target.hi);
            if (high_excess > best_excess && target.hi > current.lo + split_tol && target.hi < current.hi - split_tol) {
                best_excess = high_excess;
                split_dim = dim;
                split_value = target.hi;
            }
        }
        return split_dim >= 0;
    };

    const int effective_max_depth = std::max(0, std::min(options.max_depth, oracle_->max_tree_depth() - 1));
    // Seed-independent: canonical split depends only on (robot, domain). No
    // query-seed coupling is applied to the split values.
    OracleSplitOptions split_options = options.split;
    OracleNodeId node = oracle_->root_node();
    int changed_dim = -1;
    while (true) {
        if (context.should_stop() || (options.deadline_ms > 0.0 && elapsed_ms() > options.deadline_ms)) {
            result.deadline_reached = context.deadline().expired() || options.deadline_ms > 0.0;
            result.fail_code = 4;
            break;
        }

        auto intervals = oracle_->node_intervals(node);
        if (!intervals_overlap_local(intervals, domain, 0.0)) {
            result.node = node;
            result.intervals = std::move(intervals);
            result.fail_code = 5;
            break;
        }
        if (!oracle_->is_leaf(node)) {
            changed_dim = oracle_->split_dim(node);
            node = (seed[changed_dim] <= oracle_->split_value(node))
                ? oracle_->left_child(node)
                : oracle_->right_child(node);
            continue;
        }

        if (!intervals_subset_local(intervals, domain, 1e-12)) {
            if (oracle_->depth(node) >= effective_max_depth) {
                result.hit_unknown_depth_cap = true;
                result.node = node;
                result.intervals = std::move(intervals);
                result.fail_code = 2;
                break;
            }
            int split_dim = -1;
            double split_value = 0.0;
            if (!choose_domain_boundary_split(intervals, split_dim, split_value)) {
                result.node = node;
                result.intervals = std::move(intervals);
                result.fail_code = 6;
                break;
            }
            const auto split = oracle_->split_node_at(node, split_dim, split_value);
            if (!split.split) {
                result.node = node;
                result.intervals = std::move(intervals);
                result.fail_code = 6;
                break;
            }
            result.splits += 1;
            changed_dim = split_dim;
            node = (seed[split_dim] <= split_value) ? split.left : split.right;
            continue;
        }

        if (oracle_->is_reserved(node)) {
            if (oracle_->depth(node) >= effective_max_depth || !options.split_reserved_leaf) {
                result.hit_reserved_depth_cap = true;
                result.node = node;
                result.intervals = std::move(intervals);
                result.fail_code = 2;
                break;
            }
            const auto split = oracle_->split_node(node, intervals, changed_dim, split_options);
            if (!split.split) {
                result.fail_code = 6;
                break;
            }
            result.splits += 1;
            changed_dim = oracle_->split_dim(node);
            node = (seed[changed_dim] <= oracle_->split_value(node))
                ? oracle_->left_child(node)
                : oracle_->right_child(node);
            continue;
        }

        const auto validation = oracle_->validate_node(node, intervals, changed_dim);
        result.validation_detail = oracle_->last_validation_detail();
        result.decisions += 1;
        if (validation == BoxValidation::Free) {
            result.found = true;
            result.node = node;
            result.changed_dim = changed_dim;
            result.intervals = std::move(intervals);
            result.fail_code = 0;
            break;
        }
        if (validation == BoxValidation::Occupied) {
            result.node = node;
            result.intervals = std::move(intervals);
            result.fail_code = 3;
            break;
        }
        if (oracle_->depth(node) >= effective_max_depth || !options.split_unknown_leaf) {
            result.hit_unknown_depth_cap = true;
            result.node = node;
            result.intervals = std::move(intervals);
            result.fail_code = 2;
            break;
        }
        const auto split = oracle_->split_node(node, intervals, changed_dim, split_options);
        if (!split.split) {
            result.fail_code = 6;
            break;
        }
        result.splits += 1;
        changed_dim = oracle_->split_dim(node);
        node = (seed[changed_dim] <= oracle_->split_value(node))
            ? oracle_->left_child(node)
            : oracle_->right_child(node);
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
        const Item item = stack.back();
        stack.pop_back();
        if (item.node < 0) {
            continue;
        }
        std::vector<Interval> intervals = oracle_->node_intervals(item.node);
        if (!intervals_subset_local(intervals, removed_box.joint_intervals, 1e-12)) {
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
        const auto split = oracle_->split_node(item.node, intervals, item.changed_dim, split_options);
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
    CorridorQuery query_engine(query_config);
    QueryResult result = query_engine.run(query_cache(), start, goal);
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
        std::vector<Eigen::VectorXd> repair_path = rrt_connect(start, goal, checker, audit_robot_, repair_config, 20260511);
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
        } else if (query_config.repair_on_audit_failure) {
            const auto repair_t0 = Clock::now();
            const RRTConnectConfig repair_domain_config = oracle_
                ? with_query_root_hull_domain(config_.connector.rrt, *oracle_, start, goal)
                : config_.connector.rrt;
            const bool repaired = try_local_birrt_repair(result, audit, checker, audit_robot_, query_config, repair_domain_config);
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
    return result;
}

int RBFPlanningForest::bridge_query(const Eigen::Ref<const Eigen::VectorXd>& start,
                                const Eigen::Ref<const Eigen::VectorXd>& goal) {
    if (boxes_.empty() || !oracle_) {
        return 0;
    }
    QueryResult current = query(start, goal);
    if (current.success && current.repair_count == 0) {
        return 0;
    }
    return bridge_query_known_needed(start, goal);
}

int RBFPlanningForest::bridge_query_known_needed(const Eigen::Ref<const Eigen::VectorXd>& start,
                                             const Eigen::Ref<const Eigen::VectorXd>& goal) {
    if (boxes_.empty() || !oracle_) {
        return 0;
    }
    const int start_box_id = locate_containing_box(query_cache(), start, config_.query.nearest_if_outside);
    if (start_box_id < 0) {
        return 0;
    }
    const int goal_box_id = locate_containing_box(query_cache(), goal, config_.query.nearest_if_outside);
    if (goal_box_id < 0 || goal_box_id == start_box_id) {
        return 0;
    }
    StageContext context = StageContext::from_runtime(config_.runtime);
    CollisionChecker checker = make_audit_checker(audit_robot_, scene_, config_.query);
    RRTConnectConfig bridge_rrt = with_query_root_hull_domain(config_.connector.rrt, *oracle_, start, goal);
    bridge_rrt.segment_resolution = std::max(bridge_rrt.segment_resolution, config_.query.audit_resolution);
    auto waypoint_path = rrt_connect(start, goal, checker, audit_robot_, context, bridge_rrt, 20260503);
    if (waypoint_path.empty()) {
        return 0;
    }
    if (!audit_waypoint_path(waypoint_path,
                             checker,
                             config_.query.audit_resolution,
                             config_.query.audit_segment_step)
             .passed) {
        return 0;
    }
    int direct_segment_edges_added = 0;
    const bool defer_query_segment_edge = config_.connector.segment_edges_fallback_only;
    if (!defer_query_segment_edge && config_.connector.segment_edges_enabled && config_.connector.rrt_segment_edges) {
        const int edge_id = add_segment_edge(segment_edges_,
                                             adjacency_,
                                             start_box_id,
                                             goal_box_id,
                                             waypoint_path,
                                             SegmentEdgeType::QueryBridge,
                                             bridge_rrt.segment_resolution,
                                             SegmentEdgeValidation::CollisionChecked,
                                             false);
        direct_segment_edges_added = edge_id >= 0 ? 1 : 0;
    }
    int next_id = next_box_id();
    ChainPaveConfig pave_config = config_.connector.pave;
    if (defer_query_segment_edge) {
        pave_config.max_chain = std::max(pave_config.max_chain, 256);
        pave_config.refine_covered_waypoints = true;
        pave_config.fill_gaps = true;
        pave_config.find_free_box.max_depth = std::max(pave_config.find_free_box.max_depth, 64);
        pave_config.gap_fill_sample_step = std::min(pave_config.gap_fill_sample_step, 0.02);
        pave_config.gap_fill_time_budget_ms = std::max(pave_config.gap_fill_time_budget_ms, 200.0);
        pave_config.gap_fill_max_ffb_calls = std::max(pave_config.gap_fill_max_ffb_calls, 512);
        pave_config.gap_fill_min_arc_gain = 0.0;
        pave_config.require_connected_chain = true;
    }
    const int added = chain_pave_along_path(
        waypoint_path,
        start_box_id,
        boxes_,
        *oracle_,
        adjacency_,
        next_id,
        context,
        pave_config);
    if (added > 0) {
        rebuild_adjacency();
    }
    IslandConnectorConfig gap_config = config_.connector;
    gap_config.max_total_bridge_boxes = 0;
    IslandConnector gap_connector(*oracle_, robot_, checker, gap_config);
    const auto gap_result = gap_connector.connect_all(boxes_, adjacency_, segment_edges_, next_id, context);
    (void)gap_result;
    invalidate_query_cache();
    if (defer_query_segment_edge) {
        QueryResult box_retry = run_query_internal(start, goal, false);
        if (box_retry.success && box_retry.repair_count == 0 && box_retry.segment_edges_used == 0) {
            return added;
        }
        if (config_.connector.segment_edges_enabled && config_.connector.rrt_segment_edges) {
            const int source_box_id = locate_containing_box(query_cache(), start, config_.query.nearest_if_outside);
            const int target_box_id = locate_containing_box(query_cache(), goal, config_.query.nearest_if_outside);
            if (source_box_id >= 0 && target_box_id >= 0) {
                const int edge_id = add_segment_edge(segment_edges_,
                                                     adjacency_,
                                                     source_box_id,
                                                     target_box_id,
                                                     waypoint_path,
                                                     SegmentEdgeType::QueryBridge,
                                                     bridge_rrt.segment_resolution,
                                                     SegmentEdgeValidation::CollisionChecked,
                                                     true);
                direct_segment_edges_added = edge_id >= 0 ? 1 : 0;
                if (direct_segment_edges_added > 0) {
                    invalidate_query_cache();
                }
            }
        }
    }
    return added + direct_segment_edges_added;
}

DebugChainPaveResult RBFPlanningForest::debug_chain_pave(const Eigen::Ref<const Eigen::VectorXd>& start,
                                                         const Eigen::Ref<const Eigen::VectorXd>& goal,
                                                         const ChainPaveConfig& pave) {
    DebugChainPaveResult out;
    if (boxes_.empty() || !oracle_) {
        return out;
    }
    const int start_box_id = locate_containing_box(query_cache(), start, config_.query.nearest_if_outside);
    if (start_box_id < 0) {
        return out;
    }
    const int goal_box_id = locate_containing_box(query_cache(), goal, config_.query.nearest_if_outside);
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
    auto waypoint_path = rrt_connect(start, goal, checker, audit_robot_, context, bridge_rrt, 20260503);
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
    int next_id = next_box_id();
    out.added = chain_pave_along_path(
        waypoint_path,
        start_box_id,
        boxes_,
        *oracle_,
        adjacency_,
        next_id,
        context,
        pave);
    out.fast_gap_fill_ffb_calls = static_cast<int>(
        context.diagnostics().value("connector.chain_pave_fast_ffb_calls", 0.0));
    out.fast_gap_fill_ms =
        context.diagnostics().value("connector.chain_pave_fast_ms", 0.0);
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
        rebuild_adjacency();
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
        waypoint_path = probe.path;
    } else if (probe.success && probe.audit_passed && !probe.path.empty()) {
        if (mode == CorridorRefineMode::BoxOnlyLongPath) {
            return 0;
        }
        waypoint_path = probe.path;
        add_query_segment_edge = true;
    } else {
        StageContext rrt_context = StageContext::serial();
        RRTConnectConfig refine_rrt = with_query_root_hull_domain(config_.connector.rrt, *oracle_, start, goal);
        refine_rrt.segment_resolution = std::max(refine_rrt.segment_resolution, config_.query.audit_resolution);
        waypoint_path = rrt_connect(start, goal, checker, audit_robot_, rrt_context, refine_rrt, 20260505);
        add_query_segment_edge = mode != CorridorRefineMode::BoxOnlyLongPath;
    }
    if (waypoint_path.empty()) {
        return 0;
    }
    const int anchor_box_id = locate_containing_box(query_cache(), waypoint_path.front(), config_.query.nearest_if_outside);
    if (anchor_box_id < 0) {
        return 0;
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
        rebuild_adjacency();
        if (add_query_segment_edge &&
            config_.connector.segment_edges_enabled &&
            config_.connector.rrt_segment_edges &&
            audit_waypoint_path(waypoint_path,
                                checker,
                                config_.query.audit_resolution,
                                config_.query.audit_segment_step)
                .passed) {
            const int source_box_id = locate_containing_box(query_cache(), start, config_.query.nearest_if_outside);
            const int target_box_id = locate_containing_box(query_cache(), goal, config_.query.nearest_if_outside);
            if (source_box_id >= 0 && target_box_id >= 0) {
                add_segment_edge(segment_edges_,
                                 adjacency_,
                                 source_box_id,
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
            const BoxNode* source_box = find_box_by_id(boxes_, edge.source_box_id);
            const BoxNode* target_box = find_box_by_id(boxes_, edge.target_box_id);
            if (source_box == nullptr || target_box == nullptr ||
                !boxes_connected(*source_box, *target_box, config_.query.adjacency_tolerance)) {
                remove_local_edge(adjacency_, edge.source_box_id, edge.target_box_id);
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

    const auto adj_t0 = Clock::now();
    remove_adjacency_nodes(adjacency_, removed_box_ids);
    connect_incremental_boxes(adjacency_, boxes_, first_new_box_index, config_.query.adjacency_tolerance);
    apply_segment_edges_to_adjacency(segment_edges_, adjacency_);
    invalidate_query_cache();
    profile.adjacency_ms = std::chrono::duration<double, std::milli>(Clock::now() - adj_t0).count();
    profile.boxes_after = static_cast<int>(boxes_.size());
    profile.raw_boxes_after = static_cast<int>(raw_boxes_.size());
    profile.obstacles_after = scene_.n_obstacles();
    profile.adjacency_islands = static_cast<int>(find_islands(adjacency_).size());
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
        profile.adjacency_islands = static_cast<int>(find_islands(adjacency_).size());
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
            const BoxNode* source_box = find_box_by_id(boxes_, edge.source_box_id);
            const BoxNode* target_box = find_box_by_id(boxes_, edge.target_box_id);
            if (source_box == nullptr || target_box == nullptr ||
                !boxes_connected(*source_box, *target_box, config_.query.adjacency_tolerance)) {
                remove_local_edge(adjacency_, edge.source_box_id, edge.target_box_id);
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

    const auto adj_t0 = Clock::now();
    remove_adjacency_nodes(adjacency_, removed_box_ids);
    connect_incremental_boxes(adjacency_, boxes_, first_new_box_index, config_.query.adjacency_tolerance);
    apply_segment_edges_to_adjacency(segment_edges_, adjacency_);
    invalidate_query_cache();
    profile.adjacency_ms = std::chrono::duration<double, std::milli>(Clock::now() - adj_t0).count();
    profile.boxes_after = static_cast<int>(boxes_.size());
    profile.raw_boxes_after = static_cast<int>(raw_boxes_.size());
    profile.obstacles_after = scene_.n_obstacles();
    profile.adjacency_islands = static_cast<int>(find_islands(adjacency_).size());
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
    profile.diagnostics["segment_fallback.islands_before"] = static_cast<double>(find_islands(adjacency_).size());

    if (!oracle_ || boxes_.empty()) {
        profile.boxes_after = profile.boxes_before;
        profile.raw_boxes_after = profile.raw_boxes_before;
        profile.adjacency_islands = static_cast<int>(find_islands(adjacency_).size());
        profile.fallback_reason = boxes_.empty() ? "empty_forest" : "missing_oracle";
        profile.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
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
    profile.diagnostics["segment_fallback.islands_before"] = static_cast<double>(find_islands(adjacency_).size());

    if (!oracle_ || boxes_.empty() || start.size() != oracle_->n_dims() || goal.size() != oracle_->n_dims()) {
        profile.boxes_after = profile.boxes_before;
        profile.raw_boxes_after = profile.raw_boxes_before;
        profile.adjacency_islands = static_cast<int>(find_islands(adjacency_).size());
        profile.fallback_reason = boxes_.empty() ? "empty_forest" : !oracle_ ? "missing_oracle" : "bad_endpoint_dimension";
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
        profile.adjacency_islands = static_cast<int>(find_islands(adjacency_).size());
        profile.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
        return profile;
    }
    profile.obstacles_after = scene_.n_obstacles();
    reset_oracle(scene_);
    reserve_existing_boxes();
    const std::unordered_set<int> removed_indices{obstacle_index};
    const std::size_t first_new_box_index = boxes_.size();
    promote_unblocked_collision_cache(removed_indices, profile);

    const auto adj_t0_delete = Clock::now();
    if (profile.boxes_added > 0) {
        connect_incremental_boxes(adjacency_, boxes_, first_new_box_index, config_.query.adjacency_tolerance);
        apply_segment_edges_to_adjacency(segment_edges_, adjacency_);
        invalidate_query_cache();
    }
    profile.adjacency_ms = std::chrono::duration<double, std::milli>(Clock::now() - adj_t0_delete).count();
    profile.boxes_after = static_cast<int>(boxes_.size());
    profile.raw_boxes_after = static_cast<int>(raw_boxes_.size());
    profile.adjacency_islands = static_cast<int>(find_islands(adjacency_).size());
    profile.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
    invalidate_query_cache();
    return profile;

    const auto dirty_t0 = Clock::now();
    std::vector<int> dirty_indices;
    if (config_.dynamic_update.enable_spatial_dirty_region) {
        profile.used_spatial_dirty_region = true;
        dirty_indices = spatial_dirty_box_indices(robot_, boxes_, removed_obstacle, config_.dynamic_update, profile.dirty_boxes);
        profile.dirty_boxes_used = static_cast<int>(dirty_indices.size());
    }
    profile.dirty_region_ms = std::chrono::duration<double, std::milli>(Clock::now() - dirty_t0).count();

    auto fallback_seeds = [&]() {
        std::vector<Eigen::VectorXd> seeds = last_build_seeds_;
        if (seeds.empty()) {
            const int limit = std::max(1, config_.dynamic_update.dirty_seed_limit);
            seeds.reserve(static_cast<std::size_t>(std::min(limit, static_cast<int>(boxes_.size()))));
            for (const auto& box : boxes_) {
                seeds.push_back(box.center());
                if (static_cast<int>(seeds.size()) >= limit) {
                    break;
                }
            }
        }
        return seeds;
    };

    auto run_warm_rebuild = [&](std::string reason) {
        const auto rebuild_t0 = Clock::now();
        auto seeds = fallback_seeds();
        if (seeds.empty()) {
            profile.fallback_reason = "warm_rebuild_skipped_no_seeds";
            return false;
        }
        profile.used_warm_rebuild = true;
        profile.fallback_reason = std::move(reason);
        StageContext context = StageContext::from_runtime(config_.runtime);
        const auto before_build = profile.boxes_before;
        const auto build_profile = build_coverage(scene_.obstacles(), seeds, context);
        (void)build_profile;
        profile.warm_rebuild_ms = std::chrono::duration<double, std::milli>(Clock::now() - rebuild_t0).count();
        profile.boxes_after = static_cast<int>(boxes_.size());
        profile.raw_boxes_after = static_cast<int>(raw_boxes_.size());
        profile.boxes_added = std::max(0, profile.boxes_after - before_build);
        profile.raw_boxes_added = std::max(0, profile.raw_boxes_after - profile.raw_boxes_before);
        profile.boxes_removed = std::max(0, before_build - profile.boxes_after);
        profile.raw_boxes_removed = std::max(0, profile.raw_boxes_before - profile.raw_boxes_after);
        profile.collision_cache_boxes_after = static_cast<int>(dynamic_collision_box_cache_.size());
        profile.adjacency_islands = last_build_.adjacency_islands;
        return true;
    };

    const bool too_many_dirty = config_.dynamic_update.enable_warm_rebuild_fallback &&
        ((config_.dynamic_update.warm_rebuild_dirty_box_threshold > 0 &&
          profile.dirty_boxes > config_.dynamic_update.warm_rebuild_dirty_box_threshold) ||
         (profile.boxes_before > 0 && config_.dynamic_update.warm_rebuild_dirty_box_fraction >= 0.0 &&
          static_cast<double>(profile.dirty_boxes) / static_cast<double>(profile.boxes_before) >
              config_.dynamic_update.warm_rebuild_dirty_box_fraction));
    const bool empty_forest = config_.dynamic_update.enable_warm_rebuild_fallback &&
        config_.dynamic_update.warm_rebuild_on_empty_forest && profile.boxes_before == 0;
    const bool empty_dirty = config_.dynamic_update.enable_warm_rebuild_fallback &&
        config_.dynamic_update.warm_rebuild_on_empty_dirty_region && profile.boxes_before > 0 && profile.dirty_boxes == 0;
    if (too_many_dirty || empty_forest || empty_dirty) {
        const char* reason = too_many_dirty ? "dirty_region_too_large"
            : empty_forest ? "empty_forest_after_obstacle_removal"
            : "empty_dirty_region";
        if (run_warm_rebuild(reason)) {
            profile.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
            invalidate_query_cache();
            return profile;
        }
    }
    const auto regrow_t0 = Clock::now();
    const auto seed_candidates = make_dirty_region_seeds(boxes_,
                                                         dirty_indices,
                                                         oracle_->root_intervals(),
                                                         config_.dynamic_update,
                                                         config_.query.adjacency_tolerance,
                                                         config_.grower.boundary_epsilon);
    profile.dirty_seed_count = static_cast<int>(seed_candidates.size());
    if (!seed_candidates.empty() && config_.dynamic_update.local_regrow_box_limit > 0) {
        const Deadline deadline = config_.dynamic_update.local_regrow_timeout_ms > 0.0
            ? Deadline::after_ms(config_.dynamic_update.local_regrow_timeout_ms)
            : Deadline{};
        StageContext context = StageContext::from_runtime(config_.runtime, deadline);
        FindFreeBoxService ffb(*oracle_);
        FindFreeBoxOptions options = config_.grower.find_free_box;
        int next_id = next_box_id();
        for (const auto& candidate : seed_candidates) {
            if (context.should_stop() || profile.boxes_added >= config_.dynamic_update.local_regrow_box_limit) {
                break;
            }
            if (point_covered_by_existing_box_local(boxes_, candidate.seed)) {
                continue;
            }
            profile.regrow_attempts += 1;
            auto result = ffb.find(candidate.seed, context, options);
            if (!result.found || !intervals_contain_point_local(result.intervals, candidate.seed, config_.query.adjacency_tolerance)) {
                continue;
            }
            if (!allow_dynamic_commit(*oracle_, result, config_.grower.commit_policy)) {
                continue;
            }
            BoxNode box;
            box.id = next_id++;
            box.joint_intervals = std::move(result.intervals);
            box.seed_config = candidate.seed;
            box.tree_id = result.node;
            box.parent_box_id = candidate.parent_box_id;
            box.root_id = candidate.root_id >= 0 ? candidate.root_id : box.id;
            box.safety_status = result.validation_detail.safety_status;
            box.strict_audit_required = result.validation_detail.strict_audit_required;
            box.compute_volume();
            bool contained = false;
            for (const auto& existing : boxes_) {
                if (box_contains_box_exact_local(existing, box)) {
                    contained = true;
                    break;
                }
            }
            if (contained) {
                continue;
            }
            if (candidate.parent_box_id >= 0) {
                const BoxNode* parent = find_box_by_id(boxes_, candidate.parent_box_id);
                if (parent != nullptr && !boxes_connected(*parent, box, config_.query.adjacency_tolerance)) {
                    const double gap = std::sqrt(interval_bounds_gap_squared_local(parent->joint_intervals, box.joint_intervals));
                    if (gap > config_.grower.boundary_epsilon + config_.query.adjacency_tolerance) {
                        continue;
                    }
                }
            }
            oracle_->reserve_node(box.tree_id, box.id);
            boxes_.push_back(box);
            raw_boxes_.push_back(std::move(box));
            profile.boxes_added += 1;
            profile.raw_boxes_added += 1;
        }
    }
    profile.regrow_ms = std::chrono::duration<double, std::milli>(Clock::now() - regrow_t0).count();

    const bool local_below_min = config_.dynamic_update.enable_warm_rebuild_fallback &&
        config_.dynamic_update.warm_rebuild_min_local_boxes_added >= 0 &&
        profile.regrow_attempts > 0 &&
        profile.boxes_added < config_.dynamic_update.warm_rebuild_min_local_boxes_added;
    if (local_below_min && run_warm_rebuild("local_regrow_below_minimum")) {
        profile.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
        invalidate_query_cache();
        return profile;
    }

    const auto adj_t0 = Clock::now();
    if (profile.boxes_added > 0) {
        rebuild_adjacency();
    }
    profile.adjacency_ms = std::chrono::duration<double, std::milli>(Clock::now() - adj_t0).count();
    profile.boxes_after = static_cast<int>(boxes_.size());
    profile.raw_boxes_after = static_cast<int>(raw_boxes_.size());
    profile.adjacency_islands = static_cast<int>(find_islands(adjacency_).size());
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
        profile.adjacency_islands = static_cast<int>(find_islands(adjacency_).size());
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

    const auto adj_t0_delete = Clock::now();
    if (profile.boxes_added > 0) {
        connect_incremental_boxes(adjacency_, boxes_, first_new_box_index, config_.query.adjacency_tolerance);
        apply_segment_edges_to_adjacency(segment_edges_, adjacency_);
        invalidate_query_cache();
    }
    profile.adjacency_ms = std::chrono::duration<double, std::milli>(Clock::now() - adj_t0_delete).count();
    profile.boxes_after = static_cast<int>(boxes_.size());
    profile.raw_boxes_after = static_cast<int>(raw_boxes_.size());
    profile.adjacency_islands = static_cast<int>(find_islands(adjacency_).size());
    profile.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
    invalidate_query_cache();
    return profile;

    const auto dirty_t0 = Clock::now();
    std::vector<int> dirty_indices;
    if (config_.dynamic_update.enable_spatial_dirty_region) {
        profile.used_spatial_dirty_region = true;
        dirty_indices = spatial_dirty_box_indices(robot_, boxes_, removed_obstacles, config_.dynamic_update, profile.dirty_boxes);
        profile.dirty_boxes_used = static_cast<int>(dirty_indices.size());
    }
    profile.dirty_region_ms = std::chrono::duration<double, std::milli>(Clock::now() - dirty_t0).count();

    auto fallback_seeds = [&]() {
        std::vector<Eigen::VectorXd> seeds = last_build_seeds_;
        if (seeds.empty()) {
            const int limit = std::max(1, config_.dynamic_update.dirty_seed_limit);
            seeds.reserve(static_cast<std::size_t>(std::min(limit, static_cast<int>(boxes_.size()))));
            for (const auto& box : boxes_) {
                seeds.push_back(box.center());
                if (static_cast<int>(seeds.size()) >= limit) {
                    break;
                }
            }
        }
        return seeds;
    };

    auto run_warm_rebuild = [&](std::string reason) {
        const auto rebuild_t0 = Clock::now();
        auto seeds = fallback_seeds();
        if (seeds.empty()) {
            profile.fallback_reason = "warm_rebuild_skipped_no_seeds";
            return false;
        }
        profile.used_warm_rebuild = true;
        profile.fallback_reason = std::move(reason);
        StageContext context = StageContext::from_runtime(config_.runtime);
        const auto before_build = profile.boxes_before;
        const auto build_profile = build_coverage(scene_.obstacles(), seeds, context);
        (void)build_profile;
        profile.warm_rebuild_ms = std::chrono::duration<double, std::milli>(Clock::now() - rebuild_t0).count();
        profile.boxes_after = static_cast<int>(boxes_.size());
        profile.raw_boxes_after = static_cast<int>(raw_boxes_.size());
        profile.boxes_added = std::max(0, profile.boxes_after - before_build);
        profile.raw_boxes_added = std::max(0, profile.raw_boxes_after - profile.raw_boxes_before);
        profile.boxes_removed = std::max(0, before_build - profile.boxes_after);
        profile.raw_boxes_removed = std::max(0, profile.raw_boxes_before - profile.raw_boxes_after);
        profile.collision_cache_boxes_after = static_cast<int>(dynamic_collision_box_cache_.size());
        profile.adjacency_islands = last_build_.adjacency_islands;
        return true;
    };

    const bool too_many_dirty = config_.dynamic_update.enable_warm_rebuild_fallback &&
        ((config_.dynamic_update.warm_rebuild_dirty_box_threshold > 0 &&
          profile.dirty_boxes > config_.dynamic_update.warm_rebuild_dirty_box_threshold) ||
         (profile.boxes_before > 0 && config_.dynamic_update.warm_rebuild_dirty_box_fraction >= 0.0 &&
          static_cast<double>(profile.dirty_boxes) / static_cast<double>(profile.boxes_before) >
              config_.dynamic_update.warm_rebuild_dirty_box_fraction));
    const bool empty_forest = config_.dynamic_update.enable_warm_rebuild_fallback &&
        config_.dynamic_update.warm_rebuild_on_empty_forest && profile.boxes_before == 0;
    const bool empty_dirty = config_.dynamic_update.enable_warm_rebuild_fallback &&
        config_.dynamic_update.warm_rebuild_on_empty_dirty_region && profile.boxes_before > 0 && profile.dirty_boxes == 0;
    if (too_many_dirty || empty_forest || empty_dirty) {
        const char* reason = too_many_dirty ? "dirty_region_too_large_batch"
            : empty_forest ? "empty_forest_after_obstacle_removal_batch"
            : "empty_dirty_region_batch";
        if (run_warm_rebuild(reason)) {
            profile.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
            invalidate_query_cache();
            return profile;
        }
    }

    const auto regrow_t0 = Clock::now();
    const auto seed_candidates = make_dirty_region_seeds(boxes_,
                                                         dirty_indices,
                                                         oracle_->root_intervals(),
                                                         config_.dynamic_update,
                                                         config_.query.adjacency_tolerance,
                                                         config_.grower.boundary_epsilon);
    profile.dirty_seed_count = static_cast<int>(seed_candidates.size());
    if (!seed_candidates.empty() && config_.dynamic_update.local_regrow_box_limit > 0) {
        const Deadline deadline = config_.dynamic_update.local_regrow_timeout_ms > 0.0
            ? Deadline::after_ms(config_.dynamic_update.local_regrow_timeout_ms)
            : Deadline{};
        StageContext context = StageContext::from_runtime(config_.runtime, deadline);
        FindFreeBoxService ffb(*oracle_);
        FindFreeBoxOptions options = config_.grower.find_free_box;
        int next_id = next_box_id();
        for (const auto& candidate : seed_candidates) {
            if (context.should_stop() || profile.boxes_added >= config_.dynamic_update.local_regrow_box_limit) {
                break;
            }
            if (point_covered_by_existing_box_local(boxes_, candidate.seed)) {
                continue;
            }
            profile.regrow_attempts += 1;
            auto result = ffb.find(candidate.seed, context, options);
            if (!result.found || !intervals_contain_point_local(result.intervals, candidate.seed, config_.query.adjacency_tolerance)) {
                continue;
            }
            if (!allow_dynamic_commit(*oracle_, result, config_.grower.commit_policy)) {
                continue;
            }
            BoxNode box;
            box.id = next_id++;
            box.joint_intervals = std::move(result.intervals);
            box.seed_config = candidate.seed;
            box.tree_id = result.node;
            box.parent_box_id = candidate.parent_box_id;
            box.root_id = candidate.root_id >= 0 ? candidate.root_id : box.id;
            box.safety_status = result.validation_detail.safety_status;
            box.strict_audit_required = result.validation_detail.strict_audit_required;
            box.compute_volume();
            bool contained = false;
            for (const auto& existing : boxes_) {
                if (box_contains_box_exact_local(existing, box)) {
                    contained = true;
                    break;
                }
            }
            if (contained) {
                continue;
            }
            if (candidate.parent_box_id >= 0) {
                const BoxNode* parent = find_box_by_id(boxes_, candidate.parent_box_id);
                if (parent != nullptr && !boxes_connected(*parent, box, config_.query.adjacency_tolerance)) {
                    const double gap = std::sqrt(interval_bounds_gap_squared_local(parent->joint_intervals, box.joint_intervals));
                    if (gap > config_.grower.boundary_epsilon + config_.query.adjacency_tolerance) {
                        continue;
                    }
                }
            }
            oracle_->reserve_node(box.tree_id, box.id);
            boxes_.push_back(box);
            raw_boxes_.push_back(std::move(box));
            profile.boxes_added += 1;
            profile.raw_boxes_added += 1;
        }
    }
    profile.regrow_ms = std::chrono::duration<double, std::milli>(Clock::now() - regrow_t0).count();

    const bool local_below_min = config_.dynamic_update.enable_warm_rebuild_fallback &&
        config_.dynamic_update.warm_rebuild_min_local_boxes_added >= 0 &&
        profile.regrow_attempts > 0 &&
        profile.boxes_added < config_.dynamic_update.warm_rebuild_min_local_boxes_added;
    if (local_below_min && run_warm_rebuild("local_regrow_below_minimum_batch")) {
        profile.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
        invalidate_query_cache();
        return profile;
    }

    const auto adj_t0 = Clock::now();
    if (profile.boxes_added > 0) {
        rebuild_adjacency();
    }
    profile.adjacency_ms = std::chrono::duration<double, std::milli>(Clock::now() - adj_t0).count();
    profile.boxes_after = static_cast<int>(boxes_.size());
    profile.raw_boxes_after = static_cast<int>(raw_boxes_.size());
    profile.adjacency_islands = static_cast<int>(find_islands(adjacency_).size());
    profile.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
    invalidate_query_cache();
    return profile;
}

void RBFPlanningForest::clear_forest() {
    boxes_.clear();
    raw_boxes_.clear();
    adjacency_.clear();
    segment_edges_.clear();
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

void RBFPlanningForest::invalidate_query_cache() const {
    query_cache_dirty_ = true;
}

const QueryGraphCache& RBFPlanningForest::query_cache() const {
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
