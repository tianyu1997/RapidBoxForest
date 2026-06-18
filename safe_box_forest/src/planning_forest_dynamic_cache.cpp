#include <SBF/safe_box_forest.h>

#include <SBF/box_graph.h>
#include <SBF/connector.h>

#include <algorithm>
#include <chrono>
#include <limits>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "adaptive_grid_partition_options.h"
#include "planning_forest_audit.h"
#include "planning_forest_qroot_helpers.h"
#include "planning_forest_query_utils.h"

namespace rbf {

namespace {

bool point_covered_by_existing_box_local(const std::vector<BoxNode>& boxes,
                                         const Eigen::Ref<const Eigen::VectorXd>& point) {
    return std::any_of(boxes.begin(), boxes.end(), [&](const BoxNode& box) {
        return intervals_contain_point_strict_local(box.joint_intervals, point);
    });
}

struct SubtractiveSeedCandidate {
    Eigen::VectorXd seed;
    int parent_box_id = -1;
    int root_id = -1;
    int domain_index = -1;
};

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

void merge_diagnostic_snapshot_local(std::unordered_map<std::string, double>& diagnostics,
                                     const std::unordered_map<std::string, double>& snapshot) {
    for (const auto& [key, value] : snapshot) {
        diagnostics[key] += value;
    }
}

void initialize_segment_fallback_profile(RebuildProfile& profile,
                                         int boxes_size,
                                         int raw_boxes_size,
                                         int obstacle_count,
                                         int collision_cache_size,
                                         int segment_edge_count) {
    profile.boxes_before = boxes_size;
    profile.raw_boxes_before = raw_boxes_size;
    profile.obstacles_before = obstacle_count;
    profile.obstacles_after = obstacle_count;
    profile.collision_cache_boxes_before = collision_cache_size;
    profile.collision_cache_boxes_after = collision_cache_size;
    profile.diagnostics["segment_fallback.segment_edges_before"] =
        static_cast<double>(segment_edge_count);
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

Obstacle inflate_obstacle(const Obstacle& obstacle, double padding) {
    const float pad = static_cast<float>(std::max(0.0, padding));
    return Obstacle(obstacle.bounds[0] - pad,
                    obstacle.bounds[1] - pad,
                    obstacle.bounds[2] - pad,
                    obstacle.bounds[3] + pad,
                    obstacle.bounds[4] + pad,
                    obstacle.bounds[5] + pad);
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
    CollisionChecker dirty_checker(
        robot,
        Scene(std::vector<Obstacle>{inflate_obstacle(obstacle, config.dirty_region_padding)}));
    dirty_indices.reserve(boxes.size());
    for (int index = 0; index < static_cast<int>(boxes.size()); ++index) {
        if (dirty_checker.check_box(boxes[static_cast<std::size_t>(index)].joint_intervals)) {
            dirty_count += 1;
            dirty_indices.push_back(index);
        }
    }
    return dirty_indices;
}

bool has_adjacency_to_existing_box(const std::vector<BoxNode>& boxes,
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

bool intervals_overlap_local(const std::vector<Interval>& lhs,
                             const std::vector<Interval>& rhs,
                             double tolerance) {
    if (lhs.size() != rhs.size()) {
        return false;
    }
    for (std::size_t dim = 0; dim < lhs.size(); ++dim) {
        if (lhs[dim].hi + tolerance < rhs[dim].lo ||
            rhs[dim].hi + tolerance < lhs[dim].lo) {
            return false;
        }
    }
    return true;
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

} // namespace

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
        clear_dynamic_collision_cache();
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

void RBFPlanningForest::populate_dynamic_collision_cache(const LeafSweepResult& result,
                                                         int obstacle_count) {
    clear_dynamic_collision_cache();
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

void RBFPlanningForest::clear_dynamic_collision_cache() {
    dynamic_collision_box_cache_.clear();
    dynamic_collision_cache_blocker_index_.clear();
    dynamic_collision_cache_active_count_ = 0;
}

void RBFPlanningForest::rebuild_dynamic_collision_cache_index() {
    dynamic_collision_cache_blocker_index_.clear();
    dynamic_collision_cache_active_count_ = 0;
    for (std::size_t index = 0; index < dynamic_collision_box_cache_.size(); ++index) {
        const auto& cached = dynamic_collision_box_cache_[index];
        if (!cached.active || cached.blocking_obstacle_indices.empty()) {
            continue;
        }
        dynamic_collision_cache_active_count_ += 1;
        for (int obstacle_index : cached.blocking_obstacle_indices) {
            dynamic_collision_cache_blocker_index_[obstacle_index].push_back(index);
        }
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
    cached.active = true;
    const std::size_t cache_index = dynamic_collision_box_cache_.size();
    dynamic_collision_box_cache_.push_back(std::move(cached));
    dynamic_collision_cache_active_count_ += 1;
    for (int obstacle_index : dynamic_collision_box_cache_.back().blocking_obstacle_indices) {
        dynamic_collision_cache_blocker_index_[obstacle_index].push_back(cache_index);
    }
}

int RBFPlanningForest::promote_unblocked_collision_cache(const std::unordered_set<int>& removed_obstacle_indices,
                                                         RebuildProfile& profile) {
    if (dynamic_collision_cache_active_count_ <= 0 && !dynamic_collision_box_cache_.empty()) {
        rebuild_dynamic_collision_cache_index();
    }
    profile.collision_cache_boxes_before = dynamic_collision_cache_active_count_;
    if (removed_obstacle_indices.empty() || dynamic_collision_cache_active_count_ <= 0) {
        profile.collision_cache_boxes_after = dynamic_collision_cache_active_count_;
        return 0;
    }

    std::vector<int> sorted_removed(removed_obstacle_indices.begin(), removed_obstacle_indices.end());
    std::sort(sorted_removed.begin(), sorted_removed.end());
    sorted_removed.erase(std::unique(sorted_removed.begin(), sorted_removed.end()), sorted_removed.end());
    const int min_removed = sorted_removed.empty() ? std::numeric_limits<int>::max() : sorted_removed.front();
    const int max_removed = sorted_removed.empty() ? std::numeric_limits<int>::min() : sorted_removed.back();
    auto is_removed_index = [&](int old_index) {
        return std::binary_search(sorted_removed.begin(), sorted_removed.end(), old_index);
    };
    auto removed_before_count = [&](int old_index) {
        return static_cast<int>(
            std::lower_bound(sorted_removed.begin(), sorted_removed.end(), old_index) - sorted_removed.begin());
    };
    auto remap_obstacle_index = [&](int old_index, int& new_index) {
        if (is_removed_index(old_index)) {
            return false;
        }
        const int shift = removed_before_count(old_index);
        new_index = old_index - shift;
        return new_index >= 0 && new_index < scene_.n_obstacles();
    };

    int promoted = 0;
    int next_id = next_box_id();
    const double adjacency_tolerance = config_.query.adjacency_tolerance;
    const auto cache_scan_t0 = std::chrono::steady_clock::now();

    auto deactivate_cached = [&](CachedCollisionBox& cached) {
        if (cached.active) {
            cached.active = false;
            cached.blocking_obstacle_indices.clear();
            dynamic_collision_cache_active_count_ =
                std::max(0, dynamic_collision_cache_active_count_ - 1);
        }
    };

    auto try_promote_touched_cached = [&](CachedCollisionBox& cached) {
        bool touched = false;
        std::vector<int> remaining_blockers;
        remaining_blockers.reserve(cached.blocking_obstacle_indices.size());
        for (int old_index : cached.blocking_obstacle_indices) {
            profile.diagnostics["delete.blocker_index_checks"] += 1.0;
            if (is_removed_index(old_index)) {
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
            return;
        }
        profile.collision_cache_candidates += 1;
        if (!remaining_blockers.empty()) {
            cached.blocking_obstacle_indices = std::move(remaining_blockers);
            return;
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
            deactivate_cached(cached);
            return;
        }
        int adjacent_parent = -1;
        const auto adjacency_t0 = std::chrono::steady_clock::now();
        if (!boxes_.empty() &&
            !has_adjacency_to_existing_box(boxes_, box, adjacency_tolerance, &adjacent_parent)) {
            profile.diagnostics["delete.adjacency_check_ms"] +=
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - adjacency_t0).count();
            profile.collision_cache_rejected_disconnected += 1;
            deactivate_cached(cached);
            return;
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
        deactivate_cached(cached);
    };

    const bool suffix_remove = min_removed >= scene_.n_obstacles();
    if (suffix_remove && !dynamic_collision_cache_blocker_index_.empty()) {
        std::vector<std::size_t> candidate_indices;
        std::unordered_set<std::size_t> seen;
        for (int removed_index : sorted_removed) {
            const auto it = dynamic_collision_cache_blocker_index_.find(removed_index);
            if (it == dynamic_collision_cache_blocker_index_.end()) {
                continue;
            }
            for (std::size_t index : it->second) {
                if (index < dynamic_collision_box_cache_.size() && seen.insert(index).second) {
                    candidate_indices.push_back(index);
                }
            }
        }
        profile.diagnostics["delete.cache_indexed_suffix_path"] = 1.0;
        profile.diagnostics["delete.cache_index_removed_keys"] = static_cast<double>(sorted_removed.size());
        profile.diagnostics["delete.cache_index_candidate_entries"] =
            static_cast<double>(candidate_indices.size());
        for (std::size_t index : candidate_indices) {
            if (index >= dynamic_collision_box_cache_.size()) {
                profile.diagnostics["delete.cache_index_stale_out_of_range"] += 1.0;
                continue;
            }
            CachedCollisionBox& cached = dynamic_collision_box_cache_[index];
            if (!cached.active) {
                profile.diagnostics["delete.cache_index_stale_inactive"] += 1.0;
                continue;
            }
            profile.diagnostics["delete.cache_entries_scanned"] += 1.0;
            try_promote_touched_cached(cached);
        }
        profile.diagnostics["delete.cache_scan_ms"] +=
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - cache_scan_t0).count();
        profile.collision_cache_boxes_after = dynamic_collision_cache_active_count_;
        return promoted;
    }

    profile.diagnostics["delete.cache_indexed_suffix_path"] = 0.0;
    std::size_t write_index = 0;
    auto keep_cached_at = [&](std::size_t read_index) {
        if (write_index != read_index) {
            dynamic_collision_box_cache_[write_index] =
                std::move(dynamic_collision_box_cache_[read_index]);
        }
        ++write_index;
    };
    for (std::size_t read_index = 0; read_index < dynamic_collision_box_cache_.size(); ++read_index) {
        auto& cached = dynamic_collision_box_cache_[read_index];
        if (!cached.active) {
            continue;
        }
        profile.diagnostics["delete.cache_entries_scanned"] += 1.0;
        if (!cached.blocking_obstacle_indices.empty()) {
            const int first_blocker = cached.blocking_obstacle_indices.front();
            const int last_blocker = cached.blocking_obstacle_indices.back();
            if (last_blocker < min_removed) {
                // Common suffix-delete case: this cached box is blocked only by
                // obstacles that remain before the deleted suffix, so neither
                // blocker membership nor obstacle numbering changes.
                profile.diagnostics["delete.cache_fast_untouched_before_removed"] += 1.0;
                keep_cached_at(read_index);
                continue;
            }
            if (first_blocker > max_removed) {
                // No blocker is removed, but all blocker ids shift down by the
                // number of removed obstacles before them.
                profile.diagnostics["delete.cache_fast_remap_after_removed"] += 1.0;
                std::vector<int> remapped;
                remapped.reserve(cached.blocking_obstacle_indices.size());
                for (int old_index : cached.blocking_obstacle_indices) {
                    int new_index = -1;
                    if (remap_obstacle_index(old_index, new_index)) {
                        remapped.push_back(new_index);
                    }
                }
                std::sort(remapped.begin(), remapped.end());
                remapped.erase(std::unique(remapped.begin(), remapped.end()), remapped.end());
                cached.blocking_obstacle_indices = std::move(remapped);
                if (!cached.blocking_obstacle_indices.empty()) {
                    keep_cached_at(read_index);
                }
                continue;
            }
        }
        bool touched = false;
        std::vector<int> remaining_blockers;
        remaining_blockers.reserve(cached.blocking_obstacle_indices.size());
        for (int old_index : cached.blocking_obstacle_indices) {
            profile.diagnostics["delete.blocker_index_checks"] += 1.0;
            if (is_removed_index(old_index)) {
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
            keep_cached_at(read_index);
            continue;
        }
        profile.collision_cache_candidates += 1;
        if (!remaining_blockers.empty()) {
            cached.blocking_obstacle_indices = std::move(remaining_blockers);
            keep_cached_at(read_index);
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
        if (!boxes_.empty() &&
            !has_adjacency_to_existing_box(boxes_, box, adjacency_tolerance, &adjacent_parent)) {
            profile.diagnostics["delete.adjacency_check_ms"] +=
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - adjacency_t0).count();
            profile.collision_cache_rejected_disconnected += 1;
            cached.blocking_obstacle_indices.clear();
            keep_cached_at(read_index);
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
    dynamic_collision_box_cache_.resize(write_index);
    rebuild_dynamic_collision_cache_index();
    profile.collision_cache_boxes_after = dynamic_collision_cache_active_count_;
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
        std::vector<Interval> intervals;
    };
    std::vector<Item> stack;
    stack.push_back(Item{removed_box.tree_id, -1, removed_box.joint_intervals});
    int added = 0;
    const OracleSplitOptions split_options = config_.grower.find_free_box.split;
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
        if (config_.dynamic_update.local_regrow_box_limit > 0 &&
            profile.boxes_added >= config_.dynamic_update.local_regrow_box_limit) {
            profile.diagnostics["insert.refill_local_regrow_box_cap_hits"] += 1.0;
            break;
        }
        if (++stack_pops > max_stack_pops) {
            profile.diagnostics["insert.refill_stack_pop_cap_hits"] += 1.0;
            break;
        }
        const Item item = stack.back();
        stack.pop_back();
        if (item.node < 0) {
            continue;
        }
        const std::vector<Interval> tree_intervals = oracle_->node_intervals(item.node);
        std::vector<Interval> intervals = item.intervals;
        if (intervals.empty()) {
            bool found_matching_native_copy = false;
            for (auto candidate : oracle_->native_interval_copies_for_node(item.node, tree_intervals)) {
                if (intervals_subset_local(candidate, removed_box.joint_intervals, 1e-12)) {
                    intervals = std::move(candidate);
                    found_matching_native_copy = true;
                    break;
                }
            }
            if (!found_matching_native_copy) {
                profile.diagnostics["insert.refill_native_copy_misses"] += 1.0;
                continue;
            }
        } else if (!intervals_subset_local(intervals, removed_box.joint_intervals, 1e-10)) {
            profile.diagnostics["insert.refill_interval_subset_rejects"] += 1.0;
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
        if (split.split_dim < 0 || split.split_dim >= static_cast<int>(intervals.size())) {
            profile.diagnostics["insert.refill_split_bad_dim"] += 1.0;
            cache_collision_leaf(item.node, intervals);
            continue;
        }
        const int dim = split.split_dim;
        const double lo = intervals[static_cast<std::size_t>(dim)].lo;
        const double hi = intervals[static_cast<std::size_t>(dim)].hi;
        double value = split.split_value;
        if (!(value > lo + 1e-14 && value < hi - 1e-14)) {
            value = 0.5 * (lo + hi);
            profile.diagnostics["insert.refill_split_native_midpoint_fallbacks"] += 1.0;
        }
        if (!(value > lo && value < hi)) {
            profile.diagnostics["insert.refill_split_degenerate_intervals"] += 1.0;
            cache_collision_leaf(item.node, intervals);
            continue;
        }
        std::vector<Interval> left_intervals = intervals;
        std::vector<Interval> right_intervals = intervals;
        left_intervals[static_cast<std::size_t>(dim)].hi = value;
        right_intervals[static_cast<std::size_t>(dim)].lo = value;
        stack.push_back(Item{split.right, split.split_dim, std::move(right_intervals)});
        stack.push_back(Item{split.left, split.split_dim, std::move(left_intervals)});
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

RebuildProfile RBFPlanningForest::connect_update_endpoint_segment_fallback(
    const Eigen::Ref<const Eigen::VectorXd>& start,
    const Eigen::Ref<const Eigen::VectorXd>& goal) {
    using Clock = std::chrono::steady_clock;
    const auto t0 = Clock::now();
    RebuildProfile profile;
    initialize_segment_fallback_profile(profile,
                                        static_cast<int>(boxes_.size()),
                                        static_cast<int>(raw_boxes_.size()),
                                        scene_.n_obstacles(),
                                        static_cast<int>(dynamic_collision_box_cache_.size()),
                                        static_cast<int>(segment_edges_.size()));
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
	        merge_diagnostic_snapshot_local(profile.diagnostics, context.diagnostics().snapshot());
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
        if (!has_adjacency_to_existing_box(boxes_, box, config_.query.adjacency_tolerance, &adjacent_parent)) {
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

RebuildProfile RBFPlanningForest::connect_update_segment_fallback() {
    using Clock = std::chrono::steady_clock;
    const auto t0 = Clock::now();
    RebuildProfile profile;
    initialize_segment_fallback_profile(profile,
                                        static_cast<int>(boxes_.size()),
                                        static_cast<int>(raw_boxes_.size()),
                                        scene_.n_obstacles(),
                                        static_cast<int>(dynamic_collision_box_cache_.size()),
                                        static_cast<int>(segment_edges_.size()));
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
        const int pair_candidate_cap = partition_segment_fallback_pair_candidate_cap_from_env();
        const auto candidate_pairs =
            adaptive_partition_->nearest_component_pairs_to_largest(1, pair_candidate_cap);
        for (const auto& pair : candidate_pairs) {
            if (pair.source_box_id < 0 || pair.target_box_id < 0 ||
                pair.source_point.size() == 0 || pair.target_point.size() == 0) {
                continue;
            }
            ++attempted_pairs;
            std::vector<Eigen::VectorXd> waypoints{pair.source_point, pair.target_point};
            if (!audit_waypoint_path_passes(waypoints,
                                            checker,
                                            config_.query.audit_resolution,
                                            config_.query.audit_segment_step)) {
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

} // namespace rbf
