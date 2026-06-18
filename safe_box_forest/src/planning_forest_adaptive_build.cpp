#include <SBF/safe_box_forest.h>

#include <sbf/core/joint_symmetry.h>
#include <sbf/envelope/envelope_collision.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cstdlib>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iterator>
#include <limits>
#include <queue>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include "planning_forest_audit.h"
#include "planning_forest_adaptive_cover_utils.h"
#include "planning_forest_adaptive_merge.h"
#include "planning_forest_diagnostics.h"
#include "planning_forest_obb.h"
#include "planning_forest_qroot_helpers.h"
#include "planning_forest_query_utils.h"
#include "virtual_sparse_ffb.h"

namespace rbf {

namespace {

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


}  // namespace

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
        if (snapshot.cell_count <= 0 || snapshot.main_island_cell_count <= 0) {
            return false;
        }
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
            clear_dynamic_collision_cache();
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
                    partition_largest_island_for_profile = partition_stats.largest_island;
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
                candidate.partition_largest_island = partition_stats.largest_island;
                candidate.profile.grow_adjacency_islands = partition_stats.islands;
                candidate.profile.adjacency_islands = partition_stats.islands;
                candidate.profile.grow_largest_island = partition_stats.largest_island;
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
            snapshot.cell_count > 0 &&
            snapshot.main_island_cell_count > 0 &&
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
        clear_dynamic_collision_cache();
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


}  // namespace rbf
