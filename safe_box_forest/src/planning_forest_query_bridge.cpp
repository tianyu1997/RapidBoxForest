#include <SBF/safe_box_forest.h>

#include <SBF/box_graph.h>
#include <SBF/connector.h>

#include "env_config.h"
#include "planning_forest_audit.h"
#include "planning_forest_diagnostics.h"
#include "planning_forest_qroot_helpers.h"
#include "planning_forest_query_utils.h"
#include "virtual_sparse_ffb.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <memory>
#include <optional>
#include <queue>
#include <random>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace rbf {

using detail::env_double_list_or_empty;
using detail::env_double_or_default;
using detail::env_index_list_contains;
using detail::env_indexed_double_or_default;
using detail::env_indexed_int_or_default;
using detail::env_int_list_or_empty;
using detail::env_int_or_default;

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

	    std::vector<int> depth_schedule{
	        std::min(std::max(1, config_.database.max_tree_depth),
	                 std::max(1, final_ffb_depth))
	    };

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
    std::vector<int> anchor_depth_schedule =
        env_int_list_or_empty("RBF_QUERY_ENDPOINT_ANCHOR_FFB_DEPTHS");
    const int max_tree_depth = std::max(1, config_.database.max_tree_depth);
    auto normalize_depth = [&](int depth) {
        return std::min(max_tree_depth, std::max(1, depth));
    };
    std::vector<int> normalized_depths;
    normalized_depths.reserve(anchor_depth_schedule.size() + 1U);
    for (int depth : anchor_depth_schedule) {
        if (depth <= 0) {
            continue;
        }
        const int normalized = normalize_depth(depth);
        if (std::find(normalized_depths.begin(), normalized_depths.end(), normalized) ==
            normalized_depths.end()) {
            normalized_depths.push_back(normalized);
        }
    }
    if (normalized_depths.empty()) {
        normalized_depths.push_back(normalize_depth(options.max_depth));
    } else {
        const int final_depth = normalize_depth(options.max_depth);
        if (std::find(normalized_depths.begin(), normalized_depths.end(), final_depth) ==
            normalized_depths.end()) {
            normalized_depths.push_back(final_depth);
        }
    }
    context.diagnostics().set_value("query_bridge.endpoint_anchor_ffb_depth",
                                    static_cast<double>(normalized_depths.back()));
    context.diagnostics().set_value("query_bridge.endpoint_anchor_ffb_depth_schedule_size",
                                    static_cast<double>(normalized_depths.size()));

    BoxNode root_domain;
    root_domain.id = -1;
    root_domain.joint_intervals = oracle_->planning_intervals();
    root_domain.compute_volume();

    if (partition_native_mode()) {
        const bool endpoint_point_anchor =
            env_int_or_default("RBF_QUERY_ENDPOINT_POINT_ANCHOR", 0) != 0;
        context.diagnostics().set_value("query_bridge.endpoint_point_anchor_enabled",
                                        endpoint_point_anchor ? 1.0 : 0.0);
        if (endpoint_point_anchor) {
            bool in_domain = static_cast<int>(root_domain.joint_intervals.size()) == point.size();
            if (in_domain) {
                for (int dim = 0; dim < point.size(); ++dim) {
                    if (!root_domain.joint_intervals[static_cast<std::size_t>(dim)].contains(point[dim], 0.0)) {
                        in_domain = false;
                        break;
                    }
                }
            }
            if (!in_domain) {
                context.diagnostics().add_counter("query_bridge.endpoint_point_anchor_domain_rejects");
            } else {
                context.diagnostics().add_counter("query_bridge.endpoint_point_anchor_attempts");
                CollisionChecker checker = make_audit_checker(audit_robot_, scene_, config_.query);
                if (!checker.check_config(point)) {
                    const std::size_t boxes_before_anchor = boxes_.size();
                    BoxNode box;
                    box.id = next_box_id();
                    box.joint_intervals.reserve(static_cast<std::size_t>(point.size()));
                    for (int dim = 0; dim < point.size(); ++dim) {
                        box.joint_intervals.push_back(Interval{point[dim], point[dim]});
                    }
                    box.seed_config = point;
                    box.tree_id = kInvalidOracleNodeId;
                    box.parent_box_id = -1;
                    box.root_id = box.id;
                    box.safety_status = BoxSafetyStatus::CertifiedFree;
                    box.strict_audit_required = false;
                    box.compute_volume();
                    const int new_id = box.id;
                    boxes_.push_back(box);
                    raw_boxes_.push_back(box);
                    context.diagnostics().add_counter("query_bridge.endpoint_point_anchor_success");
                    context.diagnostics().add_counter("query_bridge.endpoint_anchor_boxes_added");
                    append_adaptive_partition_boxes(boxes_before_anchor,
                                                    &last_build_,
                                                    "query_bridge.endpoint_point_anchor");
                    invalidate_query_cache();
                    return new_id;
                }
                context.diagnostics().add_counter("query_bridge.endpoint_point_anchor_collision_rejects");
            }
        }
        const std::size_t boxes_before_anchor = boxes_.size();
        StageContext local_context = context;
        FindFreeBoxOptions anchor_options = options;
        anchor_options.materialize_result_node = false;
        auto result = find_free_box_in_domain(point,
                                              root_domain.joint_intervals,
                                              local_context,
                                              anchor_options);
        merge_diagnostic_snapshot(context.diagnostics(), local_context.diagnostics().snapshot());
        context.diagnostics().add_counter("query_bridge.endpoint_anchor_calls");
        if (!result.found ||
            !intervals_contain_point_local(result.intervals,
                                           point,
                                           config_.query.adjacency_tolerance)) {
            context.diagnostics().add_counter("query_bridge.endpoint_anchor_ffb_fail");
            context.diagnostics().add_counter(
                "query_bridge.endpoint_anchor_ffb_fail_code." +
                std::to_string(result.fail_code));
            context.diagnostics().set_value("query_bridge.endpoint_anchor_last_fail_code",
                                            static_cast<double>(result.fail_code));
            if (result.seed_collision) {
                context.diagnostics().add_counter("query_bridge.endpoint_anchor_seed_collision");
            }
            if (result.hit_unknown_depth_cap) {
                context.diagnostics().add_counter("query_bridge.endpoint_anchor_unknown_depth_cap");
            }
            if (result.hit_reserved_depth_cap) {
                context.diagnostics().add_counter("query_bridge.endpoint_anchor_reserved_depth_cap");
            }
            if (result.deadline_reached) {
                context.diagnostics().add_counter("query_bridge.endpoint_anchor_deadline");
            }
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
            if ((result.node != kInvalidOracleNodeId &&
                 existing_box.tree_id == result.node) ||
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
        if (box.tree_id != kInvalidOracleNodeId) {
            oracle_->reserve_node(box.tree_id, box.id);
        }
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
    if (stats.ffb_fail > 0) {
        context.diagnostics().add_counter("query_bridge.endpoint_anchor_ffb_fail_code.unknown_legacy",
                                          stats.ffb_fail);
    }
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
    const bool direct_start_goal_segment =
        env_int_or_default("RBF_QUERY_BRIDGE_DIRECT_SEGMENT_AFTER_RRT", 0) != 0 &&
        env_int_or_default("RBF_QUERY_BRIDGE_DIRECT_START_GOAL_SEGMENT", 1) != 0 &&
        config_.connector.segment_edges_enabled &&
        config_.connector.rrt_segment_edges;
    context.diagnostics().set_value("query_bridge.direct_start_goal_segment",
                                    direct_start_goal_segment ? 1.0 : 0.0);
    if (direct_start_goal_segment) {
        std::vector<Eigen::VectorXd> direct_path{start, goal};
        context.diagnostics().add_counter("query_bridge.direct_start_goal_segment_attempts");
        const PathAuditCheck audit =
            audit_waypoint_path(direct_path,
                                checker,
                                config_.query.audit_resolution,
                                config_.query.audit_segment_step);
        if (audit.passed) {
            const int edge_id = add_segment_edge_partition_first(
                start_box_id,
                goal_box_id,
                direct_path,
                SegmentEdgeType::QueryBridge,
                config_.query.audit_resolution,
                SegmentEdgeValidation::CollisionChecked,
                true,
                -1);
            if (edge_id >= 0) {
                context.diagnostics().add_counter("query_bridge.direct_start_goal_segment_edges");
                invalidate_query_cache();
                sync_adaptive_partition_segment_edges(&last_build_,
                                                       "query_bridge.direct_start_goal_segment");
                refresh_adaptive_partition_diagnostics(&last_build_);
                return 1;
            }
            context.diagnostics().add_counter("query_bridge.direct_start_goal_segment_add_fail");
        } else {
            context.diagnostics().add_counter("query_bridge.direct_start_goal_segment_audit_rejects");
        }
    }
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
    const double bridge_rrt_clearance =
        std::max(0.0, env_double_or_default("RBF_QUERY_BRIDGE_RRT_CLEARANCE", 0.0));
    Robot bridge_rrt_robot = make_sbf_clearance_robot(audit_robot_, bridge_rrt_clearance);
    CollisionChecker bridge_rrt_checker =
        bridge_rrt_clearance > 0.0
            ? CollisionChecker(bridge_rrt_robot, scene_)
            : checker;
    context.diagnostics().set_value("query_bridge.rrt_clearance", bridge_rrt_clearance);
    auto waypoint_path = best_audited_rrt_bridge_path(start,
                                                      goal,
                                                      bridge_rrt_checker,
                                                      bridge_rrt_robot,
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
    const bool direct_segment_after_rrt =
        env_int_or_default("RBF_QUERY_BRIDGE_DIRECT_SEGMENT_AFTER_RRT", 0) != 0;
    const double direct_segment_after_rrt_min_length =
        std::max(0.0,
                 env_double_or_default("RBF_QUERY_BRIDGE_DIRECT_SEGMENT_AFTER_RRT_MIN_LENGTH",
                                       0.0));
    context.diagnostics().set_value("query_bridge.scene_reusable_edges",
                                    scene_reusable_query_bridge_edges ? 1.0 : 0.0);
    context.diagnostics().set_value("query_bridge.direct_segment_after_rrt",
                                    direct_segment_after_rrt ? 1.0 : 0.0);
    context.diagnostics().set_value("query_bridge.direct_segment_after_rrt_min_length",
                                    direct_segment_after_rrt_min_length);
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
    auto waypoint_length = [](const std::vector<Eigen::VectorXd>& path) {
        double total = 0.0;
        for (std::size_t i = 1; i < path.size(); ++i) {
            total += (path[i] - path[i - 1]).norm();
        }
        return total;
    };
    const double bridge_waypoint_length = waypoint_length(waypoint_path);
    const bool direct_segment_after_rrt_candidate =
        direct_segment_after_rrt &&
        bridge_waypoint_length >= direct_segment_after_rrt_min_length &&
        config_.connector.segment_edges_enabled &&
        config_.connector.rrt_segment_edges;
    context.diagnostics().set_value("query_bridge.direct_segment_after_rrt_candidate",
                                    direct_segment_after_rrt_candidate ? 1.0 : 0.0);
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
    const bool bridge_waypoint_shortcut =
        env_int_or_default("RBF_QUERY_BRIDGE_WAYPOINT_SHORTCUT",
                           direct_segment_after_rrt_candidate ? 1 : 0) != 0;
    const double bridge_waypoint_shortcut_min_gain =
        std::max(0.0,
                 env_double_or_default("RBF_QUERY_BRIDGE_WAYPOINT_SHORTCUT_MIN_GAIN",
                                       1e-6));
    context.diagnostics().set_value("query_bridge.waypoint_shortcut_enabled",
                                    bridge_waypoint_shortcut ? 1.0 : 0.0);
    if (bridge_waypoint_shortcut && corridor_path.size() > 2) {
        using Clock = std::chrono::steady_clock;
        const auto shortcut_t0 = Clock::now();
        const double before_length = waypoint_length(corridor_path);
        std::vector<Eigen::VectorXd> shortened =
            collision_shortcut_path(corridor_path,
                                    checker,
                                    collision_shortcut_resolution(config_.query));
        const double after_length = waypoint_length(shortened);
        context.diagnostics().add_counter("query_bridge.waypoint_shortcut_attempts");
        set_query_bridge_task_value("waypoint_shortcut_before_length", before_length);
        set_query_bridge_task_value("waypoint_shortcut_after_length", after_length);
        if (!shortened.empty() &&
            after_length + bridge_waypoint_shortcut_min_gain < before_length) {
            const PathAuditCheck shortcut_audit =
                audit_waypoint_path(shortened,
                                    checker,
                                    config_.query.audit_resolution,
                                    config_.query.audit_segment_step);
            if (shortcut_audit.passed) {
                context.diagnostics().add_counter("query_bridge.waypoint_shortcut_accepts");
                context.diagnostics().add_counter("query_bridge.waypoint_shortcut_delta",
                                                  before_length - after_length);
                set_query_bridge_task_value("waypoint_shortcut_accepted", 1.0);
                set_query_bridge_task_value("waypoint_shortcut_delta",
                                            before_length - after_length);
                corridor_path = std::move(shortened);
            } else {
                context.diagnostics().add_counter("query_bridge.waypoint_shortcut_audit_rejects");
                set_query_bridge_task_value("waypoint_shortcut_audit_reject", 1.0);
            }
        }
        context.diagnostics().record_timing(
            "query_bridge.waypoint_shortcut_ms_total",
            std::chrono::duration<double, std::milli>(Clock::now() -
                                                      shortcut_t0).count());
    }
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
        env_int_or_default("RBF_QUERY_BRIDGE_INTERNAL_SIMPLIFY",
                           direct_segment_after_rrt_candidate ? 1 : 0) != 0;
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
    context.diagnostics().set_value("query_bridge.direct_segment_after_rrt_final_length",
                                    audited_bridge_length);
    if (direct_segment_after_rrt_candidate) {
        context.diagnostics().add_counter(
            "query_bridge.direct_segment_after_rrt_final_attempts");
        context.diagnostics().add_counter(
            "query_bridge.direct_segment_after_rrt_shortening_delta",
            std::max(0.0, bridge_waypoint_length - audited_bridge_length));
        const PathAuditCheck segment_audit =
            audit_waypoint_path(corridor_path,
                                checker,
                                config_.query.audit_resolution,
                                config_.query.audit_segment_step);
        if (!segment_audit.passed) {
            context.diagnostics().add_counter(
                "query_bridge.direct_segment_after_rrt_audit_rejects");
        } else {
            const int edge_id = add_segment_edge_partition_first(
                start_box_id,
                goal_box_id,
                corridor_path,
                SegmentEdgeType::QueryBridge,
                bridge_rrt.segment_resolution,
                SegmentEdgeValidation::CollisionChecked,
                true,
                bridge_edge_query_index);
            if (edge_id >= 0) {
                direct_segment_edges_added += 1;
                context.diagnostics().add_counter(
                    "query_bridge.direct_segment_after_rrt_edges");
                invalidate_query_cache();
                sync_adaptive_partition_segment_edges(
                    &last_build_,
                    "query_bridge.direct_segment_after_rrt");
                refresh_adaptive_partition_diagnostics(&last_build_);
                return finish_bridge(direct_segment_edges_added);
            }
            context.diagnostics().add_counter(
                "query_bridge.direct_segment_after_rrt_add_fail");
        }
    }
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
        const int partition_append_batch_size = immediate_partition_append
            ? std::max(1,
                       env_int_or_default(
                           "RBF_QUERY_BRIDGE_DIRECT_PARTITION_APPEND_BATCH_SIZE",
                           32))
            : 0;
        const bool detailed_direct_timing =
            env_int_or_default("RBF_QUERY_BRIDGE_DETAILED_TIMING", 0) != 0;
        context.diagnostics().set_value(
            "query_bridge.direct_corridor_partition_neighbor_candidates_enabled",
            use_partition_neighbor_candidates ? 1.0 : 0.0);
        context.diagnostics().set_value(
            "query_bridge.direct_corridor_immediate_partition_append_enabled",
            immediate_partition_append ? 1.0 : 0.0);
        context.diagnostics().set_value(
            "query_bridge.direct_corridor_partition_append_batch_size",
            static_cast<double>(partition_append_batch_size));
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
        std::size_t direct_partition_append_base = boxes_.size();
        std::vector<std::vector<int>> sample_layers(samples.size());
        std::vector<bool> covered(samples.size(), false);
        struct ResidualMilestone {
            double param = 0.0;
            Eigen::VectorXd point;
            int box_index = -1;
        };
        std::vector<ResidualMilestone> repair_milestones;
        repair_milestones.reserve(samples.size());
        auto seed_path_param = [&](const Eigen::VectorXd& seed, int transition_hint) {
            if (samples.empty()) {
                return 0.0;
            }
            if (transition_hint >= 0 &&
                transition_hint + 1 < static_cast<int>(samples.size())) {
                const Eigen::VectorXd& a = samples[static_cast<std::size_t>(transition_hint)];
                const Eigen::VectorXd& b = samples[static_cast<std::size_t>(transition_hint + 1)];
                const Eigen::VectorXd delta = b - a;
                const double denom = delta.squaredNorm();
                double u = 0.5;
                if (denom > 1e-18) {
                    u = (seed - a).dot(delta) / denom;
                    u = std::min(1.0, std::max(0.0, u));
                }
                return static_cast<double>(transition_hint) + u;
            }
            double best_distance = std::numeric_limits<double>::infinity();
            std::size_t best_index = 0;
            for (std::size_t index = 0; index < samples.size(); ++index) {
                const double distance = (seed - samples[index]).squaredNorm();
                if (distance < best_distance) {
                    best_distance = distance;
                    best_index = index;
                }
            }
            return static_cast<double>(best_index);
        };
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
        int assimilate_coverage_boxes = 0;
        int assimilate_coverage_span_max = 0;
        double assimilate_coverage_span_sum = 0.0;
        int segment_insert_calls = 0;
        int direct_partition_append_calls = 0;
        int direct_partition_append_boxes = 0;
        const bool local_assimilate_sample_scan =
            env_int_or_default("RBF_QUERY_BRIDGE_LOCAL_SAMPLE_ASSIMILATION", 1) != 0;
        int assimilate_local_hits = 0;
        int assimilate_full_scan_fallbacks = 0;
        int assimilate_local_sample_tests = 0;
        context.diagnostics().set_value(
            "query_bridge.direct_corridor_local_sample_assimilation_enabled",
            local_assimilate_sample_scan ? 1.0 : 0.0);
        auto append_direct_partition_batch = [&](bool force) {
            if (!immediate_partition_append ||
                !adaptive_partition_ ||
                direct_partition_append_base >= boxes_.size()) {
                return 0;
            }
            const std::size_t pending = boxes_.size() - direct_partition_append_base;
            if (!force && pending < static_cast<std::size_t>(partition_append_batch_size)) {
                return 0;
            }
            const auto partition_append_t0 =
                detailed_direct_timing ? Clock::now() : Clock::time_point{};
            const int appended = adaptive_partition_->append_boxes(
                boxes_,
                direct_partition_append_base,
                config_.query.adjacency_tolerance);
            if (detailed_direct_timing) {
                commit_partition_append_ms +=
                    std::chrono::duration<double, std::milli>(Clock::now() -
                                                              partition_append_t0).count();
            }
            direct_partition_append_calls += 1;
            direct_partition_append_boxes += std::max(0, appended);
            context.diagnostics().add_counter(
                appended > 0
                    ? "query_bridge.direct_corridor_batched_partition_appends"
                    : "query_bridge.direct_corridor_batched_partition_append_rejects");
            direct_partition_append_base = boxes_.size();
            return appended;
        };
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
            int first_covered_sample = static_cast<int>(samples.size());
            int last_covered_sample = -1;
            int covered_sample_count = 0;
            const auto sample_scan_t0 = detailed_direct_timing ? Clock::now() : Clock::time_point{};
            const auto& box_intervals = boxes_[static_cast<std::size_t>(box_index)].joint_intervals;
            auto record_sample_coverage = [&](std::size_t sample_index) {
                const int sample_index_int = static_cast<int>(sample_index);
                first_covered_sample = std::min(first_covered_sample, sample_index_int);
                last_covered_sample = std::max(last_covered_sample, sample_index_int);
                covered_sample_count += 1;
                auto& layer = sample_layers[sample_index];
                if (!layer.empty()) {
                    dsu.unite(box_index, layer.front());
                }
                if (std::find(layer.begin(), layer.end(), box_index) == layer.end()) {
                    layer.push_back(box_index);
                }
                covered[sample_index] = true;
            };
            auto sample_in_box = [&](int sample_index) {
                if (sample_index < 0 || sample_index >= static_cast<int>(samples.size())) {
                    return false;
                }
                assimilate_local_sample_tests += 1;
                return intervals_contain_point_local(
                    box_intervals,
                    samples[static_cast<std::size_t>(sample_index)],
                    config_.query.adjacency_tolerance);
            };
            bool used_full_sample_scan = true;
            if (local_assimilate_sample_scan && !samples.empty()) {
                used_full_sample_scan = false;
                int anchor = -1;
                const std::array<int, 5> anchors = {
                    transition_hint,
                    transition_hint + 1,
                    transition_hint - 1,
                    transition_hint + 2,
                    transition_hint - 2,
                };
                for (int candidate_anchor : anchors) {
                    if (sample_in_box(candidate_anchor)) {
                        anchor = candidate_anchor;
                        break;
                    }
                }
                if (anchor >= 0) {
                    int left = anchor;
                    int right = anchor;
                    while (left > 0 && sample_in_box(left - 1)) {
                        --left;
                    }
                    while (right + 1 < static_cast<int>(samples.size()) &&
                           sample_in_box(right + 1)) {
                        ++right;
                    }
                    for (int sample_index = left; sample_index <= right; ++sample_index) {
                        record_sample_coverage(static_cast<std::size_t>(sample_index));
                    }
                    assimilate_local_hits += 1;
                } else {
                    used_full_sample_scan = true;
                    assimilate_full_scan_fallbacks += 1;
                }
            }
            if (used_full_sample_scan) {
                for (std::size_t sample_index = 0; sample_index < samples.size(); ++sample_index) {
                    if (!intervals_contain_point_local(box_intervals,
                                                       samples[sample_index],
                                                       config_.query.adjacency_tolerance)) {
                        continue;
                    }
                    record_sample_coverage(sample_index);
                }
            }
            if (covered_sample_count > 0) {
                const int span = last_covered_sample - first_covered_sample + 1;
                assimilate_coverage_boxes += 1;
                assimilate_coverage_span_sum += static_cast<double>(span);
                assimilate_coverage_span_max = std::max(assimilate_coverage_span_max, span);
                context.diagnostics().add_counter(
                    "query_bridge.direct_corridor_assimilate_covered_samples",
                    static_cast<double>(covered_sample_count));
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
            if (covered_sample_count > 0) {
                add_layer(first_covered_sample - 1);
                add_layer(first_covered_sample);
                add_layer(first_covered_sample + 1);
                add_layer(last_covered_sample - 1);
                add_layer(last_covered_sample);
                add_layer(last_covered_sample + 1);
            }
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
            return covered_sample_count;
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
                const int covered_count = assimilate_box(duplicate_index, transition_hint);
                if (covered_count == 0) {
                    repair_milestones.push_back(
                        {seed_path_param(seed, transition_hint), seed, duplicate_index});
                }
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
            if (box.tree_id != kInvalidOracleNodeId) {
                oracle_->reserve_node(box.tree_id, box.id);
            }
            const int box_index = static_cast<int>(boxes_.size());
            boxes_.push_back(box);
            raw_boxes_.push_back(box);
            if (result.node != kInvalidOracleNodeId) {
                node_to_box_index.emplace(result.node, box_index);
            }
            if (use_partition_cover_index) {
                corridor_new_box_indices.push_back(box_index);
                append_direct_partition_batch(false);
            } else {
                direct_box_index.add_box(boxes_.back(),
                                         box_index,
                                         config_.query.adjacency_tolerance);
            }
            if (use_partition_neighbor_candidates) {
                box_id_to_index[box.id] = box_index;
            }
            dsu.add();
            const int covered_count = assimilate_box(box_index, transition_hint);
            if (covered_count == 0) {
                repair_milestones.push_back(
                    {seed_path_param(seed, transition_hint), seed, box_index});
            }
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
        direct_options.materialize_result_node = false;
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
            const bool residual_milestone_segments =
                env_int_or_default("RBF_QUERY_BRIDGE_RESIDUAL_MILESTONE_SEGMENTS", 0) != 0;
            context.diagnostics().set_value(
                "query_bridge.direct_corridor_residual_milestone_segments",
                residual_milestone_segments ? 1.0 : 0.0);
            context.diagnostics().set_value(
                "query_bridge.direct_corridor_repair_milestones",
                static_cast<double>(repair_milestones.size()));
            auto insert_residual_segment = [&](int lhs_index,
                                               int rhs_index,
                                               Eigen::VectorXd lhs_point,
                                               Eigen::VectorXd rhs_point,
                                               int sample_gap) {
                if (lhs_index < 0 || rhs_index < 0 ||
                    lhs_index >= static_cast<int>(boxes_.size()) ||
                    rhs_index >= static_cast<int>(boxes_.size())) {
                    return false;
                }
                if (dsu.find(lhs_index) == dsu.find(rhs_index)) {
                    return false;
                }
                std::vector<Eigen::VectorXd> gap_path{std::move(lhs_point), std::move(rhs_point)};
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
                    return false;
                }
                const auto segment_insert_t0 =
                    detailed_direct_timing ? Clock::now() : Clock::time_point{};
                const int edge_id = add_segment_edge_partition_first(
                    boxes_[static_cast<std::size_t>(lhs_index)].id,
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
                        std::max(local_segment_gap_samples_max, sample_gap);
                    dsu.unite(lhs_index, rhs_index);
                    return true;
                }
                return false;
            };
            if (residual_milestone_segments) {
                std::vector<ResidualMilestone> milestones;
                milestones.reserve(samples.size() + repair_milestones.size());
                for (std::size_t sample_index = 0; sample_index < sample_layers.size(); ++sample_index) {
                    const auto& layer = sample_layers[sample_index];
                    if (!layer.empty()) {
                        milestones.push_back(
                            {static_cast<double>(sample_index),
                             samples[sample_index],
                             layer.front()});
                    }
                }
                for (const auto& milestone : repair_milestones) {
                    if (milestone.box_index >= 0 &&
                        milestone.box_index < static_cast<int>(boxes_.size())) {
                        milestones.push_back(milestone);
                    }
                }
                std::stable_sort(milestones.begin(),
                                 milestones.end(),
                                 [](const ResidualMilestone& lhs,
                                    const ResidualMilestone& rhs) {
                                     if (std::abs(lhs.param - rhs.param) > 1e-9) {
                                         return lhs.param < rhs.param;
                                     }
                                     return lhs.box_index < rhs.box_index;
                                 });
                std::vector<ResidualMilestone> compact;
                compact.reserve(milestones.size());
                for (const auto& milestone : milestones) {
                    if (milestone.box_index < 0) {
                        continue;
                    }
                    if (!compact.empty() &&
                        std::abs(compact.back().param - milestone.param) <= 1e-9 &&
                        dsu.find(compact.back().box_index) == dsu.find(milestone.box_index)) {
                        continue;
                    }
                    compact.push_back(milestone);
                }
                context.diagnostics().set_value(
                    "query_bridge.direct_corridor_residual_milestones",
                    static_cast<double>(compact.size()));
                for (std::size_t index = 0; index + 1 < compact.size(); ++index) {
                    const auto& lhs = compact[index];
                    const auto& rhs = compact[index + 1];
                    if (rhs.param <= lhs.param + 1e-9) {
                        continue;
                    }
                    const int sample_gap = static_cast<int>(
                        std::ceil(std::max(0.0, rhs.param - lhs.param)));
                    insert_residual_segment(lhs.box_index,
                                            rhs.box_index,
                                            lhs.point,
                                            rhs.point,
                                            sample_gap);
                }
            } else {
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
                const auto lhs_point = samples[static_cast<std::size_t>(lhs_sample)];
                const auto rhs_point = samples[static_cast<std::size_t>(rhs_sample)];
                const bool inserted = insert_residual_segment(lhs_index,
                                                              rhs_index,
                                                              lhs_point,
                                                              rhs_point,
                                                              rhs_sample - lhs_sample);
                if (!inserted) {
                    if (group_residual_gaps && gap_group.first < gap_group.second) {
                        const int mid = (gap_group.first + gap_group.second) / 2;
                        pending_gap_groups.emplace_back(mid + 1, gap_group.second);
                        pending_gap_groups.emplace_back(gap_group.first, mid);
                        context.diagnostics().add_counter(
                            "query_bridge.direct_corridor_segment_group_splits");
                    }
                    continue;
                }
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
        context.diagnostics().set_value(
            "query_bridge.direct_corridor_assimilate_coverage_span_max",
            static_cast<double>(assimilate_coverage_span_max));
        context.diagnostics().set_value(
            "query_bridge.direct_corridor_assimilate_coverage_span_mean",
            assimilate_coverage_boxes > 0
                ? assimilate_coverage_span_sum / static_cast<double>(assimilate_coverage_boxes)
                : 0.0);
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
            context.diagnostics().add_counter("query_bridge.direct_corridor_partition_append_calls",
                                              static_cast<double>(direct_partition_append_calls));
            context.diagnostics().add_counter("query_bridge.direct_corridor_partition_append_boxes",
                                              static_cast<double>(direct_partition_append_boxes));
            context.diagnostics().add_counter("query_bridge.direct_corridor_assimilate_calls",
                                              static_cast<double>(assimilate_calls));
            context.diagnostics().add_counter("query_bridge.direct_corridor_assimilate_sample_scan_ms",
                                              assimilate_sample_scan_ms);
            context.diagnostics().add_counter("query_bridge.direct_corridor_assimilate_local_hits",
                                              static_cast<double>(assimilate_local_hits));
            context.diagnostics().add_counter("query_bridge.direct_corridor_assimilate_full_scan_fallbacks",
                                              static_cast<double>(assimilate_full_scan_fallbacks));
            context.diagnostics().add_counter("query_bridge.direct_corridor_assimilate_local_sample_tests",
                                              static_cast<double>(assimilate_local_sample_tests));
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
            set_query_bridge_task_value("direct_corridor_partition_append_calls",
                                        static_cast<double>(direct_partition_append_calls));
            set_query_bridge_task_value("direct_corridor_partition_append_boxes",
                                        static_cast<double>(direct_partition_append_boxes));
            set_query_bridge_task_value("direct_corridor_assimilate_calls",
                                        static_cast<double>(assimilate_calls));
            set_query_bridge_task_value("direct_corridor_assimilate_sample_scan_ms",
                                        assimilate_sample_scan_ms);
            set_query_bridge_task_value("direct_corridor_assimilate_local_hits",
                                        static_cast<double>(assimilate_local_hits));
            set_query_bridge_task_value("direct_corridor_assimilate_full_scan_fallbacks",
                                        static_cast<double>(assimilate_full_scan_fallbacks));
            set_query_bridge_task_value("direct_corridor_assimilate_local_sample_tests",
                                        static_cast<double>(assimilate_local_sample_tests));
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
            set_query_bridge_task_value("direct_corridor_assimilate_coverage_span_max",
                                        static_cast<double>(assimilate_coverage_span_max));
            set_query_bridge_task_value(
                "direct_corridor_assimilate_coverage_span_mean",
                assimilate_coverage_boxes > 0
                    ? assimilate_coverage_span_sum / static_cast<double>(assimilate_coverage_boxes)
                    : 0.0);
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
        auto set_direct_corridor_ffb_diag = [&](const std::string& ffb_key,
                                                const std::string& suffix) {
            set_query_bridge_task_value(
                "direct_corridor_" + suffix,
                context.diagnostics().value("ffb." + ffb_key, 0.0));
        };
        set_direct_corridor_ffb_diag("find_calls", "ffb_find_calls");
        set_direct_corridor_ffb_diag("binary_requested", "ffb_binary_requested");
        set_direct_corridor_ffb_diag("virtual_sparse_binary_attempts",
                                     "ffb_virtual_sparse_binary_attempts");
        set_direct_corridor_ffb_diag("virtual_sparse_binary_successes",
                                     "ffb_virtual_sparse_binary_successes");
        set_direct_corridor_ffb_diag("virtual_sparse_binary_probes",
                                     "ffb_virtual_sparse_binary_probes");
        set_direct_corridor_ffb_diag("binary_materialized_fallback_calls",
                                     "ffb_binary_materialized_fallback_calls");
        set_direct_corridor_ffb_diag("binary_blocked_adaptive_depths",
                                     "ffb_binary_blocked_adaptive_depths");
        set_direct_corridor_ffb_diag("binary_virtual_unsupported",
                                     "ffb_binary_virtual_unsupported");
        set_direct_corridor_ffb_diag("linear_descent_calls",
                                     "ffb_linear_descent_calls");
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
                const bool add_full_residual_overlay_when_connected =
                    env_int_or_default(
                        "RBF_QUERY_BRIDGE_FULL_RESIDUAL_OVERLAY_WHEN_CONNECTED",
                        0) != 0;
                int full_edge_id = -1;
                if (add_full_residual_overlay_when_connected) {
                    const PathAuditCheck full_residual_audit =
                        audit_waypoint_path(corridor_path,
                                            checker,
                                            config_.query.audit_resolution,
                                            config_.query.audit_segment_step);
                    if (full_residual_audit.passed) {
                        full_edge_id = add_segment_edge_partition_first(
                            source_box_id,
                            target_box_id,
                            corridor_path,
                            SegmentEdgeType::QueryBridge,
                            bridge_rrt.segment_resolution,
                            SegmentEdgeValidation::CollisionChecked,
                            true,
                            bridge_edge_query_index);
                        if (full_edge_id >= 0) {
                            context.diagnostics().add_counter(
                                "query_bridge.direct_corridor_full_residual_edges");
                            context.diagnostics().add_counter(
                                "query_bridge.direct_corridor_full_residual_edges_with_local_overlay");
                            set_query_bridge_task_value("direct_corridor_full_residual_edge",
                                                        1.0);
                        }
                    } else {
                        context.diagnostics().add_counter(
                            "query_bridge.direct_corridor_full_residual_audit_rejects");
                    }
                }
                try_adopt_certified_subchain(source_box_id,
                                             target_box_id,
                                             "local_residual_overlay");
                invalidate_query_cache();
                return finish_direct_corridor(direct_added + repair_added + local_segment_edges_added +
                                              (full_edge_id >= 0 ? 1 : 0));
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
        std::vector<std::vector<Eigen::VectorXd>> waypoint_fallback_paths;
        bool waypoint_path_from_partition_query = false;
        std::vector<Eigen::VectorXd> hipac_candidate_path;
        bool hipac_online_satisfied = false;
        bool direct_start_goal_satisfied = false;
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
    auto box_id_contains_query_point = [&](int box_id,
                                           const Eigen::Ref<const Eigen::VectorXd>& point) {
        if (box_id < 0) {
            return false;
        }
        const BoxNode* box = find_box_by_id(boxes_, box_id);
        return box != nullptr &&
               intervals_contain_point_local(box->joint_intervals,
                                             point,
                                             config_.query.adjacency_tolerance);
    };
    auto refresh_located_or_keep_anchor = [&](int anchor_box_id,
                                              const Eigen::Ref<const Eigen::VectorXd>& point,
                                              const char* endpoint_name) {
        const int located = locate_existing_box_for_query_bridge(point);
        if (located >= 0) {
            return located;
        }
        if (box_id_contains_query_point(anchor_box_id, point)) {
            last_build_.diagnostics[std::string("query_bridge.endpoint_anchor_keep_after_lookup_miss.") +
                                    endpoint_name] += 1.0;
            return anchor_box_id;
        }
        return -1;
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
            start_box_id = refresh_located_or_keep_anchor(start_box_id,
                                                          starts[index],
                                                          "start");
        }
        if (goal_box_id >= 0) {
            goal_box_id = refresh_located_or_keep_anchor(goal_box_id,
                                                         goals[index],
                                                         "goal");
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
    const bool direct_start_goal_segment =
        env_int_or_default("RBF_QUERY_BRIDGE_DIRECT_SEGMENT_AFTER_RRT", 0) != 0 &&
        env_int_or_default("RBF_QUERY_BRIDGE_DIRECT_START_GOAL_SEGMENT", 1) != 0 &&
        config_.connector.segment_edges_enabled &&
        config_.connector.rrt_segment_edges;
    const bool fast_direct_segment_after_rrt =
        env_int_or_default("RBF_QUERY_BRIDGE_DIRECT_SEGMENT_AFTER_RRT", 0) != 0 &&
        env_int_or_default("RBF_QUERY_BRIDGE_FAST_DIRECT_SEGMENT_AFTER_RRT", 0) != 0 &&
        config_.connector.segment_edges_enabled &&
        config_.connector.rrt_segment_edges;
    const double fast_direct_segment_after_rrt_min_length =
        std::max(0.0,
                 env_double_or_default("RBF_QUERY_BRIDGE_DIRECT_SEGMENT_AFTER_RRT_MIN_LENGTH",
                                       0.0));
    batch_context.diagnostics().set_value(
        "query_bridge.direct_start_goal_segment",
        direct_start_goal_segment ? 1.0 : 0.0);
    batch_context.diagnostics().set_value(
        "query_bridge.fast_direct_segment_after_rrt",
        fast_direct_segment_after_rrt ? 1.0 : 0.0);
    auto try_direct_start_goal_segment = [&](BridgeSearchTask& task) -> int {
        if (!direct_start_goal_segment || task.direct_start_goal_satisfied) {
            return 0;
        }
        const int source_box_id =
            locate_existing_box_for_query_bridge(task.start);
        const int target_box_id =
            locate_existing_box_for_query_bridge(task.goal);
        if (source_box_id < 0 || target_box_id < 0 || source_box_id == target_box_id) {
            batch_context.diagnostics().add_counter(
                "query_bridge.direct_start_goal_segment_missing_endpoint");
            return 0;
        }
        std::vector<Eigen::VectorXd> direct_path{task.start, task.goal};
        batch_context.diagnostics().add_counter(
            "query_bridge.direct_start_goal_segment_attempts");
        batch_context.diagnostics().add_counter(
            task_key(task.index, "direct_start_goal_segment_attempts"));
        CollisionChecker checker = make_audit_checker(audit_robot_, scene_, config_.query);
        const PathAuditCheck audit =
            audit_waypoint_path(direct_path,
                                checker,
                                config_.query.audit_resolution,
                                config_.query.audit_segment_step);
        if (!audit.passed) {
            batch_context.diagnostics().add_counter(
                "query_bridge.direct_start_goal_segment_audit_rejects");
            batch_context.diagnostics().add_counter(
                task_key(task.index, "direct_start_goal_segment_audit_rejects"));
            return 0;
        }
        const int edge_id = add_segment_edge_partition_first(
            source_box_id,
            target_box_id,
            direct_path,
            SegmentEdgeType::QueryBridge,
            config_.query.audit_resolution,
            SegmentEdgeValidation::CollisionChecked,
            true,
            edge_query_index_for(task));
        if (edge_id < 0) {
            batch_context.diagnostics().add_counter(
                "query_bridge.direct_start_goal_segment_add_fail");
            batch_context.diagnostics().add_counter(
                task_key(task.index, "direct_start_goal_segment_add_fail"));
            return 0;
        }
        task.direct_start_goal_satisfied = true;
        batch_context.diagnostics().add_counter(
            "query_bridge.direct_start_goal_segment_edges");
        batch_context.diagnostics().add_counter(
            task_key(task.index, "direct_start_goal_segment_edges"));
        invalidate_query_cache();
        sync_adaptive_partition_segment_edges(&last_build_,
                                               "query_bridge.direct_start_goal_segment");
        refresh_adaptive_partition_diagnostics(&last_build_);
        return 1;
    };
    auto try_fast_direct_segment_after_rrt = [&](BridgeSearchTask& task) -> int {
        if (!fast_direct_segment_after_rrt || task.waypoint_path.empty()) {
            return 0;
        }
        std::vector<std::vector<Eigen::VectorXd>> candidate_paths;
        candidate_paths.push_back(task.waypoint_path);
        const bool fast_direct_shortcut =
            env_int_or_default("RBF_QUERY_BRIDGE_FAST_DIRECT_SHORTCUT", 1) != 0;
        if (fast_direct_shortcut && task.waypoint_path.size() > 2) {
            const double before_length = path_length(task.waypoint_path);
            CollisionChecker checker = make_audit_checker(audit_robot_, scene_, config_.query);
            std::vector<Eigen::VectorXd> shortened =
                collision_shortcut_path(task.waypoint_path,
                                        checker,
                                        collision_shortcut_resolution(config_.query));
            const double after_length = path_length(shortened);
            batch_context.diagnostics().add_counter(
                "query_bridge.fast_direct_segment_after_rrt_shortcut_attempts");
            batch_context.diagnostics().add_counter(
                task_key(task.index, "fast_direct_segment_after_rrt_shortcut_attempts"));
            if (!shortened.empty() && after_length + 1e-12 < before_length) {
                candidate_paths.push_back(std::move(shortened));
                batch_context.diagnostics().add_counter(
                    "query_bridge.fast_direct_segment_after_rrt_shortcut_accepts");
                batch_context.diagnostics().add_counter(
                    "query_bridge.fast_direct_segment_after_rrt_shortcut_delta",
                    before_length - after_length);
                batch_context.diagnostics().add_counter(
                    task_key(task.index, "fast_direct_segment_after_rrt_shortcut_accepts"));
                batch_context.diagnostics().add_counter(
                    task_key(task.index, "fast_direct_segment_after_rrt_shortcut_delta"),
                    before_length - after_length);
            }
            const int random_shortcut_iters = std::max(
                0,
                env_int_or_default("RBF_QUERY_BRIDGE_FAST_DIRECT_RANDOM_SHORTCUT_ITERS", 0));
            const auto& random_source = candidate_paths.back();
            if (random_shortcut_iters > 0 && random_source.size() > 2U) {
                const double random_before_length = path_length(random_source);
                const std::uint32_t shortcut_seed =
                    static_cast<std::uint32_t>(0x9e3779b9U ^
                                               ((static_cast<std::uint32_t>(task.query_index) + 1U) * 2654435761U) ^
                                               (static_cast<std::uint32_t>(task.index + 1U) * 2246822519U) ^
                                               static_cast<std::uint32_t>(random_source.size()));
                std::vector<Eigen::VectorXd> random_shortened =
                    random_collision_shortcut_path(random_source,
                                                   checker,
                                                   collision_shortcut_resolution(config_.query),
                                                   random_shortcut_iters,
                                                   shortcut_seed);
                const double random_after_length = path_length(random_shortened);
                batch_context.diagnostics().add_counter(
                    "query_bridge.fast_direct_segment_after_rrt_random_shortcut_attempts");
                batch_context.diagnostics().add_counter(
                    task_key(task.index, "fast_direct_segment_after_rrt_random_shortcut_attempts"));
                batch_context.diagnostics().add_counter(
                    "query_bridge.fast_direct_segment_after_rrt_random_shortcut_iters",
                    static_cast<double>(random_shortcut_iters));
                if (!random_shortened.empty() &&
                    random_after_length + 1e-12 < random_before_length) {
                    candidate_paths.push_back(std::move(random_shortened));
                    batch_context.diagnostics().add_counter(
                        "query_bridge.fast_direct_segment_after_rrt_random_shortcut_accepts");
                    batch_context.diagnostics().add_counter(
                        "query_bridge.fast_direct_segment_after_rrt_random_shortcut_delta",
                        random_before_length - random_after_length);
                    batch_context.diagnostics().add_counter(
                        task_key(task.index, "fast_direct_segment_after_rrt_random_shortcut_accepts"));
                    batch_context.diagnostics().add_counter(
                        task_key(task.index, "fast_direct_segment_after_rrt_random_shortcut_delta"),
                        random_before_length - random_after_length);
                }
            }
        }
        std::sort(candidate_paths.begin(),
                  candidate_paths.end(),
                  [](const auto& lhs, const auto& rhs) {
                      return path_length(lhs) < path_length(rhs);
                  });
        candidate_paths.erase(std::unique(candidate_paths.begin(),
                                          candidate_paths.end(),
                                          [](const auto& lhs, const auto& rhs) {
                                              if (lhs.size() != rhs.size()) {
                                                  return false;
                                              }
                                              for (std::size_t index = 0; index < lhs.size(); ++index) {
                                                  if ((lhs[index] - rhs[index]).norm() > 1e-12) {
                                                      return false;
                                                  }
                                              }
                                              return true;
                                          }),
                              candidate_paths.end());
        if (candidate_paths.empty() ||
            !(path_length(candidate_paths.front()) >= fast_direct_segment_after_rrt_min_length)) {
            batch_context.diagnostics().add_counter(
                "query_bridge.fast_direct_segment_after_rrt_length_rejects");
            batch_context.diagnostics().add_counter(
                task_key(task.index, "fast_direct_segment_after_rrt_length_rejects"));
            return 0;
        }
        const int source_box_id = locate_existing_box_for_query_bridge(task.start);
        const int target_box_id = locate_existing_box_for_query_bridge(task.goal);
        if (source_box_id < 0 || target_box_id < 0 || source_box_id == target_box_id) {
            batch_context.diagnostics().add_counter(
                "query_bridge.fast_direct_segment_after_rrt_missing_endpoint");
            batch_context.diagnostics().add_counter(
                task_key(task.index, "fast_direct_segment_after_rrt_missing_endpoint"));
            return 0;
        }
        CollisionChecker strict_checker = make_audit_checker(audit_robot_, scene_, config_.query);
        int edge_id = -1;
        double added_length = std::numeric_limits<double>::infinity();
        for (std::size_t candidate_index = 0; candidate_index < candidate_paths.size(); ++candidate_index) {
            const auto& candidate_path = candidate_paths[candidate_index];
            if (path_length(candidate_path) + 1e-12 < fast_direct_segment_after_rrt_min_length) {
                continue;
            }
            batch_context.diagnostics().add_counter(
                "query_bridge.fast_direct_segment_after_rrt_add_candidates");
            const PathAuditCheck candidate_audit =
                audit_waypoint_path(candidate_path,
                                    strict_checker,
                                    config_.query.audit_resolution,
                                    config_.query.audit_segment_step);
            if (!candidate_audit.passed) {
                batch_context.diagnostics().add_counter(
                    "query_bridge.fast_direct_segment_after_rrt_candidate_audit_rejects");
                batch_context.diagnostics().add_counter(
                    task_key(task.index, "fast_direct_segment_after_rrt_candidate_audit_rejects"));
                continue;
            }
            edge_id = add_segment_edge_partition_first(
                source_box_id,
                target_box_id,
                candidate_path,
                SegmentEdgeType::QueryBridge,
                task.bridge_rrt.segment_resolution,
                SegmentEdgeValidation::CollisionChecked,
                true,
                edge_query_index_for(task));
            if (edge_id >= 0) {
                added_length = path_length(candidate_path);
                if (candidate_index > 0) {
                    batch_context.diagnostics().add_counter(
                        "query_bridge.fast_direct_segment_after_rrt_fallback_candidate_success");
                }
                break;
            }
            batch_context.diagnostics().add_counter(
                "query_bridge.fast_direct_segment_after_rrt_add_candidate_fail");
        }
        if (edge_id < 0) {
            batch_context.diagnostics().add_counter(
                "query_bridge.fast_direct_segment_after_rrt_add_fail");
            batch_context.diagnostics().add_counter(
                task_key(task.index, "fast_direct_segment_after_rrt_add_fail"));
            return 0;
        }
        invalidate_query_cache();
        batch_context.diagnostics().add_counter(
            "query_bridge.fast_direct_segment_after_rrt_edges");
        batch_context.diagnostics().add_counter(
            task_key(task.index, "fast_direct_segment_after_rrt_edges"));
        batch_context.diagnostics().set_value(
            task_key(task.index, "fast_direct_segment_after_rrt_length"),
            added_length);
        return 1;
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
        add("direct_corridor_ffb_find_calls", "query_bridge.direct_corridor_ffb_find_calls_total");
        add("direct_corridor_ffb_binary_requested",
            "query_bridge.direct_corridor_ffb_binary_requested_total");
        add("direct_corridor_ffb_virtual_sparse_binary_attempts",
            "query_bridge.direct_corridor_ffb_virtual_sparse_binary_attempts_total");
        add("direct_corridor_ffb_virtual_sparse_binary_successes",
            "query_bridge.direct_corridor_ffb_virtual_sparse_binary_successes_total");
        add("direct_corridor_ffb_virtual_sparse_binary_probes",
            "query_bridge.direct_corridor_ffb_virtual_sparse_binary_probes_total");
        add("direct_corridor_ffb_binary_materialized_fallback_calls",
            "query_bridge.direct_corridor_ffb_binary_materialized_fallback_calls_total");
        add("direct_corridor_ffb_binary_blocked_adaptive_depths",
            "query_bridge.direct_corridor_ffb_binary_blocked_adaptive_depths_total");
        add("direct_corridor_ffb_binary_virtual_unsupported",
            "query_bridge.direct_corridor_ffb_binary_virtual_unsupported_total");
        add("direct_corridor_ffb_linear_descent_calls",
            "query_bridge.direct_corridor_ffb_linear_descent_calls_total");
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
        add("direct_corridor_partition_append_calls",
            "query_bridge.direct_corridor_partition_append_calls");
        add("direct_corridor_partition_append_boxes",
            "query_bridge.direct_corridor_partition_append_boxes");
        add("direct_corridor_assimilate_calls",
            "query_bridge.direct_corridor_assimilate_calls");
        add("direct_corridor_assimilate_sample_scan_ms",
            "query_bridge.direct_corridor_assimilate_sample_scan_ms");
        add("direct_corridor_assimilate_local_hits",
            "query_bridge.direct_corridor_assimilate_local_hits");
        add("direct_corridor_assimilate_full_scan_fallbacks",
            "query_bridge.direct_corridor_assimilate_full_scan_fallbacks");
        add("direct_corridor_assimilate_local_sample_tests",
            "query_bridge.direct_corridor_assimilate_local_sample_tests");
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
    if (direct_start_goal_segment) {
        for (auto& task : tasks) {
            const int added = try_direct_start_goal_segment(task);
            if (added > 0) {
                added_by_query[task.index] += added;
            }
        }
    }
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
    const double query_bridge_rrt_clearance =
        std::max(0.0, env_double_or_default("RBF_QUERY_BRIDGE_RRT_CLEARANCE", 0.0));
    const std::vector<double> local_radius_schedule =
        env_double_list_or_empty("RBF_QUERY_BRIDGE_LOCAL_RADIUS_SCHEDULE");
    const bool local_radius_append_unrestricted_attempt =
        env_int_or_default("RBF_QUERY_BRIDGE_LOCAL_RADIUS_APPEND_UNRESTRICTED_ATTEMPT", 1) != 0;
    const int query_bridge_rrt_optimize_after_first_iters =
        std::max(0, env_int_or_default("RBF_QUERY_BRIDGE_RRT_OPTIMIZE_AFTER_FIRST_ITERS", 0));
    const int query_bridge_attempt_fallback_paths =
        std::max(0, env_int_or_default("RBF_QUERY_BRIDGE_ATTEMPT_FALLBACK_PATHS", 0));
    batch_context.diagnostics().set_value(
        "query_bridge.rrt_fixed_iters",
        static_cast<double>(query_bridge_rrt_fixed_iters));
    batch_context.diagnostics().set_value(
        "query_bridge.rrt_fixed_timeout_ms",
        query_bridge_rrt_fixed_timeout_ms);
    batch_context.diagnostics().set_value(
        "query_bridge.rrt_clearance",
        query_bridge_rrt_clearance);
    batch_context.diagnostics().set_value(
        "query_bridge.local_radius_schedule_size",
        static_cast<double>(local_radius_schedule.size()));
    batch_context.diagnostics().set_value(
        "query_bridge.local_radius_append_unrestricted_attempt",
        local_radius_append_unrestricted_attempt ? 1.0 : 0.0);
    batch_context.diagnostics().set_value(
        "query_bridge.rrt_optimize_after_first_iters",
        static_cast<double>(query_bridge_rrt_optimize_after_first_iters));
    batch_context.diagnostics().set_value(
        "query_bridge.attempt_fallback_paths",
        static_cast<double>(query_bridge_attempt_fallback_paths));
    const std::vector<int> no_path_retry_budget_iters =
        env_int_list_or_empty("RBF_QUERY_BRIDGE_NO_PATH_RETRY_BUDGET_ITERS");
    const std::vector<int> no_path_retry_budget_attempts =
        env_int_list_or_empty("RBF_QUERY_BRIDGE_NO_PATH_RETRY_BUDGET_ATTEMPTS");
    const std::size_t no_path_retry_budget_stages =
        std::min(no_path_retry_budget_iters.size(), no_path_retry_budget_attempts.size());
    batch_context.diagnostics().set_value(
        "query_bridge.no_path_retry_budget_stages",
        static_cast<double>(no_path_retry_budget_stages));
    for (std::size_t stage = 0; stage < no_path_retry_budget_stages; ++stage) {
        const std::string prefix =
            "query_bridge.no_path_retry_budget_stage." + std::to_string(stage) + ".";
        batch_context.diagnostics().set_value(prefix + "iters",
                                              static_cast<double>(no_path_retry_budget_iters[stage]));
        batch_context.diagnostics().set_value(prefix + "attempts",
                                              static_cast<double>(no_path_retry_budget_attempts[stage]));
    }
    const bool post_rrt_skip_forced =
        env_int_or_default("RBF_QUERY_BRIDGE_POST_RRT_SKIP_FORCED", 0) != 0;
    batch_context.diagnostics().set_value("query_bridge.post_rrt_skip_forced",
                                          post_rrt_skip_forced ? 1.0 : 0.0);
    const bool parallel_rrt_early_stop =
        env_int_or_default("RBF_QUERY_BRIDGE_PARALLEL_RRT_EARLY_STOP", 0) != 0;
    const int parallel_rrt_early_stop_min_successes =
        std::max(1, env_int_or_default("RBF_QUERY_BRIDGE_PARALLEL_RRT_EARLY_STOP_MIN_SUCCESSES", 1));
    const double parallel_rrt_early_stop_ratio =
        std::max(1.0, env_double_or_default("RBF_QUERY_BRIDGE_PARALLEL_RRT_EARLY_STOP_RATIO", 1.75));
    const double parallel_rrt_early_stop_additive =
        std::max(0.0, env_double_or_default("RBF_QUERY_BRIDGE_PARALLEL_RRT_EARLY_STOP_ADDITIVE", 0.75));
    batch_context.diagnostics().set_value("query_bridge.parallel_rrt_early_stop_enabled",
                                          parallel_rrt_early_stop ? 1.0 : 0.0);
    auto rrt_path_good_enough_for_task = [&](const BridgeSearchTask& task,
                                             const std::vector<Eigen::VectorXd>& path) {
        if (path.empty()) {
            return false;
        }
        const double direct = (task.goal - task.start).norm();
        if (direct <= 1e-9) {
            return true;
        }
        const double length = path_length(path);
        return length <= std::max(direct * parallel_rrt_early_stop_ratio,
                                  direct + parallel_rrt_early_stop_additive);
    };
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
    auto run_task_attempt = [&](const BridgeSearchTask& task,
                                int attempt,
                                int override_fixed_iters,
                                std::shared_ptr<std::atomic<bool>> cancel_override =
                                    std::shared_ptr<std::atomic<bool>>{}) {
        const int scheduled_attempt = attempt + query_bridge_attempt_offset;
        Robot bridge_robot = make_sbf_clearance_robot(audit_robot_, query_bridge_rrt_clearance);
        CollisionChecker checker =
            query_bridge_rrt_clearance > 0.0
                ? CollisionChecker(bridge_robot, scene_)
                : make_audit_checker(audit_robot_, scene_, config_.query);
        RRTConnectConfig config =
            task.short_local_profiles.empty()
                ? task.bridge_rrt
                : task.short_local_profiles[
                      static_cast<std::size_t>(scheduled_attempt) % task.short_local_profiles.size()];
        if (!local_radius_schedule.empty() &&
            attempt >= 0 &&
            static_cast<std::size_t>(attempt) < local_radius_schedule.size()) {
            const double scheduled_radius =
                local_radius_schedule[static_cast<std::size_t>(attempt)];
            if (scheduled_radius >= 0.0) {
                config.local_sampling_radius = scheduled_radius;
            }
        }
        config.optimize_after_first_iters = query_bridge_rrt_optimize_after_first_iters;
        const int effective_fixed_iters =
            override_fixed_iters > 0 ? override_fixed_iters : query_bridge_rrt_fixed_iters;
        if (effective_fixed_iters > 0) {
            config.max_iters = effective_fixed_iters;
            config.timeout_ms = query_bridge_rrt_fixed_timeout_ms;
        } else {
            config.timeout_ms = std::max(1.0, config_.connector.per_pair_timeout_ms);
        }
        std::vector<Eigen::VectorXd> path = rrt_connect(
            task.start,
            task.goal,
            checker,
            bridge_robot,
            config,
                derived_planner_seed(config_.grower.rng_seed,
                                     kSeedBatchBridgeOffset,
                                     scheduled_attempt,
                                     task.query_index,
                                     task.short_local_bridge ? 0 : kSeedAttemptStride),
            cancel_override ? cancel_override : batch_context.native_cancel_flag());
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
        const bool multi_axis_detour =
            env_int_or_default("RBF_QUERY_BRIDGE_DETOUR_MULTI_AXIS", 0) != 0;
        const int random_candidates = std::max(
            0,
            env_int_or_default("RBF_QUERY_BRIDGE_DETOUR_RANDOM_CANDIDATES", 0));
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
        if (multi_axis_detour && dim_limit >= 2) {
            for (int first_item = 0; first_item < dim_limit && candidates < max_candidates; ++first_item) {
                const int first_dim = dims[static_cast<std::size_t>(first_item)];
                const double first_width = domain[static_cast<std::size_t>(first_dim)].width();
                for (int second_item = first_item + 1;
                     second_item < dim_limit && candidates < max_candidates;
                     ++second_item) {
                    const int second_dim = dims[static_cast<std::size_t>(second_item)];
                    const double second_width = domain[static_cast<std::size_t>(second_dim)].width();
                    for (int round = 1; round <= rounds && candidates < max_candidates; ++round) {
                        const double first_mag = std::min(0.35 * std::max(0.0, first_width),
                                                          base_offset * static_cast<double>(round));
                        const double second_mag = std::min(0.35 * std::max(0.0, second_width),
                                                           base_offset * static_cast<double>(round));
                        if (first_mag <= 1e-9 || second_mag <= 1e-9) {
                            continue;
                        }
                        for (double first_sign : {1.0, -1.0}) {
                            for (double second_sign : {1.0, -1.0}) {
                                Eigen::VectorXd single = mid;
                                single[first_dim] += first_sign * first_mag;
                                single[second_dim] += second_sign * second_mag;
                                single = clamp_to_domain(std::move(single));
                                if ((single - mid).norm() > 1e-9) {
                                    try_path({task.start, single, task.goal});
                                }
                                if (candidates >= max_candidates) {
                                    break;
                                }
                                Eigen::VectorXd first = task.start + two_bend_alpha * delta;
                                Eigen::VectorXd second = task.start + (1.0 - two_bend_alpha) * delta;
                                first[first_dim] += first_sign * first_mag;
                                first[second_dim] += second_sign * second_mag;
                                second[first_dim] += first_sign * first_mag;
                                second[second_dim] += second_sign * second_mag;
                                first = clamp_to_domain(std::move(first));
                                second = clamp_to_domain(std::move(second));
                                try_path({task.start, first, second, task.goal});
                                if (candidates >= max_candidates) {
                                    break;
                                }
                            }
                            if (candidates >= max_candidates) {
                                break;
                            }
                        }
                    }
                }
            }
        }
        if (random_candidates > 0 && dim_limit > 0 && candidates < max_candidates) {
            const int random_budget = std::min(random_candidates, max_candidates - candidates);
            std::mt19937 rng(static_cast<std::uint32_t>(
                derived_planner_seed(config_.grower.rng_seed,
                                     kSeedBatchBridgeOffset,
                                     task.index,
                                     task.query_index,
                                     41443)));
            std::uniform_int_distribution<int> dim_pick(0, dim_limit - 1);
            std::uniform_real_distribution<double> unit(-1.0, 1.0);
            const double max_scale = std::max(1.0, static_cast<double>(rounds));
            for (int sample = 0; sample < random_budget && candidates < max_candidates; ++sample) {
                const int first_dim = dims[static_cast<std::size_t>(dim_pick(rng))];
                int second_dim = first_dim;
                if (dim_limit > 1) {
                    for (int guard = 0; guard < 4 && second_dim == first_dim; ++guard) {
                        second_dim = dims[static_cast<std::size_t>(dim_pick(rng))];
                    }
                }
                Eigen::VectorXd offset = Eigen::VectorXd::Zero(task.start.size());
                auto apply_random_dim = [&](int dim) {
                    const double width = domain[static_cast<std::size_t>(dim)].width();
                    const double limit = std::min(0.35 * std::max(0.0, width),
                                                  base_offset * max_scale);
                    if (limit > 1e-9) {
                        offset[dim] += unit(rng) * limit;
                    }
                };
                apply_random_dim(first_dim);
                if (second_dim != first_dim) {
                    apply_random_dim(second_dim);
                }
                if (offset.norm() <= 1e-9) {
                    continue;
                }
                if ((sample & 1) == 0) {
                    Eigen::VectorXd single = clamp_to_domain(mid + offset);
                    try_path({task.start, single, task.goal});
                } else {
                    Eigen::VectorXd first = clamp_to_domain(task.start + two_bend_alpha * delta + offset);
                    Eigen::VectorXd second = clamp_to_domain(task.start + (1.0 - two_bend_alpha) * delta + offset);
                    try_path({task.start, first, second, task.goal});
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
    const int waypoint_quality_retry_iters = std::max(
        0,
        env_int_or_default("RBF_QUERY_BRIDGE_WAYPOINT_QUALITY_RETRY_ITERS", 0));
    const double waypoint_quality_max_ratio = std::max(
        1.0,
        env_double_or_default("RBF_QUERY_BRIDGE_WAYPOINT_QUALITY_MAX_RATIO", 2.0));
    const double waypoint_quality_max_additive = std::max(
        0.0,
        env_double_or_default("RBF_QUERY_BRIDGE_WAYPOINT_QUALITY_MAX_ADDITIVE", 0.75));
    batch_context.diagnostics().set_value(
        "query_bridge.waypoint_quality_retry",
        waypoint_quality_retry ? 1.0 : 0.0);
	    auto improve_waypoint_if_needed = [&](BridgeSearchTask& task,
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
        std::vector<std::vector<Eigen::VectorXd>> retry_paths(
            static_cast<std::size_t>(waypoint_quality_retry_attempts));
        if (batch_context.executor().n_threads() > 1 &&
            waypoint_quality_retry_attempts > 1) {
            batch_context.executor().parallel_for(
                0,
                waypoint_quality_retry_attempts,
                [&](int retry) {
                    retry_paths[static_cast<std::size_t>(retry)] =
                        run_task_attempt(task,
                                         attempts_already_used + retry,
                                         waypoint_quality_retry_iters);
                });
        } else {
            for (int retry = 0; retry < waypoint_quality_retry_attempts; ++retry) {
                retry_paths[static_cast<std::size_t>(retry)] =
                    run_task_attempt(task,
                                     attempts_already_used + retry,
                                     waypoint_quality_retry_iters);
            }
        }
        for (auto& retry_path : retry_paths) {
            if (retry_path.empty()) {
                continue;
            }
            retry_successes += 1;
            const double length = path_length(retry_path);
            if (length < best_length) {
                if (!waypoint_path.empty() &&
                    task.waypoint_fallback_paths.size() < 4) {
                    task.waypoint_fallback_paths.push_back(waypoint_path);
                }
                best_length = length;
                waypoint_path = std::move(retry_path);
            }
            if (best_length <= limit) {
                break;
            }
        }
        if (batch_context.executor().n_threads() > 1 &&
            waypoint_quality_retry_attempts > 1) {
            batch_context.diagnostics().add_counter(
                "query_bridge.waypoint_quality_retry_parallel_batches");
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
    auto select_attempt_paths = [&](BridgeSearchTask& task,
                                    std::vector<std::vector<Eigen::VectorXd>>& attempt_paths,
                                    double& best_length) {
        std::vector<std::pair<double, std::size_t>> valid_paths;
        valid_paths.reserve(attempt_paths.size());
        for (std::size_t index = 0; index < attempt_paths.size(); ++index) {
            if (attempt_paths[index].empty()) {
                continue;
            }
            const double length = path_length(attempt_paths[index]);
            if (!std::isfinite(length)) {
                continue;
            }
            valid_paths.emplace_back(length, index);
        }
        if (valid_paths.empty()) {
            return;
        }
        const bool hybridize_attempt_paths =
            env_int_or_default("RBF_QUERY_BRIDGE_HYBRIDIZE_ATTEMPT_PATHS", 0) != 0;
        if (hybridize_attempt_paths && valid_paths.size() >= 2U) {
            const int hybrid_max_paths = std::max(
                2,
                env_int_or_default("RBF_QUERY_BRIDGE_HYBRID_MAX_PATHS", 8));
            const int hybrid_max_vertices = std::max(
                8,
                env_int_or_default("RBF_QUERY_BRIDGE_HYBRID_MAX_VERTICES", 128));
            const int hybrid_max_cross_checks = std::max(
                1,
                env_int_or_default("RBF_QUERY_BRIDGE_HYBRID_MAX_CROSS_CHECKS", 4096));
            CollisionChecker checker = make_audit_checker(audit_robot_, scene_, config_.query);
            const double best_input_length =
                std::min(best_length,
                         std::min_element(valid_paths.begin(),
                                          valid_paths.end(),
                                          [](const auto& lhs, const auto& rhs) {
                                              return lhs.first < rhs.first;
                                          })
                             ->first);
            std::vector<Eigen::VectorXd> hybrid =
                hybridize_collision_free_paths(attempt_paths,
                                               checker,
                                               collision_shortcut_resolution(config_.query),
                                               hybrid_max_paths,
                                               hybrid_max_vertices,
                                               hybrid_max_cross_checks);
            batch_context.diagnostics().add_counter(
                "query_bridge.hybridize_attempt_paths_tasks");
            if (!hybrid.empty()) {
                const double hybrid_length = path_length(hybrid);
                batch_context.diagnostics().add_counter(
                    "query_bridge.hybridize_attempt_paths_candidates");
                batch_context.diagnostics().add_counter(
                    task_key(task.index, "hybridize_attempt_paths_candidates"));
                if (hybrid_length + 1e-12 < best_input_length) {
                    const PathAuditCheck audit =
                        audit_waypoint_path(hybrid,
                                            checker,
                                            config_.query.audit_resolution,
                                            config_.query.audit_segment_step);
                    if (audit.passed) {
                        const std::size_t index = attempt_paths.size();
                        attempt_paths.push_back(std::move(hybrid));
                        valid_paths.emplace_back(hybrid_length, index);
                        batch_context.diagnostics().add_counter(
                            "query_bridge.hybridize_attempt_paths_accepts");
                        batch_context.diagnostics().add_counter(
                            "query_bridge.hybridize_attempt_paths_delta",
                            best_input_length - hybrid_length);
                        batch_context.diagnostics().add_counter(
                            task_key(task.index, "hybridize_attempt_paths_accepts"));
                    } else {
                        batch_context.diagnostics().add_counter(
                            "query_bridge.hybridize_attempt_paths_audit_rejects");
                    }
                }
            }
        }
        std::sort(valid_paths.begin(), valid_paths.end(),
                  [](const auto& lhs, const auto& rhs) {
                      if (std::abs(lhs.first - rhs.first) > 1e-12) {
                          return lhs.first < rhs.first;
                      }
                      return lhs.second < rhs.second;
                  });
        std::size_t selected_index = std::numeric_limits<std::size_t>::max();
        if (task.waypoint_path.empty() || valid_paths.front().first < best_length) {
            selected_index = valid_paths.front().second;
            if (!task.waypoint_path.empty() &&
                query_bridge_attempt_fallback_paths > 0 &&
                task.waypoint_fallback_paths.size() <
                    static_cast<std::size_t>(query_bridge_attempt_fallback_paths)) {
                task.waypoint_fallback_paths.push_back(std::move(task.waypoint_path));
                batch_context.diagnostics().add_counter(
                    "query_bridge.attempt_fallback_paths_stored");
            }
            best_length = valid_paths.front().first;
            task.waypoint_path = std::move(attempt_paths[selected_index]);
        }
        for (const auto& [length, index] : valid_paths) {
            (void)length;
            if (index == selected_index || attempt_paths[index].empty()) {
                continue;
            }
            if (query_bridge_attempt_fallback_paths <= 0 ||
                task.waypoint_fallback_paths.size() >=
                    static_cast<std::size_t>(query_bridge_attempt_fallback_paths)) {
                break;
            }
            task.waypoint_fallback_paths.push_back(std::move(attempt_paths[index]));
            batch_context.diagnostics().add_counter(
                "query_bridge.attempt_fallback_paths_stored");
            batch_context.diagnostics().add_counter(
                task_key(task.index, "attempt_fallback_paths_stored"));
        }
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

    auto bridge_query_with_waypoint_fallbacks =
        [&](BridgeSearchTask& task,
            int& added_accumulator) -> int {
        const bool evaluate_all_fallback_paths =
            env_int_or_default("RBF_QUERY_BRIDGE_EVALUATE_ALL_FALLBACK_PATHS", 0) != 0;
        batch_context.diagnostics().set_value(
            "query_bridge.evaluate_all_fallback_paths",
            evaluate_all_fallback_paths ? 1.0 : 0.0);
        std::vector<const std::vector<Eigen::VectorXd>*> candidate_paths;
        if (!task.waypoint_path.empty()) {
            candidate_paths.push_back(&task.waypoint_path);
        }
        for (const auto& fallback : task.waypoint_fallback_paths) {
            if (!fallback.empty()) {
                candidate_paths.push_back(&fallback);
            }
        }
        int total_added = 0;
        for (std::size_t candidate_index = 0;
             candidate_index < candidate_paths.size();
             ++candidate_index) {
            const auto& candidate_path = *candidate_paths[candidate_index];
            if (candidate_path.empty()) {
                continue;
            }
            if (candidate_index > 0) {
                batch_context.diagnostics().add_counter(
                    "query_bridge.waypoint_quality_fallback_attempts");
                batch_context.diagnostics().add_counter(
                    task_key(task.index, "waypoint_quality_fallback_attempts"));
            }
            const int bridge_added =
                bridge_query_with_waypoint_path(task.start,
                                                task.goal,
                                                candidate_path,
                                                task.short_local_bridge,
                                                task.bridge_rrt,
                                                task.query_index);
            total_added += bridge_added;
            added_accumulator += bridge_added;
            maybe_promote_query_repair(task, bridge_added);
            accumulate_task_direct_corridor_totals(task.index);
            if (candidate_index > 0) {
                batch_context.diagnostics().add_counter(
                    "query_bridge.waypoint_quality_fallback_added",
                    static_cast<double>(bridge_added));
            }
            if (current_query_good(task, false)) {
                if (candidate_index > 0) {
                    batch_context.diagnostics().add_counter(
                        "query_bridge.waypoint_quality_fallback_successes");
                    batch_context.diagnostics().set_value(
                        task_key(task.index, "waypoint_quality_fallback_success"),
                        1.0);
                    task.waypoint_path = candidate_path;
                }
                if (!evaluate_all_fallback_paths) {
                    break;
                }
            }
        }
        return total_added;
    };

	    const bool parallel_task_rrt =
	        env_int_or_default("RBF_QUERY_BRIDGE_PARALLEL_TASK_RRT", 1) != 0;
    batch_context.diagnostics().set_value("query_bridge.attempt_offset",
                                          static_cast<double>(query_bridge_attempt_offset));
    const bool has_segment_only_task =
        std::any_of(tasks.begin(), tasks.end(), [](const BridgeSearchTask& task) {
            return env_index_list_contains("RBF_QUERY_BRIDGE_SEGMENT_ONLY_INDICES",
                                           task.index);
        });
    if (parallel_task_rrt && !has_segment_only_task && no_path_retry_attempts == 0 &&
        no_path_retry_budget_stages == 0) {
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
	            if (task.hipac_online_satisfied ||
                    task.direct_start_goal_satisfied ||
                    current_query_good(task, true)) {
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
                    if (task.direct_start_goal_satisfied) {
                        batch_context.diagnostics().set_value(
                            task_key(task.index, "skipped_by_direct_start_goal_segment"),
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
            if (prepared[task_offset].attempts > 0 &&
                !local_radius_schedule.empty() &&
                local_radius_append_unrestricted_attempt) {
                prepared[task_offset].attempts = std::max(
                    prepared[task_offset].attempts,
                    static_cast<int>(local_radius_schedule.size()) + 1);
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
            std::shared_ptr<std::atomic<bool>> local_cancel =
                parallel_rrt_early_stop ? std::make_shared<std::atomic<bool>>(false)
                                        : batch_context.native_cancel_flag();
            std::atomic<int> early_successes{0};
            batch_context.executor().parallel_for(0,
                                                  static_cast<int>(jobs.size()),
                                                  [&](int job_index) {
                if (local_cancel && local_cancel->load(std::memory_order_relaxed)) {
                    return;
                }
                const PreparedJob& job = jobs[static_cast<std::size_t>(job_index)];
                auto path = run_task_attempt(tasks[job.task_offset],
                                             job.attempt,
                                             0,
                                             local_cancel);
                if (parallel_rrt_early_stop &&
                    rrt_path_good_enough_for_task(tasks[job.task_offset], path)) {
                    const int successes =
                        early_successes.fetch_add(1, std::memory_order_relaxed) + 1;
                    if (successes >= parallel_rrt_early_stop_min_successes && local_cancel) {
                        local_cancel->store(true, std::memory_order_relaxed);
                    }
                }
                attempt_paths[job.task_offset][static_cast<std::size_t>(job.attempt)] =
                    std::move(path);
            });
            if (parallel_rrt_early_stop) {
                batch_context.diagnostics().add_counter(
                    "query_bridge.parallel_rrt_early_stop_successes",
                    static_cast<double>(early_successes.load(std::memory_order_relaxed)));
                batch_context.diagnostics().add_counter(
                    local_cancel && local_cancel->load(std::memory_order_relaxed)
                        ? "query_bridge.parallel_rrt_early_stop_triggered"
                        : "query_bridge.parallel_rrt_early_stop_not_triggered");
            }
        } else {
            for (const PreparedJob& job : jobs) {
                attempt_paths[job.task_offset][static_cast<std::size_t>(job.attempt)] =
                    run_task_attempt(tasks[job.task_offset], job.attempt, 0);
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
            select_attempt_paths(task, attempt_paths[task_offset], best_length);
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
            const int fast_direct_added = try_fast_direct_segment_after_rrt(task);
            if (fast_direct_added > 0) {
                added_by_query[task.index] += fast_direct_added;
                batch_context.diagnostics().set_value(
                    task_key(task.index, "fast_direct_segment_after_rrt"),
                    1.0);
                batch_context.diagnostics().set_value(
                    task_key(task.index, "added"),
                    static_cast<double>(added_by_query[task.index]));
                batch_context.diagnostics().set_value(
                    task_key(task.index, "total_ms"),
                    elapsed_ms_since(batch_t0) - prepared[task_offset].task_start_ms);
                continue;
            }
            const auto pave_t0 = Clock::now();
            bridge_query_with_waypoint_fallbacks(task, added_by_query[task.index]);
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
	        if (task.hipac_online_satisfied ||
                task.direct_start_goal_satisfied ||
                current_query_good(task, true)) {
	            batch_context.diagnostics().add_counter("query_bridge.batch_tasks_skipped");
	            batch_context.diagnostics().record_timing("query_bridge.batch_probe_ms_total",
	                                                      elapsed_ms_since(probe_t0));
	            batch_context.diagnostics().set_value(task_key(task.index, "skipped"),
	                                                  1.0);
	            if (task.hipac_online_satisfied) {
	                batch_context.diagnostics().set_value(task_key(task.index, "skipped_by_hipac_online"),
	                                                      1.0);
	            }
                if (task.direct_start_goal_satisfied) {
                    batch_context.diagnostics().set_value(
                        task_key(task.index, "skipped_by_direct_start_goal_segment"),
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
        int effective_attempts =
            task.waypoint_path_from_partition_query && !task.waypoint_path.empty()
                ? 0
                : attempts;
        if (effective_attempts > 0 &&
            !local_radius_schedule.empty() &&
            local_radius_append_unrestricted_attempt) {
            effective_attempts = std::max(
                effective_attempts,
                static_cast<int>(local_radius_schedule.size()) + 1);
        }
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
            std::shared_ptr<std::atomic<bool>> local_cancel =
                parallel_rrt_early_stop ? std::make_shared<std::atomic<bool>>(false)
                                        : batch_context.native_cancel_flag();
            std::atomic<int> early_successes{0};
            batch_context.executor().parallel_for(0, effective_attempts, [&](int attempt) {
                if (local_cancel && local_cancel->load(std::memory_order_relaxed)) {
                    return;
                }
                auto path = run_task_attempt(task, attempt, 0, local_cancel);
                if (parallel_rrt_early_stop &&
                    rrt_path_good_enough_for_task(task, path)) {
                    const int successes =
                        early_successes.fetch_add(1, std::memory_order_relaxed) + 1;
                    if (successes >= parallel_rrt_early_stop_min_successes && local_cancel) {
                        local_cancel->store(true, std::memory_order_relaxed);
                    }
                }
                attempt_paths[static_cast<std::size_t>(attempt)] = std::move(path);
            });
            if (parallel_rrt_early_stop) {
                batch_context.diagnostics().add_counter(
                    "query_bridge.parallel_rrt_early_stop_successes",
                    static_cast<double>(early_successes.load(std::memory_order_relaxed)));
                batch_context.diagnostics().add_counter(
                    local_cancel && local_cancel->load(std::memory_order_relaxed)
                        ? "query_bridge.parallel_rrt_early_stop_triggered"
                        : "query_bridge.parallel_rrt_early_stop_not_triggered");
            }
        } else {
            for (int attempt = 0; attempt < effective_attempts; ++attempt) {
                attempt_paths[static_cast<std::size_t>(attempt)] = run_task_attempt(task, attempt, 0);
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
        select_attempt_paths(task, attempt_paths, best_length);
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
                auto retry_path = run_task_attempt(task, attempts + retry, 0);
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
            (no_path_retry_attempts > 0 || no_path_retry_budget_stages > 0)) {
            int retry_attempt_offset = attempts;
            int retry_attempts_total = 0;
            int retry_successes_total = 0;
            double retry_ms_total = 0.0;
            auto run_no_path_retry_stage = [&](int stage_index,
                                               int stage_attempts,
                                               int stage_fixed_iters,
                                               bool adaptive_stage) {
                const int effective_stage_attempts = std::max(0, stage_attempts);
                if (effective_stage_attempts == 0 || !task.waypoint_path.empty()) {
                    return;
                }
                const auto retry_t0 = Clock::now();
                int retry_successes = 0;
                int retry_attempts_run = 0;
                for (int retry = 0; retry < effective_stage_attempts; ++retry) {
                    auto retry_path =
                        run_task_attempt(task, retry_attempt_offset + retry, stage_fixed_iters);
                    retry_attempts_run += 1;
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
                retry_attempt_offset += effective_stage_attempts;
                retry_attempts_total += retry_attempts_run;
                retry_successes_total += retry_successes;
                const double retry_ms = elapsed_ms_since(retry_t0);
                retry_ms_total += retry_ms;
                const std::string key_prefix =
                    stage_index == 0
                        ? task_key(task.index, "no_path_retry_")
                        : task_key(task.index,
                                   "no_path_retry_stage." + std::to_string(stage_index) + ".");
                batch_context.diagnostics().set_value(key_prefix + "attempts",
                                                      static_cast<double>(effective_stage_attempts));
                batch_context.diagnostics().set_value(key_prefix + "attempts_run",
                                                      static_cast<double>(retry_attempts_run));
                batch_context.diagnostics().set_value(key_prefix + "successes",
                                                      static_cast<double>(retry_successes));
                batch_context.diagnostics().set_value(key_prefix + "fixed_iters",
                                                      static_cast<double>(stage_fixed_iters));
                batch_context.diagnostics().set_value(key_prefix + "ms", retry_ms);
                if (adaptive_stage) {
                    batch_context.diagnostics().add_counter(
                        "query_bridge.batch_no_path_retry_adaptive_attempts",
                        static_cast<double>(retry_attempts_run));
                    batch_context.diagnostics().add_counter(
                        "query_bridge.batch_no_path_retry_adaptive_successes",
                        static_cast<double>(retry_successes));
                    batch_context.diagnostics().record_timing(
                        "query_bridge.batch_no_path_retry_adaptive_ms_total",
                        retry_ms);
                }
            };
            run_no_path_retry_stage(0, no_path_retry_attempts, 0, false);
            for (std::size_t stage = 0;
                 task.waypoint_path.empty() && stage < no_path_retry_budget_stages;
                 ++stage) {
                run_no_path_retry_stage(static_cast<int>(stage) + 1,
                                        no_path_retry_budget_attempts[stage],
                                        no_path_retry_budget_iters[stage],
                                        true);
            }
            batch_context.diagnostics().record_timing(
                "query_bridge.batch_no_path_retry_ms_total",
                retry_ms_total);
            batch_context.diagnostics().add_counter(
                "query_bridge.batch_no_path_retry_attempts",
                static_cast<double>(retry_attempts_total));
            batch_context.diagnostics().add_counter(
                "query_bridge.batch_no_path_retry_successes",
                static_cast<double>(retry_successes_total));
            batch_context.diagnostics().set_value(
                task_key(task.index, "no_path_retry_attempts"),
                static_cast<double>(retry_attempts_total));
            batch_context.diagnostics().set_value(
                task_key(task.index, "no_path_retry_ms"),
                retry_ms_total);
            batch_context.diagnostics().set_value(
                task_key(task.index, "no_path_retry_successes"),
                static_cast<double>(retry_successes_total));
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
        const int fast_direct_added = try_fast_direct_segment_after_rrt(task);
        if (fast_direct_added > 0) {
            added_by_query[task.index] += fast_direct_added;
            batch_context.diagnostics().set_value(
                task_key(task.index, "fast_direct_segment_after_rrt"),
                1.0);
            batch_context.diagnostics().set_value(task_key(task.index, "added"),
                                                  static_cast<double>(added_by_query[task.index]));
            batch_context.diagnostics().set_value(task_key(task.index, "total_ms"),
                                                  elapsed_ms_since(task_t0));
            continue;
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
        bridge_query_with_waypoint_fallbacks(task, added_by_query[task.index]);
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


} // namespace rbf
