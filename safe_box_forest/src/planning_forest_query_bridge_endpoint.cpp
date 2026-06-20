#include <SBF/safe_box_forest.h>

#include <SBF/box_graph.h>
#include <SBF/connector.h>

#include "planning_forest_audit.h"
#include "planning_forest_diagnostics.h"
#include "planning_forest_qroot_helpers.h"
#include "planning_forest_query_utils.h"

#include <algorithm>
#include <chrono>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace rbf {

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
    auto finish_main_contact = [&](int value) {
        add_diag("main_contact_success");
        set_diag("local_adj_checks", static_cast<double>(local_adj_checks));
        invalidate_query_cache();
        return finish_endpoint_main(value);
    };
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
    auto make_seed_from_face = [&](int box_id,
                                   const Eigen::VectorXd& from,
                                   const Eigen::VectorXd& to) {
        std::vector<Interval> intervals;
        if (!(use_partition_endpoint_index &&
              adaptive_partition_->intervals_for_box(box_id, intervals))) {
            const BoxNode* box = find_box_by_id(boxes_, box_id);
            if (box == nullptr) {
                return from;
            }
            intervals = box->joint_intervals;
        }
        return boundary_seed_from_intervals(intervals,
                                            from,
                                            to,
                                            oracle_->planning_intervals(),
                                            corridor_config.face_epsilon);
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
        const int edge_id = add_segment_edge_partition_first(front_box_id,
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
                return finish_main_contact(std::max(1, added_total));
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
                return finish_main_contact(std::max(1, added_total));
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
                const auto lateral_seeds = lateral_offset_seeds_local(
                    seed,
                    direction,
                    oracle_->planning_intervals(),
                    corridor_config.lateral_rounds,
                    corridor_config.lateral_offset);
                for (const auto& lateral_seed : lateral_seeds) {
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
                        return finish_main_contact(std::max(1, added_total));
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
                return finish_main_contact(added_total);
            }
            break;
        }
        if (append_edge_if_connected(current_box_id, target_owner) ||
            box_only_path_connected_partition_first(current_box_id, target_owner)) {
            return finish_main_contact(std::max(1, added_total));
        }
        if (try_residual_segment(current_box_id, target_owner, target.point)) {
            return finish_main_contact(added_total);
        }
    }

    merge_diagnostic_snapshot(last_build_.diagnostics, context.diagnostics().snapshot());
    set_diag("local_adj_checks", static_cast<double>(local_adj_checks));
    add_diag("fallback_to_e2e");
    invalidate_query_cache();
    return finish_endpoint_main(0);
}

} // namespace rbf
