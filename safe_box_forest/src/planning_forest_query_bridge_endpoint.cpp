#include <SBF/safe_box_forest.h>

#include <SBF/box_graph.h>
#include <SBF/connector.h>

#include "planning_forest_diagnostics.h"
#include "planning_forest_qroot_helpers.h"
#include "planning_forest_query_bridge_endpoint_index.h"
#include "planning_forest_query_bridge_endpoint_runtime.h"
#include "planning_forest_query_bridge_endpoint_targets.h"
#include "planning_forest_query_utils.h"

#include <algorithm>
#include <chrono>
#include <string>
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

    std::vector<int> main_island_storage =
        endpoint_main_largest_island_partition_first(pre_anchor_main_island_storage);
    if (main_island_storage.empty()) {
        if (partition_native_mode()) {
            add_diag("partition_missing_no_graph_fallback");
            add_diag("fallback_to_e2e");
            return 0;
        }
        add_diag("fallback_to_e2e");
        return 0;
    }
    const auto& main_island = main_island_storage;
    std::unordered_set<int> main_ids(main_island.begin(), main_island.end());
    if (main_ids.find(source_box_id) != main_ids.end()) {
        add_diag("already_main");
        return 0;
    }

    const bool graphless_endpoint_main = partition_native_mode();
    const bool use_partition_endpoint_index =
        graphless_endpoint_main &&
        adaptive_partition_query_enabled_ &&
        adaptive_partition_;
    EndpointMainIndexes indexes = build_endpoint_main_indexes(
        boxes_,
        main_island,
        static_cast<int>(point.size()),
        config_.query.adjacency_tolerance,
        !use_partition_endpoint_index);

    const EndpointMainTargetSet target_set =
        endpoint_main_targets_partition_first(adaptive_partition_.get(),
                                              use_partition_endpoint_index,
                                              boxes_,
                                              indexes.box_index_by_id,
                                              point,
                                              main_island,
                                              corridor_config.target_k);
    if (target_set.used_partition_index) {
        add_diag("partition_nearest_target_queries");
    }
    if (target_set.targets.empty()) {
        add_diag("missing_target");
        add_diag("fallback_to_e2e");
        return 0;
    }
    const auto& targets = target_set.targets;
    const auto& target_box_ids = target_set.target_box_ids;
    const int target_limit = target_set.target_limit;
    EndpointMainRuntime endpoint_runtime{
        boxes_,
        adjacency_,
        adaptive_partition_.get(),
        indexes,
        main_ids,
        boxes_before_endpoint_main,
        config_.query.adjacency_tolerance,
        graphless_endpoint_main,
        use_partition_endpoint_index};

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

    auto attempt_seed = [&](const Eigen::VectorXd& seed,
                            int parent_box_id,
                            const std::vector<int>& local_candidates,
                            const std::vector<int>& target_box_ids,
                            int depth) {
        int reached_box_id = -1;
        bool reached_main = false;
        const int existing_cover = endpoint_runtime.first_existing_cover(seed);
        if (existing_cover >= 0 &&
            endpoint_runtime.append_edge_if_connected(parent_box_id,
                                                      existing_cover,
                                                      local_adj_checks)) {
            reached_box_id = existing_cover;
            reached_main = main_ids.find(existing_cover) != main_ids.end();
            return std::pair<int, bool>{reached_box_id, reached_main};
        }
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
            return std::pair<int, bool>{-1, false};
        }
        if (!allow_dynamic_commit(*oracle_, result, config_.connector.pave.commit_policy)) {
            if (is_max_depth_attempt) {
                max_depth_ffb_failed = true;
            }
            return std::pair<int, bool>{-1, false};
        }
        BoxNode candidate;
        candidate.joint_intervals = result.intervals;
        candidate.seed_config = seed;
        candidate.tree_id = result.node;
        candidate.parent_box_id = parent_box_id;
        candidate.safety_status = result.validation_detail.safety_status;
        candidate.strict_audit_required = result.validation_detail.strict_audit_required;
        candidate.compute_volume();
        BoxNode* parent_box = endpoint_runtime.box_by_id(parent_box_id);
        if (!endpoint_runtime.parent_adjacent_to_candidate(parent_box_id,
                                                           candidate,
                                                           local_adj_checks)) {
            if (is_max_depth_attempt) {
                max_depth_ffb_failed = true;
            }
            return std::pair<int, bool>{-1, false};
        }
        candidate.root_id =
            parent_box != nullptr && parent_box->root_id >= 0 ? parent_box->root_id : parent_box_id;
        candidate.id = next_id++;
        const int new_id = candidate.id;
        if (indexes.node_owner.find(candidate.tree_id) == indexes.node_owner.end()) {
            oracle_->reserve_node(candidate.tree_id, new_id);
            indexes.node_owner[candidate.tree_id] = new_id;
        }
        boxes_.push_back(candidate);
        raw_boxes_.push_back(candidate);
        const std::size_t new_index = boxes_.size() - 1;
        endpoint_runtime.add_box_to_indexes(boxes_.back(), new_index);
        if (!graphless_endpoint_main) {
            adjacency_[new_id] = {};
            append_local_edge(adjacency_, parent_box_id, new_id);
        }
        boxes_added += 1;
        added_total += 1;
        add_diag("boxes_added");

        for (int candidate_id : local_candidates) {
            endpoint_runtime.append_edge_if_connected(new_id,
                                                      candidate_id,
                                                      local_adj_checks);
        }
        for (int target_id : target_box_ids) {
            if (endpoint_runtime.append_edge_if_connected(new_id,
                                                          target_id,
                                                          local_adj_checks)) {
                reached_main = true;
            }
        }
        reached_box_id = new_id;
        return std::pair<int, bool>{reached_box_id, reached_main};
    };
    auto try_residual_segment = [&](int front_box_id, int target_box_id, const Eigen::VectorXd& target_point) {
        if (!try_add_endpoint_main_residual_segment_edge(front_box_id,
                                                         target_box_id,
                                                         target_point,
                                                         max_depth_ffb_failed,
                                                         corridor_config.residual_segment_max_length)) {
            return false;
        }
        added_total += 1;
        return true;
    };

    const int endpoint_main_ffb_depth =
        std::min(std::max(1, config_.database.max_tree_depth),
                 std::max(1, final_ffb_depth));

    for (int target_index = 0; target_index < target_limit; ++target_index) {
        if (ffb_calls >= std::max(1, corridor_config.max_ffb_calls) ||
            boxes_added >= std::max(1, corridor_config.max_boxes)) {
            break;
        }
        add_diag("targets_tested");
        const EndpointMainTargetCandidate& target = targets[static_cast<std::size_t>(target_index)];
        const EndpointMainSamplePlan sample_plan =
            endpoint_main_sample_plan(point,
                                      target,
                                      corridor_config.coarse_step,
                                      corridor_config.fine_step,
                                      [&](const Eigen::VectorXd& q) {
                                          return endpoint_runtime.main_owner(q);
                                      });
        const std::vector<Eigen::VectorXd>& samples = sample_plan.samples;
        if (samples.size() < 2 || sample_plan.target_sample_index < 1) {
            continue;
        }
        const int target_sample_index = sample_plan.target_sample_index;
        const int target_owner = sample_plan.target_owner;
        std::vector<int> chain_ids{source_box_id};
        int current_box_id = source_box_id;
        int current_sample_index =
            endpoint_runtime.furthest_sample(current_box_id, samples, 0, target_sample_index);
        int stall_count = 0;
        while (current_sample_index < target_sample_index &&
               ffb_calls < std::max(1, corridor_config.max_ffb_calls) &&
               boxes_added < std::max(1, corridor_config.max_boxes)) {
            if (endpoint_runtime.append_edge_if_connected(current_box_id,
                                                          target_owner,
                                                          local_adj_checks)) {
                return finish_main_contact(std::max(1, added_total));
            }
            const Eigen::VectorXd current_sample =
                samples[static_cast<std::size_t>(current_sample_index)];
            Eigen::VectorXd from = current_sample;
            if (!endpoint_runtime.contains_point(current_box_id, current_sample) &&
                !endpoint_runtime.closest_point_for_box(current_box_id, current_sample, from)) {
                break;
            }
            const Eigen::VectorXd seed = endpoint_runtime.make_seed_from_face(
                current_box_id,
                from,
                samples[static_cast<std::size_t>(target_sample_index)],
                oracle_->planning_intervals(),
                corridor_config.face_epsilon);
            std::vector<int> local_candidates = chain_ids;
            local_candidates.push_back(target_owner);
            const auto [new_box_id, reached_main] = attempt_seed(
                seed,
                current_box_id,
                local_candidates,
                target_box_ids,
                endpoint_main_ffb_depth);
            if (reached_main) {
                return finish_main_contact(std::max(1, added_total));
            }
            if (new_box_id >= 0) {
                const int next_sample_index =
                    endpoint_runtime.furthest_sample(new_box_id,
                                                     samples,
                                                     current_sample_index,
                                                     target_sample_index);
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
                        endpoint_main_ffb_depth);
                    if (lat_main) {
                        return finish_main_contact(std::max(1, added_total));
                    }
                    if (lat_box_id >= 0) {
                        const int next_sample_index =
                            endpoint_runtime.furthest_sample(lat_box_id,
                                                             samples,
                                                             current_sample_index,
                                                             target_sample_index);
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
        if (endpoint_runtime.append_edge_if_connected(current_box_id,
                                                      target_owner,
                                                      local_adj_checks) ||
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
