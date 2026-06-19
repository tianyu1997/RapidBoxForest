#include <SBF/safe_box_forest.h>

#include <SBF/box_graph.h>
#include <SBF/connector.h>

#include "planning_forest_audit.h"
#include "planning_forest_query_bridge_corridor_diagnostics.h"
#include "planning_forest_query_bridge_corridor_graph.h"
#include "planning_forest_query_bridge_corridor_options.h"
#include "planning_forest_query_bridge_corridor_repair.h"
#include "planning_forest_query_bridge_corridor_tasks.h"
#include "planning_forest_query_bridge_path_utils.h"
#include "planning_forest_diagnostics.h"
#include "planning_forest_qroot_helpers.h"
#include "planning_forest_query_utils.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

namespace rbf {

int RBFPlanningForest::try_query_bridge_direct_ffb_corridor(
    const Eigen::Ref<const Eigen::VectorXd>& start,
    const Eigen::Ref<const Eigen::VectorXd>& goal,
    const std::vector<Eigen::VectorXd>& corridor_path,
    const RRTConnectConfig& bridge_rrt,
    CollisionChecker& checker,
    StageContext& context,
    int query_index,
    int bridge_edge_query_index,
    int query_bridge_ffb_depth,
    double audited_bridge_length,
    bool allow_residual_segments,
    int& next_id) {
    using Clock = std::chrono::steady_clock;
    const auto t0 = Clock::now();
    const bool graphless_direct_corridor = partition_native_mode();
    const std::size_t boxes_before_direct_corridor = boxes_.size();
    const double direct_corridor_audit_step = config_.query.audit_segment_step > 0.0
        ? config_.query.audit_segment_step
        : 0.01;
    const QueryBridgeDirectCorridorRuntimeOptions direct_corridor_options =
        query_bridge_direct_corridor_runtime_options(config_,
                                                     query_index,
                                                     direct_corridor_audit_step);
    const double audit_step = direct_corridor_options.audit_step;
    const double sample_step = direct_corridor_options.sample_step;
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
        use_partition_cover_index;
    const bool immediate_partition_append =
        use_partition_cover_index;
    const int partition_append_batch_size =
        std::max(1, direct_corridor_options.partition_append_batch_size);
    context.diagnostics().set_value(
        "query_bridge.direct_corridor_partition_append_batch_size",
        static_cast<double>(partition_append_batch_size));
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

    QueryBridgeLocalDsu dsu(boxes_.size());
    QueryBridgeDirectCorridorRuntimeStats runtime_stats;
    constexpr bool local_assimilate_sample_scan = true;
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
        const int appended = adaptive_partition_->append_boxes(
            boxes_,
            direct_partition_append_base,
            config_.query.adjacency_tolerance);
        context.diagnostics().add_counter(
            appended > 0
                ? "query_bridge.direct_corridor_batched_partition_appends"
                : "query_bridge.direct_corridor_batched_partition_append_rejects");
        direct_partition_append_base = boxes_.size();
        return appended;
    };
    auto transition_connected = [&](int transition) {
        if (transition < 0 || transition + 1 >= static_cast<int>(sample_layers.size())) {
            return false;
        }
        const auto& lhs_layer = sample_layers[static_cast<std::size_t>(transition)];
        const auto& rhs_layer = sample_layers[static_cast<std::size_t>(transition + 1)];
        if (lhs_layer.empty() || rhs_layer.empty()) {
            return false;
        }
        for (int lhs : lhs_layer) {
            const int root = dsu.find(lhs);
            for (int rhs : rhs_layer) {
                if (root == dsu.find(rhs)) {
                    return true;
                }
            }
        }
        return false;
    };
    auto bad_transitions = [&]() {
        std::vector<int> bad;
        for (std::size_t sample_index = 0; sample_index + 1 < sample_layers.size(); ++sample_index) {
            if (!transition_connected(static_cast<int>(sample_index))) {
                bad.push_back(static_cast<int>(sample_index));
            }
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
        if (node != kInvalidOracleNodeId) {
            const auto node_it = node_to_box_index.find(node);
            if (node_it != node_to_box_index.end()) {
                return node_it->second;
            }
            return -1;
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
                return static_cast<int>(box_index);
            }
        }
        return -1;
    };
    std::vector<int> repair_indices;
    auto assimilate_box = [&](int box_index, int transition_hint) {
        const auto assimilate_t0 = Clock::now();
        const int box_id = boxes_[static_cast<std::size_t>(box_index)].id;
        if (!graphless_direct_corridor) {
            adjacency_[box_id];
        }
        int first_covered_sample = static_cast<int>(samples.size());
        int last_covered_sample = -1;
        int covered_sample_count = 0;
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
            runtime_stats.assimilate_local_sample_tests += 1;
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
                runtime_stats.assimilate_local_hits += 1;
            } else {
                used_full_sample_scan = true;
                runtime_stats.assimilate_full_scan_fallbacks += 1;
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
            runtime_stats.assimilate_coverage_boxes += 1;
            runtime_stats.assimilate_coverage_span_sum += static_cast<double>(span);
            runtime_stats.assimilate_coverage_span_max = std::max(runtime_stats.assimilate_coverage_span_max, span);
            context.diagnostics().add_counter(
                "query_bridge.direct_corridor_assimilate_covered_samples",
                static_cast<double>(covered_sample_count));
        }
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
    auto commit_result = [&](FindFreeBoxResult result,
                             const Eigen::VectorXd& seed,
                             int transition_hint) -> int {
        if (!result.found ||
            !intervals_contain_point_local(result.intervals,
                                           seed,
                                           config_.query.adjacency_tolerance)) {
            return -1;
        }
        const int duplicate_index = find_duplicate_box_index(result.node,
                                                             result.intervals);
        if (duplicate_index >= 0) {
            assimilate_box(duplicate_index, transition_hint);
            return duplicate_index;
        }
        if (!allow_dynamic_commit(*oracle_, result, config_.connector.pave.commit_policy)) {
            return -1;
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
        assimilate_box(box_index, transition_hint);
        return box_index;
    };
    auto current_boxes_cover_point = [&](const Eigen::VectorXd& point) {
        if (use_partition_cover_index) {
            const bool partition_covered =
                !adaptive_partition_->covering_box_ids(point,
                                                       config_.query.adjacency_tolerance).empty();
            if (partition_covered) {
                return true;
            }
        }
        if (use_partition_cover_index) {
            for (int box_index : corridor_new_box_indices) {
                if (box_index >= 0 &&
                    box_index < static_cast<int>(boxes_.size()) &&
                    intervals_contain_point_local(boxes_[static_cast<std::size_t>(box_index)].joint_intervals,
                                                   point,
                                                   config_.query.adjacency_tolerance)) {
                    return true;
                }
            }
            return false;
        }
        return direct_box_index.covering_box(boxes_,
                                             point,
                                             config_.query.adjacency_tolerance) >= 0;
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
    direct_options.record_diagnostics = false;
    const std::vector<Interval> direct_planning_domain =
        oracle_ ? oracle_->planning_intervals() : std::vector<Interval>{};
    int direct_calls = 0;
    int direct_added = 0;
    double direct_ffb_ms = 0.0;
    double repair_ffb_ms = 0.0;
    double adaptive_repair_ffb_ms = 0.0;
    double residual_segment_audit_ms = 0.0;
    const QueryBridgeDirectFfbTaskPlan direct_task_plan =
        query_bridge_prepare_direct_ffb_task_plan(
            context,
            samples,
            covered,
            std::max(direct_options.start_depth, direct_options.skip_to_depth));
    const std::vector<QueryBridgeDirectFfbTask>& direct_tasks = direct_task_plan.tasks;
    const QueryBridgeFfbTaskExecutionStats direct_task_stats =
        query_bridge_run_direct_ffb_tasks(
            context,
            direct_tasks,
            covered,
            [&](const QueryBridgeDirectFfbTask& task) {
                return find_free_box_in_domain(task.seed,
                                               direct_planning_domain,
                                               context,
                                               direct_options);
            },
            [&](FindFreeBoxResult&& result, const QueryBridgeDirectFfbTask& task) {
                const std::size_t before_boxes = boxes_.size();
                const int box_index = commit_result(std::move(result),
                                                    task.seed,
                                                    task.transition_hint);
                return QueryBridgeFfbTaskCommitResult{box_index,
                                                      boxes_.size() > before_boxes};
            });
    direct_calls = direct_task_stats.calls;
    direct_added = direct_task_stats.added;
    direct_ffb_ms = direct_task_stats.ffb_ms;
    const QueryBridgeRepairSubdivisionOptions repair_subdivision_options =
        query_bridge_repair_subdivision_options(query_index);
    const int subdivisions = repair_subdivision_options.subdivisions;
    const std::vector<double>& fractions = repair_subdivision_options.fractions;
    int repair_calls = 0;
    int repair_added = 0;
    const auto initial_bad = bad_transitions();
    if (subdivisions > 1) {
        const QueryBridgeSubdivisionRepairStats repair_stats =
            query_bridge_run_subdivision_repair_pass(
                context,
                samples,
                initial_bad,
                fractions,
                transition_connected,
                current_boxes_cover_point,
                [&](const Eigen::VectorXd& seed, int) {
                    return find_free_box_in_domain(seed,
                                                   direct_planning_domain,
                                                   context,
                                                   direct_options);
                },
                [&](FindFreeBoxResult&& result, const Eigen::VectorXd& seed, int transition) {
                    const std::size_t before_boxes = boxes_.size();
                    const int box_index = commit_result(std::move(result), seed, transition);
                    return QueryBridgeFfbTaskCommitResult{box_index,
                                                          boxes_.size() > before_boxes};
                });
        repair_calls = repair_stats.calls;
        repair_added = repair_stats.added;
        repair_ffb_ms = repair_stats.ffb_ms;
        repair_indices.insert(repair_indices.end(),
                              repair_stats.committed_indices.begin(),
                              repair_stats.committed_indices.end());
    }
    std::vector<int> final_bad = bad_transitions();
    auto bad_transition_fraction = [&](const std::vector<int>& transitions) {
        return query_bridge_transition_fraction(samples,
                                                transitions,
                                                audited_bridge_length,
                                                query_bridge_waypoint_length(samples));
    };
    const QueryBridgeAdaptiveRepairOptions adaptive_repair_options =
        query_bridge_adaptive_repair_options(query_index,
                                             subdivisions,
                                             audit_step,
                                             sample_step);
    int adaptive_repair_calls = 0;
    int adaptive_repair_added = 0;
    int adaptive_repair_max_subdivisions_used = subdivisions;
    const QueryBridgeAdaptiveRepairStats adaptive_stats =
        query_bridge_run_adaptive_repair_pass(
            context,
            samples,
            final_bad,
            subdivisions,
            adaptive_repair_options,
            transition_connected,
            current_boxes_cover_point,
            bad_transitions,
            bad_transition_fraction,
            [&](const Eigen::VectorXd& seed, int) {
                return find_free_box_in_domain(seed,
                                               direct_planning_domain,
                                               context,
                                               direct_options);
            },
            [&](FindFreeBoxResult&& result, const Eigen::VectorXd& seed, int transition) {
                const std::size_t before_boxes = boxes_.size();
                const int box_index = commit_result(std::move(result), seed, transition);
                return QueryBridgeFfbTaskCommitResult{box_index,
                                                      boxes_.size() > before_boxes};
            });
    adaptive_repair_calls = adaptive_stats.calls;
    adaptive_repair_added = adaptive_stats.added;
    adaptive_repair_max_subdivisions_used = adaptive_stats.max_subdivisions_used;
    adaptive_repair_ffb_ms = adaptive_stats.ffb_ms;
    final_bad = adaptive_stats.final_bad;
    repair_indices.insert(repair_indices.end(),
                          adaptive_stats.committed_indices.begin(),
                          adaptive_stats.committed_indices.end());
    int local_segment_edges_added = 0;
    int local_segment_gap_samples_max = 0;
    if (!final_bad.empty() &&
        allow_residual_segments &&
        config_.connector.segment_edges_enabled &&
        config_.connector.rrt_segment_edges) {
        auto insert_residual_segment = [&](int lhs_index,
                                           int rhs_index,
                                           const Eigen::VectorXd& lhs_point,
                                           const Eigen::VectorXd& rhs_point,
                                           int sample_gap) {
            if (lhs_index < 0 || rhs_index < 0 ||
                lhs_index >= static_cast<int>(boxes_.size()) ||
                rhs_index >= static_cast<int>(boxes_.size())) {
                return false;
            }
            if (dsu.find(lhs_index) == dsu.find(rhs_index)) {
                return false;
            }
            std::vector<Eigen::VectorXd> gap_path{lhs_point, rhs_point};
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
            const int edge_id = add_segment_edge_partition_first(
                boxes_[static_cast<std::size_t>(lhs_index)].id,
                boxes_[static_cast<std::size_t>(rhs_index)].id,
                std::move(gap_path),
                SegmentEdgeType::QueryBridge,
                bridge_rrt.segment_resolution,
                SegmentEdgeValidation::CollisionChecked,
                true,
                bridge_edge_query_index);
            if (edge_id >= 0) {
                local_segment_edges_added += 1;
                local_segment_gap_samples_max =
                    std::max(local_segment_gap_samples_max, sample_gap);
                dsu.unite(lhs_index, rhs_index);
                return true;
            }
            return false;
        };
        query_bridge_run_residual_segment_gap_pass(
            context,
            samples,
            sample_layers,
            final_bad,
            insert_residual_segment);
    }
    const double direct_corridor_elapsed_ms =
        std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
    auto [source_box_id, target_box_id] =
        locate_query_bridge_boxes(start, goal, context);
    const bool local_corridor_connected =
        final_bad.empty() && endpoint_layers_connected();

    QueryBridgeDirectCorridorSummaryStats summary_stats;
    summary_stats.elapsed_ms = direct_corridor_elapsed_ms;
    summary_stats.direct_ffb_ms = direct_ffb_ms;
    summary_stats.repair_ffb_ms = repair_ffb_ms;
    summary_stats.adaptive_repair_ffb_ms = adaptive_repair_ffb_ms;
    summary_stats.residual_segment_audit_ms = residual_segment_audit_ms;
    summary_stats.assimilate_coverage_span_sum =
        runtime_stats.assimilate_coverage_span_sum;
    summary_stats.sample_count = samples.size();
    summary_stats.direct_calls = direct_calls;
    summary_stats.repair_calls = repair_calls;
    summary_stats.adaptive_repair_calls = adaptive_repair_calls;
    summary_stats.direct_added = direct_added;
    summary_stats.repair_added = repair_added;
    summary_stats.adaptive_repair_added = adaptive_repair_added;
    summary_stats.adaptive_repair_max_subdivisions_used =
        adaptive_repair_max_subdivisions_used;
    summary_stats.repair_subdivisions = subdivisions;
    summary_stats.initial_bad_count = static_cast<int>(initial_bad.size());
    summary_stats.final_bad_count = static_cast<int>(final_bad.size());
    summary_stats.local_segment_edges_added = local_segment_edges_added;
    summary_stats.local_segment_gap_samples_max = local_segment_gap_samples_max;
    summary_stats.assimilate_coverage_boxes =
        runtime_stats.assimilate_coverage_boxes;
    summary_stats.assimilate_coverage_span_max =
        runtime_stats.assimilate_coverage_span_max;
    summary_stats.local_corridor_connected = local_corridor_connected;
    query_bridge_record_direct_corridor_summary(context, query_index, summary_stats);

    if (final_bad.empty() &&
        source_box_id >= 0 &&
        target_box_id >= 0 &&
        (local_corridor_connected ||
         box_only_path_connected_partition_first(source_box_id, target_box_id))) {
        try_promote_query_bridge_direct_transition(source_box_id,
                                                   target_box_id,
                                                   sample_layers,
                                                   boxes_before_direct_corridor,
                                                   context,
                                                   query_index,
                                                   bridge_edge_query_index,
                                                   "box_connected",
                                                   adopt_certified_subchain_attempted);
        const int edge_added = add_verified_query_box_corridor_edge(
            source_box_id,
            target_box_id,
            corridor_path,
            bridge_rrt.segment_resolution,
            bridge_edge_query_index);
        if (edge_added > 0) {
            return finish_query_bridge_direct_corridor(
                boxes_before_direct_corridor,
                direct_added + repair_added + local_segment_edges_added + 1);
        }
        return finish_query_bridge_direct_corridor(
            boxes_before_direct_corridor,
            direct_added + repair_added + local_segment_edges_added);
    }
    if (!final_bad.empty() &&
        allow_residual_segments &&
        local_segment_edges_added > 0 &&
        source_box_id >= 0 &&
        target_box_id >= 0) {
        refresh_query_bridge_direct_corridor_partition(boxes_before_direct_corridor);
        const bool locally_overlay_connected =
            overlay_path_connected_partition_first(source_box_id, target_box_id);
        context.diagnostics().set_value(
            "query_bridge.direct_corridor_local_residual_overlay_connected",
            locally_overlay_connected ? 1.0 : 0.0);
        query_bridge_set_task_value(context,
                                    query_index,
                                    "direct_corridor_local_residual_overlay_connected",
                                    locally_overlay_connected ? 1.0 : 0.0);
        if (locally_overlay_connected) {
            const bool add_full_residual_overlay_when_connected =
                direct_corridor_options.full_residual_overlay_when_connected;
            int full_edge_id = -1;
            if (add_full_residual_overlay_when_connected) {
                full_edge_id =
                    try_add_query_direct_corridor_full_residual_edge(
                        source_box_id,
                        target_box_id,
                        corridor_path,
                        bridge_rrt,
                        checker,
                        context,
                        bridge_edge_query_index,
                        query_index,
                        true,
                        false);
            }
            try_promote_query_bridge_direct_transition(source_box_id,
                                                       target_box_id,
                                                       sample_layers,
                                                       boxes_before_direct_corridor,
                                                       context,
                                                       query_index,
                                                       bridge_edge_query_index,
                                                       "local_residual_overlay",
                                                       adopt_certified_subchain_attempted);
            invalidate_query_cache();
            return finish_query_bridge_direct_corridor(
                boxes_before_direct_corridor,
                direct_added + repair_added + local_segment_edges_added +
                    (full_edge_id >= 0 ? 1 : 0));
        }
        const int edge_id = try_add_query_direct_corridor_full_residual_edge(
            source_box_id,
            target_box_id,
            corridor_path,
            bridge_rrt,
            checker,
            context,
            bridge_edge_query_index,
            query_index,
            false,
            true);
        if (edge_id == -2) {
            invalidate_query_cache();
            return finish_query_bridge_direct_corridor(
                boxes_before_direct_corridor,
                direct_added + repair_added + local_segment_edges_added);
        }
        try_promote_query_bridge_direct_transition(source_box_id,
                                                   target_box_id,
                                                   sample_layers,
                                                   boxes_before_direct_corridor,
                                                   context,
                                                   query_index,
                                                   bridge_edge_query_index,
                                                   "full_residual",
                                                   adopt_certified_subchain_attempted);
        invalidate_query_cache();
        return finish_query_bridge_direct_corridor(
            boxes_before_direct_corridor,
            direct_added + repair_added + local_segment_edges_added +
                (edge_id >= 0 ? 1 : 0));
    }
    return finish_query_bridge_direct_corridor(boxes_before_direct_corridor, 0);

}

}  // namespace rbf
