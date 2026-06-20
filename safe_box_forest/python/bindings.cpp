#include <SBF/sbf.h>

#include "binding_adaptive_types.h"
#include "binding_basic_types.h"
#include "binding_planner_core_types.h"
#include "binding_planning_option_types.h"
#include "binding_utils.h"
#include "ompl_binding_utils.h"

#include <cstdio>
#include <cstdlib>

#include <rbf/lect_database/read_snapshot.h>
#include <link_interval_envelope/batch.h>
#include <sbf/envelope/ifk_aa_source.h>

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <limits>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace py = pybind11;

using namespace rbf::python_binding;

PYBIND11_MODULE(_sbf_cpp, module) {
    module.doc() = "Standalone RBFPlanningForest bindings";
    module.attr("__version__") = "0.1.0";

    register_basic_types(module);
    register_planner_core_types(module);
    register_adaptive_types(module);

    register_planning_option_types(module);

    py::class_<rbf::RBFPlanningForest>(module, "RBFPlanningForest")
        .def(py::init<rbf::Robot, rbf::RBFPlanningConfig>(), py::arg("robot"), py::arg("config") = rbf::RBFPlanningConfig{})
        .def("build",
             [](rbf::RBFPlanningForest& forest,
                 const std::vector<double>& start,
                 const std::vector<double>& goal,
                const std::vector<rbf::Obstacle>& obstacles) {
                  return forest.build(eigen_vector_from_list(start), eigen_vector_from_list(goal), obstacles);
             },
             py::arg("start"), py::arg("goal"), py::arg("obstacles") = std::vector<rbf::Obstacle>{})
        .def("build_coverage",
             [](rbf::RBFPlanningForest& forest,
                const std::vector<rbf::Obstacle>& obstacles,
                 const std::vector<std::vector<double>>& seeds) {
                  return forest.build_coverage(obstacles, eigen_vectors_from_lists(seeds));
             },
             py::arg("obstacles"), py::arg("seeds"))
        .def("build_leaf_sweep",
             [](rbf::RBFPlanningForest& forest,
                const std::vector<rbf::Obstacle>& obstacles,
                int start_depth,
                int max_depth,
                const rbf::LeafSweepConfig& config) {
                 return forest.build_leaf_sweep(obstacles, start_depth, max_depth, config);
             },
             py::arg("obstacles"),
             py::arg("start_depth"),
             py::arg("max_depth"),
             py::arg("config") = rbf::LeafSweepConfig{})
        .def("build_leaf_sweep_refined",
             [](rbf::RBFPlanningForest& forest,
                const std::vector<rbf::Obstacle>& obstacles,
                const rbf::LeafSweepRefineConfig& config,
                const std::vector<std::vector<double>>& priority_points,
                const std::vector<std::vector<double>>& offline_anchor_points) {
                 return forest.build_leaf_sweep_refined(
                     obstacles,
                     config,
                     eigen_vectors_from_lists(priority_points),
                     eigen_vectors_from_lists(offline_anchor_points));
             },
             py::arg("obstacles"),
             py::arg("config") = rbf::LeafSweepRefineConfig{},
             py::arg("priority_points") = std::vector<std::vector<double>>{},
             py::arg("offline_anchor_points") = std::vector<std::vector<double>>{})
        .def("build_adaptive_deep_leaf_sweep_cover",
             [](rbf::RBFPlanningForest& forest,
                const std::vector<rbf::Obstacle>& obstacles,
                const rbf::AdaptiveLeafSweepConfig& config) {
                 return forest.build_adaptive_deep_leaf_sweep_cover(obstacles, config);
             },
             py::arg("obstacles"),
             py::arg("config") = rbf::AdaptiveLeafSweepConfig{})
        .def("oracle_counters",
             [](const rbf::RBFPlanningForest& forest) {
                 const rbf::OracleCounters* counters = forest.oracle_counters();
                 if (!counters) {
                     return py::dict{};
                 }
                 return oracle_counters_to_python(*counters);
             })
                     .def("build_subtractive",
                             [](rbf::RBFPlanningForest& forest,
                                    const std::vector<rbf::SubtractiveObstacleGroup>& obstacle_groups,
                                    const std::vector<std::vector<double>>& seeds,
                                    const rbf::SubtractiveBuildOptions& options) {
                                     return forest.build_subtractive(obstacle_groups, eigen_vectors_from_lists(seeds), options);
                             },
                             py::arg("obstacle_groups"),
                             py::arg("seeds"),
                             py::arg("options") = rbf::SubtractiveBuildOptions{})
           .def("query",
               [](const rbf::RBFPlanningForest& forest,
                 const std::vector<double>& start,
                 const std::vector<double>& goal,
                 const rbf::RBFQueryRuntimeOptions& options) {
                  return forest.query(eigen_vector_from_list(start),
                                      eigen_vector_from_list(goal),
                                      options);
               },
               py::arg("start"), py::arg("goal"),
               py::arg("options") = rbf::RBFQueryRuntimeOptions{})
                                .def("refine_query_corridor",
                                                 [](rbf::RBFPlanningForest& forest,
                                                                const std::vector<double>& start,
                                                                const std::vector<double>& goal,
                                                                int max_boxes_to_add,
                                                                const std::string& mode,
                                                                double long_path_ratio,
                                                                double long_path_min_delta) {
                                                                        rbf::CorridorRefineMode refine_mode = rbf::CorridorRefineMode::SegmentBridge;
                                                                        if (mode == "box_only_long_path") {
                                                                            refine_mode = rbf::CorridorRefineMode::BoxOnlyLongPath;
                                                                        } else if (mode != "segment_bridge") {
                                                                            throw std::invalid_argument("unsupported corridor refine mode: " + mode);
                                                                        }
                                                                        return forest.refine_query_corridor(eigen_vector_from_list(start),
                                                                                                                                             eigen_vector_from_list(goal),
                                                                                                                                             max_boxes_to_add,
                                                                                                                                             refine_mode,
                                                                                                                                             long_path_ratio,
                                                                                                                                             long_path_min_delta);
                                                 },
                                                 py::arg("start"),
                                                 py::arg("goal"),
                                                 py::arg("max_boxes_to_add"),
                                                 py::arg("mode") = "segment_bridge",
                                                 py::arg("long_path_ratio") = std::numeric_limits<double>::infinity(),
                                                 py::arg("long_path_min_delta") = std::numeric_limits<double>::infinity())
                .def("bridge_query",
                         [](rbf::RBFPlanningForest& forest,
                                const std::vector<double>& start,
                                const std::vector<double>& goal) {
                                    return forest.bridge_query(eigen_vector_from_list(start), eigen_vector_from_list(goal));
                         },
                         py::arg("start"), py::arg("goal"))
                .def("bridge_query_known_needed",
                         [](rbf::RBFPlanningForest& forest,
                                const std::vector<double>& start,
                                const std::vector<double>& goal) {
                                    return forest.bridge_query_known_needed(eigen_vector_from_list(start), eigen_vector_from_list(goal));
                         },
                         py::arg("start"), py::arg("goal"))
                .def("anchor_query_endpoint_box",
                        [](rbf::RBFPlanningForest& forest,
                           const std::vector<double>& point) {
                            return forest.anchor_query_endpoint(eigen_vector_from_list(point));
                        },
                        py::arg("point"))
                .def("connect_query_endpoint_to_main_island",
                        [](rbf::RBFPlanningForest& forest,
                           const std::vector<double>& point,
                           double max_segment_length) {
                            return forest.connect_query_endpoint_to_main_island(
                                eigen_vector_from_list(point),
                                max_segment_length);
                        },
                        py::arg("point"),
                        py::arg("max_segment_length") = 0.0)
                .def("connect_query_endpoint_to_main_box_corridor",
                        [](rbf::RBFPlanningForest& forest,
                           const std::vector<double>& point,
                           const rbf::EndpointMainBoxCorridorConfig& config) {
                            return forest.connect_query_endpoint_to_main_box_corridor(
                                eigen_vector_from_list(point),
                                config);
                        },
                        py::arg("point"),
                        py::arg("config") = rbf::EndpointMainBoxCorridorConfig{})
                .def("add_offline_shortcut_edges",
                        [](rbf::RBFPlanningForest& forest,
                           int max_edges,
                           int candidate_limit,
                           double min_gain_ratio,
                           double max_segment_length,
                           bool allow_segment_fallback) {
                                    return forest.add_offline_shortcut_edges(max_edges,
                                                                             candidate_limit,
                                                                             min_gain_ratio,
                                                                             max_segment_length,
                                                                             allow_segment_fallback);
                         },
                         py::arg("max_edges"),
                         py::arg("candidate_limit") = 48,
                         py::arg("min_gain_ratio") = 1.6,
                         py::arg("max_segment_length") = 3.0,
                         py::arg("allow_segment_fallback") = false)
                .def("bridge_queries",
                         [](rbf::RBFPlanningForest& forest,
                                const std::vector<std::vector<double>>& starts,
                                const std::vector<std::vector<double>>& goals,
                                const std::vector<int>& forced_query_indices,
                                const std::vector<int>& global_query_indices) {
                                    if (starts.size() != goals.size()) {
                                        throw std::invalid_argument("bridge_queries requires starts/goals with matching sizes");
                                    }
                                    std::vector<Eigen::VectorXd> eigen_starts;
                                    std::vector<Eigen::VectorXd> eigen_goals;
                                    eigen_starts.reserve(starts.size());
                                    eigen_goals.reserve(goals.size());
                                    for (std::size_t i = 0; i < starts.size(); ++i) {
                                        eigen_starts.push_back(eigen_vector_from_list(starts[i]));
                                        eigen_goals.push_back(eigen_vector_from_list(goals[i]));
                                    }
                                    rbf::QueryBridgeBatchOptions options;
                                    options.forced_query_indices = forced_query_indices;
                                    options.global_query_indices = global_query_indices;
                                    return forest.bridge_queries(eigen_starts, eigen_goals, options);
                         },
                         py::arg("starts"),
                         py::arg("goals"),
                         py::arg("forced_query_indices") = std::vector<int>{},
                         py::arg("global_query_indices") = std::vector<int>{})
                .def("debug_chain_pave",
                         [](rbf::RBFPlanningForest& forest,
                                const std::vector<double>& start,
                                const std::vector<double>& goal,
                                int max_chain,
                                int max_depth,
                                double edge_seed_step,
                                int max_gap_fill_steps,
                                bool fill_segment_gaps,
                                int gap_fill_max_retries,
                                double gap_fill_step_shrink,
                                double gap_fill_min_step,
                                int gap_fill_retry_depth_increment,
                                int gap_fill_max_depth,
                                double adjacency_tolerance,
                                double gap_fill_sample_step,
                                double gap_fill_time_budget_ms,
                                int gap_fill_max_ffb_calls) {
                                    rbf::ChainPaveConfig pave;
                                    pave.commit_policy = rbf::BoxCommitPolicy::CommitProvisionalAllowed;
                                    pave.max_chain = max_chain;
                                    pave.fill_gaps = fill_segment_gaps;
                                    pave.adjacency_tolerance = adjacency_tolerance;
                                    // Recursion depth bounds the bisection subdivisions (2^depth seeds).
                                    // Derive it from the requested seed budget while keeping a sane cap.
                                    pave.max_gap_fill_depth = std::max(1, std::min(20, max_gap_fill_steps));
                                    pave.gap_fill_min_step = gap_fill_min_step;
                                    pave.gap_fill_sample_step = gap_fill_sample_step;
                                    pave.gap_fill_time_budget_ms = gap_fill_time_budget_ms;
                                    pave.gap_fill_max_ffb_calls = gap_fill_max_ffb_calls;
                                    pave.find_free_box.max_depth = max_depth;
                                    pave.find_free_box.reject_seed_collision = false;
                                    (void)edge_seed_step;
                                    (void)gap_fill_max_retries;
                                    (void)gap_fill_step_shrink;
                                    (void)gap_fill_retry_depth_increment;
                                    (void)gap_fill_max_depth;
                                    auto res = forest.debug_chain_pave(eigen_vector_from_list(start),
                                                                       eigen_vector_from_list(goal),
                                                                       pave);
                                    py::dict result;
                                    result["added"] = res.added;
                                    result["bridge_found"] = res.bridge_found;
                                    result["audit_passed"] = res.audit_passed;
                                    result["start_box_id"] = res.start_box_id;
                                    result["goal_box_id"] = res.goal_box_id;
                                    result["fast_gap_fill_ffb_calls"] = res.fast_gap_fill_ffb_calls;
                                    result["fast_gap_fill_ms"] = res.fast_gap_fill_ms;
                                    result["boundary_ffb_calls"] = res.boundary_ffb_calls;
                                    result["boundary_commits"] = res.boundary_commits;
                                    result["boundary_reject_not_free"] = res.boundary_reject_not_free;
                                    result["boundary_reject_non_adjacent"] = res.boundary_reject_non_adjacent;
                                    result["boundary_fail_seed_collision"] = res.boundary_fail_seed_collision;
                                    result["boundary_fail_depth_cap"] = res.boundary_fail_depth_cap;
                                    result["boundary_fail_unknown_depth_cap"] = res.boundary_fail_unknown_depth_cap;
                                    result["boundary_fail_reserved_depth_cap"] = res.boundary_fail_reserved_depth_cap;
                                    result["boundary_fail_occupied"] = res.boundary_fail_occupied;
                                    result["boundary_fail_deadline"] = res.boundary_fail_deadline;
                                    result["boundary_fail_out_of_domain"] = res.boundary_fail_out_of_domain;
                                    result["boundary_fail_split"] = res.boundary_fail_split;
                                    result["boundary_failed_seed_memoized"] = res.boundary_failed_seed_memoized;
                                    result["boundary_skip_failed_seed"] = res.boundary_skip_failed_seed;
                                    result["boundary_stall"] = res.boundary_stall;
                                    result["boundary_target_hits"] = res.boundary_target_hits;
                                    py::list boundary_failures;
                                    for (const auto& failure : res.boundary_failures) {
                                        py::dict item;
                                        item["seed"] = failure.seed;
                                        item["intervals"] = interval_pairs_to_python(failure.intervals);
                                        item["validation_detail"] = oracle_validation_detail_to_python(failure.validation_detail);
                                        item["node"] = failure.node;
                                        item["depth"] = failure.depth;
                                        item["changed_dim"] = failure.changed_dim;
                                        item["fail_code"] = failure.fail_code;
                                        item["hit_unknown_depth_cap"] = failure.hit_unknown_depth_cap;
                                        item["hit_reserved_depth_cap"] = failure.hit_reserved_depth_cap;
                                        boundary_failures.append(std::move(item));
                                    }
                                    result["boundary_failures"] = std::move(boundary_failures);
                                    py::list waypoints;
                                    for (const auto& wp : res.waypoints) {
                                        waypoints.append(vector_to_list(wp));
                                    }
                                    result["waypoints"] = waypoints;
                                    py::list committed_boxes;
                                    for (const auto& box : res.committed_boxes) {
                                        committed_boxes.append(interval_pairs_to_python(box));
                                    }
                                    result["committed_boxes"] = committed_boxes;
                                    py::list all_boxes;
                                    for (const auto& box : res.all_boxes) {
                                        all_boxes.append(interval_pairs_to_python(box));
                                    }
                                    result["all_boxes"] = all_boxes;
                                    result["start_box"] = interval_pairs_to_python(res.start_box);
                                    result["goal_box"] = interval_pairs_to_python(res.goal_box);
                                    return result;
                         },
                         py::arg("start"),
                         py::arg("goal"),
                         py::arg("max_chain") = 4096,
                         py::arg("max_depth") = 120,
                         py::arg("edge_seed_step") = 1e-2,
                         py::arg("max_gap_fill_steps") = 64,
                         py::arg("fill_segment_gaps") = true,
                         py::arg("gap_fill_max_retries") = 6,
                         py::arg("gap_fill_step_shrink") = 0.5,
                         py::arg("gap_fill_min_step") = 1e-4,
                         py::arg("gap_fill_retry_depth_increment") = 24,
                         py::arg("gap_fill_max_depth") = 240,
                         py::arg("adjacency_tolerance") = 1e-9,
                         py::arg("gap_fill_sample_step") = 0.05,
                         py::arg("gap_fill_time_budget_ms") = 10.0,
                         py::arg("gap_fill_max_ffb_calls") = 32)
                .def("debug_chain_pave_waypoints",
                         [](rbf::RBFPlanningForest& forest,
                                const std::vector<std::vector<double>>& waypoints,
                                int max_chain,
                                int max_depth,
                                int max_gap_fill_steps,
                                bool fill_segment_gaps,
                                double gap_fill_min_step,
                                double adjacency_tolerance,
                                double gap_fill_sample_step,
                                double gap_fill_time_budget_ms,
                                int gap_fill_max_ffb_calls,
                                bool require_connected_chain,
                                bool commit_certified_only) {
                                    rbf::ChainPaveConfig pave;
                                    pave.commit_policy =
                                        commit_certified_only
                                            ? rbf::BoxCommitPolicy::CommitCertifiedOnly
                                            : rbf::BoxCommitPolicy::CommitProvisionalAllowed;
                                    pave.max_chain = max_chain;
                                    pave.fill_gaps = fill_segment_gaps;
                                    pave.max_gap_fill_depth = std::max(1, std::min(20, max_gap_fill_steps));
                                    pave.gap_fill_min_step = gap_fill_min_step;
                                    pave.adjacency_tolerance = adjacency_tolerance;
                                    pave.gap_fill_sample_step = gap_fill_sample_step;
                                    pave.gap_fill_time_budget_ms = gap_fill_time_budget_ms;
                                    pave.gap_fill_max_ffb_calls = gap_fill_max_ffb_calls;
                                    pave.require_connected_chain = require_connected_chain;
                                    pave.find_free_box.max_depth = max_depth;
                                    pave.find_free_box.reject_seed_collision = false;
                                    std::vector<Eigen::VectorXd> eigen_waypoints;
                                    eigen_waypoints.reserve(waypoints.size());
                                    for (const auto& waypoint : waypoints) {
                                        eigen_waypoints.push_back(eigen_vector_from_list(waypoint));
                                    }
                                    auto res = forest.debug_chain_pave_waypoints(eigen_waypoints, pave);
                                    py::dict result;
                                    result["added"] = res.added;
                                    result["bridge_found"] = res.bridge_found;
                                    result["audit_passed"] = res.audit_passed;
                                    result["start_box_id"] = res.start_box_id;
                                    result["goal_box_id"] = res.goal_box_id;
                                    result["fast_gap_fill_ffb_calls"] = res.fast_gap_fill_ffb_calls;
                                    result["fast_gap_fill_ms"] = res.fast_gap_fill_ms;
                                    result["boundary_ffb_calls"] = res.boundary_ffb_calls;
                                    result["boundary_commits"] = res.boundary_commits;
                                    result["boundary_reject_not_free"] = res.boundary_reject_not_free;
                                    result["boundary_reject_non_adjacent"] = res.boundary_reject_non_adjacent;
                                    result["boundary_fail_seed_collision"] = res.boundary_fail_seed_collision;
                                    result["boundary_fail_depth_cap"] = res.boundary_fail_depth_cap;
                                    result["boundary_fail_unknown_depth_cap"] = res.boundary_fail_unknown_depth_cap;
                                    result["boundary_fail_reserved_depth_cap"] = res.boundary_fail_reserved_depth_cap;
                                    result["boundary_fail_occupied"] = res.boundary_fail_occupied;
                                    result["boundary_fail_deadline"] = res.boundary_fail_deadline;
                                    result["boundary_fail_out_of_domain"] = res.boundary_fail_out_of_domain;
                                    result["boundary_fail_split"] = res.boundary_fail_split;
                                    result["boundary_failed_seed_memoized"] = res.boundary_failed_seed_memoized;
                                    result["boundary_skip_failed_seed"] = res.boundary_skip_failed_seed;
                                    result["boundary_stall"] = res.boundary_stall;
                                    result["boundary_target_hits"] = res.boundary_target_hits;
                                    py::list boundary_failures;
                                    for (const auto& failure : res.boundary_failures) {
                                        py::dict item;
                                        item["seed"] = failure.seed;
                                        item["intervals"] = interval_pairs_to_python(failure.intervals);
                                        item["validation_detail"] = oracle_validation_detail_to_python(failure.validation_detail);
                                        item["node"] = failure.node;
                                        item["depth"] = failure.depth;
                                        item["changed_dim"] = failure.changed_dim;
                                        item["fail_code"] = failure.fail_code;
                                        item["hit_unknown_depth_cap"] = failure.hit_unknown_depth_cap;
                                        item["hit_reserved_depth_cap"] = failure.hit_reserved_depth_cap;
                                        boundary_failures.append(std::move(item));
                                    }
                                    result["boundary_failures"] = std::move(boundary_failures);
                                    py::list out_waypoints;
                                    for (const auto& wp : res.waypoints) {
                                        out_waypoints.append(vector_to_list(wp));
                                    }
                                    result["waypoints"] = out_waypoints;
                                    py::list committed_boxes;
                                    for (const auto& box : res.committed_boxes) {
                                        committed_boxes.append(interval_pairs_to_python(box));
                                    }
                                    result["committed_boxes"] = committed_boxes;
                                    py::list all_boxes;
                                    for (const auto& box : res.all_boxes) {
                                        all_boxes.append(interval_pairs_to_python(box));
                                    }
                                    result["all_boxes"] = all_boxes;
                                    result["start_box"] = interval_pairs_to_python(res.start_box);
                                    result["goal_box"] = interval_pairs_to_python(res.goal_box);
                                    return result;
                         },
                         py::arg("waypoints"),
                         py::arg("max_chain") = 4096,
                         py::arg("max_depth") = 120,
                         py::arg("max_gap_fill_steps") = 64,
                         py::arg("fill_segment_gaps") = true,
                         py::arg("gap_fill_min_step") = 1e-4,
                         py::arg("adjacency_tolerance") = 1e-9,
                         py::arg("gap_fill_sample_step") = 0.05,
                         py::arg("gap_fill_time_budget_ms") = 10.0,
                         py::arg("gap_fill_max_ffb_calls") = 32,
                         py::arg("require_connected_chain") = false,
                         py::arg("commit_certified_only") = true)
        .def("add_obstacle_and_rebuild", &rbf::RBFPlanningForest::add_obstacle_and_rebuild, py::arg("obstacle"))
        .def("add_obstacles_and_rebuild", &rbf::RBFPlanningForest::add_obstacles_and_rebuild, py::arg("obstacles"))
        .def("connect_update_segment_fallback", &rbf::RBFPlanningForest::connect_update_segment_fallback)
        .def("connect_update_endpoint_segment_fallback",
             [](rbf::RBFPlanningForest& forest,
                const std::vector<double>& start,
                const std::vector<double>& goal) {
                 return forest.connect_update_endpoint_segment_fallback(eigen_vector_from_list(start),
                                                                        eigen_vector_from_list(goal));
             },
             py::arg("start"), py::arg("goal"))
        .def("remove_obstacle_and_regrow", &rbf::RBFPlanningForest::remove_obstacle_and_regrow, py::arg("obstacle_index"))
        .def("remove_obstacle_suffix_and_regrow", &rbf::RBFPlanningForest::remove_obstacle_suffix_and_regrow, py::arg("target_obstacle_count"))
        .def("clear_forest", &rbf::RBFPlanningForest::clear_forest)
        .def("database_node_count", [](const rbf::RBFPlanningForest& forest) {
            return forest.database().node_count();
        })
        .def("database_evidence_count", [](const rbf::RBFPlanningForest& forest) {
            return forest.database().evidence_count();
        })
        .def("database_root_intervals", [](const rbf::RBFPlanningForest& forest) {
            return interval_pairs_to_python(forest.database().root_intervals());
        })
        .def("database_coverage_intervals", [](const rbf::RBFPlanningForest& forest) {
            return interval_pairs_to_python(forest.database().coverage_intervals());
        })
        .def("database_checkpoint", [](rbf::RBFPlanningForest& forest) {
            return forest.database().checkpoint();
        })
        .def("database_verify", [](const rbf::RBFPlanningForest& forest, bool strict) {
            return forest.database().verify(strict).ok;
        }, py::arg("strict") = true)
        .def("database_snapshot_path", [](const rbf::RBFPlanningForest& forest) {
            return rbf::lect_database::LectReadSnapshot::default_snapshot_path(forest.config().database.path).string();
        })
        .def("debug_external_endpoint_lookup",
             [](const rbf::RBFPlanningForest& forest,
                const std::vector<std::vector<double>>& interval_pairs,
                const std::vector<rbf::Obstacle>& obstacles) {
                 py::dict result;
                 const auto intervals = intervals_from_pairs(interval_pairs);
                 result["intervals"] = interval_pairs_to_python(intervals);
                 const auto external_root = forest.config().database.external_evidence_path;
                 result["external_evidence_path"] = external_root.string();
                 if (external_root.empty()) {
                     result["found"] = false;
                     result["reason"] = "external_evidence_path is empty";
                     return result;
                 }
                 const auto snapshot_path = forest.config().database.external_evidence_snapshot_path.empty()
                     ? rbf::lect_database::LectReadSnapshot::default_snapshot_path(external_root)
                     : forest.config().database.external_evidence_snapshot_path;
                 result["snapshot_path"] = snapshot_path.string();
                 rbf::lect_database::LectReadSnapshot snapshot;
                 std::string open_reason;
                 if (!snapshot.open(snapshot_path, &open_reason)) {
                     result["found"] = false;
                     result["reason"] = open_reason;
                     return result;
                 }
                 rbf::lect_database::EvidenceKey key;
                 key.sector = rbf::lect_database::kPrimarySector;
                 key.channel = rbf::source_channel(forest.config().endpoint_source.source) == 0
                     ? rbf::lect_database::EvidenceChannel::Safe
                     : rbf::lect_database::EvidenceChannel::Rapid;
                 key.endpoint_source = forest.config().endpoint_source.source;
                 key.payload_kind = rbf::lect_database::EvidencePayloadKind::EndpointEnvelope;
                 const auto box_key = snapshot.make_box_key(intervals);
                 const auto lookup = snapshot.box_to_node_exact(box_key);
                 result["box_node_found"] = lookup.found;
                 result["box_node_id"] = lookup.found ? static_cast<unsigned long long>(lookup.node_id) : 0ULL;
                 result["box_node_reason"] = lookup.reason;
                 const auto view = snapshot.endpoint_for_box_exact(box_key, key);
                 if (!view) {
                     result["found"] = false;
                     result["reason"] = "endpoint evidence not found";
                     return result;
                 }
                 result["found"] = true;
                 result["node_id"] = static_cast<unsigned long long>(view->key.node_id);
                 result["sector"] = static_cast<unsigned int>(view->key.sector);
                 result["channel"] = static_cast<int>(view->key.channel);
                 result["endpoint_source"] = static_cast<int>(view->key.endpoint_source);
                 result["payload_kind"] = static_cast<int>(view->key.payload_kind);
                 result["payload_size"] = static_cast<unsigned long long>(view->payload.size());
                 result["child_hull"] = view->child_hull;
                 result["unavailable"] = view->unavailable;
                 result["generation"] = static_cast<unsigned long long>(view->generation);
                 result["checksum"] = static_cast<unsigned long long>(view->checksum);
                 rbf::LinkEnvelope envelope = rbf::compute_link_envelope(view->payload.data(),
                                                                         forest.robot().n_active_links(),
                                                                         forest.robot().active_link_radii(),
                                                                         forest.config().envelope_type);
                 rbf::EnvelopeCollisionOptions options;
                 options.safety_epsilon = std::max(forest.config().envelope_type.kdop_config.safety_epsilon,
                                                   forest.config().envelope_type.support_hull_config.safety_epsilon);
                 options.overlap_tolerance = std::max(forest.config().envelope_type.kdop_config.overlap_tolerance,
                                                       forest.config().envelope_type.support_hull_config.overlap_tolerance);
                 options.skip_aabb_broadphase =
                     forest.config().envelope_type.support_hull_config.skip_aabb_broadphase;
                 options.direct_support_hull_collision =
                     forest.config().envelope_type.support_hull_config.direct_collision;
                 rbf::EnvelopeCollisionStats stats;
                 const auto collision = rbf::collide_envelope_aabbs(envelope,
                                                                     obstacles.empty() ? nullptr : obstacles.data(),
                                                                     static_cast<int>(obstacles.size()),
                                                                     options,
                                                                     &stats);
                 result["external_is_definitely_free"] =
                     collision == rbf::CollisionResultKind::DefinitelyFree;
                 result["external_maybe_pairs"] = stats.maybe_pairs;
                 result["external_overlap_tolerance_rejects"] = stats.overlap_tolerance_rejects;
                 result["external_aabb_tests"] = stats.envelope_aabb_tests;
                 result["external_aabb_rejects"] = stats.envelope_aabb_rejects;
                 result["external_link_aabb_tests"] = stats.link_aabb_tests;
                 result["external_link_aabb_rejects"] = stats.link_aabb_rejects;
                 result["external_gjk_tests"] = stats.gjk_tests;
                 result["external_gjk_rejects"] = stats.gjk_rejects;
                 result["external_overlap_depth_max"] = stats.maybe_pair_overlap_depth_max;
                 result["external_overlap_volume_ratio_max"] = stats.maybe_pair_overlap_volume_ratio_max;
                 return result;
             },
             py::arg("interval_pairs"),
             py::arg("obstacles") = std::vector<rbf::Obstacle>{})
        .def("database_wait_for_snapshot_publish", [](const rbf::RBFPlanningForest& forest) {
            const auto snapshot_path = rbf::lect_database::LectReadSnapshot::default_snapshot_path(forest.config().database.path);
            return rbf::lect_database::LectReadSnapshot::build_from_legacy(forest.config().database.path, snapshot_path);
        })
        .def("prewarm_lifelong_cache",
             [](rbf::RBFPlanningForest& forest,
                int target_depth,
                const std::vector<rbf::Obstacle>& obstacles,
                bool gray_leaf_order,
                bool show_progress,
                bool streaming,
                std::size_t streaming_cap,
                double checkpoint_interval_s) {
                 if (obstacles.empty()) {
                     throw std::invalid_argument("prewarm_lifelong_cache requires a non-empty obstacle scene so endpoint evidence is materialized");
                 }
                 const auto start = std::chrono::steady_clock::now();
                 const int materialize_depth = std::max(0, target_depth);
                 // Prewarm persistence mode:
                 //   default                    -> bulk: all records resident,
                 //       fastest, RAM ~ O(records). Good up to ~D20.
                 //   streaming=true             -> resident cache is capped
                 //       so peak RAM stays bounded for deep trees (e.g. D25 ~62M
                 //       records). Records are appended to the durable store as
                 //       built; evicted child records are reloaded on demand by the
                 //       bottom-up parent sweep. Output is bit-identical to bulk.
                 const bool streaming_prewarm = streaming;
                 streaming_cap = std::max<std::size_t>(std::size_t{1}, streaming_cap);
                 checkpoint_interval_s = std::max(0.0, checkpoint_interval_s);
                 std::size_t periodic_checkpoint_attempts = 0;
                 std::size_t periodic_checkpoint_failures = 0;
                 auto last_checkpoint_time = start;
                 auto run_periodic_checkpoint = [&](const char* phase,
                                                    std::size_t done,
                                                    std::size_t total,
                                                    bool force = false) {
                     if (checkpoint_interval_s <= 0.0 || periodic_checkpoint_failures > 0) {
                         return;
                     }
                     const auto now = std::chrono::steady_clock::now();
                     const double since_last = std::chrono::duration<double>(now - last_checkpoint_time).count();
                     if (!force && since_last < checkpoint_interval_s) {
                         return;
                     }
                     const auto checkpoint_start = std::chrono::steady_clock::now();
                     const bool ok = forest.database().checkpoint();
                     const auto checkpoint_end = std::chrono::steady_clock::now();
                     ++periodic_checkpoint_attempts;
                     if (!ok) {
                         ++periodic_checkpoint_failures;
                     }
                     last_checkpoint_time = checkpoint_end;
                     if (show_progress) {
                         const double checkpoint_s = std::chrono::duration<double>(checkpoint_end - checkpoint_start).count();
                         std::fprintf(stderr,
                                      "\n[prewarm checkpoint] phase=%s done=%zu/%zu ok=%d elapsed %.1fs\n",
                                      phase,
                                      done,
                                      total,
                                      ok ? 1 : 0,
                                      checkpoint_s);
                         std::fflush(stderr);
                     }
                 };
                 const auto expected_leaf_records_for_depth = [](int depth) -> std::size_t {
                     if (depth < 0 || depth >= static_cast<int>(std::numeric_limits<std::size_t>::digits)) {
                         return 0;
                     }
                     return std::size_t{1} << depth;
                 };
                 const std::size_t expected_leaf_records = expected_leaf_records_for_depth(materialize_depth);
                 const std::size_t expected_prewarm_records =
                     expected_leaf_records > 0 &&
                             expected_leaf_records <= (std::numeric_limits<std::size_t>::max() - 64) / 2
                         ? expected_leaf_records * 2 + 64
                         : 0;
                 if (streaming_prewarm) {
                     forest.database().set_streaming_prewarm_mode(true, streaming_cap);
                 } else {
                     forest.database().set_bulk_prewarm_mode(true, expected_prewarm_records);
                 }
                 if (show_progress) {
                     std::fprintf(stderr, "[prewarm setup] ensure_depth 0/%d\n", materialize_depth);
                     std::fflush(stderr);
                 }
                 const auto ensure_start = std::chrono::steady_clock::now();
                 bool depth_ok = true;
                 if (show_progress) {
                     const std::size_t setup_total = expected_leaf_records > 0
                         ? expected_leaf_records - 1
                         : std::size_t{0};
                     std::size_t setup_done = 0;
                     for (int depth = 0; depth < materialize_depth && depth_ok; ++depth) {
                         const auto layer = forest.database().layer_nodes(depth);
                         const std::size_t layer_total = layer.size();
                         const std::size_t layer_stride =
                             std::max<std::size_t>(std::size_t{1}, layer_total / 200);
                         std::size_t layer_done = 0;
                         for (rbf::lect_database::NodeId node_id : layer) {
                             const auto children = forest.database().split_leaf(node_id);
                             if (children.first == rbf::lect_database::kInvalidNodeId ||
                                 children.second == rbf::lect_database::kInvalidNodeId) {
                                 depth_ok = false;
                                 break;
                             }
                             ++layer_done;
                             ++setup_done;
                             if (layer_done % layer_stride == 0 || layer_done == layer_total) {
                                 const double el = std::chrono::duration<double>(
                                                       std::chrono::steady_clock::now() - ensure_start)
                                                       .count();
                                 const double frac = static_cast<double>(setup_done) /
                                                     static_cast<double>(std::max<std::size_t>(setup_total, 1));
                                 const double eta = el * (1.0 - frac) / std::max(frac, 1e-9);
                                 std::fprintf(stderr,
                                              "\r[prewarm setup] depth %2d/%2d  layer %zu/%zu  %5.1f%%  elapsed %6.1fs  ETA %6.1fs   ",
                                              depth + 1,
                                              materialize_depth,
                                              layer_done,
                                              layer_total,
                                              100.0 * std::min(frac, 1.0),
                                              el,
                                              eta);
                                 std::fflush(stderr);
                             }
                             run_periodic_checkpoint("ensure_depth", setup_done, setup_total);
                         }
                     }
                     std::fprintf(stderr, "\n");
                     std::fflush(stderr);
                 } else {
                     depth_ok = forest.database().ensure_depth(materialize_depth);
                 }
                 if (show_progress) {
                     const double el = std::chrono::duration<double>(
                                           std::chrono::steady_clock::now() - ensure_start)
                                           .count();
                     std::fprintf(stderr, "[prewarm setup] ensure_depth done %.1fs\n", el);
                     std::fflush(stderr);
                 }
                 if (depth_ok) {
                     run_periodic_checkpoint("ensure_depth", expected_leaf_records, expected_leaf_records, true);
                 }
                 rbf::OracleValidationConfig prewarm_validation = forest.config().validation;
                 prewarm_validation.stateless_materialization_context = true;
                 rbf::DatabaseBoxOracle oracle(forest.robot(),
                                               forest.database(),
                                               rbf::Scene(obstacles),
                                               forest.config().endpoint_source,
                                               forest.config().envelope_type,
                                               prewarm_validation);
                 std::size_t nodes_touched = 0;
                 const std::size_t evidence_before = forest.database().evidence_count();
                 // Disable per-leaf auto-propagation; the bottom-up sweep below
                 // does all ancestor unions in a single O(leaves) pass instead of
                 // scattered O(leaves*depth) walk-ups during leaf inserts.
                 const bool prev_propagate = forest.database().propagate_parent_hulls_enabled();
                 forest.database().set_propagate_parent_hulls(false);
                 // Visit the leaf layer in reflected Gray-code (boustrophedon)
                 // order so that consecutive leaves differ in exactly one split
                 // decision -- i.e. exactly one joint interval changes between
                 // successive FK materializations. The IFK stateful endpoint
                 // source then reuses its AA-FK chain prefix and recomputes only
                 // the changed suffix; the incremental result is provably
                 // identical to a full pass, so the stored payloads are
                 // bit-for-bit unchanged -- this is a pure prewarm speedup.
                 const auto leaf_layer = forest.database().layer_nodes(materialize_depth);
                 if (!streaming_prewarm && leaf_layer.size() > expected_leaf_records) {
                     forest.database().set_bulk_prewarm_mode(true, leaf_layer.size() * 2 + 64);
                 }
                 std::vector<rbf::lect_database::NodeId> ordered_leaves;
                 ordered_leaves.reserve(leaf_layer.size());
                 {
                     const auto roots = forest.database().layer_nodes(0);
                     if (!roots.empty()) {
                         struct Frame {
                             rbf::lect_database::NodeId node;
                             int depth;
                             bool reversed;
                         };
                         std::vector<Frame> stack;
                         stack.push_back({roots.front(), 0, false});
                         while (!stack.empty()) {
                             const Frame fr = stack.back();
                             stack.pop_back();
                             if (fr.depth >= materialize_depth) {
                                 ordered_leaves.push_back(fr.node);
                                 continue;
                             }
                             const auto topo = forest.database().topology(fr.node);
                             if (topo.left == rbf::lect_database::kInvalidNodeId ||
                                 topo.right == rbf::lect_database::kInvalidNodeId) {
                                 ordered_leaves.push_back(fr.node);
                                 continue;
                             }
                             // forward: visit left (forward) then right (reversed);
                             // reversed: visit right (forward) then left (reversed).
                             const rbf::lect_database::NodeId first =
                                 fr.reversed ? topo.right : topo.left;
                             const rbf::lect_database::NodeId second =
                                 fr.reversed ? topo.left : topo.right;
                             // LIFO: push the second-visited child first.
                             stack.push_back({second, fr.depth + 1, true});
                             stack.push_back({first, fr.depth + 1, false});
                         }
                     }
                 }
                 const bool use_gray_order =
                     !ordered_leaves.empty() && ordered_leaves.size() == leaf_layer.size();
                 const std::vector<rbf::lect_database::NodeId>& leaf_iteration =
                     (gray_leaf_order && use_gray_order) ? ordered_leaves : leaf_layer;
                 const auto leaf_loop_start = std::chrono::steady_clock::now();
                 const std::size_t leaf_total = leaf_iteration.size();
                 const std::size_t leaf_stride =
                     std::max<std::size_t>(std::size_t{1}, leaf_total / 200);
                 std::size_t leaf_done = 0;
                 for (rbf::lect_database::NodeId node_id : leaf_iteration) {
                     auto intervals = forest.database().node_box(node_id);
                     if (!intervals) {
                         continue;
                     }
                     oracle.validate_node(static_cast<int>(node_id), *intervals, -1);
                     nodes_touched += 1;
                     ++leaf_done;
                     if (show_progress &&
                         (leaf_done % leaf_stride == 0 || leaf_done == leaf_total)) {
                         const double el = std::chrono::duration<double>(
                                               std::chrono::steady_clock::now() - leaf_loop_start)
                                               .count();
                         const double rate = static_cast<double>(leaf_done) / std::max(el, 1e-9);
                         const double eta =
                             static_cast<double>(leaf_total - leaf_done) / std::max(rate, 1e-9);
                         std::fprintf(stderr,
                                      "\r[prewarm leaves]  %5.1f%%  %zu/%zu  %.0f/s  elapsed %6.1fs  ETA %6.1fs   ",
                                      100.0 * static_cast<double>(leaf_done) /
                                          static_cast<double>(std::max<std::size_t>(leaf_total, 1)),
                                      leaf_done, leaf_total, rate, el, eta);
                         std::fflush(stderr);
                     }
                     run_periodic_checkpoint("leaves", leaf_done, leaf_total);
                 }
                 if (show_progress) {
                     std::fprintf(stderr, "\n");
                     std::fflush(stderr);
                 }
                 const auto leaf_loop_end = std::chrono::steady_clock::now();
                 // HARDCODED: LECT prewarm only FK-materializes the leaf layer,
                 // then derives every internal-node envelope bottom-up as the
                 // cheap conservative union of its two children (tighter than a
                 // direct parent FK). Internal nodes are never FK-recomputed.
                 const std::size_t parent_total_estimate =
                     forest.database().node_count() > leaf_total
                         ? forest.database().node_count() - leaf_total
                         : std::size_t{0};
                 const std::size_t parent_hulls_built =
                     forest.database().materialize_internal_parent_hulls_bottom_up(
                         materialize_depth, oracle.endpoint_evidence_key(0),
                         show_progress
                             ? std::function<void(int, std::size_t)>(
                                   [&](int depth, std::size_t built) {
                                       const double el =
                                           std::chrono::duration<double>(
                                               std::chrono::steady_clock::now() - leaf_loop_end)
                                               .count();
                                       const int done_layers = materialize_depth - depth;
                                       const double frac =
                                           static_cast<double>(done_layers) /
                                           static_cast<double>(std::max(materialize_depth, 1));
                                       const double eta =
                                           el * (1.0 - frac) / std::max(frac, 1e-9);
                                       std::fprintf(stderr,
                                                    "\r[prewarm parents] depth %2d  %5.1f%%  built %zu  elapsed %6.1fs  ETA %6.1fs   ",
                                                    depth, 100.0 * frac, built, el, eta);
                                       std::fflush(stderr);
                                       run_periodic_checkpoint("parents", built, parent_total_estimate);
                                   })
                             : std::function<void(int, std::size_t)>{});
                 if (show_progress) {
                     std::fprintf(stderr, "\n");
                     std::fflush(stderr);
                 }
                 const auto sweep_end = std::chrono::steady_clock::now();
                 forest.database().set_propagate_parent_hulls(prev_propagate);
                 const bool checkpoint_ok = forest.database().checkpoint();
                 forest.database().set_bulk_prewarm_mode(false);
                 forest.database().set_streaming_prewarm_mode(false, 0);
                 const auto end = std::chrono::steady_clock::now();
                 const auto& counters = oracle.counters();
                 py::dict result;
                 result["ok"] = depth_ok && checkpoint_ok && periodic_checkpoint_failures == 0;
                 result["target_depth"] = materialize_depth;
                 result["depth_ok"] = depth_ok;
                 result["checkpoint_ok"] = checkpoint_ok;
                 result["periodic_checkpoint_seconds"] = checkpoint_interval_s;
                 result["periodic_checkpoint_attempts"] = periodic_checkpoint_attempts;
                 result["periodic_checkpoint_failures"] = periodic_checkpoint_failures;
                 result["nodes_touched"] = nodes_touched;
                 result["parent_hulls_built"] = parent_hulls_built;
                 result["node_count"] = forest.database().node_count();
                 result["evidence_before"] = evidence_before;
                 result["evidence_after"] = forest.database().evidence_count();
                 result["materializations"] = counters.materializations;
                 result["reused_endpoint_cache"] = counters.materialization_reused_endpoint_cache;
                 result["reused_shared_endpoint_cache"] = counters.materialization_reused_shared_endpoint_cache;
                 result["stored_shared_endpoint_cache"] = counters.materialization_stored_shared_endpoint_cache;
                 result["incremental_fk"] = counters.materialization_incremental_fk;
                 result["source_incremental_state"] = counters.materialization_source_incremental_state;
                 result["gray_leaf_order"] = gray_leaf_order && use_gray_order;
                 // --- prewarm time breakdown (seconds) ---
                 result["t_leaf_loop_s"] =
                     std::chrono::duration<double>(leaf_loop_end - leaf_loop_start).count();
                 result["t_parent_sweep_s"] =
                     std::chrono::duration<double>(sweep_end - leaf_loop_end).count();
                 result["t_checkpoint_s"] =
                     std::chrono::duration<double>(end - sweep_end).count();
                 // --- per-leaf validate_node cost decomposition (microseconds) ---
                 result["us_validate_total"] = counters.validate_node_total_time_us;
                 result["us_validate_endpoint_path"] = counters.validate_node_endpoint_path_time_us;
                 result["us_validate_classify"] = counters.validate_node_classify_time_us;
                 result["us_endpoint_fk_wall"] = counters.materialization_endpoint_wall_time_us;
                 result["us_envelope_compute"] = counters.materialization_envelope_compute_time_us;
                 result["us_envelope_collision"] = counters.materialization_envelope_collision_time_us;
                 result["us_cache_lookup"] = counters.materialization_cache_lookup_time_us;
                 result["us_cache_read"] = counters.materialization_cache_read_time_us;
                 result["wall_s"] = std::chrono::duration<double>(end - start).count();
                 return result;
             },
             py::arg("target_depth"),
             py::arg("obstacles"),
             py::arg("gray_leaf_order") = true,
             py::arg("show_progress") = true,
             py::arg("streaming") = false,
             py::arg("streaming_cap") = static_cast<std::size_t>(2000000),
             py::arg("checkpoint_interval_s") = 0.0)
        .def("debug_validate_intervals",
             [](rbf::RBFPlanningForest& forest,
                const std::vector<rbf::Obstacle>& obstacles,
                const std::vector<std::vector<double>>& interval_pairs,
                int changed_dim,
                bool disable_caches) {
                 const auto intervals = intervals_from_pairs(interval_pairs);
                 if (static_cast<int>(intervals.size()) != forest.robot().n_joints()) {
                     throw std::invalid_argument("interval count must match robot.n_joints()");
                 }
                 rbf::OracleValidationConfig validation = disable_caches
                     ? uncached_validation_config(forest.config().validation)
                     : forest.config().validation;
                 rbf::lect_database::OnlineEnvelopeCacheTree cache(forest.database(), {});
                 std::unique_ptr<rbf::lect_database::LectReadSnapshot> external_snapshot;
                 std::unique_ptr<rbf::lect_database::LectSnapshotEvidenceSource> external_snapshot_source;
                 std::unique_ptr<rbf::lect_database::LectDatabase> external_database;
                 std::unique_ptr<rbf::lect_database::LectDatabaseEvidenceSource> external_database_source;
                 const rbf::lect_database::LectExternalEvidenceSource* external_source = nullptr;
                 const rbf::lect_database::LectDatabase* direct_external_database = nullptr;
                 if (!disable_caches && !forest.config().database.external_evidence_path.empty()) {
                     const auto& runtime = forest.config().database;
                     if (runtime.external_evidence_use_snapshot) {
                         const auto snapshot_path = runtime.external_evidence_snapshot_path.empty()
                             ? rbf::lect_database::LectReadSnapshot::default_snapshot_path(runtime.external_evidence_path)
                             : runtime.external_evidence_snapshot_path;
                         external_snapshot = std::make_unique<rbf::lect_database::LectReadSnapshot>();
                         std::string reason;
                         if (!external_snapshot->open(snapshot_path, &reason)) {
                             external_snapshot.reset();
                         } else {
                             external_snapshot_source =
                                 std::make_unique<rbf::lect_database::LectSnapshotEvidenceSource>(*external_snapshot);
                             external_source = external_snapshot_source.get();
                         }
                     }
                 }
                 rbf::DatabaseBoxOracle oracle(forest.robot(),
                                               cache,
                                               rbf::Scene(obstacles),
                                               forest.config().endpoint_source,
                                               forest.config().envelope_type,
                                               validation,
                                               external_source,
                                               direct_external_database);
                 const auto validation_result = oracle.validate_node(oracle.root_node(), intervals, changed_dim);
                 py::dict result;
                 result["validation"] = static_cast<int>(validation_result);
                 result["intervals"] = interval_pairs_to_python(intervals);
                 result["root_intervals"] = interval_pairs_to_python(oracle.root_intervals());
                 result["changed_dim"] = changed_dim;
                 result["disable_caches"] = disable_caches;
                 result["validation_detail"] = oracle_validation_detail_to_python(oracle.last_validation_detail());
                 result["counters"] = oracle_counters_to_python(oracle.counters());
                 return result;
             },
             py::arg("obstacles"),
             py::arg("interval_pairs"),
             py::arg("changed_dim") = -1,
             py::arg("disable_caches") = true)
        .def("debug_compute_envelope_summary",
             [](rbf::RBFPlanningForest& forest,
                const std::vector<std::vector<double>>& interval_pairs) {
                 const auto intervals = intervals_from_pairs(interval_pairs);
                 if (static_cast<int>(intervals.size()) != forest.robot().n_joints()) {
                     throw std::invalid_argument("interval count must match robot.n_joints()");
                 }
                 const std::vector<std::vector<rbf::Interval>> boxes{intervals};
                 const auto results = link_interval_envelope::compute_envelope_batch(
                     forest.robot(),
                     boxes,
                     forest.config().endpoint_source,
                     forest.config().envelope_type,
                     1);
                 if (results.empty()) {
                     throw std::runtime_error("envelope batch returned no results");
                 }
                 const auto& batch = results.front();
                 const auto& envelope = batch.envelope;
                 auto aabb_volume = [](const std::vector<float>& aabb, std::size_t offset) {
                     if (aabb.size() < offset + 6U) {
                         return 0.0;
                     }
                     const double dx = std::max(0.0, static_cast<double>(aabb[offset + 3U] - aabb[offset + 0U]));
                     const double dy = std::max(0.0, static_cast<double>(aabb[offset + 4U] - aabb[offset + 1U]));
                     const double dz = std::max(0.0, static_cast<double>(aabb[offset + 5U] - aabb[offset + 2U]));
                     return dx * dy * dz;
                 };
                 auto aabb_extents = [](const std::vector<float>& aabb, std::size_t offset) {
                     std::vector<double> out(3, 0.0);
                     if (aabb.size() < offset + 6U) {
                         return out;
                     }
                     for (std::size_t axis = 0; axis < 3U; ++axis) {
                         out[axis] = std::max(0.0, static_cast<double>(aabb[offset + axis + 3U] - aabb[offset + axis]));
                     }
                     return out;
                 };
                 double sum_link_union_volume = 0.0;
                 py::list link_unions;
                 for (int link = 0; link < envelope.n_active_links; ++link) {
                     const std::size_t offset = static_cast<std::size_t>(link) * 6U;
                     const double volume = aabb_volume(envelope.link_union_iaabbs, offset);
                     sum_link_union_volume += volume;
                     py::dict item;
                     item["link"] = link;
                     if (envelope.link_union_iaabbs.size() >= offset + 6U) {
                         item["aabb"] = std::vector<float>(
                             envelope.link_union_iaabbs.begin() + static_cast<std::ptrdiff_t>(offset),
                             envelope.link_union_iaabbs.begin() + static_cast<std::ptrdiff_t>(offset + 6U));
                     } else {
                         item["aabb"] = std::vector<float>{};
                     }
                     item["extents"] = aabb_extents(envelope.link_union_iaabbs, offset);
                     item["volume"] = volume;
                     link_unions.append(std::move(item));
                 }
                 py::dict result;
                 result["intervals"] = interval_pairs_to_python(intervals);
                 result["source"] = static_cast<int>(batch.source);
                 result["is_safe"] = batch.is_safe;
                 result["endpoint_safety_level"] = static_cast<int>(batch.endpoint_safety_level);
                 result["n_active_links"] = envelope.n_active_links;
                 result["n_subdivisions"] = envelope.n_subdivisions;
                 result["envelope_type"] = static_cast<int>(envelope.type);
                 result["endpoint_time_us"] = batch.endpoint_time_us;
                 result["envelope_time_us"] = batch.envelope_time_us;
                 result["envelope_aabb"] = envelope.envelope_aabb;
                 result["envelope_aabb_extents"] = aabb_extents(envelope.envelope_aabb, 0);
                 result["envelope_aabb_volume"] = aabb_volume(envelope.envelope_aabb, 0);
                 result["sum_link_union_volume"] = sum_link_union_volume;
                 result["link_unions"] = std::move(link_unions);
                 result["link_iaabb_count"] = static_cast<int>(envelope.link_iaabbs.size() / 6U);
                 result["support_hull_records"] = envelope.support_hulls.empty()
                     ? 0
                     : static_cast<int>(envelope.support_hulls.size());
                 result["kdop_values"] = static_cast<int>(envelope.kdop_intervals.size());
                 return result;
             },
             py::arg("interval_pairs"))
        .def("debug_find_free_box",
             [](rbf::RBFPlanningForest& forest,
                const std::vector<double>& seed_values,
                const std::vector<rbf::Obstacle>& obstacles,
                const rbf::FindFreeBoxOptions& options,
                bool disable_caches) {
                 using Clock = std::chrono::steady_clock;
                 const Eigen::VectorXd seed = eigen_vector_from_list(seed_values);
                 if (seed.size() != forest.robot().n_joints()) {
                     throw std::invalid_argument("seed dimension must match robot.n_joints()");
                 }

                 rbf::OracleValidationConfig validation = disable_caches
                     ? uncached_validation_config(forest.config().validation)
                     : forest.config().validation;
                 rbf::lect_database::OnlineEnvelopeCacheTree cache(forest.database(), {});
                 std::unique_ptr<rbf::lect_database::LectReadSnapshot> external_snapshot;
                 std::unique_ptr<rbf::lect_database::LectSnapshotEvidenceSource> external_snapshot_source;
                 const rbf::lect_database::LectExternalEvidenceSource* external_source = nullptr;
                 const rbf::lect_database::LectDatabase* direct_external_database = nullptr;
                 if (!disable_caches && !forest.config().database.external_evidence_path.empty()) {
                     const auto& runtime = forest.config().database;
                     if (runtime.external_evidence_use_snapshot) {
                         const auto snapshot_path = runtime.external_evidence_snapshot_path.empty()
                             ? rbf::lect_database::LectReadSnapshot::default_snapshot_path(runtime.external_evidence_path)
                             : runtime.external_evidence_snapshot_path;
                         external_snapshot = std::make_unique<rbf::lect_database::LectReadSnapshot>();
                         std::string reason;
                         if (!external_snapshot->open(snapshot_path, &reason)) {
                             external_snapshot.reset();
                         } else {
                             external_snapshot_source =
                                 std::make_unique<rbf::lect_database::LectSnapshotEvidenceSource>(*external_snapshot);
                             external_source = external_snapshot_source.get();
                         }
                     }
                 }
                 rbf::DatabaseBoxOracle oracle(forest.robot(),
                                               cache,
                                               rbf::Scene(obstacles),
                                               forest.config().endpoint_source,
                                               forest.config().envelope_type,
                                               validation,
                                               external_source,
                                               direct_external_database);

                 py::dict result;
                 py::list trace;
                 py::list split_events;
                 py::list validation_events;
                 const auto start = Clock::now();
                 result["external_source_enabled"] = external_source != nullptr;
                 result["external_direct_database_enabled"] = direct_external_database != nullptr;

                 const Eigen::VectorXd tree_seed = oracle.tree_configuration_for_query(seed);
                 result["seed"] = seed_values;
                 result["tree_seed"] = vector_to_list(tree_seed);
                 result["root_intervals"] = interval_pairs_to_python(oracle.root_intervals());
                 result["disable_caches"] = disable_caches;
                 result["trace_mode"] = "linear_trace_not_production_ffb";

                 // Seed-independent: canonical split depends only on (robot,
                 // domain). No query-seed coupling is applied to split values.
                 rbf::OracleSplitOptions split_options = options.split;

                 bool seed_in_domain = false;
                 if (seed.size() == oracle.n_dims()) {
                     seed_in_domain = oracle.contains_point(oracle.root_node(), seed);
                 }
                 result["seed_in_domain"] = seed_in_domain;
                 if (seed.size() != oracle.n_dims() || !seed_in_domain) {
                     result["found"] = false;
                     result["seed_collision"] = false;
                     result["hit_reserved_depth_cap"] = false;
                     result["hit_unknown_depth_cap"] = false;
                     result["deadline_reached"] = false;
                     result["fail_code"] = 5;
                     result["node"] = rbf::kInvalidOracleNodeId;
                     result["decisions"] = 0;
                     result["splits"] = 0;
                     result["changed_dim"] = -1;
                     result["intervals"] = py::list();
                     result["trace"] = trace;
                     result["validation_events"] = validation_events;
                     result["split_events"] = split_events;
                     result["counters"] = oracle_counters_to_python(oracle.counters());
                     result["total_ms"] = 0.0;
                     return result;
                 }

                 if (options.reject_seed_collision && oracle.point_in_collision(seed)) {
                     result["found"] = false;
                     result["seed_collision"] = true;
                     result["hit_reserved_depth_cap"] = false;
                     result["hit_unknown_depth_cap"] = false;
                     result["deadline_reached"] = false;
                     result["fail_code"] = 1;
                     result["node"] = rbf::kInvalidOracleNodeId;
                     result["decisions"] = 0;
                     result["splits"] = 0;
                     result["changed_dim"] = -1;
                     result["intervals"] = py::list();
                     result["trace"] = trace;
                     result["validation_events"] = validation_events;
                     result["split_events"] = split_events;
                     result["counters"] = oracle_counters_to_python(oracle.counters());
                     result["total_ms"] = 0.0;
                     return result;
                 }

                 auto elapsed_ms = [&]() {
                     return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
                 };

                 bool found = false;
                 bool seed_collision = false;
                 bool hit_reserved_depth_cap = false;
                 bool hit_unknown_depth_cap = false;
                 bool deadline_reached = false;
                 int fail_code = 0;
                 rbf::OracleNodeId node = oracle.root_node();
                 int changed_dim = -1;
                 int decisions = 0;
                 int splits = 0;
                 int result_changed_dim = -1;
                 std::vector<rbf::Interval> result_intervals;
                 rbf::OracleValidationDetail final_detail;
                 const int effective_max_depth = std::max(0, std::min(options.max_depth, oracle.max_tree_depth() - 1));
                 std::uint64_t step_sequence = 0;
                 std::uint64_t split_sequence = 0;
                 std::uint64_t validation_sequence = 0;

                 while (true) {
                     py::dict step;
                     step["sequence"] = step_sequence++;
                     step["node"] = node;
                     step["depth"] = oracle.depth(node);
                     step["changed_dim_in"] = changed_dim;
                     const auto tree_intervals = oracle.node_intervals(node);
                     const auto query_intervals = oracle.query_intervals_for_node(node, tree_intervals, seed);
                     step["tree_intervals"] = interval_pairs_to_python(tree_intervals);
                     step["query_intervals"] = interval_pairs_to_python(query_intervals);
                     step["is_leaf"] = oracle.is_leaf(node);
                     step["is_reserved"] = oracle.is_reserved(node);

                     if (step.cast<py::dict>()["is_reserved"].cast<bool>()) {
                         if (oracle.depth(node) >= effective_max_depth || !options.split_reserved_leaf) {
                             hit_reserved_depth_cap = true;
                             result_intervals = query_intervals;
                             fail_code = 2;
                             step["terminal"] = true;
                             step["fail_code"] = fail_code;
                             trace.append(std::move(step));
                             break;
                         }
                         if (oracle.is_leaf(node)) {
                             const auto split = oracle.split_node(node, tree_intervals, changed_dim, split_options);
                             step["split"] = split.split;
                             if (!split.split) {
                                 fail_code = 6;
                                 step["terminal"] = true;
                                 step["fail_code"] = fail_code;
                                 trace.append(std::move(step));
                                 break;
                             }
                             step["split_dim"] = split.split_dim;
                             step["split_value"] = split.split_value;
                             splits += 1;
                             py::dict split_event;
                             split_event["sequence"] = split_sequence++;
                             split_event["node"] = split.node;
                             split_event["depth"] = oracle.depth(split.node);
                             split_event["split_dim"] = split.split_dim;
                             split_event["split_val"] = split.split_value;
                             split_event["best_tighten"] = options.split.use_best_tighten;
                             split_event["sector_aligned"] = false;
                             split_events.append(std::move(split_event));
                         }
                         changed_dim = oracle.split_dim(node);
                         const double next_split_value = oracle.split_value(node);
                         const bool go_left = tree_seed[changed_dim] <= next_split_value;
                         step["next_split_dim"] = changed_dim;
                         step["next_split_value"] = next_split_value;
                         step["child_branch"] = go_left ? "left" : "right";
                         trace.append(std::move(step));
                         node = go_left ? oracle.left_child(node) : oracle.right_child(node);
                         continue;
                     }

                     const auto validation_result = oracle.validate_node(node, query_intervals, changed_dim);
                     final_detail = oracle.last_validation_detail();
                     decisions += 1;
                     step["validation"] = static_cast<int>(validation_result);
                     step["validation_detail"] = oracle_validation_detail_to_python(final_detail);
                     py::dict validation_event;
                     validation_event["sequence"] = validation_sequence++;
                     validation_event["node"] = node;
                     validation_event["depth"] = oracle.depth(node);
                     validation_event["validation"] = static_cast<int>(validation_result);
                     validation_event["safety_status"] = static_cast<int>(final_detail.safety_status);
                     validation_event["collision_possible"] = final_detail.collision_possible;
                     validation_event["strict_audit_required"] = final_detail.strict_audit_required;
                     validation_event["intervals"] = interval_pairs_to_python(query_intervals);
                     validation_events.append(std::move(validation_event));

                     if (validation_result == rbf::BoxValidation::Free) {
                         found = true;
                         result_changed_dim = changed_dim;
                         result_intervals = query_intervals;
                         fail_code = 0;
                         step["terminal"] = true;
                         step["fail_code"] = fail_code;
                         trace.append(std::move(step));
                         break;
                     }
                     if (validation_result == rbf::BoxValidation::Occupied) {
                         result_intervals = query_intervals;
                         fail_code = 3;
                         step["terminal"] = true;
                         step["fail_code"] = fail_code;
                         trace.append(std::move(step));
                         break;
                     }
                     if (oracle.depth(node) >= effective_max_depth || !options.split_unknown_leaf) {
                         hit_unknown_depth_cap = true;
                         result_intervals = query_intervals;
                         fail_code = 2;
                         step["terminal"] = true;
                         step["fail_code"] = fail_code;
                         trace.append(std::move(step));
                         break;
                     }
                     if (oracle.is_leaf(node)) {
                         const auto split = oracle.split_node(node, tree_intervals, changed_dim, split_options);
                         step["split"] = split.split;
                         if (!split.split) {
                             fail_code = 6;
                             step["terminal"] = true;
                             step["fail_code"] = fail_code;
                             trace.append(std::move(step));
                             break;
                         }
                         step["split_dim"] = split.split_dim;
                         step["split_value"] = split.split_value;
                         splits += 1;
                         py::dict split_event;
                         split_event["sequence"] = split_sequence++;
                         split_event["node"] = split.node;
                         split_event["depth"] = oracle.depth(split.node);
                         split_event["split_dim"] = split.split_dim;
                         split_event["split_val"] = split.split_value;
                         split_event["best_tighten"] = options.split.use_best_tighten;
                         split_event["sector_aligned"] = false;
                         split_events.append(std::move(split_event));
                     }
                     changed_dim = oracle.split_dim(node);
                     const double next_split_value = oracle.split_value(node);
                     const bool go_left = tree_seed[changed_dim] <= next_split_value;
                     step["next_split_dim"] = changed_dim;
                     step["next_split_value"] = next_split_value;
                     step["child_branch"] = go_left ? "left" : "right";
                     trace.append(std::move(step));
                     node = go_left ? oracle.left_child(node) : oracle.right_child(node);
                 }

                 result["found"] = found;
                 result["seed_collision"] = seed_collision;
                 result["hit_reserved_depth_cap"] = hit_reserved_depth_cap;
                 result["hit_unknown_depth_cap"] = hit_unknown_depth_cap;
                 result["deadline_reached"] = deadline_reached;
                 result["fail_code"] = fail_code;
                 result["node"] = node;
                 result["decisions"] = decisions;
                 result["splits"] = splits;
                 result["changed_dim"] = result_changed_dim;
                 result["intervals"] = interval_pairs_to_python(result_intervals);
                 result["validation_detail"] = oracle_validation_detail_to_python(final_detail);
                 result["trace"] = trace;
                 result["validation_events"] = validation_events;
                 result["split_events"] = split_events;
                 result["counters"] = oracle_counters_to_python(oracle.counters());
                 result["effective_max_depth"] = effective_max_depth;
                 result["total_ms"] = elapsed_ms();
                 return result;
             },
             py::arg("seed"),
             py::arg("obstacles"),
             py::arg("options") = rbf::FindFreeBoxOptions{},
             py::arg("disable_caches") = true)
        .def("debug_cover_path_with_ffb",
             [](rbf::RBFPlanningForest& forest,
                const std::vector<std::vector<double>>& waypoint_values,
                const std::vector<rbf::Obstacle>& obstacles,
                const rbf::FindFreeBoxOptions& options,
                double sample_step,
                int max_ffb_calls,
                double coverage_tolerance,
                bool include_existing_boxes,
                bool disable_caches,
                int max_failure_records,
                const std::vector<double>& refine_sample_steps,
                int parallel_workers,
                bool repair_corridor_adjacency,
                int repair_rounds,
                int repair_segment_subdivisions) {
                 using Clock = std::chrono::steady_clock;
                 const auto start = Clock::now();
                 const std::vector<Eigen::VectorXd> waypoints =
                     eigen_vectors_from_lists(waypoint_values);
                 if (waypoints.empty()) {
                     throw std::invalid_argument("waypoint path must be non-empty");
                 }
                 for (const auto& waypoint : waypoints) {
                     if (waypoint.size() != forest.robot().n_joints()) {
                         throw std::invalid_argument("waypoint dimension must match robot.n_joints()");
                     }
                 }
                 std::vector<double> pass_steps;
                 if (!refine_sample_steps.empty()) {
                     pass_steps.reserve(refine_sample_steps.size());
                     for (double step : refine_sample_steps) {
                         if (std::isfinite(step) && step > 0.0) {
                             pass_steps.push_back(step);
                         }
                     }
                 }
                 if (pass_steps.empty()) {
                     pass_steps.push_back(sample_step);
                 }
                 const double final_sample_step = *std::min_element(pass_steps.begin(), pass_steps.end());
                 std::vector<Eigen::VectorXd> samples =
                     densify_path_pybind(waypoints, final_sample_step);
                 const double path_length = rbf::path_length(waypoints);

                 rbf::OracleValidationConfig validation = disable_caches
                     ? uncached_validation_config(forest.config().validation)
                     : forest.config().validation;
                 rbf::lect_database::OnlineEnvelopeCacheTree cache(forest.database(), {});
                 std::unique_ptr<rbf::lect_database::LectReadSnapshot> external_snapshot;
                 std::unique_ptr<rbf::lect_database::LectSnapshotEvidenceSource> external_snapshot_source;
                 const rbf::lect_database::LectExternalEvidenceSource* external_source = nullptr;
                 const rbf::lect_database::LectDatabase* direct_external_database = nullptr;
                 if (!disable_caches && !forest.config().database.external_evidence_path.empty()) {
                     const auto& runtime = forest.config().database;
                     if (runtime.external_evidence_use_snapshot) {
                         const auto snapshot_path = runtime.external_evidence_snapshot_path.empty()
                             ? rbf::lect_database::LectReadSnapshot::default_snapshot_path(runtime.external_evidence_path)
                             : runtime.external_evidence_snapshot_path;
                         external_snapshot = std::make_unique<rbf::lect_database::LectReadSnapshot>();
                         std::string reason;
                         if (!external_snapshot->open(snapshot_path, &reason)) {
                             external_snapshot.reset();
                         } else {
                             external_snapshot_source =
                                 std::make_unique<rbf::lect_database::LectSnapshotEvidenceSource>(*external_snapshot);
                             external_source = external_snapshot_source.get();
                         }
                     }
                 }
                 rbf::DatabaseBoxOracle oracle(forest.robot(),
                                               cache,
                                               rbf::Scene(obstacles),
                                               forest.config().endpoint_source,
                                               forest.config().envelope_type,
                                               validation,
                                               external_source,
                                               direct_external_database);
                 rbf::FindFreeBoxService ffb(oracle);
                 rbf::StageContext context = rbf::StageContext::from_runtime(forest.config().runtime);

                 std::vector<std::vector<rbf::Interval>> boxes;
                 if (include_existing_boxes) {
                     for (const auto& box : forest.boxes()) {
                         boxes.push_back(box.joint_intervals);
                     }
                 }
                 const int initial_box_count = static_cast<int>(boxes.size());
                 std::vector<bool> covered(samples.size(), false);
                 std::vector<std::vector<int>> sample_layers(samples.size());
                 auto mark_covered = [&](std::size_t from_index) {
                     int changed = 0;
                     for (std::size_t sample_index = 0; sample_index < samples.size(); ++sample_index) {
                         for (std::size_t box_index = from_index; box_index < boxes.size(); ++box_index) {
                             if (intervals_contain_point_pybind(boxes[box_index],
                                                                samples[sample_index],
                                                                coverage_tolerance)) {
                                 auto& layer = sample_layers[sample_index];
                                 const int box_id = static_cast<int>(box_index);
                                 if (std::find(layer.begin(), layer.end(), box_id) == layer.end()) {
                                     layer.push_back(box_id);
                                 }
                                 if (!covered[sample_index]) {
                                     covered[sample_index] = true;
                                     changed += 1;
                                 }
                                 if (from_index == 0) {
                                     break;
                                 }
                                 if (box_index + 1 == boxes.size()) {
                                     break;
                                 }
                             }
                             if (from_index == 0 && covered[sample_index]) {
                                 break;
                             }
                         }
                     }
                     return changed;
                 };
                 mark_covered(0);

                 int ffb_calls = 0;
                 int found = 0;
                 int rejected_not_containing_seed = 0;
                 std::vector<int> fail_counts(16, 0);
                 py::list failures;
                 double presplit_ms = 0.0;
                 double repair_ms = 0.0;
                 int repair_calls = 0;
                 int repair_added = 0;
                 int repair_bad_transitions_initial = 0;
                 int repair_bad_transitions_final = 0;
                 double repair_bad_transition_length_initial = 0.0;
                 double repair_bad_transition_length_final = 0.0;
                 rbf::OracleCounters result_counters_override;
                 bool use_counter_override = false;
                 py::list pass_summaries;
                 auto point_covered_by_boxes = [&](const Eigen::VectorXd& point) {
                     for (const auto& box : boxes) {
                         if (intervals_contain_point_pybind(box, point, coverage_tolerance)) {
                             return true;
                         }
                     }
                     return false;
                 };
                 if (parallel_workers > 1 && pass_steps.size() == 1) {
                     const auto presplit_start = Clock::now();
                     const int effective_max_depth =
                         std::max(0, std::min(options.max_depth, oracle.max_tree_depth() - 1));
                     auto presplit_seed = [&](const Eigen::VectorXd& seed) {
                         if (seed.size() != oracle.n_dims() ||
                             !oracle.contains_point(oracle.root_node(), seed)) {
                             return;
                         }
                         rbf::OracleNodeId node = oracle.root_node();
                         int changed_dim = -1;
                         while (node != rbf::kInvalidOracleNodeId &&
                                oracle.depth(node) < effective_max_depth) {
                             auto tree_intervals = oracle.node_intervals(node);
                             if (oracle.is_leaf(node)) {
                                 const auto split = oracle.split_node(node,
                                                                      tree_intervals,
                                                                      changed_dim,
                                                                      options.split);
                                 if (!split.split) {
                                     return;
                                 }
                             }
                             changed_dim = oracle.split_dim(node);
                             node = oracle.child_containing_point(node, seed);
                         }
                     };
                     for (const auto& seed : samples) {
                         if (!point_covered_by_boxes(seed)) {
                             presplit_seed(seed);
                         }
                     }
                     presplit_ms =
                         std::chrono::duration<double, std::milli>(Clock::now() - presplit_start).count();

                     std::vector<std::size_t> candidate_indices;
                     candidate_indices.reserve(samples.size());
                     for (std::size_t sample_index = 0; sample_index < samples.size(); ++sample_index) {
                         if (!point_covered_by_boxes(samples[sample_index])) {
                             candidate_indices.push_back(sample_index);
                             if (max_ffb_calls >= 0 &&
                                 static_cast<int>(candidate_indices.size()) >= max_ffb_calls) {
                                 break;
                             }
                         }
                     }
                     struct ParallelCoverResult {
                         bool ran = false;
                         std::size_t sample_index = 0;
                         rbf::FindFreeBoxResult ffb;
                     };
                     std::vector<ParallelCoverResult> parallel_results(candidate_indices.size());
                     std::vector<rbf::OracleCounters> worker_counters(
                         static_cast<std::size_t>(std::max(1, parallel_workers)));
                     std::atomic<std::size_t> next{0};
                     const int worker_count =
                         std::max(1, std::min(parallel_workers, static_cast<int>(candidate_indices.size())));
                     std::vector<std::thread> workers;
                     workers.reserve(static_cast<std::size_t>(worker_count));
                     const auto parallel_start = Clock::now();
                     for (int worker_id = 0; worker_id < worker_count; ++worker_id) {
                         workers.emplace_back([&, worker_id]() {
                             rbf::OracleValidationConfig local_validation = validation;
                             rbf::lect_database::OnlineEnvelopeCacheTree local_cache(forest.database(), {});
                             rbf::DatabaseBoxOracle local_oracle(forest.robot(),
                                                                 local_cache,
                                                                 rbf::Scene(obstacles),
                                                                 forest.config().endpoint_source,
                                                                 forest.config().envelope_type,
                                                                 local_validation,
                                                                 nullptr,
                                                                 nullptr);
                             rbf::FindFreeBoxService local_ffb(local_oracle);
                             rbf::StageContext local_context = rbf::StageContext::serial();
                             rbf::FindFreeBoxOptions local_options = options;
                             local_options.split_unknown_leaf = false;
                             local_options.split_reserved_leaf = false;
                             while (true) {
                                 const std::size_t item = next.fetch_add(1);
                                 if (item >= candidate_indices.size()) {
                                     break;
                                 }
                                 const std::size_t sample_index = candidate_indices[item];
                                 ParallelCoverResult out;
                                 out.ran = true;
                                 out.sample_index = sample_index;
                                 out.ffb = local_ffb.find(samples[sample_index],
                                                          local_context,
                                                          local_options);
                                 parallel_results[item] = std::move(out);
                             }
                             worker_counters[static_cast<std::size_t>(worker_id)] =
                                 local_oracle.counters();
                         });
                     }
                     for (auto& worker : workers) {
                         worker.join();
                     }
                     const double parallel_ms =
                         std::chrono::duration<double, std::milli>(Clock::now() - parallel_start).count();
                     ffb_calls = static_cast<int>(candidate_indices.size());
                     int pass_found = 0;
                     for (const auto& item : parallel_results) {
                         if (!item.ran) {
                             continue;
                         }
                         const auto& result = item.ffb;
                         const auto& seed = samples[item.sample_index];
                         if (result.found &&
                             intervals_contain_point_pybind(result.intervals,
                                                            seed,
                                                            coverage_tolerance)) {
                             const std::size_t new_box_index = boxes.size();
                             boxes.push_back(result.intervals);
                             found += 1;
                             pass_found += 1;
                             mark_covered(new_box_index);
                             continue;
                         }
                         if (result.found) {
                             rejected_not_containing_seed += 1;
                         }
                         const int fail_code = result.found ? -1 : result.fail_code;
                         if (fail_code >= 0 && fail_code < static_cast<int>(fail_counts.size())) {
                             fail_counts[static_cast<std::size_t>(fail_code)] += 1;
                         }
                         if (failures.size() < static_cast<py::ssize_t>(std::max(0, max_failure_records))) {
                             py::dict failure;
                             failure["sample_index"] = static_cast<int>(item.sample_index);
                             failure["sample_step"] = final_sample_step;
                             failure["sample"] = vector_to_list(seed);
                             failure["found"] = result.found;
                             failure["fail_code"] = result.fail_code;
                             failure["hit_unknown_depth_cap"] = result.hit_unknown_depth_cap;
                             failure["hit_reserved_depth_cap"] = result.hit_reserved_depth_cap;
                             failure["seed_collision"] = result.seed_collision;
                             failure["deadline_reached"] = result.deadline_reached;
                             failure["intervals"] = interval_pairs_to_python(result.intervals);
                             failures.append(std::move(failure));
                         }
                     }
                     py::dict pass_summary;
                     pass_summary["sample_step"] = final_sample_step;
                     pass_summary["sample_count"] = static_cast<int>(samples.size());
                     pass_summary["skipped"] =
                         static_cast<int>(samples.size()) - static_cast<int>(candidate_indices.size());
                     pass_summary["ffb_calls"] = static_cast<int>(candidate_indices.size());
                     pass_summary["ffb_found"] = pass_found;
                     pass_summary["presplit_ms"] = presplit_ms;
                     pass_summary["parallel_ms"] = parallel_ms;
                     pass_summary["ms"] = presplit_ms + parallel_ms;
                     pass_summary["parallel_workers"] = worker_count;
                     pass_summaries.append(std::move(pass_summary));
                     result_counters_override = oracle.counters();
                     for (const auto& counters : worker_counters) {
                         result_counters_override.node_validations += counters.node_validations;
                         result_counters_override.interval_validations += counters.interval_validations;
                         result_counters_override.certified_free += counters.certified_free;
                         result_counters_override.certified_occupied += counters.certified_occupied;
                         result_counters_override.provisional_free += counters.provisional_free;
                         result_counters_override.collision_possible += counters.collision_possible;
                         result_counters_override.materializations += counters.materializations;
                         result_counters_override.materialization_endpoint_time_us += counters.materialization_endpoint_time_us;
                         result_counters_override.materialization_envelope_time_us += counters.materialization_envelope_time_us;
                         result_counters_override.validate_node_total_time_us += counters.validate_node_total_time_us;
                         result_counters_override.materialization_external_exact_hits += counters.materialization_external_exact_hits;
                         result_counters_override.materialization_external_exact_misses += counters.materialization_external_exact_misses;
                         result_counters_override.interval_replay_compatibility_checks += counters.interval_replay_compatibility_checks;
                         result_counters_override.interval_replay_compatible += counters.interval_replay_compatible;
                         result_counters_override.interval_replay_incompatible += counters.interval_replay_incompatible;
                         result_counters_override.interval_replay_direct_exact_hits += counters.interval_replay_direct_exact_hits;
                         result_counters_override.interval_replay_key_only_blocked += counters.interval_replay_key_only_blocked;
                         result_counters_override.canonical_frame_invalid += counters.canonical_frame_invalid;
                         result_counters_override.canonical_reflected_seed_misses += counters.canonical_reflected_seed_misses;
                     }
                     use_counter_override = true;
                 } else {
                 for (double pass_step : pass_steps) {
                     const auto pass_start = Clock::now();
                     const std::vector<Eigen::VectorXd> pass_samples =
                         densify_path_pybind(waypoints, pass_step);
                     const bool pass_uses_final_samples =
                         pass_samples.size() == samples.size() &&
                         std::abs(pass_step - final_sample_step) <=
                             1e-12 * std::max(1.0, final_sample_step);
                     int pass_calls = 0;
                     int pass_found = 0;
                     int pass_skipped = 0;
                     for (std::size_t sample_index = 0; sample_index < pass_samples.size(); ++sample_index) {
                         if ((pass_uses_final_samples && covered[sample_index]) ||
                             (!pass_uses_final_samples && point_covered_by_boxes(pass_samples[sample_index]))) {
                             pass_skipped += 1;
                             continue;
                         }
                         if (max_ffb_calls >= 0 && ffb_calls >= max_ffb_calls) {
                             break;
                         }
                         const rbf::FindFreeBoxResult result =
                             ffb.find(pass_samples[sample_index], context, options);
                         ffb_calls += 1;
                         pass_calls += 1;
                         if (result.found &&
                             intervals_contain_point_pybind(result.intervals,
                                                            pass_samples[sample_index],
                                                            coverage_tolerance)) {
                             const std::size_t new_box_index = boxes.size();
                             boxes.push_back(result.intervals);
                             found += 1;
                             pass_found += 1;
                             mark_covered(new_box_index);
                             continue;
                         }
                         if (result.found) {
                             rejected_not_containing_seed += 1;
                         }
                         const int fail_code = result.found ? -1 : result.fail_code;
                         if (fail_code >= 0 && fail_code < static_cast<int>(fail_counts.size())) {
                             fail_counts[static_cast<std::size_t>(fail_code)] += 1;
                         }
                         if (failures.size() < static_cast<py::ssize_t>(std::max(0, max_failure_records))) {
                             py::dict failure;
                             failure["sample_index"] = static_cast<int>(sample_index);
                             failure["sample_step"] = pass_step;
                             failure["sample"] = vector_to_list(pass_samples[sample_index]);
                             failure["found"] = result.found;
                             failure["fail_code"] = result.fail_code;
                             failure["hit_unknown_depth_cap"] = result.hit_unknown_depth_cap;
                             failure["hit_reserved_depth_cap"] = result.hit_reserved_depth_cap;
                             failure["seed_collision"] = result.seed_collision;
                             failure["deadline_reached"] = result.deadline_reached;
                             failure["intervals"] = interval_pairs_to_python(result.intervals);
                             failures.append(std::move(failure));
                         }
                     }
                     py::dict pass_summary;
                     pass_summary["sample_step"] = pass_step;
                     pass_summary["sample_count"] = static_cast<int>(pass_samples.size());
                     pass_summary["skipped"] = pass_skipped;
                     pass_summary["ffb_calls"] = pass_calls;
                     pass_summary["ffb_found"] = pass_found;
                     pass_summary["ms"] =
                         std::chrono::duration<double, std::milli>(Clock::now() - pass_start).count();
                     pass_summaries.append(std::move(pass_summary));
                     if (max_ffb_calls >= 0 && ffb_calls >= max_ffb_calls) {
                         break;
                     }
                 }
                 }

                 auto sample_cover_layers = [&]() {
                     return sample_layers;
                 };
                 auto same_box = [](const std::vector<rbf::Interval>& lhs,
                                    const std::vector<rbf::Interval>& rhs) {
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
                 auto duplicate_box = [&](const std::vector<rbf::Interval>& candidate) {
                     for (const auto& existing : boxes) {
                         if (same_box(existing, candidate)) {
                             return true;
                         }
                     }
                     return false;
                 };
                 auto transition_connected_local =
                     [&](int transition,
                         const std::vector<std::vector<int>>& layers,
                         const std::vector<int>& bridge_indices) {
                         if (transition < 0 ||
                             transition + 1 >= static_cast<int>(layers.size())) {
                             return false;
                         }
                         const auto& left_layer = layers[static_cast<std::size_t>(transition)];
                         const auto& right_layer = layers[static_cast<std::size_t>(transition + 1)];
                         if (left_layer.empty() || right_layer.empty()) {
                             return false;
                         }
                         std::vector<int> nodes;
                         nodes.reserve(left_layer.size() + right_layer.size() + bridge_indices.size());
                         nodes.insert(nodes.end(), left_layer.begin(), left_layer.end());
                         nodes.insert(nodes.end(), right_layer.begin(), right_layer.end());
                         nodes.insert(nodes.end(), bridge_indices.begin(), bridge_indices.end());
                         std::sort(nodes.begin(), nodes.end());
                         nodes.erase(std::unique(nodes.begin(), nodes.end()), nodes.end());
                         std::vector<int> parent(nodes.size());
                         for (std::size_t index = 0; index < parent.size(); ++index) {
                             parent[index] = static_cast<int>(index);
                         }
                         auto find_local = [&](int value) {
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
                         };
                         auto unite_local = [&](int lhs, int rhs) {
                             const int left = find_local(lhs);
                             const int right = find_local(rhs);
                             if (left != right) {
                                 parent[static_cast<std::size_t>(right)] = left;
                             }
                         };
                         for (std::size_t i = 0; i < nodes.size(); ++i) {
                             for (std::size_t j = i + 1; j < nodes.size(); ++j) {
                                 if (interval_boxes_connected_pybind(boxes[static_cast<std::size_t>(nodes[i])],
                                                                     boxes[static_cast<std::size_t>(nodes[j])],
                                                                     coverage_tolerance)) {
                                     unite_local(static_cast<int>(i), static_cast<int>(j));
                                 }
                             }
                         }
                         for (int lhs_box : left_layer) {
                             const auto lhs_it = std::lower_bound(nodes.begin(), nodes.end(), lhs_box);
                             if (lhs_it == nodes.end() || *lhs_it != lhs_box) {
                                 continue;
                             }
                             const int lhs_local =
                                 static_cast<int>(std::distance(nodes.begin(), lhs_it));
                             const int lhs_root = find_local(lhs_local);
                             for (int rhs_box : right_layer) {
                                 const auto rhs_it = std::lower_bound(nodes.begin(), nodes.end(), rhs_box);
                                 if (rhs_it == nodes.end() || *rhs_it != rhs_box) {
                                     continue;
                                 }
                                 const int rhs_local =
                                     static_cast<int>(std::distance(nodes.begin(), rhs_it));
                                 if (lhs_root == find_local(rhs_local)) {
                                     return true;
                                 }
                             }
                         }
                         return false;
                     };
                 auto bad_transitions_for =
                     [&](const std::vector<std::vector<int>>& layers,
                         const std::vector<std::vector<int>>& transition_bridges) {
                         std::vector<int> bad;
                         if (layers.empty()) {
                             return bad;
                         }
                         for (std::size_t sample_index = 0; sample_index + 1 < layers.size(); ++sample_index) {
                             const std::vector<int> empty_bridges;
                             const auto& bridges =
                                 sample_index < transition_bridges.size()
                                     ? transition_bridges[sample_index]
                                     : empty_bridges;
                             if (!transition_connected_local(static_cast<int>(sample_index),
                                                             layers,
                                                             bridges)) {
                                 bad.push_back(static_cast<int>(sample_index));
                             }
                         }
                         return bad;
                     };
                 auto transition_length_sum = [&](const std::vector<int>& transitions) {
                     double total = 0.0;
                     for (int transition : transitions) {
                         if (transition < 0 ||
                             transition + 1 >= static_cast<int>(samples.size())) {
                             continue;
                         }
                         total += (samples[static_cast<std::size_t>(transition + 1)] -
                                   samples[static_cast<std::size_t>(transition)])
                                      .norm();
                     }
                     return total;
                 };
                 auto direct_bad_transitions = [&]() {
                     std::vector<int> bad;
                     const auto layers = sample_cover_layers();
                     for (std::size_t sample_index = 0; sample_index + 1 < layers.size(); ++sample_index) {
                         bool ok = false;
                         for (int lhs : layers[sample_index]) {
                             for (int rhs : layers[sample_index + 1]) {
                                 if (interval_boxes_connected_pybind(
                                         boxes[static_cast<std::size_t>(lhs)],
                                         boxes[static_cast<std::size_t>(rhs)],
                                         coverage_tolerance)) {
                                     ok = true;
                                     break;
                                 }
                             }
                             if (ok) {
                                 break;
                             }
                         }
                         if (!ok) {
                             bad.push_back(static_cast<int>(sample_index));
                         }
                     }
                     return bad;
                 };
                 if (repair_corridor_adjacency && samples.size() >= 2) {
                     const auto repair_start = Clock::now();
                     const int max_rounds = std::max(0, repair_rounds);
                     const int subdivisions = std::max(1, repair_segment_subdivisions);
                     auto layers = sample_cover_layers();
                     struct RepairDsu {
                         std::vector<int> parent;
                         explicit RepairDsu(std::size_t count = 0) : parent(count) {
                             for (std::size_t index = 0; index < parent.size(); ++index) {
                                 parent[index] = static_cast<int>(index);
                             }
                         }
                         int add_node() {
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
                     RepairDsu repair_dsu(boxes.size());
                     auto transition_connected_dsu = [&](int transition) {
                         if (transition < 0 ||
                             transition + 1 >= static_cast<int>(layers.size())) {
                             return false;
                         }
                         const auto& left_layer = layers[static_cast<std::size_t>(transition)];
                         const auto& right_layer = layers[static_cast<std::size_t>(transition + 1)];
                         if (left_layer.empty() || right_layer.empty()) {
                             return false;
                         }
                         for (int lhs : left_layer) {
                             const int root = repair_dsu.find(lhs);
                             for (int rhs : right_layer) {
                                 if (root == repair_dsu.find(rhs)) {
                                     return true;
                                 }
                             }
                         }
                         return false;
                     };
                     auto bad_transitions_dsu = [&]() {
                         std::vector<int> bad;
                         for (std::size_t sample_index = 0; sample_index + 1 < layers.size(); ++sample_index) {
                             if (!transition_connected_dsu(static_cast<int>(sample_index))) {
                                 bad.push_back(static_cast<int>(sample_index));
                             }
                         }
                         return bad;
                     };
                     auto initialize_corridor_dsu = [&]() {
                         for (auto& layer : layers) {
                             if (layer.empty()) {
                                 continue;
                             }
                             const int root_box = layer.front();
                             for (int box_index : layer) {
                                 repair_dsu.unite(root_box, box_index);
                             }
                         }
                         for (std::size_t sample_index = 0; sample_index + 1 < layers.size(); ++sample_index) {
                             for (int lhs : layers[sample_index]) {
                                 for (int rhs : layers[sample_index + 1]) {
                                     if (interval_boxes_connected_pybind(
                                             boxes[static_cast<std::size_t>(lhs)],
                                             boxes[static_cast<std::size_t>(rhs)],
                                             coverage_tolerance)) {
                                         repair_dsu.unite(lhs, rhs);
                                     }
                                 }
                             }
                         }
                     };
                     initialize_corridor_dsu();
                     std::vector<int> repair_box_indices;
                     auto assimilate_repair_box = [&](int new_box_index, int transition) {
                         for (std::size_t sample_index = 0; sample_index < samples.size(); ++sample_index) {
                             if (!intervals_contain_point_pybind(
                                     boxes[static_cast<std::size_t>(new_box_index)],
                                     samples[sample_index],
                                     coverage_tolerance)) {
                                 continue;
                             }
                             auto& layer = layers[sample_index];
                             if (!layer.empty()) {
                                 repair_dsu.unite(new_box_index, layer.front());
                             }
                             layer.push_back(new_box_index);
                         }
                         std::vector<int> candidates;
                         auto add_layer_candidates = [&](int layer_index) {
                             if (layer_index < 0 || layer_index >= static_cast<int>(layers.size())) {
                                 return;
                             }
                             const auto& layer = layers[static_cast<std::size_t>(layer_index)];
                             candidates.insert(candidates.end(), layer.begin(), layer.end());
                         };
                         add_layer_candidates(transition - 1);
                         add_layer_candidates(transition);
                         add_layer_candidates(transition + 1);
                         add_layer_candidates(transition + 2);
                         candidates.insert(candidates.end(), repair_box_indices.begin(), repair_box_indices.end());
                         std::sort(candidates.begin(), candidates.end());
                         candidates.erase(std::unique(candidates.begin(), candidates.end()), candidates.end());
                         for (int candidate : candidates) {
                             if (candidate == new_box_index) {
                                 continue;
                             }
                             if (interval_boxes_connected_pybind(
                                     boxes[static_cast<std::size_t>(new_box_index)],
                                     boxes[static_cast<std::size_t>(candidate)],
                                     coverage_tolerance)) {
                                 repair_dsu.unite(new_box_index, candidate);
                             }
                         }
                     };
                     std::vector<double> repair_fractions;
                     repair_fractions.reserve(static_cast<std::size_t>(std::max(0, subdivisions - 1)));
                     for (int item = 1; item < subdivisions; ++item) {
                         repair_fractions.push_back(
                             static_cast<double>(item) / static_cast<double>(subdivisions));
                     }
                     std::stable_sort(repair_fractions.begin(),
                                      repair_fractions.end(),
                                      [](double lhs, double rhs) {
                                          return std::abs(lhs - 0.5) < std::abs(rhs - 0.5);
                                      });
                     for (int round = 0; round < max_rounds; ++round) {
                         const auto bad = bad_transitions_dsu();
                         if (round == 0) {
                             repair_bad_transitions_initial = static_cast<int>(bad.size());
                             repair_bad_transition_length_initial = transition_length_sum(bad);
                         }
                         if (bad.empty()) {
                             repair_bad_transitions_final = 0;
                             repair_bad_transition_length_final = 0.0;
                             break;
                         }
                         repair_bad_transitions_final = static_cast<int>(bad.size());
                         repair_bad_transition_length_final = transition_length_sum(bad);
                         int round_added = 0;
                         for (int transition : bad) {
                             if (transition < 0 ||
                                 transition + 1 >= static_cast<int>(samples.size())) {
                                 continue;
                             }
                             const Eigen::VectorXd& a = samples[static_cast<std::size_t>(transition)];
                             const Eigen::VectorXd& b = samples[static_cast<std::size_t>(transition + 1)];
                             if (transition_connected_dsu(transition)) {
                                 continue;
                             }
                             for (double u : repair_fractions) {
                                 const Eigen::VectorXd seed = (1.0 - u) * a + u * b;
                                 const rbf::FindFreeBoxResult result = ffb.find(seed, context, options);
                                 repair_calls += 1;
                                 if (!result.found ||
                                     !intervals_contain_point_pybind(result.intervals,
                                                                     seed,
                                                                     coverage_tolerance)) {
                                     continue;
                                 }
                                 if (duplicate_box(result.intervals)) {
                                     continue;
                                 }
                                 const int new_box_index = static_cast<int>(boxes.size());
                                 boxes.push_back(result.intervals);
                                 repair_dsu.add_node();
                                 assimilate_repair_box(new_box_index, transition);
                                 repair_box_indices.push_back(new_box_index);
                                 repair_added += 1;
                                 round_added += 1;
                                 mark_covered(static_cast<std::size_t>(new_box_index));
                                 if (transition_connected_dsu(transition)) {
                                     break;
                                 }
                             }
                         }
                         mark_covered(0);
                         if (round_added == 0) {
                             break;
                         }
                     }
                     const auto final_bad = bad_transitions_dsu();
                     repair_bad_transitions_final = static_cast<int>(final_bad.size());
                     repair_bad_transition_length_final = transition_length_sum(final_bad);
                     repair_ms =
                         std::chrono::duration<double, std::milli>(Clock::now() - repair_start).count();
                 } else if (samples.size() >= 2) {
                     const auto bad = direct_bad_transitions();
                     repair_bad_transitions_initial = static_cast<int>(bad.size());
                     repair_bad_transitions_final = static_cast<int>(bad.size());
                     repair_bad_transition_length_initial = transition_length_sum(bad);
                     repair_bad_transition_length_final = repair_bad_transition_length_initial;
                 }

                 int covered_count = 0;
                 py::list uncovered_indices;
                 for (std::size_t sample_index = 0; sample_index < samples.size(); ++sample_index) {
                     if (covered[sample_index]) {
                         covered_count += 1;
                     } else if (uncovered_indices.size() < static_cast<py::ssize_t>(std::max(0, max_failure_records))) {
                         uncovered_indices.append(static_cast<int>(sample_index));
                     }
                 }
                 py::list out_boxes;
                 for (const auto& intervals : boxes) {
                     out_boxes.append(interval_pairs_to_python(intervals));
                 }
                 py::list fail_counts_py;
                 for (int count : fail_counts) {
                     fail_counts_py.append(count);
                 }
                 py::dict result;
                 result["sample_count"] = static_cast<int>(samples.size());
                 result["covered_samples"] = covered_count;
                 result["uncovered_samples"] = static_cast<int>(samples.size()) - covered_count;
                 result["coverage"] = samples.empty()
                     ? 1.0
                     : static_cast<double>(covered_count) / static_cast<double>(samples.size());
                 result["initial_box_count"] = initial_box_count;
                 result["added_box_count"] = static_cast<int>(boxes.size()) - initial_box_count;
                 result["box_count"] = static_cast<int>(boxes.size());
                 result["ffb_calls"] = ffb_calls;
                 result["ffb_found"] = found;
                 result["pass_summaries"] = pass_summaries;
                 result["final_sample_step"] = final_sample_step;
                 result["presplit_ms"] = presplit_ms;
                 result["parallel_workers"] = parallel_workers;
                 result["repair_corridor_adjacency"] = repair_corridor_adjacency;
                 result["repair_ms"] = repair_ms;
                 result["repair_calls"] = repair_calls;
                 result["repair_added"] = repair_added;
                 result["repair_bad_transitions_initial"] = repair_bad_transitions_initial;
                 result["repair_bad_transitions_final"] = repair_bad_transitions_final;
                 result["repair_bad_transition_length_initial"] = repair_bad_transition_length_initial;
                 result["repair_bad_transition_length_final"] = repair_bad_transition_length_final;
                 result["repair_bad_transition_fraction_final"] =
                     path_length > 1e-12 ? repair_bad_transition_length_final / path_length : 0.0;
                 result["rejected_not_containing_seed"] = rejected_not_containing_seed;
                 result["fail_counts"] = fail_counts_py;
                 result["failures"] = failures;
                 result["uncovered_indices"] = uncovered_indices;
                 result["boxes"] = out_boxes;
                 result["external_source_enabled"] = external_source != nullptr;
                 result["external_direct_database_enabled"] = direct_external_database != nullptr;
                 result["counters"] = oracle_counters_to_python(
                     use_counter_override ? result_counters_override : oracle.counters());
                 result["total_ms"] =
                     std::chrono::duration<double, std::milli>(Clock::now() - start).count();
                 return result;
             },
             py::arg("waypoint_path"),
             py::arg("obstacles"),
             py::arg("options") = rbf::FindFreeBoxOptions{},
             py::arg("sample_step") = 0.01,
             py::arg("max_ffb_calls") = -1,
             py::arg("coverage_tolerance") = 1e-9,
             py::arg("include_existing_boxes") = true,
             py::arg("disable_caches") = false,
             py::arg("max_failure_records") = 32,
             py::arg("refine_sample_steps") = std::vector<double>{},
             py::arg("parallel_workers") = 1,
             py::arg("repair_corridor_adjacency") = false,
             py::arg("repair_rounds") = 2,
             py::arg("repair_segment_subdivisions") = 8)
        .def("boxes", [](const rbf::RBFPlanningForest& forest) { return forest.boxes(); })
        .def("raw_boxes", [](const rbf::RBFPlanningForest& forest) { return forest.raw_boxes(); })
        .def("audit_robot", &rbf::RBFPlanningForest::audit_robot, py::return_value_policy::reference_internal)
        .def("adjacency", [](const rbf::RBFPlanningForest& forest) { return forest.adjacency(); })
        .def("segment_edges", [](const rbf::RBFPlanningForest& forest) { return forest.segment_edges(); })
        .def("last_build_profile", &rbf::RBFPlanningForest::last_build_profile, py::return_value_policy::reference_internal);

    module.def("path_length", [](const std::vector<std::vector<double>>& path) {
        return rbf::path_length(eigen_vectors_from_lists(path));
    });

    module.def("rrt_connect_path",
        [](const rbf::Robot& robot,
           const std::vector<rbf::Obstacle>& obstacles,
           const std::vector<double>& start,
           const std::vector<double>& goal,
           const rbf::RRTConnectConfig& config,
           int seed) {
            rbf::CollisionChecker checker(robot, rbf::Scene(obstacles));
            auto path = rbf::rrt_connect(eigen_vector_from_list(start),
                                         eigen_vector_from_list(goal),
                                         checker,
                                         robot,
                                         config,
                                         seed);
            std::vector<std::vector<double>> result;
            result.reserve(path.size());
            for (const auto& waypoint : path) {
                result.emplace_back(waypoint.data(), waypoint.data() + waypoint.size());
            }
            return result;
        },
        py::arg("robot"),
        py::arg("obstacles"),
        py::arg("start"),
        py::arg("goal"),
        py::arg("config") = rbf::RRTConnectConfig{},
        py::arg("seed") = 42);

    module.def("ompl_rrt_connect_path",
        [](const rbf::Robot& robot,
           const std::vector<rbf::Obstacle>& obstacles,
           const std::vector<double>& start,
           const std::vector<double>& goal,
           double timeout_ms,
           double range,
           double segment_step,
           double simplify_time_s,
           int seed) {
            namespace ob = ompl::base;
            namespace og = ompl::geometric;
            ompl::msg::setLogLevel(ompl::msg::LOG_ERROR);
            py::dict result;
            const int dimension = robot.n_joints();
            if (static_cast<int>(start.size()) != dimension || static_cast<int>(goal.size()) != dimension) {
                throw std::invalid_argument("start/goal dimension must match robot.n_joints()");
            }
            const auto t0 = std::chrono::steady_clock::now();
            auto elapsed_s = [&]() {
                return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            };
            auto checker = std::make_shared<rbf::CollisionChecker>(robot, rbf::Scene(obstacles));
            const Eigen::VectorXd q_start_vec = eigen_vector_from_list(start);
            const Eigen::VectorXd q_goal_vec = eigen_vector_from_list(goal);
            if (checker->check_config(q_start_vec) || checker->check_config(q_goal_vec)) {
                const double solve_s = elapsed_s();
                result["ok"] = false;
                result["reason"] = "endpoint_collision";
                result["status"] = "endpoint_collision";
                result["solve_s"] = solve_s;
                result["simplify_s"] = 0.0;
                result["t_s"] = solve_s;
                result["path"] = std::vector<std::vector<double>>{};
                result["nodes"] = 0;
                return result;
            }

            const auto base_seed = normalize_seed(seed);
            auto space = std::make_shared<ob::RealVectorStateSpace>(dimension);
            ob::RealVectorBounds bounds(dimension);
            const auto& limits = robot.joint_limits().limits;
            if (static_cast<int>(limits.size()) != dimension) {
                throw std::invalid_argument("robot joint limits must match robot.n_joints()");
            }
            for (int dim = 0; dim < dimension; ++dim) {
                bounds.setLow(dim, limits[static_cast<std::size_t>(dim)].lo);
                bounds.setHigh(dim, limits[static_cast<std::size_t>(dim)].hi);
            }
            space->setBounds(bounds);
            configure_deterministic_state_sampler(space, mix_seed(base_seed, 0x52525453U));
            og::SimpleSetup setup(space);
            setup.setStateValidityChecker([checker, dimension](const ob::State* state) {
                const auto* vector_state = state->as<ob::RealVectorStateSpace::StateType>();
                Eigen::VectorXd q(dimension);
                for (int dim = 0; dim < dimension; ++dim) {
                    q[dim] = vector_state->values[dim];
                }
                return !checker->check_config(q);
            });
            const double maximum_extent = std::max(space->getMaximumExtent(), 1e-9);
            const double checking_resolution = std::clamp(std::max(segment_step, 1e-6) / maximum_extent, 1e-6, 0.05);
            setup.getSpaceInformation()->setStateValidityCheckingResolution(checking_resolution);

            ob::ScopedState<ob::RealVectorStateSpace> q_start(space), q_goal(space);
            for (int dim = 0; dim < dimension; ++dim) {
                q_start->values[dim] = start[static_cast<std::size_t>(dim)];
                q_goal->values[dim] = goal[static_cast<std::size_t>(dim)];
            }
            setup.setStartAndGoalStates(q_start, q_goal);
            auto planner = std::make_shared<SeededRRTConnect>(setup.getSpaceInformation());
            planner->setLocalSeed(mix_seed(base_seed, 0x52525450U));
            if (range > 0.0) {
                planner->setRange(range);
            }
            setup.setPlanner(planner);

            const double timeout_s = std::max(0.0, timeout_ms) / 1000.0;
            const double ptc_interval_s = std::max(1e-3, std::min(0.05, timeout_s / 20.0));
            ob::PlannerStatus status = setup.solve(ob::timedPlannerTerminationCondition(timeout_s, ptc_interval_s));
            const double solve_s = elapsed_s();
            const bool exact_solution = status == ob::PlannerStatus::EXACT_SOLUTION;
            bool ok = exact_solution;
            double simplify_s = 0.0;
            if (ok && simplify_time_s > 0.0) {
                const auto simplify_start = std::chrono::steady_clock::now();
                setup.simplifySolution(simplify_time_s);
                simplify_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - simplify_start).count();
            }
            std::vector<std::vector<double>> path;
            if (ok && setup.haveSolutionPath()) {
                auto& solution_path = setup.getSolutionPath();
                path.reserve(solution_path.getStateCount());
                for (std::size_t index = 0; index < solution_path.getStateCount(); ++index) {
                    path.push_back(list_from_ompl_state(solution_path.getState(index), dimension));
                }
            } else {
                ok = false;
            }
            ob::PlannerData planner_data(setup.getSpaceInformation());
            planner->getPlannerData(planner_data);
            result["ok"] = ok;
            result["reason"] = ok ? "connected" : std::string(status.asString());
            result["status"] = std::string(status.asString());
            result["exact_solution"] = exact_solution;
            result["solve_s"] = solve_s;
            result["simplify_s"] = simplify_s;
            result["t_s"] = solve_s + simplify_s;
            result["path"] = path;
            result["nodes"] = static_cast<int>(planner_data.numVertices());
            result["checking_resolution"] = checking_resolution;
            result["planner"] = "OMPL_RRTConnect";
            return result;
        },
        py::arg("robot"),
        py::arg("obstacles"),
        py::arg("start"),
        py::arg("goal"),
        py::arg("timeout_ms") = 1000.0,
        py::arg("range") = 0.35,
        py::arg("segment_step") = 0.05,
        py::arg("simplify_time_s") = 0.05,
        py::arg("seed") = 42);

    module.def("ompl_prm_multiquery",
        [](const rbf::Robot& robot,
           const std::vector<rbf::Obstacle>& obstacles,
           const std::vector<std::vector<double>>& starts,
           const std::vector<std::vector<double>>& goals,
           double build_budget_s,
           double query_budget_s,
           double segment_step,
           double simplify_time_s,
           int seed,
           int max_nearest_neighbors,
           const std::string& planner_kind,
           bool preload_query_endpoints) {
            namespace ob = ompl::base;
            namespace og = ompl::geometric;
            ompl::msg::setLogLevel(ompl::msg::LOG_ERROR);
            const auto base_seed = normalize_seed(seed);
            py::dict result;
            const int dimension = robot.n_joints();
            if (starts.empty() || starts.size() != goals.size()) {
                throw std::invalid_argument("starts/goals must be non-empty and have the same length");
            }
            for (std::size_t index = 0; index < starts.size(); ++index) {
                if (static_cast<int>(starts[index].size()) != dimension || static_cast<int>(goals[index].size()) != dimension) {
                    throw std::invalid_argument("all start/goal dimensions must match robot.n_joints()");
                }
            }
            auto checker = std::make_shared<rbf::CollisionChecker>(robot, rbf::Scene(obstacles));
            auto endpoint_collision = [&](const std::vector<double>& q) {
                return checker->check_config(eigen_vector_from_list(q));
            };
            double checking_resolution = 0.0;
            auto space = make_ompl_space(robot);
            configure_deterministic_state_sampler(space, mix_seed(base_seed, 0x50524D53U));
            auto si = make_ompl_space_information(robot, obstacles, space, segment_step, checking_resolution);
            auto problem = std::make_shared<ob::ProblemDefinition>(si);
            set_problem_query(problem, space, si, starts.front(), goals.front(), dimension);
            std::shared_ptr<og::PRM> planner;
            std::shared_ptr<SeededPRM> seeded_prm;
            std::shared_ptr<SeededPRMstar> seeded_prmstar;
            const bool use_prmstar = planner_kind == "prmstar" || planner_kind == "PRMstar" || planner_kind == "PRM*";
            if (use_prmstar) {
                seeded_prmstar = std::make_shared<SeededPRMstar>(si);
                seeded_prmstar->setLocalSeed(mix_seed(base_seed, 0x50524D50U));
                planner = seeded_prmstar;
            } else {
                seeded_prm = std::make_shared<SeededPRM>(si);
                seeded_prm->setLocalSeed(mix_seed(base_seed, 0x50524D50U));
                planner = seeded_prm;
            }
            if (!use_prmstar && max_nearest_neighbors > 0) {
                planner->setMaxNearestNeighbors(static_cast<unsigned int>(max_nearest_neighbors));
            }
            planner->setProblemDefinition(problem);
            planner->setup();

            const auto build_start = std::chrono::steady_clock::now();
            if (preload_query_endpoints) {
                for (std::size_t index = 0; index < starts.size(); ++index) {
                    ob::ScopedState<ob::RealVectorStateSpace> q_start(space), q_goal(space);
                    for (int dim = 0; dim < dimension; ++dim) {
                        q_start->values[dim] = starts[index][static_cast<std::size_t>(dim)];
                        q_goal->values[dim] = goals[index][static_cast<std::size_t>(dim)];
                    }
                    if (use_prmstar) {
                        seeded_prmstar->addMilestoneFromState(q_start.get());
                        seeded_prmstar->addMilestoneFromState(q_goal.get());
                    } else {
                        seeded_prm->addMilestoneFromState(q_start.get());
                        seeded_prm->addMilestoneFromState(q_goal.get());
                    }
                }
            }
            planner->constructRoadmap(ob::timedPlannerTerminationCondition(std::max(0.0, build_budget_s)));
            const double build_s = ompl_elapsed_s(build_start);

            py::list query_results;
            for (std::size_t index = 0; index < starts.size(); ++index) {
                py::dict row;
                row["index"] = static_cast<int>(index);
                if (endpoint_collision(starts[index]) || endpoint_collision(goals[index])) {
                    row["ok"] = false;
                    row["reason"] = "endpoint_collision";
                    row["status"] = "endpoint_collision";
                    row["solve_s"] = 0.0;
                    row["simplify_s"] = 0.0;
                    row["t_s"] = 0.0;
                    row["path"] = std::vector<std::vector<double>>{};
                    query_results.append(row);
                    continue;
                }
                planner->clearQuery();
                set_problem_query(problem, space, si, starts[index], goals[index], dimension);
                const auto query_start = std::chrono::steady_clock::now();
                ob::ScopedState<ob::RealVectorStateSpace> q_start(space), q_goal(space);
                for (int dim = 0; dim < dimension; ++dim) {
                    q_start->values[dim] = starts[index][static_cast<std::size_t>(dim)];
                    q_goal->values[dim] = goals[index][static_cast<std::size_t>(dim)];
                }
                if (use_prmstar) {
                    seeded_prmstar->addMilestoneFromState(q_start.get());
                    seeded_prmstar->addMilestoneFromState(q_goal.get());
                } else {
                    seeded_prm->addMilestoneFromState(q_start.get());
                    seeded_prm->addMilestoneFromState(q_goal.get());
                }
                ob::PlannerStatus status = planner->solve(ob::timedPlannerTerminationCondition(std::max(0.0, query_budget_s)));
                bool ok = status == ob::PlannerStatus::EXACT_SOLUTION || static_cast<bool>(status);
                const double solve_s = ompl_elapsed_s(query_start);
                double simplify_s = 0.0;
                if (ok && simplify_time_s > 0.0) {
                    auto path_ptr = problem->getSolutionPath();
                    auto geometric_path = std::dynamic_pointer_cast<og::PathGeometric>(path_ptr);
                    if (geometric_path) {
                        const auto simplify_start = std::chrono::steady_clock::now();
                        og::PathSimplifier simplifier(si);
                        simplifier.simplify(*geometric_path, std::max(0.0, simplify_time_s));
                        simplify_s = ompl_elapsed_s(simplify_start);
                    }
                }
                auto path = ok ? path_from_problem_solution(problem, dimension) : std::vector<std::vector<double>>{};
                if (path.size() < 2) {
                    ok = false;
                }
                row["ok"] = ok;
                row["reason"] = ok ? "connected" : std::string(status.asString());
                row["status"] = std::string(status.asString());
                row["solve_s"] = solve_s;
                row["simplify_s"] = simplify_s;
                row["t_s"] = solve_s + simplify_s;
                row["path"] = path;
                query_results.append(row);
            }
            ob::PlannerData planner_data(si);
            planner->getPlannerData(planner_data);
            result["ok"] = true;
            result["planner"] = planner_kind == "prmstar" || planner_kind == "PRMstar" || planner_kind == "PRM*" ? "OMPL_PRMstar" : "OMPL_PRM";
            result["build_s"] = build_s;
            result["nodes"] = static_cast<int>(planner_data.numVertices());
            result["checking_resolution"] = checking_resolution;
            result["preload_query_endpoints"] = preload_query_endpoints;
            result["queries"] = query_results;
            return result;
        },
        py::arg("robot"),
        py::arg("obstacles"),
        py::arg("starts"),
        py::arg("goals"),
        py::arg("build_budget_s") = 10.0,
        py::arg("query_budget_s") = 2.0,
        py::arg("segment_step") = 0.05,
        py::arg("simplify_time_s") = 0.1,
        py::arg("seed") = 42,
        py::arg("max_nearest_neighbors") = 32,
        py::arg("planner_kind") = "prm",
        py::arg("preload_query_endpoints") = false);

    module.def("ompl_prm_multiquery_cumulative",
        [](const rbf::Robot& robot,
           const std::vector<rbf::Obstacle>& obstacles,
           const std::vector<std::vector<double>>& starts,
           const std::vector<std::vector<double>>& goals,
           const std::vector<double>& build_checkpoints_s,
           double query_budget_s,
           double segment_step,
           double simplify_time_s,
           int seed,
           int max_nearest_neighbors,
           const std::string& planner_kind,
           bool preload_query_endpoints,
           int early_stop_success_stall_checkpoints,
           double early_stop_path_rel_tol,
           double unresolved_query_retry_interval_s,
           double solved_query_recheck_interval_s) {
            namespace ob = ompl::base;
            namespace og = ompl::geometric;
            ompl::msg::setLogLevel(ompl::msg::LOG_ERROR);
            const auto base_seed = normalize_seed(seed);
            py::dict result;
            const int dimension = robot.n_joints();
            if (starts.empty() || starts.size() != goals.size()) {
                throw std::invalid_argument("starts/goals must be non-empty and have the same length");
            }
            if (build_checkpoints_s.empty()) {
                throw std::invalid_argument("build_checkpoints_s must be non-empty");
            }
            double previous_checkpoint = 0.0;
            std::vector<double> checkpoints;
            checkpoints.reserve(build_checkpoints_s.size());
            for (double checkpoint : build_checkpoints_s) {
                if (!std::isfinite(checkpoint) || checkpoint <= 0.0) {
                    throw std::invalid_argument("build checkpoints must be finite positive seconds");
                }
                if (checkpoint < previous_checkpoint) {
                    throw std::invalid_argument("build checkpoints must be sorted nondecreasing");
                }
                checkpoints.push_back(checkpoint);
                previous_checkpoint = checkpoint;
            }
            for (std::size_t index = 0; index < starts.size(); ++index) {
                if (static_cast<int>(starts[index].size()) != dimension || static_cast<int>(goals[index].size()) != dimension) {
                    throw std::invalid_argument("all start/goal dimensions must match robot.n_joints()");
                }
            }
            auto checker = std::make_shared<rbf::CollisionChecker>(robot, rbf::Scene(obstacles));
            auto endpoint_collision = [&](const std::vector<double>& q) {
                return checker->check_config(eigen_vector_from_list(q));
            };
            double checking_resolution = 0.0;
            auto space = make_ompl_space(robot);
            configure_deterministic_state_sampler(space, mix_seed(base_seed, 0x50524D54U));
            auto si = make_ompl_space_information(robot, obstacles, space, segment_step, checking_resolution);
            auto problem = std::make_shared<ob::ProblemDefinition>(si);
            set_problem_query(problem, space, si, starts.front(), goals.front(), dimension);
            std::shared_ptr<og::PRM> planner;
            std::shared_ptr<SeededPRM> seeded_prm;
            std::shared_ptr<SeededPRMstar> seeded_prmstar;
            const bool use_prmstar = planner_kind == "prmstar" || planner_kind == "PRMstar" || planner_kind == "PRM*";
            if (use_prmstar) {
                seeded_prmstar = std::make_shared<SeededPRMstar>(si);
                seeded_prmstar->setLocalSeed(mix_seed(base_seed, 0x50524D51U));
                planner = seeded_prmstar;
            } else {
                seeded_prm = std::make_shared<SeededPRM>(si);
                seeded_prm->setLocalSeed(mix_seed(base_seed, 0x50524D51U));
                planner = seeded_prm;
            }
            if (!use_prmstar && max_nearest_neighbors > 0) {
                planner->setMaxNearestNeighbors(static_cast<unsigned int>(max_nearest_neighbors));
            }
            planner->setProblemDefinition(problem);
            planner->setup();

            auto add_query_milestones = [&](std::size_t index) {
                ob::ScopedState<ob::RealVectorStateSpace> q_start(space), q_goal(space);
                for (int dim = 0; dim < dimension; ++dim) {
                    q_start->values[dim] = starts[index][static_cast<std::size_t>(dim)];
                    q_goal->values[dim] = goals[index][static_cast<std::size_t>(dim)];
                }
                if (use_prmstar) {
                    seeded_prmstar->addMilestoneFromState(q_start.get());
                    seeded_prmstar->addMilestoneFromState(q_goal.get());
                } else {
                    seeded_prm->addMilestoneFromState(q_start.get());
                    seeded_prm->addMilestoneFromState(q_goal.get());
                }
            };
            std::vector<bool> milestones_inserted(starts.size(), false);
            if (preload_query_endpoints) {
                for (std::size_t index = 0; index < starts.size(); ++index) {
                    add_query_milestones(index);
                    milestones_inserted[index] = true;
                }
            }

            py::list stages;
            double cumulative_build_only_s = 0.0;
            double last_target_s = 0.0;
            double final_build_s = 0.0;
            int final_nodes = 0;
            std::vector<bool> has_incumbent(starts.size(), false);
            std::vector<double> best_lengths(starts.size(), std::numeric_limits<double>::infinity());
            std::vector<double> last_query_checkpoint_s(starts.size(), -std::numeric_limits<double>::infinity());
            int success_stall_checkpoints = 0;
            bool stopped_early = false;
            auto vector_path_length = [](const std::vector<std::vector<double>>& path) {
                if (path.size() < 2) {
                    return std::numeric_limits<double>::infinity();
                }
                double total = 0.0;
                for (std::size_t i = 1; i < path.size(); ++i) {
                    const auto& a = path[i - 1];
                    const auto& b = path[i];
                    double squared = 0.0;
                    const std::size_t dims = std::min(a.size(), b.size());
                    for (std::size_t dim = 0; dim < dims; ++dim) {
                        const double delta = b[dim] - a[dim];
                        squared += delta * delta;
                    }
                    total += std::sqrt(squared);
                }
                return total;
            };
            for (std::size_t stage_index = 0; stage_index < checkpoints.size(); ++stage_index) {
                const double target_s = checkpoints[stage_index];
                const double delta_s = std::max(0.0, target_s - last_target_s);
                const auto stage_build_start = std::chrono::steady_clock::now();
                planner->constructRoadmap(ob::timedPlannerTerminationCondition(delta_s));
                const double stage_build_delta_s = ompl_elapsed_s(stage_build_start);
                cumulative_build_only_s += stage_build_delta_s;
                const double cumulative_build_s = cumulative_build_only_s;
                last_target_s = target_s;

                ob::PlannerData planner_data(si);
                planner->getPlannerData(planner_data);
                final_build_s = cumulative_build_s;
                final_nodes = static_cast<int>(planner_data.numVertices());

                py::list query_results;
                bool stage_significant_improvement = false;
                bool stage_had_query = false;
                bool stage_had_incumbent_recheck = false;
                int stage_queries_executed = 0;
                for (std::size_t index = 0; index < starts.size(); ++index) {
                    py::dict row;
                    row["index"] = static_cast<int>(index);
                    if (endpoint_collision(starts[index]) || endpoint_collision(goals[index])) {
                        row["ok"] = false;
                        row["reason"] = "endpoint_collision";
                        row["status"] = "endpoint_collision";
                        row["solve_s"] = 0.0;
                        row["simplify_s"] = 0.0;
                        row["t_s"] = 0.0;
                        row["path"] = std::vector<std::vector<double>>{};
                        query_results.append(row);
                        continue;
                    }
                    const bool is_final_checkpoint = (stage_index + 1 == checkpoints.size());
                    const double retry_interval = has_incumbent[index]
                        ? std::max(0.0, solved_query_recheck_interval_s)
                        : std::max(0.0, unresolved_query_retry_interval_s);
                    const bool query_due =
                        is_final_checkpoint ||
                        !std::isfinite(last_query_checkpoint_s[index]) ||
                        target_s - last_query_checkpoint_s[index] >= retry_interval - 1e-12;
                    if (!query_due) {
                        row["ok"] = false;
                        row["reason"] = "skipped_between_trace_queries";
                        row["status"] = "skipped_between_trace_queries";
                        row["solve_s"] = 0.0;
                        row["simplify_s"] = 0.0;
                        row["t_s"] = 0.0;
                        row["path"] = std::vector<std::vector<double>>{};
                        query_results.append(row);
                        continue;
                    }
                    stage_had_query = true;
                    if (has_incumbent[index]) {
                        stage_had_incumbent_recheck = true;
                    }
                    ++stage_queries_executed;
                    last_query_checkpoint_s[index] = target_s;
                    planner->clearQuery();
                    set_problem_query(problem, space, si, starts[index], goals[index], dimension);
                    if (!milestones_inserted[index]) {
                        add_query_milestones(index);
                        milestones_inserted[index] = true;
                    }
                    const auto query_start = std::chrono::steady_clock::now();
                    ob::PlannerStatus status = planner->solve(ob::timedPlannerTerminationCondition(std::max(0.0, query_budget_s)));
                    bool ok = status == ob::PlannerStatus::EXACT_SOLUTION || static_cast<bool>(status);
                    const double solve_s = ompl_elapsed_s(query_start);
                    double simplify_s = 0.0;
                    if (ok && simplify_time_s > 0.0) {
                        auto path_ptr = problem->getSolutionPath();
                        auto geometric_path = std::dynamic_pointer_cast<og::PathGeometric>(path_ptr);
                        if (geometric_path) {
                            const auto simplify_start = std::chrono::steady_clock::now();
                            og::PathSimplifier simplifier(si);
                            simplifier.simplify(*geometric_path, std::max(0.0, simplify_time_s));
                            simplify_s = ompl_elapsed_s(simplify_start);
                        }
                    }
                    auto path = ok ? path_from_problem_solution(problem, dimension) : std::vector<std::vector<double>>{};
                    if (path.size() < 2) {
                        ok = false;
                    }
                    if (ok) {
                        const double length = vector_path_length(path);
                        const bool first_solution = !has_incumbent[index];
                        const double previous = best_lengths[index];
                        if (first_solution || length < previous) {
                            if (first_solution ||
                                length < previous * (1.0 - std::max(0.0, early_stop_path_rel_tol))) {
                                stage_significant_improvement = true;
                            }
                            best_lengths[index] = std::min(previous, length);
                            has_incumbent[index] = true;
                        }
                    }
                    row["ok"] = ok;
                    row["reason"] = ok ? "connected" : std::string(status.asString());
                    row["status"] = std::string(status.asString());
                    row["solve_s"] = solve_s;
                    row["simplify_s"] = simplify_s;
                    row["t_s"] = solve_s + simplify_s;
                    row["path"] = path;
                    query_results.append(row);
                }

                py::dict stage;
                stage["stage_index"] = static_cast<int>(stage_index);
                stage["checkpoint_s"] = target_s;
                stage["stage_build_delta_s"] = stage_build_delta_s;
                stage["build_s"] = cumulative_build_s;
                stage["nodes"] = static_cast<int>(planner_data.numVertices());
                stage["queries"] = query_results;
                const bool all_success = std::all_of(
                    has_incumbent.begin(),
                    has_incumbent.end(),
                    [](bool value) { return value; });
                if (all_success) {
                    if (stage_significant_improvement) {
                        success_stall_checkpoints = 0;
                    } else if (stage_had_incumbent_recheck) {
                        ++success_stall_checkpoints;
                    }
                } else {
                    success_stall_checkpoints = 0;
                }
                stage["all_queries_have_incumbent"] = all_success;
                stage["significant_improvement"] = stage_significant_improvement;
                stage["success_stall_checkpoints"] = success_stall_checkpoints;
                stage["queries_executed"] = stage_queries_executed;
                stage["had_query"] = stage_had_query;
                stage["had_incumbent_recheck"] = stage_had_incumbent_recheck;
                stage["early_stop"] = false;
                stages.append(stage);
                if (early_stop_success_stall_checkpoints > 0 &&
                    all_success &&
                    success_stall_checkpoints >= early_stop_success_stall_checkpoints) {
                    stage["early_stop"] = true;
                    stopped_early = true;
                    break;
                }
            }
            result["ok"] = true;
            result["planner"] = use_prmstar ? "OMPL_PRMstar" : "OMPL_PRM";
            result["cumulative"] = true;
            result["build_s"] = final_build_s;
            result["nodes"] = final_nodes;
            result["stopped_early"] = stopped_early;
            result["early_stop_success_stall_checkpoints"] = early_stop_success_stall_checkpoints;
            result["early_stop_path_rel_tol"] = early_stop_path_rel_tol;
            result["unresolved_query_retry_interval_s"] = unresolved_query_retry_interval_s;
            result["solved_query_recheck_interval_s"] = solved_query_recheck_interval_s;
            result["checking_resolution"] = checking_resolution;
            result["preload_query_endpoints"] = preload_query_endpoints;
            result["stages"] = stages;
            return result;
        },
        py::arg("robot"),
        py::arg("obstacles"),
        py::arg("starts"),
        py::arg("goals"),
        py::arg("build_checkpoints_s"),
        py::arg("query_budget_s") = 2.0,
        py::arg("segment_step") = 0.05,
        py::arg("simplify_time_s") = 0.1,
        py::arg("seed") = 42,
        py::arg("max_nearest_neighbors") = 32,
        py::arg("planner_kind") = "prm",
        py::arg("preload_query_endpoints") = false,
        py::arg("early_stop_success_stall_checkpoints") = 0,
        py::arg("early_stop_path_rel_tol") = 0.0,
        py::arg("unresolved_query_retry_interval_s") = 0.0,
        py::arg("solved_query_recheck_interval_s") = 0.0);

    module.def("ompl_simplify_path",
        [](const rbf::Robot& robot,
           const std::vector<rbf::Obstacle>& obstacles,
           const std::vector<std::vector<double>>& waypoints,
           double segment_step,
           double simplify_time_s) {
            namespace ob = ompl::base;
            namespace og = ompl::geometric;

            py::dict result;
            result["ok"] = false;
            result["t_s"] = 0.0;
            result["path"] = std::vector<std::vector<double>>{};

            const int dimension = robot.n_joints();
            if (waypoints.size() < 2) {
                result["reason"] = "path_too_short";
                return result;
            }
            for (const auto& waypoint : waypoints) {
                if (static_cast<int>(waypoint.size()) != dimension) {
                    throw std::invalid_argument("ompl_simplify_path waypoint dimension must match robot.n_joints()");
                }
            }

            auto space = make_ompl_space(robot);
            double checking_resolution = 0.0;
            auto si = make_ompl_space_information(robot, obstacles, space, segment_step, checking_resolution);

            og::PathGeometric geometric_path(si);
            for (const auto& waypoint : waypoints) {
                ob::ScopedState<ob::RealVectorStateSpace> state(space);
                for (int dim = 0; dim < dimension; ++dim) {
                    state->values[dim] = waypoint[static_cast<std::size_t>(dim)];
                }
                geometric_path.append(state.get());
            }

            const auto simplify_start = std::chrono::steady_clock::now();
            if (simplify_time_s > 0.0) {
                og::PathSimplifier simplifier(si);
                simplifier.simplify(geometric_path, std::max(0.0, simplify_time_s));
            }
            result["ok"] = geometric_path.getStateCount() >= 2;
            result["t_s"] = ompl_elapsed_s(simplify_start);
            result["checking_resolution"] = checking_resolution;
            result["path"] = path_from_geometric_path(geometric_path, dimension);
            result["reason"] = (geometric_path.getStateCount() >= 2) ? "simplified" : "simplify_failed";
            return result;
        },
        py::arg("robot"),
        py::arg("obstacles"),
        py::arg("waypoints"),
        py::arg("segment_step"),
        py::arg("simplify_time_s") = 0.05);

    module.def("ompl_lazy_prm_multiquery",
        [](const rbf::Robot& robot,
           const std::vector<rbf::Obstacle>& obstacles,
           const std::vector<std::vector<double>>& starts,
           const std::vector<std::vector<double>>& goals,
           double query_budget_s,
           double segment_step,
           double simplify_time_s,
           int seed,
           int max_nearest_neighbors,
           double range) {
            namespace ob = ompl::base;
            namespace og = ompl::geometric;
            ompl::msg::setLogLevel(ompl::msg::LOG_ERROR);
            const auto base_seed = normalize_seed(seed);
            py::dict result;
            const int dimension = robot.n_joints();
            if (starts.empty() || starts.size() != goals.size()) {
                throw std::invalid_argument("starts/goals must be non-empty and have the same length");
            }
            for (std::size_t index = 0; index < starts.size(); ++index) {
                if (static_cast<int>(starts[index].size()) != dimension || static_cast<int>(goals[index].size()) != dimension) {
                    throw std::invalid_argument("all start/goal dimensions must match robot.n_joints()");
                }
            }
            auto checker = std::make_shared<rbf::CollisionChecker>(robot, rbf::Scene(obstacles));
            auto endpoint_collision = [&](const std::vector<double>& q) {
                return checker->check_config(eigen_vector_from_list(q));
            };
            double checking_resolution = 0.0;
            auto space = make_ompl_space(robot);
            configure_deterministic_state_sampler(space, mix_seed(base_seed, 0x4C50524DU));
            auto si = make_ompl_space_information(robot, obstacles, space, segment_step, checking_resolution);
            auto problem = std::make_shared<ob::ProblemDefinition>(si);
            set_problem_query(problem, space, si, starts.front(), goals.front(), dimension);
            auto planner = std::make_shared<og::LazyPRM>(si);
            if (max_nearest_neighbors > 0) {
                planner->setMaxNearestNeighbors(static_cast<unsigned int>(max_nearest_neighbors));
            }
            if (range > 0.0) {
                planner->setRange(range);
            }
            planner->setProblemDefinition(problem);
            planner->setup();

            const auto total_start = std::chrono::steady_clock::now();
            py::list query_results;
            for (std::size_t index = 0; index < starts.size(); ++index) {
                py::dict row;
                row["index"] = static_cast<int>(index);
                if (endpoint_collision(starts[index]) || endpoint_collision(goals[index])) {
                    row["ok"] = false;
                    row["reason"] = "endpoint_collision";
                    row["status"] = "endpoint_collision";
                    row["solve_s"] = 0.0;
                    row["simplify_s"] = 0.0;
                    row["t_s"] = 0.0;
                    row["path"] = std::vector<std::vector<double>>{};
                    query_results.append(row);
                    continue;
                }
                set_problem_query(problem, space, si, starts[index], goals[index], dimension);
                if (index > 0) {
                    planner->clearQuery();
                }
                planner->setProblemDefinition(problem);
                planner->setup();
                const auto query_start = std::chrono::steady_clock::now();
                ob::PlannerStatus status = planner->solve(ob::timedPlannerTerminationCondition(std::max(0.0, query_budget_s)));
                bool ok = status == ob::PlannerStatus::EXACT_SOLUTION || static_cast<bool>(status);
                const double solve_s = ompl_elapsed_s(query_start);
                double simplify_s = 0.0;
                if (ok && simplify_time_s > 0.0) {
                    auto path_ptr = problem->getSolutionPath();
                    auto geometric_path = std::dynamic_pointer_cast<og::PathGeometric>(path_ptr);
                    if (geometric_path) {
                        const auto simplify_start = std::chrono::steady_clock::now();
                        og::PathSimplifier simplifier(si);
                        simplifier.simplify(*geometric_path, std::max(0.0, simplify_time_s));
                        simplify_s = ompl_elapsed_s(simplify_start);
                    }
                }
                auto path = ok ? path_from_problem_solution(problem, dimension) : std::vector<std::vector<double>>{};
                if (path.size() < 2) {
                    ok = false;
                }
                row["ok"] = ok;
                row["reason"] = ok ? "connected" : std::string(status.asString());
                row["status"] = std::string(status.asString());
                row["solve_s"] = solve_s;
                row["simplify_s"] = simplify_s;
                row["t_s"] = solve_s + simplify_s;
                row["path"] = path;
                query_results.append(row);
            }
            ob::PlannerData planner_data(si);
            planner->getPlannerData(planner_data);
            result["ok"] = true;
            result["planner"] = "OMPL_LazyPRM";
            result["build_s"] = 0.0;
            result["total_s"] = ompl_elapsed_s(total_start);
            result["nodes"] = static_cast<int>(planner_data.numVertices());
            result["checking_resolution"] = checking_resolution;
            result["queries"] = query_results;
            return result;
        },
        py::arg("robot"),
        py::arg("obstacles"),
        py::arg("starts"),
        py::arg("goals"),
        py::arg("query_budget_s") = 2.0,
        py::arg("segment_step") = 0.05,
        py::arg("simplify_time_s") = 0.1,
        py::arg("seed") = 42,
        py::arg("max_nearest_neighbors") = 32,
        py::arg("range") = 0.0);

    module.def("ompl_bitstar_path",
        [](const rbf::Robot& robot,
           const std::vector<rbf::Obstacle>& obstacles,
           const std::vector<double>& start,
           const std::vector<double>& goal,
           double timeout_ms,
           double segment_step,
           double simplify_time_s,
           int seed,
           int samples_per_batch,
           double rewire_factor,
           bool stop_on_solution_improvement,
           int use_k_nearest,
           int pruning,
           double prune_threshold_fraction,
           int delay_rewiring_until_initial_solution,
           int just_in_time_sampling,
           int drop_samples_on_prune,
           int approximate_solutions,
           int strict_queue_ordering,
           int cascading_rewirings,
           double initial_inflation_factor,
           double inflation_scaling_parameter,
           double truncation_scaling_parameter,
           int allowed_failed_sampling_attempts) {
            namespace ob = ompl::base;
            namespace og = ompl::geometric;
            ompl::msg::setLogLevel(ompl::msg::LOG_ERROR);
            const auto base_seed = normalize_seed(seed);
            py::dict result;
            const int dimension = robot.n_joints();
            if (static_cast<int>(start.size()) != dimension || static_cast<int>(goal.size()) != dimension) {
                throw std::invalid_argument("start/goal dimension must match robot.n_joints()");
            }
            const auto t0 = std::chrono::steady_clock::now();
            auto checker = std::make_shared<rbf::CollisionChecker>(robot, rbf::Scene(obstacles));
            if (checker->check_config(eigen_vector_from_list(start)) || checker->check_config(eigen_vector_from_list(goal))) {
                const double solve_s = ompl_elapsed_s(t0);
                result["ok"] = false;
                result["reason"] = "endpoint_collision";
                result["status"] = "endpoint_collision";
                result["solve_s"] = solve_s;
                result["simplify_s"] = 0.0;
                result["t_s"] = solve_s;
                result["path"] = std::vector<std::vector<double>>{};
                result["nodes"] = 0;
                return result;
            }
            auto space = make_ompl_space(robot);
            configure_deterministic_state_sampler(space, mix_seed(base_seed, 0x42495453U));
            double checking_resolution = 0.0;
            og::SimpleSetup setup(space);
            auto si = setup.getSpaceInformation();
            si->setStateValidityChecker([checker, dimension](const ob::State* state) {
                const auto* vector_state = state->as<ob::RealVectorStateSpace::StateType>();
                Eigen::VectorXd q(dimension);
                for (int dim = 0; dim < dimension; ++dim) {
                    q[dim] = vector_state->values[dim];
                }
                return !checker->check_config(q);
            });
            const double maximum_extent = std::max(space->getMaximumExtent(), 1e-9);
            checking_resolution = std::clamp(std::max(segment_step, 1e-6) / maximum_extent, 1e-6, 0.05);
            si->setStateValidityCheckingResolution(checking_resolution);

            ob::ScopedState<ob::RealVectorStateSpace> q_start(space), q_goal(space);
            for (int dim = 0; dim < dimension; ++dim) {
                q_start->values[dim] = start[static_cast<std::size_t>(dim)];
                q_goal->values[dim] = goal[static_cast<std::size_t>(dim)];
            }
            setup.setStartAndGoalStates(q_start, q_goal);
            setup.setOptimizationObjective(std::make_shared<ob::PathLengthOptimizationObjective>(si));
            auto planner = std::make_shared<SeededBITstar>(si);
            planner->setLocalSeed(mix_seed(base_seed, 0x42495450U));
            configure_bitstar_planner(
                planner,
                samples_per_batch,
                rewire_factor,
                stop_on_solution_improvement,
                use_k_nearest,
                pruning,
                prune_threshold_fraction,
                delay_rewiring_until_initial_solution,
                just_in_time_sampling,
                drop_samples_on_prune,
                approximate_solutions,
                strict_queue_ordering,
                cascading_rewirings,
                initial_inflation_factor,
                inflation_scaling_parameter,
                truncation_scaling_parameter,
                allowed_failed_sampling_attempts);
            setup.setPlanner(planner);
            const double timeout_s = std::max(0.0, timeout_ms) / 1000.0;
            const double solve_slice_s = 0.10;
            ob::PlannerStatus status = ob::PlannerStatus::UNKNOWN;
            bool exact_solution = false;
            while (ompl_elapsed_s(t0) < timeout_s - 1e-6) {
                const double remaining_s = std::max(1e-5, timeout_s - ompl_elapsed_s(t0));
                const double slice_s = std::min(solve_slice_s, remaining_s);
                const double ptc_interval_s = std::max(1e-3, std::min(0.01, slice_s / 10.0));
                const auto before_s = ompl_elapsed_s(t0);
                status = setup.solve(ob::timedPlannerTerminationCondition(slice_s, ptc_interval_s));
                exact_solution = exact_solution || status == ob::PlannerStatus::EXACT_SOLUTION;
                if (stop_on_solution_improvement && setup.haveSolutionPath()) {
                    break;
                }
                // Guard against planners that do not make observable progress or overshoot a slice.
                if (ompl_elapsed_s(t0) <= before_s + 1e-7) {
                    break;
                }
            }
            bool ok = exact_solution || setup.haveSolutionPath();
            const double solve_s = ompl_elapsed_s(t0);
            double simplify_s = 0.0;
            if (ok && simplify_time_s > 0.0) {
                const auto simplify_start = std::chrono::steady_clock::now();
                setup.simplifySolution(simplify_time_s);
                simplify_s = ompl_elapsed_s(simplify_start);
            }
            std::vector<std::vector<double>> path;
            if (ok && setup.haveSolutionPath()) {
                auto& solution_path = setup.getSolutionPath();
                path.reserve(solution_path.getStateCount());
                for (std::size_t index = 0; index < solution_path.getStateCount(); ++index) {
                    path.push_back(list_from_ompl_state(solution_path.getState(index), dimension));
                }
            }
            if (path.size() < 2) {
                ok = false;
            }
            result["ok"] = ok;
            result["reason"] = ok ? "connected" : std::string(status.asString());
            result["status"] = std::string(status.asString());
            result["exact_solution"] = exact_solution;
            result["solve_s"] = solve_s;
            result["simplify_s"] = simplify_s;
            result["t_s"] = solve_s + simplify_s;
            result["path"] = path;
            result["nodes"] = 0;
            result["checking_resolution"] = checking_resolution;
            result["planner"] = "OMPL_BITstar";
            result["iterations"] = static_cast<int>(planner->numIterations());
            result["batches"] = static_cast<int>(planner->numBatches());
            return result;
        },
        py::arg("robot"),
        py::arg("obstacles"),
        py::arg("start"),
        py::arg("goal"),
        py::arg("timeout_ms") = 10000.0,
        py::arg("segment_step") = 0.05,
        py::arg("simplify_time_s") = 0.1,
        py::arg("seed") = 42,
        py::arg("samples_per_batch") = -1,
        py::arg("rewire_factor") = -1.0,
        py::arg("stop_on_solution_improvement") = true,
        py::arg("use_k_nearest") = -1,
        py::arg("pruning") = -1,
        py::arg("prune_threshold_fraction") = -1.0,
        py::arg("delay_rewiring_until_initial_solution") = -1,
        py::arg("just_in_time_sampling") = -1,
        py::arg("drop_samples_on_prune") = -1,
        py::arg("approximate_solutions") = -1,
        py::arg("strict_queue_ordering") = -1,
        py::arg("cascading_rewirings") = -1,
        py::arg("initial_inflation_factor") = -1.0,
        py::arg("inflation_scaling_parameter") = -1.0,
        py::arg("truncation_scaling_parameter") = -1.0,
        py::arg("allowed_failed_sampling_attempts") = -1);

    module.def("ompl_cspace_rrt_connect_path",
        [](const std::vector<std::vector<double>>& limits,
           const std::vector<std::vector<double>>& obstacles,
           const std::vector<double>& start,
           const std::vector<double>& goal,
           double timeout_ms,
           double range,
           double segment_step,
           double simplify_time_s,
           int seed) {
            namespace ob = ompl::base;
            namespace og = ompl::geometric;
            ompl::msg::setLogLevel(ompl::msg::LOG_ERROR);
            py::dict result;
            const int dimension = validate_cspace_limits(limits);
            validate_cspace_vector(start, dimension, "start");
            validate_cspace_vector(goal, dimension, "goal");
            const auto t0 = std::chrono::steady_clock::now();
            if (!cspace_vector_valid(start, dimension, obstacles) || !cspace_vector_valid(goal, dimension, obstacles)) {
                const double solve_s = ompl_elapsed_s(t0);
                result["ok"] = false;
                result["reason"] = "endpoint_collision";
                result["status"] = "endpoint_collision";
                result["solve_s"] = solve_s;
                result["simplify_s"] = 0.0;
                result["t_s"] = solve_s;
                result["path"] = std::vector<std::vector<double>>{};
                result["nodes"] = 0;
                return result;
            }

            const auto base_seed = normalize_seed(seed);
            auto space = make_cspace_ompl_space(limits);
            configure_deterministic_state_sampler(space, mix_seed(base_seed, 0x43525253U));
            og::SimpleSetup setup(space);
            setup.setStateValidityChecker([obstacles, dimension](const ob::State* state) {
                const auto* vector_state = state->as<ob::RealVectorStateSpace::StateType>();
                return cspace_point_valid(vector_state->values, dimension, obstacles);
            });
            const double maximum_extent = std::max(space->getMaximumExtent(), 1e-9);
            const double checking_resolution = std::clamp(std::max(segment_step, 1e-6) / maximum_extent, 1e-6, 0.05);
            setup.getSpaceInformation()->setStateValidityCheckingResolution(checking_resolution);

            ob::ScopedState<ob::RealVectorStateSpace> q_start(space), q_goal(space);
            for (int dim = 0; dim < dimension; ++dim) {
                q_start->values[dim] = start[static_cast<std::size_t>(dim)];
                q_goal->values[dim] = goal[static_cast<std::size_t>(dim)];
            }
            setup.setStartAndGoalStates(q_start, q_goal);
            auto planner = std::make_shared<SeededRRTConnect>(setup.getSpaceInformation());
            planner->setLocalSeed(mix_seed(base_seed, 0x43525250U));
            if (range > 0.0) {
                planner->setRange(range);
            }
            setup.setPlanner(planner);

            const double timeout_s = std::max(0.0, timeout_ms) / 1000.0;
            const double ptc_interval_s = std::max(1e-3, std::min(0.05, timeout_s / 20.0));
            ob::PlannerStatus status = setup.solve(ob::timedPlannerTerminationCondition(timeout_s, ptc_interval_s));
            const double solve_s = ompl_elapsed_s(t0);
            const bool exact_solution = status == ob::PlannerStatus::EXACT_SOLUTION;
            bool ok = exact_solution;
            double simplify_s = 0.0;
            if (ok && simplify_time_s > 0.0) {
                const auto simplify_start = std::chrono::steady_clock::now();
                setup.simplifySolution(simplify_time_s);
                simplify_s = ompl_elapsed_s(simplify_start);
            }
            std::vector<std::vector<double>> path;
            if (ok && setup.haveSolutionPath()) {
                path = path_from_problem_solution(setup.getProblemDefinition(), dimension);
                ok = path.size() >= 2;
            } else {
                ok = false;
            }
            ob::PlannerData planner_data(setup.getSpaceInformation());
            planner->getPlannerData(planner_data);
            result["ok"] = ok;
            result["reason"] = ok ? "connected" : std::string(status.asString());
            result["status"] = std::string(status.asString());
            result["exact_solution"] = exact_solution;
            result["solve_s"] = solve_s;
            result["simplify_s"] = simplify_s;
            result["t_s"] = solve_s + simplify_s;
            result["path"] = path;
            result["nodes"] = static_cast<int>(planner_data.numVertices());
            result["checking_resolution"] = checking_resolution;
            result["planner"] = "OMPL_CSpace_RRTConnect";
            return result;
        },
        py::arg("limits"),
        py::arg("obstacles"),
        py::arg("start"),
        py::arg("goal"),
        py::arg("timeout_ms") = 1000.0,
        py::arg("range") = 0.35,
        py::arg("segment_step") = 0.01,
        py::arg("simplify_time_s") = 0.0,
        py::arg("seed") = 42);

    module.def("ompl_cspace_prm_multiquery",
        [](const std::vector<std::vector<double>>& limits,
           const std::vector<std::vector<double>>& obstacles,
           const std::vector<std::vector<double>>& starts,
           const std::vector<std::vector<double>>& goals,
           double build_budget_s,
           double query_budget_s,
           double segment_step,
           double simplify_time_s,
           int seed,
           int max_nearest_neighbors,
           const std::string& planner_kind,
           bool preload_query_endpoints) {
            namespace ob = ompl::base;
            namespace og = ompl::geometric;
            ompl::msg::setLogLevel(ompl::msg::LOG_ERROR);
            const auto base_seed = normalize_seed(seed);
            py::dict result;
            const int dimension = validate_cspace_limits(limits);
            if (starts.empty() || starts.size() != goals.size()) {
                throw std::invalid_argument("starts/goals must be non-empty and have the same length");
            }
            for (std::size_t index = 0; index < starts.size(); ++index) {
                validate_cspace_vector(starts[index], dimension, "start");
                validate_cspace_vector(goals[index], dimension, "goal");
            }
            auto endpoint_collision = [&](const std::vector<double>& q) {
                return !cspace_vector_valid(q, dimension, obstacles);
            };
            auto space = make_cspace_ompl_space(limits);
            configure_deterministic_state_sampler(space, mix_seed(base_seed, 0x43505253U));
            auto si = std::make_shared<ob::SpaceInformation>(space);
            si->setStateValidityChecker([obstacles, dimension](const ob::State* state) {
                const auto* vector_state = state->as<ob::RealVectorStateSpace::StateType>();
                return cspace_point_valid(vector_state->values, dimension, obstacles);
            });
            const double maximum_extent = std::max(space->getMaximumExtent(), 1e-9);
            const double checking_resolution = std::clamp(std::max(segment_step, 1e-6) / maximum_extent, 1e-6, 0.05);
            si->setStateValidityCheckingResolution(checking_resolution);
            si->setup();
            auto problem = std::make_shared<ob::ProblemDefinition>(si);
            set_problem_query(problem, space, si, starts.front(), goals.front(), dimension);
            std::shared_ptr<og::PRM> planner;
            std::shared_ptr<SeededPRM> seeded_prm;
            std::shared_ptr<SeededPRMstar> seeded_prmstar;
            const bool use_prmstar = planner_kind == "prmstar" || planner_kind == "PRMstar" || planner_kind == "PRM*";
            if (use_prmstar) {
                seeded_prmstar = std::make_shared<SeededPRMstar>(si);
                seeded_prmstar->setLocalSeed(mix_seed(base_seed, 0x43505250U));
                planner = seeded_prmstar;
            } else {
                seeded_prm = std::make_shared<SeededPRM>(si);
                seeded_prm->setLocalSeed(mix_seed(base_seed, 0x43505250U));
                planner = seeded_prm;
            }
            if (!use_prmstar && max_nearest_neighbors > 0) {
                planner->setMaxNearestNeighbors(static_cast<unsigned int>(max_nearest_neighbors));
            }
            planner->setProblemDefinition(problem);
            planner->setup();

            const auto build_start = std::chrono::steady_clock::now();
            if (preload_query_endpoints) {
                for (std::size_t index = 0; index < starts.size(); ++index) {
                    ob::ScopedState<ob::RealVectorStateSpace> q_start(space), q_goal(space);
                    for (int dim = 0; dim < dimension; ++dim) {
                        q_start->values[dim] = starts[index][static_cast<std::size_t>(dim)];
                        q_goal->values[dim] = goals[index][static_cast<std::size_t>(dim)];
                    }
                    if (use_prmstar) {
                        seeded_prmstar->addMilestoneFromState(q_start.get());
                        seeded_prmstar->addMilestoneFromState(q_goal.get());
                    } else {
                        seeded_prm->addMilestoneFromState(q_start.get());
                        seeded_prm->addMilestoneFromState(q_goal.get());
                    }
                }
            }
            planner->constructRoadmap(ob::timedPlannerTerminationCondition(std::max(0.0, build_budget_s)));
            const double build_s = ompl_elapsed_s(build_start);

            py::list query_results;
            for (std::size_t index = 0; index < starts.size(); ++index) {
                py::dict row;
                row["index"] = static_cast<int>(index);
                if (endpoint_collision(starts[index]) || endpoint_collision(goals[index])) {
                    row["ok"] = false;
                    row["reason"] = "endpoint_collision";
                    row["status"] = "endpoint_collision";
                    row["solve_s"] = 0.0;
                    row["simplify_s"] = 0.0;
                    row["t_s"] = 0.0;
                    row["path"] = std::vector<std::vector<double>>{};
                    query_results.append(row);
                    continue;
                }
                planner->clearQuery();
                set_problem_query(problem, space, si, starts[index], goals[index], dimension);
                const auto query_start = std::chrono::steady_clock::now();
                ob::ScopedState<ob::RealVectorStateSpace> q_start(space), q_goal(space);
                for (int dim = 0; dim < dimension; ++dim) {
                    q_start->values[dim] = starts[index][static_cast<std::size_t>(dim)];
                    q_goal->values[dim] = goals[index][static_cast<std::size_t>(dim)];
                }
                if (use_prmstar) {
                    seeded_prmstar->addMilestoneFromState(q_start.get());
                    seeded_prmstar->addMilestoneFromState(q_goal.get());
                } else {
                    seeded_prm->addMilestoneFromState(q_start.get());
                    seeded_prm->addMilestoneFromState(q_goal.get());
                }
                ob::PlannerStatus status = planner->solve(ob::timedPlannerTerminationCondition(std::max(0.0, query_budget_s)));
                bool ok = status == ob::PlannerStatus::EXACT_SOLUTION || static_cast<bool>(status);
                const double solve_s = ompl_elapsed_s(query_start);
                double simplify_s = 0.0;
                if (ok && simplify_time_s > 0.0) {
                    auto path_ptr = problem->getSolutionPath();
                    auto geometric_path = std::dynamic_pointer_cast<og::PathGeometric>(path_ptr);
                    if (geometric_path) {
                        const auto simplify_start = std::chrono::steady_clock::now();
                        og::PathSimplifier simplifier(si);
                        simplifier.simplify(*geometric_path, std::max(0.0, simplify_time_s));
                        simplify_s = ompl_elapsed_s(simplify_start);
                    }
                }
                auto path = ok ? path_from_problem_solution(problem, dimension) : std::vector<std::vector<double>>{};
                if (path.size() < 2) {
                    ok = false;
                }
                row["ok"] = ok;
                row["reason"] = ok ? "connected" : std::string(status.asString());
                row["status"] = std::string(status.asString());
                row["solve_s"] = solve_s;
                row["simplify_s"] = simplify_s;
                row["t_s"] = solve_s + simplify_s;
                row["path"] = path;
                query_results.append(row);
            }
            ob::PlannerData planner_data(si);
            planner->getPlannerData(planner_data);
            result["ok"] = true;
            result["planner"] = use_prmstar ? "OMPL_CSpace_PRMstar" : "OMPL_CSpace_PRM";
            result["build_s"] = build_s;
            result["nodes"] = static_cast<int>(planner_data.numVertices());
            result["checking_resolution"] = checking_resolution;
            result["preload_query_endpoints"] = preload_query_endpoints;
            result["queries"] = query_results;
            return result;
        },
        py::arg("limits"),
        py::arg("obstacles"),
        py::arg("starts"),
        py::arg("goals"),
        py::arg("build_budget_s") = 2.0,
        py::arg("query_budget_s") = 4.0,
        py::arg("segment_step") = 0.01,
        py::arg("simplify_time_s") = 0.01,
        py::arg("seed") = 42,
        py::arg("max_nearest_neighbors") = 128,
        py::arg("planner_kind") = "prm",
        py::arg("preload_query_endpoints") = true);

    module.def("ompl_cspace_bitstar_trace",
        [](const std::vector<std::vector<double>>& limits,
           const std::vector<std::vector<double>>& obstacles,
           const std::vector<double>& start,
           const std::vector<double>& goal,
           double timeout_ms,
           double checkpoint_interval_ms,
           double segment_step,
           int seed,
           int samples_per_batch,
           double rewire_factor,
           bool stop_on_solution_improvement,
           int use_k_nearest,
           int pruning,
           double prune_threshold_fraction,
           int delay_rewiring_until_initial_solution,
           int just_in_time_sampling,
           int drop_samples_on_prune,
           int approximate_solutions,
           int strict_queue_ordering,
           int cascading_rewirings,
           double initial_inflation_factor,
           double inflation_scaling_parameter,
           double truncation_scaling_parameter,
           int allowed_failed_sampling_attempts) {
            namespace ob = ompl::base;
            namespace og = ompl::geometric;
            ompl::msg::setLogLevel(ompl::msg::LOG_ERROR);
            const int dimension = validate_cspace_limits(limits);
            validate_cspace_vector(start, dimension, "start");
            validate_cspace_vector(goal, dimension, "goal");
            const auto t0 = std::chrono::steady_clock::now();
            py::dict result;
            py::list checkpoints;
            if (!cspace_vector_valid(start, dimension, obstacles) || !cspace_vector_valid(goal, dimension, obstacles)) {
                const double solve_s = ompl_elapsed_s(t0);
                result["ok"] = false;
                result["reason"] = "endpoint_collision";
                result["status"] = "endpoint_collision";
                result["solve_s"] = solve_s;
                result["simplify_s"] = 0.0;
                result["t_s"] = solve_s;
                result["path"] = std::vector<std::vector<double>>{};
                result["nodes"] = 0;
                result["checkpoints"] = checkpoints;
                return result;
            }

            const auto base_seed = normalize_seed(seed);
            auto space = make_cspace_ompl_space(limits);
            configure_deterministic_state_sampler(space, mix_seed(base_seed, 0x43424953U));
            og::SimpleSetup setup(space);
            auto si = setup.getSpaceInformation();
            si->setStateValidityChecker([obstacles, dimension](const ob::State* state) {
                const auto* vector_state = state->as<ob::RealVectorStateSpace::StateType>();
                return cspace_point_valid(vector_state->values, dimension, obstacles);
            });
            const double maximum_extent = std::max(space->getMaximumExtent(), 1e-9);
            const double checking_resolution = std::clamp(std::max(segment_step, 1e-6) / maximum_extent, 1e-6, 0.05);
            si->setStateValidityCheckingResolution(checking_resolution);

            ob::ScopedState<ob::RealVectorStateSpace> q_start(space), q_goal(space);
            for (int dim = 0; dim < dimension; ++dim) {
                q_start->values[dim] = start[static_cast<std::size_t>(dim)];
                q_goal->values[dim] = goal[static_cast<std::size_t>(dim)];
            }
            setup.setStartAndGoalStates(q_start, q_goal);
            setup.setOptimizationObjective(std::make_shared<ob::PathLengthOptimizationObjective>(si));
            auto planner = std::make_shared<SeededBITstar>(si);
            planner->setLocalSeed(mix_seed(base_seed, 0x43424950U));
            configure_bitstar_planner(
                planner,
                samples_per_batch,
                rewire_factor,
                stop_on_solution_improvement,
                use_k_nearest,
                pruning,
                prune_threshold_fraction,
                delay_rewiring_until_initial_solution,
                just_in_time_sampling,
                drop_samples_on_prune,
                approximate_solutions,
                strict_queue_ordering,
                cascading_rewirings,
                initial_inflation_factor,
                inflation_scaling_parameter,
                truncation_scaling_parameter,
                allowed_failed_sampling_attempts);
            setup.setPlanner(planner);

            const double timeout_s = std::max(0.0, timeout_ms) / 1000.0;
            const double interval_s = std::max(1e-3, checkpoint_interval_ms / 1000.0);
            std::string last_status = "not_run";
            bool last_exact = false;
            bool trace_stopped_early = false;
            const int quality_stall_checkpoints = 0;
            const double quality_stall_rel_tol = 0.005;
            std::string trace_stop_reason = "";
            double best_path_length = std::numeric_limits<double>::infinity();
            int quality_stall_count = 0;

            auto vector_path_length = [](const std::vector<std::vector<double>>& path) {
                if (path.size() < 2) {
                    return std::numeric_limits<double>::infinity();
                }
                double total = 0.0;
                for (std::size_t i = 1; i < path.size(); ++i) {
                    const auto& a = path[i - 1];
                    const auto& b = path[i];
                    double squared = 0.0;
                    const std::size_t dims = std::min(a.size(), b.size());
                    for (std::size_t dim = 0; dim < dims; ++dim) {
                        const double delta = b[dim] - a[dim];
                        squared += delta * delta;
                    }
                    total += std::sqrt(squared);
                }
                return total;
            };

            auto append_checkpoint = [&](double target_s) {
                const double elapsed_s = ompl_elapsed_s(t0);
                std::vector<std::vector<double>> path;
                bool ok = false;
                if (setup.haveSolutionPath()) {
                    path = path_from_problem_solution(setup.getProblemDefinition(), dimension);
                    ok = path.size() >= 2;
                }
                const double length = ok ? vector_path_length(path) : std::numeric_limits<double>::infinity();
                py::dict row;
                row["checkpoint_s"] = target_s;
                row["elapsed_s"] = elapsed_s;
                row["ok"] = ok;
                row["reason"] = ok ? "connected" : last_status;
                row["status"] = ok ? "solution" : last_status;
                row["exact_solution"] = ok || last_exact;
                row["solve_s"] = elapsed_s;
                row["simplify_s"] = 0.0;
                row["t_s"] = elapsed_s;
                row["path"] = path;
                row["path_length"] = std::isfinite(length) ? length : std::numeric_limits<double>::quiet_NaN();
                row["nodes"] = 0;
                row["iterations"] = static_cast<int>(planner->numIterations());
                row["batches"] = static_cast<int>(planner->numBatches());
                checkpoints.append(row);
                return length;
            };

            auto update_quality_stall = [&](double length) {
                if (quality_stall_checkpoints <= 0 || !std::isfinite(length)) {
                    return false;
                }
                const double rel_tol = std::max(0.0, quality_stall_rel_tol);
                if (!std::isfinite(best_path_length) || length < best_path_length * (1.0 - rel_tol)) {
                    best_path_length = length;
                    quality_stall_count = 0;
                    return false;
                }
                ++quality_stall_count;
                return quality_stall_count >= quality_stall_checkpoints;
            };

            if (timeout_s <= 0.0) {
                append_checkpoint(0.0);
            } else {
                for (double target_s = interval_s; target_s < timeout_s - 1e-9; target_s += interval_s) {
                    const double clamped_target_s = std::min(target_s, timeout_s);
                    while (ompl_elapsed_s(t0) < clamped_target_s - 1e-6) {
                        const double remaining_s = std::max(1e-5, clamped_target_s - ompl_elapsed_s(t0));
                        const double ptc_interval_s = std::max(1e-3, std::min(0.05, remaining_s / 20.0));
                        const auto before_s = ompl_elapsed_s(t0);
                        ob::PlannerStatus status = setup.solve(ob::timedPlannerTerminationCondition(remaining_s, ptc_interval_s));
                        last_status = std::string(status.asString());
                        last_exact = status == ob::PlannerStatus::EXACT_SOLUTION;
                        if (ompl_elapsed_s(t0) <= before_s + 1e-6) {
                            break;
                        }
                        if (stop_on_solution_improvement && setup.haveSolutionPath()) {
                            trace_stopped_early = true;
                            break;
                        }
                    }
                    const double checkpoint_length = append_checkpoint(clamped_target_s);
                    if (update_quality_stall(checkpoint_length)) {
                        trace_stopped_early = true;
                        trace_stop_reason = "quality_stall";
                    }
                    if (trace_stopped_early) {
                        break;
                    }
                }
                while (!trace_stopped_early && ompl_elapsed_s(t0) < timeout_s - 1e-6) {
                    const double remaining_s = std::max(1e-5, timeout_s - ompl_elapsed_s(t0));
                    const double ptc_interval_s = std::max(1e-3, std::min(0.05, remaining_s / 20.0));
                    const auto before_s = ompl_elapsed_s(t0);
                    ob::PlannerStatus status = setup.solve(ob::timedPlannerTerminationCondition(remaining_s, ptc_interval_s));
                    last_status = std::string(status.asString());
                    last_exact = status == ob::PlannerStatus::EXACT_SOLUTION;
                    if (ompl_elapsed_s(t0) <= before_s + 1e-6) {
                        break;
                    }
                    if (stop_on_solution_improvement && setup.haveSolutionPath()) {
                        trace_stopped_early = true;
                        break;
                    }
                }
                if (checkpoints.empty()) {
                    append_checkpoint(timeout_s);
                } else if (!trace_stopped_early) {
                    append_checkpoint(timeout_s);
                }
            }

            std::vector<std::vector<double>> final_path;
            bool ok = false;
            if (setup.haveSolutionPath()) {
                final_path = path_from_problem_solution(setup.getProblemDefinition(), dimension);
                ok = final_path.size() >= 2;
            }
            const double solve_s = ompl_elapsed_s(t0);
            result["ok"] = ok;
            result["reason"] = ok ? "connected" : last_status;
            result["status"] = ok ? "solution" : last_status;
            result["exact_solution"] = ok || last_exact;
            result["solve_s"] = solve_s;
            result["simplify_s"] = 0.0;
            result["t_s"] = solve_s;
            result["path"] = final_path;
            result["nodes"] = 0;
            result["checking_resolution"] = checking_resolution;
            result["planner"] = "OMPL_CSpace_BITstar";
            result["iterations"] = static_cast<int>(planner->numIterations());
            result["batches"] = static_cast<int>(planner->numBatches());
            result["stopped_early"] = trace_stopped_early;
            result["stop_on_solution_improvement"] = stop_on_solution_improvement;
            result["checkpoints"] = checkpoints;
            return result;
        },
        py::arg("limits"),
        py::arg("obstacles"),
        py::arg("start"),
        py::arg("goal"),
        py::arg("timeout_ms") = 1000.0,
        py::arg("checkpoint_interval_ms") = 10.0,
        py::arg("segment_step") = 0.01,
        py::arg("seed") = 42,
        py::arg("samples_per_batch") = -1,
        py::arg("rewire_factor") = -1.0,
        py::arg("stop_on_solution_improvement") = false,
        py::arg("use_k_nearest") = -1,
        py::arg("pruning") = -1,
        py::arg("prune_threshold_fraction") = -1.0,
        py::arg("delay_rewiring_until_initial_solution") = -1,
        py::arg("just_in_time_sampling") = -1,
        py::arg("drop_samples_on_prune") = -1,
        py::arg("approximate_solutions") = -1,
        py::arg("strict_queue_ordering") = -1,
        py::arg("cascading_rewirings") = -1,
        py::arg("initial_inflation_factor") = -1.0,
        py::arg("inflation_scaling_parameter") = -1.0,
        py::arg("truncation_scaling_parameter") = -1.0,
        py::arg("allowed_failed_sampling_attempts") = -1);

    module.def("ompl_bitstar_trace",
        [](const rbf::Robot& robot,
           const std::vector<rbf::Obstacle>& obstacles,
           const std::vector<double>& start,
           const std::vector<double>& goal,
           double timeout_ms,
           double checkpoint_interval_ms,
           double segment_step,
           int seed,
           int samples_per_batch,
           double rewire_factor,
           bool stop_on_solution_improvement,
           int quality_stall_checkpoints,
           double quality_stall_rel_tol,
           int use_k_nearest,
           int pruning,
           double prune_threshold_fraction,
           int delay_rewiring_until_initial_solution,
           int just_in_time_sampling,
           int drop_samples_on_prune,
           int approximate_solutions,
           int strict_queue_ordering,
           int cascading_rewirings,
           double initial_inflation_factor,
           double inflation_scaling_parameter,
           double truncation_scaling_parameter,
           int allowed_failed_sampling_attempts) {
            namespace ob = ompl::base;
            namespace og = ompl::geometric;
            ompl::msg::setLogLevel(ompl::msg::LOG_ERROR);
            const auto base_seed = normalize_seed(seed);
            py::dict result;
            py::list checkpoints;
            const int dimension = robot.n_joints();
            if (static_cast<int>(start.size()) != dimension || static_cast<int>(goal.size()) != dimension) {
                throw std::invalid_argument("start/goal dimension must match robot.n_joints()");
            }
            const auto t0 = std::chrono::steady_clock::now();
            auto checker = std::make_shared<rbf::CollisionChecker>(robot, rbf::Scene(obstacles));
            if (checker->check_config(eigen_vector_from_list(start)) || checker->check_config(eigen_vector_from_list(goal))) {
                const double solve_s = ompl_elapsed_s(t0);
                result["ok"] = false;
                result["reason"] = "endpoint_collision";
                result["status"] = "endpoint_collision";
                result["solve_s"] = solve_s;
                result["simplify_s"] = 0.0;
                result["t_s"] = solve_s;
                result["path"] = std::vector<std::vector<double>>{};
                result["nodes"] = 0;
                result["checkpoints"] = checkpoints;
                return result;
            }

            auto space = make_ompl_space(robot);
            configure_deterministic_state_sampler(space, mix_seed(base_seed, 0x42495453U));
            double checking_resolution = 0.0;
            og::SimpleSetup setup(space);
            auto si = setup.getSpaceInformation();
            si->setStateValidityChecker([checker, dimension](const ob::State* state) {
                const auto* vector_state = state->as<ob::RealVectorStateSpace::StateType>();
                Eigen::VectorXd q(dimension);
                for (int dim = 0; dim < dimension; ++dim) {
                    q[dim] = vector_state->values[dim];
                }
                return !checker->check_config(q);
            });
            const double maximum_extent = std::max(space->getMaximumExtent(), 1e-9);
            checking_resolution = std::clamp(std::max(segment_step, 1e-6) / maximum_extent, 1e-6, 0.05);
            si->setStateValidityCheckingResolution(checking_resolution);

            ob::ScopedState<ob::RealVectorStateSpace> q_start(space), q_goal(space);
            for (int dim = 0; dim < dimension; ++dim) {
                q_start->values[dim] = start[static_cast<std::size_t>(dim)];
                q_goal->values[dim] = goal[static_cast<std::size_t>(dim)];
            }
            setup.setStartAndGoalStates(q_start, q_goal);
            setup.setOptimizationObjective(std::make_shared<ob::PathLengthOptimizationObjective>(si));
            auto planner = std::make_shared<SeededBITstar>(si);
            planner->setLocalSeed(mix_seed(base_seed, 0x42495450U));
            configure_bitstar_planner(
                planner,
                samples_per_batch,
                rewire_factor,
                stop_on_solution_improvement,
                use_k_nearest,
                pruning,
                prune_threshold_fraction,
                delay_rewiring_until_initial_solution,
                just_in_time_sampling,
                drop_samples_on_prune,
                approximate_solutions,
                strict_queue_ordering,
                cascading_rewirings,
                initial_inflation_factor,
                inflation_scaling_parameter,
                truncation_scaling_parameter,
                allowed_failed_sampling_attempts);
            setup.setPlanner(planner);

            const double timeout_s = std::max(0.0, timeout_ms) / 1000.0;
            const double interval_s = std::max(1e-3, checkpoint_interval_ms / 1000.0);
            std::string last_status = "not_run";
            bool last_exact = false;
            bool trace_stopped_early = false;
            std::string trace_stop_reason = "";
            double best_path_length = std::numeric_limits<double>::infinity();
            int quality_stall_count = 0;

            auto vector_path_length = [](const std::vector<std::vector<double>>& path) {
                if (path.size() < 2) {
                    return std::numeric_limits<double>::infinity();
                }
                double total = 0.0;
                for (std::size_t i = 1; i < path.size(); ++i) {
                    const auto& a = path[i - 1];
                    const auto& b = path[i];
                    double squared = 0.0;
                    const std::size_t dims = std::min(a.size(), b.size());
                    for (std::size_t dim = 0; dim < dims; ++dim) {
                        const double delta = b[dim] - a[dim];
                        squared += delta * delta;
                    }
                    total += std::sqrt(squared);
                }
                return total;
            };

            auto append_checkpoint = [&](double target_s) {
                const double elapsed_s = ompl_elapsed_s(t0);
                std::vector<std::vector<double>> path;
                bool ok = false;
                if (setup.haveSolutionPath()) {
                    path = path_from_problem_solution(setup.getProblemDefinition(), dimension);
                    ok = path.size() >= 2;
                }
                const double length = ok ? vector_path_length(path) : std::numeric_limits<double>::infinity();
                py::dict row;
                row["checkpoint_s"] = target_s;
                row["elapsed_s"] = elapsed_s;
                row["ok"] = ok;
                row["reason"] = ok ? "connected" : last_status;
                row["status"] = ok ? "solution" : last_status;
                row["exact_solution"] = ok || last_exact;
                row["solve_s"] = elapsed_s;
                row["simplify_s"] = 0.0;
                row["t_s"] = elapsed_s;
                row["path"] = path;
                row["path_length"] = std::isfinite(length) ? length : std::numeric_limits<double>::quiet_NaN();
                row["nodes"] = 0;
                row["iterations"] = static_cast<int>(planner->numIterations());
                row["batches"] = static_cast<int>(planner->numBatches());
                checkpoints.append(row);
                return length;
            };

            auto update_quality_stall = [&](double length) {
                if (quality_stall_checkpoints <= 0 || !std::isfinite(length)) {
                    return false;
                }
                const double rel_tol = std::max(0.0, quality_stall_rel_tol);
                if (!std::isfinite(best_path_length) || length < best_path_length * (1.0 - rel_tol)) {
                    best_path_length = length;
                    quality_stall_count = 0;
                    return false;
                }
                ++quality_stall_count;
                return quality_stall_count >= quality_stall_checkpoints;
            };

            if (timeout_s <= 0.0) {
                append_checkpoint(0.0);
            } else {
                for (double target_s = interval_s; target_s < timeout_s - 1e-9; target_s += interval_s) {
                    const double clamped_target_s = std::min(target_s, timeout_s);
                    while (ompl_elapsed_s(t0) < clamped_target_s - 1e-6) {
                        const double remaining_s = std::max(1e-5, clamped_target_s - ompl_elapsed_s(t0));
                        const double ptc_interval_s = std::max(1e-3, std::min(0.05, remaining_s / 20.0));
                        const auto before_s = ompl_elapsed_s(t0);
                        ob::PlannerStatus status = setup.solve(ob::timedPlannerTerminationCondition(remaining_s, ptc_interval_s));
                        last_status = std::string(status.asString());
                        last_exact = status == ob::PlannerStatus::EXACT_SOLUTION;
                        if (ompl_elapsed_s(t0) <= before_s + 1e-6) {
                            break;
                        }
                        if (stop_on_solution_improvement && setup.haveSolutionPath()) {
                            trace_stopped_early = true;
                            trace_stop_reason = "first_solution";
                            break;
                        }
                    }
                    const double checkpoint_length = append_checkpoint(clamped_target_s);
                    if (update_quality_stall(checkpoint_length)) {
                        trace_stopped_early = true;
                        trace_stop_reason = "quality_stall";
                    }
                    if (trace_stopped_early) {
                        break;
                    }
                }
                while (!trace_stopped_early && ompl_elapsed_s(t0) < timeout_s - 1e-6) {
                    const double remaining_s = std::max(1e-5, timeout_s - ompl_elapsed_s(t0));
                    const double ptc_interval_s = std::max(1e-3, std::min(0.05, remaining_s / 20.0));
                    const auto before_s = ompl_elapsed_s(t0);
                    ob::PlannerStatus status = setup.solve(ob::timedPlannerTerminationCondition(remaining_s, ptc_interval_s));
                    last_status = std::string(status.asString());
                    last_exact = status == ob::PlannerStatus::EXACT_SOLUTION;
                    if (ompl_elapsed_s(t0) <= before_s + 1e-6) {
                        break;
                    }
                    if (stop_on_solution_improvement && setup.haveSolutionPath()) {
                        trace_stopped_early = true;
                        trace_stop_reason = "first_solution";
                        break;
                    }
                }
                if (checkpoints.empty() || !trace_stopped_early) {
                    const double checkpoint_length = append_checkpoint(timeout_s);
                    if (update_quality_stall(checkpoint_length)) {
                        trace_stopped_early = true;
                        trace_stop_reason = "quality_stall";
                    }
                }
            }

            std::vector<std::vector<double>> final_path;
            bool ok = false;
            if (setup.haveSolutionPath()) {
                final_path = path_from_problem_solution(setup.getProblemDefinition(), dimension);
                ok = final_path.size() >= 2;
            }
            result["ok"] = ok;
            result["reason"] = ok ? "connected" : last_status;
            result["status"] = ok ? "solution" : last_status;
            result["exact_solution"] = ok || last_exact;
            const double solve_s = ompl_elapsed_s(t0);
            result["solve_s"] = solve_s;
            result["simplify_s"] = 0.0;
            result["t_s"] = solve_s;
            result["path"] = final_path;
            result["nodes"] = 0;
            result["checking_resolution"] = checking_resolution;
            result["planner"] = "OMPL_BITstar";
            result["iterations"] = static_cast<int>(planner->numIterations());
            result["batches"] = static_cast<int>(planner->numBatches());
            result["stopped_early"] = trace_stopped_early;
            result["early_stop_reason"] = trace_stop_reason;
            result["stop_on_solution_improvement"] = stop_on_solution_improvement;
            result["quality_stall_checkpoints"] = quality_stall_checkpoints;
            result["quality_stall_rel_tol"] = quality_stall_rel_tol;
            result["checkpoints"] = checkpoints;
            return result;
        },
        py::arg("robot"),
        py::arg("obstacles"),
        py::arg("start"),
        py::arg("goal"),
        py::arg("timeout_ms") = 10000.0,
        py::arg("checkpoint_interval_ms") = 1000.0,
        py::arg("segment_step") = 0.05,
        py::arg("seed") = 42,
        py::arg("samples_per_batch") = -1,
        py::arg("rewire_factor") = -1.0,
        py::arg("stop_on_solution_improvement") = false,
        py::arg("quality_stall_checkpoints") = 0,
        py::arg("quality_stall_rel_tol") = 0.005,
        py::arg("use_k_nearest") = -1,
        py::arg("pruning") = -1,
        py::arg("prune_threshold_fraction") = -1.0,
        py::arg("delay_rewiring_until_initial_solution") = -1,
        py::arg("just_in_time_sampling") = -1,
        py::arg("drop_samples_on_prune") = -1,
        py::arg("approximate_solutions") = -1,
        py::arg("strict_queue_ordering") = -1,
        py::arg("cascading_rewirings") = -1,
        py::arg("initial_inflation_factor") = -1.0,
        py::arg("inflation_scaling_parameter") = -1.0,
        py::arg("truncation_scaling_parameter") = -1.0,
        py::arg("allowed_failed_sampling_attempts") = -1);

    module.def("check_config_collision",
        [](const rbf::Robot& robot,
           const std::vector<rbf::Obstacle>& obstacles,
           const std::vector<double>& q,
           double collision_tolerance) {
            rbf::CollisionChecker checker(robot, rbf::Scene(obstacles));
            checker.set_collision_tolerance(collision_tolerance);
            return checker.check_config(eigen_vector_from_list(q));
        },
        py::arg("robot"),
        py::arg("obstacles"),
        py::arg("q"),
        py::arg("collision_tolerance") = 0.0);
}
