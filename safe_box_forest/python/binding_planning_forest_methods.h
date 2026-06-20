#pragma once

#include <SBF/sbf.h>

#include "binding_planning_forest_database_methods.h"
#include "binding_planning_forest_debug_methods.h"
#include "binding_utils.h"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace py = pybind11;

namespace rbf::python_binding {

inline void register_planning_forest_methods(py::module_& module) {
    auto forest_class = py::class_<rbf::RBFPlanningForest>(module, "RBFPlanningForest")
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
        .def("boxes", [](const rbf::RBFPlanningForest& forest) { return forest.boxes(); })
        .def("raw_boxes", [](const rbf::RBFPlanningForest& forest) { return forest.raw_boxes(); })
        .def("audit_robot", &rbf::RBFPlanningForest::audit_robot, py::return_value_policy::reference_internal)
        .def("adjacency", [](const rbf::RBFPlanningForest& forest) { return forest.adjacency(); })
        .def("segment_edges", [](const rbf::RBFPlanningForest& forest) { return forest.segment_edges(); })
        .def("last_build_profile", &rbf::RBFPlanningForest::last_build_profile, py::return_value_policy::reference_internal);

    register_planning_forest_database_methods(forest_class);
    register_planning_forest_debug_methods(forest_class);
}

}  // namespace rbf::python_binding
