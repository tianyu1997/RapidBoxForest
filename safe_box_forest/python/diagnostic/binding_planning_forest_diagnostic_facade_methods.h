#pragma once

#include <SBF/safe_box_forest.h>

#include <SBF/diagnostic_result.h>
#include <SBF/subtractive_build_config.h>

#include "../binding_utils.h"
#include "../binding_oracle_utils.h"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace py = pybind11;

namespace rbf::python_binding {

inline void register_planning_forest_diagnostic_facade_methods(
    py::class_<rbf::RBFPlanningForest>& forest_class) {
    forest_class
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
                 return forest.build_subtractive(obstacle_groups,
                                                 eigen_vectors_from_lists(seeds),
                                                 options);
             },
             py::arg("obstacle_groups"),
             py::arg("seeds"),
             py::arg("options") = rbf::SubtractiveBuildOptions{})
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
        .def("add_obstacle_and_rebuild",
             &rbf::RBFPlanningForest::add_obstacle_and_rebuild,
             py::arg("obstacle"))
        .def("add_obstacles_and_rebuild",
             &rbf::RBFPlanningForest::add_obstacles_and_rebuild,
             py::arg("obstacles"))
        .def("connect_update_segment_fallback",
             &rbf::RBFPlanningForest::connect_update_segment_fallback)
        .def("connect_update_endpoint_segment_fallback",
             [](rbf::RBFPlanningForest& forest,
                const std::vector<double>& start,
                const std::vector<double>& goal) {
                 return forest.connect_update_endpoint_segment_fallback(
                     eigen_vector_from_list(start),
                     eigen_vector_from_list(goal));
             },
             py::arg("start"),
             py::arg("goal"))
        .def("remove_obstacle_and_regrow",
             &rbf::RBFPlanningForest::remove_obstacle_and_regrow,
             py::arg("obstacle_index"))
        .def("remove_obstacle_suffix_and_regrow",
             &rbf::RBFPlanningForest::remove_obstacle_suffix_and_regrow,
             py::arg("target_obstacle_count"));
}

}  // namespace rbf::python_binding
