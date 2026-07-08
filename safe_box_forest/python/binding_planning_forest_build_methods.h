#pragma once

#include <SBF/safe_box_forest.h>

#include "binding_utils.h"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <vector>

namespace py = pybind11;

namespace rbf::python_binding {

inline void register_planning_forest_build_methods(
    py::class_<rbf::RBFPlanningForest>& forest_class) {
    forest_class
        .def("build",
             [](rbf::RBFPlanningForest& forest,
                const std::vector<double>& start,
                const std::vector<double>& goal,
                const std::vector<rbf::Obstacle>& obstacles) {
                 return forest.build(eigen_vector_from_list(start),
                                     eigen_vector_from_list(goal),
                                     obstacles);
             },
             py::arg("start"),
             py::arg("goal"),
             py::arg("obstacles") = std::vector<rbf::Obstacle>{})
        .def("build_coverage",
             [](rbf::RBFPlanningForest& forest,
                const std::vector<rbf::Obstacle>& obstacles,
                const std::vector<std::vector<double>>& seeds) {
                 return forest.build_coverage(obstacles, eigen_vectors_from_lists(seeds));
             },
             py::arg("obstacles"),
             py::arg("seeds"))
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
             py::arg("config") = rbf::AdaptiveLeafSweepConfig{});
}

}  // namespace rbf::python_binding
