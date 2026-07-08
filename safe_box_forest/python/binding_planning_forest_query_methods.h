#pragma once

#include <SBF/safe_box_forest.h>

#include "binding_utils.h"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <stdexcept>
#include <vector>

namespace py = pybind11;

namespace rbf::python_binding {

inline void register_planning_forest_query_methods(
    py::class_<rbf::RBFPlanningForest>& forest_class) {
    forest_class
        .def("query",
             [](const rbf::RBFPlanningForest& forest,
                const std::vector<double>& start,
                const std::vector<double>& goal,
                const rbf::RBFQueryRuntimeOptions& options) {
                 return forest.query(eigen_vector_from_list(start),
                                     eigen_vector_from_list(goal),
                                     options);
             },
             py::arg("start"),
             py::arg("goal"),
             py::arg("options") = rbf::RBFQueryRuntimeOptions{})
        .def("bridge_query",
             [](rbf::RBFPlanningForest& forest,
                const std::vector<double>& start,
                const std::vector<double>& goal) {
                 return forest.bridge_query(eigen_vector_from_list(start),
                                            eigen_vector_from_list(goal));
             },
             py::arg("start"),
             py::arg("goal"))
        .def("bridge_query_known_needed",
             [](rbf::RBFPlanningForest& forest,
                const std::vector<double>& start,
                const std::vector<double>& goal) {
                 return forest.bridge_query_known_needed(eigen_vector_from_list(start),
                                                         eigen_vector_from_list(goal));
             },
             py::arg("start"),
             py::arg("goal"))
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
             py::arg("global_query_indices") = std::vector<int>{});
}

}  // namespace rbf::python_binding
