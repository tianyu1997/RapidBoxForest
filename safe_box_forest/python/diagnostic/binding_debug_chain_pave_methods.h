#pragma once

#include <SBF/safe_box_forest.h>

#include "binding_debug_chain_pave_result.h"

#include "../binding_utils.h"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <vector>

namespace py = pybind11;

namespace rbf::python_binding {

inline void register_debug_chain_pave_methods(py::class_<rbf::RBFPlanningForest>& forest_class) {
    forest_class
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
                 // Recursion depth bounds the bisection subdivisions (2^depth
                 // seeds). Derive it from the requested seed budget while
                 // keeping a sane cap.
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
                 return debug_chain_pave_result_to_python(res);
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
                 pave.commit_policy = commit_certified_only
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
                 return debug_chain_pave_result_to_python(res);
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
             py::arg("commit_certified_only") = true);
}

}  // namespace rbf::python_binding
