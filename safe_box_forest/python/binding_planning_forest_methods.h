#pragma once

#include <SBF/safe_box_forest.h>

#include "binding_planning_forest_build_methods.h"
#include "binding_planning_forest_database_methods.h"
#include "binding_planning_forest_query_methods.h"

#if defined(SBF_DIAGNOSTIC_API) && SBF_DIAGNOSTIC_API && \
    defined(SBF_PYTHON_DEBUG_METHODS) && SBF_PYTHON_DEBUG_METHODS
#include "diagnostic/binding_planning_forest_debug_methods.h"
#include "diagnostic/binding_planning_forest_diagnostic_facade_methods.h"
#endif

#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace rbf::python_binding {

inline void register_planning_forest_methods(py::module_& module) {
    auto forest_class = py::class_<rbf::RBFPlanningForest>(module, "RBFPlanningForest")
        .def(py::init<rbf::Robot, rbf::RBFPlanningConfig>(),
             py::arg("robot"),
             py::arg("config") = rbf::RBFPlanningConfig{})
        .def("clear_forest", &rbf::RBFPlanningForest::clear_forest)
        .def("boxes", [](const rbf::RBFPlanningForest& forest) { return forest.boxes(); })
        .def("raw_boxes", [](const rbf::RBFPlanningForest& forest) { return forest.raw_boxes(); })
        .def("audit_robot", &rbf::RBFPlanningForest::audit_robot, py::return_value_policy::reference_internal)
        .def("adjacency", [](const rbf::RBFPlanningForest& forest) { return forest.adjacency(); })
        .def("segment_edges", [](const rbf::RBFPlanningForest& forest) { return forest.segment_edges(); })
        .def("last_build_profile", &rbf::RBFPlanningForest::last_build_profile, py::return_value_policy::reference_internal);

    register_planning_forest_build_methods(forest_class);
    register_planning_forest_query_methods(forest_class);
    register_planning_forest_database_methods(forest_class);
#if defined(SBF_DIAGNOSTIC_API) && SBF_DIAGNOSTIC_API && \
    defined(SBF_PYTHON_DEBUG_METHODS) && SBF_PYTHON_DEBUG_METHODS
    register_planning_forest_diagnostic_facade_methods(forest_class);
    register_planning_forest_debug_methods(forest_class);
#endif
}

}  // namespace rbf::python_binding
