#pragma once

#include <SBF/safe_box_forest.h>

#include "binding_debug_chain_pave_methods.h"
#include "binding_debug_endpoint_oracle_methods.h"
#include "binding_debug_find_free_box_methods.h"
#include "binding_debug_path_cover_methods.h"

#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace rbf::python_binding {

inline void register_planning_forest_debug_methods(
    py::class_<rbf::RBFPlanningForest>& forest_class) {
    register_debug_chain_pave_methods(forest_class);
    register_debug_endpoint_oracle_methods(forest_class);
    register_debug_find_free_box_methods(forest_class);
    register_debug_path_cover_methods(forest_class);
}

}  // namespace rbf::python_binding
