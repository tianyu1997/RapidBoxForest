#pragma once

#include "binding_baseline_bitstar_functions.h"
#include "binding_baseline_cspace_functions.h"
#include "binding_baseline_prm_functions.h"
#include "binding_baseline_rrt_functions.h"
#include "binding_baseline_utility_functions.h"

#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace rbf::python_binding {

inline void register_baseline_planner_functions(py::module_& module) {
    register_baseline_rrt_functions(module);

    register_baseline_prm_functions(module);

    register_baseline_utility_functions(module);

    register_baseline_bitstar_functions(module);

    register_baseline_cspace_functions(module);

}

}  // namespace rbf::python_binding
