#include "binding_adaptive_types.h"
#include "baseline/binding_baseline_planner_functions.h"
#include "binding_basic_types.h"
#include "binding_planner_core_types.h"
#include "binding_planning_forest_methods.h"
#include "binding_planning_option_types.h"
#if defined(SBF_DIAGNOSTIC_API) && SBF_DIAGNOSTIC_API && \
    defined(SBF_PYTHON_DEBUG_METHODS) && SBF_PYTHON_DEBUG_METHODS
#include "diagnostic/binding_diagnostic_types.h"
#endif

#include <pybind11/pybind11.h>

namespace py = pybind11;

using namespace rbf::python_binding;

PYBIND11_MODULE(_sbf_cpp, module) {
    module.doc() = "Standalone RBFPlanningForest bindings";
    module.attr("__version__") = "0.1.0";

    register_basic_types(module);
    register_planner_core_types(module);
    register_adaptive_types(module);
#if defined(SBF_DIAGNOSTIC_API) && SBF_DIAGNOSTIC_API && \
    defined(SBF_PYTHON_DEBUG_METHODS) && SBF_PYTHON_DEBUG_METHODS
    register_diagnostic_types(module);
#endif
    register_planning_option_types(module);
    register_planning_forest_methods(module);
    register_baseline_planner_functions(module);
}
