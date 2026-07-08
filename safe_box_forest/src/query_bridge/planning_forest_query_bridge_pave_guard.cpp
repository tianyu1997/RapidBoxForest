#include "planning_forest_query_bridge_pave_guard.h"

#include <SBF/runtime.h>

namespace rbf {

bool query_bridge_skip_graph_pave_for_partition_native(bool partition_native,
                                                       StageContext& context,
                                                       const char* counter_name) {
    if (!partition_native) {
        return false;
    }
    if (counter_name != nullptr && counter_name[0] != '\0') {
        context.diagnostics().add_counter(counter_name);
    }
    return true;
}

}  // namespace rbf
