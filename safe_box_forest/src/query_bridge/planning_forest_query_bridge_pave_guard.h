#pragma once

#include <SBF/runtime_fwd.h>

namespace rbf {

bool query_bridge_skip_graph_pave_for_partition_native(bool partition_native,
                                                       StageContext& context,
                                                       const char* counter_name);

}  // namespace rbf
