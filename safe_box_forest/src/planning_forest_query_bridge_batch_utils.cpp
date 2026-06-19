#include "planning_forest_query_bridge_batch_utils.h"

#include <string>

namespace rbf {

std::string query_bridge_task_key(std::size_t index, const std::string& suffix) {
    return "query_bridge.batch_task." + std::to_string(index) + "." + suffix;
}

}  // namespace rbf
