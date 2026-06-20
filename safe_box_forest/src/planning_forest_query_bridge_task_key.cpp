#include "planning_forest_query_bridge_diagnostics.h"

#include <string>

namespace rbf {

std::string query_bridge_task_key(std::size_t index, const std::string& suffix) {
    return "query_bridge.batch_task." + std::to_string(index) + "." + suffix;
}

QueryBridgeTaskDiagnostics::QueryBridgeTaskDiagnostics(StageContext& context,
                                                       int task_index)
    : context_(context),
      task_index_(task_index) {}

void QueryBridgeTaskDiagnostics::add_counter(const std::string& suffix,
                                             double value) const {
    if (task_index_ < 0) {
        return;
    }
    context_.diagnostics().add_counter(
        query_bridge_task_key(static_cast<std::size_t>(task_index_), suffix),
        value);
}

void QueryBridgeTaskDiagnostics::set_value(const std::string& suffix,
                                           double value) const {
    if (task_index_ < 0) {
        return;
    }
    context_.diagnostics().set_value(
        query_bridge_task_key(static_cast<std::size_t>(task_index_), suffix),
        value);
}

}  // namespace rbf
