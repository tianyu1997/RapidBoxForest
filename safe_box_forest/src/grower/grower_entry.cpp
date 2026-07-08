#include <SBF/grower.h>

#include <SBF/box_graph.h>

namespace rbf {

bool RrtGrower::connected(const std::vector<BoxNode>& boxes) const {
    if (boxes.size() <= 1) {
        return true;
    }
    return find_islands(compute_adjacency(boxes, config_.adjacency_tolerance)).size() <= 1;
}

std::unique_ptr<IGrower> make_grower(BoxOracle& oracle, const GrowerConfig& config) {
    if (config.mode == GrowerConfig::Mode::Frontwave) {
        return std::make_unique<FrontwaveGrower>(oracle, config);
    }
    return std::make_unique<RrtGrower>(oracle, config);
}

}  // namespace rbf
