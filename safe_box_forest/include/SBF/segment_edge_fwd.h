#pragma once

#include <cstdint>
#include <vector>

namespace rbf {

enum class SegmentEdgeType : std::uint8_t {
    Unknown = 0,
    PointValidatedGap = 1,
    RRTConnector = 2,
    QueryBridge = 3,
    BoxCorridor = 4,
    PortalCorridor = 5,
    SegmentOBBCorridor = 6,
    RRTBridgeOBBCorridor = 7,
    TransitionOBBCorridor = 8,
};

enum class SegmentEdgeValidation : std::uint8_t {
    Unknown = 0,
    CollisionChecked = 1,
    ConservativeBoxChain = 2,
    ConservativeObbZonotope = 3,
};

struct SegmentEdge;
using SegmentEdgeList = std::vector<SegmentEdge>;

}  // namespace rbf
