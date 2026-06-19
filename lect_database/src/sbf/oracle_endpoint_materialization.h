#pragma once

#include <LECTDatabase/sbf/oracle.h>

namespace rbf {

EndpointSourceConfig hifk_config_for_materialization(
    const DatabaseBoxOracle& oracle,
    OracleNodeId node,
    EndpointSourceConfig config);

}  // namespace rbf
