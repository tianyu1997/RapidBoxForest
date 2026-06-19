#include "oracle_endpoint_materialization.h"

namespace rbf {

EndpointSourceConfig hifk_config_for_materialization(
    const DatabaseBoxOracle& oracle,
    OracleNodeId node,
    EndpointSourceConfig config) {
    if (config.source != EndpointSource::HIFK) {
        return config;
    }

    const auto& split_policy = oracle.database().split_policy_descriptor();
    config.hifk_depth_offset = oracle.depth(node);
    config.hifk_min_split_width = split_policy.min_width;
    config.hifk_depth_dimensions.clear();
    config.hifk_root_intervals.clear();

    switch (split_policy.strategy) {
    case lect_database::SplitStrategy::RoundRobin:
        config.hifk_split_strategy = HifkSplitStrategy::RoundRobin;
        break;
    case lect_database::SplitStrategy::WidestRoot:
        config.hifk_split_strategy = HifkSplitStrategy::WidestRoot;
        config.hifk_root_intervals = oracle.root_intervals();
        break;
    case lect_database::SplitStrategy::AAFKVolumeMin:
        config.hifk_split_strategy = HifkSplitStrategy::FixedDepthSchedule;
        config.hifk_depth_dimensions = split_policy.depth_dimensions;
        break;
    }

    return config;
}

}  // namespace rbf
