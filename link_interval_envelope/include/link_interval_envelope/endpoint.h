#pragma once
/// @file endpoint.h
/// @brief Public endpoint-envelope configuration and one-shot computation API.

#include <sbf/envelope/endpoint_source.h>
#include <sbf/envelope/gcpc_source.h>

namespace link_interval_envelope {

using ::rbf::EndpointIAABBResult;
using ::rbf::EndpointSafetyLevel;
using ::rbf::EndpointSource;
using ::rbf::EndpointSourceConfig;
using ::rbf::GcpcCache;
using ::rbf::HifkSplitStrategy;

using ::rbf::compute_endpoint_iaabb;
using ::rbf::endpoint_safety_is_certified;
using ::rbf::endpoint_source_default_safety;
using ::rbf::endpoint_source_name;
using ::rbf::hull_endpoint_iaabbs;
using ::rbf::recommend_hifk_depth;
using ::rbf::source_can_serve;
using ::rbf::source_channel;

}  // namespace link_interval_envelope
