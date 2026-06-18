#pragma once
/// @file fk_state.h
/// @brief Compatibility facade for interval-FK state and helpers.

#include <sbf/core/fk_state.h>

namespace link_interval_envelope {

using ::rbf::FKState;

using ::rbf::build_joint_interval;
using ::rbf::compute_fk_full;
using ::rbf::compute_fk_incremental;
using ::rbf::extract_endpoint_iaabbs;
using ::rbf::extract_link_aabbs;
using ::rbf::update_fk_inplace;

}  // namespace link_interval_envelope
