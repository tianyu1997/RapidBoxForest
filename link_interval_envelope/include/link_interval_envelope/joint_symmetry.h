#pragma once
/// @file joint_symmetry.h
/// @brief Compatibility facade for robot joint-symmetry detection.

#include <sbf/core/joint_symmetry.h>

namespace link_interval_envelope {

using ::rbf::JointSymmetry;
using ::rbf::JointSymmetryType;

using ::rbf::detect_joint_symmetries;

}  // namespace link_interval_envelope
