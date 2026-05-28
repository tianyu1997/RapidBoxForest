#pragma once

#include <rbf/core/robot.h>
#include <rbf/lect_database/types.h>

#include <span>
#include <string_view>

namespace rbf::lect_database {

inline constexpr std::string_view kJointSymmetryNativeV1 = "joint_symmetry_native_v1";

bool uses_joint_symmetry_native(std::string_view symmetry_descriptor);

std::vector<Interval> canonical_root_intervals_for_robot(const Robot& robot,
                                                         bool canonical_mode,
                                                         std::string_view symmetry_descriptor);

SectorId canonicalize_configuration_for_robot(const Robot& robot,
                                              bool canonical_mode,
                                              std::string_view symmetry_descriptor,
                                              std::span<double> values);

}  // namespace rbf::lect_database