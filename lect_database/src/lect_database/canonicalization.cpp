#include <rbf/lect_database/canonicalization.h>

#include <sbf/core/joint_symmetry.h>

namespace rbf::lect_database {
namespace {

const JointSymmetry* primary_joint_symmetry(const Robot& robot,
                                            bool canonical_mode,
                                            std::string_view symmetry_descriptor,
                                            std::vector<JointSymmetry>& owned) {
    if (!canonical_mode || !uses_joint_symmetry_native(symmetry_descriptor)) {
        return nullptr;
    }
    owned = detect_joint_symmetries(robot);
    if (owned.empty()) {
        return nullptr;
    }
    const JointSymmetry& symmetry = owned.front();
    if (symmetry.type == JointSymmetryType::NONE || symmetry.period <= 0.0) {
        return nullptr;
    }
    return &symmetry;
}

}  // namespace

bool uses_joint_symmetry_native(std::string_view symmetry_descriptor) {
    return symmetry_descriptor == kJointSymmetryNativeV1;
}

std::vector<Interval> canonical_root_intervals_for_robot(const Robot& robot,
                                                         bool canonical_mode,
                                                         std::string_view symmetry_descriptor) {
    std::vector<Interval> root_intervals = robot.joint_limits().limits;
    if (root_intervals.empty()) {
        return root_intervals;
    }

    std::vector<JointSymmetry> symmetries;
    const JointSymmetry* symmetry = primary_joint_symmetry(robot, canonical_mode, symmetry_descriptor, symmetries);
    if (symmetry == nullptr || symmetry->joint_index < 0 ||
        symmetry->joint_index >= static_cast<int>(root_intervals.size())) {
        return root_intervals;
    }

    root_intervals[static_cast<std::size_t>(symmetry->joint_index)].lo = symmetry->canonical_lo;
    root_intervals[static_cast<std::size_t>(symmetry->joint_index)].hi = symmetry->canonical_hi;
    return root_intervals;
}

SectorId canonicalize_configuration_for_robot(const Robot& robot,
                                              bool canonical_mode,
                                              std::string_view symmetry_descriptor,
                                              std::span<double> values) {
    if (values.empty()) {
        return kPrimarySector;
    }

    std::vector<JointSymmetry> symmetries;
    const JointSymmetry* symmetry = primary_joint_symmetry(robot, canonical_mode, symmetry_descriptor, symmetries);
    if (symmetry == nullptr || symmetry->joint_index < 0 ||
        symmetry->joint_index >= static_cast<int>(values.size())) {
        return kPrimarySector;
    }

    double canonical_value = values[static_cast<std::size_t>(symmetry->joint_index)];
    const SectorId sector = symmetry->canonicalize(canonical_value, canonical_value);
    values[static_cast<std::size_t>(symmetry->joint_index)] = canonical_value;
    return sector;
}

}  // namespace rbf::lect_database