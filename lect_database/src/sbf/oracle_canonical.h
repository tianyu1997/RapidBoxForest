#pragma once

#include <LECTDatabase/sbf/oracle.h>

#include <sbf/core/joint_symmetry.h>

#include <optional>
#include <utility>
#include <vector>

namespace rbf {

bool database_uses_canonical_symmetry(const lect_database::LectDatabase& database);

std::optional<JointSymmetry> primary_database_symmetry(
    const Robot& robot,
    const lect_database::LectDatabase& database);

bool active_tree_is_primary_canonical_sector(
    const Robot& robot,
    const lect_database::LectDatabase& database);

int normalize_sector(lect_database::SectorId sector);

lect_database::SectorId interval_sector_for_value(
    const JointSymmetry& symmetry,
    double value);

std::optional<lect_database::SectorId> interval_sector_for_interval(
    const JointSymmetry& symmetry,
    const Interval& interval);

std::pair<lect_database::SectorId, double> canonicalize_value_no_snap(
    const JointSymmetry& symmetry,
    double value);

Interval map_canonical_interval_to_sector(
    const JointSymmetry& symmetry,
    const Interval& canonical,
    lect_database::SectorId sector,
    const Interval& limit,
    double reference_value);

std::optional<lect_database::SectorId> sector_for_reflected_interval_containing_seed(
    const JointSymmetry& symmetry,
    const Interval& canonical,
    const Interval& limit,
    double seed_value);

struct CanonicalEvidenceFrame {
    std::vector<Interval> lookup_intervals;
    lect_database::SectorId sector = lect_database::kPrimarySector;
    std::optional<JointSymmetry> symmetry;
    bool valid = true;
};

CanonicalEvidenceFrame canonical_evidence_frame_for_intervals(
    const Robot& robot,
    const lect_database::LectDatabase& database,
    const std::vector<Interval>& query_intervals);

bool intervals_straddle_sector_boundary(
    const JointSymmetry& symmetry,
    const std::vector<Interval>& intervals);

}  // namespace rbf
