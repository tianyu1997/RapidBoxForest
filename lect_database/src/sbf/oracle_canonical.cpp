#include "oracle_canonical.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rbf {
namespace {

double normalized_joint_value(double value, double origin) {
    double normalized = std::fmod(value - origin, TWO_PI);
    if (normalized < 0.0) {
        normalized += TWO_PI;
    }
    return normalized + origin;
}

double canonical_value_in_sector(const JointSymmetry& symmetry,
                                 double value,
                                 lect_database::SectorId sector) {
    const int normalized_sector = normalize_sector(sector);
    const double sector_lo =
        symmetry.canonical_lo + static_cast<double>(normalized_sector) * symmetry.period;
    const double sector_hi = sector_lo + symmetry.period;
    double shifted = value;
    while (shifted < sector_lo - 1e-12) {
        shifted += TWO_PI;
    }
    while (shifted > sector_hi + 1e-12) {
        shifted -= TWO_PI;
    }
    return shifted - static_cast<double>(normalized_sector) * symmetry.period;
}

std::optional<Interval> canonical_interval_for_sector(const JointSymmetry& symmetry,
                                                      const Interval& interval,
                                                      lect_database::SectorId sector) {
    const double lo = canonical_value_in_sector(symmetry, interval.lo, sector);
    const double hi = canonical_value_in_sector(symmetry, interval.hi, sector);
    if (lo > hi + 1e-12) {
        return std::nullopt;
    }
    if (hi < symmetry.canonical_lo - 1e-12 ||
        lo > symmetry.canonical_hi + 1e-12) {
        return std::nullopt;
    }
    Interval canonical{
        std::max(lo, symmetry.canonical_lo),
        std::min(hi, symmetry.canonical_hi)};
    if (canonical.lo > canonical.hi + 1e-12) {
        return std::nullopt;
    }
    if (canonical.lo > canonical.hi) {
        canonical.lo = canonical.hi = 0.5 * (canonical.lo + canonical.hi);
    }
    return canonical;
}

}  // namespace

bool database_uses_canonical_symmetry(const lect_database::LectDatabase& database) {
    const auto& identity = database.identity();
    return identity.canonical_mode &&
           lect_database::uses_joint_symmetry_native(identity.symmetry_descriptor);
}

std::optional<JointSymmetry> primary_database_symmetry(
    const Robot& robot,
    const lect_database::LectDatabase& database) {
    if (!database_uses_canonical_symmetry(database)) {
        return std::nullopt;
    }
    auto symmetries = detect_joint_symmetries(robot);
    if (symmetries.empty()) {
        return std::nullopt;
    }
    JointSymmetry symmetry = symmetries.front();
    if (symmetry.type == JointSymmetryType::NONE ||
        symmetry.period <= 0.0 ||
        symmetry.joint_index < 0) {
        return std::nullopt;
    }
    return symmetry;
}

bool active_tree_is_primary_canonical_sector(
    const Robot& robot,
    const lect_database::LectDatabase& database) {
    auto symmetry = primary_database_symmetry(robot, database);
    if (!symmetry) {
        return false;
    }
    const std::size_t joint_index = static_cast<std::size_t>(symmetry->joint_index);
    const auto& root = database.root_intervals();
    if (joint_index >= root.size()) {
        return false;
    }
    const Interval& interval = root[joint_index];
    return interval.lo >= symmetry->canonical_lo - 1e-12 &&
           interval.hi <= symmetry->canonical_hi + 1e-12;
}

int normalize_sector(lect_database::SectorId sector) {
    return ((static_cast<int>(sector) % 4) + 4) % 4;
}

lect_database::SectorId interval_sector_for_value(
    const JointSymmetry& symmetry,
    double value) {
    const double rel = normalized_joint_value(value, symmetry.canonical_lo) -
        symmetry.canonical_lo;
    const int n_sectors = std::max(1, static_cast<int>(std::round(TWO_PI / symmetry.period)));
    const int boundary = static_cast<int>(std::llround(rel / symmetry.period));
    if (boundary >= 0 && boundary <= n_sectors) {
        const double boundary_value = static_cast<double>(boundary) * symmetry.period;
        if (std::abs(rel - boundary_value) <= 1e-14) {
            if (boundary <= 0 || boundary >= n_sectors) {
                return 0;
            }
            return std::clamp(boundary - 1, 0, n_sectors - 1);
        }
    }
    int sector = static_cast<int>(std::floor(rel / symmetry.period));
    if (sector >= n_sectors) {
        sector = 0;
    }
    return std::clamp(sector, 0, n_sectors - 1);
}

std::optional<lect_database::SectorId> interval_sector_for_interval(
    const JointSymmetry& symmetry,
    const Interval& interval) {
    const double width = interval.hi - interval.lo;
    if (width < -1e-14) {
        return std::nullopt;
    }
    if (width <= 1e-14) {
        return interval_sector_for_value(symmetry, interval.center());
    }
    if (width > symmetry.period + 1e-12) {
        return std::nullopt;
    }
    const double eps = std::min(0.25 * width, std::max(1e-12, width * 1e-12));
    const auto sector_lo = interval_sector_for_value(symmetry, interval.lo + eps);
    const auto sector_hi = interval_sector_for_value(symmetry, interval.hi - eps);
    if (normalize_sector(sector_lo) != normalize_sector(sector_hi)) {
        return std::nullopt;
    }
    return sector_lo;
}

std::pair<lect_database::SectorId, double> canonicalize_value_no_snap(
    const JointSymmetry& symmetry,
    double value) {
    const lect_database::SectorId sector = interval_sector_for_value(symmetry, value);
    return {sector, canonical_value_in_sector(symmetry, value, sector)};
}

Interval map_canonical_interval_to_sector(
    const JointSymmetry& symmetry,
    const Interval& canonical,
    lect_database::SectorId sector,
    const Interval& limit,
    double reference_value) {
    double base_lo = 0.0;
    double base_hi = 0.0;
    symmetry.map_interval(canonical.lo, canonical.hi, normalize_sector(sector), base_lo, base_hi);

    Interval best{1.0, 0.0};
    double best_score = std::numeric_limits<double>::infinity();
    for (int shift = -2; shift <= 2; ++shift) {
        const double lo = base_lo + static_cast<double>(shift) * TWO_PI;
        const double hi = base_hi + static_cast<double>(shift) * TWO_PI;
        const double clipped_lo = std::max(lo, limit.lo);
        const double clipped_hi = std::min(hi, limit.hi);
        if (clipped_lo > clipped_hi + 1e-12) {
            continue;
        }
        const bool contains_reference =
            reference_value >= clipped_lo - 1e-9 && reference_value <= clipped_hi + 1e-9;
        const double distance_to_reference =
            contains_reference
                ? 0.0
                : std::min(std::abs(reference_value - clipped_lo),
                           std::abs(reference_value - clipped_hi));
        const double score = (contains_reference ? -1.0 : 0.0) + distance_to_reference;
        if (score < best_score) {
            best_score = score;
            best = {clipped_lo, clipped_hi};
        }
    }
    return best;
}

std::optional<lect_database::SectorId> sector_for_reflected_interval_containing_seed(
    const JointSymmetry& symmetry,
    const Interval& canonical,
    const Interval& limit,
    double seed_value) {
    const int n_sectors = std::max(1, static_cast<int>(std::round(TWO_PI / symmetry.period)));
    for (lect_database::SectorId sector = 0; sector < n_sectors; ++sector) {
        const Interval native = map_canonical_interval_to_sector(
            symmetry,
            canonical,
            sector,
            limit,
            seed_value);
        if (native.lo <= native.hi + 1e-12 && native.contains(seed_value, 1e-9)) {
            return sector;
        }
    }
    return std::nullopt;
}

CanonicalEvidenceFrame canonical_evidence_frame_for_intervals(
    const Robot& robot,
    const lect_database::LectDatabase& database,
    const std::vector<Interval>& query_intervals) {
    CanonicalEvidenceFrame frame;
    frame.lookup_intervals = query_intervals;
    frame.symmetry = primary_database_symmetry(robot, database);
    if (!frame.symmetry || query_intervals.empty()) {
        return frame;
    }
    const JointSymmetry& symmetry = *frame.symmetry;
    const std::size_t joint_index = static_cast<std::size_t>(symmetry.joint_index);
    if (joint_index >= query_intervals.size()) {
        frame.symmetry.reset();
        return frame;
    }
    const auto sector = interval_sector_for_interval(symmetry, query_intervals[joint_index]);
    if (!sector) {
        frame.lookup_intervals = query_intervals;
        frame.sector = lect_database::kPrimarySector;
        frame.valid = false;
        return frame;
    }
    frame.sector = *sector;
    const auto canonical_interval = canonical_interval_for_sector(
        symmetry,
        query_intervals[joint_index],
        frame.sector);
    if (!canonical_interval) {
        frame.lookup_intervals = query_intervals;
        frame.sector = lect_database::kPrimarySector;
        frame.valid = false;
        return frame;
    }
    frame.lookup_intervals[joint_index] = *canonical_interval;
    if (frame.lookup_intervals[joint_index].lo > frame.lookup_intervals[joint_index].hi) {
        frame.lookup_intervals = query_intervals;
        frame.sector = lect_database::kPrimarySector;
        frame.valid = false;
        frame.symmetry.reset();
    }
    return frame;
}

bool intervals_straddle_sector_boundary(
    const JointSymmetry& symmetry,
    const std::vector<Interval>& intervals) {
    const std::size_t joint_index = static_cast<std::size_t>(symmetry.joint_index);
    if (joint_index >= intervals.size()) {
        return false;
    }
    return !interval_sector_for_interval(symmetry, intervals[joint_index]).has_value();
}

}  // namespace rbf
