#pragma once

#include <SBF/grower.h>
#include <SBF/oracle.h>

#include <algorithm>
#include <limits>
#include <string>

namespace rbf {

inline double box_gap_squared(const BoxNode& lhs, const BoxNode& rhs) {
    const int nd = lhs.n_dims();
    if (nd != rhs.n_dims()) {
        return std::numeric_limits<double>::infinity();
    }
    double gap_sq = 0.0;
    for (int dim = 0; dim < nd; ++dim) {
        double gap = 0.0;
        if (lhs.joint_intervals[dim].hi < rhs.joint_intervals[dim].lo) {
            gap = rhs.joint_intervals[dim].lo - lhs.joint_intervals[dim].hi;
        } else if (rhs.joint_intervals[dim].hi < lhs.joint_intervals[dim].lo) {
            gap = lhs.joint_intervals[dim].lo - rhs.joint_intervals[dim].hi;
        }
        gap_sq += gap * gap;
    }
    return gap_sq;
}

inline double interval_point_gap(const Interval& interval, double value) {
    if (value < interval.lo) {
        return interval.lo - value;
    }
    if (value > interval.hi) {
        return value - interval.hi;
    }
    return 0.0;
}

inline double intervals_point_gap(const std::vector<Interval>& intervals,
                                  const Eigen::Ref<const Eigen::VectorXd>& point) {
    if (point.size() != static_cast<int>(intervals.size())) {
        return std::numeric_limits<double>::infinity();
    }
    double gap = 0.0;
    for (int dim = 0; dim < point.size(); ++dim) {
        gap = std::max(gap, interval_point_gap(intervals[static_cast<std::size_t>(dim)], point[dim]));
    }
    return gap;
}

inline bool intervals_contain_point(const std::vector<Interval>& intervals,
                                    const Eigen::Ref<const Eigen::VectorXd>& point,
                                    double tolerance) {
    return intervals_point_gap(intervals, point) <= tolerance;
}

inline double box_point_gap(const BoxNode& box, const Eigen::Ref<const Eigen::VectorXd>& point) {
    return intervals_point_gap(box.joint_intervals, point);
}

inline bool box_contains_box_exact(const BoxNode& outer, const BoxNode& inner) {
    if (outer.n_dims() != inner.n_dims()) {
        return false;
    }
    for (int dim = 0; dim < outer.n_dims(); ++dim) {
        if (outer.joint_intervals[dim].lo > inner.joint_intervals[dim].lo ||
            outer.joint_intervals[dim].hi < inner.joint_intervals[dim].hi) {
            return false;
        }
    }
    return outer.id != inner.id;
}

inline bool box_contains_point_exact(const BoxNode& box, const Eigen::Ref<const Eigen::VectorXd>& point) {
    if (box.n_dims() != point.size()) {
        return false;
    }
    for (int dim = 0; dim < box.n_dims(); ++dim) {
        if (point[dim] < box.joint_intervals[dim].lo ||
            point[dim] > box.joint_intervals[dim].hi) {
            return false;
        }
    }
    return true;
}

inline bool point_covered_by_existing_box(const std::vector<BoxNode>& boxes,
                                          const Eigen::Ref<const Eigen::VectorXd>& point) {
    return std::any_of(boxes.begin(), boxes.end(), [&](const BoxNode& box) {
        return box_contains_point_exact(box, point);
    });
}

inline bool allow_box_commit(BoxOracle& oracle,
                             FindFreeBoxResult& result,
                             BoxCommitPolicy policy,
                             StageContext& context) {
    if (result.validation_detail.safety_status == BoxSafetyStatus::CertifiedFree) {
        return true;
    }
    if (result.validation_detail.safety_status != BoxSafetyStatus::ProvisionalFree) {
        context.diagnostics().add_counter("grower.commit_rejected_unknown_status");
        return false;
    }
    if (policy == BoxCommitPolicy::CommitCertifiedOnly) {
        context.diagnostics().add_counter("grower.commit_rejected_provisional");
        return false;
    }
    if (policy == BoxCommitPolicy::CommitProvisionalAllowed) {
        context.diagnostics().add_counter("grower.commit_provisional_allowed");
        return true;
    }
    if (policy == BoxCommitPolicy::AuditBeforeCommit) {
        context.diagnostics().add_counter("grower.commit_audit_attempted");
        if (oracle.validate_intervals(result.intervals)) {
            result.validation_detail.safety_status = BoxSafetyStatus::CertifiedFree;
            result.validation_detail.strict_audit_required = false;
            context.diagnostics().add_counter("grower.commit_audit_success");
            return true;
        }
        context.diagnostics().add_counter("grower.commit_audit_failed");
        return false;
    }
    return false;
}

inline void set_max_diagnostic(StageContext& context, const std::string& key, double value) {
    context.diagnostics().set_value(key, std::max(context.diagnostics().value(key), value));
}

inline void record_committed_box_stats(StageContext& context, const BoxNode& box) {
    context.diagnostics().add_counter("grower.committed_box_volume_sum", box.volume);
    set_max_diagnostic(context, "grower.committed_box_volume_max", box.volume);
    if (box.safety_status == BoxSafetyStatus::CertifiedFree) {
        context.diagnostics().add_counter("grower.certified_boxes_committed");
        context.diagnostics().add_counter("grower.certified_box_volume_sum", box.volume);
    } else if (box.safety_status == BoxSafetyStatus::ProvisionalFree) {
        context.diagnostics().add_counter("grower.provisional_boxes_committed");
        context.diagnostics().add_counter("grower.provisional_box_volume_sum", box.volume);
    }
    if (box.strict_audit_required) {
        context.diagnostics().add_counter("grower.strict_audit_required_boxes");
        context.diagnostics().add_counter("grower.strict_audit_required_volume_sum", box.volume);
    }
}

inline void finalize_result(GrowerResult& result, double adjacency_tol) {
    result.adjacency = compute_adjacency(result.boxes, adjacency_tol);
    auto islands = find_islands(result.adjacency);
    result.adjacency_islands = static_cast<int>(islands.size());
    result.all_connected = islands.size() <= 1;
    result.adjacency_largest_island = 0;
    for (const auto& island : islands) {
        result.adjacency_largest_island = std::max(result.adjacency_largest_island, static_cast<int>(island.size()));
    }
}

inline OracleNodeId find_leaf_containing(BoxOracle& oracle, const Eigen::Ref<const Eigen::VectorXd>& q) {
    if (q.size() != oracle.n_dims() || !oracle.contains_point(oracle.root_node(), q)) {
        return kInvalidOracleNodeId;
    }
    OracleNodeId node = oracle.root_node();
    while (!oracle.is_leaf(node)) {
        node = oracle.child_containing_point(node, q);
        if (node == kInvalidOracleNodeId) {
            break;
        }
    }
    return node;
}

}  // namespace rbf
