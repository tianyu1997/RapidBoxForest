#pragma once

#include <SBF/sbf.h>

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

namespace py = pybind11;

namespace rbf::python_binding {

inline std::vector<int> active_link_map_vec(const rbf::Robot& robot) {
    const int* map = robot.active_link_map();
    if (map == nullptr) return {};
    return std::vector<int>(map, map + robot.n_active_links());
}

inline std::vector<double> active_link_radii_vec(const rbf::Robot& robot) {
    const double* radii = robot.active_link_radii();
    if (radii == nullptr) return {};
    return std::vector<double>(radii, radii + robot.n_active_links());
}

inline Eigen::VectorXd eigen_vector_from_list(const std::vector<double>& values) {
    Eigen::VectorXd vector(static_cast<Eigen::Index>(values.size()));
    for (std::size_t i = 0; i < values.size(); ++i) {
        vector[static_cast<Eigen::Index>(i)] = values[i];
    }
    return vector;
}

inline std::vector<Eigen::VectorXd> eigen_vectors_from_lists(
    const std::vector<std::vector<double>>& values) {
    std::vector<Eigen::VectorXd> result;
    result.reserve(values.size());
    for (const auto& item : values) {
        result.push_back(eigen_vector_from_list(item));
    }
    return result;
}

inline std::vector<double> vector_to_list(const Eigen::VectorXd& values) {
    std::vector<double> result(static_cast<std::size_t>(values.size()));
    for (Eigen::Index i = 0; i < values.size(); ++i) {
        result[static_cast<std::size_t>(i)] = values[i];
    }
    return result;
}

inline std::vector<std::vector<double>> eigen_path_to_lists(
    const std::vector<Eigen::VectorXd>& path) {
    std::vector<std::vector<double>> result;
    result.reserve(path.size());
    for (const auto& waypoint : path) {
        result.push_back(vector_to_list(waypoint));
    }
    return result;
}

inline std::vector<rbf::Interval> intervals_from_pairs(
    const std::vector<std::vector<double>>& pairs) {
    std::vector<rbf::Interval> intervals;
    intervals.reserve(pairs.size());
    for (const auto& pair : pairs) {
        if (pair.size() != 2) {
            throw std::invalid_argument("each interval pair must have exactly two entries");
        }
        if (pair[1] < pair[0]) {
            throw std::invalid_argument("interval upper bound must be >= lower bound");
        }
        intervals.push_back(rbf::Interval{pair[0], pair[1]});
    }
    return intervals;
}

inline py::list interval_pairs_to_python(const std::vector<rbf::Interval>& intervals) {
    py::list result;
    for (const auto& interval : intervals) {
        py::list pair;
        pair.append(interval.lo);
        pair.append(interval.hi);
        result.append(std::move(pair));
    }
    return result;
}

inline bool intervals_contain_point_pybind(const std::vector<rbf::Interval>& intervals,
                                           const Eigen::Ref<const Eigen::VectorXd>& point,
                                           double tolerance) {
    if (intervals.size() != static_cast<std::size_t>(point.size())) {
        return false;
    }
    for (Eigen::Index dim = 0; dim < point.size(); ++dim) {
        const auto& interval = intervals[static_cast<std::size_t>(dim)];
        const double value = point[dim];
        if (value < interval.lo - tolerance || value > interval.hi + tolerance) {
            return false;
        }
    }
    return true;
}

inline bool interval_boxes_connected_pybind(const std::vector<rbf::Interval>& lhs,
                                            const std::vector<rbf::Interval>& rhs,
                                            double tolerance) {
    if (lhs.size() != rhs.size()) {
        return false;
    }
    int shared_dims = 0;
    int overlap_dims = 0;
    for (std::size_t dim = 0; dim < lhs.size(); ++dim) {
        const double overlap_lo = std::max(lhs[dim].lo, rhs[dim].lo);
        const double overlap_hi = std::min(lhs[dim].hi, rhs[dim].hi);
        if (overlap_hi < overlap_lo - tolerance) {
            return false;
        }
        if (overlap_hi - overlap_lo < tolerance) {
            shared_dims += 1;
        } else {
            overlap_dims += 1;
        }
    }
    return shared_dims >= 1 || overlap_dims == static_cast<int>(lhs.size());
}

inline std::vector<Eigen::VectorXd> densify_path_pybind(
    const std::vector<Eigen::VectorXd>& waypoints,
    double step) {
    std::vector<Eigen::VectorXd> samples;
    if (waypoints.empty()) {
        return samples;
    }
    if (waypoints.size() == 1) {
        samples.push_back(waypoints.front());
        return samples;
    }
    const double effective_step = std::max(step, 1e-12);
    for (std::size_t index = 1; index < waypoints.size(); ++index) {
        const Eigen::VectorXd& a = waypoints[index - 1];
        const Eigen::VectorXd& b = waypoints[index];
        const double length = (b - a).norm();
        const int n = std::max(1, static_cast<int>(std::ceil(length / effective_step)));
        for (int sample = 0; sample < n; ++sample) {
            const double u = static_cast<double>(sample) / static_cast<double>(n);
            samples.push_back((1.0 - u) * a + u * b);
        }
    }
    samples.push_back(waypoints.back());
    return samples;
}

inline py::dict oracle_validation_detail_to_python(const rbf::OracleValidationDetail& detail) {
    py::dict result;
    result["node"] = detail.node;
    result["depth"] = detail.depth;
    result["mode"] = static_cast<int>(detail.mode);
    result["validation"] = static_cast<int>(detail.validation);
    result["safety_status"] = static_cast<int>(detail.safety_status);
    result["collision_possible"] = detail.collision_possible;
    result["strict_audit_required"] = detail.strict_audit_required;
    result["endpoint_source"] = static_cast<int>(detail.endpoint_source);
    result["endpoint_is_safe"] = detail.endpoint_is_safe;
    result["endpoint_safety_level"] = static_cast<int>(detail.endpoint_safety_level);
    result["materialized"] = detail.materialized;
    result["changed_dim"] = detail.changed_dim;
    result["used_incremental_fk"] = detail.used_incremental_fk;
    result["used_source_incremental_state"] = detail.used_source_incremental_state;
    result["reused_fk"] = detail.reused_fk;
    result["reused_endpoint_cache"] = detail.reused_endpoint_cache;
    result["reused_external_evidence"] = detail.reused_external_evidence;
    result["endpoint_time_us"] = detail.endpoint_time_us;
    result["envelope_time_us"] = detail.envelope_time_us;
    result["candidate_dirty_count"] = detail.candidate_dirty_count;
    result["predh_rebuild_count"] = detail.predh_rebuild_count;
    result["aabb_overlap"] = detail.aabb_overlap;
    result["aabb_overlap_depth"] = detail.aabb_overlap_depth;
    result["aabb_overlap_volume_ratio"] = detail.aabb_overlap_volume_ratio;
    result["blocker_active_link_index"] = detail.blocker_active_link_index;
    result["blocker_link_id"] = detail.blocker_link_id;
    result["blocker_obstacle_id"] = detail.blocker_obstacle_id;
    result["blocker_stage"] = detail.blocker_stage;
    result["blocker_margin"] = detail.blocker_margin;
    result["blocker_overlap_depth"] = detail.blocker_overlap_depth;
    result["blocker_overlap_volume_ratio"] = detail.blocker_overlap_volume_ratio;
    py::list affected_joints;
    for (int joint : detail.blocker_affected_joints) {
        affected_joints.append(joint);
    }
    result["blocker_affected_joints"] = std::move(affected_joints);
    py::list blockers;
    for (const auto& blocker : detail.blockers) {
        py::dict item;
        item["active_link_index"] = blocker.active_link_index;
        item["link_id"] = blocker.link_id;
        item["obstacle_id"] = blocker.obstacle_id;
        item["stage"] = blocker.stage;
        item["margin"] = blocker.margin;
        item["overlap_depth"] = blocker.overlap_depth;
        item["overlap_volume_ratio"] = blocker.overlap_volume_ratio;
        py::list item_affected_joints;
        for (int joint : blocker.affected_joints) {
            item_affected_joints.append(joint);
        }
        item["affected_joints"] = std::move(item_affected_joints);
        blockers.append(std::move(item));
    }
    result["blockers"] = std::move(blockers);
    result["blocker_signature_hash"] = detail.blocker_signature_hash;
    result["occupied_certificate_checked"] = detail.occupied_certificate_checked;
    result["occupied_certificate_found"] = detail.occupied_certificate_found;
    result["occupied_witness_link_id"] = detail.occupied_witness_link_id;
    result["occupied_witness_obstacle_id"] = detail.occupied_witness_obstacle_id;
    result["occupied_witness_center_signed_distance"] =
        detail.occupied_witness_center_signed_distance;
    result["occupied_witness_motion_bound"] = detail.occupied_witness_motion_bound;
    return result;
}

inline py::dict oracle_counters_to_python(const rbf::OracleCounters& counters) {
    py::dict result;
    result["node_validations"] = counters.node_validations;
    result["interval_validations"] = counters.interval_validations;
    result["certified_free"] = counters.certified_free;
    result["certified_occupied"] = counters.certified_occupied;
    result["provisional_free"] = counters.provisional_free;
    result["collision_possible"] = counters.collision_possible;
    result["unsafe_free_rejected"] = counters.unsafe_free_rejected;
    result["validation_cache_hits"] = counters.validation_cache_hits;
    result["validation_cache_misses"] = counters.validation_cache_misses;
    result["materializations"] = counters.materializations;
    result["materialization_stored_endpoint"] = counters.materialization_stored_endpoint;
    result["materialization_skipped_endpoint_cache"] =
        counters.materialization_skipped_endpoint_cache;
    result["materialization_endpoint_time_us"] = counters.materialization_endpoint_time_us;
    result["materialization_endpoint_wall_time_us"] =
        counters.materialization_endpoint_wall_time_us;
    result["materialization_envelope_time_us"] = counters.materialization_envelope_time_us;
    result["validate_node_total_time_us"] = counters.validate_node_total_time_us;
    result["validate_node_preamble_time_us"] = counters.validate_node_preamble_time_us;
    result["validate_node_endpoint_path_time_us"] = counters.validate_node_endpoint_path_time_us;
    result["validate_node_classify_time_us"] = counters.validate_node_classify_time_us;
    result["validate_node_overhead_time_us"] = counters.validate_node_overhead_time_us;
    result["materialization_cache_lookup_time_us"] = counters.materialization_cache_lookup_time_us;
    result["materialization_cache_read_time_us"] = counters.materialization_cache_read_time_us;
    result["materialization_external_lookup_time_us"] =
        counters.materialization_external_lookup_time_us;
    result["materialization_external_read_time_us"] =
        counters.materialization_external_read_time_us;
    result["materialization_envelope_compute_time_us"] =
        counters.materialization_envelope_compute_time_us;
    result["materialization_envelope_read_time_us"] =
        counters.materialization_envelope_read_time_us;
    result["materialization_envelope_collision_time_us"] =
        counters.materialization_envelope_collision_time_us;
    result["materialization_incremental_fk"] = counters.materialization_incremental_fk;
    result["materialization_source_incremental_state"] =
        counters.materialization_source_incremental_state;
    result["materialization_reused_fk"] = counters.materialization_reused_fk;
    result["materialization_reused_endpoint_cache"] =
        counters.materialization_reused_endpoint_cache;
    result["materialization_reused_external_evidence"] =
        counters.materialization_reused_external_evidence;
    result["materialization_external_exact_hits"] = counters.materialization_external_exact_hits;
    result["materialization_external_exact_misses"] = counters.materialization_external_exact_misses;
    result["materialization_external_live_fallbacks"] =
        counters.materialization_external_live_fallbacks;
    result["materialization_external_maybe_live_retries"] =
        counters.materialization_external_maybe_live_retries;
    result["materialization_external_maybe_live_retry_free"] =
        counters.materialization_external_maybe_live_retry_free;
    result["interval_replay_compatibility_checks"] = counters.interval_replay_compatibility_checks;
    result["interval_replay_compatible"] = counters.interval_replay_compatible;
    result["interval_replay_incompatible"] = counters.interval_replay_incompatible;
    result["interval_replay_direct_exact_hits"] = counters.interval_replay_direct_exact_hits;
    result["interval_replay_key_only_blocked"] = counters.interval_replay_key_only_blocked;
    result["materialization_reused_shared_endpoint_cache"] =
        counters.materialization_reused_shared_endpoint_cache;
    result["materialization_stored_shared_endpoint_cache"] =
        counters.materialization_stored_shared_endpoint_cache;
    result["materialization_reused_cached_envelope"] =
        counters.materialization_reused_cached_envelope;
    result["materialization_candidate_dirty_count"] =
        counters.materialization_candidate_dirty_count;
    result["materialization_predh_rebuild_count"] =
        counters.materialization_predh_rebuild_count;
    result["canonical_frame_invalid"] = counters.canonical_frame_invalid;
    result["canonical_reflected_seed_misses"] = counters.canonical_reflected_seed_misses;
    result["scoring_evaluations"] = counters.scoring_evaluations;
    result["scoring_changed_dim_inferred"] = counters.scoring_changed_dim_inferred;
    result["scoring_incremental_fk"] = counters.scoring_incremental_fk;
    result["scoring_source_incremental_state"] = counters.scoring_source_incremental_state;
    result["scoring_reused_fk"] = counters.scoring_reused_fk;
    result["scoring_reused_endpoint_cache"] = counters.scoring_reused_endpoint_cache;
    result["scoring_reused_external_evidence"] = counters.scoring_reused_external_evidence;
    result["scoring_endpoint_time_us"] = counters.scoring_endpoint_time_us;
    result["scoring_envelope_time_us"] = counters.scoring_envelope_time_us;
    result["scoring_candidate_dirty_count"] = counters.scoring_candidate_dirty_count;
    result["scoring_predh_rebuild_count"] = counters.scoring_predh_rebuild_count;
    result["envelope_collision_queries"] = counters.envelope_collision_queries;
    result["envelope_collision_free"] = counters.envelope_collision_free;
    result["envelope_collision_maybe"] = counters.envelope_collision_maybe;
    result["envelope_collision_envelope_aabb_tests"] =
        counters.envelope_collision_envelope_aabb_tests;
    result["envelope_collision_envelope_aabb_rejects"] =
        counters.envelope_collision_envelope_aabb_rejects;
    result["envelope_collision_link_union_aabb_tests"] =
        counters.envelope_collision_link_union_aabb_tests;
    result["envelope_collision_link_union_aabb_rejects"] =
        counters.envelope_collision_link_union_aabb_rejects;
    result["envelope_collision_link_aabb_tests"] = counters.envelope_collision_link_aabb_tests;
    result["envelope_collision_link_aabb_rejects"] =
        counters.envelope_collision_link_aabb_rejects;
    result["envelope_collision_kdop_tests"] = counters.envelope_collision_kdop_tests;
    result["envelope_collision_kdop_rejects"] = counters.envelope_collision_kdop_rejects;
    result["envelope_collision_kdop_axes_tested"] = counters.envelope_collision_kdop_axes_tested;
    result["envelope_collision_gjk_tests"] = counters.envelope_collision_gjk_tests;
    result["envelope_collision_gjk_rejects"] = counters.envelope_collision_gjk_rejects;
    result["envelope_collision_gjk_iterations"] = counters.envelope_collision_gjk_iterations;
    result["envelope_collision_overlap_depth_sum"] =
        counters.envelope_collision_overlap_depth_sum;
    result["envelope_collision_overlap_depth_max"] =
        counters.envelope_collision_overlap_depth_max;
    result["envelope_collision_overlap_volume_ratio_max"] =
        counters.envelope_collision_overlap_volume_ratio_max;
    return result;
}

inline rbf::OracleValidationConfig uncached_validation_config(
    rbf::OracleValidationConfig config) {
    config.enable_validation_cache = false;
    config.enable_endpoint_evidence_cache = false;
    config.store_endpoint_evidence_cache = false;
    config.external_evidence_materialization = false;
    config.external_evidence_scoring = false;
    return config;
}

} // namespace rbf::python_binding
