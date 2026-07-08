#pragma once

#include <LECTDatabase/sbf/oracle_types.h>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <utility>

namespace py = pybind11;

namespace rbf::python_binding {

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
