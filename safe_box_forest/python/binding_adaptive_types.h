#pragma once

#include <SBF/adaptive_leaf_sweep_config.h>
#include <SBF/planning_result.h>
#include <SBF/query_runtime_config.h>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

namespace rbf::python_binding {

inline void register_adaptive_types(py::module_& module) {
    py::class_<rbf::LeafSweepRefineConfig>(module, "LeafSweepRefineConfig")
        .def(py::init<>())
        .def_readwrite("leaf_start_depth", &rbf::LeafSweepRefineConfig::leaf_start_depth)
        .def_readwrite("leaf_max_depth", &rbf::LeafSweepRefineConfig::leaf_max_depth)
        .def_readwrite("obstacle_cluster_gap", &rbf::LeafSweepRefineConfig::obstacle_cluster_gap)
        .def_readwrite("use_virtual_topology", &rbf::LeafSweepRefineConfig::use_virtual_topology)
        .def_readwrite("parallel_virtual_validation",
                       &rbf::LeafSweepRefineConfig::parallel_virtual_validation)
        .def_readwrite("store_group_results", &rbf::LeafSweepRefineConfig::store_group_results)
        .def_readwrite("validation_batch_size", &rbf::LeafSweepRefineConfig::validation_batch_size)
        .def_readwrite("leaf_threads", &rbf::LeafSweepRefineConfig::leaf_threads)
        .def_readwrite("leaf_timeout_ms", &rbf::LeafSweepRefineConfig::leaf_timeout_ms)
        .def_readwrite("deep_max_boxes", &rbf::LeafSweepRefineConfig::deep_max_boxes)
        .def_readwrite("deep_ffb_depth", &rbf::LeafSweepRefineConfig::deep_ffb_depth)
        .def_readwrite("domain_seed_cap", &rbf::LeafSweepRefineConfig::domain_seed_cap)
        .def_readwrite("domain_success_cap", &rbf::LeafSweepRefineConfig::domain_success_cap)
        .def_readwrite("domain_attempt_cap", &rbf::LeafSweepRefineConfig::domain_attempt_cap)
        .def_readwrite("allow_anchor_roots", &rbf::LeafSweepRefineConfig::allow_anchor_roots)
        .def_readwrite("refine_timeout_ms", &rbf::LeafSweepRefineConfig::refine_timeout_ms)
        .def_readwrite("priority_prune_radius",
                       &rbf::LeafSweepRefineConfig::priority_prune_radius)
        .def_readwrite("collision_overlap_prune_min_depth",
                       &rbf::LeafSweepRefineConfig::collision_overlap_prune_min_depth)
        .def_readwrite("collision_overlap_prune_threshold",
                       &rbf::LeafSweepRefineConfig::collision_overlap_prune_threshold)
        .def_readwrite("collision_overlap_prune_ratio_threshold",
                       &rbf::LeafSweepRefineConfig::collision_overlap_prune_ratio_threshold);

    py::class_<rbf::AdaptiveLeafSweepConfig>(module, "AdaptiveLeafSweepConfig")
        .def(py::init<>())
        .def_readwrite("shallow_start_depth", &rbf::AdaptiveLeafSweepConfig::shallow_start_depth)
        .def_readwrite("shallow_max_depth", &rbf::AdaptiveLeafSweepConfig::shallow_max_depth)
        .def_readwrite("target_max_depth", &rbf::AdaptiveLeafSweepConfig::target_max_depth)
        .def_readwrite("time_budget_ms", &rbf::AdaptiveLeafSweepConfig::time_budget_ms)
        .def_readwrite("node_budget", &rbf::AdaptiveLeafSweepConfig::node_budget)
        .def_readwrite("threads", &rbf::AdaptiveLeafSweepConfig::threads)
        .def_readwrite("validation_batch_size", &rbf::AdaptiveLeafSweepConfig::validation_batch_size)
        .def_readwrite("obstacle_cluster_gap", &rbf::AdaptiveLeafSweepConfig::obstacle_cluster_gap)
        .def_readwrite("use_virtual_topology", &rbf::AdaptiveLeafSweepConfig::use_virtual_topology)
        .def_readwrite("parallel_virtual_validation",
                       &rbf::AdaptiveLeafSweepConfig::parallel_virtual_validation)
        .def_readwrite("store_group_results", &rbf::AdaptiveLeafSweepConfig::store_group_results)
        .def_readwrite("fast_virtual_checkpoint_mode",
                       &rbf::AdaptiveLeafSweepConfig::fast_virtual_checkpoint_mode)
        .def_readwrite("defer_min_depth", &rbf::AdaptiveLeafSweepConfig::defer_min_depth)
        .def_readwrite("overlap_depth_threshold",
                       &rbf::AdaptiveLeafSweepConfig::overlap_depth_threshold)
        .def_readwrite("overlap_depth_min_threshold",
                       &rbf::AdaptiveLeafSweepConfig::overlap_depth_min_threshold)
        .def_readwrite("overlap_depth_decay_per_depth",
                       &rbf::AdaptiveLeafSweepConfig::overlap_depth_decay_per_depth)
        .def_readwrite("overlap_ratio_threshold",
                       &rbf::AdaptiveLeafSweepConfig::overlap_ratio_threshold)
        .def_readwrite("seed_probe_count", &rbf::AdaptiveLeafSweepConfig::seed_probe_count)
        .def_readwrite("seed_probe_rng_seed", &rbf::AdaptiveLeafSweepConfig::seed_probe_rng_seed)
        .def_readwrite("seed_promote_uncovered",
                       &rbf::AdaptiveLeafSweepConfig::seed_promote_uncovered)
        .def_readwrite("seed_anchor_probe_cap",
                       &rbf::AdaptiveLeafSweepConfig::seed_anchor_probe_cap)
        .def_readwrite("promotion_interval", &rbf::AdaptiveLeafSweepConfig::promotion_interval)
        .def_readwrite("adaptive_depth_enabled",
                       &rbf::AdaptiveLeafSweepConfig::adaptive_depth_enabled)
        .def_readwrite("adaptive_depth_min", &rbf::AdaptiveLeafSweepConfig::adaptive_depth_min)
        .def_readwrite("adaptive_depth_max", &rbf::AdaptiveLeafSweepConfig::adaptive_depth_max)
        .def_readwrite("adaptive_depth_probe_count",
                       &rbf::AdaptiveLeafSweepConfig::adaptive_depth_probe_count)
        .def_readwrite("adaptive_depth_anchor_probe_cap",
                       &rbf::AdaptiveLeafSweepConfig::adaptive_depth_anchor_probe_cap)
        .def_readwrite("adaptive_depth_probe_seed",
                       &rbf::AdaptiveLeafSweepConfig::adaptive_depth_probe_seed)
        .def_readwrite("adaptive_depth_min_free_probes",
                       &rbf::AdaptiveLeafSweepConfig::adaptive_depth_min_free_probes)
        .def_readwrite("adaptive_depth_min_covered_probes",
                       &rbf::AdaptiveLeafSweepConfig::adaptive_depth_min_covered_probes)
        .def_readwrite("adaptive_depth_min_main_probes",
                       &rbf::AdaptiveLeafSweepConfig::adaptive_depth_min_main_probes)
        .def_readwrite("adaptive_depth_min_main_ratio",
                       &rbf::AdaptiveLeafSweepConfig::adaptive_depth_min_main_ratio)
        .def_readwrite("adaptive_depth_min_cells",
                       &rbf::AdaptiveLeafSweepConfig::adaptive_depth_min_cells)
        .def_readwrite("adaptive_depth_min_main_cells",
                       &rbf::AdaptiveLeafSweepConfig::adaptive_depth_min_main_cells)
        .def_readwrite("adaptive_depth_max_online_cells",
                       &rbf::AdaptiveLeafSweepConfig::adaptive_depth_max_online_cells)
        .def_readwrite("adaptive_depth_max_probe_ms",
                       &rbf::AdaptiveLeafSweepConfig::adaptive_depth_max_probe_ms)
        .def_readwrite("max_merge_ms", &rbf::AdaptiveLeafSweepConfig::max_merge_ms)
        .def_readwrite("max_merge_rounds", &rbf::AdaptiveLeafSweepConfig::max_merge_rounds)
        .def_readwrite("max_merge_input_boxes",
                       &rbf::AdaptiveLeafSweepConfig::max_merge_input_boxes)
        .def_readwrite("max_free_boxes", &rbf::AdaptiveLeafSweepConfig::max_free_boxes)
        .def_readwrite("max_unresolved_domains",
                       &rbf::AdaptiveLeafSweepConfig::max_unresolved_domains)
        .def_readwrite("planning_backend", &rbf::AdaptiveLeafSweepConfig::planning_backend)
        .def_readwrite("grid_target_depth", &rbf::AdaptiveLeafSweepConfig::grid_target_depth)
        .def_readwrite("grid_face_index_enabled",
                       &rbf::AdaptiveLeafSweepConfig::grid_face_index_enabled)
        .def_readwrite("grid_planning_max_expansions",
                       &rbf::AdaptiveLeafSweepConfig::grid_planning_max_expansions)
        .def_readwrite("hipac_portal_connectivity",
                       &rbf::AdaptiveLeafSweepConfig::hipac_portal_connectivity)
        .def_readwrite("hipac_portal_cell_native_validate",
                       &rbf::AdaptiveLeafSweepConfig::hipac_portal_cell_native_validate)
        .def_readwrite("hipac_portal_max_internal_boxes",
                       &rbf::AdaptiveLeafSweepConfig::hipac_portal_max_internal_boxes)
        .def_readwrite("hipac_portal_max_recursion_depth",
                       &rbf::AdaptiveLeafSweepConfig::hipac_portal_max_recursion_depth)
        .def_readwrite("hipac_portal_ffb_depth",
                       &rbf::AdaptiveLeafSweepConfig::hipac_portal_ffb_depth)
        .def_readwrite("hipac_portal_ffb_deadline_ms",
                       &rbf::AdaptiveLeafSweepConfig::hipac_portal_ffb_deadline_ms)
        .def_readwrite("hipac_online_connectivity",
                       &rbf::AdaptiveLeafSweepConfig::hipac_online_connectivity)
        .def_readwrite("hipac_online_before_query_bridge",
                       &rbf::AdaptiveLeafSweepConfig::hipac_online_before_query_bridge)
        .def_readwrite("hipac_promote_query_repairs",
                       &rbf::AdaptiveLeafSweepConfig::hipac_promote_query_repairs)
        .def_readwrite("hipac_online_candidate_max_length",
                       &rbf::AdaptiveLeafSweepConfig::hipac_online_candidate_max_length)
        .def_readwrite("hipac_online_max_resolves_per_query",
                       &rbf::AdaptiveLeafSweepConfig::hipac_online_max_resolves_per_query)
        .def_readwrite("hipac_online_max_hidden_boxes_per_portal",
                       &rbf::AdaptiveLeafSweepConfig::hipac_online_max_hidden_boxes_per_portal)
        .def_readwrite("hipac_online_max_ffb_calls_per_portal",
                       &rbf::AdaptiveLeafSweepConfig::hipac_online_max_ffb_calls_per_portal)
        .def_readwrite("hipac_online_prebridge_portal",
                       &rbf::AdaptiveLeafSweepConfig::hipac_online_prebridge_portal)
        .def_readwrite("hipac_online_prebridge_candidate_limit",
                       &rbf::AdaptiveLeafSweepConfig::hipac_online_prebridge_candidate_limit)
        .def_readwrite("hipac_online_prebridge_max_pair_distance",
                       &rbf::AdaptiveLeafSweepConfig::hipac_online_prebridge_max_pair_distance)
        .def_readwrite("hipac_online_prebridge_route_distance_weight",
                       &rbf::AdaptiveLeafSweepConfig::hipac_online_prebridge_route_distance_weight)
        .def_readwrite("hipac_online_prebridge_pair_distance_weight",
                       &rbf::AdaptiveLeafSweepConfig::hipac_online_prebridge_pair_distance_weight)
        .def_readwrite("hipac_transition_obb_portal",
                       &rbf::AdaptiveLeafSweepConfig::hipac_transition_obb_portal)
        .def_readwrite("hipac_transition_obb_lateral_radius",
                       &rbf::AdaptiveLeafSweepConfig::hipac_transition_obb_lateral_radius)
        .def_readwrite("hipac_transition_obb_longitudinal_margin",
                       &rbf::AdaptiveLeafSweepConfig::hipac_transition_obb_longitudinal_margin)
        .def_readwrite("hipac_transition_obb_safety_epsilon",
                       &rbf::AdaptiveLeafSweepConfig::hipac_transition_obb_safety_epsilon)
        .def_readwrite("segment_edge_obb_cover",
                       &rbf::AdaptiveLeafSweepConfig::segment_edge_obb_cover)
        .def_readwrite("rrt_bridge_obb_cover",
                       &rbf::AdaptiveLeafSweepConfig::rrt_bridge_obb_cover)
        .def_readwrite("strict_obb_bridge_cover",
                       &rbf::AdaptiveLeafSweepConfig::strict_obb_bridge_cover)
        .def_readwrite("segment_edge_obb_lateral_radius",
                       &rbf::AdaptiveLeafSweepConfig::segment_edge_obb_lateral_radius)
        .def_readwrite("segment_edge_obb_longitudinal_margin",
                       &rbf::AdaptiveLeafSweepConfig::segment_edge_obb_longitudinal_margin)
        .def_readwrite("segment_edge_obb_safety_epsilon",
                       &rbf::AdaptiveLeafSweepConfig::segment_edge_obb_safety_epsilon)
        .def_readwrite("segment_edge_obb_grow_iterations",
                       &rbf::AdaptiveLeafSweepConfig::segment_edge_obb_grow_iterations)
        .def_readwrite("segment_edge_obb_binary_iterations",
                       &rbf::AdaptiveLeafSweepConfig::segment_edge_obb_binary_iterations)
        .def_readwrite("segment_edge_obb_split_depth",
                       &rbf::AdaptiveLeafSweepConfig::segment_edge_obb_split_depth)
        .def_readwrite("obb_max_window_segments",
                       &rbf::AdaptiveLeafSweepConfig::obb_max_window_segments)
        .def_readwrite("obb_max_validations_per_window",
                       &rbf::AdaptiveLeafSweepConfig::obb_max_validations_per_window)
        .def_readwrite("obb_fast_primary_orientation",
                       &rbf::AdaptiveLeafSweepConfig::obb_fast_primary_orientation)
        .def_readwrite("obb_fallback_orientations_on_primary_fail",
                       &rbf::AdaptiveLeafSweepConfig::obb_fallback_orientations_on_primary_fail)
        .def_readwrite("obb_sampled_support_enabled",
                       &rbf::AdaptiveLeafSweepConfig::obb_sampled_support_enabled)
        .def_readwrite("obb_clearance_sampled_support_enabled",
                       &rbf::AdaptiveLeafSweepConfig::obb_clearance_sampled_support_enabled)
        .def_readwrite("obb_clearance_lateral_l1_max",
                       &rbf::AdaptiveLeafSweepConfig::obb_clearance_lateral_l1_max)
        .def_readwrite("obb_clearance_samples",
                       &rbf::AdaptiveLeafSweepConfig::obb_clearance_samples)
        .def_readwrite("obb_clearance_dense_line_l1_threshold",
                       &rbf::AdaptiveLeafSweepConfig::obb_clearance_dense_line_l1_threshold)
        .def_readwrite("obb_clearance_dense_samples",
                       &rbf::AdaptiveLeafSweepConfig::obb_clearance_dense_samples)
        .def_readwrite("obb_clearance_fast_samples",
                       &rbf::AdaptiveLeafSweepConfig::obb_clearance_fast_samples)
        .def_readwrite("obb_clearance_first",
                       &rbf::AdaptiveLeafSweepConfig::obb_clearance_first)
        .def_readwrite("obb_clearance_retry_attempts",
                       &rbf::AdaptiveLeafSweepConfig::obb_clearance_retry_attempts)
        .def_readwrite("obb_clearance_retry_values",
                       &rbf::AdaptiveLeafSweepConfig::obb_clearance_retry_values)
        .def_readwrite("obb_clearance_retry_iters",
                       &rbf::AdaptiveLeafSweepConfig::obb_clearance_retry_iters)
        .def_readwrite("obb_clearance_retry_timeout_ms",
                       &rbf::AdaptiveLeafSweepConfig::obb_clearance_retry_timeout_ms)
        .def_readwrite("segment_edge_obb_metadata_only",
                       &rbf::AdaptiveLeafSweepConfig::segment_edge_obb_metadata_only)
        .def_readwrite("segment_edge_obb_metadata_require_cover",
                       &rbf::AdaptiveLeafSweepConfig::segment_edge_obb_metadata_require_cover)
        .def_readwrite("hipac_promote_transition_slices",
                       &rbf::AdaptiveLeafSweepConfig::hipac_promote_transition_slices)
        .def_readwrite("hipac_promote_transition_target_query_indices",
                       &rbf::AdaptiveLeafSweepConfig::hipac_promote_transition_target_query_indices)
        .def_readwrite("hipac_promote_transition_min_boxes",
                       &rbf::AdaptiveLeafSweepConfig::hipac_promote_transition_min_boxes)
        .def_readwrite("hipac_promote_transition_max_boxes",
                       &rbf::AdaptiveLeafSweepConfig::hipac_promote_transition_max_boxes)
        .def_readwrite("hipac_promote_transition_max_attempts_per_query",
                       &rbf::AdaptiveLeafSweepConfig::hipac_promote_transition_max_attempts_per_query);

    py::class_<rbf::EndpointMainBoxCorridorConfig>(module, "EndpointMainBoxCorridorConfig")
        .def(py::init<>())
        .def_readwrite("target_k", &rbf::EndpointMainBoxCorridorConfig::target_k)
        .def_readwrite("coarse_step", &rbf::EndpointMainBoxCorridorConfig::coarse_step)
        .def_readwrite("fine_step", &rbf::EndpointMainBoxCorridorConfig::fine_step)
        .def_readwrite("max_ffb_calls", &rbf::EndpointMainBoxCorridorConfig::max_ffb_calls)
        .def_readwrite("max_boxes", &rbf::EndpointMainBoxCorridorConfig::max_boxes)
        .def_readwrite("residual_segment_max_length",
                       &rbf::EndpointMainBoxCorridorConfig::residual_segment_max_length)
        .def_readwrite("lateral_offset", &rbf::EndpointMainBoxCorridorConfig::lateral_offset)
        .def_readwrite("lateral_rounds", &rbf::EndpointMainBoxCorridorConfig::lateral_rounds)
        .def_readwrite("face_epsilon", &rbf::EndpointMainBoxCorridorConfig::face_epsilon);

    py::class_<rbf::LeafSweepRefineResult>(module, "LeafSweepRefineResult")
        .def_readonly("leaf_sweep", &rbf::LeafSweepRefineResult::leaf_sweep)
        .def_readonly("profile", &rbf::LeafSweepRefineResult::profile)
        .def_readonly("leaf_free_count", &rbf::LeafSweepRefineResult::leaf_free_count)
        .def_readonly("leaf_collision_count", &rbf::LeafSweepRefineResult::leaf_collision_count)
        .def_readonly("deep_boxes_added", &rbf::LeafSweepRefineResult::deep_boxes_added)
        .def_readonly("deep_domain_attempts", &rbf::LeafSweepRefineResult::deep_domain_attempts)
        .def_readonly("deep_ffb_success", &rbf::LeafSweepRefineResult::deep_ffb_success)
        .def_readonly("deep_ffb_fail", &rbf::LeafSweepRefineResult::deep_ffb_fail)
        .def_readonly("deep_commit_rejects", &rbf::LeafSweepRefineResult::deep_commit_rejects)
        .def_readonly("deep_domain_rejects", &rbf::LeafSweepRefineResult::deep_domain_rejects)
        .def_readonly("deep_contained_rejects",
                      &rbf::LeafSweepRefineResult::deep_contained_rejects)
        .def_readonly("deep_adjacency_rejects",
                      &rbf::LeafSweepRefineResult::deep_adjacency_rejects)
        .def_readonly("deep_anchor_roots_added",
                      &rbf::LeafSweepRefineResult::deep_anchor_roots_added)
        .def_readonly("leaf_sweep_ms", &rbf::LeafSweepRefineResult::leaf_sweep_ms)
        .def_readonly("deep_refine_ms", &rbf::LeafSweepRefineResult::deep_refine_ms)
        .def_readonly("connector_ms", &rbf::LeafSweepRefineResult::connector_ms)
        .def_readonly("total_ms", &rbf::LeafSweepRefineResult::total_ms)
        .def_readonly("diagnostics", &rbf::LeafSweepRefineResult::diagnostics);

    py::class_<rbf::AdaptiveLeafSweepResult>(module, "AdaptiveLeafSweepResult")
        .def_readonly("leaf_sweep", &rbf::AdaptiveLeafSweepResult::leaf_sweep)
        .def_readonly("profile", &rbf::AdaptiveLeafSweepResult::profile)
        .def_readonly("shallow_free_count", &rbf::AdaptiveLeafSweepResult::shallow_free_count)
        .def_readonly("shallow_collision_count",
                      &rbf::AdaptiveLeafSweepResult::shallow_collision_count)
        .def_readonly("adaptive_free_added",
                      &rbf::AdaptiveLeafSweepResult::adaptive_free_added)
        .def_readonly("adaptive_validated", &rbf::AdaptiveLeafSweepResult::adaptive_validated)
        .def_readonly("adaptive_splits", &rbf::AdaptiveLeafSweepResult::adaptive_splits)
        .def_readonly("adaptive_deferred", &rbf::AdaptiveLeafSweepResult::adaptive_deferred)
        .def_readonly("adaptive_promoted", &rbf::AdaptiveLeafSweepResult::adaptive_promoted)
        .def_readonly("unresolved_domains", &rbf::AdaptiveLeafSweepResult::unresolved_domains)
        .def_readonly("seed_probe_count", &rbf::AdaptiveLeafSweepResult::seed_probe_count)
        .def_readonly("seed_probe_free_count",
                      &rbf::AdaptiveLeafSweepResult::seed_probe_free_count)
        .def_readonly("seed_probe_box_covered",
                      &rbf::AdaptiveLeafSweepResult::seed_probe_box_covered)
        .def_readonly("seed_probe_anchor_success",
                      &rbf::AdaptiveLeafSweepResult::seed_probe_anchor_success)
        .def_readonly("seed_probe_main_accessible",
                      &rbf::AdaptiveLeafSweepResult::seed_probe_main_accessible)
        .def_readonly("p_box_covered", &rbf::AdaptiveLeafSweepResult::p_box_covered)
        .def_readonly("p_anchor_success", &rbf::AdaptiveLeafSweepResult::p_anchor_success)
        .def_readonly("p_main_accessible", &rbf::AdaptiveLeafSweepResult::p_main_accessible)
        .def_readonly("p_anchor_to_main_uncovered",
                      &rbf::AdaptiveLeafSweepResult::p_anchor_to_main_uncovered)
        .def_readonly("selected_leaf_depth", &rbf::AdaptiveLeafSweepResult::selected_leaf_depth)
        .def_readonly("adaptive_depth_readiness_met",
                      &rbf::AdaptiveLeafSweepResult::adaptive_depth_readiness_met)
        .def_readonly("adaptive_depth_stop_reason",
                      &rbf::AdaptiveLeafSweepResult::adaptive_depth_stop_reason)
        .def_readonly("adaptive_depth_snapshots_json",
                      &rbf::AdaptiveLeafSweepResult::adaptive_depth_snapshots_json)
        .def_readonly("leaf_sweep_ms", &rbf::AdaptiveLeafSweepResult::leaf_sweep_ms)
        .def_readonly("adaptive_ms", &rbf::AdaptiveLeafSweepResult::adaptive_ms)
        .def_readonly("coverage_probe_ms", &rbf::AdaptiveLeafSweepResult::coverage_probe_ms)
        .def_readonly("total_ms", &rbf::AdaptiveLeafSweepResult::total_ms)
        .def_readonly("partition_cell_count",
                      &rbf::AdaptiveLeafSweepResult::partition_cell_count)
        .def_readonly("partition_grid_cell_count",
                      &rbf::AdaptiveLeafSweepResult::partition_grid_cell_count)
        .def_readonly("partition_non_grid_cell_count",
                      &rbf::AdaptiveLeafSweepResult::partition_non_grid_cell_count)
        .def_readonly("partition_face_index_entries",
                      &rbf::AdaptiveLeafSweepResult::partition_face_index_entries)
        .def_readonly("partition_islands", &rbf::AdaptiveLeafSweepResult::partition_islands)
        .def_readonly("partition_largest_island",
                      &rbf::AdaptiveLeafSweepResult::partition_largest_island)
        .def_readonly("diagnostics", &rbf::AdaptiveLeafSweepResult::diagnostics);
}

} // namespace rbf::python_binding
