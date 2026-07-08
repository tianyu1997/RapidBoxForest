set(SBF_ADAPTIVE_GRID_PARTITION_SOURCES
    src/graph_partition/adaptive_grid_partition.cpp
    src/graph_partition/adaptive_grid_partition_connectivity.cpp
    src/graph_partition/adaptive_grid_partition_geometry.cpp
    src/graph_partition/adaptive_grid_partition_indices.cpp
    src/graph_partition/adaptive_grid_partition_merge.cpp
    src/graph_partition/adaptive_grid_partition_neighbors.cpp
    src/graph_partition/adaptive_grid_partition_overlay.cpp
    src/graph_partition/adaptive_grid_partition_overlay_components.cpp
    src/graph_partition/adaptive_grid_partition_path_query.cpp
    src/graph_partition/adaptive_grid_partition_point_index.cpp
    src/graph_partition/adaptive_grid_partition_query.cpp
    src/graph_partition/adaptive_grid_partition_sparse.cpp
)

set(SBF_BOX_GRAPH_SOURCES
    src/graph_partition/box_graph.cpp
    src/graph_partition/box_graph_edges.cpp
    src/graph_partition/box_graph_query.cpp
    src/graph_partition/box_graph_search.cpp
    src/graph_partition/box_graph_sequence.cpp
    src/graph_partition/box_graph_topology.cpp
)

set(SBF_GRAPH_PARTITION_SOURCES
    ${SBF_ADAPTIVE_GRID_PARTITION_SOURCES}
    ${SBF_BOX_GRAPH_SOURCES}
)

set(SBF_FFB_CORE_SOURCES
    src/free_box/find_free_box.cpp
    src/free_box/find_free_box_binary.cpp
    src/free_box/find_free_box_internal.cpp
    src/free_box/virtual_sparse_ffb.cpp
)

set(SBF_FRONTWAVE_GROWER_SOURCES
    src/grower/frontwave_grower.cpp
)

set(SBF_RRT_GROWER_SOURCES
    src/grower/grower.cpp
    src/grower/grower_bootstrap.cpp
    src/grower/grower_commit.cpp
    src/grower/grower_component_connect.cpp
    src/grower/grower_component_connect_chain.cpp
    src/grower/grower_component_connect_global.cpp
    src/grower/grower_components.cpp
    src/grower/grower_entry.cpp
    src/grower/grower_failure_cooling.cpp
    src/grower/grower_frontier.cpp
    src/grower/grower_frontier_helpers.cpp
    src/grower/grower_frontier_memory.cpp
    src/grower/grower_internal.cpp
    src/grower/grower_options.cpp
    src/grower/grower_roots.cpp
    src/grower/grower_sampling.cpp
    src/grower/grower_task_builder.cpp
    src/grower/grower_task_filter.cpp
    src/grower/grower_task_requests.cpp
    src/grower/grower_trace.cpp
    src/grower/grower_workers.cpp
)

set(SBF_LEAF_SWEEP_GROWER_SOURCES
    src/leaf_sweep_grower/leaf_sweep_grower.cpp
    src/leaf_sweep_grower/leaf_sweep_grower_cluster.cpp
    src/leaf_sweep_grower/leaf_sweep_grower_compose.cpp
    src/leaf_sweep_grower/leaf_sweep_grower_diagnostics.cpp
    src/leaf_sweep_grower/leaf_sweep_grower_frontier.cpp
    src/leaf_sweep_grower/leaf_sweep_grower_group.cpp
    src/leaf_sweep_grower/leaf_sweep_grower_internal.cpp
    src/leaf_sweep_grower/leaf_sweep_grower_virtual.cpp
)

set(SBF_FFB_GROWER_SOURCES
    ${SBF_FFB_CORE_SOURCES}
    ${SBF_FRONTWAVE_GROWER_SOURCES}
    ${SBF_RRT_GROWER_SOURCES}
    ${SBF_LEAF_SWEEP_GROWER_SOURCES}
)

set(SBF_CONNECTOR_CORE_SOURCES
    src/connector/connector.cpp
    src/connector/connector_broadphase.cpp
    src/connector/connector_birrt.cpp
    src/connector/connector_entry.cpp
    src/connector/connector_frontier_bridge.cpp
    src/connector/connector_internal.cpp
)

set(SBF_CONNECTOR_PAIR_PIPELINE_SOURCES
    src/connector/connector_pair_candidates.cpp
    src/connector/connector_pair_commit.cpp
    src/connector/connector_pair_tasks.cpp
)

set(SBF_CHAIN_PAVE_SOURCES
    src/connector/connector_chain_pave.cpp
    src/connector/connector_chain_pave_commit.cpp
    src/connector/connector_chain_pave_hooks.cpp
    src/connector/connector_chain_pave_internal.cpp
)

set(SBF_MERGER_SOURCES
    src/connector/merger.cpp
)

set(SBF_CONNECTOR_SOURCES
    ${SBF_MERGER_SOURCES}
    ${SBF_CONNECTOR_CORE_SOURCES}
    ${SBF_CONNECTOR_PAIR_PIPELINE_SOURCES}
    ${SBF_CHAIN_PAVE_SOURCES}
)

set(SBF_PLANNING_CORE_SOURCES
    src/planning_core/planning_forest_audit.cpp
    src/planning_core/planning_forest_core.cpp
    src/planning_core/planning_forest_database.cpp
    src/planning_core/planning_forest_diagnostics.cpp
    src/planning_core/planning_forest_diagnostic_hooks.cpp
)

set(SBF_PLANNING_BUILD_PIPELINE_SOURCES
    src/planning_build/planning_forest_build.cpp
    src/planning_build/planning_forest_leaf_refine.cpp
    src/planning_build/planning_forest_ffb.cpp
    src/planning_build/planning_forest_ffb_binary.cpp
    src/planning_build/planning_forest_ffb_binary_sparse.cpp
    src/planning_build/planning_forest_partition.cpp
    src/planning_build/planning_forest_partition_diagnostics.cpp
)

set(SBF_PLANNING_FACADE_SOURCES
    ${SBF_PLANNING_CORE_SOURCES}
    ${SBF_PLANNING_BUILD_PIPELINE_SOURCES}
)

set(SBF_PLANNING_ADAPTIVE_SOURCES
    src/planning_adaptive/planning_forest_adaptive_build.cpp
    src/planning_adaptive/planning_forest_adaptive_checkpoint.cpp
    src/planning_adaptive/planning_forest_adaptive_commit.cpp
    src/planning_adaptive/planning_forest_adaptive_connectivity.cpp
    src/planning_adaptive/planning_forest_adaptive_depth.cpp
    src/planning_adaptive/planning_forest_adaptive_diagnostics.cpp
    src/planning_adaptive/planning_forest_adaptive_fast_candidate.cpp
    src/planning_adaptive/planning_forest_adaptive_fast_checkpoint.cpp
    src/planning_adaptive/planning_forest_adaptive_finalize.cpp
    src/planning_adaptive/planning_forest_adaptive_fixed.cpp
    src/planning_adaptive/planning_forest_adaptive_frontier.cpp
    src/planning_adaptive/planning_forest_adaptive_cover_utils.cpp
    src/planning_adaptive/planning_forest_adaptive_merge.cpp
    src/planning_adaptive/planning_forest_adaptive_merge_grid.cpp
    src/planning_adaptive/planning_forest_adaptive_snapshot.cpp
    src/planning_adaptive/planning_forest_adaptive_topology.cpp
    src/planning_adaptive/planning_forest_adaptive_validation.cpp
)

set(SBF_PLANNING_QROOT_SOURCES
    src/qroot/planning_forest_qroot_dsu.cpp
    src/qroot/planning_forest_qroot_helpers.cpp
    src/qroot/planning_forest_qroot_growers.cpp
    src/qroot/planning_forest_qroot_offline.cpp
    src/qroot/planning_forest_qroot_spatial_index.cpp
)

set(SBF_PLANNING_BUILD_SOURCES
    ${SBF_PLANNING_FACADE_SOURCES}
    ${SBF_PLANNING_ADAPTIVE_SOURCES}
    ${SBF_PLANNING_QROOT_SOURCES}
)

set(SBF_OBB_VALIDATION_SOURCES
    src/obb/planning_forest_obb.cpp
    src/obb/planning_forest_obb_diagnostics.cpp
    src/obb/planning_forest_obb_geometry.cpp
    src/obb/planning_forest_obb_path_cover.cpp
    src/obb/planning_forest_obb_path_windows.cpp
    src/obb/planning_forest_obb_stats.cpp
    src/obb/planning_forest_obb_validation.cpp
    src/obb/planning_forest_obb_zonotope.cpp
)

set(SBF_OBB_SAMPLED_BACKEND_SOURCES
    src/obb/planning_forest_obb_sampled.cpp
    src/obb/planning_forest_obb_sampled_clearance.cpp
)

set(SBF_OVERLAY_PORTAL_SOURCES
    src/overlay/planning_forest_overlay.cpp
    src/overlay/planning_forest_overlay_edges.cpp
    src/overlay/planning_forest_overlay_obb_retry.cpp
    src/overlay/planning_forest_overlay_portal_resolvers.cpp
    src/overlay/planning_forest_overlay_portal_resolvers_ffb.cpp
)

set(SBF_OBB_OVERLAY_SOURCES
    ${SBF_OBB_VALIDATION_SOURCES}
    ${SBF_OBB_SAMPLED_BACKEND_SOURCES}
    ${SBF_OVERLAY_PORTAL_SOURCES}
)

set(SBF_QUERY_BRIDGE_CORE_SOURCES
    src/query_bridge/planning_forest_query_bridge_acceptance_options.cpp
    src/query_bridge/planning_forest_query_bridge_anchor.cpp
    src/query_bridge/planning_forest_query_bridge_attempt_paths.cpp
    src/query_bridge/planning_forest_query_bridge_path_utils.cpp
    src/query_bridge/planning_forest_query_bridge_repair_options.cpp
    src/query_bridge/planning_forest_query_bridge_edges.cpp
    src/query_bridge/planning_forest_query_bridge_hipac.cpp
    src/query_bridge/planning_forest_query_bridge_hipac_utils.cpp
    src/query_bridge/planning_forest_query_bridge_pair.cpp
    src/query_bridge/planning_forest_query_bridge_pave.cpp
    src/query_bridge/planning_forest_query_bridge_pave_guard.cpp
    src/query_bridge/planning_forest_query_bridge_task_key.cpp
    src/query_bridge/planning_forest_query_bridge_waypoint.cpp
)

set(SBF_QUERY_BRIDGE_BATCH_SOURCES
    src/query_bridge/planning_forest_query_bridge_batch.cpp
    src/query_bridge/planning_forest_query_bridge_batch_diagnostics.cpp
    src/query_bridge/planning_forest_query_bridge_batch_finalize.cpp
    src/query_bridge/planning_forest_query_bridge_batch_options.cpp
    src/query_bridge/planning_forest_query_bridge_batch_parallel.cpp
    src/query_bridge/planning_forest_query_bridge_batch_policy.cpp
    src/query_bridge/planning_forest_query_bridge_batch_serial.cpp
    src/query_bridge/planning_forest_query_bridge_batch_tasks.cpp
)

set(SBF_QUERY_BRIDGE_CORRIDOR_SOURCES
    src/query_bridge/planning_forest_query_bridge_corridor_options.cpp
    src/query_bridge/planning_forest_query_bridge_corridor_diagnostics.cpp
    src/query_bridge/planning_forest_query_bridge_corridor_commit.cpp
    src/query_bridge/planning_forest_query_bridge_corridor_graph.cpp
    src/query_bridge/planning_forest_query_bridge_corridor_local_graph.cpp
    src/query_bridge/planning_forest_query_bridge_corridor_repair.cpp
    src/query_bridge/planning_forest_query_bridge_corridor_samples.cpp
    src/query_bridge/planning_forest_query_bridge_corridor_tasks.cpp
)

set(SBF_QUERY_BRIDGE_DIRECT_SOURCES
    src/query_bridge/planning_forest_query_bridge_direct_corridor.cpp
    src/query_bridge/planning_forest_query_bridge_direct_finalize.cpp
    src/query_bridge/planning_forest_query_bridge_direct_promotion.cpp
    src/query_bridge/planning_forest_query_bridge_direct_residual.cpp
    src/query_bridge/planning_forest_query_bridge_direct_segments.cpp
)

set(SBF_QUERY_BRIDGE_ENDPOINT_SOURCES
    src/query_bridge/planning_forest_query_bridge_endpoint.cpp
    src/query_bridge/planning_forest_query_bridge_endpoint_direct.cpp
    src/query_bridge/planning_forest_query_bridge_endpoint_index.cpp
    src/query_bridge/planning_forest_query_bridge_endpoint_island.cpp
    src/query_bridge/planning_forest_query_bridge_endpoint_residual.cpp
    src/query_bridge/planning_forest_query_bridge_endpoint_runtime.cpp
    src/query_bridge/planning_forest_query_bridge_endpoint_targets.cpp
)

set(SBF_QUERY_BRIDGE_RRT_SOURCES
    src/query_bridge/planning_forest_query_bridge_rrt_options.cpp
    src/query_bridge/planning_forest_query_bridge_rrt_parallel.cpp
    src/query_bridge/planning_forest_query_bridge_rrt_schedule.cpp
    src/query_bridge/planning_forest_query_bridge_rrt_utils.cpp
)

set(SBF_QUERY_BRIDGE_SOURCES
    ${SBF_QUERY_BRIDGE_CORE_SOURCES}
    ${SBF_QUERY_BRIDGE_BATCH_SOURCES}
    ${SBF_QUERY_BRIDGE_CORRIDOR_SOURCES}
    ${SBF_QUERY_BRIDGE_DIRECT_SOURCES}
    ${SBF_QUERY_BRIDGE_ENDPOINT_SOURCES}
    ${SBF_QUERY_BRIDGE_RRT_SOURCES}
)

set(SBF_QUERY_FACADE_SOURCES
    src/query_runtime/planning_forest_query.cpp
)

set(SBF_QUERY_UTILITY_SOURCES
    src/query_runtime/planning_forest_query_geometry.cpp
    src/query_runtime/planning_forest_query_repair.cpp
    src/query_runtime/planning_forest_query_utils.cpp
    src/query_runtime/planning_forest_query_utils_rrt.cpp
    src/query_runtime/planning_forest_query_utils_shortcut.cpp
)

set(SBF_QUERY_SHORTCUT_SOURCES
    src/query_runtime/planning_forest_shortcut.cpp
)

set(SBF_QUERY_RESULT_SOURCES
    src/query_runtime/query.cpp
)

set(SBF_RUNTIME_INFRA_SOURCES
    src/query_runtime/runtime.cpp
)

set(SBF_QUERY_RUNTIME_SOURCES
    ${SBF_QUERY_FACADE_SOURCES}
    ${SBF_QUERY_UTILITY_SOURCES}
    ${SBF_QUERY_SHORTCUT_SOURCES}
    ${SBF_QUERY_RESULT_SOURCES}
    ${SBF_RUNTIME_INFRA_SOURCES}
)

set(SBF_CORE_SOURCES
    ${SBF_GRAPH_PARTITION_SOURCES}
    ${SBF_FFB_GROWER_SOURCES}
    ${SBF_CONNECTOR_SOURCES}
    ${SBF_PLANNING_BUILD_SOURCES}
    ${SBF_OBB_OVERLAY_SOURCES}
    ${SBF_QUERY_BRIDGE_SOURCES}
    ${SBF_QUERY_RUNTIME_SOURCES}
)

set(SBF_DIAGNOSTIC_SOURCES
    src/diagnostic/connector_chain_pave_debug.cpp
    src/diagnostic/planning_forest_corridor_refine.cpp
    src/diagnostic/planning_forest_debug.cpp
    src/diagnostic/planning_forest_dynamic_cache.cpp
    src/diagnostic/planning_forest_dynamic_collision_cache.cpp
    src/diagnostic/planning_forest_dynamic_helpers.cpp
    src/diagnostic/planning_forest_dynamic_partition.cpp
    src/diagnostic/planning_forest_dynamic_remove.cpp
    src/diagnostic/planning_forest_dynamic_refill.cpp
    src/diagnostic/planning_forest_dynamic_segment_endpoint.cpp
    src/diagnostic/planning_forest_dynamic_segment_fallback.cpp
    src/diagnostic/planning_forest_subtractive.cpp
    src/diagnostic/planning_forest_subtractive_seeds.cpp
)
