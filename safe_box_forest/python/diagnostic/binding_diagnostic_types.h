#pragma once

#include <SBF/diagnostic_result.h>
#include <SBF/dynamic_update_config.h>
#include <SBF/planning_config.h>
#include <SBF/subtractive_build_config.h>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

namespace rbf::python_binding {

#if defined(SBF_DIAGNOSTIC_API) && SBF_DIAGNOSTIC_API && \
    defined(SBF_PYTHON_DEBUG_METHODS) && SBF_PYTHON_DEBUG_METHODS
inline void register_diagnostic_types(py::module_& module) {
    py::class_<rbf::DynamicUpdateConfig>(module, "DynamicUpdateConfig")
        .def(py::init<>())
        .def_readwrite("enable_spatial_dirty_region",
                       &rbf::DynamicUpdateConfig::enable_spatial_dirty_region)
        .def_readwrite("dirty_region_padding", &rbf::DynamicUpdateConfig::dirty_region_padding)
        .def_readwrite("dirty_anchor_limit", &rbf::DynamicUpdateConfig::dirty_anchor_limit)
        .def_readwrite("dirty_seed_limit", &rbf::DynamicUpdateConfig::dirty_seed_limit)
        .def_readwrite("local_regrow_box_limit",
                       &rbf::DynamicUpdateConfig::local_regrow_box_limit)
        .def_readwrite("local_regrow_timeout_ms",
                       &rbf::DynamicUpdateConfig::local_regrow_timeout_ms)
        .def_readwrite("insertion_leaf_sweep_max_depth",
                       &rbf::DynamicUpdateConfig::insertion_leaf_sweep_max_depth)
        .def_readwrite("insertion_leaf_sweep_relative_depth",
                       &rbf::DynamicUpdateConfig::insertion_leaf_sweep_relative_depth)
        .def_readwrite("enable_warm_rebuild_fallback",
                       &rbf::DynamicUpdateConfig::enable_warm_rebuild_fallback)
        .def_readwrite("warm_rebuild_on_empty_forest",
                       &rbf::DynamicUpdateConfig::warm_rebuild_on_empty_forest)
        .def_readwrite("warm_rebuild_on_empty_dirty_region",
                       &rbf::DynamicUpdateConfig::warm_rebuild_on_empty_dirty_region)
        .def_readwrite("warm_rebuild_dirty_box_threshold",
                       &rbf::DynamicUpdateConfig::warm_rebuild_dirty_box_threshold)
        .def_readwrite("warm_rebuild_dirty_box_fraction",
                       &rbf::DynamicUpdateConfig::warm_rebuild_dirty_box_fraction)
        .def_readwrite("warm_rebuild_min_local_boxes_added",
                       &rbf::DynamicUpdateConfig::warm_rebuild_min_local_boxes_added);

    py::class_<rbf::SubtractiveObstacleGroup>(module, "SubtractiveObstacleGroup")
        .def(py::init<>())
        .def_readwrite("name", &rbf::SubtractiveObstacleGroup::name)
        .def_readwrite("carving_obstacles", &rbf::SubtractiveObstacleGroup::carving_obstacles)
        .def_readwrite("validation_obstacles",
                       &rbf::SubtractiveObstacleGroup::validation_obstacles);

    py::class_<rbf::SubtractiveBuildOptions>(module, "SubtractiveBuildOptions")
        .def(py::init<>())
        .def_readwrite("run_connector", &rbf::SubtractiveBuildOptions::run_connector)
        .def_readwrite("use_validation_obstacles_for_final_scene",
                       &rbf::SubtractiveBuildOptions::use_validation_obstacles_for_final_scene);

    py::class_<rbf::RebuildProfile>(module, "RebuildProfile")
        .def_readonly("boxes_before", &rbf::RebuildProfile::boxes_before)
        .def_readonly("boxes_after", &rbf::RebuildProfile::boxes_after)
        .def_readonly("boxes_removed", &rbf::RebuildProfile::boxes_removed)
        .def_readonly("raw_boxes_before", &rbf::RebuildProfile::raw_boxes_before)
        .def_readonly("raw_boxes_after", &rbf::RebuildProfile::raw_boxes_after)
        .def_readonly("raw_boxes_removed", &rbf::RebuildProfile::raw_boxes_removed)
        .def_readonly("adjacency_islands", &rbf::RebuildProfile::adjacency_islands)
        .def_readonly("obstacles_before", &rbf::RebuildProfile::obstacles_before)
        .def_readonly("obstacles_after", &rbf::RebuildProfile::obstacles_after)
        .def_readonly("removed_obstacle_index", &rbf::RebuildProfile::removed_obstacle_index)
        .def_readonly("dirty_boxes", &rbf::RebuildProfile::dirty_boxes)
        .def_readonly("dirty_boxes_used", &rbf::RebuildProfile::dirty_boxes_used)
        .def_readonly("dirty_seed_count", &rbf::RebuildProfile::dirty_seed_count)
        .def_readonly("regrow_attempts", &rbf::RebuildProfile::regrow_attempts)
        .def_readonly("boxes_added", &rbf::RebuildProfile::boxes_added)
        .def_readonly("raw_boxes_added", &rbf::RebuildProfile::raw_boxes_added)
        .def_readonly("bridge_boxes_added", &rbf::RebuildProfile::bridge_boxes_added)
        .def_readonly("segment_edges_added", &rbf::RebuildProfile::segment_edges_added)
        .def_readonly("rrt_segment_edges_added", &rbf::RebuildProfile::rrt_segment_edges_added)
        .def_readonly("point_gap_segment_edges_added", &rbf::RebuildProfile::point_gap_segment_edges_added)
        .def_readonly("collision_cache_boxes_before", &rbf::RebuildProfile::collision_cache_boxes_before)
        .def_readonly("collision_cache_boxes_after", &rbf::RebuildProfile::collision_cache_boxes_after)
        .def_readonly("collision_cache_candidates", &rbf::RebuildProfile::collision_cache_candidates)
        .def_readonly("collision_cache_promoted", &rbf::RebuildProfile::collision_cache_promoted)
        .def_readonly("collision_cache_rejected_collision", &rbf::RebuildProfile::collision_cache_rejected_collision)
        .def_readonly("collision_cache_rejected_contained", &rbf::RebuildProfile::collision_cache_rejected_contained)
        .def_readonly("collision_cache_rejected_disconnected",
                      &rbf::RebuildProfile::collision_cache_rejected_disconnected)
        .def_readonly("used_spatial_dirty_region", &rbf::RebuildProfile::used_spatial_dirty_region)
        .def_readonly("used_warm_rebuild", &rbf::RebuildProfile::used_warm_rebuild)
        .def_readonly("fallback_reason", &rbf::RebuildProfile::fallback_reason)
        .def_readonly("dirty_region_ms", &rbf::RebuildProfile::dirty_region_ms)
        .def_readonly("regrow_ms", &rbf::RebuildProfile::regrow_ms)
        .def_readonly("warm_rebuild_ms", &rbf::RebuildProfile::warm_rebuild_ms)
        .def_readonly("collision_check_ms", &rbf::RebuildProfile::collision_check_ms)
        .def_readonly("adjacency_ms", &rbf::RebuildProfile::adjacency_ms)
        .def_readonly("total_ms", &rbf::RebuildProfile::total_ms)
        .def_readonly("diagnostics", &rbf::RebuildProfile::diagnostics);
}

inline void register_planning_config_diagnostic_fields(
    py::class_<rbf::RBFPlanningConfig>& planning_config_class) {
    planning_config_class.def_readwrite("dynamic_update", &rbf::RBFPlanningConfig::dynamic_update);
}
#endif

}  // namespace rbf::python_binding
