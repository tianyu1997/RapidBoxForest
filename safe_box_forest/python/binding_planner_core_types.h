#pragma once

#include <SBF/grower_types.h>
#include <SBF/leaf_sweep_types.h>
#include <SBF/runtime_config.h>
#include <SBF/segment_edge_types.h>

#include <rbf/core.h>
#include <sbf/envelope/endpoint_source.h>
#include <sbf/envelope/envelope_type.h>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

namespace rbf::python_binding {

inline void register_planner_core_types(py::module_& module) {
    py::class_<rbf::KdopConfig>(module, "KdopConfig")
        .def(py::init<>())
        .def_readwrite("direction_set", &rbf::KdopConfig::direction_set)
        .def_readwrite("safety_epsilon", &rbf::KdopConfig::safety_epsilon)
        .def_readwrite("overlap_tolerance", &rbf::KdopConfig::overlap_tolerance);

    py::class_<rbf::SupportHullConfig>(module, "SupportHullConfig")
        .def(py::init<>())
        .def_readwrite("keep_kdop", &rbf::SupportHullConfig::keep_kdop)
        .def_readwrite("skip_aabb_broadphase", &rbf::SupportHullConfig::skip_aabb_broadphase)
        .def_readwrite("direct_collision", &rbf::SupportHullConfig::direct_collision)
        .def_readwrite("safety_epsilon", &rbf::SupportHullConfig::safety_epsilon)
        .def_readwrite("overlap_tolerance", &rbf::SupportHullConfig::overlap_tolerance);

    py::class_<rbf::EndpointSourceConfig>(module, "EndpointSourceConfig")
        .def(py::init<>())
        .def_readwrite("source", &rbf::EndpointSourceConfig::source)
        .def_readwrite("n_samples_crit", &rbf::EndpointSourceConfig::n_samples_crit)
        .def_readwrite("n_threads", &rbf::EndpointSourceConfig::n_threads)
        .def_readwrite("parallel_min_combos", &rbf::EndpointSourceConfig::parallel_min_combos)
        .def_readwrite("max_phase_analytical", &rbf::EndpointSourceConfig::max_phase_analytical)
        .def_readwrite("bypass_narrow_skip", &rbf::EndpointSourceConfig::bypass_narrow_skip)
        .def_readwrite("gcpc_match_analytical", &rbf::EndpointSourceConfig::gcpc_match_analytical)
        .def_readwrite("hifk_max_depth", &rbf::EndpointSourceConfig::hifk_max_depth)
        .def_readwrite("hifk_n_threads", &rbf::EndpointSourceConfig::hifk_n_threads)
        .def_readwrite("hifk_vol_ratio_thresh", &rbf::EndpointSourceConfig::hifk_vol_ratio_thresh)
        .def_readwrite("hifk_depth_offset", &rbf::EndpointSourceConfig::hifk_depth_offset)
        .def_readwrite("hifk_min_split_width", &rbf::EndpointSourceConfig::hifk_min_split_width)
        .def_readwrite("hifk_depth_dimensions", &rbf::EndpointSourceConfig::hifk_depth_dimensions)
        .def_readwrite("hifk_root_intervals", &rbf::EndpointSourceConfig::hifk_root_intervals);

    py::class_<rbf::EnvelopeTypeConfig>(module, "EnvelopeTypeConfig")
        .def(py::init<>())
        .def_readwrite("type", &rbf::EnvelopeTypeConfig::type)
        .def_readwrite("n_subdivisions", &rbf::EnvelopeTypeConfig::n_subdivisions)
        .def_readwrite("kdop_config", &rbf::EnvelopeTypeConfig::kdop_config)
        .def_readwrite("support_hull_config", &rbf::EnvelopeTypeConfig::support_hull_config);

    py::class_<rbf::BoxNode>(module, "BoxNode")
        .def(py::init<>())
        .def_readwrite("id", &rbf::BoxNode::id)
        .def_readwrite("joint_intervals", &rbf::BoxNode::joint_intervals)
        .def_readwrite("seed_config", &rbf::BoxNode::seed_config)
        .def_readwrite("volume", &rbf::BoxNode::volume)
        .def_readwrite("tree_id", &rbf::BoxNode::tree_id)
        .def_readwrite("parent_box_id", &rbf::BoxNode::parent_box_id)
        .def_readwrite("root_id", &rbf::BoxNode::root_id)
        .def_readwrite("safety_status", &rbf::BoxNode::safety_status)
        .def_readwrite("strict_audit_required", &rbf::BoxNode::strict_audit_required)
        .def("center", &rbf::BoxNode::center)
        .def("contains", &rbf::BoxNode::contains);

    py::class_<rbf::SegmentEdge>(module, "SegmentEdge")
        .def(py::init<>())
        .def_readwrite("id", &rbf::SegmentEdge::id)
        .def_readwrite("source_box_id", &rbf::SegmentEdge::source_box_id)
        .def_readwrite("target_box_id", &rbf::SegmentEdge::target_box_id)
        .def_readwrite("waypoints", &rbf::SegmentEdge::waypoints)
        .def_readwrite("internal_boxes", &rbf::SegmentEdge::internal_boxes)
        .def_readwrite("obb_centers", &rbf::SegmentEdge::obb_centers)
        .def_readwrite("obb_generators", &rbf::SegmentEdge::obb_generators)
        .def_readwrite("obb_covered_length", &rbf::SegmentEdge::obb_covered_length)
        .def_readwrite("type", &rbf::SegmentEdge::type)
        .def_readwrite("validation", &rbf::SegmentEdge::validation)
        .def_readwrite("segment_resolution", &rbf::SegmentEdge::segment_resolution)
        .def_readwrite("length", &rbf::SegmentEdge::length)
        .def_readwrite("strict_audit_required", &rbf::SegmentEdge::strict_audit_required)
        .def_readwrite("query_index", &rbf::SegmentEdge::query_index)
        .def_readwrite("portal_domain_id", &rbf::SegmentEdge::portal_domain_id)
        .def_readwrite("conservative_certificate", &rbf::SegmentEdge::conservative_certificate);

    py::enum_<rbf::GrowerConfig::Mode>(module, "GrowerMode")
        .value("RRT", rbf::GrowerConfig::Mode::RRT)
        .value("Frontwave", rbf::GrowerConfig::Mode::Frontwave);

    py::enum_<rbf::ExecutionMode>(module, "ExecutionMode")
        .value("Inline", rbf::ExecutionMode::Inline)
        .value("Parallel", rbf::ExecutionMode::Parallel);

    py::class_<rbf::RuntimeConfig>(module, "RuntimeConfig")
        .def(py::init<>())
        .def_readwrite("mode", &rbf::RuntimeConfig::mode)
        .def_readwrite("n_threads", &rbf::RuntimeConfig::n_threads)
        .def_readwrite("batch_size", &rbf::RuntimeConfig::batch_size)
        .def_readwrite("parallel_threshold", &rbf::RuntimeConfig::parallel_threshold)
        .def_readwrite("deterministic_reduce", &rbf::RuntimeConfig::deterministic_reduce);

    py::class_<rbf::LeafSweepConfig>(module, "LeafSweepConfig")
        .def(py::init<>())
        .def_readwrite("obstacle_cluster_gap", &rbf::LeafSweepConfig::obstacle_cluster_gap)
        .def_readwrite("n_threads", &rbf::LeafSweepConfig::n_threads)
        .def_readwrite("validation_batch_size", &rbf::LeafSweepConfig::validation_batch_size)
        .def_readwrite("timeout_ms", &rbf::LeafSweepConfig::timeout_ms)
        .def_readwrite("store_group_results", &rbf::LeafSweepConfig::store_group_results)
        .def_readwrite("pre_split_to_max_depth", &rbf::LeafSweepConfig::pre_split_to_max_depth)
        .def_readwrite("use_virtual_topology", &rbf::LeafSweepConfig::use_virtual_topology)
        .def_readwrite("parallel_virtual_validation",
                       &rbf::LeafSweepConfig::parallel_virtual_validation)
        .def_readwrite("collision_overlap_prune_min_threshold",
                       &rbf::LeafSweepConfig::collision_overlap_prune_min_threshold)
        .def_readwrite("collision_overlap_prune_decay_per_depth",
                       &rbf::LeafSweepConfig::collision_overlap_prune_decay_per_depth)
        .def_readwrite("max_free_boxes", &rbf::LeafSweepConfig::max_free_boxes)
        .def_readwrite("max_collision_boxes", &rbf::LeafSweepConfig::max_collision_boxes);

    py::class_<rbf::LeafSweepGroupResult>(module, "LeafSweepGroupResult")
        .def_readonly("group_id", &rbf::LeafSweepGroupResult::group_id)
        .def_readonly("obstacle_indices", &rbf::LeafSweepGroupResult::obstacle_indices)
        .def_readonly("obstacles", &rbf::LeafSweepGroupResult::obstacles)
        .def_readonly("aggregate_obstacle", &rbf::LeafSweepGroupResult::aggregate_obstacle)
        .def_readonly("free_boxes", &rbf::LeafSweepGroupResult::free_boxes)
        .def_readonly("collision_boxes", &rbf::LeafSweepGroupResult::collision_boxes);

    py::class_<rbf::LeafSweepResult>(module, "LeafSweepResult")
        .def_readonly("free_boxes", &rbf::LeafSweepResult::free_boxes)
        .def_readonly("collision_boxes", &rbf::LeafSweepResult::collision_boxes)
        .def_readonly("collision_box_obstacle_indices",
                      &rbf::LeafSweepResult::collision_box_obstacle_indices)
        .def_readonly("groups", &rbf::LeafSweepResult::groups)
        .def_readonly("obstacle_group_ids", &rbf::LeafSweepResult::obstacle_group_ids)
        .def_readonly("deadline_reached", &rbf::LeafSweepResult::deadline_reached)
        .def_readonly("initialize_ms", &rbf::LeafSweepResult::initialize_ms)
        .def_readonly("group_sweep_ms", &rbf::LeafSweepResult::group_sweep_ms)
        .def_readonly("compose_ms", &rbf::LeafSweepResult::compose_ms)
        .def_readonly("total_ms", &rbf::LeafSweepResult::total_ms)
        .def_readonly("diagnostics", &rbf::LeafSweepResult::diagnostics);
}

} // namespace rbf::python_binding
