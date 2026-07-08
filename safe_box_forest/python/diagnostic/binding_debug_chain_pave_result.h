#pragma once

#include <SBF/debug.h>

#include "../binding_utils.h"
#include "../binding_oracle_utils.h"

#include <pybind11/pybind11.h>

#include <utility>

namespace py = pybind11;

namespace rbf::python_binding {

inline py::dict debug_chain_pave_result_to_python(const rbf::DebugChainPaveResult& res) {
    py::dict result;
    result["added"] = res.added;
    result["bridge_found"] = res.bridge_found;
    result["audit_passed"] = res.audit_passed;
    result["start_box_id"] = res.start_box_id;
    result["goal_box_id"] = res.goal_box_id;
    result["fast_gap_fill_ffb_calls"] = res.fast_gap_fill_ffb_calls;
    result["fast_gap_fill_ms"] = res.fast_gap_fill_ms;
    result["boundary_ffb_calls"] = res.boundary_ffb_calls;
    result["boundary_commits"] = res.boundary_commits;
    result["boundary_reject_not_free"] = res.boundary_reject_not_free;
    result["boundary_reject_non_adjacent"] = res.boundary_reject_non_adjacent;
    result["boundary_fail_seed_collision"] = res.boundary_fail_seed_collision;
    result["boundary_fail_depth_cap"] = res.boundary_fail_depth_cap;
    result["boundary_fail_unknown_depth_cap"] = res.boundary_fail_unknown_depth_cap;
    result["boundary_fail_reserved_depth_cap"] = res.boundary_fail_reserved_depth_cap;
    result["boundary_fail_occupied"] = res.boundary_fail_occupied;
    result["boundary_fail_deadline"] = res.boundary_fail_deadline;
    result["boundary_fail_out_of_domain"] = res.boundary_fail_out_of_domain;
    result["boundary_fail_split"] = res.boundary_fail_split;
    result["boundary_failed_seed_memoized"] = res.boundary_failed_seed_memoized;
    result["boundary_skip_failed_seed"] = res.boundary_skip_failed_seed;
    result["boundary_stall"] = res.boundary_stall;
    result["boundary_target_hits"] = res.boundary_target_hits;

    py::list boundary_failures;
    for (const auto& failure : res.boundary_failures) {
        py::dict item;
        item["seed"] = failure.seed;
        item["intervals"] = interval_pairs_to_python(failure.intervals);
        item["validation_detail"] =
            oracle_validation_detail_to_python(failure.validation_detail);
        item["node"] = failure.node;
        item["depth"] = failure.depth;
        item["changed_dim"] = failure.changed_dim;
        item["fail_code"] = failure.fail_code;
        item["hit_unknown_depth_cap"] = failure.hit_unknown_depth_cap;
        item["hit_reserved_depth_cap"] = failure.hit_reserved_depth_cap;
        boundary_failures.append(std::move(item));
    }
    result["boundary_failures"] = std::move(boundary_failures);

    py::list waypoints;
    for (const auto& wp : res.waypoints) {
        waypoints.append(vector_to_list(wp));
    }
    result["waypoints"] = waypoints;

    py::list committed_boxes;
    for (const auto& box : res.committed_boxes) {
        committed_boxes.append(interval_pairs_to_python(box));
    }
    result["committed_boxes"] = committed_boxes;

    py::list all_boxes;
    for (const auto& box : res.all_boxes) {
        all_boxes.append(interval_pairs_to_python(box));
    }
    result["all_boxes"] = all_boxes;
    result["start_box"] = interval_pairs_to_python(res.start_box);
    result["goal_box"] = interval_pairs_to_python(res.goal_box);
    return result;
}

}  // namespace rbf::python_binding
