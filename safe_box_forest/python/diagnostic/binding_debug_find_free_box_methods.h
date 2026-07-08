#pragma once

#include <SBF/find_free_box.h>
#include <SBF/oracle.h>
#include <SBF/safe_box_forest.h>

#include "../binding_utils.h"
#include "../binding_oracle_utils.h"

#include <rbf/lect_database/read_snapshot.h>

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace py = pybind11;

namespace rbf::python_binding {

inline void register_debug_find_free_box_methods(py::class_<rbf::RBFPlanningForest>& forest_class) {
    forest_class
        .def("debug_find_free_box",
             [](rbf::RBFPlanningForest& forest,
                const std::vector<double>& seed_values,
                const std::vector<rbf::Obstacle>& obstacles,
                const rbf::FindFreeBoxOptions& options,
                bool disable_caches) {
                 using Clock = std::chrono::steady_clock;
                 const Eigen::VectorXd seed = eigen_vector_from_list(seed_values);
                 if (seed.size() != forest.robot().n_joints()) {
                     throw std::invalid_argument("seed dimension must match robot.n_joints()");
                 }

                 rbf::OracleValidationConfig validation = disable_caches
                     ? uncached_validation_config(forest.config().validation)
                     : forest.config().validation;
                 rbf::lect_database::OnlineEnvelopeCacheTree cache(forest.database(), {});
                 std::unique_ptr<rbf::lect_database::LectReadSnapshot> external_snapshot;
                 std::unique_ptr<rbf::lect_database::LectSnapshotEvidenceSource> external_snapshot_source;
                 const rbf::lect_database::LectExternalEvidenceSource* external_source = nullptr;
                 const rbf::lect_database::LectDatabase* direct_external_database = nullptr;
                 if (!disable_caches && !forest.config().database.external_evidence_path.empty()) {
                     const auto& runtime = forest.config().database;
                     if (runtime.external_evidence_use_snapshot) {
                         const auto snapshot_path = runtime.external_evidence_snapshot_path.empty()
                             ? rbf::lect_database::LectReadSnapshot::default_snapshot_path(runtime.external_evidence_path)
                             : runtime.external_evidence_snapshot_path;
                         external_snapshot = std::make_unique<rbf::lect_database::LectReadSnapshot>();
                         std::string reason;
                         if (!external_snapshot->open(snapshot_path, &reason)) {
                             external_snapshot.reset();
                         } else {
                             external_snapshot_source =
                                 std::make_unique<rbf::lect_database::LectSnapshotEvidenceSource>(*external_snapshot);
                             external_source = external_snapshot_source.get();
                         }
                     }
                 }
                 rbf::DatabaseBoxOracle oracle(forest.robot(),
                                               cache,
                                               rbf::Scene(obstacles),
                                               forest.config().endpoint_source,
                                               forest.config().envelope_type,
                                               validation,
                                               external_source,
                                               direct_external_database);

                 py::dict result;
                 py::list trace;
                 py::list split_events;
                 py::list validation_events;
                 const auto start = Clock::now();
                 result["external_source_enabled"] = external_source != nullptr;
                 result["external_direct_database_enabled"] = direct_external_database != nullptr;

                 const Eigen::VectorXd tree_seed = oracle.tree_configuration_for_query(seed);
                 result["seed"] = seed_values;
                 result["tree_seed"] = vector_to_list(tree_seed);
                 result["root_intervals"] = interval_pairs_to_python(oracle.root_intervals());
                 result["disable_caches"] = disable_caches;
                 result["trace_mode"] = "linear_trace_not_production_ffb";

                 // Seed-independent: canonical split depends only on (robot,
                 // domain). No query-seed coupling is applied to split values.
                 rbf::OracleSplitOptions split_options = options.split;

                 bool seed_in_domain = false;
                 if (seed.size() == oracle.n_dims()) {
                     seed_in_domain = oracle.contains_point(oracle.root_node(), seed);
                 }
                 result["seed_in_domain"] = seed_in_domain;
                 if (seed.size() != oracle.n_dims() || !seed_in_domain) {
                     result["found"] = false;
                     result["seed_collision"] = false;
                     result["hit_reserved_depth_cap"] = false;
                     result["hit_unknown_depth_cap"] = false;
                     result["deadline_reached"] = false;
                     result["fail_code"] = 5;
                     result["node"] = rbf::kInvalidOracleNodeId;
                     result["decisions"] = 0;
                     result["splits"] = 0;
                     result["changed_dim"] = -1;
                     result["intervals"] = py::list();
                     result["trace"] = trace;
                     result["validation_events"] = validation_events;
                     result["split_events"] = split_events;
                     result["counters"] = oracle_counters_to_python(oracle.counters());
                     result["total_ms"] = 0.0;
                     return result;
                 }

                 if (options.reject_seed_collision && oracle.point_in_collision(seed)) {
                     result["found"] = false;
                     result["seed_collision"] = true;
                     result["hit_reserved_depth_cap"] = false;
                     result["hit_unknown_depth_cap"] = false;
                     result["deadline_reached"] = false;
                     result["fail_code"] = 1;
                     result["node"] = rbf::kInvalidOracleNodeId;
                     result["decisions"] = 0;
                     result["splits"] = 0;
                     result["changed_dim"] = -1;
                     result["intervals"] = py::list();
                     result["trace"] = trace;
                     result["validation_events"] = validation_events;
                     result["split_events"] = split_events;
                     result["counters"] = oracle_counters_to_python(oracle.counters());
                     result["total_ms"] = 0.0;
                     return result;
                 }

                 auto elapsed_ms = [&]() {
                     return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
                 };

                 bool found = false;
                 bool seed_collision = false;
                 bool hit_reserved_depth_cap = false;
                 bool hit_unknown_depth_cap = false;
                 bool deadline_reached = false;
                 int fail_code = 0;
                 rbf::OracleNodeId node = oracle.root_node();
                 int changed_dim = -1;
                 int decisions = 0;
                 int splits = 0;
                 int result_changed_dim = -1;
                 std::vector<rbf::Interval> result_intervals;
                 rbf::OracleValidationDetail final_detail;
                 const int effective_max_depth = std::max(0, std::min(options.max_depth, oracle.max_tree_depth() - 1));
                 std::uint64_t step_sequence = 0;
                 std::uint64_t split_sequence = 0;
                 std::uint64_t validation_sequence = 0;

                 while (true) {
                     py::dict step;
                     step["sequence"] = step_sequence++;
                     step["node"] = node;
                     step["depth"] = oracle.depth(node);
                     step["changed_dim_in"] = changed_dim;
                     const auto tree_intervals = oracle.node_intervals(node);
                     const auto query_intervals = oracle.query_intervals_for_node(node, tree_intervals, seed);
                     step["tree_intervals"] = interval_pairs_to_python(tree_intervals);
                     step["query_intervals"] = interval_pairs_to_python(query_intervals);
                     step["is_leaf"] = oracle.is_leaf(node);
                     step["is_reserved"] = oracle.is_reserved(node);

                     if (step.cast<py::dict>()["is_reserved"].cast<bool>()) {
                         if (oracle.depth(node) >= effective_max_depth || !options.split_reserved_leaf) {
                             hit_reserved_depth_cap = true;
                             result_intervals = query_intervals;
                             fail_code = 2;
                             step["terminal"] = true;
                             step["fail_code"] = fail_code;
                             trace.append(std::move(step));
                             break;
                         }
                         if (oracle.is_leaf(node)) {
                             const auto split = oracle.split_node(node, tree_intervals, changed_dim, split_options);
                             step["split"] = split.split;
                             if (!split.split) {
                                 fail_code = 6;
                                 step["terminal"] = true;
                                 step["fail_code"] = fail_code;
                                 trace.append(std::move(step));
                                 break;
                             }
                             step["split_dim"] = split.split_dim;
                             step["split_value"] = split.split_value;
                             splits += 1;
                             py::dict split_event;
                             split_event["sequence"] = split_sequence++;
                             split_event["node"] = split.node;
                             split_event["depth"] = oracle.depth(split.node);
                             split_event["split_dim"] = split.split_dim;
                             split_event["split_val"] = split.split_value;
                             split_event["best_tighten"] = options.split.use_best_tighten;
                             split_event["sector_aligned"] = false;
                             split_events.append(std::move(split_event));
                         }
                         changed_dim = oracle.split_dim(node);
                         const double next_split_value = oracle.split_value(node);
                         const bool go_left = tree_seed[changed_dim] <= next_split_value;
                         step["next_split_dim"] = changed_dim;
                         step["next_split_value"] = next_split_value;
                         step["child_branch"] = go_left ? "left" : "right";
                         trace.append(std::move(step));
                         node = go_left ? oracle.left_child(node) : oracle.right_child(node);
                         continue;
                     }

                     const auto validation_result = oracle.validate_node(node, query_intervals, changed_dim);
                     final_detail = oracle.last_validation_detail();
                     decisions += 1;
                     step["validation"] = static_cast<int>(validation_result);
                     step["validation_detail"] = oracle_validation_detail_to_python(final_detail);
                     py::dict validation_event;
                     validation_event["sequence"] = validation_sequence++;
                     validation_event["node"] = node;
                     validation_event["depth"] = oracle.depth(node);
                     validation_event["validation"] = static_cast<int>(validation_result);
                     validation_event["safety_status"] = static_cast<int>(final_detail.safety_status);
                     validation_event["collision_possible"] = final_detail.collision_possible;
                     validation_event["strict_audit_required"] = final_detail.strict_audit_required;
                     validation_event["intervals"] = interval_pairs_to_python(query_intervals);
                     validation_events.append(std::move(validation_event));

                     if (validation_result == rbf::BoxValidation::Free) {
                         found = true;
                         result_changed_dim = changed_dim;
                         result_intervals = query_intervals;
                         fail_code = 0;
                         step["terminal"] = true;
                         step["fail_code"] = fail_code;
                         trace.append(std::move(step));
                         break;
                     }
                     if (validation_result == rbf::BoxValidation::Occupied) {
                         result_intervals = query_intervals;
                         fail_code = 3;
                         step["terminal"] = true;
                         step["fail_code"] = fail_code;
                         trace.append(std::move(step));
                         break;
                     }
                     if (oracle.depth(node) >= effective_max_depth || !options.split_unknown_leaf) {
                         hit_unknown_depth_cap = true;
                         result_intervals = query_intervals;
                         fail_code = 2;
                         step["terminal"] = true;
                         step["fail_code"] = fail_code;
                         trace.append(std::move(step));
                         break;
                     }
                     if (oracle.is_leaf(node)) {
                         const auto split = oracle.split_node(node, tree_intervals, changed_dim, split_options);
                         step["split"] = split.split;
                         if (!split.split) {
                             fail_code = 6;
                             step["terminal"] = true;
                             step["fail_code"] = fail_code;
                             trace.append(std::move(step));
                             break;
                         }
                         step["split_dim"] = split.split_dim;
                         step["split_value"] = split.split_value;
                         splits += 1;
                         py::dict split_event;
                         split_event["sequence"] = split_sequence++;
                         split_event["node"] = split.node;
                         split_event["depth"] = oracle.depth(split.node);
                         split_event["split_dim"] = split.split_dim;
                         split_event["split_val"] = split.split_value;
                         split_event["best_tighten"] = options.split.use_best_tighten;
                         split_event["sector_aligned"] = false;
                         split_events.append(std::move(split_event));
                     }
                     changed_dim = oracle.split_dim(node);
                     const double next_split_value = oracle.split_value(node);
                     const bool go_left = tree_seed[changed_dim] <= next_split_value;
                     step["next_split_dim"] = changed_dim;
                     step["next_split_value"] = next_split_value;
                     step["child_branch"] = go_left ? "left" : "right";
                     trace.append(std::move(step));
                     node = go_left ? oracle.left_child(node) : oracle.right_child(node);
                 }

                 result["found"] = found;
                 result["seed_collision"] = seed_collision;
                 result["hit_reserved_depth_cap"] = hit_reserved_depth_cap;
                 result["hit_unknown_depth_cap"] = hit_unknown_depth_cap;
                 result["deadline_reached"] = deadline_reached;
                 result["fail_code"] = fail_code;
                 result["node"] = node;
                 result["decisions"] = decisions;
                 result["splits"] = splits;
                 result["changed_dim"] = result_changed_dim;
                 result["intervals"] = interval_pairs_to_python(result_intervals);
                 result["validation_detail"] = oracle_validation_detail_to_python(final_detail);
                 result["trace"] = trace;
                 result["validation_events"] = validation_events;
                 result["split_events"] = split_events;
                 result["counters"] = oracle_counters_to_python(oracle.counters());
                 result["effective_max_depth"] = effective_max_depth;
                 result["total_ms"] = elapsed_ms();
                 return result;
             },
             py::arg("seed"),
             py::arg("obstacles"),
             py::arg("options") = rbf::FindFreeBoxOptions{},
             py::arg("disable_caches") = true);
}

}  // namespace rbf::python_binding
