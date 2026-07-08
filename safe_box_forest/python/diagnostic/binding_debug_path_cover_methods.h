#pragma once

#include <SBF/box_graph.h>
#include <SBF/find_free_box.h>
#include <SBF/oracle.h>
#include <SBF/runtime.h>
#include <SBF/safe_box_forest.h>

#include "../binding_utils.h"
#include "../binding_oracle_utils.h"

#include <rbf/lect_database/read_snapshot.h>

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace py = pybind11;

namespace rbf::python_binding {

inline void register_debug_path_cover_methods(
    py::class_<rbf::RBFPlanningForest>& forest_class) {
    forest_class
        .def("debug_cover_path_with_ffb",
             [](rbf::RBFPlanningForest& forest,
                const std::vector<std::vector<double>>& waypoint_values,
                const std::vector<rbf::Obstacle>& obstacles,
                const rbf::FindFreeBoxOptions& options,
                double sample_step,
                int max_ffb_calls,
                double coverage_tolerance,
                bool include_existing_boxes,
                bool disable_caches,
                int max_failure_records,
                const std::vector<double>& refine_sample_steps,
                int parallel_workers,
                bool repair_corridor_adjacency,
                int repair_rounds,
                int repair_segment_subdivisions) {
                 using Clock = std::chrono::steady_clock;
                 const auto start = Clock::now();
                 const std::vector<Eigen::VectorXd> waypoints =
                     eigen_vectors_from_lists(waypoint_values);
                 if (waypoints.empty()) {
                     throw std::invalid_argument("waypoint path must be non-empty");
                 }
                 for (const auto& waypoint : waypoints) {
                     if (waypoint.size() != forest.robot().n_joints()) {
                         throw std::invalid_argument("waypoint dimension must match robot.n_joints()");
                     }
                 }
                 std::vector<double> pass_steps;
                 if (!refine_sample_steps.empty()) {
                     pass_steps.reserve(refine_sample_steps.size());
                     for (double step : refine_sample_steps) {
                         if (std::isfinite(step) && step > 0.0) {
                             pass_steps.push_back(step);
                         }
                     }
                 }
                 if (pass_steps.empty()) {
                     pass_steps.push_back(sample_step);
                 }
                 const double final_sample_step = *std::min_element(pass_steps.begin(), pass_steps.end());
                 std::vector<Eigen::VectorXd> samples =
                     densify_path_pybind(waypoints, final_sample_step);
                 const double path_length = rbf::path_length(waypoints);

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
                 rbf::FindFreeBoxService ffb(oracle);
                 rbf::StageContext context = rbf::StageContext::from_runtime(forest.config().runtime);

                 std::vector<std::vector<rbf::Interval>> boxes;
                 if (include_existing_boxes) {
                     for (const auto& box : forest.boxes()) {
                         boxes.push_back(box.joint_intervals);
                     }
                 }
                 const int initial_box_count = static_cast<int>(boxes.size());
                 std::vector<bool> covered(samples.size(), false);
                 std::vector<std::vector<int>> sample_layers(samples.size());
                 auto mark_covered = [&](std::size_t from_index) {
                     int changed = 0;
                     for (std::size_t sample_index = 0; sample_index < samples.size(); ++sample_index) {
                         for (std::size_t box_index = from_index; box_index < boxes.size(); ++box_index) {
                             if (intervals_contain_point_pybind(boxes[box_index],
                                                                samples[sample_index],
                                                                coverage_tolerance)) {
                                 auto& layer = sample_layers[sample_index];
                                 const int box_id = static_cast<int>(box_index);
                                 if (std::find(layer.begin(), layer.end(), box_id) == layer.end()) {
                                     layer.push_back(box_id);
                                 }
                                 if (!covered[sample_index]) {
                                     covered[sample_index] = true;
                                     changed += 1;
                                 }
                                 if (from_index == 0) {
                                     break;
                                 }
                                 if (box_index + 1 == boxes.size()) {
                                     break;
                                 }
                             }
                             if (from_index == 0 && covered[sample_index]) {
                                 break;
                             }
                         }
                     }
                     return changed;
                 };
                 mark_covered(0);

                 int ffb_calls = 0;
                 int found = 0;
                 int rejected_not_containing_seed = 0;
                 std::vector<int> fail_counts(16, 0);
                 py::list failures;
                 double presplit_ms = 0.0;
                 double repair_ms = 0.0;
                 int repair_calls = 0;
                 int repair_added = 0;
                 int repair_bad_transitions_initial = 0;
                 int repair_bad_transitions_final = 0;
                 double repair_bad_transition_length_initial = 0.0;
                 double repair_bad_transition_length_final = 0.0;
                 rbf::OracleCounters result_counters_override;
                 bool use_counter_override = false;
                 py::list pass_summaries;
                 auto point_covered_by_boxes = [&](const Eigen::VectorXd& point) {
                     for (const auto& box : boxes) {
                         if (intervals_contain_point_pybind(box, point, coverage_tolerance)) {
                             return true;
                         }
                     }
                     return false;
                 };
                 if (parallel_workers > 1 && pass_steps.size() == 1) {
                     const auto presplit_start = Clock::now();
                     const int effective_max_depth =
                         std::max(0, std::min(options.max_depth, oracle.max_tree_depth() - 1));
                     auto presplit_seed = [&](const Eigen::VectorXd& seed) {
                         if (seed.size() != oracle.n_dims() ||
                             !oracle.contains_point(oracle.root_node(), seed)) {
                             return;
                         }
                         rbf::OracleNodeId node = oracle.root_node();
                         int changed_dim = -1;
                         while (node != rbf::kInvalidOracleNodeId &&
                                oracle.depth(node) < effective_max_depth) {
                             auto tree_intervals = oracle.node_intervals(node);
                             if (oracle.is_leaf(node)) {
                                 const auto split = oracle.split_node(node,
                                                                      tree_intervals,
                                                                      changed_dim,
                                                                      options.split);
                                 if (!split.split) {
                                     return;
                                 }
                             }
                             changed_dim = oracle.split_dim(node);
                             node = oracle.child_containing_point(node, seed);
                         }
                     };
                     for (const auto& seed : samples) {
                         if (!point_covered_by_boxes(seed)) {
                             presplit_seed(seed);
                         }
                     }
                     presplit_ms =
                         std::chrono::duration<double, std::milli>(Clock::now() - presplit_start).count();

                     std::vector<std::size_t> candidate_indices;
                     candidate_indices.reserve(samples.size());
                     for (std::size_t sample_index = 0; sample_index < samples.size(); ++sample_index) {
                         if (!point_covered_by_boxes(samples[sample_index])) {
                             candidate_indices.push_back(sample_index);
                             if (max_ffb_calls >= 0 &&
                                 static_cast<int>(candidate_indices.size()) >= max_ffb_calls) {
                                 break;
                             }
                         }
                     }
                     struct ParallelCoverResult {
                         bool ran = false;
                         std::size_t sample_index = 0;
                         rbf::FindFreeBoxResult ffb;
                     };
                     std::vector<ParallelCoverResult> parallel_results(candidate_indices.size());
                     std::vector<rbf::OracleCounters> worker_counters(
                         static_cast<std::size_t>(std::max(1, parallel_workers)));
                     std::atomic<std::size_t> next{0};
                     const int worker_count =
                         std::max(1, std::min(parallel_workers, static_cast<int>(candidate_indices.size())));
                     std::vector<std::thread> workers;
                     workers.reserve(static_cast<std::size_t>(worker_count));
                     const auto parallel_start = Clock::now();
                     for (int worker_id = 0; worker_id < worker_count; ++worker_id) {
                         workers.emplace_back([&, worker_id]() {
                             rbf::OracleValidationConfig local_validation = validation;
                             rbf::lect_database::OnlineEnvelopeCacheTree local_cache(forest.database(), {});
                             rbf::DatabaseBoxOracle local_oracle(forest.robot(),
                                                                 local_cache,
                                                                 rbf::Scene(obstacles),
                                                                 forest.config().endpoint_source,
                                                                 forest.config().envelope_type,
                                                                 local_validation,
                                                                 nullptr,
                                                                 nullptr);
                             rbf::FindFreeBoxService local_ffb(local_oracle);
                             rbf::StageContext local_context = rbf::StageContext::serial();
                             rbf::FindFreeBoxOptions local_options = options;
                             local_options.split_unknown_leaf = false;
                             local_options.split_reserved_leaf = false;
                             while (true) {
                                 const std::size_t item = next.fetch_add(1);
                                 if (item >= candidate_indices.size()) {
                                     break;
                                 }
                                 const std::size_t sample_index = candidate_indices[item];
                                 ParallelCoverResult out;
                                 out.ran = true;
                                 out.sample_index = sample_index;
                                 out.ffb = local_ffb.find(samples[sample_index],
                                                          local_context,
                                                          local_options);
                                 parallel_results[item] = std::move(out);
                             }
                             worker_counters[static_cast<std::size_t>(worker_id)] =
                                 local_oracle.counters();
                         });
                     }
                     for (auto& worker : workers) {
                         worker.join();
                     }
                     const double parallel_ms =
                         std::chrono::duration<double, std::milli>(Clock::now() - parallel_start).count();
                     ffb_calls = static_cast<int>(candidate_indices.size());
                     int pass_found = 0;
                     for (const auto& item : parallel_results) {
                         if (!item.ran) {
                             continue;
                         }
                         const auto& result = item.ffb;
                         const auto& seed = samples[item.sample_index];
                         if (result.found &&
                             intervals_contain_point_pybind(result.intervals,
                                                            seed,
                                                            coverage_tolerance)) {
                             const std::size_t new_box_index = boxes.size();
                             boxes.push_back(result.intervals);
                             found += 1;
                             pass_found += 1;
                             mark_covered(new_box_index);
                             continue;
                         }
                         if (result.found) {
                             rejected_not_containing_seed += 1;
                         }
                         const int fail_code = result.found ? -1 : result.fail_code;
                         if (fail_code >= 0 && fail_code < static_cast<int>(fail_counts.size())) {
                             fail_counts[static_cast<std::size_t>(fail_code)] += 1;
                         }
                         if (failures.size() < static_cast<py::ssize_t>(std::max(0, max_failure_records))) {
                             py::dict failure;
                             failure["sample_index"] = static_cast<int>(item.sample_index);
                             failure["sample_step"] = final_sample_step;
                             failure["sample"] = vector_to_list(seed);
                             failure["found"] = result.found;
                             failure["fail_code"] = result.fail_code;
                             failure["hit_unknown_depth_cap"] = result.hit_unknown_depth_cap;
                             failure["hit_reserved_depth_cap"] = result.hit_reserved_depth_cap;
                             failure["seed_collision"] = result.seed_collision;
                             failure["deadline_reached"] = result.deadline_reached;
                             failure["intervals"] = interval_pairs_to_python(result.intervals);
                             failures.append(std::move(failure));
                         }
                     }
                     py::dict pass_summary;
                     pass_summary["sample_step"] = final_sample_step;
                     pass_summary["sample_count"] = static_cast<int>(samples.size());
                     pass_summary["skipped"] =
                         static_cast<int>(samples.size()) - static_cast<int>(candidate_indices.size());
                     pass_summary["ffb_calls"] = static_cast<int>(candidate_indices.size());
                     pass_summary["ffb_found"] = pass_found;
                     pass_summary["presplit_ms"] = presplit_ms;
                     pass_summary["parallel_ms"] = parallel_ms;
                     pass_summary["ms"] = presplit_ms + parallel_ms;
                     pass_summary["parallel_workers"] = worker_count;
                     pass_summaries.append(std::move(pass_summary));
                     result_counters_override = oracle.counters();
                     for (const auto& counters : worker_counters) {
                         result_counters_override.node_validations += counters.node_validations;
                         result_counters_override.interval_validations += counters.interval_validations;
                         result_counters_override.certified_free += counters.certified_free;
                         result_counters_override.certified_occupied += counters.certified_occupied;
                         result_counters_override.provisional_free += counters.provisional_free;
                         result_counters_override.collision_possible += counters.collision_possible;
                         result_counters_override.materializations += counters.materializations;
                         result_counters_override.materialization_endpoint_time_us += counters.materialization_endpoint_time_us;
                         result_counters_override.materialization_envelope_time_us += counters.materialization_envelope_time_us;
                         result_counters_override.validate_node_total_time_us += counters.validate_node_total_time_us;
                         result_counters_override.materialization_external_exact_hits += counters.materialization_external_exact_hits;
                         result_counters_override.materialization_external_exact_misses += counters.materialization_external_exact_misses;
                         result_counters_override.interval_replay_compatibility_checks += counters.interval_replay_compatibility_checks;
                         result_counters_override.interval_replay_compatible += counters.interval_replay_compatible;
                         result_counters_override.interval_replay_incompatible += counters.interval_replay_incompatible;
                         result_counters_override.interval_replay_direct_exact_hits += counters.interval_replay_direct_exact_hits;
                         result_counters_override.interval_replay_key_only_blocked += counters.interval_replay_key_only_blocked;
                         result_counters_override.canonical_frame_invalid += counters.canonical_frame_invalid;
                         result_counters_override.canonical_reflected_seed_misses += counters.canonical_reflected_seed_misses;
                     }
                     use_counter_override = true;
                 } else {
                 for (double pass_step : pass_steps) {
                     const auto pass_start = Clock::now();
                     const std::vector<Eigen::VectorXd> pass_samples =
                         densify_path_pybind(waypoints, pass_step);
                     const bool pass_uses_final_samples =
                         pass_samples.size() == samples.size() &&
                         std::abs(pass_step - final_sample_step) <=
                             1e-12 * std::max(1.0, final_sample_step);
                     int pass_calls = 0;
                     int pass_found = 0;
                     int pass_skipped = 0;
                     for (std::size_t sample_index = 0; sample_index < pass_samples.size(); ++sample_index) {
                         if ((pass_uses_final_samples && covered[sample_index]) ||
                             (!pass_uses_final_samples && point_covered_by_boxes(pass_samples[sample_index]))) {
                             pass_skipped += 1;
                             continue;
                         }
                         if (max_ffb_calls >= 0 && ffb_calls >= max_ffb_calls) {
                             break;
                         }
                         const rbf::FindFreeBoxResult result =
                             ffb.find(pass_samples[sample_index], context, options);
                         ffb_calls += 1;
                         pass_calls += 1;
                         if (result.found &&
                             intervals_contain_point_pybind(result.intervals,
                                                            pass_samples[sample_index],
                                                            coverage_tolerance)) {
                             const std::size_t new_box_index = boxes.size();
                             boxes.push_back(result.intervals);
                             found += 1;
                             pass_found += 1;
                             mark_covered(new_box_index);
                             continue;
                         }
                         if (result.found) {
                             rejected_not_containing_seed += 1;
                         }
                         const int fail_code = result.found ? -1 : result.fail_code;
                         if (fail_code >= 0 && fail_code < static_cast<int>(fail_counts.size())) {
                             fail_counts[static_cast<std::size_t>(fail_code)] += 1;
                         }
                         if (failures.size() < static_cast<py::ssize_t>(std::max(0, max_failure_records))) {
                             py::dict failure;
                             failure["sample_index"] = static_cast<int>(sample_index);
                             failure["sample_step"] = pass_step;
                             failure["sample"] = vector_to_list(pass_samples[sample_index]);
                             failure["found"] = result.found;
                             failure["fail_code"] = result.fail_code;
                             failure["hit_unknown_depth_cap"] = result.hit_unknown_depth_cap;
                             failure["hit_reserved_depth_cap"] = result.hit_reserved_depth_cap;
                             failure["seed_collision"] = result.seed_collision;
                             failure["deadline_reached"] = result.deadline_reached;
                             failure["intervals"] = interval_pairs_to_python(result.intervals);
                             failures.append(std::move(failure));
                         }
                     }
                     py::dict pass_summary;
                     pass_summary["sample_step"] = pass_step;
                     pass_summary["sample_count"] = static_cast<int>(pass_samples.size());
                     pass_summary["skipped"] = pass_skipped;
                     pass_summary["ffb_calls"] = pass_calls;
                     pass_summary["ffb_found"] = pass_found;
                     pass_summary["ms"] =
                         std::chrono::duration<double, std::milli>(Clock::now() - pass_start).count();
                     pass_summaries.append(std::move(pass_summary));
                     if (max_ffb_calls >= 0 && ffb_calls >= max_ffb_calls) {
                         break;
                     }
                 }
                 }

                 auto sample_cover_layers = [&]() {
                     return sample_layers;
                 };
                 auto same_box = [](const std::vector<rbf::Interval>& lhs,
                                    const std::vector<rbf::Interval>& rhs) {
                     if (lhs.size() != rhs.size()) {
                         return false;
                     }
                     for (std::size_t dim = 0; dim < lhs.size(); ++dim) {
                         if (std::abs(lhs[dim].lo - rhs[dim].lo) > 1e-12 ||
                             std::abs(lhs[dim].hi - rhs[dim].hi) > 1e-12) {
                             return false;
                         }
                     }
                     return true;
                 };
                 auto duplicate_box = [&](const std::vector<rbf::Interval>& candidate) {
                     for (const auto& existing : boxes) {
                         if (same_box(existing, candidate)) {
                             return true;
                         }
                     }
                     return false;
                 };
                 auto transition_connected_local =
                     [&](int transition,
                         const std::vector<std::vector<int>>& layers,
                         const std::vector<int>& bridge_indices) {
                         if (transition < 0 ||
                             transition + 1 >= static_cast<int>(layers.size())) {
                             return false;
                         }
                         const auto& left_layer = layers[static_cast<std::size_t>(transition)];
                         const auto& right_layer = layers[static_cast<std::size_t>(transition + 1)];
                         if (left_layer.empty() || right_layer.empty()) {
                             return false;
                         }
                         std::vector<int> nodes;
                         nodes.reserve(left_layer.size() + right_layer.size() + bridge_indices.size());
                         nodes.insert(nodes.end(), left_layer.begin(), left_layer.end());
                         nodes.insert(nodes.end(), right_layer.begin(), right_layer.end());
                         nodes.insert(nodes.end(), bridge_indices.begin(), bridge_indices.end());
                         std::sort(nodes.begin(), nodes.end());
                         nodes.erase(std::unique(nodes.begin(), nodes.end()), nodes.end());
                         std::vector<int> parent(nodes.size());
                         for (std::size_t index = 0; index < parent.size(); ++index) {
                             parent[index] = static_cast<int>(index);
                         }
                         auto find_local = [&](int value) {
                             int root = value;
                             while (parent[static_cast<std::size_t>(root)] != root) {
                                 root = parent[static_cast<std::size_t>(root)];
                             }
                             while (parent[static_cast<std::size_t>(value)] != value) {
                                 const int next = parent[static_cast<std::size_t>(value)];
                                 parent[static_cast<std::size_t>(value)] = root;
                                 value = next;
                             }
                             return root;
                         };
                         auto unite_local = [&](int lhs, int rhs) {
                             const int left = find_local(lhs);
                             const int right = find_local(rhs);
                             if (left != right) {
                                 parent[static_cast<std::size_t>(right)] = left;
                             }
                         };
                         for (std::size_t i = 0; i < nodes.size(); ++i) {
                             for (std::size_t j = i + 1; j < nodes.size(); ++j) {
                                 if (interval_boxes_connected_pybind(boxes[static_cast<std::size_t>(nodes[i])],
                                                                     boxes[static_cast<std::size_t>(nodes[j])],
                                                                     coverage_tolerance)) {
                                     unite_local(static_cast<int>(i), static_cast<int>(j));
                                 }
                             }
                         }
                         for (int lhs_box : left_layer) {
                             const auto lhs_it = std::lower_bound(nodes.begin(), nodes.end(), lhs_box);
                             if (lhs_it == nodes.end() || *lhs_it != lhs_box) {
                                 continue;
                             }
                             const int lhs_local =
                                 static_cast<int>(std::distance(nodes.begin(), lhs_it));
                             const int lhs_root = find_local(lhs_local);
                             for (int rhs_box : right_layer) {
                                 const auto rhs_it = std::lower_bound(nodes.begin(), nodes.end(), rhs_box);
                                 if (rhs_it == nodes.end() || *rhs_it != rhs_box) {
                                     continue;
                                 }
                                 const int rhs_local =
                                     static_cast<int>(std::distance(nodes.begin(), rhs_it));
                                 if (lhs_root == find_local(rhs_local)) {
                                     return true;
                                 }
                             }
                         }
                         return false;
                     };
                 auto bad_transitions_for =
                     [&](const std::vector<std::vector<int>>& layers,
                         const std::vector<std::vector<int>>& transition_bridges) {
                         std::vector<int> bad;
                         if (layers.empty()) {
                             return bad;
                         }
                         for (std::size_t sample_index = 0; sample_index + 1 < layers.size(); ++sample_index) {
                             const std::vector<int> empty_bridges;
                             const auto& bridges =
                                 sample_index < transition_bridges.size()
                                     ? transition_bridges[sample_index]
                                     : empty_bridges;
                             if (!transition_connected_local(static_cast<int>(sample_index),
                                                             layers,
                                                             bridges)) {
                                 bad.push_back(static_cast<int>(sample_index));
                             }
                         }
                         return bad;
                     };
                 auto transition_length_sum = [&](const std::vector<int>& transitions) {
                     double total = 0.0;
                     for (int transition : transitions) {
                         if (transition < 0 ||
                             transition + 1 >= static_cast<int>(samples.size())) {
                             continue;
                         }
                         total += (samples[static_cast<std::size_t>(transition + 1)] -
                                   samples[static_cast<std::size_t>(transition)])
                                      .norm();
                     }
                     return total;
                 };
                 auto direct_bad_transitions = [&]() {
                     std::vector<int> bad;
                     const auto layers = sample_cover_layers();
                     for (std::size_t sample_index = 0; sample_index + 1 < layers.size(); ++sample_index) {
                         bool ok = false;
                         for (int lhs : layers[sample_index]) {
                             for (int rhs : layers[sample_index + 1]) {
                                 if (interval_boxes_connected_pybind(
                                         boxes[static_cast<std::size_t>(lhs)],
                                         boxes[static_cast<std::size_t>(rhs)],
                                         coverage_tolerance)) {
                                     ok = true;
                                     break;
                                 }
                             }
                             if (ok) {
                                 break;
                             }
                         }
                         if (!ok) {
                             bad.push_back(static_cast<int>(sample_index));
                         }
                     }
                     return bad;
                 };
                 if (repair_corridor_adjacency && samples.size() >= 2) {
                     const auto repair_start = Clock::now();
                     const int max_rounds = std::max(0, repair_rounds);
                     const int subdivisions = std::max(1, repair_segment_subdivisions);
                     auto layers = sample_cover_layers();
                     struct RepairDsu {
                         std::vector<int> parent;
                         explicit RepairDsu(std::size_t count = 0) : parent(count) {
                             for (std::size_t index = 0; index < parent.size(); ++index) {
                                 parent[index] = static_cast<int>(index);
                             }
                         }
                         int add_node() {
                             const int id = static_cast<int>(parent.size());
                             parent.push_back(id);
                             return id;
                         }
                         int find(int value) {
                             int root = value;
                             while (parent[static_cast<std::size_t>(root)] != root) {
                                 root = parent[static_cast<std::size_t>(root)];
                             }
                             while (parent[static_cast<std::size_t>(value)] != value) {
                                 const int next = parent[static_cast<std::size_t>(value)];
                                 parent[static_cast<std::size_t>(value)] = root;
                                 value = next;
                             }
                             return root;
                         }
                         void unite(int lhs, int rhs) {
                             if (lhs < 0 || rhs < 0 ||
                                 lhs >= static_cast<int>(parent.size()) ||
                                 rhs >= static_cast<int>(parent.size())) {
                                 return;
                             }
                             const int left = find(lhs);
                             const int right = find(rhs);
                             if (left != right) {
                                 parent[static_cast<std::size_t>(right)] = left;
                             }
                         }
                     };
                     RepairDsu repair_dsu(boxes.size());
                     auto transition_connected_dsu = [&](int transition) {
                         if (transition < 0 ||
                             transition + 1 >= static_cast<int>(layers.size())) {
                             return false;
                         }
                         const auto& left_layer = layers[static_cast<std::size_t>(transition)];
                         const auto& right_layer = layers[static_cast<std::size_t>(transition + 1)];
                         if (left_layer.empty() || right_layer.empty()) {
                             return false;
                         }
                         for (int lhs : left_layer) {
                             const int root = repair_dsu.find(lhs);
                             for (int rhs : right_layer) {
                                 if (root == repair_dsu.find(rhs)) {
                                     return true;
                                 }
                             }
                         }
                         return false;
                     };
                     auto bad_transitions_dsu = [&]() {
                         std::vector<int> bad;
                         for (std::size_t sample_index = 0; sample_index + 1 < layers.size(); ++sample_index) {
                             if (!transition_connected_dsu(static_cast<int>(sample_index))) {
                                 bad.push_back(static_cast<int>(sample_index));
                             }
                         }
                         return bad;
                     };
                     auto initialize_corridor_dsu = [&]() {
                         for (auto& layer : layers) {
                             if (layer.empty()) {
                                 continue;
                             }
                             const int root_box = layer.front();
                             for (int box_index : layer) {
                                 repair_dsu.unite(root_box, box_index);
                             }
                         }
                         for (std::size_t sample_index = 0; sample_index + 1 < layers.size(); ++sample_index) {
                             for (int lhs : layers[sample_index]) {
                                 for (int rhs : layers[sample_index + 1]) {
                                     if (interval_boxes_connected_pybind(
                                             boxes[static_cast<std::size_t>(lhs)],
                                             boxes[static_cast<std::size_t>(rhs)],
                                             coverage_tolerance)) {
                                         repair_dsu.unite(lhs, rhs);
                                     }
                                 }
                             }
                         }
                     };
                     initialize_corridor_dsu();
                     std::vector<int> repair_box_indices;
                     auto assimilate_repair_box = [&](int new_box_index, int transition) {
                         for (std::size_t sample_index = 0; sample_index < samples.size(); ++sample_index) {
                             if (!intervals_contain_point_pybind(
                                     boxes[static_cast<std::size_t>(new_box_index)],
                                     samples[sample_index],
                                     coverage_tolerance)) {
                                 continue;
                             }
                             auto& layer = layers[sample_index];
                             if (!layer.empty()) {
                                 repair_dsu.unite(new_box_index, layer.front());
                             }
                             layer.push_back(new_box_index);
                         }
                         std::vector<int> candidates;
                         auto add_layer_candidates = [&](int layer_index) {
                             if (layer_index < 0 || layer_index >= static_cast<int>(layers.size())) {
                                 return;
                             }
                             const auto& layer = layers[static_cast<std::size_t>(layer_index)];
                             candidates.insert(candidates.end(), layer.begin(), layer.end());
                         };
                         add_layer_candidates(transition - 1);
                         add_layer_candidates(transition);
                         add_layer_candidates(transition + 1);
                         add_layer_candidates(transition + 2);
                         candidates.insert(candidates.end(), repair_box_indices.begin(), repair_box_indices.end());
                         std::sort(candidates.begin(), candidates.end());
                         candidates.erase(std::unique(candidates.begin(), candidates.end()), candidates.end());
                         for (int candidate : candidates) {
                             if (candidate == new_box_index) {
                                 continue;
                             }
                             if (interval_boxes_connected_pybind(
                                     boxes[static_cast<std::size_t>(new_box_index)],
                                     boxes[static_cast<std::size_t>(candidate)],
                                     coverage_tolerance)) {
                                 repair_dsu.unite(new_box_index, candidate);
                             }
                         }
                     };
                     std::vector<double> repair_fractions;
                     repair_fractions.reserve(static_cast<std::size_t>(std::max(0, subdivisions - 1)));
                     for (int item = 1; item < subdivisions; ++item) {
                         repair_fractions.push_back(
                             static_cast<double>(item) / static_cast<double>(subdivisions));
                     }
                     std::stable_sort(repair_fractions.begin(),
                                      repair_fractions.end(),
                                      [](double lhs, double rhs) {
                                          return std::abs(lhs - 0.5) < std::abs(rhs - 0.5);
                                      });
                     for (int round = 0; round < max_rounds; ++round) {
                         const auto bad = bad_transitions_dsu();
                         if (round == 0) {
                             repair_bad_transitions_initial = static_cast<int>(bad.size());
                             repair_bad_transition_length_initial = transition_length_sum(bad);
                         }
                         if (bad.empty()) {
                             repair_bad_transitions_final = 0;
                             repair_bad_transition_length_final = 0.0;
                             break;
                         }
                         repair_bad_transitions_final = static_cast<int>(bad.size());
                         repair_bad_transition_length_final = transition_length_sum(bad);
                         int round_added = 0;
                         for (int transition : bad) {
                             if (transition < 0 ||
                                 transition + 1 >= static_cast<int>(samples.size())) {
                                 continue;
                             }
                             const Eigen::VectorXd& a = samples[static_cast<std::size_t>(transition)];
                             const Eigen::VectorXd& b = samples[static_cast<std::size_t>(transition + 1)];
                             if (transition_connected_dsu(transition)) {
                                 continue;
                             }
                             for (double u : repair_fractions) {
                                 const Eigen::VectorXd seed = (1.0 - u) * a + u * b;
                                 const rbf::FindFreeBoxResult result = ffb.find(seed, context, options);
                                 repair_calls += 1;
                                 if (!result.found ||
                                     !intervals_contain_point_pybind(result.intervals,
                                                                     seed,
                                                                     coverage_tolerance)) {
                                     continue;
                                 }
                                 if (duplicate_box(result.intervals)) {
                                     continue;
                                 }
                                 const int new_box_index = static_cast<int>(boxes.size());
                                 boxes.push_back(result.intervals);
                                 repair_dsu.add_node();
                                 assimilate_repair_box(new_box_index, transition);
                                 repair_box_indices.push_back(new_box_index);
                                 repair_added += 1;
                                 round_added += 1;
                                 mark_covered(static_cast<std::size_t>(new_box_index));
                                 if (transition_connected_dsu(transition)) {
                                     break;
                                 }
                             }
                         }
                         mark_covered(0);
                         if (round_added == 0) {
                             break;
                         }
                     }
                     const auto final_bad = bad_transitions_dsu();
                     repair_bad_transitions_final = static_cast<int>(final_bad.size());
                     repair_bad_transition_length_final = transition_length_sum(final_bad);
                     repair_ms =
                         std::chrono::duration<double, std::milli>(Clock::now() - repair_start).count();
                 } else if (samples.size() >= 2) {
                     const auto bad = direct_bad_transitions();
                     repair_bad_transitions_initial = static_cast<int>(bad.size());
                     repair_bad_transitions_final = static_cast<int>(bad.size());
                     repair_bad_transition_length_initial = transition_length_sum(bad);
                     repair_bad_transition_length_final = repair_bad_transition_length_initial;
                 }

                 int covered_count = 0;
                 py::list uncovered_indices;
                 for (std::size_t sample_index = 0; sample_index < samples.size(); ++sample_index) {
                     if (covered[sample_index]) {
                         covered_count += 1;
                     } else if (uncovered_indices.size() < static_cast<py::ssize_t>(std::max(0, max_failure_records))) {
                         uncovered_indices.append(static_cast<int>(sample_index));
                     }
                 }
                 py::list out_boxes;
                 for (const auto& intervals : boxes) {
                     out_boxes.append(interval_pairs_to_python(intervals));
                 }
                 py::list fail_counts_py;
                 for (int count : fail_counts) {
                     fail_counts_py.append(count);
                 }
                 py::dict result;
                 result["sample_count"] = static_cast<int>(samples.size());
                 result["covered_samples"] = covered_count;
                 result["uncovered_samples"] = static_cast<int>(samples.size()) - covered_count;
                 result["coverage"] = samples.empty()
                     ? 1.0
                     : static_cast<double>(covered_count) / static_cast<double>(samples.size());
                 result["initial_box_count"] = initial_box_count;
                 result["added_box_count"] = static_cast<int>(boxes.size()) - initial_box_count;
                 result["box_count"] = static_cast<int>(boxes.size());
                 result["ffb_calls"] = ffb_calls;
                 result["ffb_found"] = found;
                 result["pass_summaries"] = pass_summaries;
                 result["final_sample_step"] = final_sample_step;
                 result["presplit_ms"] = presplit_ms;
                 result["parallel_workers"] = parallel_workers;
                 result["repair_corridor_adjacency"] = repair_corridor_adjacency;
                 result["repair_ms"] = repair_ms;
                 result["repair_calls"] = repair_calls;
                 result["repair_added"] = repair_added;
                 result["repair_bad_transitions_initial"] = repair_bad_transitions_initial;
                 result["repair_bad_transitions_final"] = repair_bad_transitions_final;
                 result["repair_bad_transition_length_initial"] = repair_bad_transition_length_initial;
                 result["repair_bad_transition_length_final"] = repair_bad_transition_length_final;
                 result["repair_bad_transition_fraction_final"] =
                     path_length > 1e-12 ? repair_bad_transition_length_final / path_length : 0.0;
                 result["rejected_not_containing_seed"] = rejected_not_containing_seed;
                 result["fail_counts"] = fail_counts_py;
                 result["failures"] = failures;
                 result["uncovered_indices"] = uncovered_indices;
                 result["boxes"] = out_boxes;
                 result["external_source_enabled"] = external_source != nullptr;
                 result["external_direct_database_enabled"] = direct_external_database != nullptr;
                 result["counters"] = oracle_counters_to_python(
                     use_counter_override ? result_counters_override : oracle.counters());
                 result["total_ms"] =
                     std::chrono::duration<double, std::milli>(Clock::now() - start).count();
                 return result;
             },
             py::arg("waypoint_path"),
             py::arg("obstacles"),
             py::arg("options") = rbf::FindFreeBoxOptions{},
             py::arg("sample_step") = 0.01,
             py::arg("max_ffb_calls") = -1,
             py::arg("coverage_tolerance") = 1e-9,
             py::arg("include_existing_boxes") = true,
             py::arg("disable_caches") = false,
             py::arg("max_failure_records") = 32,
             py::arg("refine_sample_steps") = std::vector<double>{},
             py::arg("parallel_workers") = 1,
             py::arg("repair_corridor_adjacency") = false,
             py::arg("repair_rounds") = 2,
             py::arg("repair_segment_subdivisions") = 8)
;
}

}  // namespace rbf::python_binding
