#pragma once

#include <SBF/sbf.h>

#include "binding_utils.h"

#include <rbf/lect_database/read_snapshot.h>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace py = pybind11;

namespace rbf::python_binding {

inline void register_planning_forest_database_methods(
    py::class_<rbf::RBFPlanningForest>& forest_class) {
    forest_class
        .def("database_node_count", [](const rbf::RBFPlanningForest& forest) {
            return forest.database().node_count();
        })
        .def("database_evidence_count", [](const rbf::RBFPlanningForest& forest) {
            return forest.database().evidence_count();
        })
        .def("database_root_intervals", [](const rbf::RBFPlanningForest& forest) {
            return interval_pairs_to_python(forest.database().root_intervals());
        })
        .def("database_coverage_intervals", [](const rbf::RBFPlanningForest& forest) {
            return interval_pairs_to_python(forest.database().coverage_intervals());
        })
        .def("database_checkpoint", [](rbf::RBFPlanningForest& forest) {
            return forest.database().checkpoint();
        })
        .def("database_verify", [](const rbf::RBFPlanningForest& forest, bool strict) {
            return forest.database().verify(strict).ok;
        }, py::arg("strict") = true)
        .def("database_snapshot_path", [](const rbf::RBFPlanningForest& forest) {
            return rbf::lect_database::LectReadSnapshot::default_snapshot_path(forest.config().database.path).string();
        })
        .def("database_wait_for_snapshot_publish", [](const rbf::RBFPlanningForest& forest) {
            const auto snapshot_path = rbf::lect_database::LectReadSnapshot::default_snapshot_path(forest.config().database.path);
            return rbf::lect_database::LectReadSnapshot::build_from_legacy(forest.config().database.path, snapshot_path);
        })
        .def("prewarm_lifelong_cache",
             [](rbf::RBFPlanningForest& forest,
                int target_depth,
                const std::vector<rbf::Obstacle>& obstacles,
                bool gray_leaf_order,
                bool show_progress,
                bool streaming,
                std::size_t streaming_cap,
                double checkpoint_interval_s) {
                 if (obstacles.empty()) {
                     throw std::invalid_argument("prewarm_lifelong_cache requires a non-empty obstacle scene so endpoint evidence is materialized");
                 }
                 const auto start = std::chrono::steady_clock::now();
                 const int materialize_depth = std::max(0, target_depth);
                 // Prewarm persistence mode:
                 //   default                    -> bulk: all records resident,
                 //       fastest, RAM ~ O(records). Good up to ~D20.
                 //   streaming=true             -> resident cache is capped
                 //       so peak RAM stays bounded for deep trees (e.g. D25 ~62M
                 //       records). Records are appended to the durable store as
                 //       built; evicted child records are reloaded on demand by the
                 //       bottom-up parent sweep. Output is bit-identical to bulk.
                 const bool streaming_prewarm = streaming;
                 streaming_cap = std::max<std::size_t>(std::size_t{1}, streaming_cap);
                 checkpoint_interval_s = std::max(0.0, checkpoint_interval_s);
                 std::size_t periodic_checkpoint_attempts = 0;
                 std::size_t periodic_checkpoint_failures = 0;
                 auto last_checkpoint_time = start;
                 auto run_periodic_checkpoint = [&](const char* phase,
                                                    std::size_t done,
                                                    std::size_t total,
                                                    bool force = false) {
                     if (checkpoint_interval_s <= 0.0 || periodic_checkpoint_failures > 0) {
                         return;
                     }
                     const auto now = std::chrono::steady_clock::now();
                     const double since_last = std::chrono::duration<double>(now - last_checkpoint_time).count();
                     if (!force && since_last < checkpoint_interval_s) {
                         return;
                     }
                     const auto checkpoint_start = std::chrono::steady_clock::now();
                     const bool ok = forest.database().checkpoint();
                     const auto checkpoint_end = std::chrono::steady_clock::now();
                     ++periodic_checkpoint_attempts;
                     if (!ok) {
                         ++periodic_checkpoint_failures;
                     }
                     last_checkpoint_time = checkpoint_end;
                     if (show_progress) {
                         const double checkpoint_s = std::chrono::duration<double>(checkpoint_end - checkpoint_start).count();
                         std::fprintf(stderr,
                                      "\n[prewarm checkpoint] phase=%s done=%zu/%zu ok=%d elapsed %.1fs\n",
                                      phase,
                                      done,
                                      total,
                                      ok ? 1 : 0,
                                      checkpoint_s);
                         std::fflush(stderr);
                     }
                 };
                 const auto expected_leaf_records_for_depth = [](int depth) -> std::size_t {
                     if (depth < 0 || depth >= static_cast<int>(std::numeric_limits<std::size_t>::digits)) {
                         return 0;
                     }
                     return std::size_t{1} << depth;
                 };
                 const std::size_t expected_leaf_records = expected_leaf_records_for_depth(materialize_depth);
                 const std::size_t expected_prewarm_records =
                     expected_leaf_records > 0 &&
                             expected_leaf_records <= (std::numeric_limits<std::size_t>::max() - 64) / 2
                         ? expected_leaf_records * 2 + 64
                         : 0;
                 if (streaming_prewarm) {
                     forest.database().set_streaming_prewarm_mode(true, streaming_cap);
                 } else {
                     forest.database().set_bulk_prewarm_mode(true, expected_prewarm_records);
                 }
                 if (show_progress) {
                     std::fprintf(stderr, "[prewarm setup] ensure_depth 0/%d\n", materialize_depth);
                     std::fflush(stderr);
                 }
                 const auto ensure_start = std::chrono::steady_clock::now();
                 bool depth_ok = true;
                 if (show_progress) {
                     const std::size_t setup_total = expected_leaf_records > 0
                         ? expected_leaf_records - 1
                         : std::size_t{0};
                     std::size_t setup_done = 0;
                     for (int depth = 0; depth < materialize_depth && depth_ok; ++depth) {
                         const auto layer = forest.database().layer_nodes(depth);
                         const std::size_t layer_total = layer.size();
                         const std::size_t layer_stride =
                             std::max<std::size_t>(std::size_t{1}, layer_total / 200);
                         std::size_t layer_done = 0;
                         for (rbf::lect_database::NodeId node_id : layer) {
                             const auto children = forest.database().split_leaf(node_id);
                             if (children.first == rbf::lect_database::kInvalidNodeId ||
                                 children.second == rbf::lect_database::kInvalidNodeId) {
                                 depth_ok = false;
                                 break;
                             }
                             ++layer_done;
                             ++setup_done;
                             if (layer_done % layer_stride == 0 || layer_done == layer_total) {
                                 const double el = std::chrono::duration<double>(
                                                       std::chrono::steady_clock::now() - ensure_start)
                                                       .count();
                                 const double frac = static_cast<double>(setup_done) /
                                                     static_cast<double>(std::max<std::size_t>(setup_total, 1));
                                 const double eta = el * (1.0 - frac) / std::max(frac, 1e-9);
                                 std::fprintf(stderr,
                                              "\r[prewarm setup] depth %2d/%2d  layer %zu/%zu  %5.1f%%  elapsed %6.1fs  ETA %6.1fs   ",
                                              depth + 1,
                                              materialize_depth,
                                              layer_done,
                                              layer_total,
                                              100.0 * std::min(frac, 1.0),
                                              el,
                                              eta);
                                 std::fflush(stderr);
                             }
                             run_periodic_checkpoint("ensure_depth", setup_done, setup_total);
                         }
                     }
                     std::fprintf(stderr, "\n");
                     std::fflush(stderr);
                 } else {
                     depth_ok = forest.database().ensure_depth(materialize_depth);
                 }
                 if (show_progress) {
                     const double el = std::chrono::duration<double>(
                                           std::chrono::steady_clock::now() - ensure_start)
                                           .count();
                     std::fprintf(stderr, "[prewarm setup] ensure_depth done %.1fs\n", el);
                     std::fflush(stderr);
                 }
                 if (depth_ok) {
                     run_periodic_checkpoint("ensure_depth", expected_leaf_records, expected_leaf_records, true);
                 }
                 rbf::OracleValidationConfig prewarm_validation = forest.config().validation;
                 prewarm_validation.stateless_materialization_context = true;
                 rbf::DatabaseBoxOracle oracle(forest.robot(),
                                               forest.database(),
                                               rbf::Scene(obstacles),
                                               forest.config().endpoint_source,
                                               forest.config().envelope_type,
                                               prewarm_validation);
                 std::size_t nodes_touched = 0;
                 const std::size_t evidence_before = forest.database().evidence_count();
                 // Disable per-leaf auto-propagation; the bottom-up sweep below
                 // does all ancestor unions in a single O(leaves) pass instead of
                 // scattered O(leaves*depth) walk-ups during leaf inserts.
                 const bool prev_propagate = forest.database().propagate_parent_hulls_enabled();
                 forest.database().set_propagate_parent_hulls(false);
                 // Visit the leaf layer in reflected Gray-code (boustrophedon)
                 // order so that consecutive leaves differ in exactly one split
                 // decision -- i.e. exactly one joint interval changes between
                 // successive FK materializations. The IFK stateful endpoint
                 // source then reuses its AA-FK chain prefix and recomputes only
                 // the changed suffix; the incremental result is provably
                 // identical to a full pass, so the stored payloads are
                 // bit-for-bit unchanged -- this is a pure prewarm speedup.
                 const auto leaf_layer = forest.database().layer_nodes(materialize_depth);
                 if (!streaming_prewarm && leaf_layer.size() > expected_leaf_records) {
                     forest.database().set_bulk_prewarm_mode(true, leaf_layer.size() * 2 + 64);
                 }
                 std::vector<rbf::lect_database::NodeId> ordered_leaves;
                 ordered_leaves.reserve(leaf_layer.size());
                 {
                     const auto roots = forest.database().layer_nodes(0);
                     if (!roots.empty()) {
                         struct Frame {
                             rbf::lect_database::NodeId node;
                             int depth;
                             bool reversed;
                         };
                         std::vector<Frame> stack;
                         stack.push_back({roots.front(), 0, false});
                         while (!stack.empty()) {
                             const Frame fr = stack.back();
                             stack.pop_back();
                             if (fr.depth >= materialize_depth) {
                                 ordered_leaves.push_back(fr.node);
                                 continue;
                             }
                             const auto topo = forest.database().topology(fr.node);
                             if (topo.left == rbf::lect_database::kInvalidNodeId ||
                                 topo.right == rbf::lect_database::kInvalidNodeId) {
                                 ordered_leaves.push_back(fr.node);
                                 continue;
                             }
                             // forward: visit left (forward) then right (reversed);
                             // reversed: visit right (forward) then left (reversed).
                             const rbf::lect_database::NodeId first =
                                 fr.reversed ? topo.right : topo.left;
                             const rbf::lect_database::NodeId second =
                                 fr.reversed ? topo.left : topo.right;
                             // LIFO: push the second-visited child first.
                             stack.push_back({second, fr.depth + 1, true});
                             stack.push_back({first, fr.depth + 1, false});
                         }
                     }
                 }
                 const bool use_gray_order =
                     !ordered_leaves.empty() && ordered_leaves.size() == leaf_layer.size();
                 const std::vector<rbf::lect_database::NodeId>& leaf_iteration =
                     (gray_leaf_order && use_gray_order) ? ordered_leaves : leaf_layer;
                 const auto leaf_loop_start = std::chrono::steady_clock::now();
                 const std::size_t leaf_total = leaf_iteration.size();
                 const std::size_t leaf_stride =
                     std::max<std::size_t>(std::size_t{1}, leaf_total / 200);
                 std::size_t leaf_done = 0;
                 for (rbf::lect_database::NodeId node_id : leaf_iteration) {
                     auto intervals = forest.database().node_box(node_id);
                     if (!intervals) {
                         continue;
                     }
                     oracle.validate_node(static_cast<int>(node_id), *intervals, -1);
                     nodes_touched += 1;
                     ++leaf_done;
                     if (show_progress &&
                         (leaf_done % leaf_stride == 0 || leaf_done == leaf_total)) {
                         const double el = std::chrono::duration<double>(
                                               std::chrono::steady_clock::now() - leaf_loop_start)
                                               .count();
                         const double rate = static_cast<double>(leaf_done) / std::max(el, 1e-9);
                         const double eta =
                             static_cast<double>(leaf_total - leaf_done) / std::max(rate, 1e-9);
                         std::fprintf(stderr,
                                      "\r[prewarm leaves]  %5.1f%%  %zu/%zu  %.0f/s  elapsed %6.1fs  ETA %6.1fs   ",
                                      100.0 * static_cast<double>(leaf_done) /
                                          static_cast<double>(std::max<std::size_t>(leaf_total, 1)),
                                      leaf_done, leaf_total, rate, el, eta);
                         std::fflush(stderr);
                     }
                     run_periodic_checkpoint("leaves", leaf_done, leaf_total);
                 }
                 if (show_progress) {
                     std::fprintf(stderr, "\n");
                     std::fflush(stderr);
                 }
                 const auto leaf_loop_end = std::chrono::steady_clock::now();
                 // HARDCODED: LECT prewarm only FK-materializes the leaf layer,
                 // then derives every internal-node envelope bottom-up as the
                 // cheap conservative union of its two children (tighter than a
                 // direct parent FK). Internal nodes are never FK-recomputed.
                 const std::size_t parent_total_estimate =
                     forest.database().node_count() > leaf_total
                         ? forest.database().node_count() - leaf_total
                         : std::size_t{0};
                 const std::size_t parent_hulls_built =
                     forest.database().materialize_internal_parent_hulls_bottom_up(
                         materialize_depth, oracle.endpoint_evidence_key(0),
                         show_progress
                             ? std::function<void(int, std::size_t)>(
                                   [&](int depth, std::size_t built) {
                                       const double el =
                                           std::chrono::duration<double>(
                                               std::chrono::steady_clock::now() - leaf_loop_end)
                                               .count();
                                       const int done_layers = materialize_depth - depth;
                                       const double frac =
                                           static_cast<double>(done_layers) /
                                           static_cast<double>(std::max(materialize_depth, 1));
                                       const double eta =
                                           el * (1.0 - frac) / std::max(frac, 1e-9);
                                       std::fprintf(stderr,
                                                    "\r[prewarm parents] depth %2d  %5.1f%%  built %zu  elapsed %6.1fs  ETA %6.1fs   ",
                                                    depth, 100.0 * frac, built, el, eta);
                                       std::fflush(stderr);
                                       run_periodic_checkpoint("parents", built, parent_total_estimate);
                                   })
                             : std::function<void(int, std::size_t)>{});
                 if (show_progress) {
                     std::fprintf(stderr, "\n");
                     std::fflush(stderr);
                 }
                 const auto sweep_end = std::chrono::steady_clock::now();
                 forest.database().set_propagate_parent_hulls(prev_propagate);
                 const bool checkpoint_ok = forest.database().checkpoint();
                 forest.database().set_bulk_prewarm_mode(false);
                 forest.database().set_streaming_prewarm_mode(false, 0);
                 const auto end = std::chrono::steady_clock::now();
                 const auto& counters = oracle.counters();
                 py::dict result;
                 result["ok"] = depth_ok && checkpoint_ok && periodic_checkpoint_failures == 0;
                 result["target_depth"] = materialize_depth;
                 result["depth_ok"] = depth_ok;
                 result["checkpoint_ok"] = checkpoint_ok;
                 result["periodic_checkpoint_seconds"] = checkpoint_interval_s;
                 result["periodic_checkpoint_attempts"] = periodic_checkpoint_attempts;
                 result["periodic_checkpoint_failures"] = periodic_checkpoint_failures;
                 result["nodes_touched"] = nodes_touched;
                 result["parent_hulls_built"] = parent_hulls_built;
                 result["node_count"] = forest.database().node_count();
                 result["evidence_before"] = evidence_before;
                 result["evidence_after"] = forest.database().evidence_count();
                 result["materializations"] = counters.materializations;
                 result["reused_endpoint_cache"] = counters.materialization_reused_endpoint_cache;
                 result["reused_shared_endpoint_cache"] = counters.materialization_reused_shared_endpoint_cache;
                 result["stored_shared_endpoint_cache"] = counters.materialization_stored_shared_endpoint_cache;
                 result["incremental_fk"] = counters.materialization_incremental_fk;
                 result["source_incremental_state"] = counters.materialization_source_incremental_state;
                 result["gray_leaf_order"] = gray_leaf_order && use_gray_order;
                 // --- prewarm time breakdown (seconds) ---
                 result["t_leaf_loop_s"] =
                     std::chrono::duration<double>(leaf_loop_end - leaf_loop_start).count();
                 result["t_parent_sweep_s"] =
                     std::chrono::duration<double>(sweep_end - leaf_loop_end).count();
                 result["t_checkpoint_s"] =
                     std::chrono::duration<double>(end - sweep_end).count();
                 // --- per-leaf validate_node cost decomposition (microseconds) ---
                 result["us_validate_total"] = counters.validate_node_total_time_us;
                 result["us_validate_endpoint_path"] = counters.validate_node_endpoint_path_time_us;
                 result["us_validate_classify"] = counters.validate_node_classify_time_us;
                 result["us_endpoint_fk_wall"] = counters.materialization_endpoint_wall_time_us;
                 result["us_envelope_compute"] = counters.materialization_envelope_compute_time_us;
                 result["us_envelope_collision"] = counters.materialization_envelope_collision_time_us;
                 result["us_cache_lookup"] = counters.materialization_cache_lookup_time_us;
                 result["us_cache_read"] = counters.materialization_cache_read_time_us;
                 result["wall_s"] = std::chrono::duration<double>(end - start).count();
                 return result;
             },
             py::arg("target_depth"),
             py::arg("obstacles"),
             py::arg("gray_leaf_order") = true,
             py::arg("show_progress") = true,
             py::arg("streaming") = false,
             py::arg("streaming_cap") = static_cast<std::size_t>(2000000),
             py::arg("checkpoint_interval_s") = 0.0)
;
}

}  // namespace rbf::python_binding
