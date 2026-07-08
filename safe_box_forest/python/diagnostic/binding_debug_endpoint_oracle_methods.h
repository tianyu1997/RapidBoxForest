#pragma once

#include <SBF/oracle.h>
#include <SBF/safe_box_forest.h>

#include "../binding_utils.h"
#include "../binding_oracle_utils.h"

#include <link_interval_envelope/batch.h>
#include <rbf/lect_database/read_snapshot.h>
#include <sbf/envelope/ifk_aa_source.h>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace py = pybind11;

namespace rbf::python_binding {

inline void register_debug_endpoint_oracle_methods(py::class_<rbf::RBFPlanningForest>& forest_class) {
    forest_class
        .def("debug_external_endpoint_lookup",
             [](const rbf::RBFPlanningForest& forest,
                const std::vector<std::vector<double>>& interval_pairs,
                const std::vector<rbf::Obstacle>& obstacles) {
                 py::dict result;
                 const auto intervals = intervals_from_pairs(interval_pairs);
                 result["intervals"] = interval_pairs_to_python(intervals);
                 const auto external_root = forest.config().database.external_evidence_path;
                 result["external_evidence_path"] = external_root.string();
                 if (external_root.empty()) {
                     result["found"] = false;
                     result["reason"] = "external_evidence_path is empty";
                     return result;
                 }
                 const auto snapshot_path = forest.config().database.external_evidence_snapshot_path.empty()
                     ? rbf::lect_database::LectReadSnapshot::default_snapshot_path(external_root)
                     : forest.config().database.external_evidence_snapshot_path;
                 result["snapshot_path"] = snapshot_path.string();
                 rbf::lect_database::LectReadSnapshot snapshot;
                 std::string open_reason;
                 if (!snapshot.open(snapshot_path, &open_reason)) {
                     result["found"] = false;
                     result["reason"] = open_reason;
                     return result;
                 }
                 rbf::lect_database::EvidenceKey key;
                 key.sector = rbf::lect_database::kPrimarySector;
                 key.channel = rbf::source_channel(forest.config().endpoint_source.source) == 0
                     ? rbf::lect_database::EvidenceChannel::Safe
                     : rbf::lect_database::EvidenceChannel::Rapid;
                 key.endpoint_source = forest.config().endpoint_source.source;
                 key.payload_kind = rbf::lect_database::EvidencePayloadKind::EndpointEnvelope;
                 const auto box_key = snapshot.make_box_key(intervals);
                 const auto lookup = snapshot.box_to_node_exact(box_key);
                 result["box_node_found"] = lookup.found;
                 result["box_node_id"] = lookup.found ? static_cast<unsigned long long>(lookup.node_id) : 0ULL;
                 result["box_node_reason"] = lookup.reason;
                 const auto view = snapshot.endpoint_for_box_exact(box_key, key);
                 if (!view) {
                     result["found"] = false;
                     result["reason"] = "endpoint evidence not found";
                     return result;
                 }
                 result["found"] = true;
                 result["node_id"] = static_cast<unsigned long long>(view->key.node_id);
                 result["sector"] = static_cast<unsigned int>(view->key.sector);
                 result["channel"] = static_cast<int>(view->key.channel);
                 result["endpoint_source"] = static_cast<int>(view->key.endpoint_source);
                 result["payload_kind"] = static_cast<int>(view->key.payload_kind);
                 result["payload_size"] = static_cast<unsigned long long>(view->payload.size());
                 result["child_hull"] = view->child_hull;
                 result["unavailable"] = view->unavailable;
                 result["generation"] = static_cast<unsigned long long>(view->generation);
                 result["checksum"] = static_cast<unsigned long long>(view->checksum);
                 rbf::LinkEnvelope envelope = rbf::compute_link_envelope(view->payload.data(),
                                                                         forest.robot().n_active_links(),
                                                                         forest.robot().active_link_radii(),
                                                                         forest.config().envelope_type);
                 rbf::EnvelopeCollisionOptions options;
                 options.safety_epsilon = std::max(forest.config().envelope_type.kdop_config.safety_epsilon,
                                                   forest.config().envelope_type.support_hull_config.safety_epsilon);
                 options.overlap_tolerance = std::max(forest.config().envelope_type.kdop_config.overlap_tolerance,
                                                       forest.config().envelope_type.support_hull_config.overlap_tolerance);
                 options.skip_aabb_broadphase =
                     forest.config().envelope_type.support_hull_config.skip_aabb_broadphase;
                 options.direct_support_hull_collision =
                     forest.config().envelope_type.support_hull_config.direct_collision;
                 rbf::EnvelopeCollisionStats stats;
                 const auto collision = rbf::collide_envelope_aabbs(envelope,
                                                                     obstacles.empty() ? nullptr : obstacles.data(),
                                                                     static_cast<int>(obstacles.size()),
                                                                     options,
                                                                     &stats);
                 result["external_is_definitely_free"] =
                     collision == rbf::CollisionResultKind::DefinitelyFree;
                 result["external_maybe_pairs"] = stats.maybe_pairs;
                 result["external_overlap_tolerance_rejects"] = stats.overlap_tolerance_rejects;
                 result["external_aabb_tests"] = stats.envelope_aabb_tests;
                 result["external_aabb_rejects"] = stats.envelope_aabb_rejects;
                 result["external_link_aabb_tests"] = stats.link_aabb_tests;
                 result["external_link_aabb_rejects"] = stats.link_aabb_rejects;
                 result["external_gjk_tests"] = stats.gjk_tests;
                 result["external_gjk_rejects"] = stats.gjk_rejects;
                 result["external_overlap_depth_max"] = stats.maybe_pair_overlap_depth_max;
                 result["external_overlap_volume_ratio_max"] = stats.maybe_pair_overlap_volume_ratio_max;
                 return result;
             },
             py::arg("interval_pairs"),
             py::arg("obstacles") = std::vector<rbf::Obstacle>{})
        .def("debug_validate_intervals",
             [](rbf::RBFPlanningForest& forest,
                const std::vector<rbf::Obstacle>& obstacles,
                const std::vector<std::vector<double>>& interval_pairs,
                int changed_dim,
                bool disable_caches) {
                 const auto intervals = intervals_from_pairs(interval_pairs);
                 if (static_cast<int>(intervals.size()) != forest.robot().n_joints()) {
                     throw std::invalid_argument("interval count must match robot.n_joints()");
                 }
                 rbf::OracleValidationConfig validation = disable_caches
                     ? uncached_validation_config(forest.config().validation)
                     : forest.config().validation;
                 rbf::lect_database::OnlineEnvelopeCacheTree cache(forest.database(), {});
                 std::unique_ptr<rbf::lect_database::LectReadSnapshot> external_snapshot;
                 std::unique_ptr<rbf::lect_database::LectSnapshotEvidenceSource> external_snapshot_source;
                 std::unique_ptr<rbf::lect_database::LectDatabase> external_database;
                 std::unique_ptr<rbf::lect_database::LectDatabaseEvidenceSource> external_database_source;
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
                 const auto validation_result = oracle.validate_node(oracle.root_node(), intervals, changed_dim);
                 py::dict result;
                 result["validation"] = static_cast<int>(validation_result);
                 result["intervals"] = interval_pairs_to_python(intervals);
                 result["root_intervals"] = interval_pairs_to_python(oracle.root_intervals());
                 result["changed_dim"] = changed_dim;
                 result["disable_caches"] = disable_caches;
                 result["validation_detail"] = oracle_validation_detail_to_python(oracle.last_validation_detail());
                 result["counters"] = oracle_counters_to_python(oracle.counters());
                 return result;
             },
             py::arg("obstacles"),
             py::arg("interval_pairs"),
             py::arg("changed_dim") = -1,
             py::arg("disable_caches") = true)
        .def("debug_compute_envelope_summary",
             [](rbf::RBFPlanningForest& forest,
                const std::vector<std::vector<double>>& interval_pairs) {
                 const auto intervals = intervals_from_pairs(interval_pairs);
                 if (static_cast<int>(intervals.size()) != forest.robot().n_joints()) {
                     throw std::invalid_argument("interval count must match robot.n_joints()");
                 }
                 const std::vector<std::vector<rbf::Interval>> boxes{intervals};
                 const auto results = link_interval_envelope::compute_envelope_batch(
                     forest.robot(),
                     boxes,
                     forest.config().endpoint_source,
                     forest.config().envelope_type,
                     1);
                 if (results.empty()) {
                     throw std::runtime_error("envelope batch returned no results");
                 }
                 const auto& batch = results.front();
                 const auto& envelope = batch.envelope;
                 auto aabb_volume = [](const std::vector<float>& aabb, std::size_t offset) {
                     if (aabb.size() < offset + 6U) {
                         return 0.0;
                     }
                     const double dx = std::max(0.0, static_cast<double>(aabb[offset + 3U] - aabb[offset + 0U]));
                     const double dy = std::max(0.0, static_cast<double>(aabb[offset + 4U] - aabb[offset + 1U]));
                     const double dz = std::max(0.0, static_cast<double>(aabb[offset + 5U] - aabb[offset + 2U]));
                     return dx * dy * dz;
                 };
                 auto aabb_extents = [](const std::vector<float>& aabb, std::size_t offset) {
                     std::vector<double> out(3, 0.0);
                     if (aabb.size() < offset + 6U) {
                         return out;
                     }
                     for (std::size_t axis = 0; axis < 3U; ++axis) {
                         out[axis] = std::max(0.0, static_cast<double>(aabb[offset + axis + 3U] - aabb[offset + axis]));
                     }
                     return out;
                 };
                 double sum_link_union_volume = 0.0;
                 py::list link_unions;
                 for (int link = 0; link < envelope.n_active_links; ++link) {
                     const std::size_t offset = static_cast<std::size_t>(link) * 6U;
                     const double volume = aabb_volume(envelope.link_union_iaabbs, offset);
                     sum_link_union_volume += volume;
                     py::dict item;
                     item["link"] = link;
                     if (envelope.link_union_iaabbs.size() >= offset + 6U) {
                         item["aabb"] = std::vector<float>(
                             envelope.link_union_iaabbs.begin() + static_cast<std::ptrdiff_t>(offset),
                             envelope.link_union_iaabbs.begin() + static_cast<std::ptrdiff_t>(offset + 6U));
                     } else {
                         item["aabb"] = std::vector<float>{};
                     }
                     item["extents"] = aabb_extents(envelope.link_union_iaabbs, offset);
                     item["volume"] = volume;
                     link_unions.append(std::move(item));
                 }
                 py::dict result;
                 result["intervals"] = interval_pairs_to_python(intervals);
                 result["source"] = static_cast<int>(batch.source);
                 result["is_safe"] = batch.is_safe;
                 result["endpoint_safety_level"] = static_cast<int>(batch.endpoint_safety_level);
                 result["n_active_links"] = envelope.n_active_links;
                 result["n_subdivisions"] = envelope.n_subdivisions;
                 result["envelope_type"] = static_cast<int>(envelope.type);
                 result["endpoint_time_us"] = batch.endpoint_time_us;
                 result["envelope_time_us"] = batch.envelope_time_us;
                 result["envelope_aabb"] = envelope.envelope_aabb;
                 result["envelope_aabb_extents"] = aabb_extents(envelope.envelope_aabb, 0);
                 result["envelope_aabb_volume"] = aabb_volume(envelope.envelope_aabb, 0);
                 result["sum_link_union_volume"] = sum_link_union_volume;
                 result["link_unions"] = std::move(link_unions);
                 result["link_iaabb_count"] = static_cast<int>(envelope.link_iaabbs.size() / 6U);
                 result["support_hull_records"] = envelope.support_hulls.empty()
                     ? 0
                     : static_cast<int>(envelope.support_hulls.size());
                 result["kdop_values"] = static_cast<int>(envelope.kdop_intervals.size());
                 return result;
             },
             py::arg("interval_pairs"));
}

}  // namespace rbf::python_binding
