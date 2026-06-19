#pragma once

#include <SBF/sbf.h>

#include "binding_utils.h"

#include <rbf/lect_database/read_snapshot.h>
#include <sbf/envelope/ifk_aa_source.h>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <filesystem>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace py = pybind11;

namespace rbf::python_binding {

inline void register_basic_types(py::module_& module) {
    py::class_<rbf::Obstacle>(module, "Obstacle")
        .def(py::init<float, float, float, float, float, float>())
        .def_property("bounds",
            [](const rbf::Obstacle& obstacle) {
                return std::vector<float>(obstacle.bounds, obstacle.bounds + 6);
            },
            [](rbf::Obstacle& obstacle, const std::vector<float>& bounds) {
                if (bounds.size() != 6) {
                    throw std::invalid_argument("Obstacle.bounds must contain 6 values");
                }
                std::copy(bounds.begin(), bounds.end(), obstacle.bounds);
            });

    py::class_<rbf::Interval>(module, "Interval")
        .def(py::init<>())
        .def(py::init<double, double>())
        .def_readwrite("lo", &rbf::Interval::lo)
        .def_readwrite("hi", &rbf::Interval::hi)
        .def("width", &rbf::Interval::width)
        .def("center", &rbf::Interval::center);

    py::class_<rbf::DHParam>(module, "DHParam")
        .def(py::init<>())
        .def_readwrite("alpha", &rbf::DHParam::alpha)
        .def_readwrite("a", &rbf::DHParam::a)
        .def_readwrite("d", &rbf::DHParam::d)
        .def_readwrite("theta", &rbf::DHParam::theta)
        .def_readwrite("joint_type", &rbf::DHParam::joint_type);

    py::class_<rbf::JointLimits>(module, "JointLimits")
        .def(py::init<>())
        .def_readwrite("limits", &rbf::JointLimits::limits)
        .def("n_dims", &rbf::JointLimits::n_dims);

    py::class_<rbf::Robot>(module, "Robot")
        .def(py::init([](std::string name,
                         std::vector<rbf::DHParam> dh_params,
                         rbf::JointLimits limits,
                         std::optional<rbf::DHParam> tool_frame,
                         std::vector<double> link_radii) {
                 return rbf::Robot(std::move(name),
                                   std::move(dh_params),
                                   std::move(limits),
                                   std::move(tool_frame),
                                   std::move(link_radii));
             }),
             py::arg("name"),
             py::arg("dh_params"),
             py::arg("limits"),
             py::arg("tool_frame") = std::nullopt,
             py::arg("link_radii") = std::vector<double>{})
        .def_static("from_json", &rbf::Robot::from_json, py::arg("path"))
        .def("name", &rbf::Robot::name)
        .def("n_joints", &rbf::Robot::n_joints)
        .def("n_active_links", &rbf::Robot::n_active_links)
        .def("has_tool", &rbf::Robot::has_tool)
        .def("fingerprint", &rbf::Robot::fingerprint)
        .def("joint_limits", &rbf::Robot::joint_limits,
             py::return_value_policy::reference_internal)
        .def("link_radii", &rbf::Robot::link_radii,
             py::return_value_policy::reference_internal)
        .def("active_link_map", &active_link_map_vec)
        .def("active_link_radii", &active_link_radii_vec);

    module.def("aafk_volume_min_depth_schedule",
        [](const rbf::Robot& robot, int max_depth, int sample_nodes_per_depth) {
            return rbf::aafk_volume_min_depth_schedule(
                robot,
                robot.joint_limits().limits,
                max_depth,
                sample_nodes_per_depth);
        },
        py::arg("robot"),
        py::arg("max_depth"),
        py::arg("sample_nodes_per_depth") = 8);

    module.def("aafk_volume_min_depth_schedule",
        [](const rbf::Robot& robot,
           const std::vector<rbf::Interval>& root_intervals,
           int max_depth,
           int sample_nodes_per_depth) {
            return rbf::aafk_volume_min_depth_schedule(
                robot,
                root_intervals,
                max_depth,
                sample_nodes_per_depth);
        },
        py::arg("robot"),
        py::arg("root_intervals"),
        py::arg("max_depth"),
        py::arg("sample_nodes_per_depth") = 8);

    module.def("support_hull_volume_min_depth_schedule",
        [](const rbf::Robot& robot, int max_depth, int sample_nodes_per_depth) {
            return rbf::support_hull_volume_min_depth_schedule(
                robot,
                robot.joint_limits().limits,
                max_depth,
                sample_nodes_per_depth);
        },
        py::arg("robot"),
        py::arg("max_depth"),
        py::arg("sample_nodes_per_depth") = 8);

    module.def("support_hull_volume_min_depth_schedule",
        [](const rbf::Robot& robot,
           const std::vector<rbf::Interval>& root_intervals,
           int max_depth,
           int sample_nodes_per_depth) {
            return rbf::support_hull_volume_min_depth_schedule(
                robot,
                root_intervals,
                max_depth,
                sample_nodes_per_depth);
        },
        py::arg("robot"),
        py::arg("root_intervals"),
        py::arg("max_depth"),
        py::arg("sample_nodes_per_depth") = 8);

    module.def("canonical_root_intervals_for_robot",
        [](const rbf::Robot& robot,
           bool canonical_mode,
           const std::string& symmetry_descriptor) {
            return rbf::lect_database::canonical_root_intervals_for_robot(
                robot,
                canonical_mode,
                symmetry_descriptor);
        },
        py::arg("robot"),
        py::arg("canonical_mode") = true,
        py::arg("symmetry_descriptor") = "joint_symmetry_native_v1");

    module.def("canonicalize_configuration_for_robot",
        [](const rbf::Robot& robot,
           const std::vector<double>& values,
           bool canonical_mode,
           const std::string& symmetry_descriptor) {
            std::vector<double> out = values;
            rbf::lect_database::canonicalize_configuration_for_robot(
                robot,
                canonical_mode,
                symmetry_descriptor,
                std::span<double>(out.data(), out.size()));
            return out;
        },
        py::arg("robot"),
        py::arg("values"),
        py::arg("canonical_mode") = true,
        py::arg("symmetry_descriptor") = "joint_symmetry_native_v1");

    module.def("build_lect_snapshot_from_legacy",
        [](const std::string& legacy_root, const std::string& snapshot_path) {
            std::string reason;
            const std::filesystem::path legacy_path(legacy_root);
            const std::filesystem::path snapshot_path_fs = snapshot_path.empty()
                ? rbf::lect_database::LectReadSnapshot::default_snapshot_path(legacy_path)
                : std::filesystem::path(snapshot_path);
            if (!rbf::lect_database::LectReadSnapshot::build_from_legacy(
                    legacy_path,
                    snapshot_path_fs,
                    &reason)) {
                throw std::runtime_error(
                    reason.empty() ? "failed to build LECT snapshot from legacy cache" : reason);
            }
            return snapshot_path_fs.string();
        },
        py::arg("legacy_root"),
        py::arg("snapshot_path") = "");

    py::enum_<rbf::EndpointSource>(module, "EndpointSource")
        .value("IFK", rbf::EndpointSource::IFK)
        .value("CritSample", rbf::EndpointSource::CritSample)
        .value("Analytical", rbf::EndpointSource::Analytical)
        .value("GCPC", rbf::EndpointSource::GCPC)
        .value("MC", rbf::EndpointSource::MC)
        .value("HIFK", rbf::EndpointSource::HIFK);

    py::enum_<rbf::EndpointSafetyLevel>(module, "EndpointSafetyLevel")
        .value("Certified", rbf::EndpointSafetyLevel::Certified)
        .value("Provisional", rbf::EndpointSafetyLevel::Provisional)
        .value("UnsafeHeuristic", rbf::EndpointSafetyLevel::UnsafeHeuristic)
        .value("SafeCertified", rbf::EndpointSafetyLevel::Certified);

    py::enum_<rbf::EnvelopeType>(module, "EnvelopeType")
        .value("LinkIAABB", rbf::EnvelopeType::LinkIAABB)
        .value("KDOP", rbf::EnvelopeType::KDOP)
        .value("SupportHull", rbf::EnvelopeType::SupportHull);

    py::enum_<rbf::KdopDirectionSet>(module, "KdopDirectionSet")
        .value("DOP6", rbf::KdopDirectionSet::DOP6)
        .value("DOP18", rbf::KdopDirectionSet::DOP18)
        .value("DOP26", rbf::KdopDirectionSet::DOP26);

    py::enum_<rbf::OracleValidationMode>(module, "OracleValidationMode")
        .value("Strict", rbf::OracleValidationMode::Strict)
        .value("StrictCertificate", rbf::OracleValidationMode::Strict)
        .value("CoverageHeuristic", rbf::OracleValidationMode::CoverageHeuristic);

    py::enum_<rbf::BoxSafetyStatus>(module, "BoxSafetyStatus")
        .value("Unknown", rbf::BoxSafetyStatus::Unknown)
        .value("CertifiedFree", rbf::BoxSafetyStatus::CertifiedFree)
        .value("ProvisionalFree", rbf::BoxSafetyStatus::ProvisionalFree)
        .value("Occupied", rbf::BoxSafetyStatus::Occupied);

    py::enum_<rbf::BoxCommitPolicy>(module, "BoxCommitPolicy")
        .value("CommitCertifiedOnly", rbf::BoxCommitPolicy::CommitCertifiedOnly)
        .value("CommitProvisionalAllowed", rbf::BoxCommitPolicy::CommitProvisionalAllowed)
        .value("AuditBeforeCommit", rbf::BoxCommitPolicy::AuditBeforeCommit);

    py::enum_<rbf::SegmentEdgeType>(module, "SegmentEdgeType")
        .value("Unknown", rbf::SegmentEdgeType::Unknown)
        .value("RRTConnector", rbf::SegmentEdgeType::RRTConnector)
        .value("PointValidatedGap", rbf::SegmentEdgeType::PointValidatedGap)
        .value("QueryBridge", rbf::SegmentEdgeType::QueryBridge)
        .value("BoxCorridor", rbf::SegmentEdgeType::BoxCorridor)
        .value("PortalCorridor", rbf::SegmentEdgeType::PortalCorridor)
        .value("SegmentOBBCorridor", rbf::SegmentEdgeType::SegmentOBBCorridor)
        .value("RRTBridgeOBBCorridor", rbf::SegmentEdgeType::RRTBridgeOBBCorridor)
        .value("TransitionOBBCorridor", rbf::SegmentEdgeType::TransitionOBBCorridor);

    py::enum_<rbf::SegmentEdgeValidation>(module, "SegmentEdgeValidation")
        .value("Unknown", rbf::SegmentEdgeValidation::Unknown)
        .value("CollisionChecked", rbf::SegmentEdgeValidation::CollisionChecked)
        .value("ConservativeBoxChain", rbf::SegmentEdgeValidation::ConservativeBoxChain)
        .value("ConservativeObbZonotope", rbf::SegmentEdgeValidation::ConservativeObbZonotope);

    py::enum_<rbf::PathAuditStatus>(module, "PathAuditStatus")
        .value("NotRun", rbf::PathAuditStatus::NotRun)
        .value("Unchecked", rbf::PathAuditStatus::NotRun)
        .value("Passed", rbf::PathAuditStatus::Passed)
        .value("Failed", rbf::PathAuditStatus::Failed)
        .value("Repaired", rbf::PathAuditStatus::Repaired);

    py::enum_<rbf::PortalMembershipPolicy>(module, "PortalMembershipPolicy")
        .value("GlobalForestOnly", rbf::PortalMembershipPolicy::GlobalForestOnly)
        .value("PortalInteriorIndex", rbf::PortalMembershipPolicy::PortalInteriorIndex);
}

} // namespace rbf::python_binding
