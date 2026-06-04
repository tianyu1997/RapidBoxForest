#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <link_interval_envelope/batch.h>
#include <link_interval_envelope/incremental_context.h>

#include <sbf/core/fk_state.h>
#include <sbf/core/robot.h>
#include <sbf/envelope/endpoint_source.h>
#include <sbf/envelope/envelope_collision.h>
#include <sbf/envelope/envelope_type.h>
#include <sbf/envelope/gcpc_source.h>

#include <chrono>
#include <stdexcept>
#include <string>
#include <vector>

namespace py = pybind11;
namespace lie = link_interval_envelope;

namespace {

using Clock = std::chrono::steady_clock;

struct OutputOptions {
    bool include_arrays = true;
    bool include_inflated = true;
    bool include_midpoints = true;
};

OutputOptions parse_output_mode(const std::string& mode) {
    if (mode == "full") {
        return {};
    }
    if (mode == "summary" || mode == "metadata") {
        return {false, false, false};
    }
    if (mode == "arrays") {
        return {true, false, false};
    }
    if (mode == "compact") {
        return {false, false, false};
    }
    throw std::invalid_argument("output_mode must be one of: full, arrays, summary, metadata, compact");
}

std::vector<double> active_link_radii_vec(const rbf::Robot& robot) {
    const double* radii = robot.active_link_radii();
    if (!radii) return {};
    return std::vector<double>(radii, radii + robot.n_active_links());
}

std::vector<int> active_link_map_vec(const rbf::Robot& robot) {
    const int* map = robot.active_link_map();
    if (!map) return {};
    return std::vector<int>(map, map + robot.n_active_links());
}

std::vector<float> inflate_link_iaabbs(
    const std::vector<float>& link_iaabbs,
    int n_active_links,
    int n_subdivisions,
    const double* link_radii)
{
    std::vector<float> out = link_iaabbs;
    const int n_sub = std::max(1, n_subdivisions);
    const int n_boxes = n_active_links * n_sub;
    for (int i = 0; i < n_boxes; ++i) {
        const int link_idx = i / n_sub;
        const float radius = link_radii ? static_cast<float>(link_radii[link_idx]) : 0.0f;
        float* box = out.data() + i * 6;
        box[0] -= radius;
        box[1] -= radius;
        box[2] -= radius;
        box[3] += radius;
        box[4] += radius;
        box[5] += radius;
    }
    return out;
}

rbf::EnvelopeCollisionMode parse_collision_mode(const std::string& mode) {
    if (mode == "auto") return rbf::EnvelopeCollisionMode::Auto;
    if (mode == "aabb_only" || mode == "link_aabb") return rbf::EnvelopeCollisionMode::LinkAABB;
    if (mode == "support_hull_only" || mode == "gjk") return rbf::EnvelopeCollisionMode::GJK;
    if (mode == "kdop") return rbf::EnvelopeCollisionMode::KDOP;
    if (mode == "kdop_then_gjk") return rbf::EnvelopeCollisionMode::KDOPThenGJK;
    throw std::invalid_argument("collision_mode must be one of: auto, aabb_only, support_hull_only, kdop, kdop_then_gjk");
}

std::vector<double> midpoint_endpoint_positions(
    const rbf::Robot& robot,
    const std::vector<rbf::Interval>& intervals)
{
    std::vector<rbf::Interval> midpoint_intervals;
    midpoint_intervals.reserve(intervals.size());
    for (const auto& interval : intervals) {
        const double center = interval.center();
        midpoint_intervals.emplace_back(center, center);
    }

    const rbf::FKState fk = rbf::compute_fk_full(robot, midpoint_intervals);
    const int n_active = robot.n_active_links();
    std::vector<float> endpoint_iaabbs(n_active * 2 * 6);
    rbf::extract_endpoint_iaabbs(
        fk,
        robot.active_link_map(),
        n_active,
        endpoint_iaabbs.data());

    std::vector<double> positions;
    positions.reserve(static_cast<std::size_t>(n_active) * 2 * 3);
    for (int endpoint = 0; endpoint < n_active * 2; ++endpoint) {
        const float* box = endpoint_iaabbs.data() + endpoint * 6;
        positions.push_back(0.5 * (static_cast<double>(box[0]) + static_cast<double>(box[3])));
        positions.push_back(0.5 * (static_cast<double>(box[1]) + static_cast<double>(box[4])));
        positions.push_back(0.5 * (static_cast<double>(box[2]) + static_cast<double>(box[5])));
    }
    return positions;
}

py::dict envelope_to_dict(
    const rbf::Robot& robot,
    const rbf::EndpointIAABBResult& endpoint_result,
    const rbf::LinkEnvelope& envelope,
    double endpoint_time_us,
    double envelope_time_us,
    const std::string& output_mode,
    const std::vector<rbf::Interval>* intervals)
{
    const OutputOptions output = parse_output_mode(output_mode);
    py::dict result;
    result["robot_name"] = robot.name();
    result["n_joints"] = robot.n_joints();
    result["n_active_links"] = endpoint_result.n_active_links;
    result["active_link_map"] = active_link_map_vec(robot);
    result["active_link_radii"] = active_link_radii_vec(robot);
    result["endpoint_source"] = std::string(rbf::endpoint_source_name(endpoint_result.source));
    result["endpoint_is_safe"] = endpoint_result.is_safe;
    result["n_pruned_links"] = endpoint_result.n_pruned_links;
    result["combo_count"] = endpoint_result.combo_count;
    result["enumerate_threads"] = endpoint_result.enumerate_threads;
    result["enumerate_time_us"] = endpoint_result.enumerate_time_us;
    result["changed_dim"] = endpoint_result.changed_dim;
    result["parallel_min_combos_used"] = endpoint_result.parallel_min_combos_used;
    result["enumerate_chunk_size"] = endpoint_result.enumerate_chunk_size;
    result["enumerate_chunk_count"] = endpoint_result.enumerate_chunk_count;
    result["candidate_dirty_count"] = endpoint_result.candidate_dirty_count;
    result["predh_rebuild_count"] = endpoint_result.predh_rebuild_count;
    result["endpoint_cache_reused"] = endpoint_result.endpoint_cache_reused;
    result["endpoint_shape"] = py::make_tuple(endpoint_result.n_active_links, 2, 6);
    if (output.include_arrays) {
        result["endpoint_iaabbs"] = endpoint_result.endpoint_iaabbs;
    }
    result["envelope_type"] = std::string(rbf::envelope_type_name(envelope.type));
    result["n_subdivisions"] = envelope.n_subdivisions;
    result["link_shape"] = py::make_tuple(envelope.n_active_links, envelope.n_subdivisions, 6);
    if (output.include_arrays) {
        result["link_iaabbs"] = envelope.link_iaabbs;
        result["support_hulls"] = envelope.support_hulls;
    }
    if (output.include_inflated) {
        result["inflated_link_iaabbs"] = inflate_link_iaabbs(
            envelope.link_iaabbs,
            envelope.n_active_links,
            envelope.n_subdivisions,
            robot.active_link_radii());
    }
    result["endpoint_time_us"] = endpoint_time_us;
    result["envelope_time_us"] = envelope_time_us;
    result["total_time_us"] = endpoint_time_us + envelope_time_us;
    if (output.include_midpoints && intervals != nullptr) {
        result["midpoint_endpoint_positions"] = midpoint_endpoint_positions(robot, *intervals);
    } else if (output.include_midpoints) {
        result["midpoint_endpoint_positions"] = std::vector<double>{};
    }
    return result;
}

py::dict batch_result_to_dict(
    const rbf::Robot& robot,
    const lie::EnvelopeBatchResult& item,
    const std::string& output_mode,
    const std::vector<rbf::Interval>& intervals)
{
    const OutputOptions output = parse_output_mode(output_mode);
    py::dict result;
    result["robot_name"] = robot.name();
    result["n_joints"] = robot.n_joints();
    result["n_active_links"] = item.n_active_links;
    result["active_link_map"] = active_link_map_vec(robot);
    result["active_link_radii"] = active_link_radii_vec(robot);
    result["endpoint_source"] = std::string(rbf::endpoint_source_name(item.source));
    result["endpoint_is_safe"] = item.is_safe;
    result["n_pruned_links"] = item.n_pruned_links;
    result["combo_count"] = item.combo_count;
    result["enumerate_threads"] = item.enumerate_threads;
    result["enumerate_time_us"] = item.enumerate_time_us;
    result["changed_dim"] = item.changed_dim;
    result["parallel_min_combos_used"] = item.parallel_min_combos_used;
    result["enumerate_chunk_size"] = item.enumerate_chunk_size;
    result["enumerate_chunk_count"] = item.enumerate_chunk_count;
    result["candidate_dirty_count"] = item.candidate_dirty_count;
    result["predh_rebuild_count"] = item.predh_rebuild_count;
    result["endpoint_cache_reused"] = item.endpoint_cache_reused;
    result["nested_parallelism_suppressed"] = item.nested_parallelism_suppressed;
    result["endpoint_shape"] = py::make_tuple(item.n_active_links, 2, 6);
    if (output.include_arrays) {
        result["endpoint_iaabbs"] = item.endpoint_iaabbs;
    }
    result["envelope_type"] = std::string(rbf::envelope_type_name(item.envelope.type));
    result["n_subdivisions"] = item.envelope.n_subdivisions;
    result["link_shape"] = py::make_tuple(
        item.envelope.n_active_links,
        item.envelope.n_subdivisions,
        6);
    if (output.include_arrays) {
        result["link_iaabbs"] = item.envelope.link_iaabbs;
        result["support_hulls"] = item.envelope.support_hulls;
    }
    if (output.include_inflated) {
        result["inflated_link_iaabbs"] = inflate_link_iaabbs(
            item.envelope.link_iaabbs,
            item.envelope.n_active_links,
            item.envelope.n_subdivisions,
            robot.active_link_radii());
    }
    result["endpoint_time_us"] = item.endpoint_time_us;
    result["envelope_time_us"] = item.envelope_time_us;
    result["total_time_us"] = item.endpoint_time_us + item.envelope_time_us;
    if (output.include_midpoints) {
        result["midpoint_endpoint_positions"] = midpoint_endpoint_positions(robot, intervals);
    }
    return result;
}

}  // namespace

PYBIND11_MODULE(_link_interval_envelope_cpp, module) {
    module.doc() = "Self-contained link interval envelope package";

    py::class_<rbf::Interval>(module, "Interval")
        .def(py::init<>())
        .def(py::init<double, double>(), py::arg("lo"), py::arg("hi"))
        .def_readwrite("lo", &rbf::Interval::lo)
        .def_readwrite("hi", &rbf::Interval::hi)
        .def("width", &rbf::Interval::width)
        .def("center", &rbf::Interval::center)
        .def("empty", &rbf::Interval::empty);

    py::class_<rbf::JointLimits>(module, "JointLimits")
        .def(py::init<>())
        .def_readwrite("limits", &rbf::JointLimits::limits)
        .def("n_dims", &rbf::JointLimits::n_dims);

    py::class_<rbf::FKState>(module, "FKState")
        .def(py::init<>())
        .def_readwrite("n_tf", &rbf::FKState::n_tf)
        .def_readwrite("n_jm", &rbf::FKState::n_jm)
        .def_readwrite("valid", &rbf::FKState::valid)
        .def("reset", [](rbf::FKState& self) { self = rbf::FKState{}; });

    py::class_<rbf::Robot>(module, "Robot")
        .def_static("from_json", &rbf::Robot::from_json, py::arg("path"))
        .def("name", &rbf::Robot::name)
        .def("n_joints", &rbf::Robot::n_joints)
        .def("n_active_links", &rbf::Robot::n_active_links)
        .def("has_tool", &rbf::Robot::has_tool)
        .def("fingerprint", &rbf::Robot::fingerprint)
        .def("joint_limits", &rbf::Robot::joint_limits, py::return_value_policy::reference_internal)
        .def("link_radii", &rbf::Robot::link_radii, py::return_value_policy::reference_internal)
        .def("active_link_map", &active_link_map_vec)
        .def("active_link_radii", &active_link_radii_vec);

    py::enum_<rbf::EndpointSource>(module, "EndpointSource")
        .value("IFK", rbf::EndpointSource::IFK)
        .value("CritSample", rbf::EndpointSource::CritSample)
        .value("Analytical", rbf::EndpointSource::Analytical)
        .value("GCPC", rbf::EndpointSource::GCPC)
        .value("MC", rbf::EndpointSource::MC)
        .value("HIFK", rbf::EndpointSource::HIFK);

    py::enum_<rbf::EnvelopeType>(module, "EnvelopeType")
        .value("LinkIAABB", rbf::EnvelopeType::LinkIAABB)
        .value("KDOP", rbf::EnvelopeType::KDOP)
        .value("SupportHull", rbf::EnvelopeType::SupportHull);

    py::enum_<rbf::KdopDirectionSet>(module, "KdopDirectionSet")
        .value("DOP6", rbf::KdopDirectionSet::DOP6)
        .value("DOP18", rbf::KdopDirectionSet::DOP18)
        .value("DOP26", rbf::KdopDirectionSet::DOP26);

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
        .def("set_gcpc_cache", [](rbf::EndpointSourceConfig& self, const rbf::GcpcCache& cache) {
            self.gcpc_cache = &cache;
        }, py::arg("cache"));

    py::class_<rbf::KdopConfig>(module, "KdopConfig")
        .def(py::init<>())
        .def_readwrite("direction_set", &rbf::KdopConfig::direction_set)
        .def_readwrite("safety_epsilon", &rbf::KdopConfig::safety_epsilon);

    py::class_<rbf::SupportHullConfig>(module, "SupportHullConfig")
        .def(py::init<>())
        .def_readwrite("keep_kdop", &rbf::SupportHullConfig::keep_kdop)
        .def_readwrite("safety_epsilon", &rbf::SupportHullConfig::safety_epsilon);

    py::class_<rbf::EnvelopeTypeConfig>(module, "EnvelopeTypeConfig")
        .def(py::init<>())
        .def_readwrite("type", &rbf::EnvelopeTypeConfig::type)
        .def_readwrite("n_subdivisions", &rbf::EnvelopeTypeConfig::n_subdivisions)
        .def_readwrite("kdop_config", &rbf::EnvelopeTypeConfig::kdop_config)
        .def_readwrite("support_hull_config", &rbf::EnvelopeTypeConfig::support_hull_config);

    py::class_<rbf::GcpcCache>(module, "GcpcCache")
        .def(py::init<>())
        .def_static("load", &rbf::GcpcCache::load, py::arg("path"))
        .def("save", &rbf::GcpcCache::save, py::arg("path"))
        .def("n_points", &rbf::GcpcCache::n_points)
        .def("n_dims", &rbf::GcpcCache::n_dims)
        .def("empty", &rbf::GcpcCache::empty);

    module.def("recommend_hifk_depth",
               py::overload_cast<const std::vector<rbf::Interval>&, int>(&rbf::recommend_hifk_depth),
               py::arg("intervals"),
               py::arg("max_depth_cap") = 5);
    module.def("recommend_hifk_depth_for_robot",
               py::overload_cast<const rbf::Robot&, const std::vector<rbf::Interval>&, int>(&rbf::recommend_hifk_depth),
               py::arg("robot"),
               py::arg("intervals"),
               py::arg("max_depth_cap") = 5);

    module.def("compute_endpoint_iaabb_info", [](
        const rbf::Robot& robot,
        const std::vector<rbf::Interval>& intervals,
        rbf::EndpointSourceConfig endpoint_config,
        const rbf::GcpcCache* gcpc_cache,
        const std::string& output_mode) {
        const OutputOptions output = parse_output_mode(output_mode);
        if (endpoint_config.source == rbf::EndpointSource::GCPC) {
            if (!gcpc_cache) {
                throw std::invalid_argument("GCPC endpoint source requires a GcpcCache");
            }
            endpoint_config.gcpc_cache = gcpc_cache;
        }
        rbf::EndpointIAABBResult endpoint_result;
        Clock::time_point start;
        Clock::time_point stop;
        {
            py::gil_scoped_release release;
            start = Clock::now();
            endpoint_result = rbf::compute_endpoint_iaabb(robot, intervals, endpoint_config);
            stop = Clock::now();
        }
        py::dict result;
        result["robot_name"] = robot.name();
        result["n_joints"] = robot.n_joints();
        result["n_active_links"] = endpoint_result.n_active_links;
        result["active_link_map"] = active_link_map_vec(robot);
        result["active_link_radii"] = active_link_radii_vec(robot);
        result["source"] = std::string(rbf::endpoint_source_name(endpoint_result.source));
        result["is_safe"] = endpoint_result.is_safe;
        result["n_pruned_links"] = endpoint_result.n_pruned_links;
        result["combo_count"] = endpoint_result.combo_count;
        result["enumerate_threads"] = endpoint_result.enumerate_threads;
        result["enumerate_time_us"] = endpoint_result.enumerate_time_us;
        result["changed_dim"] = endpoint_result.changed_dim;
        result["parallel_min_combos_used"] = endpoint_result.parallel_min_combos_used;
        result["enumerate_chunk_size"] = endpoint_result.enumerate_chunk_size;
        result["enumerate_chunk_count"] = endpoint_result.enumerate_chunk_count;
        result["candidate_dirty_count"] = endpoint_result.candidate_dirty_count;
        result["predh_rebuild_count"] = endpoint_result.predh_rebuild_count;
        result["endpoint_cache_reused"] = endpoint_result.endpoint_cache_reused;
        result["endpoint_shape"] = py::make_tuple(endpoint_result.n_active_links, 2, 6);
        if (output.include_arrays) {
            result["endpoint_iaabbs"] = endpoint_result.endpoint_iaabbs;
        }
        result["endpoint_time_us"] = std::chrono::duration<double, std::micro>(stop - start).count();
        if (output.include_midpoints) {
            result["midpoint_endpoint_positions"] = midpoint_endpoint_positions(robot, intervals);
        }
        return result;
    }, py::arg("robot"), py::arg("intervals"), py::arg("endpoint_config"), py::arg("gcpc_cache") = nullptr,
       py::arg("output_mode") = "full");

    module.def("compute_link_envelope_from_endpoints", [](
        const rbf::Robot& robot,
        const std::vector<float>& endpoint_iaabbs,
        const rbf::EnvelopeTypeConfig& envelope_config,
        const std::string& output_mode) {
        const OutputOptions output = parse_output_mode(output_mode);
        const int n_active = robot.n_active_links();
        if (static_cast<int>(endpoint_iaabbs.size()) != n_active * 2 * 6) {
            throw std::invalid_argument("endpoint_iaabbs must have shape [n_active, 2, 6]");
        }
        rbf::LinkEnvelope envelope;
        Clock::time_point start;
        Clock::time_point stop;
        {
            py::gil_scoped_release release;
            start = Clock::now();
            envelope = rbf::compute_link_envelope(
                endpoint_iaabbs.data(), n_active, robot.active_link_radii(), envelope_config);
            stop = Clock::now();
        }
        py::dict result;
        result["robot_name"] = robot.name();
        result["n_joints"] = robot.n_joints();
        result["n_active_links"] = n_active;
        result["active_link_map"] = active_link_map_vec(robot);
        result["active_link_radii"] = active_link_radii_vec(robot);
        result["envelope_type"] = std::string(rbf::envelope_type_name(envelope.type));
        result["n_subdivisions"] = envelope.n_subdivisions;
        result["link_shape"] = py::make_tuple(n_active, envelope.n_subdivisions, 6);
        if (output.include_arrays) {
            result["link_iaabbs"] = envelope.link_iaabbs;
            result["support_hulls"] = envelope.support_hulls;
        }
        if (output.include_inflated) {
            result["inflated_link_iaabbs"] = inflate_link_iaabbs(
                envelope.link_iaabbs, n_active, envelope.n_subdivisions, robot.active_link_radii());
        }
        result["envelope_time_us"] = std::chrono::duration<double, std::micro>(stop - start).count();
        return result;
    }, py::arg("robot"), py::arg("endpoint_iaabbs"), py::arg("envelope_config"),
       py::arg("output_mode") = "full");

    module.def("compute_envelope_info", [](
        const rbf::Robot& robot,
        const std::vector<rbf::Interval>& intervals,
        rbf::EndpointSourceConfig endpoint_config,
        const rbf::EnvelopeTypeConfig& envelope_config,
        const rbf::GcpcCache* gcpc_cache,
        const std::string& output_mode) {
        if (endpoint_config.source == rbf::EndpointSource::GCPC) {
            if (!gcpc_cache) {
                throw std::invalid_argument("GCPC endpoint source requires a GcpcCache");
            }
            endpoint_config.gcpc_cache = gcpc_cache;
        }

        rbf::EndpointIAABBResult endpoint_result;
        rbf::LinkEnvelope envelope;
        Clock::time_point endpoint_start;
        Clock::time_point endpoint_stop;
        Clock::time_point envelope_start;
        Clock::time_point envelope_stop;
        {
            py::gil_scoped_release release;
            endpoint_start = Clock::now();
            endpoint_result = rbf::compute_endpoint_iaabb(robot, intervals, endpoint_config);
            endpoint_stop = Clock::now();

            envelope_start = Clock::now();
            envelope = rbf::compute_link_envelope(
                endpoint_result.endpoint_iaabbs.data(),
                endpoint_result.n_active_links,
                robot.active_link_radii(),
                envelope_config);
            envelope_stop = Clock::now();
        }

        return envelope_to_dict(
            robot,
            endpoint_result,
            envelope,
            std::chrono::duration<double, std::micro>(endpoint_stop - endpoint_start).count(),
            std::chrono::duration<double, std::micro>(envelope_stop - envelope_start).count(),
                        output_mode,
            &intervals);
    }, py::arg("robot"), py::arg("intervals"), py::arg("endpoint_config"),
       py::arg("envelope_config"), py::arg("gcpc_cache") = nullptr,
             py::arg("output_mode") = "full");

    module.def("compute_envelope_collision_info", [](
        const rbf::Robot& robot,
        const std::vector<rbf::Interval>& intervals,
        rbf::EndpointSourceConfig endpoint_config,
        const rbf::EnvelopeTypeConfig& envelope_config,
        const std::vector<float>& obstacle_aabbs,
        const std::string& collision_mode,
        const std::string& output_mode,
        bool use_link_aabb_broadphase,
        bool count_all_pairs) {
        if (obstacle_aabbs.size() % 6u != 0u) {
            throw std::invalid_argument("obstacle_aabbs must contain 6 floats per obstacle");
        }
        std::vector<rbf::Obstacle> obstacles;
        obstacles.reserve(obstacle_aabbs.size() / 6u);
        for (std::size_t offset = 0; offset < obstacle_aabbs.size(); offset += 6u) {
            obstacles.emplace_back(
                obstacle_aabbs[offset + 0u],
                obstacle_aabbs[offset + 1u],
                obstacle_aabbs[offset + 2u],
                obstacle_aabbs[offset + 3u],
                obstacle_aabbs[offset + 4u],
                obstacle_aabbs[offset + 5u]);
        }

        rbf::EndpointIAABBResult endpoint_result;
        rbf::LinkEnvelope envelope;
        rbf::EnvelopeCollisionStats stats;
        rbf::CollisionResultKind collision;
        Clock::time_point endpoint_start;
        Clock::time_point endpoint_stop;
        Clock::time_point envelope_start;
        Clock::time_point envelope_stop;
        Clock::time_point collision_start;
        Clock::time_point collision_stop;
        {
            py::gil_scoped_release release;
            endpoint_start = Clock::now();
            endpoint_result = rbf::compute_endpoint_iaabb(robot, intervals, endpoint_config);
            endpoint_stop = Clock::now();

            envelope_start = Clock::now();
            envelope = rbf::compute_link_envelope(
                endpoint_result.endpoint_iaabbs.data(),
                endpoint_result.n_active_links,
                robot.active_link_radii(),
                envelope_config);
            envelope_stop = Clock::now();

            rbf::EnvelopeCollisionOptions options;
            options.mode = parse_collision_mode(collision_mode);
            options.use_link_aabb_broadphase = use_link_aabb_broadphase;
            options.count_all_pairs = count_all_pairs;
            collision_start = Clock::now();
            collision = rbf::collide_envelope_aabbs(
                envelope,
                obstacles.empty() ? nullptr : obstacles.data(),
                static_cast<int>(obstacles.size()),
                options,
                &stats);
            collision_stop = Clock::now();
        }

        py::dict result;
        result["endpoint_time_us"] = std::chrono::duration<double, std::micro>(endpoint_stop - endpoint_start).count();
        result["envelope_time_us"] = std::chrono::duration<double, std::micro>(envelope_stop - envelope_start).count();
        result["collision_time_us"] = std::chrono::duration<double, std::micro>(collision_stop - collision_start).count();
        result["total_time_us"] = result["endpoint_time_us"].cast<double>()
            + result["envelope_time_us"].cast<double>()
            + result["collision_time_us"].cast<double>();
        result["is_definitely_free"] = collision == rbf::CollisionResultKind::DefinitelyFree;
        result["collision_mode"] = collision_mode;
        result["maybe_pairs"] = stats.maybe_pairs;
        result["link_aabb_tests"] = stats.link_aabb_tests;
        result["link_aabb_rejects"] = stats.link_aabb_rejects;
        result["gjk_tests"] = stats.gjk_tests;
        result["gjk_rejects"] = stats.gjk_rejects;
        result["gjk_iterations"] = stats.gjk_iterations;
        result["kdop_tests"] = stats.kdop_tests;
        if (output_mode == "full") {
            result["endpoint_is_safe"] = endpoint_result.is_safe;
            result["n_active_links"] = endpoint_result.n_active_links;
            result["n_subdivisions"] = envelope.n_subdivisions;
        } else if (output_mode != "summary" && output_mode != "metadata" && output_mode != "compact") {
            throw std::invalid_argument("output_mode must be one of: full, summary, metadata, compact");
        }
        return result;
    }, py::arg("robot"), py::arg("intervals"), py::arg("endpoint_config"),
       py::arg("envelope_config"), py::arg("obstacle_aabbs"),
       py::arg("collision_mode") = "auto", py::arg("output_mode") = "summary",
       py::arg("use_link_aabb_broadphase") = true,
       py::arg("count_all_pairs") = false);

    module.def("compute_envelope_batch_info", [](
        const rbf::Robot& robot,
        const std::vector<std::vector<rbf::Interval>>& interval_boxes,
        const rbf::EndpointSourceConfig& endpoint_config,
        const rbf::EnvelopeTypeConfig& envelope_config,
        int n_threads,
        const std::string& output_mode) {
        std::vector<lie::EnvelopeBatchResult> computed;
        {
            py::gil_scoped_release release;
            computed = lie::compute_envelope_batch(
                robot,
                interval_boxes,
                endpoint_config,
                envelope_config,
                n_threads);
        }
        py::list out;
        for (std::size_t i = 0; i < computed.size(); ++i) {
            out.append(batch_result_to_dict(
                robot,
                computed[i],
                output_mode,
                interval_boxes[i]));
        }
        return out;
    }, py::arg("robot"), py::arg("interval_boxes"), py::arg("endpoint_config"),
       py::arg("envelope_config"), py::arg("n_threads") = 0,
         py::arg("output_mode") = "full");

    py::class_<lie::IncrementalEnvelopeContext>(module, "IncrementalEnvelopeContext")
        .def(py::init<rbf::Robot, rbf::EndpointSourceConfig, rbf::EnvelopeTypeConfig>(),
             py::arg("robot"), py::arg("endpoint_config"), py::arg("envelope_config"))
        .def("reset", &lie::IncrementalEnvelopeContext::reset)
        .def("has_valid_fk", &lie::IncrementalEnvelopeContext::has_valid_fk)
        .def("fk_state", static_cast<rbf::FKState& (lie::IncrementalEnvelopeContext::*)()>(
                &lie::IncrementalEnvelopeContext::fk_state),
             py::return_value_policy::reference_internal)
        .def("compute", [](
            lie::IncrementalEnvelopeContext& self,
            const std::vector<rbf::Interval>& intervals,
            int changed_dim,
            const std::string& output_mode) {
            lie::IncrementalEnvelopeResult computed;
            {
                py::gil_scoped_release release;
                computed = self.compute(intervals, changed_dim);
            }
            py::dict result = envelope_to_dict(
                self.robot(),
                computed.endpoint,
                computed.envelope,
                computed.endpoint_time_us,
                computed.envelope_time_us,
                output_mode,
                &intervals);
            result["changed_dim"] = computed.changed_dim;
            result["used_incremental_fk"] = computed.used_incremental_fk;
            result["used_source_incremental_state"] = computed.used_source_incremental_state;
            result["reused_fk"] = computed.reused_fk;
            result["reused_endpoint_cache"] = computed.reused_endpoint_cache;
            result["fk_valid"] = self.has_valid_fk();
            return result;
                }, py::arg("intervals"), py::arg("changed_dim") = -1, py::arg("output_mode") = "full");
}
