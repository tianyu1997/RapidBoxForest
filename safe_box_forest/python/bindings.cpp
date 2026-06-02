#include <SBF/sbf.h>

#include <cstdio>
#include <cstdlib>

#include <rbf/lect_database/read_snapshot.h>
#include <sbf/envelope/ifk_aa_source.h>

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#define private public
#define protected public
#include <ompl/base/samplers/informed/PathLengthDirectInfSampler.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/informedtrees/bitstar/ImplicitGraph.h>
#undef protected
#undef private

#include <ompl/base/PlannerData.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/util/Console.h>
#include <ompl/util/RandomNumbers.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace py = pybind11;

namespace {

std::vector<int> active_link_map_vec(const rbf::Robot& robot) {
    const int* map = robot.active_link_map();
    if (map == nullptr) return {};
    return std::vector<int>(map, map + robot.n_active_links());
}

std::vector<double> active_link_radii_vec(const rbf::Robot& robot) {
    const double* radii = robot.active_link_radii();
    if (radii == nullptr) return {};
    return std::vector<double>(radii, radii + robot.n_active_links());
}

Eigen::VectorXd eigen_vector_from_list(const std::vector<double>& values) {
    Eigen::VectorXd vector(static_cast<Eigen::Index>(values.size()));
    for (std::size_t i = 0; i < values.size(); ++i) {
        vector[static_cast<Eigen::Index>(i)] = values[i];
    }
    return vector;
}

std::vector<Eigen::VectorXd> eigen_vectors_from_lists(const std::vector<std::vector<double>>& values) {
    std::vector<Eigen::VectorXd> result;
    result.reserve(values.size());
    for (const auto& item : values) {
        result.push_back(eigen_vector_from_list(item));
    }
    return result;
}

std::vector<double> vector_to_list(const Eigen::VectorXd& values) {
    std::vector<double> result(static_cast<std::size_t>(values.size()));
    for (Eigen::Index i = 0; i < values.size(); ++i) {
        result[static_cast<std::size_t>(i)] = values[i];
    }
    return result;
}

std::vector<rbf::Interval> intervals_from_pairs(const std::vector<std::vector<double>>& pairs) {
    std::vector<rbf::Interval> intervals;
    intervals.reserve(pairs.size());
    for (const auto& pair : pairs) {
        if (pair.size() != 2) {
            throw std::invalid_argument("each interval pair must have exactly two entries");
        }
        if (pair[1] < pair[0]) {
            throw std::invalid_argument("interval upper bound must be >= lower bound");
        }
        intervals.push_back(rbf::Interval{pair[0], pair[1]});
    }
    return intervals;
}

py::list interval_pairs_to_python(const std::vector<rbf::Interval>& intervals) {
    py::list result;
    for (const auto& interval : intervals) {
        py::list pair;
        pair.append(interval.lo);
        pair.append(interval.hi);
        result.append(std::move(pair));
    }
    return result;
}

py::dict oracle_validation_detail_to_python(const rbf::OracleValidationDetail& detail) {
    py::dict result;
    result["node"] = detail.node;
    result["depth"] = detail.depth;
    result["mode"] = static_cast<int>(detail.mode);
    result["validation"] = static_cast<int>(detail.validation);
    result["safety_status"] = static_cast<int>(detail.safety_status);
    result["collision_possible"] = detail.collision_possible;
    result["strict_audit_required"] = detail.strict_audit_required;
    result["endpoint_source"] = static_cast<int>(detail.endpoint_source);
    result["endpoint_is_safe"] = detail.endpoint_is_safe;
    result["endpoint_safety_level"] = static_cast<int>(detail.endpoint_safety_level);
    result["materialized"] = detail.materialized;
    result["changed_dim"] = detail.changed_dim;
    result["used_incremental_fk"] = detail.used_incremental_fk;
    result["used_source_incremental_state"] = detail.used_source_incremental_state;
    result["reused_fk"] = detail.reused_fk;
    result["reused_endpoint_cache"] = detail.reused_endpoint_cache;
    result["reused_external_evidence"] = detail.reused_external_evidence;
    result["endpoint_time_us"] = detail.endpoint_time_us;
    result["envelope_time_us"] = detail.envelope_time_us;
    result["candidate_dirty_count"] = detail.candidate_dirty_count;
    result["predh_rebuild_count"] = detail.predh_rebuild_count;
    result["aabb_overlap"] = detail.aabb_overlap;
    return result;
}

py::dict oracle_counters_to_python(const rbf::OracleCounters& counters) {
    py::dict result;
    result["node_validations"] = counters.node_validations;
    result["interval_validations"] = counters.interval_validations;
    result["certified_free"] = counters.certified_free;
    result["provisional_free"] = counters.provisional_free;
    result["collision_possible"] = counters.collision_possible;
    result["unsafe_free_rejected"] = counters.unsafe_free_rejected;
    result["validation_cache_hits"] = counters.validation_cache_hits;
    result["validation_cache_misses"] = counters.validation_cache_misses;
    result["materializations"] = counters.materializations;
    result["materialization_stored_endpoint"] = counters.materialization_stored_endpoint;
    result["materialization_skipped_endpoint_cache"] = counters.materialization_skipped_endpoint_cache;
    result["materialization_endpoint_time_us"] = counters.materialization_endpoint_time_us;
    result["materialization_endpoint_wall_time_us"] = counters.materialization_endpoint_wall_time_us;
    result["materialization_envelope_time_us"] = counters.materialization_envelope_time_us;
    result["validate_node_total_time_us"] = counters.validate_node_total_time_us;
    result["validate_node_preamble_time_us"] = counters.validate_node_preamble_time_us;
    result["validate_node_endpoint_path_time_us"] = counters.validate_node_endpoint_path_time_us;
    result["validate_node_classify_time_us"] = counters.validate_node_classify_time_us;
    result["validate_node_overhead_time_us"] = counters.validate_node_overhead_time_us;
    result["materialization_cache_lookup_time_us"] = counters.materialization_cache_lookup_time_us;
    result["materialization_cache_read_time_us"] = counters.materialization_cache_read_time_us;
    result["materialization_external_lookup_time_us"] = counters.materialization_external_lookup_time_us;
    result["materialization_external_read_time_us"] = counters.materialization_external_read_time_us;
    result["materialization_envelope_compute_time_us"] = counters.materialization_envelope_compute_time_us;
    result["materialization_envelope_read_time_us"] = counters.materialization_envelope_read_time_us;
    result["materialization_envelope_collision_time_us"] = counters.materialization_envelope_collision_time_us;
    result["materialization_incremental_fk"] = counters.materialization_incremental_fk;
    result["materialization_source_incremental_state"] = counters.materialization_source_incremental_state;
    result["materialization_reused_fk"] = counters.materialization_reused_fk;
    result["materialization_reused_endpoint_cache"] = counters.materialization_reused_endpoint_cache;
    result["materialization_reused_external_evidence"] = counters.materialization_reused_external_evidence;
    result["materialization_reused_shared_endpoint_cache"] = counters.materialization_reused_shared_endpoint_cache;
    result["materialization_stored_shared_endpoint_cache"] = counters.materialization_stored_shared_endpoint_cache;
    result["materialization_reused_cached_envelope"] = counters.materialization_reused_cached_envelope;
    result["materialization_candidate_dirty_count"] = counters.materialization_candidate_dirty_count;
    result["materialization_predh_rebuild_count"] = counters.materialization_predh_rebuild_count;
    result["scoring_evaluations"] = counters.scoring_evaluations;
    result["scoring_changed_dim_inferred"] = counters.scoring_changed_dim_inferred;
    result["scoring_incremental_fk"] = counters.scoring_incremental_fk;
    result["scoring_source_incremental_state"] = counters.scoring_source_incremental_state;
    result["scoring_reused_fk"] = counters.scoring_reused_fk;
    result["scoring_reused_endpoint_cache"] = counters.scoring_reused_endpoint_cache;
    result["scoring_reused_external_evidence"] = counters.scoring_reused_external_evidence;
    result["scoring_endpoint_time_us"] = counters.scoring_endpoint_time_us;
    result["scoring_envelope_time_us"] = counters.scoring_envelope_time_us;
    result["scoring_candidate_dirty_count"] = counters.scoring_candidate_dirty_count;
    result["scoring_predh_rebuild_count"] = counters.scoring_predh_rebuild_count;
    result["envelope_collision_queries"] = counters.envelope_collision_queries;
    result["envelope_collision_free"] = counters.envelope_collision_free;
    result["envelope_collision_maybe"] = counters.envelope_collision_maybe;
    result["envelope_collision_envelope_aabb_tests"] = counters.envelope_collision_envelope_aabb_tests;
    result["envelope_collision_envelope_aabb_rejects"] = counters.envelope_collision_envelope_aabb_rejects;
    result["envelope_collision_link_union_aabb_tests"] = counters.envelope_collision_link_union_aabb_tests;
    result["envelope_collision_link_union_aabb_rejects"] = counters.envelope_collision_link_union_aabb_rejects;
    result["envelope_collision_link_aabb_tests"] = counters.envelope_collision_link_aabb_tests;
    result["envelope_collision_link_aabb_rejects"] = counters.envelope_collision_link_aabb_rejects;
    result["envelope_collision_kdop_tests"] = counters.envelope_collision_kdop_tests;
    result["envelope_collision_kdop_rejects"] = counters.envelope_collision_kdop_rejects;
    result["envelope_collision_kdop_axes_tested"] = counters.envelope_collision_kdop_axes_tested;
    result["envelope_collision_gjk_tests"] = counters.envelope_collision_gjk_tests;
    result["envelope_collision_gjk_rejects"] = counters.envelope_collision_gjk_rejects;
    result["envelope_collision_gjk_iterations"] = counters.envelope_collision_gjk_iterations;
    return result;
}

rbf::OracleValidationConfig uncached_validation_config(rbf::OracleValidationConfig config) {
    config.enable_validation_cache = false;
    config.enable_endpoint_evidence_cache = false;
    config.store_endpoint_evidence_cache = false;
    config.external_evidence_materialization = false;
    config.external_evidence_scoring = false;
    return config;
}

std::vector<double> list_from_ompl_state(const ompl::base::State* state, int dimension) {
    const auto* vector_state = state->as<ompl::base::RealVectorStateSpace::StateType>();
    std::vector<double> out(static_cast<std::size_t>(dimension));
    for (int dim = 0; dim < dimension; ++dim) {
        out[static_cast<std::size_t>(dim)] = vector_state->values[dim];
    }
    return out;
}

std::uint_fast32_t normalize_seed(int seed) {
    return static_cast<std::uint_fast32_t>(std::max(0, seed));
}

std::uint_fast32_t mix_seed(std::uint_fast32_t base_seed, std::uint_fast32_t stream_id) {
    std::uint64_t z = static_cast<std::uint64_t>(base_seed) + 0x9E3779B97F4A7C15ULL;
    z ^= static_cast<std::uint64_t>(stream_id) + 0x9E3779B97F4A7C15ULL + (z << 6U) + (z >> 2U);
    z ^= (z >> 30U);
    z *= 0xBF58476D1CE4E5B9ULL;
    z ^= (z >> 27U);
    z *= 0x94D049BB133111EBULL;
    z ^= (z >> 31U);
    return static_cast<std::uint_fast32_t>(z & 0xFFFFFFFFULL);
}

void seed_state_sampler_tree(const ompl::base::StateSamplerPtr& sampler,
                             std::uint_fast32_t base_seed,
                             std::uint_fast32_t& next_stream_id) {
    if (!sampler) {
        return;
    }
    sampler->rng_.setLocalSeed(mix_seed(base_seed, next_stream_id++));
    auto compound_sampler = std::dynamic_pointer_cast<ompl::base::CompoundStateSampler>(sampler);
    if (compound_sampler) {
        for (const auto& child_sampler : compound_sampler->samplers_) {
            seed_state_sampler_tree(child_sampler, base_seed, next_stream_id);
        }
    }
    auto subspace_sampler = std::dynamic_pointer_cast<ompl::base::SubspaceStateSampler>(sampler);
    if (subspace_sampler) {
        seed_state_sampler_tree(subspace_sampler->subspaceSampler_, base_seed, next_stream_id);
    }
}

class DeterministicRealVectorStateSampler final : public ompl::base::RealVectorStateSampler {
public:
    DeterministicRealVectorStateSampler(const ompl::base::StateSpace* space, std::uint_fast32_t local_seed)
        : ompl::base::RealVectorStateSampler(space) {
        rng_.setLocalSeed(local_seed);
    }
};

void configure_deterministic_state_sampler(const std::shared_ptr<ompl::base::RealVectorStateSpace>& space,
                                           std::uint_fast32_t base_seed) {
    auto sampler_counter = std::make_shared<std::atomic<std::uint_fast32_t>>(0U);
    space->setStateSamplerAllocator(
        [base_seed, sampler_counter](const ompl::base::StateSpace* state_space) -> ompl::base::StateSamplerPtr {
            const auto sampler_index = sampler_counter->fetch_add(1U, std::memory_order_relaxed);
            return std::make_shared<DeterministicRealVectorStateSampler>(state_space, mix_seed(base_seed, sampler_index));
        });
}

class SeededRRTConnect final : public ompl::geometric::RRTConnect {
public:
    using ompl::geometric::RRTConnect::RRTConnect;

    void setLocalSeed(std::uint_fast32_t local_seed) {
        rng_.setLocalSeed(local_seed);
    }
};

class SeededPRM final : public ompl::geometric::PRM {
public:
    using ompl::geometric::PRM::PRM;

    void setLocalSeed(std::uint_fast32_t local_seed) {
        rng_.setLocalSeed(local_seed);
    }
};

class SeededBITstar final : public ompl::geometric::BITstar {
public:
    using ompl::geometric::BITstar::BITstar;

    void setLocalSeed(std::uint_fast32_t local_seed) {
        local_seed_ = local_seed;
        applyLocalSeed();
    }

    void setup() override {
        ompl::geometric::BITstar::setup();
        applyLocalSeed();
    }

    void clear() override {
        ompl::geometric::BITstar::clear();
        applyLocalSeed();
    }

private:
    void applyLocalSeed() {
        if (graphPtr_) {
            graphPtr_->rng_.setLocalSeed(local_seed_);
            auto direct_sampler = std::dynamic_pointer_cast<ompl::base::PathLengthDirectInfSampler>(graphPtr_->sampler_);
            if (direct_sampler) {
                direct_sampler->rng_.setLocalSeed(mix_seed(local_seed_, 1U));
                std::uint_fast32_t sampler_stream_id = 2U;
                seed_state_sampler_tree(direct_sampler->baseSampler_, local_seed_, sampler_stream_id);
                seed_state_sampler_tree(direct_sampler->uninformedSubSampler_, local_seed_, sampler_stream_id);
            }
        }
    }

    std::uint_fast32_t local_seed_{0U};
};

double ompl_elapsed_s(const std::chrono::steady_clock::time_point& start) {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}

std::shared_ptr<ompl::base::RealVectorStateSpace> make_ompl_space(const rbf::Robot& robot) {
    namespace ob = ompl::base;
    const int dimension = robot.n_joints();
    auto space = std::make_shared<ob::RealVectorStateSpace>(dimension);
    ob::RealVectorBounds bounds(dimension);
    const auto& limits = robot.joint_limits().limits;
    if (static_cast<int>(limits.size()) != dimension) {
        throw std::invalid_argument("robot joint limits must match robot.n_joints()");
    }
    for (int dim = 0; dim < dimension; ++dim) {
        bounds.setLow(dim, limits[static_cast<std::size_t>(dim)].lo);
        bounds.setHigh(dim, limits[static_cast<std::size_t>(dim)].hi);
    }
    space->setBounds(bounds);
    return space;
}

std::shared_ptr<ompl::base::SpaceInformation> make_ompl_space_information(
    const rbf::Robot& robot,
    const std::vector<rbf::Obstacle>& obstacles,
    const std::shared_ptr<ompl::base::RealVectorStateSpace>& space,
    double segment_step,
    double& checking_resolution) {
    namespace ob = ompl::base;
    const int dimension = robot.n_joints();
    auto checker = std::make_shared<rbf::CollisionChecker>(robot, rbf::Scene(obstacles));
    auto si = std::make_shared<ob::SpaceInformation>(space);
    si->setStateValidityChecker([checker, dimension](const ob::State* state) {
        const auto* vector_state = state->as<ob::RealVectorStateSpace::StateType>();
        Eigen::VectorXd q(dimension);
        for (int dim = 0; dim < dimension; ++dim) {
            q[dim] = vector_state->values[dim];
        }
        return !checker->check_config(q);
    });
    const double maximum_extent = std::max(space->getMaximumExtent(), 1e-9);
    checking_resolution = std::clamp(std::max(segment_step, 1e-6) / maximum_extent, 1e-6, 0.05);
    si->setStateValidityCheckingResolution(checking_resolution);
    si->setup();
    return si;
}

void set_problem_query(const std::shared_ptr<ompl::base::ProblemDefinition>& problem,
                       const std::shared_ptr<ompl::base::RealVectorStateSpace>& space,
                       const std::shared_ptr<ompl::base::SpaceInformation>& si,
                       const std::vector<double>& start,
                       const std::vector<double>& goal,
                       int dimension) {
    namespace ob = ompl::base;
    problem->clearStartStates();
    problem->clearSolutionPaths();
    ob::ScopedState<ob::RealVectorStateSpace> q_start(space), q_goal(space);
    for (int dim = 0; dim < dimension; ++dim) {
        q_start->values[dim] = start[static_cast<std::size_t>(dim)];
        q_goal->values[dim] = goal[static_cast<std::size_t>(dim)];
    }
    problem->addStartState(q_start.get());
    auto goal_state = std::make_shared<ob::GoalState>(si);
    goal_state->setState(q_goal.get());
    problem->setGoal(goal_state);
}

std::vector<std::vector<double>> path_from_problem_solution(const std::shared_ptr<ompl::base::ProblemDefinition>& problem,
                                                            int dimension) {
    namespace og = ompl::geometric;
    auto path_ptr = problem->getSolutionPath();
    auto geometric_path = std::dynamic_pointer_cast<og::PathGeometric>(path_ptr);
    std::vector<std::vector<double>> path;
    if (!geometric_path) {
        return path;
    }
    path.reserve(geometric_path->getStateCount());
    for (std::size_t index = 0; index < geometric_path->getStateCount(); ++index) {
        path.push_back(list_from_ompl_state(geometric_path->getState(index), dimension));
    }
    return path;
}

std::vector<std::vector<double>> path_from_geometric_path(const ompl::geometric::PathGeometric& geometric_path,
                                                          int dimension) {
    std::vector<std::vector<double>> path;
    path.reserve(geometric_path.getStateCount());
    for (std::size_t index = 0; index < geometric_path.getStateCount(); ++index) {
        path.push_back(list_from_ompl_state(geometric_path.getState(index), dimension));
    }
    return path;
}

}  // namespace

PYBIND11_MODULE(_sbf_cpp, module) {
    module.doc() = "Standalone RBFPlanningForest bindings";
    module.attr("__version__") = "0.1.0";

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
        .def("joint_limits", &rbf::Robot::joint_limits, py::return_value_policy::reference_internal)
        .def("link_radii", &rbf::Robot::link_radii, py::return_value_policy::reference_internal)
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
        py::arg("robot"), py::arg("max_depth"), py::arg("sample_nodes_per_depth") = 8);

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
        py::arg("robot"), py::arg("max_depth"), py::arg("sample_nodes_per_depth") = 8);

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
            if (!rbf::lect_database::LectReadSnapshot::build_from_legacy(legacy_path, snapshot_path_fs, &reason)) {
                throw std::runtime_error(reason.empty() ? "failed to build LECT snapshot from legacy cache" : reason);
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
        .value("RRTConnector", rbf::SegmentEdgeType::RRTConnector)
        .value("PointValidatedGap", rbf::SegmentEdgeType::PointValidatedGap)
        .value("QueryBridge", rbf::SegmentEdgeType::QueryBridge);

    py::enum_<rbf::SegmentEdgeValidation>(module, "SegmentEdgeValidation")
        .value("Unknown", rbf::SegmentEdgeValidation::Unknown)
        .value("CollisionChecked", rbf::SegmentEdgeValidation::CollisionChecked);

    py::enum_<rbf::PathAuditStatus>(module, "PathAuditStatus")
        .value("NotRun", rbf::PathAuditStatus::NotRun)
        .value("Unchecked", rbf::PathAuditStatus::NotRun)
        .value("Passed", rbf::PathAuditStatus::Passed)
        .value("Failed", rbf::PathAuditStatus::Failed)
        .value("Repaired", rbf::PathAuditStatus::Repaired);

    py::class_<rbf::KdopConfig>(module, "KdopConfig")
        .def(py::init<>())
        .def_readwrite("direction_set", &rbf::KdopConfig::direction_set)
        .def_readwrite("safety_epsilon", &rbf::KdopConfig::safety_epsilon);

    py::class_<rbf::SupportHullConfig>(module, "SupportHullConfig")
        .def(py::init<>())
        .def_readwrite("safety_epsilon", &rbf::SupportHullConfig::safety_epsilon);

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
        .def_readwrite("type", &rbf::SegmentEdge::type)
        .def_readwrite("validation", &rbf::SegmentEdge::validation)
        .def_readwrite("segment_resolution", &rbf::SegmentEdge::segment_resolution)
        .def_readwrite("length", &rbf::SegmentEdge::length)
        .def_readwrite("strict_audit_required", &rbf::SegmentEdge::strict_audit_required);

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

    py::class_<rbf::DynamicUpdateConfig>(module, "DynamicUpdateConfig")
        .def(py::init<>())
        .def_readwrite("enable_spatial_dirty_region", &rbf::DynamicUpdateConfig::enable_spatial_dirty_region)
        .def_readwrite("dirty_region_padding", &rbf::DynamicUpdateConfig::dirty_region_padding)
        .def_readwrite("dirty_anchor_limit", &rbf::DynamicUpdateConfig::dirty_anchor_limit)
        .def_readwrite("dirty_seed_limit", &rbf::DynamicUpdateConfig::dirty_seed_limit)
        .def_readwrite("local_regrow_box_limit", &rbf::DynamicUpdateConfig::local_regrow_box_limit)
        .def_readwrite("local_regrow_timeout_ms", &rbf::DynamicUpdateConfig::local_regrow_timeout_ms)
        .def_readwrite("enable_warm_rebuild_fallback", &rbf::DynamicUpdateConfig::enable_warm_rebuild_fallback)
        .def_readwrite("warm_rebuild_on_empty_forest", &rbf::DynamicUpdateConfig::warm_rebuild_on_empty_forest)
        .def_readwrite("warm_rebuild_on_empty_dirty_region", &rbf::DynamicUpdateConfig::warm_rebuild_on_empty_dirty_region)
        .def_readwrite("warm_rebuild_dirty_box_threshold", &rbf::DynamicUpdateConfig::warm_rebuild_dirty_box_threshold)
        .def_readwrite("warm_rebuild_dirty_box_fraction", &rbf::DynamicUpdateConfig::warm_rebuild_dirty_box_fraction)
        .def_readwrite("warm_rebuild_min_local_boxes_added", &rbf::DynamicUpdateConfig::warm_rebuild_min_local_boxes_added);

    py::class_<rbf::SubtractiveObstacleGroup>(module, "SubtractiveObstacleGroup")
        .def(py::init<>())
        .def_readwrite("name", &rbf::SubtractiveObstacleGroup::name)
        .def_readwrite("carving_obstacles", &rbf::SubtractiveObstacleGroup::carving_obstacles)
        .def_readwrite("validation_obstacles", &rbf::SubtractiveObstacleGroup::validation_obstacles);

    py::class_<rbf::SubtractiveBuildOptions>(module, "SubtractiveBuildOptions")
        .def(py::init<>())
        .def_readwrite("run_connector", &rbf::SubtractiveBuildOptions::run_connector)
        .def_readwrite("use_validation_obstacles_for_final_scene", &rbf::SubtractiveBuildOptions::use_validation_obstacles_for_final_scene);

    py::class_<rbf::LeafSweepConfig>(module, "LeafSweepConfig")
        .def(py::init<>())
        .def_readwrite("obstacle_cluster_gap", &rbf::LeafSweepConfig::obstacle_cluster_gap)
        .def_readwrite("n_threads", &rbf::LeafSweepConfig::n_threads)
        .def_readwrite("validation_batch_size", &rbf::LeafSweepConfig::validation_batch_size)
        .def_readwrite("timeout_ms", &rbf::LeafSweepConfig::timeout_ms)
        .def_readwrite("store_group_results", &rbf::LeafSweepConfig::store_group_results)
        .def_readwrite("pre_split_to_max_depth", &rbf::LeafSweepConfig::pre_split_to_max_depth)
        .def_readwrite("use_virtual_topology", &rbf::LeafSweepConfig::use_virtual_topology)
        .def_readwrite("parallel_virtual_validation", &rbf::LeafSweepConfig::parallel_virtual_validation);

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
        .def_readonly("groups", &rbf::LeafSweepResult::groups)
        .def_readonly("obstacle_group_ids", &rbf::LeafSweepResult::obstacle_group_ids)
        .def_readonly("deadline_reached", &rbf::LeafSweepResult::deadline_reached)
        .def_readonly("initialize_ms", &rbf::LeafSweepResult::initialize_ms)
        .def_readonly("group_sweep_ms", &rbf::LeafSweepResult::group_sweep_ms)
        .def_readonly("compose_ms", &rbf::LeafSweepResult::compose_ms)
        .def_readonly("total_ms", &rbf::LeafSweepResult::total_ms)
        .def_readonly("diagnostics", &rbf::LeafSweepResult::diagnostics);

    py::class_<rbf::LeafSweepRefineConfig>(module, "LeafSweepRefineConfig")
        .def(py::init<>())
        .def_readwrite("leaf_start_depth", &rbf::LeafSweepRefineConfig::leaf_start_depth)
        .def_readwrite("leaf_max_depth", &rbf::LeafSweepRefineConfig::leaf_max_depth)
        .def_readwrite("obstacle_cluster_gap", &rbf::LeafSweepRefineConfig::obstacle_cluster_gap)
        .def_readwrite("use_virtual_topology", &rbf::LeafSweepRefineConfig::use_virtual_topology)
        .def_readwrite("parallel_virtual_validation", &rbf::LeafSweepRefineConfig::parallel_virtual_validation)
        .def_readwrite("store_group_results", &rbf::LeafSweepRefineConfig::store_group_results)
        .def_readwrite("validation_batch_size", &rbf::LeafSweepRefineConfig::validation_batch_size)
        .def_readwrite("leaf_threads", &rbf::LeafSweepRefineConfig::leaf_threads)
        .def_readwrite("leaf_timeout_ms", &rbf::LeafSweepRefineConfig::leaf_timeout_ms)
        .def_readwrite("deep_max_boxes", &rbf::LeafSweepRefineConfig::deep_max_boxes)
        .def_readwrite("deep_ffb_depth", &rbf::LeafSweepRefineConfig::deep_ffb_depth)
        .def_readwrite("domain_seed_cap", &rbf::LeafSweepRefineConfig::domain_seed_cap)
        .def_readwrite("domain_success_cap", &rbf::LeafSweepRefineConfig::domain_success_cap)
        .def_readwrite("domain_attempt_cap", &rbf::LeafSweepRefineConfig::domain_attempt_cap)
        .def_readwrite("allow_anchor_roots", &rbf::LeafSweepRefineConfig::allow_anchor_roots)
        .def_readwrite("refine_timeout_ms", &rbf::LeafSweepRefineConfig::refine_timeout_ms);

    py::class_<rbf::LeafSweepRefineResult>(module, "LeafSweepRefineResult")
        .def_readonly("leaf_sweep", &rbf::LeafSweepRefineResult::leaf_sweep)
        .def_readonly("profile", &rbf::LeafSweepRefineResult::profile)
        .def_readonly("leaf_free_count", &rbf::LeafSweepRefineResult::leaf_free_count)
        .def_readonly("leaf_collision_count", &rbf::LeafSweepRefineResult::leaf_collision_count)
        .def_readonly("deep_boxes_added", &rbf::LeafSweepRefineResult::deep_boxes_added)
        .def_readonly("deep_domain_attempts", &rbf::LeafSweepRefineResult::deep_domain_attempts)
        .def_readonly("deep_ffb_success", &rbf::LeafSweepRefineResult::deep_ffb_success)
        .def_readonly("deep_ffb_fail", &rbf::LeafSweepRefineResult::deep_ffb_fail)
        .def_readonly("deep_commit_rejects", &rbf::LeafSweepRefineResult::deep_commit_rejects)
        .def_readonly("deep_domain_rejects", &rbf::LeafSweepRefineResult::deep_domain_rejects)
        .def_readonly("deep_contained_rejects", &rbf::LeafSweepRefineResult::deep_contained_rejects)
        .def_readonly("deep_adjacency_rejects", &rbf::LeafSweepRefineResult::deep_adjacency_rejects)
        .def_readonly("deep_anchor_roots_added", &rbf::LeafSweepRefineResult::deep_anchor_roots_added)
        .def_readonly("leaf_sweep_ms", &rbf::LeafSweepRefineResult::leaf_sweep_ms)
        .def_readonly("deep_refine_ms", &rbf::LeafSweepRefineResult::deep_refine_ms)
        .def_readonly("connector_ms", &rbf::LeafSweepRefineResult::connector_ms)
        .def_readonly("total_ms", &rbf::LeafSweepRefineResult::total_ms)
        .def_readonly("diagnostics", &rbf::LeafSweepRefineResult::diagnostics);

    py::class_<rbf::BestTightenOptions>(module, "BestTightenOptions")
        .def(py::init<>())
        .def_readwrite("depth_synchronous", &rbf::BestTightenOptions::depth_synchronous)
        .def_readwrite("prefer_sector_boundary", &rbf::BestTightenOptions::prefer_sector_boundary)
        .def_readwrite("use_minimax", &rbf::BestTightenOptions::use_minimax)
        .def_readwrite("max_candidate_dim", &rbf::BestTightenOptions::max_candidate_dim)
        .def_readwrite("min_candidate_width", &rbf::BestTightenOptions::min_candidate_width)
        .def_readwrite("width_penalty", &rbf::BestTightenOptions::width_penalty)
        .def_readwrite("shape_balancing", &rbf::BestTightenOptions::shape_balancing)
        .def_readwrite("max_child_aspect", &rbf::BestTightenOptions::max_child_aspect)
        .def_readwrite("min_split_width_fraction", &rbf::BestTightenOptions::min_split_width_fraction)
        .def_readwrite("shape_weight", &rbf::BestTightenOptions::shape_weight)
        .def_readwrite("balance_weight", &rbf::BestTightenOptions::balance_weight)
        .def_readwrite("relative_gain_weight", &rbf::BestTightenOptions::relative_gain_weight)
        .def_readwrite("widest_tiebreak_weight", &rbf::BestTightenOptions::widest_tiebreak_weight)
        .def_readwrite("recent_dim_cooling", &rbf::BestTightenOptions::recent_dim_cooling)
        .def_readwrite("recent_dim_window", &rbf::BestTightenOptions::recent_dim_window)
        .def_readwrite("recent_dim_weight", &rbf::BestTightenOptions::recent_dim_weight)
        .def_readwrite("recent_dim_shape_aspect_trigger", &rbf::BestTightenOptions::recent_dim_shape_aspect_trigger)
        .def_readwrite("dim_mask", &rbf::BestTightenOptions::dim_mask)
        .def_readwrite("dim_priority_weights", &rbf::BestTightenOptions::dim_priority_weights)
        .def_readwrite("dim_priority_weight", &rbf::BestTightenOptions::dim_priority_weight);
    py::class_<rbf::OracleSplitOptions>(module, "OracleSplitOptions")
        .def(py::init<>())
        .def_readwrite("use_best_tighten", &rbf::OracleSplitOptions::use_best_tighten)
        .def_readwrite("best_tighten", &rbf::OracleSplitOptions::best_tighten);

    py::class_<rbf::OracleValidationConfig>(module, "OracleValidationConfig")
        .def(py::init<>())
        .def_readwrite("mode", &rbf::OracleValidationConfig::mode)
        .def_readwrite("accept_unsafe_free", &rbf::OracleValidationConfig::accept_unsafe_free)
        .def_readwrite("enable_validation_cache", &rbf::OracleValidationConfig::enable_validation_cache)
        .def_readwrite("validation_cache_max_entries", &rbf::OracleValidationConfig::validation_cache_max_entries)
        .def_readwrite("enable_endpoint_evidence_cache", &rbf::OracleValidationConfig::enable_endpoint_evidence_cache)
        .def_readwrite("store_endpoint_evidence_cache", &rbf::OracleValidationConfig::store_endpoint_evidence_cache)
        .def_readwrite("endpoint_cache_min_effective_width", &rbf::OracleValidationConfig::endpoint_cache_min_effective_width)
        .def_readwrite("external_evidence_materialization", &rbf::OracleValidationConfig::external_evidence_materialization)
        .def_readwrite("external_evidence_scoring", &rbf::OracleValidationConfig::external_evidence_scoring)
        .def_readwrite("external_evidence_backfill_active", &rbf::OracleValidationConfig::external_evidence_backfill_active)
        .def_readwrite("stateless_materialization_context", &rbf::OracleValidationConfig::stateless_materialization_context)
        .def_readwrite("enable_worker_shared_endpoint_cache", &rbf::OracleValidationConfig::enable_worker_shared_endpoint_cache)
        .def_readwrite("shared_endpoint_cache_max_entries", &rbf::OracleValidationConfig::shared_endpoint_cache_max_entries)
        .def_readwrite("shared_endpoint_cache_max_bytes", &rbf::OracleValidationConfig::shared_endpoint_cache_max_bytes);

    py::class_<rbf::FindFreeBoxOptions>(module, "FindFreeBoxOptions")
        .def(py::init<>())
        .def_readwrite("max_depth", &rbf::FindFreeBoxOptions::max_depth)
        .def_readwrite("skip_to_depth", &rbf::FindFreeBoxOptions::skip_to_depth)
        .def_readwrite("deadline_ms", &rbf::FindFreeBoxOptions::deadline_ms)
        .def_readwrite("split_reserved_leaf", &rbf::FindFreeBoxOptions::split_reserved_leaf)
        .def_readwrite("split_unknown_leaf", &rbf::FindFreeBoxOptions::split_unknown_leaf)
        .def_readwrite("reject_seed_collision", &rbf::FindFreeBoxOptions::reject_seed_collision)
        .def_readwrite("split", &rbf::FindFreeBoxOptions::split);

    py::class_<rbf::GrowerConfig::DepthStage>(module, "GrowerDepthStage")
        .def(py::init<>())
        .def_readwrite("box_limit", &rbf::GrowerConfig::DepthStage::box_limit)
        .def_readwrite("ffb_depth", &rbf::GrowerConfig::DepthStage::ffb_depth)
        .def_readwrite("component_connect_ffb_depth_increment",
                       &rbf::GrowerConfig::DepthStage::component_connect_ffb_depth_increment)
        .def_readwrite("component_connect_ffb_max_depth",
                       &rbf::GrowerConfig::DepthStage::component_connect_ffb_max_depth);

    py::class_<rbf::GrowerConfig>(module, "GrowerConfig")
        .def(py::init<>())
        .def_readwrite("mode", &rbf::GrowerConfig::mode)
        .def_readwrite("find_free_box", &rbf::GrowerConfig::find_free_box)
        .def_readwrite("commit_policy", &rbf::GrowerConfig::commit_policy)
        .def_readwrite("max_boxes", &rbf::GrowerConfig::max_boxes)
        .def_readwrite("timeout_ms", &rbf::GrowerConfig::timeout_ms)
        .def_readwrite("max_consecutive_miss", &rbf::GrowerConfig::max_consecutive_miss)
        .def_readwrite("adjacency_tolerance", &rbf::GrowerConfig::adjacency_tolerance)
        .def_readwrite("rng_seed", &rbf::GrowerConfig::rng_seed)
        .def_readwrite("n_threads", &rbf::GrowerConfig::n_threads)
        .def_readwrite("task_batch_size", &rbf::GrowerConfig::task_batch_size)
        .def_readwrite("parallel_threshold", &rbf::GrowerConfig::parallel_threshold)
        .def_readwrite("deterministic_reduce", &rbf::GrowerConfig::deterministic_reduce)
        .def_readwrite("worker_local_ffb", &rbf::GrowerConfig::worker_local_ffb)
        .def_readwrite("rrt_goal_bias", &rbf::GrowerConfig::rrt_goal_bias)
        .def_readwrite("intertree_goal_bias", &rbf::GrowerConfig::intertree_goal_bias)
        .def_readwrite("sustained_goal_bias_cap", &rbf::GrowerConfig::sustained_goal_bias_cap)
        .def_readwrite("high_goal_bias_pulse_period", &rbf::GrowerConfig::high_goal_bias_pulse_period)
        .def_readwrite("rrt_step_ratio", &rbf::GrowerConfig::rrt_step_ratio)
        .def_readwrite("unexplored_sample_prob", &rbf::GrowerConfig::unexplored_sample_prob)
        .def_readwrite("sample_categorical_allocation", &rbf::GrowerConfig::sample_categorical_allocation)
        .def_readwrite("sample_uniform_prob", &rbf::GrowerConfig::sample_uniform_prob)
        .def_readwrite("expand_all_roots_per_sample", &rbf::GrowerConfig::expand_all_roots_per_sample)
        .def_readwrite("extra_random_roots", &rbf::GrowerConfig::extra_random_roots)
        .def_readwrite("random_anchor_targets", &rbf::GrowerConfig::random_anchor_targets)
        .def_readwrite("anchor_target_prob", &rbf::GrowerConfig::anchor_target_prob)
        .def_readwrite("anchor_target_candidate_count", &rbf::GrowerConfig::anchor_target_candidate_count)
        .def_readwrite("anchor_target_max_lca_depth", &rbf::GrowerConfig::anchor_target_max_lca_depth)
        .def_readwrite("anchor_wave_targets_per_batch", &rbf::GrowerConfig::anchor_wave_targets_per_batch)
        .def("set_fixed_anchor_targets", [](rbf::GrowerConfig& config,
                                            const std::vector<std::vector<double>>& anchors) {
            config.fixed_anchor_targets = eigen_vectors_from_lists(anchors);
        })
        .def_readwrite("root_seed_candidate_count", &rbf::GrowerConfig::root_seed_candidate_count)
        .def_readwrite("root_seed_min_normalized_linf", &rbf::GrowerConfig::root_seed_min_normalized_linf)
        .def_readwrite("root_seed_max_lca_depth", &rbf::GrowerConfig::root_seed_max_lca_depth)
        .def_readwrite("root_seed_include_user_seeds", &rbf::GrowerConfig::root_seed_include_user_seeds)
        .def_readwrite("connect_mode", &rbf::GrowerConfig::connect_mode)
        .def_readwrite("component_connect_prob", &rbf::GrowerConfig::component_connect_prob)
        .def_readwrite("component_connect_max_parent_failures", &rbf::GrowerConfig::component_connect_max_parent_failures)
        .def_readwrite("component_connect_candidate_limit", &rbf::GrowerConfig::component_connect_candidate_limit)
        .def_readwrite("component_connect_island_aware", &rbf::GrowerConfig::component_connect_island_aware)
        .def_readwrite("component_connect_frontier_cache", &rbf::GrowerConfig::component_connect_frontier_cache)
        .def_readwrite("component_connect_staged_growth", &rbf::GrowerConfig::component_connect_staged_growth)
        .def_readwrite("component_connect_stage_normalized_linf", &rbf::GrowerConfig::component_connect_stage_normalized_linf)
        .def_readwrite("component_connect_neighbor_root_bias", &rbf::GrowerConfig::component_connect_neighbor_root_bias)
        .def_readwrite("component_connect_neighbor_root_window", &rbf::GrowerConfig::component_connect_neighbor_root_window)
        .def_readwrite("component_connect_lateral_sample_prob", &rbf::GrowerConfig::component_connect_lateral_sample_prob)
        .def_readwrite("component_connect_lateral_sample_attempts", &rbf::GrowerConfig::component_connect_lateral_sample_attempts)
        .def_readwrite("component_connect_require_target_direction", &rbf::GrowerConfig::component_connect_require_target_direction)
        .def_readwrite("component_connect_adaptive_ffb", &rbf::GrowerConfig::component_connect_adaptive_ffb)
        .def_readwrite("component_connect_ffb_depth_increment", &rbf::GrowerConfig::component_connect_ffb_depth_increment)
        .def_readwrite("component_connect_ffb_max_depth", &rbf::GrowerConfig::component_connect_ffb_max_depth)
        .def_readwrite("component_connect_depth_after_unknown_only", &rbf::GrowerConfig::component_connect_depth_after_unknown_only)
        .def_readwrite("component_connect_chain_steps", &rbf::GrowerConfig::component_connect_chain_steps)
        .def_readwrite("component_connect_chain_max_boxes", &rbf::GrowerConfig::component_connect_chain_max_boxes)
        .def_readwrite("frontier_face_memory", &rbf::GrowerConfig::frontier_face_memory)
        .def_readwrite("frontier_face_bins_per_dim", &rbf::GrowerConfig::frontier_face_bins_per_dim)
        .def_readwrite("frontier_face_min_attempts", &rbf::GrowerConfig::frontier_face_min_attempts)
        .def_readwrite("frontier_face_max_attempts", &rbf::GrowerConfig::frontier_face_max_attempts)
        .def_readwrite("frontier_face_area_attempt_scale", &rbf::GrowerConfig::frontier_face_area_attempt_scale)
        .def_readwrite("frontier_face_candidate_limit", &rbf::GrowerConfig::frontier_face_candidate_limit)
        .def_readwrite("frontwave_bootstrap_boxes", &rbf::GrowerConfig::frontwave_bootstrap_boxes)
        .def_readwrite("frontwave_bootstrap_depth", &rbf::GrowerConfig::frontwave_bootstrap_depth)
        .def_readwrite("frontwave_bootstrap_boundary_samples", &rbf::GrowerConfig::frontwave_bootstrap_boundary_samples)
        .def_readwrite("stop_after_connect", &rbf::GrowerConfig::stop_after_connect)
        .def_readwrite("post_connect_extra_boxes", &rbf::GrowerConfig::post_connect_extra_boxes)
        .def_readwrite("quality_min_connected_boxes", &rbf::GrowerConfig::quality_min_connected_boxes)
        .def_readwrite("post_connect_time_budget_ms", &rbf::GrowerConfig::post_connect_time_budget_ms)
        .def_readwrite("depth_stages", &rbf::GrowerConfig::depth_stages)
        .def_readwrite("failure_cooling_enabled", &rbf::GrowerConfig::failure_cooling_enabled)
        .def_readwrite("failure_cooling_threshold", &rbf::GrowerConfig::failure_cooling_threshold)
        .def_readwrite("failure_cooling_box_horizon", &rbf::GrowerConfig::failure_cooling_box_horizon)
        .def_readwrite("failure_cooling_min_depth", &rbf::GrowerConfig::failure_cooling_min_depth)
        .def_readwrite("failure_cooling_unknown_only", &rbf::GrowerConfig::failure_cooling_unknown_only)
        .def_readwrite("failure_cooling_retry_on_depth_raise", &rbf::GrowerConfig::failure_cooling_retry_on_depth_raise)
        .def_readwrite("coverage_first_stop_loss", &rbf::GrowerConfig::coverage_first_stop_loss)
        .def_readwrite("hard_frontier_failure_threshold", &rbf::GrowerConfig::hard_frontier_failure_threshold)
        .def_readwrite("hard_frontier_box_horizon", &rbf::GrowerConfig::hard_frontier_box_horizon)
        .def_readwrite("boundary_epsilon", &rbf::GrowerConfig::boundary_epsilon)
        .def_readwrite("n_boundary_samples", &rbf::GrowerConfig::n_boundary_samples)
        .def_readwrite("trace_enabled", &rbf::GrowerConfig::trace_enabled)
        .def_readwrite("trace_path", &rbf::GrowerConfig::trace_path)
        .def_readwrite("trace_max_events", &rbf::GrowerConfig::trace_max_events)
        .def_readwrite("trace_face_candidate_limit", &rbf::GrowerConfig::trace_face_candidate_limit);

    py::class_<rbf::MergerConfig>(module, "MergerConfig")
        .def(py::init<>())
        .def_readwrite("exact_face_merge", &rbf::MergerConfig::exact_face_merge)
        .def_readwrite("greedy_hull_merge", &rbf::MergerConfig::greedy_hull_merge)
        .def_readwrite("containment_prune", &rbf::MergerConfig::containment_prune)
        .def_readwrite("max_rounds", &rbf::MergerConfig::max_rounds)
        .def_readwrite("target_boxes", &rbf::MergerConfig::target_boxes)
        .def_readwrite("n_threads", &rbf::MergerConfig::n_threads)
        .def_readwrite("candidate_batch_size", &rbf::MergerConfig::candidate_batch_size)
        .def_readwrite("parallel_threshold", &rbf::MergerConfig::parallel_threshold)
        .def_readwrite("deterministic_reduce", &rbf::MergerConfig::deterministic_reduce)
        .def_readwrite("adjacency_tolerance", &rbf::MergerConfig::adjacency_tolerance)
        .def_readwrite("score_threshold", &rbf::MergerConfig::score_threshold);

    py::class_<rbf::RRTConnectConfig>(module, "RRTConnectConfig")
        .def(py::init<>())
        .def_readwrite("max_iters", &rbf::RRTConnectConfig::max_iters)
        .def_readwrite("step_size", &rbf::RRTConnectConfig::step_size)
        .def_readwrite("goal_bias", &rbf::RRTConnectConfig::goal_bias)
        .def_readwrite("timeout_ms", &rbf::RRTConnectConfig::timeout_ms)
        .def_readwrite("segment_resolution", &rbf::RRTConnectConfig::segment_resolution)
        .def_readwrite("local_sampling_radius", &rbf::RRTConnectConfig::local_sampling_radius)
        .def_readwrite("shortcut_path", &rbf::RRTConnectConfig::shortcut_path)
        .def_readwrite("domain_tolerance", &rbf::RRTConnectConfig::domain_tolerance)
        .def_readwrite("domain_intervals", &rbf::RRTConnectConfig::domain_intervals);

    py::class_<rbf::ChainPaveConfig>(module, "ChainPaveConfig")
        .def(py::init<>())
        .def_readwrite("max_chain", &rbf::ChainPaveConfig::max_chain)
        .def_readwrite("max_steps_per_waypoint", &rbf::ChainPaveConfig::max_steps_per_waypoint)
        .def_readwrite("adjacency_tolerance", &rbf::ChainPaveConfig::adjacency_tolerance)
        .def_readwrite("refine_covered_waypoints", &rbf::ChainPaveConfig::refine_covered_waypoints)
        .def_readwrite("fill_gaps", &rbf::ChainPaveConfig::fill_gaps)
        .def_readwrite("max_gap_fill_depth", &rbf::ChainPaveConfig::max_gap_fill_depth)
        .def_readwrite("gap_fill_min_step", &rbf::ChainPaveConfig::gap_fill_min_step)
        .def_readwrite("gap_fill_sample_step", &rbf::ChainPaveConfig::gap_fill_sample_step)
        .def_readwrite("gap_fill_time_budget_ms", &rbf::ChainPaveConfig::gap_fill_time_budget_ms)
        .def_readwrite("gap_fill_max_ffb_calls", &rbf::ChainPaveConfig::gap_fill_max_ffb_calls)
        .def_readwrite("gap_fill_min_arc_gain", &rbf::ChainPaveConfig::gap_fill_min_arc_gain)
        .def_readwrite("require_connected_chain", &rbf::ChainPaveConfig::require_connected_chain)
        .def_readwrite("find_free_box", &rbf::ChainPaveConfig::find_free_box)
        .def_readwrite("commit_policy", &rbf::ChainPaveConfig::commit_policy);

    py::class_<rbf::IslandConnectorConfig>(module, "IslandConnectorConfig")
        .def(py::init<>())
        .def_readwrite("rrt", &rbf::IslandConnectorConfig::rrt)
        .def_readwrite("pave", &rbf::IslandConnectorConfig::pave)
        .def_readwrite("enable_birrt", &rbf::IslandConnectorConfig::enable_birrt)
        .def_readwrite("frontier_bridge", &rbf::IslandConnectorConfig::frontier_bridge)
        .def_readwrite("frontier_bridge_adaptive_ffb", &rbf::IslandConnectorConfig::frontier_bridge_adaptive_ffb)
        .def_readwrite("frontier_bridge_gap_stall_iterations", &rbf::IslandConnectorConfig::frontier_bridge_gap_stall_iterations)
        .def_readwrite("frontier_bridge_ffb_depth_increment", &rbf::IslandConnectorConfig::frontier_bridge_ffb_depth_increment)
        .def_readwrite("frontier_bridge_ffb_max_depth", &rbf::IslandConnectorConfig::frontier_bridge_ffb_max_depth)
        .def_readwrite("frontier_bridge_candidate_limit", &rbf::IslandConnectorConfig::frontier_bridge_candidate_limit)
        .def_readwrite("frontier_bridge_boundary_epsilon", &rbf::IslandConnectorConfig::frontier_bridge_boundary_epsilon)
        .def_readwrite("per_pair_timeout_ms", &rbf::IslandConnectorConfig::per_pair_timeout_ms)
        .def_readwrite("point_validated_gap_tolerance", &rbf::IslandConnectorConfig::point_validated_gap_tolerance)
        .def_readwrite("point_validated_gap_resolution", &rbf::IslandConnectorConfig::point_validated_gap_resolution)
        .def_readwrite("max_pairs_per_gap", &rbf::IslandConnectorConfig::max_pairs_per_gap)
        .def_readwrite("max_total_bridge_boxes", &rbf::IslandConnectorConfig::max_total_bridge_boxes)
        .def_readwrite("segment_edges_enabled", &rbf::IslandConnectorConfig::segment_edges_enabled)
        .def_readwrite("rrt_segment_edges", &rbf::IslandConnectorConfig::rrt_segment_edges)
        .def_readwrite("point_gap_segment_edges", &rbf::IslandConnectorConfig::point_gap_segment_edges)
        .def_readwrite("segment_edges_fallback_only", &rbf::IslandConnectorConfig::segment_edges_fallback_only)
        .def_readwrite("n_threads", &rbf::IslandConnectorConfig::n_threads)
        .def_readwrite("pair_batch_size", &rbf::IslandConnectorConfig::pair_batch_size)
        .def_readwrite("parallel_threshold", &rbf::IslandConnectorConfig::parallel_threshold)
        .def_readwrite("deterministic_reduce", &rbf::IslandConnectorConfig::deterministic_reduce);

    py::enum_<rbf::lect_database::SplitStrategy>(module, "SplitStrategy")
        .value("RoundRobin", rbf::lect_database::SplitStrategy::RoundRobin)
        .value("WidestRoot", rbf::lect_database::SplitStrategy::WidestRoot)
        .value("AAFKVolumeMin", rbf::lect_database::SplitStrategy::AAFKVolumeMin);

    py::class_<rbf::lect_database::SplitPolicyDescriptor>(module, "SplitPolicyDescriptor")
        .def(py::init<>())
        .def_readwrite("strategy", &rbf::lect_database::SplitPolicyDescriptor::strategy)
        .def_readwrite("min_width", &rbf::lect_database::SplitPolicyDescriptor::min_width)
        .def_readwrite("midpoint", &rbf::lect_database::SplitPolicyDescriptor::midpoint)
        .def_readwrite("deterministic_tie_break", &rbf::lect_database::SplitPolicyDescriptor::deterministic_tie_break)
        .def_readwrite("dimension_schedule_hash", &rbf::lect_database::SplitPolicyDescriptor::dimension_schedule_hash)
        .def_readwrite("depth_dimensions", &rbf::lect_database::SplitPolicyDescriptor::depth_dimensions);

    module.def("split_policy_descriptor", &rbf::lect_database::split_policy_descriptor, py::arg("descriptor"));
    module.def("split_policy_hash", &rbf::lect_database::split_policy_hash, py::arg("descriptor"));
    module.def("stable_hash",
        [](const std::string& text) {
            return rbf::lect_database::stable_hash(text);
        },
        py::arg("text"));

    py::class_<rbf::lect_database::OnlineEnvelopeCacheConfig>(module, "OnlineEnvelopeCacheConfig")
        .def(py::init<>())
        .def_readwrite("max_nodes", &rbf::lect_database::OnlineEnvelopeCacheConfig::max_nodes)
        .def_readwrite("max_payload_bytes", &rbf::lect_database::OnlineEnvelopeCacheConfig::max_payload_bytes)
        .def_readwrite("allow_database_backfill", &rbf::lect_database::OnlineEnvelopeCacheConfig::allow_database_backfill);

    py::class_<rbf::LectDatabaseRuntimeConfig>(module, "LectDatabaseRuntimeConfig")
        .def(py::init<>())
        .def_property("path",
            [](const rbf::LectDatabaseRuntimeConfig& config) { return config.path.string(); },
            [](rbf::LectDatabaseRuntimeConfig& config, const std::string& path) { config.path = path; })
        .def_property("external_evidence_path",
            [](const rbf::LectDatabaseRuntimeConfig& config) { return config.external_evidence_path.string(); },
            [](rbf::LectDatabaseRuntimeConfig& config, const std::string& path) { config.external_evidence_path = path; })
        .def_property("external_evidence_snapshot_path",
            [](const rbf::LectDatabaseRuntimeConfig& config) { return config.external_evidence_snapshot_path.string(); },
            [](rbf::LectDatabaseRuntimeConfig& config, const std::string& path) { config.external_evidence_snapshot_path = path; })
        .def_readwrite("root_intervals_override", &rbf::LectDatabaseRuntimeConfig::root_intervals_override)
        .def_readwrite("split_policy", &rbf::LectDatabaseRuntimeConfig::split_policy)
        .def_readwrite("online_cache", &rbf::LectDatabaseRuntimeConfig::online_cache)
        .def_readwrite("external_evidence_use_snapshot", &rbf::LectDatabaseRuntimeConfig::external_evidence_use_snapshot)
        .def_readwrite("external_evidence_auto_build_snapshot", &rbf::LectDatabaseRuntimeConfig::external_evidence_auto_build_snapshot)
        .def_readwrite("read_only", &rbf::LectDatabaseRuntimeConfig::read_only)
        .def_readwrite("create_if_missing", &rbf::LectDatabaseRuntimeConfig::create_if_missing)
        .def_readwrite("verify_identity", &rbf::LectDatabaseRuntimeConfig::verify_identity)
        .def_readwrite("replay_journal", &rbf::LectDatabaseRuntimeConfig::replay_journal)
        .def_readwrite("propagate_parent_hulls", &rbf::LectDatabaseRuntimeConfig::propagate_parent_hulls)
        .def_readwrite("defer_parent_hull_writes", &rbf::LectDatabaseRuntimeConfig::defer_parent_hull_writes)
        .def_readwrite("canonical_mode", &rbf::LectDatabaseRuntimeConfig::canonical_mode)
        .def_readwrite("checkpoint_after_build", &rbf::LectDatabaseRuntimeConfig::checkpoint_after_build)
        .def_readwrite("symmetry_descriptor", &rbf::LectDatabaseRuntimeConfig::symmetry_descriptor)
        .def_readwrite("page_size_bytes", &rbf::LectDatabaseRuntimeConfig::page_size_bytes)
        .def_readwrite("max_resident_pages", &rbf::LectDatabaseRuntimeConfig::max_resident_pages)
        .def_readwrite("max_tree_depth", &rbf::LectDatabaseRuntimeConfig::max_tree_depth);

    py::class_<rbf::QueryConfig>(module, "QueryConfig")
        .def(py::init<>())
        .def_readwrite("nearest_if_outside", &rbf::QueryConfig::nearest_if_outside)
        .def_readwrite("shortcut_boxes", &rbf::QueryConfig::shortcut_boxes)
        .def_readwrite("collision_shortcut", &rbf::QueryConfig::collision_shortcut)
        .def_readwrite("collision_shortcut_resolution", &rbf::QueryConfig::collision_shortcut_resolution)
        .def_readwrite("strict_path_audit", &rbf::QueryConfig::strict_path_audit)
        .def_readwrite("audit_resolution", &rbf::QueryConfig::audit_resolution)
        .def_readwrite("audit_segment_step", &rbf::QueryConfig::audit_segment_step)
        .def_readwrite("audit_collision_tolerance", &rbf::QueryConfig::audit_collision_tolerance)
        .def_readwrite("repair_on_audit_failure", &rbf::QueryConfig::repair_on_audit_failure)
        .def_readwrite("repair_max_attempts", &rbf::QueryConfig::repair_max_attempts)
        .def_readwrite("repair_rrt_max_iters", &rbf::QueryConfig::repair_rrt_max_iters)
        .def_readwrite("repair_timeout_ms", &rbf::QueryConfig::repair_timeout_ms)
        .def_readwrite("repair_local_sampling_radius", &rbf::QueryConfig::repair_local_sampling_radius)
        .def_readwrite("repair_local_sampling_growth", &rbf::QueryConfig::repair_local_sampling_growth)
        .def_readwrite("adjacency_tolerance", &rbf::QueryConfig::adjacency_tolerance);

    py::class_<rbf::RBFPlanningConfig>(module, "RBFPlanningConfig")
        .def(py::init<>())
        .def_readwrite("grower", &rbf::RBFPlanningConfig::grower)
        .def_readwrite("merger", &rbf::RBFPlanningConfig::merger)
        .def_readwrite("connector", &rbf::RBFPlanningConfig::connector)
        .def_readwrite("query", &rbf::RBFPlanningConfig::query)
        .def_readwrite("endpoint_source", &rbf::RBFPlanningConfig::endpoint_source)
        .def_readwrite("envelope_type", &rbf::RBFPlanningConfig::envelope_type)
        .def_readwrite("validation", &rbf::RBFPlanningConfig::validation)
        .def_readwrite("database", &rbf::RBFPlanningConfig::database)
        .def_readwrite("runtime", &rbf::RBFPlanningConfig::runtime)
        .def_readwrite("dynamic_update", &rbf::RBFPlanningConfig::dynamic_update)
        .def_readwrite("enable_merger", &rbf::RBFPlanningConfig::enable_merger)
        .def_readwrite("enable_connector", &rbf::RBFPlanningConfig::enable_connector);

    py::class_<rbf::BuildProfile>(module, "BuildProfile")
        .def_readonly("total_ms", &rbf::BuildProfile::total_ms)
        .def_readonly("grow_ms", &rbf::BuildProfile::grow_ms)
        .def_readonly("merge_ms", &rbf::BuildProfile::merge_ms)
        .def_readonly("connector_ms", &rbf::BuildProfile::connector_ms)
        .def_readonly("adjacency_ms", &rbf::BuildProfile::adjacency_ms)
        .def_readonly("raw_boxes", &rbf::BuildProfile::raw_boxes)
        .def_readonly("final_boxes", &rbf::BuildProfile::final_boxes)
        .def_readonly("grow_adjacency_islands", &rbf::BuildProfile::grow_adjacency_islands)
        .def_readonly("grow_largest_island", &rbf::BuildProfile::grow_largest_island)
        .def_readonly("bridge_boxes_added", &rbf::BuildProfile::bridge_boxes_added)
        .def_readonly("segment_edges", &rbf::BuildProfile::segment_edges)
        .def_readonly("segment_edges_added", &rbf::BuildProfile::segment_edges_added)
        .def_readonly("rrt_segment_edges_added", &rbf::BuildProfile::rrt_segment_edges_added)
        .def_readonly("point_gap_segment_edges_added", &rbf::BuildProfile::point_gap_segment_edges_added)
        .def_readonly("connector_attempted_pairs", &rbf::BuildProfile::connector_attempted_pairs)
        .def_readonly("connector_connected", &rbf::BuildProfile::connector_connected)
        .def_readonly("adjacency_islands", &rbf::BuildProfile::adjacency_islands)
        .def_readonly("diagnostics", &rbf::BuildProfile::diagnostics);

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
        .def_readonly("used_spatial_dirty_region", &rbf::RebuildProfile::used_spatial_dirty_region)
        .def_readonly("used_warm_rebuild", &rbf::RebuildProfile::used_warm_rebuild)
        .def_readonly("fallback_reason", &rbf::RebuildProfile::fallback_reason)
        .def_readonly("dirty_region_ms", &rbf::RebuildProfile::dirty_region_ms)
        .def_readonly("regrow_ms", &rbf::RebuildProfile::regrow_ms)
        .def_readonly("warm_rebuild_ms", &rbf::RebuildProfile::warm_rebuild_ms)
        .def_readonly("collision_check_ms", &rbf::RebuildProfile::collision_check_ms)
        .def_readonly("adjacency_ms", &rbf::RebuildProfile::adjacency_ms)
        .def_readonly("total_ms", &rbf::RebuildProfile::total_ms);

    py::class_<rbf::QueryResult>(module, "QueryResult")
        .def_readonly("success", &rbf::QueryResult::success)
        .def_readonly("start_box_id", &rbf::QueryResult::start_box_id)
        .def_readonly("goal_box_id", &rbf::QueryResult::goal_box_id)
        .def_readonly("box_sequence", &rbf::QueryResult::box_sequence)
        .def_readonly("segment_edge_sequence", &rbf::QueryResult::segment_edge_sequence)
        .def_readonly("path", &rbf::QueryResult::path)
        .def_readonly("path_length", &rbf::QueryResult::path_length)
        .def_readonly("query_time_ms", &rbf::QueryResult::query_time_ms)
        .def_readonly("segment_edges_used", &rbf::QueryResult::segment_edges_used)
        .def_readonly("audit_status", &rbf::QueryResult::audit_status)
        .def_readonly("audit_passed", &rbf::QueryResult::audit_passed)
        .def_readonly("audit_time_ms", &rbf::QueryResult::audit_time_ms)
        .def_readonly("repair_time_ms", &rbf::QueryResult::repair_time_ms)
        .def_readonly("repair_count", &rbf::QueryResult::repair_count)
        .def_readonly("failed_segment_index", &rbf::QueryResult::failed_segment_index)
        .def_readonly("certified_box_length", &rbf::QueryResult::certified_box_length)
        .def_readonly("provisional_audited_length", &rbf::QueryResult::provisional_audited_length)
        .def_readonly("segment_edge_length", &rbf::QueryResult::segment_edge_length)
        .def_readonly("remaining_unsafe_assumptions", &rbf::QueryResult::remaining_unsafe_assumptions);

    py::class_<rbf::RBFPlanningForest>(module, "RBFPlanningForest")
        .def(py::init<rbf::Robot, rbf::RBFPlanningConfig>(), py::arg("robot"), py::arg("config") = rbf::RBFPlanningConfig{})
        .def("build",
             [](rbf::RBFPlanningForest& forest,
                 const std::vector<double>& start,
                 const std::vector<double>& goal,
                const std::vector<rbf::Obstacle>& obstacles) {
                  return forest.build(eigen_vector_from_list(start), eigen_vector_from_list(goal), obstacles);
             },
             py::arg("start"), py::arg("goal"), py::arg("obstacles") = std::vector<rbf::Obstacle>{})
        .def("build_coverage",
             [](rbf::RBFPlanningForest& forest,
                const std::vector<rbf::Obstacle>& obstacles,
                 const std::vector<std::vector<double>>& seeds) {
                  return forest.build_coverage(obstacles, eigen_vectors_from_lists(seeds));
             },
             py::arg("obstacles"), py::arg("seeds"))
        .def("build_leaf_sweep",
             [](rbf::RBFPlanningForest& forest,
                const std::vector<rbf::Obstacle>& obstacles,
                int start_depth,
                int max_depth,
                const rbf::LeafSweepConfig& config) {
                 return forest.build_leaf_sweep(obstacles, start_depth, max_depth, config);
             },
             py::arg("obstacles"),
             py::arg("start_depth"),
             py::arg("max_depth"),
             py::arg("config") = rbf::LeafSweepConfig{})
        .def("build_leaf_sweep_refined",
             [](rbf::RBFPlanningForest& forest,
                const std::vector<rbf::Obstacle>& obstacles,
                const rbf::LeafSweepRefineConfig& config,
                const std::vector<std::vector<double>>& priority_points) {
                 return forest.build_leaf_sweep_refined(
                     obstacles,
                     config,
                     eigen_vectors_from_lists(priority_points));
             },
             py::arg("obstacles"),
             py::arg("config") = rbf::LeafSweepRefineConfig{},
             py::arg("priority_points") = std::vector<std::vector<double>>{})
        .def("oracle_counters",
             [](const rbf::RBFPlanningForest& forest) {
                 const rbf::OracleCounters* counters = forest.oracle_counters();
                 if (!counters) {
                     return py::dict{};
                 }
                 return oracle_counters_to_python(*counters);
             })
                     .def("build_subtractive",
                             [](rbf::RBFPlanningForest& forest,
                                    const std::vector<rbf::SubtractiveObstacleGroup>& obstacle_groups,
                                    const std::vector<std::vector<double>>& seeds,
                                    const rbf::SubtractiveBuildOptions& options) {
                                     return forest.build_subtractive(obstacle_groups, eigen_vectors_from_lists(seeds), options);
                             },
                             py::arg("obstacle_groups"),
                             py::arg("seeds"),
                             py::arg("options") = rbf::SubtractiveBuildOptions{})
           .def("query",
               [](const rbf::RBFPlanningForest& forest,
                 const std::vector<double>& start,
                 const std::vector<double>& goal) {
                  return forest.query(eigen_vector_from_list(start), eigen_vector_from_list(goal));
               },
               py::arg("start"), py::arg("goal"))
                                .def("refine_query_corridor",
                                                 [](rbf::RBFPlanningForest& forest,
                                                                const std::vector<double>& start,
                                                                const std::vector<double>& goal,
                                                                int max_boxes_to_add,
                                                                const std::string& mode,
                                                                double long_path_ratio,
                                                                double long_path_min_delta) {
                                                                        rbf::CorridorRefineMode refine_mode = rbf::CorridorRefineMode::LegacyBridge;
                                                                        if (mode == "box_only_long_path") {
                                                                            refine_mode = rbf::CorridorRefineMode::BoxOnlyLongPath;
                                                                        } else if (mode != "legacy_bridge") {
                                                                            throw std::invalid_argument("unsupported corridor refine mode: " + mode);
                                                                        }
                                                                        return forest.refine_query_corridor(eigen_vector_from_list(start),
                                                                                                                                             eigen_vector_from_list(goal),
                                                                                                                                             max_boxes_to_add,
                                                                                                                                             refine_mode,
                                                                                                                                             long_path_ratio,
                                                                                                                                             long_path_min_delta);
                                                 },
                                                 py::arg("start"),
                                                 py::arg("goal"),
                                                 py::arg("max_boxes_to_add"),
                                                 py::arg("mode") = "legacy_bridge",
                                                 py::arg("long_path_ratio") = std::numeric_limits<double>::infinity(),
                                                 py::arg("long_path_min_delta") = std::numeric_limits<double>::infinity())
                .def("bridge_query",
                         [](rbf::RBFPlanningForest& forest,
                                const std::vector<double>& start,
                                const std::vector<double>& goal) {
                                    return forest.bridge_query(eigen_vector_from_list(start), eigen_vector_from_list(goal));
                         },
                         py::arg("start"), py::arg("goal"))
                .def("bridge_query_known_needed",
                         [](rbf::RBFPlanningForest& forest,
                                const std::vector<double>& start,
                                const std::vector<double>& goal) {
                                    return forest.bridge_query_known_needed(eigen_vector_from_list(start), eigen_vector_from_list(goal));
                         },
                         py::arg("start"), py::arg("goal"))
                .def("debug_chain_pave",
                         [](rbf::RBFPlanningForest& forest,
                                const std::vector<double>& start,
                                const std::vector<double>& goal,
                                int max_chain,
                                int max_depth,
                                double edge_seed_step,
                                int max_gap_fill_steps,
                                bool fill_segment_gaps,
                                int gap_fill_max_retries,
                                double gap_fill_step_shrink,
                                double gap_fill_min_step,
                                int gap_fill_retry_depth_increment,
                                int gap_fill_max_depth,
                                double adjacency_tolerance,
                                double gap_fill_sample_step,
                                double gap_fill_time_budget_ms,
                                int gap_fill_max_ffb_calls,
                                double gap_fill_min_arc_gain) {
                                    rbf::ChainPaveConfig pave;
                                    pave.commit_policy = rbf::BoxCommitPolicy::CommitProvisionalAllowed;
                                    pave.max_chain = max_chain;
                                    pave.fill_gaps = fill_segment_gaps;
                                    pave.adjacency_tolerance = adjacency_tolerance;
                                    // Recursion depth bounds the bisection subdivisions (2^depth seeds).
                                    // Derive it from the requested seed budget while keeping a sane cap.
                                    pave.max_gap_fill_depth = std::max(1, std::min(20, max_gap_fill_steps));
                                    pave.gap_fill_min_step = gap_fill_min_step;
                                    pave.gap_fill_sample_step = gap_fill_sample_step;
                                    pave.gap_fill_time_budget_ms = gap_fill_time_budget_ms;
                                    pave.gap_fill_max_ffb_calls = gap_fill_max_ffb_calls;
                                    pave.gap_fill_min_arc_gain = gap_fill_min_arc_gain;
                                    pave.find_free_box.max_depth = max_depth;
                                    pave.find_free_box.reject_seed_collision = false;
                                    (void)edge_seed_step;
                                    (void)gap_fill_max_retries;
                                    (void)gap_fill_step_shrink;
                                    (void)gap_fill_retry_depth_increment;
                                    (void)gap_fill_max_depth;
                                    auto res = forest.debug_chain_pave(eigen_vector_from_list(start),
                                                                       eigen_vector_from_list(goal),
                                                                       pave);
                                    py::dict result;
                                    result["added"] = res.added;
                                    result["bridge_found"] = res.bridge_found;
                                    result["audit_passed"] = res.audit_passed;
                                    result["start_box_id"] = res.start_box_id;
                                    result["goal_box_id"] = res.goal_box_id;
                                    result["fast_gap_fill_ffb_calls"] = res.fast_gap_fill_ffb_calls;
                                    result["fast_gap_fill_ms"] = res.fast_gap_fill_ms;
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
                         },
                         py::arg("start"),
                         py::arg("goal"),
                         py::arg("max_chain") = 4096,
                         py::arg("max_depth") = 120,
                         py::arg("edge_seed_step") = 1e-2,
                         py::arg("max_gap_fill_steps") = 64,
                         py::arg("fill_segment_gaps") = true,
                         py::arg("gap_fill_max_retries") = 6,
                         py::arg("gap_fill_step_shrink") = 0.5,
                         py::arg("gap_fill_min_step") = 1e-4,
                         py::arg("gap_fill_retry_depth_increment") = 24,
                         py::arg("gap_fill_max_depth") = 240,
                         py::arg("adjacency_tolerance") = 1e-9,
                         py::arg("gap_fill_sample_step") = 0.05,
                         py::arg("gap_fill_time_budget_ms") = 10.0,
                         py::arg("gap_fill_max_ffb_calls") = 32,
                         py::arg("gap_fill_min_arc_gain") = 0.01)
        .def("add_obstacle_and_rebuild", &rbf::RBFPlanningForest::add_obstacle_and_rebuild, py::arg("obstacle"))
        .def("remove_obstacle_and_regrow", &rbf::RBFPlanningForest::remove_obstacle_and_regrow, py::arg("obstacle_index"))
        .def("remove_obstacle_suffix_and_regrow", &rbf::RBFPlanningForest::remove_obstacle_suffix_and_regrow, py::arg("target_obstacle_count"))
        .def("clear_forest", &rbf::RBFPlanningForest::clear_forest)
        .def("database_node_count", [](const rbf::RBFPlanningForest& forest) {
            return forest.database().node_count();
        })
        .def("database_evidence_count", [](const rbf::RBFPlanningForest& forest) {
            return forest.database().evidence_count();
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
                bool gray_leaf_order) {
                 if (obstacles.empty()) {
                     throw std::invalid_argument("prewarm_lifelong_cache requires a non-empty obstacle scene so endpoint evidence is materialized");
                 }
                 const auto start = std::chrono::steady_clock::now();
                 const int materialize_depth = std::max(0, target_depth);
                 // Progress bar + ETA to stderr (disable with SBF_PREWARM_PROGRESS=0).
                 const char* progress_env = std::getenv("SBF_PREWARM_PROGRESS");
                 const bool show_progress = (progress_env == nullptr || std::string(progress_env) != "0");
                 // Prewarm persistence mode (env-selected):
                 //   default                    -> bulk: all records resident,
                 //       fastest, RAM ~ O(records). Good up to ~D20.
                 //   SBF_PREWARM_STREAMING=1     -> streaming: resident cache is
                 //       capped (SBF_PREWARM_RESIDENT_CAP records, default 2,000,000)
                 //       so peak RAM stays bounded for deep trees (e.g. D25 ~62M
                 //       records). Records are appended to the durable store as
                 //       built; evicted child records are reloaded on demand by the
                 //       bottom-up parent sweep. Output is bit-identical to bulk.
                 //   SBF_DISABLE_BULK_PREWARM=1  -> legacy path (A/B baseline only).
                 const bool legacy_prewarm = std::getenv("SBF_DISABLE_BULK_PREWARM") != nullptr;
                 const bool streaming_prewarm = std::getenv("SBF_PREWARM_STREAMING") != nullptr;
                 std::size_t streaming_cap = 2000000;
                 if (const char* cap_env = std::getenv("SBF_PREWARM_RESIDENT_CAP")) {
                     char* endp = nullptr;
                     const unsigned long long parsed = std::strtoull(cap_env, &endp, 10);
                     if (endp != cap_env && parsed > 0) {
                         streaming_cap = static_cast<std::size_t>(parsed);
                     }
                 }
                 double checkpoint_interval_s = 0.0;
                 if (const char* checkpoint_env = std::getenv("SBF_PREWARM_CHECKPOINT_SECONDS")) {
                     char* endp = nullptr;
                     const double parsed = std::strtod(checkpoint_env, &endp);
                     if (endp != checkpoint_env && parsed > 0.0) {
                         checkpoint_interval_s = parsed;
                     }
                 }
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
                 if (!legacy_prewarm) {
                     if (streaming_prewarm) {
                         forest.database().set_streaming_prewarm_mode(true, streaming_cap);
                     } else {
                         forest.database().set_bulk_prewarm_mode(true, expected_prewarm_records);
                     }
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
                 rbf::DatabaseBoxOracle oracle(forest.robot(),
                                               forest.database(),
                                               rbf::Scene(obstacles),
                                               forest.config().endpoint_source,
                                               forest.config().envelope_type,
                                               forest.config().validation);
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
                 if (!legacy_prewarm && !streaming_prewarm && leaf_layer.size() > expected_leaf_records) {
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
             py::arg("target_depth"), py::arg("obstacles"), py::arg("gray_leaf_order") = true)
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
                 rbf::DatabaseBoxOracle oracle(forest.robot(),
                                               cache,
                                               rbf::Scene(obstacles),
                                               forest.config().endpoint_source,
                                               forest.config().envelope_type,
                                               validation);
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
                 rbf::DatabaseBoxOracle oracle(forest.robot(),
                                               cache,
                                               rbf::Scene(obstacles),
                                               forest.config().endpoint_source,
                                               forest.config().envelope_type,
                                               validation);

                 py::dict result;
                 py::list trace;
                 py::list split_events;
                 py::list validation_events;
                 const auto start = Clock::now();

                 const Eigen::VectorXd tree_seed = oracle.tree_configuration_for_query(seed);
                 result["seed"] = seed_values;
                 result["tree_seed"] = vector_to_list(tree_seed);
                 result["root_intervals"] = interval_pairs_to_python(oracle.root_intervals());
                 result["disable_caches"] = disable_caches;

                 // Seed-independent: canonical split depends only on (robot,
                 // domain). No query-seed coupling is applied to split values.
                 rbf::OracleSplitOptions split_options = options.split;

                 bool seed_in_domain = false;
                 if (tree_seed.size() == oracle.n_dims()) {
                     seed_in_domain = oracle.contains_point(oracle.root_node(), tree_seed);
                 }
                 result["seed_in_domain"] = seed_in_domain;
                 if (tree_seed.size() != oracle.n_dims() || !seed_in_domain) {
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
             py::arg("disable_caches") = true)
        .def("boxes", [](const rbf::RBFPlanningForest& forest) { return forest.boxes(); })
        .def("raw_boxes", [](const rbf::RBFPlanningForest& forest) { return forest.raw_boxes(); })
        .def("audit_robot", &rbf::RBFPlanningForest::audit_robot, py::return_value_policy::reference_internal)
        .def("adjacency", [](const rbf::RBFPlanningForest& forest) { return forest.adjacency(); })
        .def("segment_edges", [](const rbf::RBFPlanningForest& forest) { return forest.segment_edges(); })
        .def("last_build_profile", &rbf::RBFPlanningForest::last_build_profile, py::return_value_policy::reference_internal);

    module.def("path_length", [](const std::vector<std::vector<double>>& path) {
        return rbf::path_length(eigen_vectors_from_lists(path));
    });

    module.def("rrt_connect_path",
        [](const rbf::Robot& robot,
           const std::vector<rbf::Obstacle>& obstacles,
           const std::vector<double>& start,
           const std::vector<double>& goal,
           const rbf::RRTConnectConfig& config,
           int seed) {
            rbf::CollisionChecker checker(robot, rbf::Scene(obstacles));
            auto path = rbf::rrt_connect(eigen_vector_from_list(start),
                                         eigen_vector_from_list(goal),
                                         checker,
                                         robot,
                                         config,
                                         seed);
            std::vector<std::vector<double>> result;
            result.reserve(path.size());
            for (const auto& waypoint : path) {
                result.emplace_back(waypoint.data(), waypoint.data() + waypoint.size());
            }
            return result;
        },
        py::arg("robot"),
        py::arg("obstacles"),
        py::arg("start"),
        py::arg("goal"),
        py::arg("config") = rbf::RRTConnectConfig{},
        py::arg("seed") = 42);

    module.def("ompl_rrt_connect_path",
        [](const rbf::Robot& robot,
           const std::vector<rbf::Obstacle>& obstacles,
           const std::vector<double>& start,
           const std::vector<double>& goal,
           double timeout_ms,
           double range,
           double segment_step,
           double simplify_time_s,
           int seed) {
            namespace ob = ompl::base;
            namespace og = ompl::geometric;
            ompl::msg::setLogLevel(ompl::msg::LOG_ERROR);
            py::dict result;
            const int dimension = robot.n_joints();
            if (static_cast<int>(start.size()) != dimension || static_cast<int>(goal.size()) != dimension) {
                throw std::invalid_argument("start/goal dimension must match robot.n_joints()");
            }
            const auto t0 = std::chrono::steady_clock::now();
            auto elapsed_s = [&]() {
                return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            };
            auto checker = std::make_shared<rbf::CollisionChecker>(robot, rbf::Scene(obstacles));
            const Eigen::VectorXd q_start_vec = eigen_vector_from_list(start);
            const Eigen::VectorXd q_goal_vec = eigen_vector_from_list(goal);
            if (checker->check_config(q_start_vec) || checker->check_config(q_goal_vec)) {
                result["ok"] = false;
                result["reason"] = "endpoint_collision";
                result["status"] = "endpoint_collision";
                result["t_s"] = elapsed_s();
                result["path"] = std::vector<std::vector<double>>{};
                result["nodes"] = 0;
                return result;
            }

            const auto base_seed = normalize_seed(seed);
            auto space = std::make_shared<ob::RealVectorStateSpace>(dimension);
            ob::RealVectorBounds bounds(dimension);
            const auto& limits = robot.joint_limits().limits;
            if (static_cast<int>(limits.size()) != dimension) {
                throw std::invalid_argument("robot joint limits must match robot.n_joints()");
            }
            for (int dim = 0; dim < dimension; ++dim) {
                bounds.setLow(dim, limits[static_cast<std::size_t>(dim)].lo);
                bounds.setHigh(dim, limits[static_cast<std::size_t>(dim)].hi);
            }
            space->setBounds(bounds);
            configure_deterministic_state_sampler(space, mix_seed(base_seed, 0x52525453U));
            og::SimpleSetup setup(space);
            setup.setStateValidityChecker([checker, dimension](const ob::State* state) {
                const auto* vector_state = state->as<ob::RealVectorStateSpace::StateType>();
                Eigen::VectorXd q(dimension);
                for (int dim = 0; dim < dimension; ++dim) {
                    q[dim] = vector_state->values[dim];
                }
                return !checker->check_config(q);
            });
            const double maximum_extent = std::max(space->getMaximumExtent(), 1e-9);
            const double checking_resolution = std::clamp(std::max(segment_step, 1e-6) / maximum_extent, 1e-6, 0.05);
            setup.getSpaceInformation()->setStateValidityCheckingResolution(checking_resolution);

            ob::ScopedState<ob::RealVectorStateSpace> q_start(space), q_goal(space);
            for (int dim = 0; dim < dimension; ++dim) {
                q_start->values[dim] = start[static_cast<std::size_t>(dim)];
                q_goal->values[dim] = goal[static_cast<std::size_t>(dim)];
            }
            setup.setStartAndGoalStates(q_start, q_goal);
            auto planner = std::make_shared<SeededRRTConnect>(setup.getSpaceInformation());
            planner->setLocalSeed(mix_seed(base_seed, 0x52525450U));
            if (range > 0.0) {
                planner->setRange(range);
            }
            setup.setPlanner(planner);

            const double timeout_s = std::max(0.0, timeout_ms) / 1000.0;
            const double ptc_interval_s = std::max(1e-3, std::min(0.05, timeout_s / 20.0));
            ob::PlannerStatus status = setup.solve(ob::timedPlannerTerminationCondition(timeout_s, ptc_interval_s));
            const bool exact_solution = status == ob::PlannerStatus::EXACT_SOLUTION;
            bool ok = exact_solution;
            if (ok && simplify_time_s > 0.0) {
                setup.simplifySolution(simplify_time_s);
            }
            std::vector<std::vector<double>> path;
            if (ok && setup.haveSolutionPath()) {
                auto& solution_path = setup.getSolutionPath();
                path.reserve(solution_path.getStateCount());
                for (std::size_t index = 0; index < solution_path.getStateCount(); ++index) {
                    path.push_back(list_from_ompl_state(solution_path.getState(index), dimension));
                }
            } else {
                ok = false;
            }
            ob::PlannerData planner_data(setup.getSpaceInformation());
            planner->getPlannerData(planner_data);
            result["ok"] = ok;
            result["reason"] = ok ? "connected" : std::string(status.asString());
            result["status"] = std::string(status.asString());
            result["exact_solution"] = exact_solution;
            result["t_s"] = elapsed_s();
            result["path"] = path;
            result["nodes"] = static_cast<int>(planner_data.numVertices());
            result["checking_resolution"] = checking_resolution;
            result["planner"] = "OMPL_RRTConnect";
            return result;
        },
        py::arg("robot"),
        py::arg("obstacles"),
        py::arg("start"),
        py::arg("goal"),
        py::arg("timeout_ms") = 1000.0,
        py::arg("range") = 0.35,
        py::arg("segment_step") = 0.05,
        py::arg("simplify_time_s") = 0.05,
        py::arg("seed") = 42);

    module.def("ompl_prm_multiquery",
        [](const rbf::Robot& robot,
           const std::vector<rbf::Obstacle>& obstacles,
           const std::vector<std::vector<double>>& starts,
           const std::vector<std::vector<double>>& goals,
           double build_budget_s,
           double query_budget_s,
           double segment_step,
           double simplify_time_s,
           int seed,
           int max_nearest_neighbors) {
            namespace ob = ompl::base;
            namespace og = ompl::geometric;
            ompl::msg::setLogLevel(ompl::msg::LOG_ERROR);
            const auto base_seed = normalize_seed(seed);
            py::dict result;
            const int dimension = robot.n_joints();
            if (starts.empty() || starts.size() != goals.size()) {
                throw std::invalid_argument("starts/goals must be non-empty and have the same length");
            }
            for (std::size_t index = 0; index < starts.size(); ++index) {
                if (static_cast<int>(starts[index].size()) != dimension || static_cast<int>(goals[index].size()) != dimension) {
                    throw std::invalid_argument("all start/goal dimensions must match robot.n_joints()");
                }
            }
            auto checker = std::make_shared<rbf::CollisionChecker>(robot, rbf::Scene(obstacles));
            auto endpoint_collision = [&](const std::vector<double>& q) {
                return checker->check_config(eigen_vector_from_list(q));
            };
            double checking_resolution = 0.0;
            auto space = make_ompl_space(robot);
            configure_deterministic_state_sampler(space, mix_seed(base_seed, 0x50524D53U));
            auto si = make_ompl_space_information(robot, obstacles, space, segment_step, checking_resolution);
            auto problem = std::make_shared<ob::ProblemDefinition>(si);
            set_problem_query(problem, space, si, starts.front(), goals.front(), dimension);
            auto planner = std::make_shared<SeededPRM>(si);
            planner->setLocalSeed(mix_seed(base_seed, 0x50524D50U));
            if (max_nearest_neighbors > 0) {
                planner->setMaxNearestNeighbors(static_cast<unsigned int>(max_nearest_neighbors));
            }
            planner->setProblemDefinition(problem);
            planner->setup();

            const auto build_start = std::chrono::steady_clock::now();
            planner->constructRoadmap(ob::timedPlannerTerminationCondition(std::max(0.0, build_budget_s)));
            const double build_s = ompl_elapsed_s(build_start);

            py::list query_results;
            for (std::size_t index = 0; index < starts.size(); ++index) {
                py::dict row;
                row["index"] = static_cast<int>(index);
                if (endpoint_collision(starts[index]) || endpoint_collision(goals[index])) {
                    row["ok"] = false;
                    row["reason"] = "endpoint_collision";
                    row["status"] = "endpoint_collision";
                    row["t_s"] = 0.0;
                    row["path"] = std::vector<std::vector<double>>{};
                    query_results.append(row);
                    continue;
                }
                planner->clearQuery();
                set_problem_query(problem, space, si, starts[index], goals[index], dimension);
                const auto query_start = std::chrono::steady_clock::now();
                ob::PlannerStatus status = planner->solve(ob::timedPlannerTerminationCondition(std::max(0.0, query_budget_s)));
                bool ok = status == ob::PlannerStatus::EXACT_SOLUTION || static_cast<bool>(status);
                double query_s = ompl_elapsed_s(query_start);
                if (ok && simplify_time_s > 0.0) {
                    auto path_ptr = problem->getSolutionPath();
                    auto geometric_path = std::dynamic_pointer_cast<og::PathGeometric>(path_ptr);
                    if (geometric_path) {
                        const auto simplify_start = std::chrono::steady_clock::now();
                        og::PathSimplifier simplifier(si);
                        simplifier.simplify(*geometric_path, std::max(0.0, simplify_time_s));
                        query_s += ompl_elapsed_s(simplify_start);
                    }
                }
                auto path = ok ? path_from_problem_solution(problem, dimension) : std::vector<std::vector<double>>{};
                if (path.size() < 2) {
                    ok = false;
                }
                row["ok"] = ok;
                row["reason"] = ok ? "connected" : std::string(status.asString());
                row["status"] = std::string(status.asString());
                row["t_s"] = query_s;
                row["path"] = path;
                query_results.append(row);
            }
            ob::PlannerData planner_data(si);
            planner->getPlannerData(planner_data);
            result["ok"] = true;
            result["planner"] = "OMPL_PRM";
            result["build_s"] = build_s;
            result["nodes"] = static_cast<int>(planner_data.numVertices());
            result["checking_resolution"] = checking_resolution;
            result["queries"] = query_results;
            return result;
        },
        py::arg("robot"),
        py::arg("obstacles"),
        py::arg("starts"),
        py::arg("goals"),
        py::arg("build_budget_s") = 10.0,
        py::arg("query_budget_s") = 2.0,
        py::arg("segment_step") = 0.05,
        py::arg("simplify_time_s") = 0.1,
        py::arg("seed") = 42,
        py::arg("max_nearest_neighbors") = 32);

    module.def("ompl_simplify_path",
        [](const rbf::Robot& robot,
           const std::vector<rbf::Obstacle>& obstacles,
           const std::vector<std::vector<double>>& waypoints,
           double segment_step,
           double simplify_time_s) {
            namespace ob = ompl::base;
            namespace og = ompl::geometric;

            py::dict result;
            result["ok"] = false;
            result["t_s"] = 0.0;
            result["path"] = std::vector<std::vector<double>>{};

            const int dimension = robot.n_joints();
            if (waypoints.size() < 2) {
                result["reason"] = "path_too_short";
                return result;
            }
            for (const auto& waypoint : waypoints) {
                if (static_cast<int>(waypoint.size()) != dimension) {
                    throw std::invalid_argument("ompl_simplify_path waypoint dimension must match robot.n_joints()");
                }
            }

            auto space = make_ompl_space(robot);
            double checking_resolution = 0.0;
            auto si = make_ompl_space_information(robot, obstacles, space, segment_step, checking_resolution);

            og::PathGeometric geometric_path(si);
            for (const auto& waypoint : waypoints) {
                ob::ScopedState<ob::RealVectorStateSpace> state(space);
                for (int dim = 0; dim < dimension; ++dim) {
                    state->values[dim] = waypoint[static_cast<std::size_t>(dim)];
                }
                geometric_path.append(state.get());
            }

            const auto simplify_start = std::chrono::steady_clock::now();
            if (simplify_time_s > 0.0) {
                og::PathSimplifier simplifier(si);
                simplifier.simplify(geometric_path, std::max(0.0, simplify_time_s));
            }
            result["ok"] = geometric_path.getStateCount() >= 2;
            result["t_s"] = ompl_elapsed_s(simplify_start);
            result["checking_resolution"] = checking_resolution;
            result["path"] = path_from_geometric_path(geometric_path, dimension);
            result["reason"] = (geometric_path.getStateCount() >= 2) ? "simplified" : "simplify_failed";
            return result;
        },
        py::arg("robot"),
        py::arg("obstacles"),
        py::arg("waypoints"),
        py::arg("segment_step"),
        py::arg("simplify_time_s") = 0.05);

    module.def("ompl_bitstar_path",
        [](const rbf::Robot& robot,
           const std::vector<rbf::Obstacle>& obstacles,
           const std::vector<double>& start,
           const std::vector<double>& goal,
           double timeout_ms,
           double segment_step,
           double simplify_time_s,
           int seed,
           int samples_per_batch,
           double rewire_factor,
           bool stop_on_solution_improvement) {
            namespace ob = ompl::base;
            namespace og = ompl::geometric;
            ompl::msg::setLogLevel(ompl::msg::LOG_ERROR);
            const auto base_seed = normalize_seed(seed);
            py::dict result;
            const int dimension = robot.n_joints();
            if (static_cast<int>(start.size()) != dimension || static_cast<int>(goal.size()) != dimension) {
                throw std::invalid_argument("start/goal dimension must match robot.n_joints()");
            }
            const auto t0 = std::chrono::steady_clock::now();
            auto checker = std::make_shared<rbf::CollisionChecker>(robot, rbf::Scene(obstacles));
            if (checker->check_config(eigen_vector_from_list(start)) || checker->check_config(eigen_vector_from_list(goal))) {
                result["ok"] = false;
                result["reason"] = "endpoint_collision";
                result["status"] = "endpoint_collision";
                result["t_s"] = ompl_elapsed_s(t0);
                result["path"] = std::vector<std::vector<double>>{};
                result["nodes"] = 0;
                return result;
            }
            auto space = make_ompl_space(robot);
            configure_deterministic_state_sampler(space, mix_seed(base_seed, 0x42495453U));
            double checking_resolution = 0.0;
            og::SimpleSetup setup(space);
            auto si = setup.getSpaceInformation();
            si->setStateValidityChecker([checker, dimension](const ob::State* state) {
                const auto* vector_state = state->as<ob::RealVectorStateSpace::StateType>();
                Eigen::VectorXd q(dimension);
                for (int dim = 0; dim < dimension; ++dim) {
                    q[dim] = vector_state->values[dim];
                }
                return !checker->check_config(q);
            });
            const double maximum_extent = std::max(space->getMaximumExtent(), 1e-9);
            checking_resolution = std::clamp(std::max(segment_step, 1e-6) / maximum_extent, 1e-6, 0.05);
            si->setStateValidityCheckingResolution(checking_resolution);

            ob::ScopedState<ob::RealVectorStateSpace> q_start(space), q_goal(space);
            for (int dim = 0; dim < dimension; ++dim) {
                q_start->values[dim] = start[static_cast<std::size_t>(dim)];
                q_goal->values[dim] = goal[static_cast<std::size_t>(dim)];
            }
            setup.setStartAndGoalStates(q_start, q_goal);
            setup.setOptimizationObjective(std::make_shared<ob::PathLengthOptimizationObjective>(si));
            auto planner = std::make_shared<SeededBITstar>(si);
            planner->setLocalSeed(mix_seed(base_seed, 0x42495450U));
            if (samples_per_batch > 0) {
                planner->setSamplesPerBatch(static_cast<unsigned int>(samples_per_batch));
            }
            if (rewire_factor > 0.0) {
                planner->setRewireFactor(rewire_factor);
            }
            planner->setStopOnSolnImprovement(stop_on_solution_improvement);
            setup.setPlanner(planner);
            const double timeout_s = std::max(0.0, timeout_ms) / 1000.0;
            const double ptc_interval_s = std::max(1e-3, std::min(0.05, timeout_s / 20.0));
            ob::PlannerStatus status = setup.solve(ob::timedPlannerTerminationCondition(timeout_s, ptc_interval_s));
            bool ok = status == ob::PlannerStatus::EXACT_SOLUTION || static_cast<bool>(status);
            if (ok && simplify_time_s > 0.0) {
                setup.simplifySolution(simplify_time_s);
            }
            std::vector<std::vector<double>> path;
            if (ok && setup.haveSolutionPath()) {
                auto& solution_path = setup.getSolutionPath();
                path.reserve(solution_path.getStateCount());
                for (std::size_t index = 0; index < solution_path.getStateCount(); ++index) {
                    path.push_back(list_from_ompl_state(solution_path.getState(index), dimension));
                }
            }
            if (path.size() < 2) {
                ok = false;
            }
            const double elapsed_before_diagnostics_s = ompl_elapsed_s(t0);
            result["ok"] = ok;
            result["reason"] = ok ? "connected" : std::string(status.asString());
            result["status"] = std::string(status.asString());
            result["exact_solution"] = status == ob::PlannerStatus::EXACT_SOLUTION;
            result["t_s"] = elapsed_before_diagnostics_s;
            result["path"] = path;
            result["nodes"] = 0;
            result["checking_resolution"] = checking_resolution;
            result["planner"] = "OMPL_BITstar";
            result["iterations"] = static_cast<int>(planner->numIterations());
            result["batches"] = static_cast<int>(planner->numBatches());
            return result;
        },
        py::arg("robot"),
        py::arg("obstacles"),
        py::arg("start"),
        py::arg("goal"),
        py::arg("timeout_ms") = 10000.0,
        py::arg("segment_step") = 0.05,
        py::arg("simplify_time_s") = 0.1,
        py::arg("seed") = 42,
        py::arg("samples_per_batch") = -1,
        py::arg("rewire_factor") = -1.0,
        py::arg("stop_on_solution_improvement") = true);

    module.def("ompl_bitstar_trace",
        [](const rbf::Robot& robot,
           const std::vector<rbf::Obstacle>& obstacles,
           const std::vector<double>& start,
           const std::vector<double>& goal,
           double timeout_ms,
           double checkpoint_interval_ms,
           double segment_step,
           int seed,
           int samples_per_batch,
           double rewire_factor,
           bool stop_on_solution_improvement) {
            namespace ob = ompl::base;
            namespace og = ompl::geometric;
            ompl::msg::setLogLevel(ompl::msg::LOG_ERROR);
            const auto base_seed = normalize_seed(seed);
            py::dict result;
            py::list checkpoints;
            const int dimension = robot.n_joints();
            if (static_cast<int>(start.size()) != dimension || static_cast<int>(goal.size()) != dimension) {
                throw std::invalid_argument("start/goal dimension must match robot.n_joints()");
            }
            const auto t0 = std::chrono::steady_clock::now();
            auto checker = std::make_shared<rbf::CollisionChecker>(robot, rbf::Scene(obstacles));
            if (checker->check_config(eigen_vector_from_list(start)) || checker->check_config(eigen_vector_from_list(goal))) {
                result["ok"] = false;
                result["reason"] = "endpoint_collision";
                result["status"] = "endpoint_collision";
                result["t_s"] = ompl_elapsed_s(t0);
                result["path"] = std::vector<std::vector<double>>{};
                result["nodes"] = 0;
                result["checkpoints"] = checkpoints;
                return result;
            }

            auto space = make_ompl_space(robot);
            configure_deterministic_state_sampler(space, mix_seed(base_seed, 0x42495453U));
            double checking_resolution = 0.0;
            og::SimpleSetup setup(space);
            auto si = setup.getSpaceInformation();
            si->setStateValidityChecker([checker, dimension](const ob::State* state) {
                const auto* vector_state = state->as<ob::RealVectorStateSpace::StateType>();
                Eigen::VectorXd q(dimension);
                for (int dim = 0; dim < dimension; ++dim) {
                    q[dim] = vector_state->values[dim];
                }
                return !checker->check_config(q);
            });
            const double maximum_extent = std::max(space->getMaximumExtent(), 1e-9);
            checking_resolution = std::clamp(std::max(segment_step, 1e-6) / maximum_extent, 1e-6, 0.05);
            si->setStateValidityCheckingResolution(checking_resolution);

            ob::ScopedState<ob::RealVectorStateSpace> q_start(space), q_goal(space);
            for (int dim = 0; dim < dimension; ++dim) {
                q_start->values[dim] = start[static_cast<std::size_t>(dim)];
                q_goal->values[dim] = goal[static_cast<std::size_t>(dim)];
            }
            setup.setStartAndGoalStates(q_start, q_goal);
            setup.setOptimizationObjective(std::make_shared<ob::PathLengthOptimizationObjective>(si));
            auto planner = std::make_shared<SeededBITstar>(si);
            planner->setLocalSeed(mix_seed(base_seed, 0x42495450U));
            if (samples_per_batch > 0) {
                planner->setSamplesPerBatch(static_cast<unsigned int>(samples_per_batch));
            }
            if (rewire_factor > 0.0) {
                planner->setRewireFactor(rewire_factor);
            }
            planner->setStopOnSolnImprovement(stop_on_solution_improvement);
            setup.setPlanner(planner);

            const double timeout_s = std::max(0.0, timeout_ms) / 1000.0;
            const double interval_s = std::max(1e-3, checkpoint_interval_ms / 1000.0);
            std::string last_status = "not_run";
            bool last_exact = false;

            auto append_checkpoint = [&](double target_s) {
                const double elapsed_s = ompl_elapsed_s(t0);
                std::vector<std::vector<double>> path;
                bool ok = false;
                if (setup.haveSolutionPath()) {
                    path = path_from_problem_solution(setup.getProblemDefinition(), dimension);
                    ok = path.size() >= 2;
                }
                py::dict row;
                row["checkpoint_s"] = target_s;
                row["elapsed_s"] = elapsed_s;
                row["ok"] = ok;
                row["reason"] = ok ? "connected" : last_status;
                row["status"] = ok ? "solution" : last_status;
                row["exact_solution"] = ok || last_exact;
                row["t_s"] = elapsed_s;
                row["path"] = path;
                row["nodes"] = 0;
                row["iterations"] = static_cast<int>(planner->numIterations());
                row["batches"] = static_cast<int>(planner->numBatches());
                checkpoints.append(row);
            };

            if (timeout_s <= 0.0) {
                append_checkpoint(0.0);
            } else {
                for (double target_s = interval_s; target_s < timeout_s - 1e-9; target_s += interval_s) {
                    const double clamped_target_s = std::min(target_s, timeout_s);
                    while (ompl_elapsed_s(t0) < clamped_target_s - 1e-6) {
                        const double remaining_s = std::max(1e-5, clamped_target_s - ompl_elapsed_s(t0));
                        const double ptc_interval_s = std::max(1e-3, std::min(0.05, remaining_s / 20.0));
                        const auto before_s = ompl_elapsed_s(t0);
                        ob::PlannerStatus status = setup.solve(ob::timedPlannerTerminationCondition(remaining_s, ptc_interval_s));
                        last_status = std::string(status.asString());
                        last_exact = status == ob::PlannerStatus::EXACT_SOLUTION;
                        if (ompl_elapsed_s(t0) <= before_s + 1e-6) {
                            break;
                        }
                    }
                    append_checkpoint(clamped_target_s);
                }
                while (ompl_elapsed_s(t0) < timeout_s - 1e-6) {
                    const double remaining_s = std::max(1e-5, timeout_s - ompl_elapsed_s(t0));
                    const double ptc_interval_s = std::max(1e-3, std::min(0.05, remaining_s / 20.0));
                    const auto before_s = ompl_elapsed_s(t0);
                    ob::PlannerStatus status = setup.solve(ob::timedPlannerTerminationCondition(remaining_s, ptc_interval_s));
                    last_status = std::string(status.asString());
                    last_exact = status == ob::PlannerStatus::EXACT_SOLUTION;
                    if (ompl_elapsed_s(t0) <= before_s + 1e-6) {
                        break;
                    }
                }
                append_checkpoint(timeout_s);
            }

            std::vector<std::vector<double>> final_path;
            bool ok = false;
            if (setup.haveSolutionPath()) {
                final_path = path_from_problem_solution(setup.getProblemDefinition(), dimension);
                ok = final_path.size() >= 2;
            }
            result["ok"] = ok;
            result["reason"] = ok ? "connected" : last_status;
            result["status"] = ok ? "solution" : last_status;
            result["exact_solution"] = ok || last_exact;
            result["t_s"] = ompl_elapsed_s(t0);
            result["path"] = final_path;
            result["nodes"] = 0;
            result["checking_resolution"] = checking_resolution;
            result["planner"] = "OMPL_BITstar";
            result["iterations"] = static_cast<int>(planner->numIterations());
            result["batches"] = static_cast<int>(planner->numBatches());
            result["checkpoints"] = checkpoints;
            return result;
        },
        py::arg("robot"),
        py::arg("obstacles"),
        py::arg("start"),
        py::arg("goal"),
        py::arg("timeout_ms") = 10000.0,
        py::arg("checkpoint_interval_ms") = 1000.0,
        py::arg("segment_step") = 0.05,
        py::arg("seed") = 42,
        py::arg("samples_per_batch") = -1,
        py::arg("rewire_factor") = -1.0,
        py::arg("stop_on_solution_improvement") = false);

    module.def("check_config_collision",
        [](const rbf::Robot& robot,
           const std::vector<rbf::Obstacle>& obstacles,
           const std::vector<double>& q,
           double collision_tolerance) {
            rbf::CollisionChecker checker(robot, rbf::Scene(obstacles));
            checker.set_collision_tolerance(collision_tolerance);
            return checker.check_config(eigen_vector_from_list(q));
        },
        py::arg("robot"),
        py::arg("obstacles"),
        py::arg("q"),
        py::arg("collision_tolerance") = 0.0);
}
