#include <SBF/sbf.h>

#include "binding_utils.h"

#include <cstdio>
#include <cstdlib>

#include <rbf/lect_database/read_snapshot.h>
#include <link_interval_envelope/batch.h>
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
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/LazyPRM.h>
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
#include <thread>
#include <utility>
#include <vector>

namespace py = pybind11;

namespace {

using namespace rbf::python_binding;

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

    Vertex addMilestoneFromState(const ompl::base::State* state) {
        return addMilestone(si_->cloneState(state));
    }
};

class SeededPRMstar final : public ompl::geometric::PRMstar {
public:
    using ompl::geometric::PRMstar::PRMstar;

    void setLocalSeed(std::uint_fast32_t local_seed) {
        rng_.setLocalSeed(local_seed);
    }

    Vertex addMilestoneFromState(const ompl::base::State* state) {
        return addMilestone(si_->cloneState(state));
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

    void setInitialInflationFactorPublic(double factor) {
        setInitialInflationFactor(factor);
    }

    void setInflationScalingParameterPublic(double parameter) {
        setInflationScalingParameter(parameter);
    }

    void setTruncationScalingParameterPublic(double parameter) {
        setTruncationScalingParameter(parameter);
    }

    void enableCascadingRewiringsPublic(bool enable) {
        enableCascadingRewirings(enable);
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

void configure_bitstar_planner(const std::shared_ptr<SeededBITstar>& planner,
                               int samples_per_batch,
                               double rewire_factor,
                               bool stop_on_solution_improvement,
                               int use_k_nearest,
                               int pruning,
                               double prune_threshold_fraction,
                               int delay_rewiring_until_initial_solution,
                               int just_in_time_sampling,
                               int drop_samples_on_prune,
                               int approximate_solutions,
                               int strict_queue_ordering,
                               int cascading_rewirings,
                               double initial_inflation_factor,
                               double inflation_scaling_parameter,
                               double truncation_scaling_parameter,
                               int allowed_failed_sampling_attempts) {
    if (samples_per_batch > 0) {
        planner->setSamplesPerBatch(static_cast<unsigned int>(samples_per_batch));
    }
    if (rewire_factor > 0.0) {
        planner->setRewireFactor(rewire_factor);
    }
    planner->setStopOnSolnImprovement(stop_on_solution_improvement);
    if (use_k_nearest >= 0) {
        planner->setUseKNearest(use_k_nearest != 0);
    }
    if (pruning >= 0) {
        planner->setPruning(pruning != 0);
    }
    if (prune_threshold_fraction >= 0.0) {
        planner->setPruneThresholdFraction(prune_threshold_fraction);
    }
    if (delay_rewiring_until_initial_solution >= 0) {
        planner->setDelayRewiringUntilInitialSolution(delay_rewiring_until_initial_solution != 0);
    }
    if (just_in_time_sampling >= 0) {
        planner->setJustInTimeSampling(just_in_time_sampling != 0);
    }
    if (drop_samples_on_prune >= 0) {
        planner->setDropSamplesOnPrune(drop_samples_on_prune != 0);
    }
    if (approximate_solutions >= 0) {
        planner->setConsiderApproximateSolutions(approximate_solutions != 0);
    }
    if (strict_queue_ordering >= 0) {
        planner->setStrictQueueOrdering(strict_queue_ordering != 0);
    }
    if (cascading_rewirings >= 0) {
        planner->enableCascadingRewiringsPublic(cascading_rewirings != 0);
    }
    if (initial_inflation_factor > 0.0) {
        planner->setInitialInflationFactorPublic(initial_inflation_factor);
    }
    if (inflation_scaling_parameter >= 0.0) {
        planner->setInflationScalingParameterPublic(inflation_scaling_parameter);
    }
    if (truncation_scaling_parameter >= 0.0) {
        planner->setTruncationScalingParameterPublic(truncation_scaling_parameter);
    }
    if (allowed_failed_sampling_attempts >= 0) {
        planner->setAverageNumOfAllowedFailedAttemptsWhenSampling(
            static_cast<std::size_t>(allowed_failed_sampling_attempts));
    }
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

int validate_cspace_limits(const std::vector<std::vector<double>>& limits) {
    if (limits.empty()) {
        throw std::invalid_argument("cspace limits must be non-empty");
    }
    for (const auto& interval : limits) {
        if (interval.size() != 2U || !(interval[0] < interval[1])) {
            throw std::invalid_argument("each cspace limit must be [lo, hi] with lo < hi");
        }
    }
    return static_cast<int>(limits.size());
}

void validate_cspace_vector(const std::vector<double>& q, int dimension, const char* name) {
    if (static_cast<int>(q.size()) != dimension) {
        throw std::invalid_argument(std::string(name) + " dimension must match cspace limits");
    }
}

std::shared_ptr<ompl::base::RealVectorStateSpace> make_cspace_ompl_space(
    const std::vector<std::vector<double>>& limits) {
    namespace ob = ompl::base;
    const int dimension = validate_cspace_limits(limits);
    auto space = std::make_shared<ob::RealVectorStateSpace>(dimension);
    ob::RealVectorBounds bounds(dimension);
    for (int dim = 0; dim < dimension; ++dim) {
        bounds.setLow(dim, limits[static_cast<std::size_t>(dim)][0]);
        bounds.setHigh(dim, limits[static_cast<std::size_t>(dim)][1]);
    }
    space->setBounds(bounds);
    return space;
}

bool cspace_point_in_box(const double* values, int dimension, const std::vector<double>& flattened_box) {
    if (static_cast<int>(flattened_box.size()) != 2 * dimension) {
        throw std::invalid_argument("each cspace obstacle must be flattened [lo0, hi0, lo1, hi1, ...]");
    }
    for (int dim = 0; dim < dimension; ++dim) {
        const double lo = flattened_box[static_cast<std::size_t>(2 * dim)];
        const double hi = flattened_box[static_cast<std::size_t>(2 * dim + 1)];
        if (!(lo <= hi)) {
            throw std::invalid_argument("cspace obstacle interval must satisfy lo <= hi");
        }
        if (values[dim] < lo || values[dim] > hi) {
            return false;
        }
    }
    return true;
}

bool cspace_point_valid(const double* values,
                        int dimension,
                        const std::vector<std::vector<double>>& flattened_obstacles) {
    for (const auto& obstacle : flattened_obstacles) {
        if (cspace_point_in_box(values, dimension, obstacle)) {
            return false;
        }
    }
    return true;
}

bool cspace_vector_valid(const std::vector<double>& q,
                         int dimension,
                         const std::vector<std::vector<double>>& flattened_obstacles) {
    validate_cspace_vector(q, dimension, "q");
    return cspace_point_valid(q.data(), dimension, flattened_obstacles);
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

    py::class_<rbf::DynamicUpdateConfig>(module, "DynamicUpdateConfig")
        .def(py::init<>())
        .def_readwrite("enable_spatial_dirty_region", &rbf::DynamicUpdateConfig::enable_spatial_dirty_region)
        .def_readwrite("dirty_region_padding", &rbf::DynamicUpdateConfig::dirty_region_padding)
        .def_readwrite("dirty_anchor_limit", &rbf::DynamicUpdateConfig::dirty_anchor_limit)
        .def_readwrite("dirty_seed_limit", &rbf::DynamicUpdateConfig::dirty_seed_limit)
        .def_readwrite("local_regrow_box_limit", &rbf::DynamicUpdateConfig::local_regrow_box_limit)
        .def_readwrite("local_regrow_timeout_ms", &rbf::DynamicUpdateConfig::local_regrow_timeout_ms)
        .def_readwrite("insertion_leaf_sweep_max_depth", &rbf::DynamicUpdateConfig::insertion_leaf_sweep_max_depth)
        .def_readwrite("insertion_leaf_sweep_relative_depth", &rbf::DynamicUpdateConfig::insertion_leaf_sweep_relative_depth)
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
        .def_readwrite("parallel_virtual_validation", &rbf::LeafSweepConfig::parallel_virtual_validation)
        .def_readwrite("collision_overlap_prune_min_threshold", &rbf::LeafSweepConfig::collision_overlap_prune_min_threshold)
        .def_readwrite("collision_overlap_prune_decay_per_depth", &rbf::LeafSweepConfig::collision_overlap_prune_decay_per_depth)
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
        .def_readonly("collision_box_obstacle_indices", &rbf::LeafSweepResult::collision_box_obstacle_indices)
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
        .def_readwrite("refine_timeout_ms", &rbf::LeafSweepRefineConfig::refine_timeout_ms)
        .def_readwrite("priority_prune_radius", &rbf::LeafSweepRefineConfig::priority_prune_radius)
        .def_readwrite("collision_overlap_prune_min_depth", &rbf::LeafSweepRefineConfig::collision_overlap_prune_min_depth)
        .def_readwrite("collision_overlap_prune_threshold", &rbf::LeafSweepRefineConfig::collision_overlap_prune_threshold)
        .def_readwrite("collision_overlap_prune_ratio_threshold", &rbf::LeafSweepRefineConfig::collision_overlap_prune_ratio_threshold);

    py::class_<rbf::AdaptiveLeafSweepConfig>(module, "AdaptiveLeafSweepConfig")
        .def(py::init<>())
        .def_readwrite("shallow_start_depth", &rbf::AdaptiveLeafSweepConfig::shallow_start_depth)
        .def_readwrite("shallow_max_depth", &rbf::AdaptiveLeafSweepConfig::shallow_max_depth)
        .def_readwrite("target_max_depth", &rbf::AdaptiveLeafSweepConfig::target_max_depth)
        .def_readwrite("time_budget_ms", &rbf::AdaptiveLeafSweepConfig::time_budget_ms)
        .def_readwrite("node_budget", &rbf::AdaptiveLeafSweepConfig::node_budget)
        .def_readwrite("threads", &rbf::AdaptiveLeafSweepConfig::threads)
        .def_readwrite("validation_batch_size", &rbf::AdaptiveLeafSweepConfig::validation_batch_size)
        .def_readwrite("obstacle_cluster_gap", &rbf::AdaptiveLeafSweepConfig::obstacle_cluster_gap)
        .def_readwrite("use_virtual_topology", &rbf::AdaptiveLeafSweepConfig::use_virtual_topology)
        .def_readwrite("parallel_virtual_validation", &rbf::AdaptiveLeafSweepConfig::parallel_virtual_validation)
        .def_readwrite("store_group_results", &rbf::AdaptiveLeafSweepConfig::store_group_results)
        .def_readwrite("fast_virtual_checkpoint_mode", &rbf::AdaptiveLeafSweepConfig::fast_virtual_checkpoint_mode)
        .def_readwrite("defer_min_depth", &rbf::AdaptiveLeafSweepConfig::defer_min_depth)
        .def_readwrite("overlap_depth_threshold", &rbf::AdaptiveLeafSweepConfig::overlap_depth_threshold)
        .def_readwrite("overlap_depth_min_threshold", &rbf::AdaptiveLeafSweepConfig::overlap_depth_min_threshold)
        .def_readwrite("overlap_depth_decay_per_depth", &rbf::AdaptiveLeafSweepConfig::overlap_depth_decay_per_depth)
        .def_readwrite("overlap_ratio_threshold", &rbf::AdaptiveLeafSweepConfig::overlap_ratio_threshold)
        .def_readwrite("seed_probe_count", &rbf::AdaptiveLeafSweepConfig::seed_probe_count)
        .def_readwrite("seed_probe_rng_seed", &rbf::AdaptiveLeafSweepConfig::seed_probe_rng_seed)
        .def_readwrite("seed_promote_uncovered", &rbf::AdaptiveLeafSweepConfig::seed_promote_uncovered)
        .def_readwrite("seed_anchor_probe_cap", &rbf::AdaptiveLeafSweepConfig::seed_anchor_probe_cap)
        .def_readwrite("promotion_interval", &rbf::AdaptiveLeafSweepConfig::promotion_interval)
        .def_readwrite("adaptive_depth_enabled", &rbf::AdaptiveLeafSweepConfig::adaptive_depth_enabled)
        .def_readwrite("adaptive_depth_min", &rbf::AdaptiveLeafSweepConfig::adaptive_depth_min)
        .def_readwrite("adaptive_depth_max", &rbf::AdaptiveLeafSweepConfig::adaptive_depth_max)
        .def_readwrite("adaptive_depth_probe_count", &rbf::AdaptiveLeafSweepConfig::adaptive_depth_probe_count)
        .def_readwrite("adaptive_depth_anchor_probe_cap", &rbf::AdaptiveLeafSweepConfig::adaptive_depth_anchor_probe_cap)
        .def_readwrite("adaptive_depth_probe_seed", &rbf::AdaptiveLeafSweepConfig::adaptive_depth_probe_seed)
        .def_readwrite("adaptive_depth_min_free_probes", &rbf::AdaptiveLeafSweepConfig::adaptive_depth_min_free_probes)
        .def_readwrite("adaptive_depth_min_covered_probes", &rbf::AdaptiveLeafSweepConfig::adaptive_depth_min_covered_probes)
        .def_readwrite("adaptive_depth_min_main_probes", &rbf::AdaptiveLeafSweepConfig::adaptive_depth_min_main_probes)
        .def_readwrite("adaptive_depth_min_main_ratio", &rbf::AdaptiveLeafSweepConfig::adaptive_depth_min_main_ratio)
        .def_readwrite("adaptive_depth_min_cells", &rbf::AdaptiveLeafSweepConfig::adaptive_depth_min_cells)
        .def_readwrite("adaptive_depth_min_main_cells", &rbf::AdaptiveLeafSweepConfig::adaptive_depth_min_main_cells)
        .def_readwrite("adaptive_depth_max_online_cells", &rbf::AdaptiveLeafSweepConfig::adaptive_depth_max_online_cells)
        .def_readwrite("adaptive_depth_max_probe_ms", &rbf::AdaptiveLeafSweepConfig::adaptive_depth_max_probe_ms)
        .def_readwrite("max_merge_ms", &rbf::AdaptiveLeafSweepConfig::max_merge_ms)
        .def_readwrite("max_merge_rounds", &rbf::AdaptiveLeafSweepConfig::max_merge_rounds)
        .def_readwrite("max_merge_input_boxes", &rbf::AdaptiveLeafSweepConfig::max_merge_input_boxes)
        .def_readwrite("max_free_boxes", &rbf::AdaptiveLeafSweepConfig::max_free_boxes)
        .def_readwrite("max_unresolved_domains", &rbf::AdaptiveLeafSweepConfig::max_unresolved_domains)
        .def_readwrite("planning_backend", &rbf::AdaptiveLeafSweepConfig::planning_backend)
        .def_readwrite("grid_target_depth", &rbf::AdaptiveLeafSweepConfig::grid_target_depth)
        .def_readwrite("grid_face_index_enabled", &rbf::AdaptiveLeafSweepConfig::grid_face_index_enabled)
        .def_readwrite("grid_planning_max_expansions", &rbf::AdaptiveLeafSweepConfig::grid_planning_max_expansions)
        .def_readwrite("hipac_portal_connectivity", &rbf::AdaptiveLeafSweepConfig::hipac_portal_connectivity)
        .def_readwrite("hipac_portal_cell_native_validate", &rbf::AdaptiveLeafSweepConfig::hipac_portal_cell_native_validate)
        .def_readwrite("hipac_portal_max_internal_boxes", &rbf::AdaptiveLeafSweepConfig::hipac_portal_max_internal_boxes)
        .def_readwrite("hipac_portal_max_recursion_depth", &rbf::AdaptiveLeafSweepConfig::hipac_portal_max_recursion_depth)
        .def_readwrite("hipac_portal_ffb_depth", &rbf::AdaptiveLeafSweepConfig::hipac_portal_ffb_depth)
        .def_readwrite("hipac_portal_ffb_deadline_ms", &rbf::AdaptiveLeafSweepConfig::hipac_portal_ffb_deadline_ms)
        .def_readwrite("hipac_online_connectivity", &rbf::AdaptiveLeafSweepConfig::hipac_online_connectivity)
        .def_readwrite("hipac_online_before_query_bridge", &rbf::AdaptiveLeafSweepConfig::hipac_online_before_query_bridge)
        .def_readwrite("hipac_promote_query_repairs", &rbf::AdaptiveLeafSweepConfig::hipac_promote_query_repairs)
        .def_readwrite("hipac_online_candidate_max_length", &rbf::AdaptiveLeafSweepConfig::hipac_online_candidate_max_length)
        .def_readwrite("hipac_online_max_resolves_per_query", &rbf::AdaptiveLeafSweepConfig::hipac_online_max_resolves_per_query)
        .def_readwrite("hipac_online_max_hidden_boxes_per_portal", &rbf::AdaptiveLeafSweepConfig::hipac_online_max_hidden_boxes_per_portal)
        .def_readwrite("hipac_online_max_ffb_calls_per_portal", &rbf::AdaptiveLeafSweepConfig::hipac_online_max_ffb_calls_per_portal)
        .def_readwrite("hipac_online_prebridge_portal", &rbf::AdaptiveLeafSweepConfig::hipac_online_prebridge_portal)
        .def_readwrite("hipac_online_prebridge_candidate_limit", &rbf::AdaptiveLeafSweepConfig::hipac_online_prebridge_candidate_limit)
        .def_readwrite("hipac_online_prebridge_max_pair_distance", &rbf::AdaptiveLeafSweepConfig::hipac_online_prebridge_max_pair_distance)
        .def_readwrite("hipac_online_prebridge_route_distance_weight", &rbf::AdaptiveLeafSweepConfig::hipac_online_prebridge_route_distance_weight)
        .def_readwrite("hipac_online_prebridge_pair_distance_weight", &rbf::AdaptiveLeafSweepConfig::hipac_online_prebridge_pair_distance_weight)
        .def_readwrite("hipac_transition_obb_portal", &rbf::AdaptiveLeafSweepConfig::hipac_transition_obb_portal)
        .def_readwrite("hipac_transition_obb_lateral_radius", &rbf::AdaptiveLeafSweepConfig::hipac_transition_obb_lateral_radius)
        .def_readwrite("hipac_transition_obb_longitudinal_margin", &rbf::AdaptiveLeafSweepConfig::hipac_transition_obb_longitudinal_margin)
        .def_readwrite("hipac_transition_obb_safety_epsilon", &rbf::AdaptiveLeafSweepConfig::hipac_transition_obb_safety_epsilon)
        .def_readwrite("segment_edge_obb_cover", &rbf::AdaptiveLeafSweepConfig::segment_edge_obb_cover)
        .def_readwrite("rrt_bridge_obb_cover", &rbf::AdaptiveLeafSweepConfig::rrt_bridge_obb_cover)
        .def_readwrite("strict_obb_bridge_cover", &rbf::AdaptiveLeafSweepConfig::strict_obb_bridge_cover)
        .def_readwrite("segment_edge_obb_lateral_radius", &rbf::AdaptiveLeafSweepConfig::segment_edge_obb_lateral_radius)
        .def_readwrite("segment_edge_obb_longitudinal_margin", &rbf::AdaptiveLeafSweepConfig::segment_edge_obb_longitudinal_margin)
        .def_readwrite("segment_edge_obb_safety_epsilon", &rbf::AdaptiveLeafSweepConfig::segment_edge_obb_safety_epsilon)
        .def_readwrite("segment_edge_obb_grow_iterations", &rbf::AdaptiveLeafSweepConfig::segment_edge_obb_grow_iterations)
        .def_readwrite("segment_edge_obb_binary_iterations", &rbf::AdaptiveLeafSweepConfig::segment_edge_obb_binary_iterations)
        .def_readwrite("segment_edge_obb_split_depth", &rbf::AdaptiveLeafSweepConfig::segment_edge_obb_split_depth)
        .def_readwrite("obb_max_window_segments", &rbf::AdaptiveLeafSweepConfig::obb_max_window_segments)
        .def_readwrite("obb_max_validations_per_window", &rbf::AdaptiveLeafSweepConfig::obb_max_validations_per_window)
        .def_readwrite("obb_fast_primary_orientation", &rbf::AdaptiveLeafSweepConfig::obb_fast_primary_orientation)
        .def_readwrite("obb_fallback_orientations_on_primary_fail", &rbf::AdaptiveLeafSweepConfig::obb_fallback_orientations_on_primary_fail)
        .def_readwrite("obb_sampled_support_enabled", &rbf::AdaptiveLeafSweepConfig::obb_sampled_support_enabled)
        .def_readwrite("obb_clearance_sampled_support_enabled", &rbf::AdaptiveLeafSweepConfig::obb_clearance_sampled_support_enabled)
        .def_readwrite("obb_clearance_lateral_l1_max", &rbf::AdaptiveLeafSweepConfig::obb_clearance_lateral_l1_max)
        .def_readwrite("obb_clearance_samples", &rbf::AdaptiveLeafSweepConfig::obb_clearance_samples)
        .def_readwrite("obb_clearance_dense_line_l1_threshold", &rbf::AdaptiveLeafSweepConfig::obb_clearance_dense_line_l1_threshold)
        .def_readwrite("obb_clearance_dense_samples", &rbf::AdaptiveLeafSweepConfig::obb_clearance_dense_samples)
        .def_readwrite("obb_clearance_fast_samples", &rbf::AdaptiveLeafSweepConfig::obb_clearance_fast_samples)
        .def_readwrite("obb_clearance_first", &rbf::AdaptiveLeafSweepConfig::obb_clearance_first)
        .def_readwrite("obb_clearance_retry_attempts", &rbf::AdaptiveLeafSweepConfig::obb_clearance_retry_attempts)
        .def_readwrite("obb_clearance_retry_values", &rbf::AdaptiveLeafSweepConfig::obb_clearance_retry_values)
        .def_readwrite("obb_clearance_retry_iters", &rbf::AdaptiveLeafSweepConfig::obb_clearance_retry_iters)
        .def_readwrite("obb_clearance_retry_timeout_ms", &rbf::AdaptiveLeafSweepConfig::obb_clearance_retry_timeout_ms)
        .def_readwrite("segment_edge_obb_metadata_only", &rbf::AdaptiveLeafSweepConfig::segment_edge_obb_metadata_only)
        .def_readwrite("segment_edge_obb_metadata_require_cover", &rbf::AdaptiveLeafSweepConfig::segment_edge_obb_metadata_require_cover)
        .def_readwrite("hipac_promote_transition_slices", &rbf::AdaptiveLeafSweepConfig::hipac_promote_transition_slices)
        .def_readwrite("hipac_promote_transition_target_query_indices", &rbf::AdaptiveLeafSweepConfig::hipac_promote_transition_target_query_indices)
        .def_readwrite("hipac_promote_transition_min_boxes", &rbf::AdaptiveLeafSweepConfig::hipac_promote_transition_min_boxes)
        .def_readwrite("hipac_promote_transition_max_boxes", &rbf::AdaptiveLeafSweepConfig::hipac_promote_transition_max_boxes)
        .def_readwrite("hipac_promote_transition_max_attempts_per_query", &rbf::AdaptiveLeafSweepConfig::hipac_promote_transition_max_attempts_per_query);

    py::class_<rbf::EndpointMainBoxCorridorConfig>(module, "EndpointMainBoxCorridorConfig")
        .def(py::init<>())
        .def_readwrite("target_k", &rbf::EndpointMainBoxCorridorConfig::target_k)
        .def_readwrite("coarse_step", &rbf::EndpointMainBoxCorridorConfig::coarse_step)
        .def_readwrite("fine_step", &rbf::EndpointMainBoxCorridorConfig::fine_step)
        .def_readwrite("max_ffb_calls", &rbf::EndpointMainBoxCorridorConfig::max_ffb_calls)
        .def_readwrite("max_boxes", &rbf::EndpointMainBoxCorridorConfig::max_boxes)
        .def_readwrite("residual_segment_max_length", &rbf::EndpointMainBoxCorridorConfig::residual_segment_max_length)
        .def_readwrite("lateral_offset", &rbf::EndpointMainBoxCorridorConfig::lateral_offset)
        .def_readwrite("lateral_rounds", &rbf::EndpointMainBoxCorridorConfig::lateral_rounds)
        .def_readwrite("face_epsilon", &rbf::EndpointMainBoxCorridorConfig::face_epsilon);

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

    py::class_<rbf::AdaptiveLeafSweepResult>(module, "AdaptiveLeafSweepResult")
        .def_readonly("leaf_sweep", &rbf::AdaptiveLeafSweepResult::leaf_sweep)
        .def_readonly("profile", &rbf::AdaptiveLeafSweepResult::profile)
        .def_readonly("shallow_free_count", &rbf::AdaptiveLeafSweepResult::shallow_free_count)
        .def_readonly("shallow_collision_count", &rbf::AdaptiveLeafSweepResult::shallow_collision_count)
        .def_readonly("adaptive_free_added", &rbf::AdaptiveLeafSweepResult::adaptive_free_added)
        .def_readonly("adaptive_validated", &rbf::AdaptiveLeafSweepResult::adaptive_validated)
        .def_readonly("adaptive_splits", &rbf::AdaptiveLeafSweepResult::adaptive_splits)
        .def_readonly("adaptive_deferred", &rbf::AdaptiveLeafSweepResult::adaptive_deferred)
        .def_readonly("adaptive_promoted", &rbf::AdaptiveLeafSweepResult::adaptive_promoted)
        .def_readonly("unresolved_domains", &rbf::AdaptiveLeafSweepResult::unresolved_domains)
        .def_readonly("seed_probe_count", &rbf::AdaptiveLeafSweepResult::seed_probe_count)
        .def_readonly("seed_probe_free_count", &rbf::AdaptiveLeafSweepResult::seed_probe_free_count)
        .def_readonly("seed_probe_box_covered", &rbf::AdaptiveLeafSweepResult::seed_probe_box_covered)
        .def_readonly("seed_probe_anchor_success", &rbf::AdaptiveLeafSweepResult::seed_probe_anchor_success)
        .def_readonly("seed_probe_main_accessible", &rbf::AdaptiveLeafSweepResult::seed_probe_main_accessible)
        .def_readonly("p_box_covered", &rbf::AdaptiveLeafSweepResult::p_box_covered)
        .def_readonly("p_anchor_success", &rbf::AdaptiveLeafSweepResult::p_anchor_success)
        .def_readonly("p_main_accessible", &rbf::AdaptiveLeafSweepResult::p_main_accessible)
        .def_readonly("p_anchor_to_main_uncovered", &rbf::AdaptiveLeafSweepResult::p_anchor_to_main_uncovered)
        .def_readonly("selected_leaf_depth", &rbf::AdaptiveLeafSweepResult::selected_leaf_depth)
        .def_readonly("adaptive_depth_readiness_met", &rbf::AdaptiveLeafSweepResult::adaptive_depth_readiness_met)
        .def_readonly("adaptive_depth_stop_reason", &rbf::AdaptiveLeafSweepResult::adaptive_depth_stop_reason)
        .def_readonly("adaptive_depth_snapshots_json", &rbf::AdaptiveLeafSweepResult::adaptive_depth_snapshots_json)
        .def_readonly("leaf_sweep_ms", &rbf::AdaptiveLeafSweepResult::leaf_sweep_ms)
        .def_readonly("adaptive_ms", &rbf::AdaptiveLeafSweepResult::adaptive_ms)
        .def_readonly("coverage_probe_ms", &rbf::AdaptiveLeafSweepResult::coverage_probe_ms)
        .def_readonly("total_ms", &rbf::AdaptiveLeafSweepResult::total_ms)
        .def_readonly("partition_cell_count", &rbf::AdaptiveLeafSweepResult::partition_cell_count)
        .def_readonly("partition_grid_cell_count", &rbf::AdaptiveLeafSweepResult::partition_grid_cell_count)
        .def_readonly("partition_non_grid_cell_count", &rbf::AdaptiveLeafSweepResult::partition_non_grid_cell_count)
        .def_readonly("partition_face_index_entries", &rbf::AdaptiveLeafSweepResult::partition_face_index_entries)
        .def_readonly("partition_islands", &rbf::AdaptiveLeafSweepResult::partition_islands)
        .def_readonly("partition_largest_island", &rbf::AdaptiveLeafSweepResult::partition_largest_island)
        .def_readonly("diagnostics", &rbf::AdaptiveLeafSweepResult::diagnostics);

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

    py::class_<rbf::OccupiedCertificateConfig>(module, "OccupiedCertificateConfig")
        .def(py::init<>())
        .def_readwrite("enabled", &rbf::OccupiedCertificateConfig::enabled)
        .def_readwrite("numerical_epsilon", &rbf::OccupiedCertificateConfig::numerical_epsilon)
        .def_readwrite("min_penetration_margin", &rbf::OccupiedCertificateConfig::min_penetration_margin);

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
        .def_readwrite("external_evidence_live_retry_on_maybe", &rbf::OracleValidationConfig::external_evidence_live_retry_on_maybe)
        .def_readwrite("stateless_materialization_context", &rbf::OracleValidationConfig::stateless_materialization_context)
        .def_readwrite("enable_worker_shared_endpoint_cache", &rbf::OracleValidationConfig::enable_worker_shared_endpoint_cache)
        .def_readwrite("shared_endpoint_cache_max_entries", &rbf::OracleValidationConfig::shared_endpoint_cache_max_entries)
        .def_readwrite("shared_endpoint_cache_max_bytes", &rbf::OracleValidationConfig::shared_endpoint_cache_max_bytes)
        .def_readwrite("collect_full_overlap_stats", &rbf::OracleValidationConfig::collect_full_overlap_stats)
        .def_readwrite("occupied_certificate", &rbf::OracleValidationConfig::occupied_certificate);

    py::enum_<rbf::FindFreeBoxSearchMode>(module, "FindFreeBoxSearchMode")
        .value("Linear", rbf::FindFreeBoxSearchMode::Linear)
        .value("BinaryDepth", rbf::FindFreeBoxSearchMode::BinaryDepth);

    py::class_<rbf::FindFreeBoxOptions>(module, "FindFreeBoxOptions")
        .def(py::init<>())
        .def_readwrite("max_depth", &rbf::FindFreeBoxOptions::max_depth)
        .def_readwrite("start_depth", &rbf::FindFreeBoxOptions::start_depth)
        .def_readwrite("skip_to_depth", &rbf::FindFreeBoxOptions::skip_to_depth)
        .def_readwrite("search_mode", &rbf::FindFreeBoxOptions::search_mode)
        .def_readwrite("binary_probe_depth", &rbf::FindFreeBoxOptions::binary_probe_depth)
        .def_readwrite("adaptive_depths", &rbf::FindFreeBoxOptions::adaptive_depths)
        .def_readwrite("deadline_ms", &rbf::FindFreeBoxOptions::deadline_ms)
        .def_readwrite("split_reserved_leaf", &rbf::FindFreeBoxOptions::split_reserved_leaf)
        .def_readwrite("split_unknown_leaf", &rbf::FindFreeBoxOptions::split_unknown_leaf)
        .def_readwrite("reject_seed_collision", &rbf::FindFreeBoxOptions::reject_seed_collision)
        .def_readwrite("skip_existing_cover_check", &rbf::FindFreeBoxOptions::skip_existing_cover_check)
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
        .def_readwrite("segment_step", &rbf::RRTConnectConfig::segment_step)
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
        .def_readwrite("point_validated_gap_step", &rbf::IslandConnectorConfig::point_validated_gap_step)
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
        .def_readwrite("coverage_intervals_override", &rbf::LectDatabaseRuntimeConfig::coverage_intervals_override)
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
        .def_readwrite("shortcut_cost_aware", &rbf::QueryConfig::shortcut_cost_aware)
        .def_readwrite("shortcut_cost_factor", &rbf::QueryConfig::shortcut_cost_factor)
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
        .def_readwrite("final_rrt_simplify", &rbf::QueryConfig::final_rrt_simplify)
        .def_readwrite("final_rrt_simplify_timeout_ms", &rbf::QueryConfig::final_rrt_simplify_timeout_ms)
        .def_readwrite("final_rrt_simplify_max_iters", &rbf::QueryConfig::final_rrt_simplify_max_iters)
        .def_readwrite("final_rrt_simplify_attempts", &rbf::QueryConfig::final_rrt_simplify_attempts)
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
        .def_readwrite("enable_connector", &rbf::RBFPlanningConfig::enable_connector)
        .def_readwrite("query_bridge_pave_depth", &rbf::RBFPlanningConfig::query_bridge_pave_depth)
        .def_readwrite("query_bridge_ffb_start_depth", &rbf::RBFPlanningConfig::query_bridge_ffb_start_depth)
        .def_readwrite("query_endpoint_anchor_ffb_depth", &rbf::RBFPlanningConfig::query_endpoint_anchor_ffb_depth)
        .def_readwrite("query_endpoint_anchor_ffb_depths", &rbf::RBFPlanningConfig::query_endpoint_anchor_ffb_depths)
        .def_readwrite("query_endpoint_point_anchor", &rbf::RBFPlanningConfig::query_endpoint_point_anchor)
        .def_readwrite("endpoint_shortlink_max_length", &rbf::RBFPlanningConfig::endpoint_shortlink_max_length)
        .def_readwrite("query_bridge_accept_segment_fraction", &rbf::RBFPlanningConfig::query_bridge_accept_segment_fraction)
        .def_readwrite("query_bridge_accept_path_ratio", &rbf::RBFPlanningConfig::query_bridge_accept_path_ratio)
        .def_readwrite("query_bridge_accept_path_additive", &rbf::RBFPlanningConfig::query_bridge_accept_path_additive)
        .def_readwrite("query_bridge_accept_max_path_length", &rbf::RBFPlanningConfig::query_bridge_accept_max_path_length)
        .def_readwrite("query_bridge_no_path_retry_attempts", &rbf::RBFPlanningConfig::query_bridge_no_path_retry_attempts)
        .def_readwrite("query_bridge_no_path_retry_stop_on_first_success", &rbf::RBFPlanningConfig::query_bridge_no_path_retry_stop_on_first_success)
        .def_readwrite("query_bridge_forced_attempts", &rbf::RBFPlanningConfig::query_bridge_forced_attempts)
        .def_readwrite("query_bridge_attempt_offset", &rbf::RBFPlanningConfig::query_bridge_attempt_offset)
        .def_readwrite("query_bridge_rrt_fixed_iters", &rbf::RBFPlanningConfig::query_bridge_rrt_fixed_iters)
        .def_readwrite("query_bridge_local_radius_schedule", &rbf::RBFPlanningConfig::query_bridge_local_radius_schedule)
        .def_readwrite("query_bridge_no_path_retry_budget_iters", &rbf::RBFPlanningConfig::query_bridge_no_path_retry_budget_iters)
        .def_readwrite("query_bridge_no_path_retry_budget_attempts", &rbf::RBFPlanningConfig::query_bridge_no_path_retry_budget_attempts)
        .def_readwrite("query_bridge_hybridize_attempt_paths", &rbf::RBFPlanningConfig::query_bridge_hybridize_attempt_paths)
        .def_readwrite("query_bridge_hybrid_max_paths", &rbf::RBFPlanningConfig::query_bridge_hybrid_max_paths)
        .def_readwrite("query_bridge_hybrid_max_vertices", &rbf::RBFPlanningConfig::query_bridge_hybrid_max_vertices)
        .def_readwrite("query_bridge_hybrid_max_cross_checks", &rbf::RBFPlanningConfig::query_bridge_hybrid_max_cross_checks)
        .def_readwrite("query_bridge_parallel_rrt_early_stop", &rbf::RBFPlanningConfig::query_bridge_parallel_rrt_early_stop)
        .def_readwrite("query_bridge_parallel_rrt_early_stop_min_successes", &rbf::RBFPlanningConfig::query_bridge_parallel_rrt_early_stop_min_successes)
        .def_readwrite("query_bridge_parallel_rrt_early_stop_ratio", &rbf::RBFPlanningConfig::query_bridge_parallel_rrt_early_stop_ratio)
        .def_readwrite("query_bridge_parallel_rrt_early_stop_additive", &rbf::RBFPlanningConfig::query_bridge_parallel_rrt_early_stop_additive)
        .def_readwrite("query_bridge_scene_reusable_edges", &rbf::RBFPlanningConfig::query_bridge_scene_reusable_edges)
        .def_readwrite("query_bridge_direct_segment_after_rrt", &rbf::RBFPlanningConfig::query_bridge_direct_segment_after_rrt)
        .def_readwrite("query_bridge_fast_direct_segment_after_rrt", &rbf::RBFPlanningConfig::query_bridge_fast_direct_segment_after_rrt)
        .def_readwrite("query_bridge_fast_direct_random_shortcut_iters", &rbf::RBFPlanningConfig::query_bridge_fast_direct_random_shortcut_iters)
        .def_readwrite("query_bridge_direct_max_length", &rbf::RBFPlanningConfig::query_bridge_direct_max_length)
        .def_readwrite("query_bridge_direct_sample_step", &rbf::RBFPlanningConfig::query_bridge_direct_sample_step)
        .def_readwrite("query_bridge_direct_sample_steps_by_query", &rbf::RBFPlanningConfig::query_bridge_direct_sample_steps_by_query)
        .def_readwrite("query_bridge_full_residual_overlay_when_connected", &rbf::RBFPlanningConfig::query_bridge_full_residual_overlay_when_connected)
        .def_readwrite("query_bridge_adaptive_max_repair_subdivisions", &rbf::RBFPlanningConfig::query_bridge_adaptive_max_repair_subdivisions)
        .def_readwrite("query_bridge_adaptive_fine_step", &rbf::RBFPlanningConfig::query_bridge_adaptive_fine_step)
        .def_readwrite("query_bridge_adaptive_max_repair_calls", &rbf::RBFPlanningConfig::query_bridge_adaptive_max_repair_calls)
        .def_readwrite("query_bridge_adaptive_max_repair_calls_by_query", &rbf::RBFPlanningConfig::query_bridge_adaptive_max_repair_calls_by_query)
        .def_readwrite("query_box_transition_edge_cost_penalty", &rbf::RBFPlanningConfig::query_box_transition_edge_cost_penalty)
        .def_readwrite("query_box_transition_nonprogress_penalty", &rbf::RBFPlanningConfig::query_box_transition_nonprogress_penalty)
        .def_readwrite("query_box_transition_line_deviation_penalty", &rbf::RBFPlanningConfig::query_box_transition_line_deviation_penalty)
        .def_readwrite("query_bridge_edge_cost_penalty", &rbf::RBFPlanningConfig::query_bridge_edge_cost_penalty)
        .def_readwrite("query_foreign_edge_cost_penalty", &rbf::RBFPlanningConfig::query_foreign_edge_cost_penalty)
        .def_readwrite("portal_membership_policy", &rbf::RBFPlanningConfig::portal_membership_policy);

    py::class_<rbf::RBFQueryRuntimeOptions>(module, "RBFQueryRuntimeOptions")
        .def(py::init<>())
        .def_readwrite("active_query_index", &rbf::RBFQueryRuntimeOptions::active_query_index);

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
        .def_readonly("collision_cache_rejected_disconnected", &rbf::RebuildProfile::collision_cache_rejected_disconnected)
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

    py::class_<rbf::QueryResult>(module, "QueryResult")
        .def_readonly("success", &rbf::QueryResult::success)
        .def_readonly("start_box_id", &rbf::QueryResult::start_box_id)
        .def_readonly("goal_box_id", &rbf::QueryResult::goal_box_id)
        .def_readonly("box_sequence", &rbf::QueryResult::box_sequence)
        .def_readonly("segment_edge_sequence", &rbf::QueryResult::segment_edge_sequence)
        .def_readonly("path", &rbf::QueryResult::path)
        .def("path_as_lists", [](const rbf::QueryResult& result) {
            return eigen_path_to_lists(result.path);
        })
        .def_readonly("path_length", &rbf::QueryResult::path_length)
        .def_readonly("raw_path_length", &rbf::QueryResult::raw_path_length)
        .def_readonly("query_time_ms", &rbf::QueryResult::query_time_ms)
        .def_readonly("segment_edges_used", &rbf::QueryResult::segment_edges_used)
        .def_readonly("audit_status", &rbf::QueryResult::audit_status)
        .def_readonly("audit_passed", &rbf::QueryResult::audit_passed)
        .def_readonly("audit_time_ms", &rbf::QueryResult::audit_time_ms)
        .def_readonly("repair_time_ms", &rbf::QueryResult::repair_time_ms)
        .def_readonly("final_simplify_time_ms", &rbf::QueryResult::final_simplify_time_ms)
        .def_readonly("repair_count", &rbf::QueryResult::repair_count)
        .def_readonly("failed_segment_index", &rbf::QueryResult::failed_segment_index)
        .def_readonly("certified_box_length", &rbf::QueryResult::certified_box_length)
        .def_readonly("provisional_audited_length", &rbf::QueryResult::provisional_audited_length)
        .def_readonly("segment_edge_length", &rbf::QueryResult::segment_edge_length)
        .def_readonly("obb_edges_used", &rbf::QueryResult::obb_edges_used)
        .def_readonly("obb_regions_used", &rbf::QueryResult::obb_regions_used)
        .def_readonly("obb_edge_length", &rbf::QueryResult::obb_edge_length)
        .def_readonly("partition_cells_used", &rbf::QueryResult::partition_cells_used)
        .def_readonly("partition_search_ms", &rbf::QueryResult::partition_search_ms)
        .def_readonly("partition_repair_ms", &rbf::QueryResult::partition_repair_ms)
        .def_readonly("non_grid_cells_used", &rbf::QueryResult::non_grid_cells_used)
        .def_readonly("residual_segment_fraction", &rbf::QueryResult::residual_segment_fraction)
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
                const std::vector<std::vector<double>>& priority_points,
                const std::vector<std::vector<double>>& offline_anchor_points) {
                 return forest.build_leaf_sweep_refined(
                     obstacles,
                     config,
                     eigen_vectors_from_lists(priority_points),
                     eigen_vectors_from_lists(offline_anchor_points));
             },
             py::arg("obstacles"),
             py::arg("config") = rbf::LeafSweepRefineConfig{},
             py::arg("priority_points") = std::vector<std::vector<double>>{},
             py::arg("offline_anchor_points") = std::vector<std::vector<double>>{})
        .def("build_adaptive_deep_leaf_sweep_cover",
             [](rbf::RBFPlanningForest& forest,
                const std::vector<rbf::Obstacle>& obstacles,
                const rbf::AdaptiveLeafSweepConfig& config) {
                 return forest.build_adaptive_deep_leaf_sweep_cover(obstacles, config);
             },
             py::arg("obstacles"),
             py::arg("config") = rbf::AdaptiveLeafSweepConfig{})
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
                 const std::vector<double>& goal,
                 const rbf::RBFQueryRuntimeOptions& options) {
                  return forest.query(eigen_vector_from_list(start),
                                      eigen_vector_from_list(goal),
                                      options);
               },
               py::arg("start"), py::arg("goal"),
               py::arg("options") = rbf::RBFQueryRuntimeOptions{})
                                .def("refine_query_corridor",
                                                 [](rbf::RBFPlanningForest& forest,
                                                                const std::vector<double>& start,
                                                                const std::vector<double>& goal,
                                                                int max_boxes_to_add,
                                                                const std::string& mode,
                                                                double long_path_ratio,
                                                                double long_path_min_delta) {
                                                                        rbf::CorridorRefineMode refine_mode = rbf::CorridorRefineMode::SegmentBridge;
                                                                        if (mode == "box_only_long_path") {
                                                                            refine_mode = rbf::CorridorRefineMode::BoxOnlyLongPath;
                                                                        } else if (mode != "segment_bridge") {
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
                                                 py::arg("mode") = "segment_bridge",
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
                .def("anchor_query_endpoint_box",
                        [](rbf::RBFPlanningForest& forest,
                           const std::vector<double>& point) {
                            return forest.anchor_query_endpoint(eigen_vector_from_list(point));
                        },
                        py::arg("point"))
                .def("connect_query_endpoint_to_main_island",
                        [](rbf::RBFPlanningForest& forest,
                           const std::vector<double>& point,
                           double max_segment_length) {
                            return forest.connect_query_endpoint_to_main_island(
                                eigen_vector_from_list(point),
                                max_segment_length);
                        },
                        py::arg("point"),
                        py::arg("max_segment_length") = 0.0)
                .def("connect_query_endpoint_to_main_box_corridor",
                        [](rbf::RBFPlanningForest& forest,
                           const std::vector<double>& point,
                           const rbf::EndpointMainBoxCorridorConfig& config) {
                            return forest.connect_query_endpoint_to_main_box_corridor(
                                eigen_vector_from_list(point),
                                config);
                        },
                        py::arg("point"),
                        py::arg("config") = rbf::EndpointMainBoxCorridorConfig{})
                .def("add_offline_shortcut_edges",
                        [](rbf::RBFPlanningForest& forest,
                           int max_edges,
                           int candidate_limit,
                           double min_gain_ratio,
                           double max_segment_length,
                           bool allow_segment_fallback) {
                                    return forest.add_offline_shortcut_edges(max_edges,
                                                                             candidate_limit,
                                                                             min_gain_ratio,
                                                                             max_segment_length,
                                                                             allow_segment_fallback);
                         },
                         py::arg("max_edges"),
                         py::arg("candidate_limit") = 48,
                         py::arg("min_gain_ratio") = 1.6,
                         py::arg("max_segment_length") = 3.0,
                         py::arg("allow_segment_fallback") = false)
                .def("bridge_queries",
                         [](rbf::RBFPlanningForest& forest,
                                const std::vector<std::vector<double>>& starts,
                                const std::vector<std::vector<double>>& goals,
                                const std::vector<int>& forced_query_indices,
                                const std::vector<int>& global_query_indices) {
                                    if (starts.size() != goals.size()) {
                                        throw std::invalid_argument("bridge_queries requires starts/goals with matching sizes");
                                    }
                                    std::vector<Eigen::VectorXd> eigen_starts;
                                    std::vector<Eigen::VectorXd> eigen_goals;
                                    eigen_starts.reserve(starts.size());
                                    eigen_goals.reserve(goals.size());
                                    for (std::size_t i = 0; i < starts.size(); ++i) {
                                        eigen_starts.push_back(eigen_vector_from_list(starts[i]));
                                        eigen_goals.push_back(eigen_vector_from_list(goals[i]));
                                    }
                                    rbf::QueryBridgeBatchOptions options;
                                    options.forced_query_indices = forced_query_indices;
                                    options.global_query_indices = global_query_indices;
                                    return forest.bridge_queries(eigen_starts, eigen_goals, options);
                         },
                         py::arg("starts"),
                         py::arg("goals"),
                         py::arg("forced_query_indices") = std::vector<int>{},
                         py::arg("global_query_indices") = std::vector<int>{})
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
                                int gap_fill_max_ffb_calls) {
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
                                        item["validation_detail"] = oracle_validation_detail_to_python(failure.validation_detail);
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
                         py::arg("gap_fill_max_ffb_calls") = 32)
                .def("debug_chain_pave_waypoints",
                         [](rbf::RBFPlanningForest& forest,
                                const std::vector<std::vector<double>>& waypoints,
                                int max_chain,
                                int max_depth,
                                int max_gap_fill_steps,
                                bool fill_segment_gaps,
                                double gap_fill_min_step,
                                double adjacency_tolerance,
                                double gap_fill_sample_step,
                                double gap_fill_time_budget_ms,
                                int gap_fill_max_ffb_calls,
                                bool require_connected_chain,
                                bool commit_certified_only) {
                                    rbf::ChainPaveConfig pave;
                                    pave.commit_policy =
                                        commit_certified_only
                                            ? rbf::BoxCommitPolicy::CommitCertifiedOnly
                                            : rbf::BoxCommitPolicy::CommitProvisionalAllowed;
                                    pave.max_chain = max_chain;
                                    pave.fill_gaps = fill_segment_gaps;
                                    pave.max_gap_fill_depth = std::max(1, std::min(20, max_gap_fill_steps));
                                    pave.gap_fill_min_step = gap_fill_min_step;
                                    pave.adjacency_tolerance = adjacency_tolerance;
                                    pave.gap_fill_sample_step = gap_fill_sample_step;
                                    pave.gap_fill_time_budget_ms = gap_fill_time_budget_ms;
                                    pave.gap_fill_max_ffb_calls = gap_fill_max_ffb_calls;
                                    pave.require_connected_chain = require_connected_chain;
                                    pave.find_free_box.max_depth = max_depth;
                                    pave.find_free_box.reject_seed_collision = false;
                                    std::vector<Eigen::VectorXd> eigen_waypoints;
                                    eigen_waypoints.reserve(waypoints.size());
                                    for (const auto& waypoint : waypoints) {
                                        eigen_waypoints.push_back(eigen_vector_from_list(waypoint));
                                    }
                                    auto res = forest.debug_chain_pave_waypoints(eigen_waypoints, pave);
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
                                        item["validation_detail"] = oracle_validation_detail_to_python(failure.validation_detail);
                                        item["node"] = failure.node;
                                        item["depth"] = failure.depth;
                                        item["changed_dim"] = failure.changed_dim;
                                        item["fail_code"] = failure.fail_code;
                                        item["hit_unknown_depth_cap"] = failure.hit_unknown_depth_cap;
                                        item["hit_reserved_depth_cap"] = failure.hit_reserved_depth_cap;
                                        boundary_failures.append(std::move(item));
                                    }
                                    result["boundary_failures"] = std::move(boundary_failures);
                                    py::list out_waypoints;
                                    for (const auto& wp : res.waypoints) {
                                        out_waypoints.append(vector_to_list(wp));
                                    }
                                    result["waypoints"] = out_waypoints;
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
                         py::arg("waypoints"),
                         py::arg("max_chain") = 4096,
                         py::arg("max_depth") = 120,
                         py::arg("max_gap_fill_steps") = 64,
                         py::arg("fill_segment_gaps") = true,
                         py::arg("gap_fill_min_step") = 1e-4,
                         py::arg("adjacency_tolerance") = 1e-9,
                         py::arg("gap_fill_sample_step") = 0.05,
                         py::arg("gap_fill_time_budget_ms") = 10.0,
                         py::arg("gap_fill_max_ffb_calls") = 32,
                         py::arg("require_connected_chain") = false,
                         py::arg("commit_certified_only") = true)
        .def("add_obstacle_and_rebuild", &rbf::RBFPlanningForest::add_obstacle_and_rebuild, py::arg("obstacle"))
        .def("add_obstacles_and_rebuild", &rbf::RBFPlanningForest::add_obstacles_and_rebuild, py::arg("obstacles"))
        .def("connect_update_segment_fallback", &rbf::RBFPlanningForest::connect_update_segment_fallback)
        .def("connect_update_endpoint_segment_fallback",
             [](rbf::RBFPlanningForest& forest,
                const std::vector<double>& start,
                const std::vector<double>& goal) {
                 return forest.connect_update_endpoint_segment_fallback(eigen_vector_from_list(start),
                                                                        eigen_vector_from_list(goal));
             },
             py::arg("start"), py::arg("goal"))
        .def("remove_obstacle_and_regrow", &rbf::RBFPlanningForest::remove_obstacle_and_regrow, py::arg("obstacle_index"))
        .def("remove_obstacle_suffix_and_regrow", &rbf::RBFPlanningForest::remove_obstacle_suffix_and_regrow, py::arg("target_obstacle_count"))
        .def("clear_forest", &rbf::RBFPlanningForest::clear_forest)
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
             py::arg("interval_pairs"))
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
             py::arg("disable_caches") = true)
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
                const double solve_s = elapsed_s();
                result["ok"] = false;
                result["reason"] = "endpoint_collision";
                result["status"] = "endpoint_collision";
                result["solve_s"] = solve_s;
                result["simplify_s"] = 0.0;
                result["t_s"] = solve_s;
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
            const double solve_s = elapsed_s();
            const bool exact_solution = status == ob::PlannerStatus::EXACT_SOLUTION;
            bool ok = exact_solution;
            double simplify_s = 0.0;
            if (ok && simplify_time_s > 0.0) {
                const auto simplify_start = std::chrono::steady_clock::now();
                setup.simplifySolution(simplify_time_s);
                simplify_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - simplify_start).count();
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
            result["solve_s"] = solve_s;
            result["simplify_s"] = simplify_s;
            result["t_s"] = solve_s + simplify_s;
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
           int max_nearest_neighbors,
           const std::string& planner_kind,
           bool preload_query_endpoints) {
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
            std::shared_ptr<og::PRM> planner;
            std::shared_ptr<SeededPRM> seeded_prm;
            std::shared_ptr<SeededPRMstar> seeded_prmstar;
            const bool use_prmstar = planner_kind == "prmstar" || planner_kind == "PRMstar" || planner_kind == "PRM*";
            if (use_prmstar) {
                seeded_prmstar = std::make_shared<SeededPRMstar>(si);
                seeded_prmstar->setLocalSeed(mix_seed(base_seed, 0x50524D50U));
                planner = seeded_prmstar;
            } else {
                seeded_prm = std::make_shared<SeededPRM>(si);
                seeded_prm->setLocalSeed(mix_seed(base_seed, 0x50524D50U));
                planner = seeded_prm;
            }
            if (!use_prmstar && max_nearest_neighbors > 0) {
                planner->setMaxNearestNeighbors(static_cast<unsigned int>(max_nearest_neighbors));
            }
            planner->setProblemDefinition(problem);
            planner->setup();

            const auto build_start = std::chrono::steady_clock::now();
            if (preload_query_endpoints) {
                for (std::size_t index = 0; index < starts.size(); ++index) {
                    ob::ScopedState<ob::RealVectorStateSpace> q_start(space), q_goal(space);
                    for (int dim = 0; dim < dimension; ++dim) {
                        q_start->values[dim] = starts[index][static_cast<std::size_t>(dim)];
                        q_goal->values[dim] = goals[index][static_cast<std::size_t>(dim)];
                    }
                    if (use_prmstar) {
                        seeded_prmstar->addMilestoneFromState(q_start.get());
                        seeded_prmstar->addMilestoneFromState(q_goal.get());
                    } else {
                        seeded_prm->addMilestoneFromState(q_start.get());
                        seeded_prm->addMilestoneFromState(q_goal.get());
                    }
                }
            }
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
                    row["solve_s"] = 0.0;
                    row["simplify_s"] = 0.0;
                    row["t_s"] = 0.0;
                    row["path"] = std::vector<std::vector<double>>{};
                    query_results.append(row);
                    continue;
                }
                planner->clearQuery();
                set_problem_query(problem, space, si, starts[index], goals[index], dimension);
                const auto query_start = std::chrono::steady_clock::now();
                ob::ScopedState<ob::RealVectorStateSpace> q_start(space), q_goal(space);
                for (int dim = 0; dim < dimension; ++dim) {
                    q_start->values[dim] = starts[index][static_cast<std::size_t>(dim)];
                    q_goal->values[dim] = goals[index][static_cast<std::size_t>(dim)];
                }
                if (use_prmstar) {
                    seeded_prmstar->addMilestoneFromState(q_start.get());
                    seeded_prmstar->addMilestoneFromState(q_goal.get());
                } else {
                    seeded_prm->addMilestoneFromState(q_start.get());
                    seeded_prm->addMilestoneFromState(q_goal.get());
                }
                ob::PlannerStatus status = planner->solve(ob::timedPlannerTerminationCondition(std::max(0.0, query_budget_s)));
                bool ok = status == ob::PlannerStatus::EXACT_SOLUTION || static_cast<bool>(status);
                const double solve_s = ompl_elapsed_s(query_start);
                double simplify_s = 0.0;
                if (ok && simplify_time_s > 0.0) {
                    auto path_ptr = problem->getSolutionPath();
                    auto geometric_path = std::dynamic_pointer_cast<og::PathGeometric>(path_ptr);
                    if (geometric_path) {
                        const auto simplify_start = std::chrono::steady_clock::now();
                        og::PathSimplifier simplifier(si);
                        simplifier.simplify(*geometric_path, std::max(0.0, simplify_time_s));
                        simplify_s = ompl_elapsed_s(simplify_start);
                    }
                }
                auto path = ok ? path_from_problem_solution(problem, dimension) : std::vector<std::vector<double>>{};
                if (path.size() < 2) {
                    ok = false;
                }
                row["ok"] = ok;
                row["reason"] = ok ? "connected" : std::string(status.asString());
                row["status"] = std::string(status.asString());
                row["solve_s"] = solve_s;
                row["simplify_s"] = simplify_s;
                row["t_s"] = solve_s + simplify_s;
                row["path"] = path;
                query_results.append(row);
            }
            ob::PlannerData planner_data(si);
            planner->getPlannerData(planner_data);
            result["ok"] = true;
            result["planner"] = planner_kind == "prmstar" || planner_kind == "PRMstar" || planner_kind == "PRM*" ? "OMPL_PRMstar" : "OMPL_PRM";
            result["build_s"] = build_s;
            result["nodes"] = static_cast<int>(planner_data.numVertices());
            result["checking_resolution"] = checking_resolution;
            result["preload_query_endpoints"] = preload_query_endpoints;
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
        py::arg("max_nearest_neighbors") = 32,
        py::arg("planner_kind") = "prm",
        py::arg("preload_query_endpoints") = false);

    module.def("ompl_prm_multiquery_cumulative",
        [](const rbf::Robot& robot,
           const std::vector<rbf::Obstacle>& obstacles,
           const std::vector<std::vector<double>>& starts,
           const std::vector<std::vector<double>>& goals,
           const std::vector<double>& build_checkpoints_s,
           double query_budget_s,
           double segment_step,
           double simplify_time_s,
           int seed,
           int max_nearest_neighbors,
           const std::string& planner_kind,
           bool preload_query_endpoints,
           int early_stop_success_stall_checkpoints,
           double early_stop_path_rel_tol,
           double unresolved_query_retry_interval_s,
           double solved_query_recheck_interval_s) {
            namespace ob = ompl::base;
            namespace og = ompl::geometric;
            ompl::msg::setLogLevel(ompl::msg::LOG_ERROR);
            const auto base_seed = normalize_seed(seed);
            py::dict result;
            const int dimension = robot.n_joints();
            if (starts.empty() || starts.size() != goals.size()) {
                throw std::invalid_argument("starts/goals must be non-empty and have the same length");
            }
            if (build_checkpoints_s.empty()) {
                throw std::invalid_argument("build_checkpoints_s must be non-empty");
            }
            double previous_checkpoint = 0.0;
            std::vector<double> checkpoints;
            checkpoints.reserve(build_checkpoints_s.size());
            for (double checkpoint : build_checkpoints_s) {
                if (!std::isfinite(checkpoint) || checkpoint <= 0.0) {
                    throw std::invalid_argument("build checkpoints must be finite positive seconds");
                }
                if (checkpoint < previous_checkpoint) {
                    throw std::invalid_argument("build checkpoints must be sorted nondecreasing");
                }
                checkpoints.push_back(checkpoint);
                previous_checkpoint = checkpoint;
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
            configure_deterministic_state_sampler(space, mix_seed(base_seed, 0x50524D54U));
            auto si = make_ompl_space_information(robot, obstacles, space, segment_step, checking_resolution);
            auto problem = std::make_shared<ob::ProblemDefinition>(si);
            set_problem_query(problem, space, si, starts.front(), goals.front(), dimension);
            std::shared_ptr<og::PRM> planner;
            std::shared_ptr<SeededPRM> seeded_prm;
            std::shared_ptr<SeededPRMstar> seeded_prmstar;
            const bool use_prmstar = planner_kind == "prmstar" || planner_kind == "PRMstar" || planner_kind == "PRM*";
            if (use_prmstar) {
                seeded_prmstar = std::make_shared<SeededPRMstar>(si);
                seeded_prmstar->setLocalSeed(mix_seed(base_seed, 0x50524D51U));
                planner = seeded_prmstar;
            } else {
                seeded_prm = std::make_shared<SeededPRM>(si);
                seeded_prm->setLocalSeed(mix_seed(base_seed, 0x50524D51U));
                planner = seeded_prm;
            }
            if (!use_prmstar && max_nearest_neighbors > 0) {
                planner->setMaxNearestNeighbors(static_cast<unsigned int>(max_nearest_neighbors));
            }
            planner->setProblemDefinition(problem);
            planner->setup();

            auto add_query_milestones = [&](std::size_t index) {
                ob::ScopedState<ob::RealVectorStateSpace> q_start(space), q_goal(space);
                for (int dim = 0; dim < dimension; ++dim) {
                    q_start->values[dim] = starts[index][static_cast<std::size_t>(dim)];
                    q_goal->values[dim] = goals[index][static_cast<std::size_t>(dim)];
                }
                if (use_prmstar) {
                    seeded_prmstar->addMilestoneFromState(q_start.get());
                    seeded_prmstar->addMilestoneFromState(q_goal.get());
                } else {
                    seeded_prm->addMilestoneFromState(q_start.get());
                    seeded_prm->addMilestoneFromState(q_goal.get());
                }
            };
            std::vector<bool> milestones_inserted(starts.size(), false);
            if (preload_query_endpoints) {
                for (std::size_t index = 0; index < starts.size(); ++index) {
                    add_query_milestones(index);
                    milestones_inserted[index] = true;
                }
            }

            py::list stages;
            double cumulative_build_only_s = 0.0;
            double last_target_s = 0.0;
            double final_build_s = 0.0;
            int final_nodes = 0;
            std::vector<bool> has_incumbent(starts.size(), false);
            std::vector<double> best_lengths(starts.size(), std::numeric_limits<double>::infinity());
            std::vector<double> last_query_checkpoint_s(starts.size(), -std::numeric_limits<double>::infinity());
            int success_stall_checkpoints = 0;
            bool stopped_early = false;
            auto vector_path_length = [](const std::vector<std::vector<double>>& path) {
                if (path.size() < 2) {
                    return std::numeric_limits<double>::infinity();
                }
                double total = 0.0;
                for (std::size_t i = 1; i < path.size(); ++i) {
                    const auto& a = path[i - 1];
                    const auto& b = path[i];
                    double squared = 0.0;
                    const std::size_t dims = std::min(a.size(), b.size());
                    for (std::size_t dim = 0; dim < dims; ++dim) {
                        const double delta = b[dim] - a[dim];
                        squared += delta * delta;
                    }
                    total += std::sqrt(squared);
                }
                return total;
            };
            for (std::size_t stage_index = 0; stage_index < checkpoints.size(); ++stage_index) {
                const double target_s = checkpoints[stage_index];
                const double delta_s = std::max(0.0, target_s - last_target_s);
                const auto stage_build_start = std::chrono::steady_clock::now();
                planner->constructRoadmap(ob::timedPlannerTerminationCondition(delta_s));
                const double stage_build_delta_s = ompl_elapsed_s(stage_build_start);
                cumulative_build_only_s += stage_build_delta_s;
                const double cumulative_build_s = cumulative_build_only_s;
                last_target_s = target_s;

                ob::PlannerData planner_data(si);
                planner->getPlannerData(planner_data);
                final_build_s = cumulative_build_s;
                final_nodes = static_cast<int>(planner_data.numVertices());

                py::list query_results;
                bool stage_significant_improvement = false;
                bool stage_had_query = false;
                bool stage_had_incumbent_recheck = false;
                int stage_queries_executed = 0;
                for (std::size_t index = 0; index < starts.size(); ++index) {
                    py::dict row;
                    row["index"] = static_cast<int>(index);
                    if (endpoint_collision(starts[index]) || endpoint_collision(goals[index])) {
                        row["ok"] = false;
                        row["reason"] = "endpoint_collision";
                        row["status"] = "endpoint_collision";
                        row["solve_s"] = 0.0;
                        row["simplify_s"] = 0.0;
                        row["t_s"] = 0.0;
                        row["path"] = std::vector<std::vector<double>>{};
                        query_results.append(row);
                        continue;
                    }
                    const bool is_final_checkpoint = (stage_index + 1 == checkpoints.size());
                    const double retry_interval = has_incumbent[index]
                        ? std::max(0.0, solved_query_recheck_interval_s)
                        : std::max(0.0, unresolved_query_retry_interval_s);
                    const bool query_due =
                        is_final_checkpoint ||
                        !std::isfinite(last_query_checkpoint_s[index]) ||
                        target_s - last_query_checkpoint_s[index] >= retry_interval - 1e-12;
                    if (!query_due) {
                        row["ok"] = false;
                        row["reason"] = "skipped_between_trace_queries";
                        row["status"] = "skipped_between_trace_queries";
                        row["solve_s"] = 0.0;
                        row["simplify_s"] = 0.0;
                        row["t_s"] = 0.0;
                        row["path"] = std::vector<std::vector<double>>{};
                        query_results.append(row);
                        continue;
                    }
                    stage_had_query = true;
                    if (has_incumbent[index]) {
                        stage_had_incumbent_recheck = true;
                    }
                    ++stage_queries_executed;
                    last_query_checkpoint_s[index] = target_s;
                    planner->clearQuery();
                    set_problem_query(problem, space, si, starts[index], goals[index], dimension);
                    if (!milestones_inserted[index]) {
                        add_query_milestones(index);
                        milestones_inserted[index] = true;
                    }
                    const auto query_start = std::chrono::steady_clock::now();
                    ob::PlannerStatus status = planner->solve(ob::timedPlannerTerminationCondition(std::max(0.0, query_budget_s)));
                    bool ok = status == ob::PlannerStatus::EXACT_SOLUTION || static_cast<bool>(status);
                    const double solve_s = ompl_elapsed_s(query_start);
                    double simplify_s = 0.0;
                    if (ok && simplify_time_s > 0.0) {
                        auto path_ptr = problem->getSolutionPath();
                        auto geometric_path = std::dynamic_pointer_cast<og::PathGeometric>(path_ptr);
                        if (geometric_path) {
                            const auto simplify_start = std::chrono::steady_clock::now();
                            og::PathSimplifier simplifier(si);
                            simplifier.simplify(*geometric_path, std::max(0.0, simplify_time_s));
                            simplify_s = ompl_elapsed_s(simplify_start);
                        }
                    }
                    auto path = ok ? path_from_problem_solution(problem, dimension) : std::vector<std::vector<double>>{};
                    if (path.size() < 2) {
                        ok = false;
                    }
                    if (ok) {
                        const double length = vector_path_length(path);
                        const bool first_solution = !has_incumbent[index];
                        const double previous = best_lengths[index];
                        if (first_solution || length < previous) {
                            if (first_solution ||
                                length < previous * (1.0 - std::max(0.0, early_stop_path_rel_tol))) {
                                stage_significant_improvement = true;
                            }
                            best_lengths[index] = std::min(previous, length);
                            has_incumbent[index] = true;
                        }
                    }
                    row["ok"] = ok;
                    row["reason"] = ok ? "connected" : std::string(status.asString());
                    row["status"] = std::string(status.asString());
                    row["solve_s"] = solve_s;
                    row["simplify_s"] = simplify_s;
                    row["t_s"] = solve_s + simplify_s;
                    row["path"] = path;
                    query_results.append(row);
                }

                py::dict stage;
                stage["stage_index"] = static_cast<int>(stage_index);
                stage["checkpoint_s"] = target_s;
                stage["stage_build_delta_s"] = stage_build_delta_s;
                stage["build_s"] = cumulative_build_s;
                stage["nodes"] = static_cast<int>(planner_data.numVertices());
                stage["queries"] = query_results;
                const bool all_success = std::all_of(
                    has_incumbent.begin(),
                    has_incumbent.end(),
                    [](bool value) { return value; });
                if (all_success) {
                    if (stage_significant_improvement) {
                        success_stall_checkpoints = 0;
                    } else if (stage_had_incumbent_recheck) {
                        ++success_stall_checkpoints;
                    }
                } else {
                    success_stall_checkpoints = 0;
                }
                stage["all_queries_have_incumbent"] = all_success;
                stage["significant_improvement"] = stage_significant_improvement;
                stage["success_stall_checkpoints"] = success_stall_checkpoints;
                stage["queries_executed"] = stage_queries_executed;
                stage["had_query"] = stage_had_query;
                stage["had_incumbent_recheck"] = stage_had_incumbent_recheck;
                stage["early_stop"] = false;
                stages.append(stage);
                if (early_stop_success_stall_checkpoints > 0 &&
                    all_success &&
                    success_stall_checkpoints >= early_stop_success_stall_checkpoints) {
                    stage["early_stop"] = true;
                    stopped_early = true;
                    break;
                }
            }
            result["ok"] = true;
            result["planner"] = use_prmstar ? "OMPL_PRMstar" : "OMPL_PRM";
            result["cumulative"] = true;
            result["build_s"] = final_build_s;
            result["nodes"] = final_nodes;
            result["stopped_early"] = stopped_early;
            result["early_stop_success_stall_checkpoints"] = early_stop_success_stall_checkpoints;
            result["early_stop_path_rel_tol"] = early_stop_path_rel_tol;
            result["unresolved_query_retry_interval_s"] = unresolved_query_retry_interval_s;
            result["solved_query_recheck_interval_s"] = solved_query_recheck_interval_s;
            result["checking_resolution"] = checking_resolution;
            result["preload_query_endpoints"] = preload_query_endpoints;
            result["stages"] = stages;
            return result;
        },
        py::arg("robot"),
        py::arg("obstacles"),
        py::arg("starts"),
        py::arg("goals"),
        py::arg("build_checkpoints_s"),
        py::arg("query_budget_s") = 2.0,
        py::arg("segment_step") = 0.05,
        py::arg("simplify_time_s") = 0.1,
        py::arg("seed") = 42,
        py::arg("max_nearest_neighbors") = 32,
        py::arg("planner_kind") = "prm",
        py::arg("preload_query_endpoints") = false,
        py::arg("early_stop_success_stall_checkpoints") = 0,
        py::arg("early_stop_path_rel_tol") = 0.0,
        py::arg("unresolved_query_retry_interval_s") = 0.0,
        py::arg("solved_query_recheck_interval_s") = 0.0);

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

    module.def("ompl_lazy_prm_multiquery",
        [](const rbf::Robot& robot,
           const std::vector<rbf::Obstacle>& obstacles,
           const std::vector<std::vector<double>>& starts,
           const std::vector<std::vector<double>>& goals,
           double query_budget_s,
           double segment_step,
           double simplify_time_s,
           int seed,
           int max_nearest_neighbors,
           double range) {
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
            configure_deterministic_state_sampler(space, mix_seed(base_seed, 0x4C50524DU));
            auto si = make_ompl_space_information(robot, obstacles, space, segment_step, checking_resolution);
            auto problem = std::make_shared<ob::ProblemDefinition>(si);
            set_problem_query(problem, space, si, starts.front(), goals.front(), dimension);
            auto planner = std::make_shared<og::LazyPRM>(si);
            if (max_nearest_neighbors > 0) {
                planner->setMaxNearestNeighbors(static_cast<unsigned int>(max_nearest_neighbors));
            }
            if (range > 0.0) {
                planner->setRange(range);
            }
            planner->setProblemDefinition(problem);
            planner->setup();

            const auto total_start = std::chrono::steady_clock::now();
            py::list query_results;
            for (std::size_t index = 0; index < starts.size(); ++index) {
                py::dict row;
                row["index"] = static_cast<int>(index);
                if (endpoint_collision(starts[index]) || endpoint_collision(goals[index])) {
                    row["ok"] = false;
                    row["reason"] = "endpoint_collision";
                    row["status"] = "endpoint_collision";
                    row["solve_s"] = 0.0;
                    row["simplify_s"] = 0.0;
                    row["t_s"] = 0.0;
                    row["path"] = std::vector<std::vector<double>>{};
                    query_results.append(row);
                    continue;
                }
                set_problem_query(problem, space, si, starts[index], goals[index], dimension);
                if (index > 0) {
                    planner->clearQuery();
                }
                planner->setProblemDefinition(problem);
                planner->setup();
                const auto query_start = std::chrono::steady_clock::now();
                ob::PlannerStatus status = planner->solve(ob::timedPlannerTerminationCondition(std::max(0.0, query_budget_s)));
                bool ok = status == ob::PlannerStatus::EXACT_SOLUTION || static_cast<bool>(status);
                const double solve_s = ompl_elapsed_s(query_start);
                double simplify_s = 0.0;
                if (ok && simplify_time_s > 0.0) {
                    auto path_ptr = problem->getSolutionPath();
                    auto geometric_path = std::dynamic_pointer_cast<og::PathGeometric>(path_ptr);
                    if (geometric_path) {
                        const auto simplify_start = std::chrono::steady_clock::now();
                        og::PathSimplifier simplifier(si);
                        simplifier.simplify(*geometric_path, std::max(0.0, simplify_time_s));
                        simplify_s = ompl_elapsed_s(simplify_start);
                    }
                }
                auto path = ok ? path_from_problem_solution(problem, dimension) : std::vector<std::vector<double>>{};
                if (path.size() < 2) {
                    ok = false;
                }
                row["ok"] = ok;
                row["reason"] = ok ? "connected" : std::string(status.asString());
                row["status"] = std::string(status.asString());
                row["solve_s"] = solve_s;
                row["simplify_s"] = simplify_s;
                row["t_s"] = solve_s + simplify_s;
                row["path"] = path;
                query_results.append(row);
            }
            ob::PlannerData planner_data(si);
            planner->getPlannerData(planner_data);
            result["ok"] = true;
            result["planner"] = "OMPL_LazyPRM";
            result["build_s"] = 0.0;
            result["total_s"] = ompl_elapsed_s(total_start);
            result["nodes"] = static_cast<int>(planner_data.numVertices());
            result["checking_resolution"] = checking_resolution;
            result["queries"] = query_results;
            return result;
        },
        py::arg("robot"),
        py::arg("obstacles"),
        py::arg("starts"),
        py::arg("goals"),
        py::arg("query_budget_s") = 2.0,
        py::arg("segment_step") = 0.05,
        py::arg("simplify_time_s") = 0.1,
        py::arg("seed") = 42,
        py::arg("max_nearest_neighbors") = 32,
        py::arg("range") = 0.0);

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
           bool stop_on_solution_improvement,
           int use_k_nearest,
           int pruning,
           double prune_threshold_fraction,
           int delay_rewiring_until_initial_solution,
           int just_in_time_sampling,
           int drop_samples_on_prune,
           int approximate_solutions,
           int strict_queue_ordering,
           int cascading_rewirings,
           double initial_inflation_factor,
           double inflation_scaling_parameter,
           double truncation_scaling_parameter,
           int allowed_failed_sampling_attempts) {
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
                const double solve_s = ompl_elapsed_s(t0);
                result["ok"] = false;
                result["reason"] = "endpoint_collision";
                result["status"] = "endpoint_collision";
                result["solve_s"] = solve_s;
                result["simplify_s"] = 0.0;
                result["t_s"] = solve_s;
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
            configure_bitstar_planner(
                planner,
                samples_per_batch,
                rewire_factor,
                stop_on_solution_improvement,
                use_k_nearest,
                pruning,
                prune_threshold_fraction,
                delay_rewiring_until_initial_solution,
                just_in_time_sampling,
                drop_samples_on_prune,
                approximate_solutions,
                strict_queue_ordering,
                cascading_rewirings,
                initial_inflation_factor,
                inflation_scaling_parameter,
                truncation_scaling_parameter,
                allowed_failed_sampling_attempts);
            setup.setPlanner(planner);
            const double timeout_s = std::max(0.0, timeout_ms) / 1000.0;
            const double solve_slice_s = 0.10;
            ob::PlannerStatus status = ob::PlannerStatus::UNKNOWN;
            bool exact_solution = false;
            while (ompl_elapsed_s(t0) < timeout_s - 1e-6) {
                const double remaining_s = std::max(1e-5, timeout_s - ompl_elapsed_s(t0));
                const double slice_s = std::min(solve_slice_s, remaining_s);
                const double ptc_interval_s = std::max(1e-3, std::min(0.01, slice_s / 10.0));
                const auto before_s = ompl_elapsed_s(t0);
                status = setup.solve(ob::timedPlannerTerminationCondition(slice_s, ptc_interval_s));
                exact_solution = exact_solution || status == ob::PlannerStatus::EXACT_SOLUTION;
                if (stop_on_solution_improvement && setup.haveSolutionPath()) {
                    break;
                }
                // Guard against planners that do not make observable progress or overshoot a slice.
                if (ompl_elapsed_s(t0) <= before_s + 1e-7) {
                    break;
                }
            }
            bool ok = exact_solution || setup.haveSolutionPath();
            const double solve_s = ompl_elapsed_s(t0);
            double simplify_s = 0.0;
            if (ok && simplify_time_s > 0.0) {
                const auto simplify_start = std::chrono::steady_clock::now();
                setup.simplifySolution(simplify_time_s);
                simplify_s = ompl_elapsed_s(simplify_start);
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
            result["ok"] = ok;
            result["reason"] = ok ? "connected" : std::string(status.asString());
            result["status"] = std::string(status.asString());
            result["exact_solution"] = exact_solution;
            result["solve_s"] = solve_s;
            result["simplify_s"] = simplify_s;
            result["t_s"] = solve_s + simplify_s;
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
        py::arg("stop_on_solution_improvement") = true,
        py::arg("use_k_nearest") = -1,
        py::arg("pruning") = -1,
        py::arg("prune_threshold_fraction") = -1.0,
        py::arg("delay_rewiring_until_initial_solution") = -1,
        py::arg("just_in_time_sampling") = -1,
        py::arg("drop_samples_on_prune") = -1,
        py::arg("approximate_solutions") = -1,
        py::arg("strict_queue_ordering") = -1,
        py::arg("cascading_rewirings") = -1,
        py::arg("initial_inflation_factor") = -1.0,
        py::arg("inflation_scaling_parameter") = -1.0,
        py::arg("truncation_scaling_parameter") = -1.0,
        py::arg("allowed_failed_sampling_attempts") = -1);

    module.def("ompl_cspace_rrt_connect_path",
        [](const std::vector<std::vector<double>>& limits,
           const std::vector<std::vector<double>>& obstacles,
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
            const int dimension = validate_cspace_limits(limits);
            validate_cspace_vector(start, dimension, "start");
            validate_cspace_vector(goal, dimension, "goal");
            const auto t0 = std::chrono::steady_clock::now();
            if (!cspace_vector_valid(start, dimension, obstacles) || !cspace_vector_valid(goal, dimension, obstacles)) {
                const double solve_s = ompl_elapsed_s(t0);
                result["ok"] = false;
                result["reason"] = "endpoint_collision";
                result["status"] = "endpoint_collision";
                result["solve_s"] = solve_s;
                result["simplify_s"] = 0.0;
                result["t_s"] = solve_s;
                result["path"] = std::vector<std::vector<double>>{};
                result["nodes"] = 0;
                return result;
            }

            const auto base_seed = normalize_seed(seed);
            auto space = make_cspace_ompl_space(limits);
            configure_deterministic_state_sampler(space, mix_seed(base_seed, 0x43525253U));
            og::SimpleSetup setup(space);
            setup.setStateValidityChecker([obstacles, dimension](const ob::State* state) {
                const auto* vector_state = state->as<ob::RealVectorStateSpace::StateType>();
                return cspace_point_valid(vector_state->values, dimension, obstacles);
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
            planner->setLocalSeed(mix_seed(base_seed, 0x43525250U));
            if (range > 0.0) {
                planner->setRange(range);
            }
            setup.setPlanner(planner);

            const double timeout_s = std::max(0.0, timeout_ms) / 1000.0;
            const double ptc_interval_s = std::max(1e-3, std::min(0.05, timeout_s / 20.0));
            ob::PlannerStatus status = setup.solve(ob::timedPlannerTerminationCondition(timeout_s, ptc_interval_s));
            const double solve_s = ompl_elapsed_s(t0);
            const bool exact_solution = status == ob::PlannerStatus::EXACT_SOLUTION;
            bool ok = exact_solution;
            double simplify_s = 0.0;
            if (ok && simplify_time_s > 0.0) {
                const auto simplify_start = std::chrono::steady_clock::now();
                setup.simplifySolution(simplify_time_s);
                simplify_s = ompl_elapsed_s(simplify_start);
            }
            std::vector<std::vector<double>> path;
            if (ok && setup.haveSolutionPath()) {
                path = path_from_problem_solution(setup.getProblemDefinition(), dimension);
                ok = path.size() >= 2;
            } else {
                ok = false;
            }
            ob::PlannerData planner_data(setup.getSpaceInformation());
            planner->getPlannerData(planner_data);
            result["ok"] = ok;
            result["reason"] = ok ? "connected" : std::string(status.asString());
            result["status"] = std::string(status.asString());
            result["exact_solution"] = exact_solution;
            result["solve_s"] = solve_s;
            result["simplify_s"] = simplify_s;
            result["t_s"] = solve_s + simplify_s;
            result["path"] = path;
            result["nodes"] = static_cast<int>(planner_data.numVertices());
            result["checking_resolution"] = checking_resolution;
            result["planner"] = "OMPL_CSpace_RRTConnect";
            return result;
        },
        py::arg("limits"),
        py::arg("obstacles"),
        py::arg("start"),
        py::arg("goal"),
        py::arg("timeout_ms") = 1000.0,
        py::arg("range") = 0.35,
        py::arg("segment_step") = 0.01,
        py::arg("simplify_time_s") = 0.0,
        py::arg("seed") = 42);

    module.def("ompl_cspace_prm_multiquery",
        [](const std::vector<std::vector<double>>& limits,
           const std::vector<std::vector<double>>& obstacles,
           const std::vector<std::vector<double>>& starts,
           const std::vector<std::vector<double>>& goals,
           double build_budget_s,
           double query_budget_s,
           double segment_step,
           double simplify_time_s,
           int seed,
           int max_nearest_neighbors,
           const std::string& planner_kind,
           bool preload_query_endpoints) {
            namespace ob = ompl::base;
            namespace og = ompl::geometric;
            ompl::msg::setLogLevel(ompl::msg::LOG_ERROR);
            const auto base_seed = normalize_seed(seed);
            py::dict result;
            const int dimension = validate_cspace_limits(limits);
            if (starts.empty() || starts.size() != goals.size()) {
                throw std::invalid_argument("starts/goals must be non-empty and have the same length");
            }
            for (std::size_t index = 0; index < starts.size(); ++index) {
                validate_cspace_vector(starts[index], dimension, "start");
                validate_cspace_vector(goals[index], dimension, "goal");
            }
            auto endpoint_collision = [&](const std::vector<double>& q) {
                return !cspace_vector_valid(q, dimension, obstacles);
            };
            auto space = make_cspace_ompl_space(limits);
            configure_deterministic_state_sampler(space, mix_seed(base_seed, 0x43505253U));
            auto si = std::make_shared<ob::SpaceInformation>(space);
            si->setStateValidityChecker([obstacles, dimension](const ob::State* state) {
                const auto* vector_state = state->as<ob::RealVectorStateSpace::StateType>();
                return cspace_point_valid(vector_state->values, dimension, obstacles);
            });
            const double maximum_extent = std::max(space->getMaximumExtent(), 1e-9);
            const double checking_resolution = std::clamp(std::max(segment_step, 1e-6) / maximum_extent, 1e-6, 0.05);
            si->setStateValidityCheckingResolution(checking_resolution);
            si->setup();
            auto problem = std::make_shared<ob::ProblemDefinition>(si);
            set_problem_query(problem, space, si, starts.front(), goals.front(), dimension);
            std::shared_ptr<og::PRM> planner;
            std::shared_ptr<SeededPRM> seeded_prm;
            std::shared_ptr<SeededPRMstar> seeded_prmstar;
            const bool use_prmstar = planner_kind == "prmstar" || planner_kind == "PRMstar" || planner_kind == "PRM*";
            if (use_prmstar) {
                seeded_prmstar = std::make_shared<SeededPRMstar>(si);
                seeded_prmstar->setLocalSeed(mix_seed(base_seed, 0x43505250U));
                planner = seeded_prmstar;
            } else {
                seeded_prm = std::make_shared<SeededPRM>(si);
                seeded_prm->setLocalSeed(mix_seed(base_seed, 0x43505250U));
                planner = seeded_prm;
            }
            if (!use_prmstar && max_nearest_neighbors > 0) {
                planner->setMaxNearestNeighbors(static_cast<unsigned int>(max_nearest_neighbors));
            }
            planner->setProblemDefinition(problem);
            planner->setup();

            const auto build_start = std::chrono::steady_clock::now();
            if (preload_query_endpoints) {
                for (std::size_t index = 0; index < starts.size(); ++index) {
                    ob::ScopedState<ob::RealVectorStateSpace> q_start(space), q_goal(space);
                    for (int dim = 0; dim < dimension; ++dim) {
                        q_start->values[dim] = starts[index][static_cast<std::size_t>(dim)];
                        q_goal->values[dim] = goals[index][static_cast<std::size_t>(dim)];
                    }
                    if (use_prmstar) {
                        seeded_prmstar->addMilestoneFromState(q_start.get());
                        seeded_prmstar->addMilestoneFromState(q_goal.get());
                    } else {
                        seeded_prm->addMilestoneFromState(q_start.get());
                        seeded_prm->addMilestoneFromState(q_goal.get());
                    }
                }
            }
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
                    row["solve_s"] = 0.0;
                    row["simplify_s"] = 0.0;
                    row["t_s"] = 0.0;
                    row["path"] = std::vector<std::vector<double>>{};
                    query_results.append(row);
                    continue;
                }
                planner->clearQuery();
                set_problem_query(problem, space, si, starts[index], goals[index], dimension);
                const auto query_start = std::chrono::steady_clock::now();
                ob::ScopedState<ob::RealVectorStateSpace> q_start(space), q_goal(space);
                for (int dim = 0; dim < dimension; ++dim) {
                    q_start->values[dim] = starts[index][static_cast<std::size_t>(dim)];
                    q_goal->values[dim] = goals[index][static_cast<std::size_t>(dim)];
                }
                if (use_prmstar) {
                    seeded_prmstar->addMilestoneFromState(q_start.get());
                    seeded_prmstar->addMilestoneFromState(q_goal.get());
                } else {
                    seeded_prm->addMilestoneFromState(q_start.get());
                    seeded_prm->addMilestoneFromState(q_goal.get());
                }
                ob::PlannerStatus status = planner->solve(ob::timedPlannerTerminationCondition(std::max(0.0, query_budget_s)));
                bool ok = status == ob::PlannerStatus::EXACT_SOLUTION || static_cast<bool>(status);
                const double solve_s = ompl_elapsed_s(query_start);
                double simplify_s = 0.0;
                if (ok && simplify_time_s > 0.0) {
                    auto path_ptr = problem->getSolutionPath();
                    auto geometric_path = std::dynamic_pointer_cast<og::PathGeometric>(path_ptr);
                    if (geometric_path) {
                        const auto simplify_start = std::chrono::steady_clock::now();
                        og::PathSimplifier simplifier(si);
                        simplifier.simplify(*geometric_path, std::max(0.0, simplify_time_s));
                        simplify_s = ompl_elapsed_s(simplify_start);
                    }
                }
                auto path = ok ? path_from_problem_solution(problem, dimension) : std::vector<std::vector<double>>{};
                if (path.size() < 2) {
                    ok = false;
                }
                row["ok"] = ok;
                row["reason"] = ok ? "connected" : std::string(status.asString());
                row["status"] = std::string(status.asString());
                row["solve_s"] = solve_s;
                row["simplify_s"] = simplify_s;
                row["t_s"] = solve_s + simplify_s;
                row["path"] = path;
                query_results.append(row);
            }
            ob::PlannerData planner_data(si);
            planner->getPlannerData(planner_data);
            result["ok"] = true;
            result["planner"] = use_prmstar ? "OMPL_CSpace_PRMstar" : "OMPL_CSpace_PRM";
            result["build_s"] = build_s;
            result["nodes"] = static_cast<int>(planner_data.numVertices());
            result["checking_resolution"] = checking_resolution;
            result["preload_query_endpoints"] = preload_query_endpoints;
            result["queries"] = query_results;
            return result;
        },
        py::arg("limits"),
        py::arg("obstacles"),
        py::arg("starts"),
        py::arg("goals"),
        py::arg("build_budget_s") = 2.0,
        py::arg("query_budget_s") = 4.0,
        py::arg("segment_step") = 0.01,
        py::arg("simplify_time_s") = 0.01,
        py::arg("seed") = 42,
        py::arg("max_nearest_neighbors") = 128,
        py::arg("planner_kind") = "prm",
        py::arg("preload_query_endpoints") = true);

    module.def("ompl_cspace_bitstar_trace",
        [](const std::vector<std::vector<double>>& limits,
           const std::vector<std::vector<double>>& obstacles,
           const std::vector<double>& start,
           const std::vector<double>& goal,
           double timeout_ms,
           double checkpoint_interval_ms,
           double segment_step,
           int seed,
           int samples_per_batch,
           double rewire_factor,
           bool stop_on_solution_improvement,
           int use_k_nearest,
           int pruning,
           double prune_threshold_fraction,
           int delay_rewiring_until_initial_solution,
           int just_in_time_sampling,
           int drop_samples_on_prune,
           int approximate_solutions,
           int strict_queue_ordering,
           int cascading_rewirings,
           double initial_inflation_factor,
           double inflation_scaling_parameter,
           double truncation_scaling_parameter,
           int allowed_failed_sampling_attempts) {
            namespace ob = ompl::base;
            namespace og = ompl::geometric;
            ompl::msg::setLogLevel(ompl::msg::LOG_ERROR);
            const int dimension = validate_cspace_limits(limits);
            validate_cspace_vector(start, dimension, "start");
            validate_cspace_vector(goal, dimension, "goal");
            const auto t0 = std::chrono::steady_clock::now();
            py::dict result;
            py::list checkpoints;
            if (!cspace_vector_valid(start, dimension, obstacles) || !cspace_vector_valid(goal, dimension, obstacles)) {
                const double solve_s = ompl_elapsed_s(t0);
                result["ok"] = false;
                result["reason"] = "endpoint_collision";
                result["status"] = "endpoint_collision";
                result["solve_s"] = solve_s;
                result["simplify_s"] = 0.0;
                result["t_s"] = solve_s;
                result["path"] = std::vector<std::vector<double>>{};
                result["nodes"] = 0;
                result["checkpoints"] = checkpoints;
                return result;
            }

            const auto base_seed = normalize_seed(seed);
            auto space = make_cspace_ompl_space(limits);
            configure_deterministic_state_sampler(space, mix_seed(base_seed, 0x43424953U));
            og::SimpleSetup setup(space);
            auto si = setup.getSpaceInformation();
            si->setStateValidityChecker([obstacles, dimension](const ob::State* state) {
                const auto* vector_state = state->as<ob::RealVectorStateSpace::StateType>();
                return cspace_point_valid(vector_state->values, dimension, obstacles);
            });
            const double maximum_extent = std::max(space->getMaximumExtent(), 1e-9);
            const double checking_resolution = std::clamp(std::max(segment_step, 1e-6) / maximum_extent, 1e-6, 0.05);
            si->setStateValidityCheckingResolution(checking_resolution);

            ob::ScopedState<ob::RealVectorStateSpace> q_start(space), q_goal(space);
            for (int dim = 0; dim < dimension; ++dim) {
                q_start->values[dim] = start[static_cast<std::size_t>(dim)];
                q_goal->values[dim] = goal[static_cast<std::size_t>(dim)];
            }
            setup.setStartAndGoalStates(q_start, q_goal);
            setup.setOptimizationObjective(std::make_shared<ob::PathLengthOptimizationObjective>(si));
            auto planner = std::make_shared<SeededBITstar>(si);
            planner->setLocalSeed(mix_seed(base_seed, 0x43424950U));
            configure_bitstar_planner(
                planner,
                samples_per_batch,
                rewire_factor,
                stop_on_solution_improvement,
                use_k_nearest,
                pruning,
                prune_threshold_fraction,
                delay_rewiring_until_initial_solution,
                just_in_time_sampling,
                drop_samples_on_prune,
                approximate_solutions,
                strict_queue_ordering,
                cascading_rewirings,
                initial_inflation_factor,
                inflation_scaling_parameter,
                truncation_scaling_parameter,
                allowed_failed_sampling_attempts);
            setup.setPlanner(planner);

            const double timeout_s = std::max(0.0, timeout_ms) / 1000.0;
            const double interval_s = std::max(1e-3, checkpoint_interval_ms / 1000.0);
            std::string last_status = "not_run";
            bool last_exact = false;
            bool trace_stopped_early = false;
            const int quality_stall_checkpoints = 0;
            const double quality_stall_rel_tol = 0.005;
            std::string trace_stop_reason = "";
            double best_path_length = std::numeric_limits<double>::infinity();
            int quality_stall_count = 0;

            auto vector_path_length = [](const std::vector<std::vector<double>>& path) {
                if (path.size() < 2) {
                    return std::numeric_limits<double>::infinity();
                }
                double total = 0.0;
                for (std::size_t i = 1; i < path.size(); ++i) {
                    const auto& a = path[i - 1];
                    const auto& b = path[i];
                    double squared = 0.0;
                    const std::size_t dims = std::min(a.size(), b.size());
                    for (std::size_t dim = 0; dim < dims; ++dim) {
                        const double delta = b[dim] - a[dim];
                        squared += delta * delta;
                    }
                    total += std::sqrt(squared);
                }
                return total;
            };

            auto append_checkpoint = [&](double target_s) {
                const double elapsed_s = ompl_elapsed_s(t0);
                std::vector<std::vector<double>> path;
                bool ok = false;
                if (setup.haveSolutionPath()) {
                    path = path_from_problem_solution(setup.getProblemDefinition(), dimension);
                    ok = path.size() >= 2;
                }
                const double length = ok ? vector_path_length(path) : std::numeric_limits<double>::infinity();
                py::dict row;
                row["checkpoint_s"] = target_s;
                row["elapsed_s"] = elapsed_s;
                row["ok"] = ok;
                row["reason"] = ok ? "connected" : last_status;
                row["status"] = ok ? "solution" : last_status;
                row["exact_solution"] = ok || last_exact;
                row["solve_s"] = elapsed_s;
                row["simplify_s"] = 0.0;
                row["t_s"] = elapsed_s;
                row["path"] = path;
                row["path_length"] = std::isfinite(length) ? length : std::numeric_limits<double>::quiet_NaN();
                row["nodes"] = 0;
                row["iterations"] = static_cast<int>(planner->numIterations());
                row["batches"] = static_cast<int>(planner->numBatches());
                checkpoints.append(row);
                return length;
            };

            auto update_quality_stall = [&](double length) {
                if (quality_stall_checkpoints <= 0 || !std::isfinite(length)) {
                    return false;
                }
                const double rel_tol = std::max(0.0, quality_stall_rel_tol);
                if (!std::isfinite(best_path_length) || length < best_path_length * (1.0 - rel_tol)) {
                    best_path_length = length;
                    quality_stall_count = 0;
                    return false;
                }
                ++quality_stall_count;
                return quality_stall_count >= quality_stall_checkpoints;
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
                        if (stop_on_solution_improvement && setup.haveSolutionPath()) {
                            trace_stopped_early = true;
                            break;
                        }
                    }
                    const double checkpoint_length = append_checkpoint(clamped_target_s);
                    if (update_quality_stall(checkpoint_length)) {
                        trace_stopped_early = true;
                        trace_stop_reason = "quality_stall";
                    }
                    if (trace_stopped_early) {
                        break;
                    }
                }
                while (!trace_stopped_early && ompl_elapsed_s(t0) < timeout_s - 1e-6) {
                    const double remaining_s = std::max(1e-5, timeout_s - ompl_elapsed_s(t0));
                    const double ptc_interval_s = std::max(1e-3, std::min(0.05, remaining_s / 20.0));
                    const auto before_s = ompl_elapsed_s(t0);
                    ob::PlannerStatus status = setup.solve(ob::timedPlannerTerminationCondition(remaining_s, ptc_interval_s));
                    last_status = std::string(status.asString());
                    last_exact = status == ob::PlannerStatus::EXACT_SOLUTION;
                    if (ompl_elapsed_s(t0) <= before_s + 1e-6) {
                        break;
                    }
                    if (stop_on_solution_improvement && setup.haveSolutionPath()) {
                        trace_stopped_early = true;
                        break;
                    }
                }
                if (checkpoints.empty()) {
                    append_checkpoint(timeout_s);
                } else if (!trace_stopped_early) {
                    append_checkpoint(timeout_s);
                }
            }

            std::vector<std::vector<double>> final_path;
            bool ok = false;
            if (setup.haveSolutionPath()) {
                final_path = path_from_problem_solution(setup.getProblemDefinition(), dimension);
                ok = final_path.size() >= 2;
            }
            const double solve_s = ompl_elapsed_s(t0);
            result["ok"] = ok;
            result["reason"] = ok ? "connected" : last_status;
            result["status"] = ok ? "solution" : last_status;
            result["exact_solution"] = ok || last_exact;
            result["solve_s"] = solve_s;
            result["simplify_s"] = 0.0;
            result["t_s"] = solve_s;
            result["path"] = final_path;
            result["nodes"] = 0;
            result["checking_resolution"] = checking_resolution;
            result["planner"] = "OMPL_CSpace_BITstar";
            result["iterations"] = static_cast<int>(planner->numIterations());
            result["batches"] = static_cast<int>(planner->numBatches());
            result["stopped_early"] = trace_stopped_early;
            result["stop_on_solution_improvement"] = stop_on_solution_improvement;
            result["checkpoints"] = checkpoints;
            return result;
        },
        py::arg("limits"),
        py::arg("obstacles"),
        py::arg("start"),
        py::arg("goal"),
        py::arg("timeout_ms") = 1000.0,
        py::arg("checkpoint_interval_ms") = 10.0,
        py::arg("segment_step") = 0.01,
        py::arg("seed") = 42,
        py::arg("samples_per_batch") = -1,
        py::arg("rewire_factor") = -1.0,
        py::arg("stop_on_solution_improvement") = false,
        py::arg("use_k_nearest") = -1,
        py::arg("pruning") = -1,
        py::arg("prune_threshold_fraction") = -1.0,
        py::arg("delay_rewiring_until_initial_solution") = -1,
        py::arg("just_in_time_sampling") = -1,
        py::arg("drop_samples_on_prune") = -1,
        py::arg("approximate_solutions") = -1,
        py::arg("strict_queue_ordering") = -1,
        py::arg("cascading_rewirings") = -1,
        py::arg("initial_inflation_factor") = -1.0,
        py::arg("inflation_scaling_parameter") = -1.0,
        py::arg("truncation_scaling_parameter") = -1.0,
        py::arg("allowed_failed_sampling_attempts") = -1);

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
           bool stop_on_solution_improvement,
           int quality_stall_checkpoints,
           double quality_stall_rel_tol,
           int use_k_nearest,
           int pruning,
           double prune_threshold_fraction,
           int delay_rewiring_until_initial_solution,
           int just_in_time_sampling,
           int drop_samples_on_prune,
           int approximate_solutions,
           int strict_queue_ordering,
           int cascading_rewirings,
           double initial_inflation_factor,
           double inflation_scaling_parameter,
           double truncation_scaling_parameter,
           int allowed_failed_sampling_attempts) {
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
                const double solve_s = ompl_elapsed_s(t0);
                result["ok"] = false;
                result["reason"] = "endpoint_collision";
                result["status"] = "endpoint_collision";
                result["solve_s"] = solve_s;
                result["simplify_s"] = 0.0;
                result["t_s"] = solve_s;
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
            configure_bitstar_planner(
                planner,
                samples_per_batch,
                rewire_factor,
                stop_on_solution_improvement,
                use_k_nearest,
                pruning,
                prune_threshold_fraction,
                delay_rewiring_until_initial_solution,
                just_in_time_sampling,
                drop_samples_on_prune,
                approximate_solutions,
                strict_queue_ordering,
                cascading_rewirings,
                initial_inflation_factor,
                inflation_scaling_parameter,
                truncation_scaling_parameter,
                allowed_failed_sampling_attempts);
            setup.setPlanner(planner);

            const double timeout_s = std::max(0.0, timeout_ms) / 1000.0;
            const double interval_s = std::max(1e-3, checkpoint_interval_ms / 1000.0);
            std::string last_status = "not_run";
            bool last_exact = false;
            bool trace_stopped_early = false;
            std::string trace_stop_reason = "";
            double best_path_length = std::numeric_limits<double>::infinity();
            int quality_stall_count = 0;

            auto vector_path_length = [](const std::vector<std::vector<double>>& path) {
                if (path.size() < 2) {
                    return std::numeric_limits<double>::infinity();
                }
                double total = 0.0;
                for (std::size_t i = 1; i < path.size(); ++i) {
                    const auto& a = path[i - 1];
                    const auto& b = path[i];
                    double squared = 0.0;
                    const std::size_t dims = std::min(a.size(), b.size());
                    for (std::size_t dim = 0; dim < dims; ++dim) {
                        const double delta = b[dim] - a[dim];
                        squared += delta * delta;
                    }
                    total += std::sqrt(squared);
                }
                return total;
            };

            auto append_checkpoint = [&](double target_s) {
                const double elapsed_s = ompl_elapsed_s(t0);
                std::vector<std::vector<double>> path;
                bool ok = false;
                if (setup.haveSolutionPath()) {
                    path = path_from_problem_solution(setup.getProblemDefinition(), dimension);
                    ok = path.size() >= 2;
                }
                const double length = ok ? vector_path_length(path) : std::numeric_limits<double>::infinity();
                py::dict row;
                row["checkpoint_s"] = target_s;
                row["elapsed_s"] = elapsed_s;
                row["ok"] = ok;
                row["reason"] = ok ? "connected" : last_status;
                row["status"] = ok ? "solution" : last_status;
                row["exact_solution"] = ok || last_exact;
                row["solve_s"] = elapsed_s;
                row["simplify_s"] = 0.0;
                row["t_s"] = elapsed_s;
                row["path"] = path;
                row["path_length"] = std::isfinite(length) ? length : std::numeric_limits<double>::quiet_NaN();
                row["nodes"] = 0;
                row["iterations"] = static_cast<int>(planner->numIterations());
                row["batches"] = static_cast<int>(planner->numBatches());
                checkpoints.append(row);
                return length;
            };

            auto update_quality_stall = [&](double length) {
                if (quality_stall_checkpoints <= 0 || !std::isfinite(length)) {
                    return false;
                }
                const double rel_tol = std::max(0.0, quality_stall_rel_tol);
                if (!std::isfinite(best_path_length) || length < best_path_length * (1.0 - rel_tol)) {
                    best_path_length = length;
                    quality_stall_count = 0;
                    return false;
                }
                ++quality_stall_count;
                return quality_stall_count >= quality_stall_checkpoints;
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
                        if (stop_on_solution_improvement && setup.haveSolutionPath()) {
                            trace_stopped_early = true;
                            trace_stop_reason = "first_solution";
                            break;
                        }
                    }
                    const double checkpoint_length = append_checkpoint(clamped_target_s);
                    if (update_quality_stall(checkpoint_length)) {
                        trace_stopped_early = true;
                        trace_stop_reason = "quality_stall";
                    }
                    if (trace_stopped_early) {
                        break;
                    }
                }
                while (!trace_stopped_early && ompl_elapsed_s(t0) < timeout_s - 1e-6) {
                    const double remaining_s = std::max(1e-5, timeout_s - ompl_elapsed_s(t0));
                    const double ptc_interval_s = std::max(1e-3, std::min(0.05, remaining_s / 20.0));
                    const auto before_s = ompl_elapsed_s(t0);
                    ob::PlannerStatus status = setup.solve(ob::timedPlannerTerminationCondition(remaining_s, ptc_interval_s));
                    last_status = std::string(status.asString());
                    last_exact = status == ob::PlannerStatus::EXACT_SOLUTION;
                    if (ompl_elapsed_s(t0) <= before_s + 1e-6) {
                        break;
                    }
                    if (stop_on_solution_improvement && setup.haveSolutionPath()) {
                        trace_stopped_early = true;
                        trace_stop_reason = "first_solution";
                        break;
                    }
                }
                if (checkpoints.empty() || !trace_stopped_early) {
                    const double checkpoint_length = append_checkpoint(timeout_s);
                    if (update_quality_stall(checkpoint_length)) {
                        trace_stopped_early = true;
                        trace_stop_reason = "quality_stall";
                    }
                }
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
            const double solve_s = ompl_elapsed_s(t0);
            result["solve_s"] = solve_s;
            result["simplify_s"] = 0.0;
            result["t_s"] = solve_s;
            result["path"] = final_path;
            result["nodes"] = 0;
            result["checking_resolution"] = checking_resolution;
            result["planner"] = "OMPL_BITstar";
            result["iterations"] = static_cast<int>(planner->numIterations());
            result["batches"] = static_cast<int>(planner->numBatches());
            result["stopped_early"] = trace_stopped_early;
            result["early_stop_reason"] = trace_stop_reason;
            result["stop_on_solution_improvement"] = stop_on_solution_improvement;
            result["quality_stall_checkpoints"] = quality_stall_checkpoints;
            result["quality_stall_rel_tol"] = quality_stall_rel_tol;
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
        py::arg("stop_on_solution_improvement") = false,
        py::arg("quality_stall_checkpoints") = 0,
        py::arg("quality_stall_rel_tol") = 0.005,
        py::arg("use_k_nearest") = -1,
        py::arg("pruning") = -1,
        py::arg("prune_threshold_fraction") = -1.0,
        py::arg("delay_rewiring_until_initial_solution") = -1,
        py::arg("just_in_time_sampling") = -1,
        py::arg("drop_samples_on_prune") = -1,
        py::arg("approximate_solutions") = -1,
        py::arg("strict_queue_ordering") = -1,
        py::arg("cascading_rewirings") = -1,
        py::arg("initial_inflation_factor") = -1.0,
        py::arg("inflation_scaling_parameter") = -1.0,
        py::arg("truncation_scaling_parameter") = -1.0,
        py::arg("allowed_failed_sampling_attempts") = -1);

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
