#pragma once

#include <SBF/sbf.h>

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
#include <ompl/geometric/planners/prm/LazyPRM.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/util/Console.h>
#include <ompl/util/RandomNumbers.h>

#include <Eigen/Core>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace rbf::python_binding {

inline std::vector<double> list_from_ompl_state(const ompl::base::State* state,
                                                int dimension) {
    const auto* vector_state = state->as<ompl::base::RealVectorStateSpace::StateType>();
    std::vector<double> out(static_cast<std::size_t>(dimension));
    for (int dim = 0; dim < dimension; ++dim) {
        out[static_cast<std::size_t>(dim)] = vector_state->values[dim];
    }
    return out;
}

inline std::uint_fast32_t normalize_seed(int seed) {
    return static_cast<std::uint_fast32_t>(std::max(0, seed));
}

inline std::uint_fast32_t mix_seed(std::uint_fast32_t base_seed,
                                   std::uint_fast32_t stream_id) {
    std::uint64_t z = static_cast<std::uint64_t>(base_seed) + 0x9E3779B97F4A7C15ULL;
    z ^= static_cast<std::uint64_t>(stream_id) + 0x9E3779B97F4A7C15ULL + (z << 6U) +
         (z >> 2U);
    z ^= (z >> 30U);
    z *= 0xBF58476D1CE4E5B9ULL;
    z ^= (z >> 27U);
    z *= 0x94D049BB133111EBULL;
    z ^= (z >> 31U);
    return static_cast<std::uint_fast32_t>(z & 0xFFFFFFFFULL);
}

inline void seed_state_sampler_tree(const ompl::base::StateSamplerPtr& sampler,
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
    DeterministicRealVectorStateSampler(const ompl::base::StateSpace* space,
                                        std::uint_fast32_t local_seed)
        : ompl::base::RealVectorStateSampler(space) {
        rng_.setLocalSeed(local_seed);
    }
};

inline void configure_deterministic_state_sampler(
    const std::shared_ptr<ompl::base::RealVectorStateSpace>& space,
    std::uint_fast32_t base_seed) {
    auto sampler_counter = std::make_shared<std::atomic<std::uint_fast32_t>>(0U);
    space->setStateSamplerAllocator(
        [base_seed, sampler_counter](const ompl::base::StateSpace* state_space)
            -> ompl::base::StateSamplerPtr {
            const auto sampler_index =
                sampler_counter->fetch_add(1U, std::memory_order_relaxed);
            return std::make_shared<DeterministicRealVectorStateSampler>(
                state_space,
                mix_seed(base_seed, sampler_index));
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
            auto direct_sampler =
                std::dynamic_pointer_cast<ompl::base::PathLengthDirectInfSampler>(
                    graphPtr_->sampler_);
            if (direct_sampler) {
                direct_sampler->rng_.setLocalSeed(mix_seed(local_seed_, 1U));
                std::uint_fast32_t sampler_stream_id = 2U;
                seed_state_sampler_tree(direct_sampler->baseSampler_,
                                        local_seed_,
                                        sampler_stream_id);
                seed_state_sampler_tree(direct_sampler->uninformedSubSampler_,
                                        local_seed_,
                                        sampler_stream_id);
            }
        }
    }

    std::uint_fast32_t local_seed_{0U};
};

inline double ompl_elapsed_s(const std::chrono::steady_clock::time_point& start) {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}

inline void configure_bitstar_planner(
    const std::shared_ptr<SeededBITstar>& planner,
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

inline std::shared_ptr<ompl::base::RealVectorStateSpace> make_ompl_space(
    const rbf::Robot& robot) {
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

inline int validate_cspace_limits(const std::vector<std::vector<double>>& limits) {
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

inline void validate_cspace_vector(const std::vector<double>& q,
                                   int dimension,
                                   const char* name) {
    if (static_cast<int>(q.size()) != dimension) {
        throw std::invalid_argument(std::string(name) + " dimension must match cspace limits");
    }
}

inline std::shared_ptr<ompl::base::RealVectorStateSpace> make_cspace_ompl_space(
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

inline bool cspace_point_in_box(const double* values,
                                int dimension,
                                const std::vector<double>& flattened_box) {
    if (static_cast<int>(flattened_box.size()) != 2 * dimension) {
        throw std::invalid_argument(
            "each cspace obstacle must be flattened [lo0, hi0, lo1, hi1, ...]");
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

inline bool cspace_point_valid(
    const double* values,
    int dimension,
    const std::vector<std::vector<double>>& flattened_obstacles) {
    for (const auto& obstacle : flattened_obstacles) {
        if (cspace_point_in_box(values, dimension, obstacle)) {
            return false;
        }
    }
    return true;
}

inline bool cspace_vector_valid(
    const std::vector<double>& q,
    int dimension,
    const std::vector<std::vector<double>>& flattened_obstacles) {
    validate_cspace_vector(q, dimension, "q");
    return cspace_point_valid(q.data(), dimension, flattened_obstacles);
}

inline std::shared_ptr<ompl::base::SpaceInformation> make_ompl_space_information(
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
    checking_resolution =
        std::clamp(std::max(segment_step, 1e-6) / maximum_extent, 1e-6, 0.05);
    si->setStateValidityCheckingResolution(checking_resolution);
    si->setup();
    return si;
}

inline void set_problem_query(
    const std::shared_ptr<ompl::base::ProblemDefinition>& problem,
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

inline std::vector<std::vector<double>> path_from_problem_solution(
    const std::shared_ptr<ompl::base::ProblemDefinition>& problem,
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

inline std::vector<std::vector<double>> path_from_geometric_path(
    const ompl::geometric::PathGeometric& geometric_path,
    int dimension) {
    std::vector<std::vector<double>> path;
    path.reserve(geometric_path.getStateCount());
    for (std::size_t index = 0; index < geometric_path.getStateCount(); ++index) {
        path.push_back(list_from_ompl_state(geometric_path.getState(index), dimension));
    }
    return path;
}

} // namespace rbf::python_binding
