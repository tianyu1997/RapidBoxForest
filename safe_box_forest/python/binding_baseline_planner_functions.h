#pragma once

#include <SBF/sbf.h>

#include "binding_baseline_rrt_functions.h"
#include "binding_baseline_prm_functions.h"
#include "binding_utils.h"
#include "ompl_binding_utils.h"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
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

inline void register_baseline_planner_functions(py::module_& module) {
    register_baseline_rrt_functions(module);

    register_baseline_prm_functions(module);

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

}  // namespace rbf::python_binding
