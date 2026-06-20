#pragma once

#include <SBF/sbf.h>

#include "binding_baseline_rrt_functions.h"
#include "binding_baseline_prm_functions.h"
#include "binding_baseline_bitstar_functions.h"
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

    register_baseline_bitstar_functions(module);

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
