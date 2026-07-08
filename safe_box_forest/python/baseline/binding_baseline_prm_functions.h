#pragma once

#include <SBF/sbf.h>

#include "binding_utils.h"
#include "ompl_binding_utils.h"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace py = pybind11;

namespace rbf::python_binding {

inline void register_baseline_prm_functions(py::module_& module) {
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
}

}  // namespace rbf::python_binding
