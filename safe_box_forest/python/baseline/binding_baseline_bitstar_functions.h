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
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace py = pybind11;

namespace rbf::python_binding {

inline void register_baseline_bitstar_functions(py::module_& module) {
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
}

}  // namespace rbf::python_binding
