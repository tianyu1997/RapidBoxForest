#pragma once

#include <SBF/box_graph.h>
#include <SBF/connector.h>
#include <SBF/scene.h>

#include "../binding_utils.h"
#include "../ompl_binding_utils.h"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <vector>

namespace py = pybind11;

namespace rbf::python_binding {

inline void register_baseline_rrt_functions(py::module_& module) {
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
}

}  // namespace rbf::python_binding
