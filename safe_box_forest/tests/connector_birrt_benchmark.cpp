#include <SBF/sbf.h>
#include <SBF/connector.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

namespace {

rbf::Robot make_toy_robot() {
    std::vector<rbf::DHParam> dh = {
        {0.0, 0.35, 0.0, 0.0, 0},
        {0.0, 0.30, 0.0, 0.0, 0},
    };
    rbf::JointLimits limits;
    limits.limits = {{-1.0, 1.0}, {-1.0, 1.0}};
    return rbf::Robot("toy2", dh, limits, std::nullopt, {0.03, 0.03});
}

std::vector<rbf::Obstacle> make_scene(const std::string& name) {
    if (name == "none") {
        return {};
    }
    if (name == "central_block") {
        return {rbf::Obstacle(0.28f, -0.07f, -0.20f, 0.48f, 0.07f, 0.20f)};
    }
    if (name == "offset_block") {
        return {rbf::Obstacle(0.18f, 0.10f, -0.20f, 0.58f, 0.34f, 0.20f)};
    }
    if (name == "twin_blocks") {
        return {
            rbf::Obstacle(0.24f, -0.22f, -0.20f, 0.46f, -0.08f, 0.20f),
            rbf::Obstacle(0.24f, 0.08f, -0.20f, 0.46f, 0.22f, 0.20f),
        };
    }
    throw std::invalid_argument("unknown scene preset: " + name);
}

std::vector<double> parse_csv_vector(const std::string& raw, int expected) {
    std::vector<double> values;
    std::stringstream stream(raw);
    std::string token;
    while (std::getline(stream, token, ',')) {
        if (!token.empty()) {
            values.push_back(std::stod(token));
        }
    }
    if (static_cast<int>(values.size()) != expected) {
        throw std::invalid_argument("expected " + std::to_string(expected) + " comma-separated values, got: " + raw);
    }
    return values;
}

std::string format_vector(const std::vector<double>& values) {
    std::ostringstream stream;
    stream << '[';
    for (std::size_t index = 0; index < values.size(); ++index) {
        if (index > 0) {
            stream << ',';
        }
        stream << std::fixed << std::setprecision(3) << values[index];
    }
    stream << ']';
    return stream.str();
}

Eigen::VectorXd eigen_from_std(const std::vector<double>& values) {
    Eigen::VectorXd vector(static_cast<Eigen::Index>(values.size()));
    for (std::size_t index = 0; index < values.size(); ++index) {
        vector[static_cast<Eigen::Index>(index)] = values[index];
    }
    return vector;
}

struct CliConfig {
    bool scan_grid = false;
    std::string scene = "central_block";
    std::vector<double> start{-0.8, -0.4};
    std::vector<double> goal{0.8, 0.4};
    double timeout_ms = 80.0;
    int max_iters = 50000;
    double step_size = 0.25;
    double goal_bias = 0.4;
    int segment_resolution = 16;
    double local_sampling_radius = 0.0;
    int seed = 7;
    int repeats = 3;
    double grid_min = -0.9;
    double grid_max = 0.9;
    double grid_step = 0.6;
    double min_pair_distance = 0.75;
    int top_k = 5;
};

struct PairMetrics {
    std::vector<double> start;
    std::vector<double> goal;
    bool start_free = false;
    bool goal_free = false;
    int repeats = 0;
    int successes = 0;
    int failures = 0;
    int timeouts = 0;
    double mean_ms = 0.0;
    double max_ms = 0.0;
    double mean_iterations = 0.0;
    double mean_waypoints = 0.0;
};

CliConfig parse_args(int argc, char** argv) {
    CliConfig config;
    for (int index = 1; index < argc; ++index) {
        const std::string arg = argv[index];
        auto require_value = [&](const std::string& name) -> std::string {
            if (index + 1 >= argc) {
                throw std::invalid_argument("missing value for " + name);
            }
            return std::string(argv[++index]);
        };
        if (arg == "--scan-grid") {
            config.scan_grid = true;
        } else if (arg == "--scene") {
            config.scene = require_value(arg);
        } else if (arg == "--start") {
            config.start = parse_csv_vector(require_value(arg), 2);
        } else if (arg == "--goal") {
            config.goal = parse_csv_vector(require_value(arg), 2);
        } else if (arg == "--timeout-ms") {
            config.timeout_ms = std::stod(require_value(arg));
        } else if (arg == "--max-iters") {
            config.max_iters = std::stoi(require_value(arg));
        } else if (arg == "--step-size") {
            config.step_size = std::stod(require_value(arg));
        } else if (arg == "--goal-bias") {
            config.goal_bias = std::stod(require_value(arg));
        } else if (arg == "--segment-resolution") {
            config.segment_resolution = std::stoi(require_value(arg));
        } else if (arg == "--local-radius") {
            config.local_sampling_radius = std::stod(require_value(arg));
        } else if (arg == "--seed") {
            config.seed = std::stoi(require_value(arg));
        } else if (arg == "--repeats") {
            config.repeats = std::stoi(require_value(arg));
        } else if (arg == "--grid-min") {
            config.grid_min = std::stod(require_value(arg));
        } else if (arg == "--grid-max") {
            config.grid_max = std::stod(require_value(arg));
        } else if (arg == "--grid-step") {
            config.grid_step = std::stod(require_value(arg));
        } else if (arg == "--min-pair-distance") {
            config.min_pair_distance = std::stod(require_value(arg));
        } else if (arg == "--top-k") {
            config.top_k = std::stoi(require_value(arg));
        } else if (arg == "--help" || arg == "-h") {
            std::cout
                << "connector_birrt_benchmark [--scan-grid] [--scene central_block|offset_block|twin_blocks|none]\n"
                << "  [--start q0,q1 --goal q0,q1] [--timeout-ms 80] [--max-iters 50000]\n"
                << "  [--step-size 0.25] [--goal-bias 0.4] [--segment-resolution 16] [--local-radius 0.0]\n"
                << "  [--repeats 3] [--seed 7] [--grid-min -0.9 --grid-max 0.9 --grid-step 0.6]\n"
                << "  [--min-pair-distance 0.75] [--top-k 5]\n";
            std::exit(0);
        } else {
            throw std::invalid_argument("unknown argument: " + arg);
        }
    }
    return config;
}

PairMetrics run_pair(const rbf::Robot& robot,
                     const rbf::CollisionChecker& checker,
                     const rbf::RRTConnectConfig& config,
                     const std::vector<double>& start,
                     const std::vector<double>& goal,
                     int seed,
                     int repeats) {
    PairMetrics metrics;
    metrics.start = start;
    metrics.goal = goal;
    metrics.repeats = repeats;
    const Eigen::VectorXd start_vec = eigen_from_std(start);
    const Eigen::VectorXd goal_vec = eigen_from_std(goal);
    metrics.start_free = !checker.check_config(start_vec);
    metrics.goal_free = !checker.check_config(goal_vec);
    if (!metrics.start_free || !metrics.goal_free) {
        return metrics;
    }

    double total_ms = 0.0;
    double total_iterations = 0.0;
    double total_waypoints = 0.0;
    for (int repeat = 0; repeat < repeats; ++repeat) {
        rbf::StageContext context = rbf::StageContext::serial();
        const auto path = rbf::rrt_connect(start_vec, goal_vec, checker, robot, context, config, seed + repeat);
        const auto diagnostics = context.diagnostics().snapshot();
        const auto ms_it = diagnostics.find("connector.birrt");
        const auto iterations_it = diagnostics.find("connector.birrt_last_iterations");
        const auto timeouts_it = diagnostics.find("connector.birrt_timeouts");
        const double elapsed_ms = ms_it == diagnostics.end() ? 0.0 : ms_it->second;
        const double iterations = iterations_it == diagnostics.end() ? 0.0 : iterations_it->second;
        const bool timed_out = timeouts_it != diagnostics.end() && timeouts_it->second > 0.0;
        total_ms += elapsed_ms;
        total_iterations += iterations;
        total_waypoints += static_cast<double>(path.size());
        metrics.max_ms = std::max(metrics.max_ms, elapsed_ms);
        if (timed_out) {
            metrics.timeouts += 1;
        }
        if (path.empty()) {
            metrics.failures += 1;
        } else {
            metrics.successes += 1;
        }
    }
    metrics.mean_ms = total_ms / static_cast<double>(repeats);
    metrics.mean_iterations = total_iterations / static_cast<double>(repeats);
    metrics.mean_waypoints = total_waypoints / static_cast<double>(repeats);
    return metrics;
}

std::vector<double> grid_values(double min_value, double max_value, double step) {
    if (!(step > 0.0)) {
        throw std::invalid_argument("grid step must be > 0");
    }
    std::vector<double> values;
    for (double value = min_value; value <= max_value + 1e-9; value += step) {
        values.push_back(value);
    }
    return values;
}

void print_pair_metrics(const std::string& label, const PairMetrics& metrics) {
    std::cout
        << label
        << " start=" << format_vector(metrics.start)
        << " goal=" << format_vector(metrics.goal)
        << " start_free=" << metrics.start_free
        << " goal_free=" << metrics.goal_free
        << " successes=" << metrics.successes << '/' << metrics.repeats
        << " failures=" << metrics.failures
        << " timeouts=" << metrics.timeouts
        << " mean_ms=" << std::fixed << std::setprecision(3) << metrics.mean_ms
        << " max_ms=" << metrics.max_ms
        << " mean_iters=" << metrics.mean_iterations
        << " mean_waypoints=" << metrics.mean_waypoints
        << '\n';
}

int run_scan(const CliConfig& cli, const rbf::Robot& robot, const rbf::CollisionChecker& checker, const rbf::RRTConnectConfig& config) {
    const std::vector<double> q0_values = grid_values(cli.grid_min, cli.grid_max, cli.grid_step);
    const std::vector<double> q1_values = grid_values(cli.grid_min, cli.grid_max, cli.grid_step);
    std::vector<std::vector<double>> points;
    for (double q0 : q0_values) {
        for (double q1 : q1_values) {
            const std::vector<double> point{q0, q1};
            const Eigen::VectorXd eigen_point = eigen_from_std(point);
            if (!checker.check_config(eigen_point)) {
                points.push_back(point);
            }
        }
    }

    std::vector<PairMetrics> timeout_candidates;
    std::vector<PairMetrics> success_candidates;
    int scanned_pairs = 0;
    for (const auto& start : points) {
        for (const auto& goal : points) {
            if (start == goal) {
                continue;
            }
            const double dq0 = start[0] - goal[0];
            const double dq1 = start[1] - goal[1];
            const double distance = std::sqrt(dq0 * dq0 + dq1 * dq1);
            if (distance < cli.min_pair_distance) {
                continue;
            }
            ++scanned_pairs;
            PairMetrics metrics = run_pair(robot, checker, config, start, goal, cli.seed, 1);
            if (metrics.timeouts > 0) {
                timeout_candidates.push_back(metrics);
            }
            if (metrics.successes > 0) {
                success_candidates.push_back(metrics);
            }
        }
    }

    std::sort(timeout_candidates.begin(), timeout_candidates.end(), [](const PairMetrics& lhs, const PairMetrics& rhs) {
        return std::tie(lhs.timeouts, lhs.mean_ms, lhs.mean_iterations) >
               std::tie(rhs.timeouts, rhs.mean_ms, rhs.mean_iterations);
    });
    std::sort(success_candidates.begin(), success_candidates.end(), [](const PairMetrics& lhs, const PairMetrics& rhs) {
        return std::tie(lhs.mean_ms, lhs.mean_iterations, lhs.mean_waypoints) <
               std::tie(rhs.mean_ms, rhs.mean_iterations, rhs.mean_waypoints);
    });

    std::cout
        << "scan scene=" << cli.scene
        << " free_points=" << points.size()
        << " scanned_pairs=" << scanned_pairs
        << " timeout_candidates=" << timeout_candidates.size()
        << " success_candidates=" << success_candidates.size()
        << '\n';
    const int timeout_count = std::min(cli.top_k, static_cast<int>(timeout_candidates.size()));
    for (int index = 0; index < timeout_count; ++index) {
        print_pair_metrics("timeout_candidate[" + std::to_string(index) + "]", timeout_candidates[static_cast<std::size_t>(index)]);
    }
    const int success_count = std::min(cli.top_k, static_cast<int>(success_candidates.size()));
    for (int index = 0; index < success_count; ++index) {
        print_pair_metrics("success_candidate[" + std::to_string(index) + "]", success_candidates[static_cast<std::size_t>(index)]);
    }
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const CliConfig cli = parse_args(argc, argv);
        const rbf::Robot robot = make_toy_robot();
        const std::vector<rbf::Obstacle> obstacles = make_scene(cli.scene);
        const rbf::CollisionChecker checker(robot, rbf::Scene(obstacles));

        rbf::RRTConnectConfig config;
        config.timeout_ms = cli.timeout_ms;
        config.max_iters = cli.max_iters;
        config.step_size = cli.step_size;
        config.goal_bias = cli.goal_bias;
        config.segment_resolution = cli.segment_resolution;
        config.local_sampling_radius = cli.local_sampling_radius;

        if (cli.scan_grid) {
            return run_scan(cli, robot, checker, config);
        }

        const PairMetrics metrics = run_pair(robot, checker, config, cli.start, cli.goal, cli.seed, cli.repeats);
        std::cout << "scene=" << cli.scene << " obstacles=" << obstacles.size() << '\n';
        print_pair_metrics("pair", metrics);
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "connector_birrt_benchmark error: " << ex.what() << '\n';
        return 2;
    }
}
