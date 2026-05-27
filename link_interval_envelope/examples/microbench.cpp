#include <link_interval_envelope/incremental_context.h>

#include <sbf/core/fk_state.h>
#include <sbf/core/robot.h>
#include <sbf/envelope/endpoint_source.h>
#include <sbf/envelope/envelope_type.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;

volatile double g_sink = 0.0;

void consume(double value) {
    g_sink = value;
}

struct Args {
    std::string robot_path = std::string(LIE_EXAMPLE_DATA_DIR) + "/panda.json";
    int repeats = 7;
    int iterations = 1000;
    int threads = 4;
    int sequence_length = 256;
    bool sweep = false;
    bool sequence = false;
};

struct Timing {
    double median_us = 0.0;
    double p95_us = 0.0;
};

Args parse_args(int argc, char** argv) {
    Args args;
    for (int arg_idx = 1; arg_idx < argc; ++arg_idx) {
        const std::string key = argv[arg_idx];
        auto require_value = [&](const char* name) -> std::string {
            if (arg_idx + 1 >= argc) {
                throw std::runtime_error(std::string("missing value for ") + name);
            }
            return argv[++arg_idx];
        };
        if (key == "--robot") args.robot_path = require_value("--robot");
        else if (key == "--repeats") args.repeats = std::stoi(require_value("--repeats"));
        else if (key == "--iterations") args.iterations = std::stoi(require_value("--iterations"));
        else if (key == "--threads") args.threads = std::stoi(require_value("--threads"));
        else if (key == "--sequence-length") args.sequence_length = std::stoi(require_value("--sequence-length"));
        else if (key == "--sweep") args.sweep = true;
        else if (key == "--sequence") args.sequence = true;
        else if (key == "--help") {
            std::cout << "usage: lie_microbench [--robot PATH] [--repeats N] "
                      << "[--iterations N] [--threads N] [--sequence-length N] "
                      << "[--sweep] [--sequence]\n";
            std::exit(0);
        } else {
            throw std::runtime_error("unknown argument: " + key);
        }
    }
    args.repeats = std::max(1, args.repeats);
    args.iterations = std::max(1, args.iterations);
    args.threads = std::max(1, args.threads);
    args.sequence_length = std::max(2, args.sequence_length);
    return args;
}

Timing summarize(std::vector<double> samples) {
    std::sort(samples.begin(), samples.end());
    const double median = samples[samples.size() / 2];
    const std::size_t p95_idx = std::min(samples.size() - 1,
        static_cast<std::size_t>(std::ceil(samples.size() * 0.95)) - 1);
    return {median, samples[p95_idx]};
}

Timing measure_us(int repeats, int iterations, const std::function<void(int)>& body) {
    std::vector<double> samples;
    samples.reserve(static_cast<std::size_t>(repeats));
    for (int repeat = 0; repeat < repeats; ++repeat) {
        const auto start = Clock::now();
        for (int iter = 0; iter < iterations; ++iter) body(iter);
        const auto stop = Clock::now();
        const double total_us = std::chrono::duration<double, std::micro>(stop - start).count();
        samples.push_back(total_us / static_cast<double>(iterations));
    }
    return summarize(std::move(samples));
}

Timing measure_and_print(const std::string& name, int repeats, int iterations,
                         const std::function<void(int)>& body) {
    const Timing timing = measure_us(repeats, iterations, body);
    std::cout << name << "," << std::fixed << std::setprecision(3)
              << timing.median_us << "," << timing.p95_us << "," << iterations << "\n";
    return timing;
}

std::vector<rbf::Interval> make_scaled_box(const rbf::Robot& robot, double scale) {
    std::vector<rbf::Interval> intervals;
    intervals.reserve(static_cast<std::size_t>(robot.n_joints()));
    for (const auto& limit : robot.joint_limits().limits) {
        const double span = limit.hi - limit.lo;
        const double width = std::min(span, std::max(0.02, span * scale));
        const double center = 0.5 * (limit.lo + limit.hi);
        intervals.emplace_back(
            std::max(limit.lo, center - 0.5 * width),
            std::min(limit.hi, center + 0.5 * width));
    }
    return intervals;
}

std::vector<rbf::Interval> make_base_box(const rbf::Robot& robot) {
    return make_scaled_box(robot, 0.18);
}

std::vector<rbf::Interval> perturb_one_dim(
    const rbf::Robot& robot,
    const std::vector<rbf::Interval>& base,
    int changed_dim,
    int step)
{
    std::vector<rbf::Interval> out = base;
    const auto& limit = robot.joint_limits().limits[static_cast<std::size_t>(changed_dim)];
    const double width = out[static_cast<std::size_t>(changed_dim)].width();
    const double max_shift = std::max(0.0, 0.5 * ((limit.hi - limit.lo) - width));
    const double shift = max_shift > 0.0
        ? (((step % 17) - 8) * std::min(0.01, max_shift / 8.0))
        : 0.0;
    out[static_cast<std::size_t>(changed_dim)].lo =
        std::max(limit.lo, base[static_cast<std::size_t>(changed_dim)].lo + shift);
    out[static_cast<std::size_t>(changed_dim)].hi =
        std::min(limit.hi, out[static_cast<std::size_t>(changed_dim)].lo + width);
    return out;
}

std::vector<std::vector<rbf::Interval>> make_sequence(
    const rbf::Robot& robot,
    const std::vector<rbf::Interval>& base,
    int length,
    int offset)
{
    std::vector<std::vector<rbf::Interval>> sequence;
    sequence.reserve(static_cast<std::size_t>(length));
    std::vector<rbf::Interval> current = base;
    sequence.push_back(current);
    for (int step = 1; step < length; ++step) {
        const int changed_dim = (step + offset) % robot.n_joints();
        current = perturb_one_dim(robot, current, changed_dim, step + offset * 13);
        sequence.push_back(current);
    }
    return sequence;
}

template <bool UseFma>
void scalar_fk_positions(const rbf::Robot& robot, const std::vector<double>& q,
                         double positions[][3]) {
    const auto& dh_params = robot.dh_params();
    const int n_joints = robot.n_joints();
    double transform[16] = {};
    transform[0] = transform[5] = transform[10] = transform[15] = 1.0;
    positions[0][0] = positions[0][1] = positions[0][2] = 0.0;

    auto multiply = [&](const double joint[16], double next[16]) {
        for (int row = 0; row < 3; ++row) {
            const double* tr = transform + row * 4;
            for (int col = 0; col < 4; ++col) {
                if constexpr (UseFma) {
                    next[row * 4 + col] = std::fma(
                        tr[0], joint[col],
                        std::fma(tr[1], joint[4 + col],
                                 std::fma(tr[2], joint[8 + col], tr[3] * joint[12 + col])));
                } else {
                    next[row * 4 + col] =
                        tr[0] * joint[col] +
                        tr[1] * joint[4 + col] +
                        tr[2] * joint[8 + col] +
                        tr[3] * joint[12 + col];
                }
            }
        }
        next[15] = 1.0;
    };

    for (int joint_idx = 0; joint_idx < n_joints; ++joint_idx) {
        const auto& dh = dh_params[static_cast<std::size_t>(joint_idx)];
        const double d_val = dh.joint_type == 1 ? q[static_cast<std::size_t>(joint_idx)] + dh.d : dh.d;
        const double angle = dh.joint_type == 0 ? q[static_cast<std::size_t>(joint_idx)] + dh.theta : dh.theta;
        const double ct = std::cos(angle), st = std::sin(angle);
        const double ca = std::cos(dh.alpha), sa = std::sin(dh.alpha);
        const double joint[16] = {
            ct,      -st,      0.0,  dh.a,
            st*ca,    ct*ca,  -sa,  -d_val*sa,
            st*sa,    ct*sa,   ca,   d_val*ca,
            0.0,      0.0,    0.0,   1.0
        };
        double next[16] = {};
        multiply(joint, next);
        std::copy(std::begin(next), std::end(next), transform);
        positions[joint_idx + 1][0] = transform[3];
        positions[joint_idx + 1][1] = transform[7];
        positions[joint_idx + 1][2] = transform[11];
    }

    if (robot.has_tool()) {
        const auto& tool = *robot.tool_frame();
        const double ct = std::cos(tool.theta), st = std::sin(tool.theta);
        const double ca = std::cos(tool.alpha), sa = std::sin(tool.alpha);
        const double joint[16] = {
            ct,      -st,      0.0,  tool.a,
            st*ca,    ct*ca,  -sa,  -tool.d*sa,
            st*sa,    ct*sa,   ca,   tool.d*ca,
            0.0,      0.0,    0.0,   1.0
        };
        double next[16] = {};
        multiply(joint, next);
        positions[n_joints + 1][0] = next[3];
        positions[n_joints + 1][1] = next[7];
        positions[n_joints + 1][2] = next[11];
    }
}

rbf::EndpointSourceConfig crit_config(int threads, int parallel_min_combos) {
    rbf::EndpointSourceConfig config;
    config.source = rbf::EndpointSource::CritSample;
    config.n_threads = threads;
    config.parallel_min_combos = parallel_min_combos;
    return config;
}

rbf::EnvelopeTypeConfig envelope_config(rbf::EnvelopeType type) {
    rbf::EnvelopeTypeConfig config;
    config.type = type;
    config.n_subdivisions = 4;
    return config;
}

void print_endpoint_probe(const std::string& name, const rbf::EndpointIAABBResult& probe) {
    std::cout << name << "," << probe.combo_count
              << ",threads," << probe.enumerate_threads
              << ",auto_min," << probe.parallel_min_combos_used
              << ",chunk_size," << probe.enumerate_chunk_size
              << ",chunks," << probe.enumerate_chunk_count
              << ",enumerate_us," << std::fixed << std::setprecision(3)
              << probe.enumerate_time_us
              << ",dirty," << probe.candidate_dirty_count
              << ",predh," << probe.predh_rebuild_count
              << ",cache_reused," << (probe.endpoint_cache_reused ? 1 : 0) << "\n";
}

void run_default(const rbf::Robot& robot, const Args& args) {
    const std::vector<rbf::Interval> base = make_base_box(robot);
    const int n_joints = robot.n_joints();

    std::cout << "robot," << robot.name() << ",n_joints," << n_joints
              << ",iterations," << args.iterations
              << ",repeats," << args.repeats
              << ",threads," << args.threads << "\n";
    std::cout << "metric,median_us,p95_us,iterations\n";

    measure_and_print("fk_full", args.repeats, args.iterations, [&](int) {
        const auto state = rbf::compute_fk_full(robot, base);
        consume(state.prefix_lo[n_joints][3]);
    });

    std::vector<int> changed_dim_counts(static_cast<std::size_t>(n_joints), 0);
    rbf::FKState parent = rbf::compute_fk_full(robot, base);
    measure_and_print("fk_incremental_copy", args.repeats, args.iterations, [&](int iter) {
        const int changed_dim = iter % n_joints;
        changed_dim_counts[static_cast<std::size_t>(changed_dim)]++;
        const auto child = perturb_one_dim(robot, base, changed_dim, iter);
        const auto state = rbf::compute_fk_incremental(parent, robot, child, changed_dim);
        consume(state.prefix_lo[n_joints][3]);
    });

    parent = rbf::compute_fk_full(robot, base);
    measure_and_print("fk_update_inplace", args.repeats, args.iterations, [&](int iter) {
        const int changed_dim = iter % n_joints;
        const auto child = perturb_one_dim(robot, base, changed_dim, iter);
        rbf::update_fk_inplace(parent, robot, child, changed_dim);
        consume(parent.prefix_lo[n_joints][3]);
    });

    std::vector<double> q(static_cast<std::size_t>(n_joints));
    for (int joint_idx = 0; joint_idx < n_joints; ++joint_idx) {
        q[static_cast<std::size_t>(joint_idx)] = base[static_cast<std::size_t>(joint_idx)].center();
    }
    measure_and_print("scalar_fk", args.repeats, args.iterations, [&](int iter) {
        const int changed_dim = iter % n_joints;
        q[static_cast<std::size_t>(changed_dim)] =
            perturb_one_dim(robot, base, changed_dim, iter)[static_cast<std::size_t>(changed_dim)].center();
        double positions[rbf::MAX_TF][3];
        scalar_fk_positions<false>(robot, q, positions);
        consume(positions[n_joints][0]);
    });
    measure_and_print("scalar_fk_fma_experiment", args.repeats, args.iterations, [&](int iter) {
        const int changed_dim = iter % n_joints;
        q[static_cast<std::size_t>(changed_dim)] =
            perturb_one_dim(robot, base, changed_dim, iter)[static_cast<std::size_t>(changed_dim)].center();
        double positions[rbf::MAX_TF][3];
        scalar_fk_positions<true>(robot, q, positions);
        consume(positions[n_joints][0]);
    });

    rbf::EndpointIAABBResult crit_probe;
    const auto crit_serial = crit_config(1, 0);
    const auto crit_parallel = crit_config(args.threads, 0);
    measure_and_print("critsample_serial_endpoint", args.repeats, std::max(1, args.iterations / 10), [&](int) {
        crit_probe = rbf::compute_endpoint_iaabb(robot, base, crit_serial);
        consume(crit_probe.endpoint_iaabbs.empty() ? 0.0 : crit_probe.endpoint_iaabbs[0]);
    });
    print_endpoint_probe("critsample_serial_combos", crit_probe);

    measure_and_print("critsample_parallel_endpoint", args.repeats, std::max(1, args.iterations / 10), [&](int) {
        crit_probe = rbf::compute_endpoint_iaabb(robot, base, crit_parallel);
        consume(crit_probe.endpoint_iaabbs.empty() ? 0.0 : crit_probe.endpoint_iaabbs[0]);
    });
    print_endpoint_probe("critsample_parallel_combos", crit_probe);

    const auto ifk_endpoint = rbf::compute_endpoint_iaabb(robot, base, rbf::EndpointSourceConfig{});
    rbf::LinkEnvelope kdop_probe;
    const auto kdop_cfg = envelope_config(rbf::EnvelopeType::KDOP);
    measure_and_print("kdop_envelope", args.repeats, std::max(1, args.iterations / 10), [&](int) {
        kdop_probe = rbf::compute_link_envelope(
            ifk_endpoint.endpoint_iaabbs.data(), ifk_endpoint.n_active_links,
            robot.active_link_radii(), kdop_cfg);
        consume(kdop_probe.kdop_intervals.size());
    });

    rbf::LinkEnvelope support_probe;
    const auto support_cfg = envelope_config(rbf::EnvelopeType::SupportHull);
    measure_and_print("support_hull_envelope", args.repeats, std::max(1, args.iterations / 10), [&](int) {
        support_probe = rbf::compute_link_envelope(
            ifk_endpoint.endpoint_iaabbs.data(), ifk_endpoint.n_active_links,
            robot.active_link_radii(), support_cfg);
        consume(support_probe.support_hulls.size());
    });

    std::cout << "changed_dim_counts";
    for (int changed_dim = 0; changed_dim < n_joints; ++changed_dim) {
        std::cout << ",dim" << changed_dim << ","
                  << changed_dim_counts[static_cast<std::size_t>(changed_dim)];
    }
    std::cout << "\n";
}

void run_sweep(const rbf::Robot& robot, const Args& args) {
    const std::vector<double> scales = {0.04, 0.08, 0.12, 0.18, 0.30, 0.45};
    std::vector<int> thread_counts = {1, 2, 4, args.threads};
    std::sort(thread_counts.begin(), thread_counts.end());
    thread_counts.erase(std::unique(thread_counts.begin(), thread_counts.end()), thread_counts.end());
    const std::vector<int> min_combo_options = {0, 1, 1024, 4096};
    const int endpoint_iterations = std::max(1, args.iterations / 20);

    std::cout << "sweep,robot," << robot.name() << ",repeats," << args.repeats
              << ",iterations," << endpoint_iterations << "\n";
    std::cout << "kind,scale,threads,requested_min,used_min,chunk_size,chunks,"
              << "combo_count,enum_threads,median_us,p95_us,enumerate_us,dirty,predh,cache_reused\n";

    for (double scale : scales) {
        const auto box = make_scaled_box(robot, scale);
        for (int threads : thread_counts) {
            if (threads > args.threads) continue;
            for (int requested_min : min_combo_options) {
                const auto config = crit_config(threads, requested_min);
                rbf::EndpointIAABBResult probe;
                const Timing timing = measure_us(args.repeats, endpoint_iterations, [&](int) {
                    probe = rbf::compute_endpoint_iaabb(robot, box, config);
                    consume(probe.endpoint_iaabbs.empty() ? 0.0 : probe.endpoint_iaabbs[0]);
                });
                std::cout << "critsample," << scale << "," << threads << ","
                          << requested_min << "," << probe.parallel_min_combos_used << ","
                          << probe.enumerate_chunk_size << "," << probe.enumerate_chunk_count << ","
                          << probe.combo_count << "," << probe.enumerate_threads << ","
                          << std::fixed << std::setprecision(3)
                          << timing.median_us << "," << timing.p95_us << ","
                          << probe.enumerate_time_us << ","
                          << probe.candidate_dirty_count << "," << probe.predh_rebuild_count << ","
                          << (probe.endpoint_cache_reused ? 1 : 0) << "\n";
            }
        }
    }
}

Timing measure_sequence(const std::string& name, int repeats, int item_count,
                        const std::function<void()>& setup,
                        const std::function<void(int)>& body) {
    std::vector<double> samples;
    samples.reserve(static_cast<std::size_t>(repeats));
    for (int repeat = 0; repeat < repeats; ++repeat) {
        setup();
        const auto start = Clock::now();
        for (int item = 0; item < item_count; ++item) body(item);
        const auto stop = Clock::now();
        const double total_us = std::chrono::duration<double, std::micro>(stop - start).count();
        samples.push_back(total_us / static_cast<double>(item_count));
    }
    const Timing timing = summarize(std::move(samples));
    std::cout << name << "," << std::fixed << std::setprecision(3)
              << timing.median_us << "," << timing.p95_us << "," << item_count << "\n";
    return timing;
}

void run_sequence(const rbf::Robot& robot, const Args& args) {
    const auto base = make_base_box(robot);
    const auto sequence = make_sequence(robot, base, args.sequence_length, 0);
    const auto link_cfg = envelope_config(rbf::EnvelopeType::LinkIAABB);
    const auto support_cfg = envelope_config(rbf::EnvelopeType::SupportHull);
    const auto ifk_config = rbf::EndpointSourceConfig{};
    const auto crit_auto = crit_config(args.threads, 0);
    rbf::EndpointIAABBResult endpoint_probe;
    link_interval_envelope::IncrementalEnvelopeContext context(robot, ifk_config, link_cfg);
    link_interval_envelope::IncrementalEnvelopeContext crit_context(robot, crit_auto, link_cfg);

    std::cout << "sequence,robot," << robot.name() << ",length," << args.sequence_length
              << ",repeats," << args.repeats << ",threads," << args.threads << "\n";
    std::cout << "metric,median_us,p95_us,items\n";

    measure_sequence("sequence_ifk_full", args.repeats, args.sequence_length, []() {}, [&](int item) {
        endpoint_probe = rbf::compute_endpoint_iaabb(robot, sequence[static_cast<std::size_t>(item)], ifk_config);
        const auto envelope = rbf::compute_link_envelope(
            endpoint_probe.endpoint_iaabbs.data(), endpoint_probe.n_active_links,
            robot.active_link_radii(), link_cfg);
        consume(envelope.link_iaabbs.empty() ? 0.0 : envelope.link_iaabbs[0]);
    });

    measure_sequence("sequence_ifk_incremental_context", args.repeats, args.sequence_length,
        [&]() { context.reset(); }, [&](int item) {
            const auto result = context.compute(sequence[static_cast<std::size_t>(item)]);
            consume(result.envelope.link_iaabbs.empty() ? 0.0 : result.envelope.link_iaabbs[0]);
        });

    measure_sequence("sequence_critsample_full", args.repeats, args.sequence_length, []() {}, [&](int item) {
        endpoint_probe = rbf::compute_endpoint_iaabb(robot, sequence[static_cast<std::size_t>(item)], crit_auto);
        const auto envelope = rbf::compute_link_envelope(
            endpoint_probe.endpoint_iaabbs.data(), endpoint_probe.n_active_links,
            robot.active_link_radii(), link_cfg);
        consume(envelope.link_iaabbs.empty() ? 0.0 : envelope.link_iaabbs[0]);
    });

    measure_sequence("sequence_critsample_incremental_context", args.repeats, args.sequence_length,
        [&]() { crit_context.reset(); }, [&](int item) {
            const auto result = crit_context.compute(sequence[static_cast<std::size_t>(item)]);
            consume(result.envelope.link_iaabbs.empty() ? 0.0 : result.envelope.link_iaabbs[0]);
        });

    const int pool_size = std::max(1, args.threads);
    std::vector<std::vector<std::vector<rbf::Interval>>> streams;
    streams.reserve(static_cast<std::size_t>(pool_size));
    for (int worker = 0; worker < pool_size; ++worker) {
        streams.push_back(make_sequence(robot, base, args.sequence_length, worker));
    }
    std::vector<link_interval_envelope::IncrementalEnvelopeContext> pool;
    pool.reserve(static_cast<std::size_t>(pool_size));
    auto reset_pool = [&]() {
        pool.clear();
        for (int worker = 0; worker < pool_size; ++worker) {
            pool.emplace_back(robot, crit_auto, support_cfg);
        }
    };
    measure_sequence("sequence_critsample_state_pool_support_hull", args.repeats,
        args.sequence_length * pool_size, reset_pool, [&](int item) {
            const int worker = item % pool_size;
            const int step = item / pool_size;
            const auto result = pool[static_cast<std::size_t>(worker)].compute(
                streams[static_cast<std::size_t>(worker)][static_cast<std::size_t>(step)]);
            consume(result.envelope.support_hulls.size());
        });
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const Args args = parse_args(argc, argv);
        const rbf::Robot robot = rbf::Robot::from_json(args.robot_path);
        if (args.sweep) run_sweep(robot, args);
        else if (args.sequence) run_sequence(robot, args);
        else run_default(robot, args);
        return 0;
    } catch (const std::exception& err) {
        std::cerr << "microbench error: " << err.what() << "\n";
        return 1;
    }
}