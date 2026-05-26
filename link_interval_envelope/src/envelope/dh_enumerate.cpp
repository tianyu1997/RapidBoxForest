// RBFPlanningForest v6 — Shared DH enumeration implementation
#include <sbf/envelope/dh_enumerate.h>
#include <sbf/envelope/endpoint_source.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
#include <thread>
#include <vector>

namespace rbf {

namespace {

void init_base_transform(double* stack_T, double positions[MAX_TF][3]) {
    double* T0 = stack_T;
    for (int i = 0; i < 12; ++i) T0[i] = 0.0;
    T0[0] = T0[5] = T0[10] = 1.0;
    positions[0][0] = 0.0;
    positions[0][1] = 0.0;
    positions[0][2] = 0.0;
}

void build_tool_matrix(const Robot& robot, double tool_A[12]) {
    for (int i = 0; i < 12; ++i) tool_A[i] = 0.0;
    if (!robot.has_tool()) return;
    const auto& tf = *robot.tool_frame();
    double ct = std::cos(tf.theta), st = std::sin(tf.theta);
    double ca = std::cos(tf.alpha), sa = std::sin(tf.alpha);
    tool_A[0]  = ct;      tool_A[1]  = -st;     tool_A[2]  = 0.0;  tool_A[3]  = tf.a;
    tool_A[4]  = st*ca;   tool_A[5]  = ct*ca;   tool_A[6]  = -sa;  tool_A[7]  = -tf.d*sa;
    tool_A[8]  = st*sa;   tool_A[9]  = ct*sa;   tool_A[10] = ca;   tool_A[11] = tf.d*ca;
}

void decode_combo_index(int64_t linear, const std::vector<int>& n_cands, int* idx) {
    for (int j = static_cast<int>(n_cands.size()) - 1; j >= 0; --j) {
        const int radix = n_cands[j];
        idx[j] = static_cast<int>(linear % radix);
        linear /= radix;
    }
}

void build_prefix_for_index(
    const std::vector<std::vector<PreDH>>& pre_dh,
    const int* idx,
    int n,
    double* stack_T,
    double positions[MAX_TF][3])
{
    init_base_transform(stack_T, positions);
    for (int j = 0; j < n; ++j) {
        const double* Tprev = stack_T + j * 12;
        double* Tnext = stack_T + (j + 1) * 12;
        mul_prefix_dh(Tprev, pre_dh[j][idx[j]].A, Tnext);
        positions[j + 1][0] = Tnext[3];
        positions[j + 1][1] = Tnext[7];
        positions[j + 1][2] = Tnext[11];
    }
}

void apply_tool_if_needed(
    bool has_tool,
    const double tool_A[12],
    int n,
    const double* stack_T,
    double positions[MAX_TF][3])
{
    if (!has_tool) return;
    double tool_R[12];
    mul_prefix_dh(stack_T + n * 12, tool_A, tool_R);
    positions[n + 1][0] = tool_R[3];
    positions[n + 1][1] = tool_R[7];
    positions[n + 1][2] = tool_R[11];
}

int increment_odometer(int* idx, const std::vector<int>& n_cands, int n) {
    int carry = n - 1;
    while (carry >= 0) {
        idx[carry]++;
        if (idx[carry] < n_cands[carry]) break;
        idx[carry] = 0;
        carry--;
    }
    return carry;
}

void enumerate_combo_range(
    const Robot& robot,
    const std::vector<std::vector<PreDH>>& pre_dh,
    const std::vector<int>& n_cands,
    const int* active_link_map,
    int n_active,
    int64_t begin,
    int64_t end,
    float* out)
{
    if (begin >= end) return;
    const int n = robot.n_joints();

    double stack_T[(MAX_JOINTS + 2) * 12];
    double positions[MAX_TF][3];
    double tool_A[12] = {};
    const bool has_tool = robot.has_tool();
    build_tool_matrix(robot, tool_A);

    int idx[MAX_JOINTS];
    decode_combo_index(begin, n_cands, idx);
    build_prefix_for_index(pre_dh, idx, n, stack_T, positions);
    apply_tool_if_needed(has_tool, tool_A, n, stack_T, positions);
    update_endpoints_from_positions(positions, active_link_map, n_active, out);

    for (int64_t combo = begin + 1; combo < end; ++combo) {
        const int carry = increment_odometer(idx, n_cands, n);
        if (carry < 0) break;
        for (int j = carry; j < n; ++j) {
            const double* Tprev = stack_T + j * 12;
            double* Tnext = stack_T + (j + 1) * 12;
            mul_prefix_dh(Tprev, pre_dh[j][idx[j]].A, Tnext);
            positions[j + 1][0] = Tnext[3];
            positions[j + 1][1] = Tnext[7];
            positions[j + 1][2] = Tnext[11];
        }
        apply_tool_if_needed(has_tool, tool_A, n, stack_T, positions);
        update_endpoints_from_positions(positions, active_link_map, n_active, out);
    }
}

int resolve_enumeration_threads(int requested_threads, int64_t combo_count) {
    if (combo_count <= 1) return 1;
    if (requested_threads > 0) {
        return std::max<int>(1, std::min<int64_t>(requested_threads, combo_count));
    }
    const unsigned hw = std::thread::hardware_concurrency();
    const int available = hw == 0 ? 1 : static_cast<int>(hw);
    return std::max<int>(1, std::min<int64_t>({available, combo_count, 8}));
}

int64_t resolve_parallel_min_combos(int requested_min, int workers) {
    if (requested_min > 0) return requested_min;
    return std::max<int64_t>(1024, static_cast<int64_t>(workers) * 512);
}

int64_t resolve_chunk_size(int64_t total, int workers, int64_t min_combos) {
    if (workers <= 1) return total;
    const int64_t target_chunks = std::max<int64_t>(workers, static_cast<int64_t>(workers) * 4);
    const int64_t balanced = std::max<int64_t>(1, (total + target_chunks - 1) / target_chunks);
    const int64_t floor_size = std::max<int64_t>(1, min_combos / std::max(1, workers));
    return std::max<int64_t>(balanced, floor_size);
}

}  // namespace

int64_t count_critical_combinations(const std::vector<int>& n_cands) {
    int64_t total = 1;
    for (int count : n_cands) {
        if (count <= 0) return 0;
        if (total > std::numeric_limits<int64_t>::max() / count) {
            return std::numeric_limits<int64_t>::max();
        }
        total *= static_cast<int64_t>(count);
    }
    return total;
}

DHEnumerationStats enumerate_critical_iterative(
    const Robot& robot,
    const std::vector<std::vector<PreDH>>& pre_dh,
    const std::vector<int>& n_cands,
    const int* active_link_map, int n_active,
    float* out)
{
    const int64_t total = count_critical_combinations(n_cands);
    enumerate_combo_range(robot, pre_dh, n_cands, active_link_map, n_active, 0, total, out);
    return {total, 1, total, total, total > 0 ? 1 : 0};
}

DHEnumerationStats enumerate_critical_parallel(
    const Robot& robot,
    const std::vector<std::vector<PreDH>>& pre_dh,
    const std::vector<int>& n_cands,
    const int* active_link_map, int n_active,
    float* out,
    int requested_threads,
    int parallel_min_combos)
{
    const int64_t total = count_critical_combinations(n_cands);
    if (total <= 0) return {0, 1, 0, 0, 0};

    const int workers = resolve_enumeration_threads(requested_threads, total);
    const int64_t min_combos = resolve_parallel_min_combos(parallel_min_combos, workers);
    if (workers <= 1 || total < min_combos) {
        enumerate_combo_range(robot, pre_dh, n_cands, active_link_map, n_active, 0, total, out);
        return {total, 1, min_combos, total, 1};
    }

    const int64_t chunk_size = resolve_chunk_size(total, workers, min_combos);
    const int chunk_count = static_cast<int>((total + chunk_size - 1) / chunk_size);

    const std::size_t len = static_cast<std::size_t>(n_active * 2 * 6);
    std::vector<std::vector<float>> local_outputs(
        static_cast<std::size_t>(workers),
        std::vector<float>(len));
    for (auto& local : local_outputs) {
        init_endpoints_inf(local.data(), n_active);
    }

    std::vector<std::thread> threads;
    threads.reserve(static_cast<std::size_t>(workers));
    std::atomic<int64_t> next_begin{0};
    for (int w = 0; w < workers; ++w) {
        threads.emplace_back([&, w]() {
            float* local = local_outputs[static_cast<std::size_t>(w)].data();
            while (true) {
                const int64_t begin = next_begin.fetch_add(chunk_size);
                if (begin >= total) break;
                const int64_t end = std::min<int64_t>(total, begin + chunk_size);
                enumerate_combo_range(
                    robot,
                    pre_dh,
                    n_cands,
                    active_link_map,
                    n_active,
                    begin,
                    end,
                    local);
            }
        });
    }
    for (auto& thread : threads) thread.join();

    for (const auto& local : local_outputs) {
        hull_endpoint_iaabbs(out, local.data(), n_active * 2);
    }
    return {total, workers, min_combos, chunk_size, chunk_count};
}

}  // namespace rbf
