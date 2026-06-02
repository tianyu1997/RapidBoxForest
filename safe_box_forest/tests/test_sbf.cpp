#include <SBF/sbf.h>

#ifdef NDEBUG
#undef NDEBUG
#endif

#include <algorithm>
#include <atomic>
#include <cassert>
#include <filesystem>
#include <iostream>
#include <optional>
#include <set>
#include <stdexcept>

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

std::filesystem::path temp_database_path(const std::string& name) {
    auto path = std::filesystem::temp_directory_path() / name;
    std::filesystem::remove_all(path);
    return path;
}

rbf::RBFPlanningConfig base_config(const std::string& name) {
    rbf::RBFPlanningConfig config;
    config.database.path = temp_database_path(name);
    config.database.checkpoint_after_build = false;
    config.database.online_cache.max_payload_bytes = 1024u * 1024u;
    config.enable_merger = false;
    config.grower.find_free_box.max_depth = 6;
    config.grower.find_free_box.split.use_best_tighten = false;
    return config;
}

class AcceptAllOracle final : public rbf::BoxOracle {
public:
    AcceptAllOracle() : root_{{0.0, 3.0}, {0.0, 1.0}} {}

    int n_dims() const override { return static_cast<int>(root_.size()); }
    rbf::OracleNodeId root_node() const override { return 0; }
    const std::vector<rbf::Interval>& root_intervals() const override { return root_; }
    std::vector<rbf::Interval> node_intervals(rbf::OracleNodeId) const override { return root_; }
    bool contains_point(rbf::OracleNodeId, const Eigen::Ref<const Eigen::VectorXd>&) const override { return true; }
    bool is_leaf(rbf::OracleNodeId) const override { return true; }
    int depth(rbf::OracleNodeId) const override { return 0; }
    int split_dim(rbf::OracleNodeId) const override { return 0; }
    double split_value(rbf::OracleNodeId) const override { return 0.0; }
    rbf::OracleNodeId left_child(rbf::OracleNodeId) const override { return -1; }
    rbf::OracleNodeId right_child(rbf::OracleNodeId) const override { return -1; }
    rbf::SplitNodeResult split_node(rbf::OracleNodeId,
                                    const std::vector<rbf::Interval>&,
                                    int,
                                    const rbf::OracleSplitOptions&) override { return {}; }
    rbf::SplitNodeResult split_node_at(rbf::OracleNodeId, int, double) override { return {}; }
    bool point_in_collision(const Eigen::Ref<const Eigen::VectorXd>&) const override { return false; }
    rbf::BoxValidation validate_node(rbf::OracleNodeId, const std::vector<rbf::Interval>&, int = -1) override { return rbf::BoxValidation::Free; }
    bool validate_intervals(const std::vector<rbf::Interval>&) override {
        interval_validations += 1;
        counters_.interval_validations += 1;
        return true;
    }
    bool is_reserved(rbf::OracleNodeId) const override { return false; }
    std::optional<int> reservation_owner(rbf::OracleNodeId) const override { return std::nullopt; }
    void reserve_node(rbf::OracleNodeId, int) override {}
    void release_node(rbf::OracleNodeId) override {}
    void release_box(int box_id) override { released_boxes.push_back(box_id); }
    void clear_reservations() override {}
    rbf::OracleNodeId select_unexplored_node() const override { return -1; }
    const rbf::OracleCounters& counters() const override { return counters_; }
    void reset_counters() override { counters_ = {}; }

    int interval_validations = 0;
    std::vector<int> released_boxes;

private:
    std::vector<rbf::Interval> root_;
    rbf::OracleCounters counters_;
};

rbf::BoxNode make_test_box(int id, double x0, double x1) {
    rbf::BoxNode box;
    box.id = id;
    box.joint_intervals = {{x0, x1}, {0.0, 1.0}};
    box.compute_volume();
    return box;
}

std::set<std::pair<int, int>> edge_set(const rbf::AdjacencyGraph& graph) {
    std::set<std::pair<int, int>> edges;
    for (const auto& [id, neighbors] : graph) {
        for (int neighbor : neighbors) {
            edges.insert({std::min(id, neighbor), std::max(id, neighbor)});
        }
    }
    return edges;
}

void test_runtime_context() {
    rbf::RuntimeConfig runtime;
    runtime.mode = rbf::ExecutionMode::Parallel;
    runtime.n_threads = 2;
    rbf::StageContext context(runtime);
    std::atomic<int> total{0};
    context.executor().parallel_for(0, 8, [&](int item) {
        total.fetch_add(item, std::memory_order_relaxed);
    });
    assert(total.load(std::memory_order_relaxed) == 28);
    context.diagnostics().add_counter("runtime.items", 8.0);
    assert(context.diagnostics().value("runtime.items") == 8.0);
    context.cancellation().cancel();
    assert(context.should_stop());
}

void test_graph() {
    rbf::BoxNode a = make_test_box(0, 0.0, 1.0);
    rbf::BoxNode b = make_test_box(1, 1.0, 2.0);
    assert(rbf::boxes_connected(a, b));
    auto graph = rbf::compute_adjacency({a, b});
    assert(rbf::find_islands(graph).size() == 1);
    auto dijkstra = rbf::dijkstra_search(graph, {a, b}, 0, 1);
    assert(dijkstra.found);
}

void test_indexed_graph_equivalence() {
    std::vector<rbf::BoxNode> boxes;
    int id = 0;
    for (int ix = 0; ix < 9; ++ix) {
        for (int iy = 0; iy < 7; ++iy) {
            rbf::BoxNode box;
            box.id = id++;
            box.joint_intervals = {
                {0.31 * ix, 0.31 * ix + 0.31},
                {0.27 * iy, 0.27 * iy + 0.27},
                {0.013 * ((ix + iy) % 3), 0.013 * ((ix + iy) % 3) + 0.5},
            };
            box.compute_volume();
            boxes.push_back(box);
        }
    }
    auto reference = rbf::compute_adjacency_reference(boxes, 1e-9, 0, 0.0);
    auto indexed = rbf::compute_adjacency(boxes, 1e-9, 0, 0.0);
    assert(edge_set(reference) == edge_set(indexed));
}

void test_birrt_connect_api() {
    auto robot = make_toy_robot();
    rbf::CollisionChecker checker(robot, rbf::Scene{});
    Eigen::VectorXd start(2), goal(2);
    start << -0.8, -0.4;
    goal << 0.75, 0.45;

    rbf::RRTConnectConfig config;
    config.max_iters = 128;
    config.step_size = 0.2;
    config.segment_resolution = 8;
    auto path = rbf::rrt_connect(start, goal, checker, robot, config, 7);
    assert(path.size() == 2);
    assert((path.front() - start).norm() < 1e-12);
    assert((path.back() - goal).norm() < 1e-12);
}

void test_parallel_merger_candidates() {
    AcceptAllOracle oracle;
    std::vector<rbf::BoxNode> boxes = {
        make_test_box(0, 0.0, 1.0),
        make_test_box(1, 1.0, 2.0),
        make_test_box(2, 2.0, 3.0),
    };
    rbf::MergerConfig config;
    config.exact_face_merge = false;
    config.greedy_hull_merge = true;
    config.containment_prune = false;
    config.max_rounds = 1;
    config.n_threads = 2;
    config.parallel_threshold = 1;
    rbf::Consolidator consolidator(oracle, config);

    rbf::RuntimeConfig runtime;
    runtime.mode = rbf::ExecutionMode::Parallel;
    runtime.n_threads = 2;
    rbf::StageContext context(runtime);
    auto result = consolidator.run(boxes, context);

    assert(result.greedy_merges == 1);
    assert(boxes.size() == 2);
    assert(oracle.interval_validations >= 1);
}

void test_database_oracle_session_commit() {
    auto robot = make_toy_robot();
    auto config = base_config("sbf_database_oracle_session");
    rbf::RBFPlanningForest forest(robot, config);
    rbf::DatabaseBoxOracle oracle(robot, forest.online_cache(), {}, config.endpoint_source, config.envelope_type, config.validation);
    rbf::OracleSessionConfig session_config;
    session_config.read_only = false;
    session_config.domain_root = oracle.root_node();
    auto session = oracle.make_session(session_config);

    rbf::OracleSplitOptions split_options;
    split_options.use_best_tighten = false;
    const auto intervals = session->oracle().node_intervals(session->domain_root());
    auto split = session->oracle().split_node(session->domain_root(), intervals, 0, split_options);
    assert(split.split);
    assert(session->commit());
    assert(session->map_node_to_master(split.left) >= 0);
    assert(session->map_node_to_master(split.right) >= 0);
}

void test_safe_box_forest_rrt() {
    auto robot = make_toy_robot();
    auto config = base_config("sbf_rrt");
    config.grower.mode = rbf::GrowerConfig::Mode::RRT;
    config.grower.max_boxes = 12;
    config.grower.max_consecutive_miss = 100;
    config.envelope_type.n_subdivisions = 4;
    config.query.nearest_if_outside = true;
    rbf::RBFPlanningForest forest(robot, config);
    Eigen::VectorXd start(2), goal(2);
    start << -0.5, -0.25;
    goal << 0.6, 0.35;
    auto profile = forest.build(start, goal, {});
    assert(profile.raw_boxes >= 1);
    assert(!forest.boxes().empty());
    auto query = forest.query(start, goal);
    assert(query.success);
    assert(query.path.size() >= 2);
}

void test_audit_segment_step_requires_finer_sampling_than_fixed_resolution() {
    Eigen::VectorXd start(2);
    Eigen::VectorXd goal(2);
    start << 0.0, 0.0;
    goal << 2.7032930184929542, 0.0;
    const int coarse_only = 32;
    const double fine_step = 0.01;
    const int fine_resolution = std::max(
        coarse_only,
        std::max(2, static_cast<int>(std::ceil((goal - start).norm() / fine_step))));
    assert(fine_resolution > coarse_only);
}

void test_query_strict_path_audit() {
    auto robot = make_toy_robot();
    auto config = base_config("sbf_query_strict_path_audit");
    config.enable_connector = false;
    config.query.strict_path_audit = true;
    config.query.audit_resolution = 8;
    config.query.nearest_if_outside = true;
    rbf::RBFPlanningForest forest(robot, config);
    Eigen::VectorXd start(2), goal(2);
    start << -0.5, -0.25;
    goal << 0.6, 0.35;
    forest.build(start, goal, {});
    auto query = forest.query(start, goal);
    assert(query.success);
    assert(query.audit_passed);
    assert(query.audit_status == rbf::PathAuditStatus::Passed);
}

void test_safe_box_forest_frontwave() {
    auto robot = make_toy_robot();
    auto config = base_config("sbf_frontwave");
    config.grower.mode = rbf::GrowerConfig::Mode::Frontwave;
    config.grower.max_boxes = 10;
    config.grower.frontwave_stages = {{4}, {8}, {10}};
    rbf::RBFPlanningForest forest(robot, config);
    Eigen::VectorXd seed(2);
    seed << 0.0, 0.0;
    auto profile = forest.build_coverage({}, {seed});
    assert(profile.raw_boxes >= 1);
}

void test_leaf_sweep_empty_scene() {
    auto robot = make_toy_robot();
    auto config = base_config("sbf_leaf_sweep_empty");
    config.endpoint_source.source = rbf::EndpointSource::IFK;
    config.envelope_type.type = rbf::EnvelopeType::LinkIAABB;
    rbf::RBFPlanningForest forest(robot, config);
    rbf::LeafSweepConfig sweep_config;
    auto result = forest.build_leaf_sweep({}, 2, 2, sweep_config);
    assert(result.free_boxes.size() == 4);
    assert(result.collision_boxes.empty());
    assert(forest.boxes().size() == result.free_boxes.size());
    assert(result.diagnostics.at("leaf_sweep.start_depth") == 2.0);
    bool threw = false;
    try {
        forest.build_leaf_sweep({}, 2, 1, sweep_config);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    assert(threw);
}

void test_leaf_sweep_single_obstacle_collision() {
    auto robot = make_toy_robot();
    auto config = base_config("sbf_leaf_sweep_collision");
    config.endpoint_source.source = rbf::EndpointSource::IFK;
    config.envelope_type.type = rbf::EnvelopeType::LinkIAABB;
    rbf::RBFPlanningForest forest(robot, config);
    rbf::LeafSweepConfig sweep_config;
    const std::vector<rbf::Obstacle> obstacles = {
        rbf::Obstacle(-10.0f, -10.0f, -10.0f, 10.0f, 10.0f, 10.0f),
    };
    auto result = forest.build_leaf_sweep(obstacles, 0, 1, sweep_config);
    assert(!result.collision_boxes.empty());
    assert(result.free_boxes.empty());
    assert(result.groups.size() == 1);
    assert(result.groups.front().collision_boxes.size() == result.collision_boxes.size());
}

void test_leaf_sweep_grouping_and_composition() {
    auto robot = make_toy_robot();
    auto config = base_config("sbf_leaf_sweep_grouping");
    config.endpoint_source.source = rbf::EndpointSource::IFK;
    config.envelope_type.type = rbf::EnvelopeType::LinkIAABB;
    const std::vector<rbf::Obstacle> obstacles = {
        rbf::Obstacle(100.0f, 100.0f, 100.0f, 101.0f, 101.0f, 101.0f),
        rbf::Obstacle(100.5f, 100.5f, 100.5f, 101.5f, 101.5f, 101.5f),
        rbf::Obstacle(110.0f, 110.0f, 110.0f, 111.0f, 111.0f, 111.0f),
    };
    rbf::LeafSweepConfig sweep_config;
    sweep_config.obstacle_cluster_gap = 0.0;
    rbf::RBFPlanningForest forest(robot, config);
    auto result = forest.build_leaf_sweep(obstacles, 1, 1, sweep_config);
    assert(result.groups.size() == 2);
    assert(result.obstacle_group_ids.size() == obstacles.size());
    assert(result.obstacle_group_ids[0] == result.obstacle_group_ids[1]);
    assert(result.obstacle_group_ids[2] != result.obstacle_group_ids[0]);

    rbf::LeafSweepConfig merged_config;
    merged_config.obstacle_cluster_gap = 1000.0;
    auto merged_base = base_config("sbf_leaf_sweep_grouping_merged");
    merged_base.endpoint_source.source = rbf::EndpointSource::IFK;
    merged_base.envelope_type.type = rbf::EnvelopeType::LinkIAABB;
    rbf::RBFPlanningForest merged_forest(robot, merged_base);
    auto merged = merged_forest.build_leaf_sweep(obstacles, 1, 1, merged_config);
    assert(merged.groups.size() == 1);
    assert(result.free_boxes.size() == merged.free_boxes.size());
    assert(result.collision_boxes.size() == merged.collision_boxes.size());
}

void test_leaf_sweep_thread_count_consistency() {
    auto robot = make_toy_robot();
    auto config = base_config("sbf_leaf_sweep_threads_a");
    config.endpoint_source.source = rbf::EndpointSource::IFK;
    config.envelope_type.type = rbf::EnvelopeType::LinkIAABB;
    const std::vector<rbf::Obstacle> obstacles = {
        rbf::Obstacle(-10.0f, -10.0f, -10.0f, 10.0f, 10.0f, 10.0f),
    };
    rbf::LeafSweepConfig serial_config;
    serial_config.n_threads = 1;
    rbf::RBFPlanningForest serial_forest(robot, config);
    auto serial = serial_forest.build_leaf_sweep(obstacles, 1, 2, serial_config);

    auto parallel_base = base_config("sbf_leaf_sweep_threads_b");
    parallel_base.endpoint_source.source = rbf::EndpointSource::IFK;
    parallel_base.envelope_type.type = rbf::EnvelopeType::LinkIAABB;
    rbf::LeafSweepConfig parallel_config;
    parallel_config.n_threads = 4;
    rbf::RBFPlanningForest parallel_forest(robot, parallel_base);
    auto parallel = parallel_forest.build_leaf_sweep(obstacles, 1, 2, parallel_config);

    assert(serial.free_boxes.size() == parallel.free_boxes.size());
    assert(serial.collision_boxes.size() == parallel.collision_boxes.size());
}

void test_obstacle_rebuild() {
    auto robot = make_toy_robot();
    auto config = base_config("sbf_obstacle_rebuild");
    config.grower.max_boxes = 4;
    config.grower.find_free_box.max_depth = 4;
    rbf::RBFPlanningForest forest(robot, config);
    Eigen::VectorXd start(2), goal(2);
    start << -0.5, -0.2;
    goal << 0.5, 0.2;
    forest.build(start, goal, {});
    const int before = static_cast<int>(forest.boxes().size());
    auto rebuild = forest.add_obstacle_and_rebuild(rbf::Obstacle(-10.0f, -10.0f, -10.0f, 10.0f, 10.0f, 10.0f));
    assert(rebuild.boxes_before == before);
    assert(rebuild.boxes_after <= rebuild.boxes_before);
}

void test_query_audit_gated_repair_without_graph() {
    auto robot = make_toy_robot();
    auto config = base_config("sbf_query_audit_repair");
    config.enable_connector = false;
    config.query.strict_path_audit = true;
    config.query.repair_on_audit_failure = true;
    config.query.repair_timeout_ms = 100.0;
    config.connector.rrt.max_iters = 2000;
    config.connector.rrt.timeout_ms = 100.0;
    rbf::RBFPlanningForest forest(robot, config);
    Eigen::VectorXd start(2), goal(2);
    start << -0.6, -0.2;
    goal << 0.6, 0.2;
    auto query = forest.query(start, goal);
    assert(query.success);
    assert(query.audit_passed);
    assert(query.repair_count > 0);
}

}  // namespace

int main() {
    test_runtime_context();
    test_graph();
    test_indexed_graph_equivalence();
    test_birrt_connect_api();
    test_parallel_merger_candidates();
    test_database_oracle_session_commit();
    test_safe_box_forest_rrt();
    test_audit_segment_step_requires_finer_sampling_than_fixed_resolution();
    test_query_strict_path_audit();
    test_safe_box_forest_frontwave();
    test_leaf_sweep_empty_scene();
    test_leaf_sweep_single_obstacle_collision();
    test_leaf_sweep_grouping_and_composition();
    test_leaf_sweep_thread_count_consistency();
    test_obstacle_rebuild();
    test_query_audit_gated_repair_without_graph();
    std::cout << "SBF C++ tests passed.\n";
    return 0;
}
