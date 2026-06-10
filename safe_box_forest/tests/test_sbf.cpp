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
#include <unordered_set>

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

void test_extract_waypoints_uses_overlap_transitions() {
    rbf::BoxNode a;
    a.id = 0;
    a.joint_intervals = {{0.0, 1.0}, {0.0, 2.0}};
    a.compute_volume();
    rbf::BoxNode b;
    b.id = 1;
    b.joint_intervals = {{1.0, 2.0}, {0.0, 2.0}};
    b.compute_volume();
    Eigen::VectorXd start(2), goal(2);
    start << 0.1, 0.1;
    goal << 1.9, 0.1;

    const auto path = rbf::extract_waypoints({0, 1}, std::vector<rbf::BoxNode>{a, b}, start, goal);
    assert(path.size() == 3);
    assert((path[1] - Eigen::Vector2d(1.0, 0.1)).norm() < 1e-12);
    assert(rbf::path_length(path) < 1.81);
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
    auto stats = rbf::last_adjacency_build_stats();
    assert(edge_set(reference) == edge_set(indexed));
    assert(stats.boxes == static_cast<int>(boxes.size()));
    assert(stats.selected_dims >= 1);
    assert(stats.exact_tests <= stats.candidate_pairs);
}

void test_adaptive_grid_partition_query_matches_graph() {
    std::vector<rbf::BoxNode> boxes;
    auto add_box = [&](int id, double x0, double x1, double y0, double y1) {
        rbf::BoxNode box;
        box.id = id;
        box.joint_intervals = {{x0, x1}, {y0, y1}};
        box.safety_status = rbf::BoxSafetyStatus::CertifiedFree;
        box.compute_volume();
        boxes.push_back(box);
    };
    add_box(0, 0.0, 1.0, 0.0, 2.0);
    add_box(1, 1.0, 2.0, 0.0, 1.0);
    add_box(2, 1.0, 2.0, 1.0, 2.0);

    rbf::lect_database::SplitPolicyDescriptor split;
    split.strategy = rbf::lect_database::SplitStrategy::AAFKVolumeMin;
    split.depth_dimensions = {0, 1};
    rbf::AdaptiveGridPartition partition;
    const bool ok = partition.rebuild({{0.0, 2.0}, {0.0, 2.0}},
                                      split,
                                      0,
                                      2,
                                      boxes,
                                      1e-9);
    assert(ok);
    assert(partition.stats().cells == 3);
    assert(partition.stats().grid_cells == 3);
    assert(partition.stats().islands == 1);

    const auto reference = rbf::compute_adjacency_reference(boxes, 1e-9, 0, 0.0);
    for (const auto& box : boxes) {
        const auto neighbors = partition.neighbor_box_ids(box.id);
        std::set<int> partition_neighbors(neighbors.begin(), neighbors.end());
        const auto it = reference.find(box.id);
        std::set<int> reference_neighbors;
        if (it != reference.end()) {
            reference_neighbors.insert(it->second.begin(), it->second.end());
        }
        assert(partition_neighbors == reference_neighbors);
        for (const auto& other : boxes) {
            const bool reference_adjacent =
                reference_neighbors.find(other.id) != reference_neighbors.end();
            assert(partition.boxes_are_neighbors(box.id, other.id) == reference_adjacent);
        }
    }

    Eigen::Vector2d start(0.25, 0.25);
    Eigen::Vector2d goal(1.75, 1.75);
    rbf::AdaptiveGridPartitionQueryOptions options;
    options.adjacency_tolerance = 1e-9;
    const auto query = partition.query(start, goal, options);
    assert(query.found);
    assert(query.start_box_id == 0);
    assert(query.goal_box_id == 2);
    assert(!query.box_sequence.empty());
    assert(query.path.size() >= 2);
    assert((query.path.front() - start).norm() < 1e-12);
    assert((query.path.back() - goal).norm() < 1e-12);
}

void test_adaptive_grid_partition_append_and_merge() {
    auto make_box2 = [](int id, double x0, double x1, double y0, double y1) {
        rbf::BoxNode box;
        box.id = id;
        box.joint_intervals = {{x0, x1}, {y0, y1}};
        box.safety_status = rbf::BoxSafetyStatus::CertifiedFree;
        box.compute_volume();
        return box;
    };

    rbf::lect_database::SplitPolicyDescriptor split;
    split.strategy = rbf::lect_database::SplitStrategy::AAFKVolumeMin;
    split.depth_dimensions = {0, 0};

    std::vector<rbf::BoxNode> append_boxes = {
        make_box2(0, 0.0, 1.0, 0.0, 1.0),
        make_box2(1, 2.0, 3.0, 0.0, 1.0),
    };
    rbf::AdaptiveGridPartition partition;
	    assert(partition.rebuild({{0.0, 4.0}, {0.0, 1.0}},
	                             split,
	                             0,
	                             2,
	                             append_boxes,
	                             1e-9));
	    assert(partition.stats().islands == 2);
	    assert(partition.component_count_with_overlay() == 2);
	    Eigen::VectorXd center;
	    assert(partition.center_for_box(0, center));
	    assert(center.size() == 2);
	    assert(std::abs(center[0] - 0.5) < 1e-12);
	    assert(std::abs(center[1] - 0.5) < 1e-12);
	    Eigen::VectorXd closest;
	    double distance_sq = -1.0;
	    assert(partition.closest_point_for_box(1,
	                                           Eigen::Vector2d(1.5, 0.25),
	                                           closest,
	                                           &distance_sq));
	    assert((closest - Eigen::Vector2d(2.0, 0.25)).norm() < 1e-12);
	    assert(std::abs(distance_sq - 0.25) < 1e-12);
	    assert(partition.box_contains_point(0, Eigen::Vector2d(0.25, 0.25), 1e-9));
	    assert(!partition.box_contains_point(0, Eigen::Vector2d(1.25, 0.25), 1e-9));
	    const auto bridge_candidate = make_box2(100, 1.0, 2.0, 0.0, 1.0);
	    const std::unordered_set<int> left_island{0};
	    const std::unordered_set<int> right_island{1};
	    assert(partition.box_adjacent_to_box(0, bridge_candidate, 1e-9));
	    assert(partition.box_adjacent_to_box(1, bridge_candidate, 1e-9));
	    assert(!partition.boxes_are_neighbors(0, 1));
	    assert(partition.box_adjacent_to_any(bridge_candidate, left_island, 1e-9));
	    assert(partition.box_adjacent_to_any(bridge_candidate, right_island, 1e-9));
	    const auto bridge_neighbors = partition.adjacent_box_ids(bridge_candidate, 1e-9);
	    const std::set<int> bridge_neighbor_set(bridge_neighbors.begin(), bridge_neighbors.end());
	    assert((bridge_neighbor_set == std::set<int>{0, 1}));
	    const auto far_candidate = make_box2(101, 3.0, 4.0, 0.0, 1.0);
	    assert(!partition.box_adjacent_to_any(far_candidate, left_island, 1e-9));
	    const auto component_pairs = partition.nearest_component_pairs_to_largest(1, 8);
	    assert(component_pairs.size() == 1);
	    assert((std::set<int>{component_pairs.front().source_box_id,
	                          component_pairs.front().target_box_id} == std::set<int>{0, 1}));
	    assert(std::abs(component_pairs.front().distance_sq - 1.0) < 1e-12);
	    const auto largest_landmarks = partition.landmarks(true, 8);
	    assert(largest_landmarks.size() == 1);
	    assert(largest_landmarks.front().box_id == 0 || largest_landmarks.front().box_id == 1);
	    assert(largest_landmarks.front().center.size() == 2);
	    const auto all_landmarks = partition.landmarks(false, 8);
	    assert(all_landmarks.size() == 2);
	    rbf::SegmentEdge overlay;
	    overlay.id = 0;
	    overlay.source_box_id = 0;
	    overlay.target_box_id = 1;
	    overlay.type = rbf::SegmentEdgeType::QueryBridge;
	    overlay.validation = rbf::SegmentEdgeValidation::CollisionChecked;
	    overlay.strict_audit_required = true;
	    overlay.waypoints = {Eigen::Vector2d(0.5, 0.5), Eigen::Vector2d(2.5, 0.5)};
	    assert(partition.append_segment_edge(overlay));
	    assert(!partition.boxes_are_neighbors(0, 1));
	    assert(partition.same_component_with_overlay(0, 1));
	    assert(partition.component_count_with_overlay() == 1);
	    const auto overlay_components = partition.component_box_ids_with_overlay();
	    assert(overlay_components.size() == 1);
	    assert((std::set<int>(overlay_components.front().begin(),
	                          overlay_components.front().end()) == std::set<int>{0, 1}));
	    const auto overlay_largest = partition.largest_component_box_ids_with_overlay();
	    assert((std::set<int>(overlay_largest.begin(), overlay_largest.end()) == std::set<int>{0, 1}));
	    assert(partition.nearest_component_pairs_to_largest(1, 8).empty());
    Eigen::Vector2d start(0.5, 0.5);
    Eigen::Vector2d goal(2.5, 0.5);
    rbf::AdaptiveGridPartitionQueryOptions options;
    const auto overlay_query = partition.query(start, goal, options);
    assert(overlay_query.found);
    assert(overlay_query.overlay_edges_used >= 1);
    assert(overlay_query.path.size() >= 2);
    assert((overlay_query.path.front() - start).norm() < 1e-12);
    assert((overlay_query.path.back() - goal).norm() < 1e-12);
	    append_boxes.push_back(make_box2(2, 1.0, 2.0, 0.0, 1.0));
	    assert(partition.append_boxes(append_boxes, 2, 1e-9) == 1);
    assert(partition.same_island(0, 1));
    const auto query = partition.query(start, goal, options);
    assert(query.found);
    assert(query.path.size() >= 2);
    assert((query.path.front() - start).norm() < 1e-12);
    assert((query.path.back() - goal).norm() < 1e-12);

    std::vector<rbf::BoxNode> delta_boxes = {
        make_box2(20, 0.0, 1.0, 0.0, 1.0),
        make_box2(21, 1.0, 2.0, 0.0, 1.0),
        make_box2(22, 2.0, 3.0, 0.0, 1.0),
    };
    rbf::AdaptiveGridPartition delta_partition;
    assert(delta_partition.rebuild({{0.0, 4.0}, {0.0, 1.0}},
                                   split,
                                   0,
                                   2,
                                   delta_boxes,
                                   1e-9));
    const auto delta_query_before = delta_partition.query(start, goal, options);
    assert(delta_query_before.found);
    assert(delta_partition.remove_box_ids(std::unordered_set<int>{21}) == 1);
    const auto delta_query_removed = delta_partition.query(start, goal, options);
    assert(!delta_query_removed.found);
    assert(delta_partition.stats().cells == 2);
    assert(delta_partition.stats().islands == 2);
    assert(delta_partition.append_box(make_box2(23, 1.0, 2.0, 0.0, 1.0), 1e-9));
    const auto delta_query_restored = delta_partition.query(start, goal, options);
    assert(delta_query_restored.found);
    assert(delta_query_restored.path.size() >= 2);
    assert(delta_partition.same_island(20, 23));
    assert(delta_partition.same_island(23, 22));

    std::vector<rbf::BoxNode> replace_boxes = {
        make_box2(30, 0.0, 1.0, 0.0, 1.0),
        make_box2(31, 1.0, 2.0, 0.0, 1.0),
        make_box2(32, 2.0, 3.0, 0.0, 1.0),
    };
    rbf::AdaptiveGridPartition replace_partition;
    assert(replace_partition.rebuild({{0.0, 4.0}, {0.0, 1.0}},
                                     split,
                                     0,
                                     2,
                                     replace_boxes,
                                     1e-9));
    replace_boxes.push_back(make_box2(33, 1.0, 2.0, 0.0, 1.0));
    const auto delta = replace_partition.replace_box_ids_with_boxes(
        std::unordered_set<int>{31}, replace_boxes, 3, 1e-9);
    assert(delta.boxes_removed == 1);
    assert(delta.boxes_appended == 1);
    assert(replace_partition.stats().cells == 3);
    assert(replace_partition.query(start, goal, options).found);
    assert(replace_partition.same_island(30, 33));
    assert(replace_partition.same_island(33, 32));

    split.depth_dimensions = {0, 1};
    std::vector<rbf::BoxNode> merge_boxes = {
        make_box2(10, 0.0, 1.0, 0.0, 2.0),
        make_box2(11, 1.0, 2.0, 0.0, 1.0),
        make_box2(12, 1.0, 2.0, 1.0, 2.0),
    };
    rbf::AdaptiveGridPartition merge_partition;
    assert(merge_partition.rebuild({{0.0, 2.0}, {0.0, 2.0}},
                                   split,
                                   0,
                                   2,
                                   merge_boxes,
                                   1e-9));
    rbf::AdaptiveGridPartitionMergeOptions merge_options;
    merge_options.max_rounds = 2;
    merge_options.containment_prune = false;
    const auto merge_result = merge_partition.merge_boxes(merge_boxes,
                                                          merge_options,
                                                          1e-9);
    assert(merge_result.grid_merges >= 1);
    assert(merge_boxes.size() < 3);
    assert(merge_partition.stats().cells == static_cast<int>(merge_boxes.size()));
    assert(merge_partition.stats().islands == 1);
    Eigen::Vector2d merge_start(0.25, 0.25);
    Eigen::Vector2d merge_goal(1.75, 1.75);
    const auto merge_query = merge_partition.query(merge_start, merge_goal, options);
    assert(merge_query.found);
    assert(merge_query.path.size() >= 2);
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
    assert(result.free_boxes.size() == 8);
    assert(result.collision_boxes.empty());
    assert(forest.boxes().size() == result.free_boxes.size());
    bool saw_negative_dim0 = false;
    bool saw_positive_dim0 = false;
    for (const auto& box : result.free_boxes) {
        assert(!box.joint_intervals.empty());
        saw_negative_dim0 = saw_negative_dim0 || box.joint_intervals[0].lo < -1e-12;
        saw_positive_dim0 = saw_positive_dim0 || box.joint_intervals[0].hi > 1e-12;
    }
    assert(saw_negative_dim0);
    assert(saw_positive_dim0);
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
    assert(result.collision_box_obstacle_indices.size() == result.collision_boxes.size());
    for (const auto& blockers : result.collision_box_obstacle_indices) {
        assert(blockers.size() == 1);
        assert(blockers.front() == 0);
    }
    assert(result.free_boxes.empty());
    assert(result.groups.size() == 1);
    assert(result.groups.front().collision_boxes.size() == result.collision_boxes.size());
}

void test_leaf_sweep_grouping_and_composition() {
    auto robot = make_toy_robot();
    auto config = base_config("sbf_leaf_sweep_grouping");
    config.database.canonical_mode = false;
    config.endpoint_source.source = rbf::EndpointSource::IFK;
    config.envelope_type.type = rbf::EnvelopeType::LinkIAABB;
    const std::vector<rbf::Obstacle> obstacles = {
        rbf::Obstacle(100.0f, 100.0f, 100.0f, 101.0f, 101.0f, 101.0f),
        rbf::Obstacle(100.5f, 100.5f, 100.5f, 101.5f, 101.5f, 101.5f),
        rbf::Obstacle(110.0f, 110.0f, 110.0f, 111.0f, 111.0f, 111.0f),
    };
    rbf::LeafSweepConfig sweep_config;
    sweep_config.obstacle_cluster_gap = 0.0;
    sweep_config.use_virtual_topology = false;
    rbf::RBFPlanningForest forest(robot, config);
    auto result = forest.build_leaf_sweep(obstacles, 1, 1, sweep_config);
    assert(result.groups.size() == 2);
    assert(result.obstacle_group_ids.size() == obstacles.size());
    assert(result.obstacle_group_ids[0] == result.obstacle_group_ids[1]);
    assert(result.obstacle_group_ids[2] != result.obstacle_group_ids[0]);
    assert(result.collision_box_obstacle_indices.size() == result.collision_boxes.size());

    rbf::LeafSweepConfig merged_config;
    merged_config.obstacle_cluster_gap = 1000.0;
    merged_config.use_virtual_topology = false;
    auto merged_base = base_config("sbf_leaf_sweep_grouping_merged");
    merged_base.database.canonical_mode = false;
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

void test_leaf_sweep_refined_empty_scene() {
    auto robot = make_toy_robot();
    auto config = base_config("sbf_leaf_sweep_refined_empty");
    config.endpoint_source.source = rbf::EndpointSource::IFK;
    config.envelope_type.type = rbf::EnvelopeType::LinkIAABB;
    config.enable_connector = false;
    config.query.nearest_if_outside = false;
    rbf::RBFPlanningForest forest(robot, config);
    rbf::LeafSweepRefineConfig refine_config;
    refine_config.leaf_start_depth = 1;
    refine_config.leaf_max_depth = 1;
    refine_config.deep_max_boxes = 4;
    refine_config.use_virtual_topology = false;
    auto result = forest.build_leaf_sweep_refined({}, refine_config);
    assert(result.leaf_free_count == 4);
    assert(result.leaf_collision_count == 0);
    assert(result.deep_boxes_added == 0);
    assert(result.profile.final_boxes == 4);
    assert(forest.boxes().size() == 4);
    assert(result.diagnostics.at("leaf_refine.leaf_free_count") == 4.0);
}

void test_adaptive_leaf_sweep_empty_scene() {
    auto robot = make_toy_robot();
    auto config = base_config("sbf_adaptive_leaf_sweep_empty");
    config.endpoint_source.source = rbf::EndpointSource::IFK;
    config.envelope_type.type = rbf::EnvelopeType::LinkIAABB;
    config.enable_connector = false;
    rbf::RBFPlanningForest forest(robot, config);
    rbf::AdaptiveLeafSweepConfig adaptive_config;
    adaptive_config.shallow_start_depth = 1;
    adaptive_config.shallow_max_depth = 1;
    adaptive_config.target_max_depth = 4;
    adaptive_config.time_budget_ms = 1000.0;
    adaptive_config.threads = 1;
    adaptive_config.use_virtual_topology = false;
    adaptive_config.parallel_virtual_validation = false;
    adaptive_config.seed_probe_count = 32;
    adaptive_config.seed_anchor_probe_cap = 0;
    adaptive_config.adaptive_depth_enabled = true;
    adaptive_config.adaptive_depth_min = 1;
    adaptive_config.adaptive_depth_max = 4;
    adaptive_config.adaptive_depth_probe_count = 32;
    adaptive_config.adaptive_depth_anchor_probe_cap = 0;
    adaptive_config.adaptive_depth_min_free_probes = 0;
    adaptive_config.adaptive_depth_min_covered_probes = 4;
    adaptive_config.adaptive_depth_min_main_probes = 4;
    adaptive_config.adaptive_depth_max_online_cells = 16;
    auto result = forest.build_adaptive_deep_leaf_sweep_cover({}, adaptive_config);
    assert(result.shallow_free_count == 4);
    assert(result.shallow_collision_count == 0);
    assert(result.adaptive_free_added == 0);
    assert(result.unresolved_domains == 0);
    assert(result.profile.final_boxes == 4);
    assert(result.seed_probe_free_count == 32);
    assert(result.seed_probe_box_covered == 32);
    assert(result.p_box_covered == 1.0);
    assert(result.selected_leaf_depth == 1);
    assert(result.adaptive_depth_readiness_met);
    assert(result.adaptive_depth_stop_reason == "coverage_ready");
    assert(!result.adaptive_depth_snapshots_json.empty());
    assert(result.diagnostics.at("adaptive.qroot_pairs_total") == 0.0);
    assert(result.diagnostics.find("adaptive.merge_input_boxes") != result.diagnostics.end());
    assert(result.diagnostics.find("adaptive.adjacency_exact_tests") != result.diagnostics.end());
    assert(result.partition_cell_count == 4);
    assert(result.partition_grid_cell_count == 4);
    assert(forest.adjacency().empty());
    Eigen::VectorXd start(2), goal(2);
    start << -0.75, -0.75;
    goal << 0.75, 0.75;
	    auto query = forest.query(start, goal);
	    assert(query.success);
	    assert(query.partition_cells_used > 0);
	    assert(query.partition_search_ms >= 0.0);
	    Eigen::VectorXd outside_start(2), outside_goal(2);
	    outside_start << -1.5, -1.5;
	    outside_goal << 1.5, 1.5;
	    const auto outside_query = forest.query(outside_start, outside_goal);
	    assert(!outside_query.success);
	    assert(outside_query.partition_search_ms >= 0.0);
	    const std::size_t edges_before = forest.segment_edges().size();
	    const int shortcut_added = forest.add_offline_shortcut_edges(1, 4, 1.0, 10.0);
	    assert(shortcut_added >= 0);
	    assert(forest.adjacency().empty());
	    const auto shortcut_diag = forest.last_build_profile().diagnostics.find(
	        "offline_shortcut.partition_native");
	    assert(shortcut_diag != forest.last_build_profile().diagnostics.end());
	    assert(shortcut_diag->second >= 1.0);
	    const auto direct_overlay_diag = forest.last_build_profile().diagnostics.find(
	        "offline_shortcut.partition_native_direct_overlay");
	    assert(direct_overlay_diag != forest.last_build_profile().diagnostics.end());
	    const auto query_bridge_skipped_diag = forest.last_build_profile().diagnostics.find(
	        "offline_shortcut.partition_native_query_bridge_skipped");
	    assert(query_bridge_skipped_diag != forest.last_build_profile().diagnostics.end());
	    assert(forest.segment_edges().size() >= edges_before);
	    for (std::size_t edge_index = edges_before;
	         edge_index < forest.segment_edges().size();
	         ++edge_index) {
	        assert(forest.segment_edges()[edge_index].type == rbf::SegmentEdgeType::BoxCorridor);
	    }
	    const auto fallback = forest.connect_update_segment_fallback();
	    const auto fallback_diag = fallback.diagnostics.find("segment_fallback.partition_native");
	    assert(fallback_diag != fallback.diagnostics.end());
	    assert(fallback_diag->second >= 1.0);
	    assert(forest.adjacency().empty());
	    const auto endpoint_fallback = forest.connect_update_endpoint_segment_fallback(start, goal);
	    const auto endpoint_diag = endpoint_fallback.diagnostics.find(
	        "segment_fallback.endpoint_partition_native");
	    assert(endpoint_diag != endpoint_fallback.diagnostics.end());
	    assert(endpoint_diag->second >= 1.0);
	    assert(forest.adjacency().empty());
	}

void test_leaf_sweep_refined_domain_invariant() {
    auto robot = make_toy_robot();
    auto config = base_config("sbf_leaf_sweep_refined_domain");
    config.endpoint_source.source = rbf::EndpointSource::IFK;
    config.envelope_type.type = rbf::EnvelopeType::LinkIAABB;
    config.enable_connector = false;
    config.grower.find_free_box.max_depth = 4;
    rbf::RBFPlanningForest forest(robot, config);
    const std::vector<rbf::Obstacle> obstacles = {
        rbf::Obstacle(-10.0f, -10.0f, -10.0f, 0.0f, 10.0f, 10.0f),
    };
    rbf::LeafSweepRefineConfig refine_config;
    refine_config.leaf_start_depth = 1;
    refine_config.leaf_max_depth = 2;
    refine_config.deep_max_boxes = 4;
    refine_config.deep_ffb_depth = 4;
    refine_config.domain_seed_cap = 4;
    refine_config.domain_success_cap = 1;
    auto result = forest.build_leaf_sweep_refined(obstacles, refine_config);
    assert(result.profile.final_boxes == static_cast<int>(forest.boxes().size()));
    assert(result.leaf_free_count == static_cast<int>(result.leaf_sweep.free_boxes.size()));
    assert(result.leaf_collision_count == static_cast<int>(result.leaf_sweep.collision_boxes.size()));
    for (const auto& box : forest.boxes()) {
        if (box.id < result.leaf_free_count) {
            continue;
        }
        bool in_domain = false;
        for (const auto& domain : result.leaf_sweep.collision_boxes) {
            if (box.joint_intervals.size() == domain.joint_intervals.size()) {
                bool subset = true;
                for (std::size_t dim = 0; dim < box.joint_intervals.size(); ++dim) {
                    if (box.joint_intervals[dim].lo < domain.joint_intervals[dim].lo - 1e-12 ||
                        box.joint_intervals[dim].hi > domain.joint_intervals[dim].hi + 1e-12) {
                        subset = false;
                        break;
                    }
                }
                if (subset) {
                    in_domain = true;
                    break;
                }
            }
        }
        assert(in_domain);
    }
}

void test_endpoint_main_corridor_already_main_noop() {
    auto robot = make_toy_robot();
    auto config = base_config("sbf_endpoint_main_corridor_noop");
    config.endpoint_source.source = rbf::EndpointSource::IFK;
    config.envelope_type.type = rbf::EnvelopeType::LinkIAABB;
    config.database.canonical_mode = false;
    config.enable_connector = false;
    rbf::RBFPlanningForest forest(robot, config);
    rbf::LeafSweepRefineConfig refine_config;
    refine_config.leaf_start_depth = 1;
    refine_config.leaf_max_depth = 1;
    refine_config.deep_max_boxes = 0;
    refine_config.use_virtual_topology = false;
    auto result = forest.build_leaf_sweep_refined({}, refine_config);
    assert(result.leaf_free_count > 0);
    assert(!forest.boxes().empty());

    const std::size_t boxes_before = forest.boxes().size();
    const std::size_t edges_before = forest.segment_edges().size();
    Eigen::VectorXd point = forest.boxes().front().center();
    rbf::EndpointMainBoxCorridorConfig corridor_config;
    const int added = forest.connect_query_endpoint_to_main_box_corridor(point, corridor_config);
    assert(added == 0);
    assert(forest.boxes().size() == boxes_before);
    assert(forest.segment_edges().size() == edges_before);
    const auto& diagnostics = forest.last_build_profile().diagnostics;
    const auto it = diagnostics.find("endpoint_main.already_main");
    assert(it != diagnostics.end());
    assert(it->second >= 1.0);
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
    config.query.repair_timeout_ms = 1000.0;
    config.connector.rrt.max_iters = 2000;
    config.connector.rrt.timeout_ms = 1000.0;
    rbf::RBFPlanningForest forest(robot, config);
    Eigen::VectorXd start(2), goal(2);
    start << -0.6, -0.2;
    goal << 0.6, 0.2;
    forest.build(start, goal, {});
    auto query = forest.query(start, goal);
    assert(query.success);
    assert(query.audit_passed);
}

}  // namespace

int main() {
    test_runtime_context();
    test_graph();
    test_extract_waypoints_uses_overlap_transitions();
    test_indexed_graph_equivalence();
    test_adaptive_grid_partition_query_matches_graph();
    test_adaptive_grid_partition_append_and_merge();
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
    test_leaf_sweep_refined_empty_scene();
    test_adaptive_leaf_sweep_empty_scene();
    test_leaf_sweep_refined_domain_invariant();
    test_endpoint_main_corridor_already_main_noop();
    test_obstacle_rebuild();
    test_query_audit_gated_repair_without_graph();
    std::cout << "SBF C++ tests passed.\n";
    return 0;
}
