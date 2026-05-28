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

class ThresholdOracle final : public rbf::BoxOracle {
public:
    explicit ThresholdOracle(int free_depth,
                             int max_tree_depth = 16,
                                                         rbf::BoxValidation blocked_validation = rbf::BoxValidation::Unknown,
                                                         bool reuse_external_evidence = false)
        : free_depth_(free_depth),
          max_tree_depth_(std::max(max_tree_depth, free_depth + 1)),
                    blocked_validation_(blocked_validation),
                    reuse_external_evidence_(reuse_external_evidence) {
        Node root;
        root.depth = 0;
        root.intervals = {{0.0, 1.0}, {0.0, 1.0}};
        nodes_.push_back(root);
    }

    int n_dims() const override { return 2; }
    rbf::OracleNodeId root_node() const override { return 0; }
    int max_tree_depth() const override { return max_tree_depth_; }
    const std::vector<rbf::Interval>& root_intervals() const override {
        return nodes_.front().intervals;
    }
    std::vector<rbf::Interval> node_intervals(rbf::OracleNodeId node) const override {
        return nodes_[static_cast<std::size_t>(node)].intervals;
    }
    bool contains_point(rbf::OracleNodeId node,
                        const Eigen::Ref<const Eigen::VectorXd>& q) const override {
        if (q.size() != n_dims()) {
            return false;
        }
        const auto& intervals = nodes_[static_cast<std::size_t>(node)].intervals;
        for (int dim = 0; dim < q.size(); ++dim) {
            const auto& interval = intervals[static_cast<std::size_t>(dim)];
            if (q[dim] < interval.lo || q[dim] > interval.hi) {
                return false;
            }
        }
        return true;
    }
    bool is_leaf(rbf::OracleNodeId node) const override {
        return nodes_[static_cast<std::size_t>(node)].leaf;
    }
    int depth(rbf::OracleNodeId node) const override {
        return nodes_[static_cast<std::size_t>(node)].depth;
    }
    int split_dim(rbf::OracleNodeId node) const override {
        return nodes_[static_cast<std::size_t>(node)].split_dim;
    }
    double split_value(rbf::OracleNodeId node) const override {
        return nodes_[static_cast<std::size_t>(node)].split_value;
    }
    rbf::OracleNodeId left_child(rbf::OracleNodeId node) const override {
        return nodes_[static_cast<std::size_t>(node)].left;
    }
    rbf::OracleNodeId right_child(rbf::OracleNodeId node) const override {
        return nodes_[static_cast<std::size_t>(node)].right;
    }
    rbf::SplitNodeResult split_node(rbf::OracleNodeId node,
                                    const std::vector<rbf::Interval>& intervals,
                                    int,
                                    const rbf::OracleSplitOptions&) override {
        if (!is_leaf(node) || depth(node) >= max_tree_depth_ - 1) {
            return {};
        }
        const int dim = depth(node) % n_dims();
        const auto& interval = intervals[static_cast<std::size_t>(dim)];
        const double split = 0.5 * (interval.lo + interval.hi);
        return split_node_impl(node, dim, split, intervals);
    }
    rbf::SplitNodeResult split_node_at(rbf::OracleNodeId node, int split_dim, double split_value) override {
        if (!is_leaf(node) || depth(node) >= max_tree_depth_ - 1) {
            return {};
        }
        return split_node_impl(node,
                               split_dim,
                               split_value,
                               nodes_[static_cast<std::size_t>(node)].intervals);
    }
    bool point_in_collision(const Eigen::Ref<const Eigen::VectorXd>&) const override { return false; }
    rbf::BoxValidation validate_node(rbf::OracleNodeId node,
                                     const std::vector<rbf::Interval>&,
                                     int = -1) override {
        return classify(node, free_depth_, reuse_external_evidence_);
    }
    bool validate_intervals(const std::vector<rbf::Interval>&) override { return true; }
    bool is_reserved(rbf::OracleNodeId) const override { return false; }
    std::optional<int> reservation_owner(rbf::OracleNodeId) const override { return std::nullopt; }
    void reserve_node(rbf::OracleNodeId, int) override {}
    void release_node(rbf::OracleNodeId) override {}
    void release_box(int) override {}
    void clear_reservations() override {}
    rbf::OracleNodeId select_unexplored_node() const override { return -1; }
    rbf::OracleValidationDetail last_validation_detail() const override { return last_detail_; }
    const rbf::OracleCounters& counters() const override { return counters_; }
    void reset_counters() override {
        counters_ = {};
        last_detail_ = {};
    }

private:
    struct Node {
        int depth = 0;
        bool leaf = true;
        int split_dim = 0;
        double split_value = 0.0;
        rbf::OracleNodeId left = rbf::kInvalidOracleNodeId;
        rbf::OracleNodeId right = rbf::kInvalidOracleNodeId;
        std::vector<rbf::Interval> intervals;
    };

    rbf::SplitNodeResult split_node_impl(rbf::OracleNodeId node,
                                         int split_dim,
                                         double split_value,
                                         const std::vector<rbf::Interval>& intervals) {
        if (split_dim < 0 || split_dim >= static_cast<int>(intervals.size())) {
            return {};
        }
        const auto& interval = intervals[static_cast<std::size_t>(split_dim)];
        if (split_value <= interval.lo || split_value >= interval.hi) {
            return {};
        }

        Node left_node;
        left_node.depth = depth(node) + 1;
        left_node.intervals = intervals;
        left_node.intervals[static_cast<std::size_t>(split_dim)].hi = split_value;

        Node right_node;
        right_node.depth = depth(node) + 1;
        right_node.intervals = intervals;
        right_node.intervals[static_cast<std::size_t>(split_dim)].lo = split_value;

        const auto left = static_cast<rbf::OracleNodeId>(nodes_.size());
        nodes_.push_back(std::move(left_node));
        const auto right = static_cast<rbf::OracleNodeId>(nodes_.size());
        nodes_.push_back(std::move(right_node));

        auto& current = nodes_[static_cast<std::size_t>(node)];
        current.leaf = false;
        current.split_dim = split_dim;
        current.split_value = split_value;
        current.left = left;
        current.right = right;
        return {
            .split = true,
            .node = node,
            .left = left,
            .right = right,
            .split_dim = split_dim,
            .split_value = split_value,
        };
    }

    int free_depth_ = 0;
    int max_tree_depth_ = 0;
    rbf::BoxValidation blocked_validation_ = rbf::BoxValidation::Unknown;
    bool reuse_external_evidence_ = false;
    std::vector<Node> nodes_;
    rbf::OracleCounters counters_;
    rbf::OracleValidationDetail last_detail_;

    rbf::BoxValidation classify(rbf::OracleNodeId node,
                                int free_depth,
                                bool reused_external_evidence) {
        counters_.node_validations += 1;
        last_detail_ = {};
        last_detail_.node = node;
        last_detail_.depth = depth(node);
        last_detail_.reused_external_evidence = reused_external_evidence;
        if (depth(node) >= free_depth) {
            counters_.certified_free += 1;
            last_detail_.validation = rbf::BoxValidation::Free;
            last_detail_.collision_possible = false;
            return rbf::BoxValidation::Free;
        }
        if (blocked_validation_ == rbf::BoxValidation::Occupied) {
            counters_.collision_possible += 1;
            last_detail_.validation = rbf::BoxValidation::Occupied;
            last_detail_.collision_possible = true;
            return rbf::BoxValidation::Occupied;
        }
        counters_.collision_possible += 1;
        last_detail_.validation = rbf::BoxValidation::Unknown;
        last_detail_.collision_possible = true;
        return rbf::BoxValidation::Unknown;
    }
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

double diagnostic_value(const std::unordered_map<std::string, double>& diagnostics,
                        const std::string& key) {
    const auto it = diagnostics.find(key);
    return it == diagnostics.end() ? 0.0 : it->second;
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

void test_query_strict_path_audit() {
    auto robot = make_toy_robot();
    auto config = base_config("sbf_query_strict_path_audit");
    config.enable_connector = false;
    config.query.strict_path_audit = true;
    config.query.audit_resolution = 8;
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

void test_find_free_box_binary_accepts_start_depth() {
    ThresholdOracle oracle(2, 10);
    rbf::FindFreeBoxService service(oracle);
    Eigen::VectorXd seed(2);
    seed << 0.25, 0.25;

    rbf::FindFreeBoxOptions options;
    options.search_mode = rbf::FindFreeBoxSearchMode::BinaryDepth;
    options.start_depth = 2;
    options.max_depth = 6;

    const auto result = service.find(seed, options);
    assert(result.found);
    assert(oracle.depth(result.node) == 2);
    assert(result.decisions == 1);
}

void test_find_free_box_binary_returns_shallowest_free_depth() {
    ThresholdOracle oracle(5, 12);
    rbf::FindFreeBoxService service(oracle);
    Eigen::VectorXd seed(2);
    seed << 0.25, 0.25;

    rbf::FindFreeBoxOptions options;
    options.search_mode = rbf::FindFreeBoxSearchMode::BinaryDepth;
    options.start_depth = 2;
    options.max_depth = 8;

    const auto result = service.find(seed, options);
    assert(result.found);
    assert(oracle.depth(result.node) == 5);
    assert(result.decisions == 5);
}

void test_find_free_box_binary_fails_when_max_depth_not_free() {
    ThresholdOracle oracle(9, 12);
    rbf::FindFreeBoxService service(oracle);
    Eigen::VectorXd seed(2);
    seed << 0.25, 0.25;

    rbf::FindFreeBoxOptions options;
    options.search_mode = rbf::FindFreeBoxSearchMode::BinaryDepth;
    options.start_depth = 2;
    options.max_depth = 8;

    const auto result = service.find(seed, options);
    assert(!result.found);
    assert(result.fail_code == 2);
    assert(result.hit_unknown_depth_cap);
    assert(oracle.depth(result.node) == 8);
    assert(result.decisions == 2);
}

void test_find_free_box_binary_treats_occupied_as_nonfree() {
    ThresholdOracle oracle(4, 12, rbf::BoxValidation::Occupied);
    rbf::FindFreeBoxService service(oracle);
    Eigen::VectorXd seed(2);
    seed << 0.25, 0.25;

    rbf::FindFreeBoxOptions options;
    options.search_mode = rbf::FindFreeBoxSearchMode::BinaryDepth;
    options.start_depth = 1;
    options.max_depth = 6;

    const auto result = service.find(seed, options);
    assert(result.found);
    assert(oracle.depth(result.node) == 4);
}

void test_find_free_box_linear_mode_ignores_start_depth() {
    ThresholdOracle oracle(3, 10);
    rbf::FindFreeBoxService service(oracle);
    Eigen::VectorXd seed(2);
    seed << 0.25, 0.25;

    rbf::FindFreeBoxOptions options;
    options.search_mode = rbf::FindFreeBoxSearchMode::Linear;
    options.start_depth = 5;
    options.max_depth = 6;

    const auto result = service.find(seed, options);
    assert(result.found);
    assert(oracle.depth(result.node) == 3);
    assert(result.decisions == 4);
}

void test_find_free_box_binary_external_unknown_stays_nonfree_at_max_depth() {
    ThresholdOracle oracle(99, 12, rbf::BoxValidation::Unknown, true);
    rbf::FindFreeBoxService service(oracle);
    Eigen::VectorXd seed(2);
    seed << 0.25, 0.25;

    rbf::FindFreeBoxOptions options;
    options.search_mode = rbf::FindFreeBoxSearchMode::BinaryDepth;
    options.start_depth = 2;
    options.max_depth = 6;

    const auto result = service.find(seed, options);
    assert(!result.found);
    assert(result.hit_unknown_depth_cap);
    assert(result.fail_code == 2);
    assert(oracle.depth(result.node) == 6);
    assert(result.decisions == 2);
    assert(result.validation_detail.reused_external_evidence);
}

void test_build_subtractive_hits_domain_binary_ffb() {
    auto robot = make_toy_robot();
    auto config = base_config("sbf_subtractive_domain_binary");
    config.enable_merger = false;
    config.enable_connector = false;
    config.runtime.mode = rbf::ExecutionMode::Inline;
    config.grower.mode = rbf::GrowerConfig::Mode::Frontwave;
    config.grower.max_boxes = 24;
    config.grower.max_consecutive_miss = 256;
    config.grower.find_free_box.max_depth = 6;
    config.grower.find_free_box.start_depth = 2;
    config.grower.find_free_box.search_mode = rbf::FindFreeBoxSearchMode::BinaryDepth;
    config.endpoint_source.source = rbf::EndpointSource::IFK;
    config.envelope_type.type = rbf::EnvelopeType::LinkIAABB;
    config.dynamic_update.dirty_region_padding = 0.0;
    config.dynamic_update.dirty_seed_limit = 16;

    rbf::RBFPlanningForest forest(robot, config);

    rbf::SubtractiveObstacleGroup group;
    group.name = "domain_binary_regrow";
    group.carving_obstacles = {
        rbf::Obstacle(0.14f, -0.37f, -0.1f, 0.46f, -0.13f, 0.1f),
    };
    group.validation_obstacles = group.carving_obstacles;

    std::vector<Eigen::VectorXd> seeds;
    for (const std::array<double, 2>& values : {
             std::array<double, 2>{0.0, 0.0},
             std::array<double, 2>{0.4, -0.2},
             std::array<double, 2>{-0.4, 0.2},
             std::array<double, 2>{0.6, 0.3},
             std::array<double, 2>{-0.6, -0.3},
         }) {
        Eigen::VectorXd seed(2);
        seed << values[0], values[1];
        seeds.push_back(seed);
    }

    const auto profile = forest.build_subtractive({group}, seeds);
    assert(profile.final_boxes > 0);
    assert(diagnostic_value(profile.diagnostics, "subtractive.regrow_seeds") > 0.0);
    assert(diagnostic_value(profile.diagnostics, "subtractive.carve_boxes_removed") > 0.0);
    assert(diagnostic_value(profile.diagnostics, "subtractive.carve_boxes_added") > 0.0);
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
    test_query_strict_path_audit();
    test_safe_box_forest_frontwave();
    test_obstacle_rebuild();
    test_query_audit_gated_repair_without_graph();
    test_find_free_box_binary_accepts_start_depth();
    test_find_free_box_binary_returns_shallowest_free_depth();
    test_find_free_box_binary_fails_when_max_depth_not_free();
    test_find_free_box_binary_treats_occupied_as_nonfree();
    test_find_free_box_linear_mode_ignores_start_depth();
    test_find_free_box_binary_external_unknown_stays_nonfree_at_max_depth();
    test_build_subtractive_hits_domain_binary_ffb();
    std::cout << "SBF C++ tests passed.\n";
    return 0;
}
