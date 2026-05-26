#include <LECTDatabase/sbf/oracle.h>

#ifdef NDEBUG
#undef NDEBUG
#endif

#include <cassert>
#include <cmath>
#include <filesystem>
#include <optional>
#include <vector>

namespace ld = rbf::lect_database;

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

void require(bool condition) {
    if (!condition) {
        std::abort();
    }
}

void test_database_box_oracle_topology_and_cache() {
    const auto dir = std::filesystem::temp_directory_path() / "lectdb_sbf_adapter_test";
    std::filesystem::remove_all(dir);
    auto robot = make_toy_robot();
    const auto root = robot.joint_limits().limits;

    ld::SplitPolicyDescriptor split;
    split.strategy = ld::SplitStrategy::WidestRoot;
    split.midpoint = true;
    ld::LectDatabaseIdentity identity;
    identity.robot_fingerprint = robot.fingerprint();
    identity.root_domain_fingerprint = ld::fingerprint_intervals(root);
    identity.split_policy_hash = ld::split_policy_hash(split);
    identity.split_policy_descriptor = ld::split_policy_descriptor(split);
    identity.endpoint_descriptor = "planning_database_oracle_ifk";
    identity.envelope_descriptor = "planning_database_oracle_link_iaabb";
    identity.payload_layout = "endpoint_envelope_v1";

    ld::LectDatabaseConfig db_config;
    db_config.path = dir;
    db_config.root_intervals = root;
    db_config.split_policy = split;
    db_config.identity = identity;
    db_config.page_size_bytes = 160;
    db_config.max_resident_pages = 2;
    ld::LectDatabase database;
    std::string reason;
    require(database.open(db_config, &reason));
    require(database.root_intervals().size() == root.size());
    require(database.node_count() == 1);

    rbf::EndpointSourceConfig endpoint_config;
    endpoint_config.source = rbf::EndpointSource::IFK;
    rbf::EnvelopeTypeConfig envelope_config;
    envelope_config.type = rbf::EnvelopeType::LinkIAABB;
    rbf::DatabaseBoxOracle oracle(robot, database, rbf::Scene{}, endpoint_config, envelope_config, {});
    require(oracle.is_leaf(oracle.root_node()));
    const auto split_result = oracle.split_node_at(oracle.root_node(), 0, 0.25);
    require(split_result.split);
    require(std::abs(oracle.split_value(oracle.root_node()) - 0.25) < 1e-12);
    require(database.node_count() == 3);
    require(oracle.left_child(0) == split_result.left);
    require(oracle.right_child(0) == split_result.right);
    require(oracle.select_unexplored_node() == split_result.left);

    oracle.reserve_node(split_result.left, 7);
    require(oracle.is_reserved(split_result.left));
    require(oracle.reservation_owner(split_result.left).value() == 7);
    oracle.release_box(7);
    require(!oracle.is_reserved(split_result.left));

    const std::vector<rbf::Obstacle> obstacles = {rbf::Obstacle(-10.0f, -10.0f, -10.0f, 10.0f, 10.0f, 10.0f)};
    oracle.set_scene(rbf::Scene(obstacles));
    const auto left_box = oracle.node_intervals(split_result.left);
    const auto first = oracle.validate_node(split_result.left, left_box, split_result.split_dim);
    require(first == rbf::BoxValidation::Unknown || first == rbf::BoxValidation::Free);
    require(database.evidence_count() > 0);
    const auto before_hits = oracle.counters().materialization_reused_endpoint_cache;
    (void)oracle.validate_node(split_result.left, left_box, split_result.split_dim);
    require(oracle.counters().materialization_reused_endpoint_cache > before_hits);

    require(database.checkpoint());
    auto reopened = ld::LectDatabase::open_existing(dir, true, &reason);
    require(reopened.has_value());
    require(reopened->verify(true).ok);
    require(reopened->node_count() == database.node_count());
    std::filesystem::remove_all(dir);
}

void test_database_box_oracle_sessions_commit_and_remap() {
    const auto dir = std::filesystem::temp_directory_path() / "lectdb_sbf_session_test";
    std::filesystem::remove_all(dir);
    auto robot = make_toy_robot();
    const auto root = robot.joint_limits().limits;

    ld::SplitPolicyDescriptor split;
    split.strategy = ld::SplitStrategy::WidestRoot;
    split.midpoint = true;
    ld::LectDatabaseIdentity identity;
    identity.robot_fingerprint = robot.fingerprint();
    identity.root_domain_fingerprint = ld::fingerprint_intervals(root);
    identity.split_policy_hash = ld::split_policy_hash(split);
    identity.split_policy_descriptor = ld::split_policy_descriptor(split);
    identity.endpoint_descriptor = "planning_database_oracle_ifk";
    identity.envelope_descriptor = "planning_database_oracle_link_iaabb";
    identity.payload_layout = "endpoint_envelope_v1";

    ld::LectDatabaseConfig db_config;
    db_config.path = dir;
    db_config.root_intervals = root;
    db_config.split_policy = split;
    db_config.identity = identity;
    ld::LectDatabase database;
    std::string reason;
    require(database.open(db_config, &reason));

    rbf::EndpointSourceConfig endpoint_config;
    endpoint_config.source = rbf::EndpointSource::IFK;
    rbf::EnvelopeTypeConfig envelope_config;
    envelope_config.type = rbf::EnvelopeType::LinkIAABB;
    rbf::DatabaseBoxOracle master(robot, database, rbf::Scene{}, endpoint_config, envelope_config, {});
    const auto root_split = master.split_node(master.root_node(), master.root_intervals(), -1, {});
    require(root_split.split);
    require(master.is_leaf(root_split.left));

    {
        rbf::DatabaseBoxOracleFactory factory(master);
        rbf::OracleSessionConfig session_config;
        session_config.worker_id = 3;
        session_config.read_only = true;
        session_config.domain_root = root_split.left;
        auto session = factory.make_session(session_config);
        require(session->domain_root() == root_split.left);
        require(session->map_node_to_master(session->oracle().root_node()) == root_split.left);
        require(session->commit());
    }

    const std::vector<rbf::Obstacle> obstacles = {rbf::Obstacle(-10.0f, -10.0f, -10.0f, 10.0f, 10.0f, 10.0f)};
    master.set_scene(rbf::Scene(obstacles));
    const std::size_t node_count_before = database.node_count();
    const std::size_t evidence_count_before = database.evidence_count();

    rbf::DatabaseBoxOracleFactory factory(master);
    rbf::OracleSessionConfig session_config;
    session_config.worker_id = 4;
    session_config.read_only = false;
    session_config.domain_root = root_split.left;
    auto session = factory.make_session(session_config);
    auto& worker_oracle = session->oracle();
    const auto worker_split = worker_oracle.split_node(worker_oracle.root_node(), worker_oracle.root_intervals(), -1, {});
    require(worker_split.split);
    const auto worker_left_box = worker_oracle.node_intervals(worker_split.left);
    const auto validation = worker_oracle.validate_node(worker_split.left, worker_left_box, worker_split.split_dim);
    require(validation == rbf::BoxValidation::Unknown || validation == rbf::BoxValidation::Free);
    require(session->commit());

    require(database.node_count() == node_count_before + 2);
    const int mapped_root = session->map_node_to_master(worker_oracle.root_node());
    const int mapped_left = session->map_node_to_master(worker_split.left);
    const int mapped_right = session->map_node_to_master(worker_split.right);
    require(mapped_root == root_split.left);
    require(mapped_left == master.left_child(root_split.left));
    require(mapped_right == master.right_child(root_split.left));
    require(database.evidence_count() >= evidence_count_before);

    const auto master_left_box = master.node_intervals(mapped_left);
    const auto before_cache_hit = master.counters().materialization_reused_endpoint_cache;
    (void)master.validate_node(mapped_left, master_left_box, master.split_dim(root_split.left));
    require(master.counters().materialization_reused_endpoint_cache >= before_cache_hit);

    std::filesystem::remove_all(dir);
}

}  // namespace

int main() {
    test_database_box_oracle_topology_and_cache();
    test_database_box_oracle_sessions_commit_and_remap();
    return 0;
}