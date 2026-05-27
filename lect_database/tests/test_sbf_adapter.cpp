#include <LECTDatabase/sbf/oracle.h>

#ifdef NDEBUG
#undef NDEBUG
#endif

#include <cassert>
#include <cmath>
#include <filesystem>
#include <limits>
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

void require_close(std::span<const float> lhs, const std::vector<float>& rhs) {
    require(lhs.size() == rhs.size());
    for (std::size_t i = 0; i < lhs.size(); ++i) {
        require(std::abs(lhs[i] - rhs[i]) < 1e-6f);
    }
}

void require_not_close(std::span<const float> lhs, const std::vector<float>& rhs) {
    require(lhs.size() == rhs.size());
    bool differs = false;
    for (std::size_t i = 0; i < lhs.size(); ++i) {
        if (std::abs(lhs[i] - rhs[i]) >= 1e-6f) {
            differs = true;
            break;
        }
    }
    require(differs);
}

ld::LectDatabaseConfig make_test_db_config(const std::filesystem::path& dir,
                                          const rbf::Robot& robot,
                                          const std::vector<rbf::Interval>& root,
                                          std::string endpoint_descriptor,
                                          std::string envelope_descriptor) {
    ld::SplitPolicyDescriptor split;
    split.strategy = ld::SplitStrategy::WidestRoot;
    split.midpoint = true;

    ld::LectDatabaseIdentity identity;
    identity.robot_fingerprint = robot.fingerprint();
    identity.root_domain_fingerprint = ld::fingerprint_intervals(root);
    identity.split_policy_hash = ld::split_policy_hash(split);
    identity.split_policy_descriptor = ld::split_policy_descriptor(split);
    identity.endpoint_descriptor = std::move(endpoint_descriptor);
    identity.envelope_descriptor = std::move(envelope_descriptor);
    identity.payload_layout = "endpoint_envelope";

    ld::LectDatabaseConfig db_config;
    db_config.path = dir;
    db_config.root_intervals = root;
    db_config.split_policy = split;
    db_config.identity = identity;
    return db_config;
}

void test_database_box_oracle_topology_and_cache() {
    const auto dir = std::filesystem::temp_directory_path() / "lectdb_sbf_adapter_test";
    std::filesystem::remove_all(dir);
    auto robot = make_toy_robot();
    const auto root = robot.joint_limits().limits;

    ld::LectDatabaseConfig db_config = make_test_db_config(
        dir,
        robot,
        root,
        "planning_database_oracle_ifk",
        "planning_database_oracle_link_iaabb");
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

void test_database_box_oracle_hifk_reuses_depth_aligned_split_policy() {
    const auto dir = std::filesystem::temp_directory_path() / "lectdb_sbf_hifk_split_policy_test";
    std::filesystem::remove_all(dir);

    auto robot = make_toy_robot();
    const auto root = robot.joint_limits().limits;

    ld::LectDatabaseConfig db_config = make_test_db_config(
        dir,
        robot,
        root,
        "planning_database_oracle_hifk",
        "planning_database_oracle_link_iaabb");
    db_config.split_policy.strategy = ld::SplitStrategy::AAFKVolumeMin;
    db_config.split_policy.depth_dimensions = {1, 1, 1, 1};
    db_config.identity.split_policy_hash = ld::split_policy_hash(db_config.split_policy);
    db_config.identity.split_policy_descriptor = ld::split_policy_descriptor(db_config.split_policy);

    ld::LectDatabase database;
    std::string reason;
    require(database.open(db_config, &reason));

    rbf::EndpointSourceConfig endpoint_config;
    endpoint_config.source = rbf::EndpointSource::HIFK;
    endpoint_config.hifk_max_depth = 1;
    rbf::EnvelopeTypeConfig envelope_config;
    envelope_config.type = rbf::EnvelopeType::LinkIAABB;

    rbf::DatabaseBoxOracle oracle(robot, database, rbf::Scene{}, endpoint_config, envelope_config, {});
    const auto split_result = oracle.split_node_at(oracle.root_node(), 0, 0.0);
    require(split_result.split);

    const std::vector<rbf::Obstacle> obstacles = {rbf::Obstacle(-10.0f, -10.0f, -10.0f, 10.0f, 10.0f, 10.0f)};
    oracle.set_scene(rbf::Scene(obstacles));
    const auto target_box = oracle.node_intervals(split_result.left);
    (void)oracle.validate_node(split_result.left, target_box, split_result.split_dim);

    ld::EvidenceKey key;
    key.node_id = static_cast<ld::NodeId>(split_result.left);
    const auto topology = database.topology(key.node_id);
    key.node_path = topology.path;
    key.node_path_valid = true;
    key.sector = ld::kPrimarySector;
    key.channel = ld::EvidenceChannel::Safe;
    key.endpoint_source = rbf::EndpointSource::HIFK;
    key.payload_kind = ld::EvidencePayloadKind::EndpointEnvelope;
    const auto stored = database.evidence(key);
    require(stored.has_value());

    rbf::EndpointSourceConfig expected_config = endpoint_config;
    expected_config.hifk_split_strategy = rbf::HifkSplitStrategy::FixedDepthSchedule;
    expected_config.hifk_depth_dimensions = db_config.split_policy.depth_dimensions;
    expected_config.hifk_depth_offset = oracle.depth(split_result.left);
    const auto expected = rbf::compute_endpoint_iaabb(robot, target_box, expected_config);
    require_close(stored->payload, expected.endpoint_iaabbs);

    const auto standalone = rbf::compute_endpoint_iaabb(robot, target_box, endpoint_config);
    require_not_close(stored->payload, standalone.endpoint_iaabbs);

    std::filesystem::remove_all(dir);
}

void test_database_box_oracle_sessions_commit_and_remap() {
    const auto dir = std::filesystem::temp_directory_path() / "lectdb_sbf_session_test";
    std::filesystem::remove_all(dir);
    auto robot = make_toy_robot();
    const auto root = robot.joint_limits().limits;

    ld::LectDatabaseConfig db_config = make_test_db_config(
        dir,
        robot,
        root,
        "planning_database_oracle_ifk",
        "planning_database_oracle_link_iaabb");
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

void test_database_box_oracle_supports_deep_path_keys() {
    const auto dir = std::filesystem::temp_directory_path() / "lectdb_sbf_deep_path_test";
    std::filesystem::remove_all(dir);
    auto robot = make_toy_robot();
    const auto root = robot.joint_limits().limits;

    ld::LectDatabaseConfig db_config = make_test_db_config(
        dir,
        robot,
        root,
        "planning_database_oracle_ifk",
        "planning_database_oracle_link_iaabb");
    db_config.max_tree_depth = 80;
    ld::LectDatabase database;
    std::string reason;
    require(database.open(db_config, &reason));

    rbf::EndpointSourceConfig endpoint_config;
    endpoint_config.source = rbf::EndpointSource::IFK;
    rbf::EnvelopeTypeConfig envelope_config;
    envelope_config.type = rbf::EnvelopeType::LinkIAABB;
    rbf::DatabaseBoxOracle oracle(robot, database, rbf::Scene{}, endpoint_config, envelope_config, {});

    rbf::OracleNodeId node = oracle.root_node();
    for (int depth = 0; depth < 70; ++depth) {
        const auto intervals = oracle.node_intervals(node);
        const auto split = oracle.split_node(node, intervals, -1, {});
        require(split.split);
        require(split.left >= 0);
        require(split.right >= 0);
        require(split.left != split.right);
        node = split.right;
        require(oracle.depth(node) == depth + 1);
        require(!oracle.node_intervals(node).empty());
    }

    require(oracle.depth(node) == 70);
    oracle.reserve_node(node, 99);
    require(oracle.is_reserved(node));
    require(oracle.reservation_owner(node).value() == 99);
    oracle.release_node(node);
    require(!oracle.is_reserved(node));

    std::filesystem::remove_all(dir);
}

void test_external_evidence_reuses_when_handles_differ() {
    const auto active_dir = std::filesystem::temp_directory_path() / "lectdb_sbf_active_mismatch_test";
    const auto external_dir = std::filesystem::temp_directory_path() / "lectdb_sbf_external_mismatch_test";
    std::filesystem::remove_all(active_dir);
    std::filesystem::remove_all(external_dir);

    auto robot = make_toy_robot();
    const auto root = robot.joint_limits().limits;

    auto active_config = make_test_db_config(
        active_dir,
        robot,
        root,
        "planning_database_oracle_ifk",
        "planning_database_oracle_link_iaabb");
    auto external_config = make_test_db_config(
        external_dir,
        robot,
        root,
        "planning_database_oracle_ifk",
        "planning_database_oracle_link_iaabb");

    ld::NodeId active_target = ld::kInvalidNodeId;
    ld::NodeId active_parent = ld::kInvalidNodeId;
    ld::NodeId external_target = ld::kInvalidNodeId;
    std::string reason;
    {
        ld::LectDatabase active_database;
        ld::LectDatabase external_database;
        require(active_database.open(active_config, &reason));
        require(external_database.open(external_config, &reason));

        const auto active_root_children = active_database.split_leaf(active_database.root_node());
        require(ld::valid_node_id(active_root_children.first));
        require(ld::valid_node_id(active_root_children.second));
        const auto active_left_children = active_database.split_leaf(active_root_children.first);
        require(ld::valid_node_id(active_left_children.first));
        const auto active_right_children = active_database.split_leaf(active_root_children.second);
        active_target = active_right_children.first;
        active_parent = active_root_children.second;

        const auto external_root_children = external_database.split_leaf(external_database.root_node());
        require(ld::valid_node_id(external_root_children.first));
        require(ld::valid_node_id(external_root_children.second));
        const auto external_right_children = external_database.split_leaf(external_root_children.second);
        external_target = external_right_children.first;
        require(active_target != external_target);

        ld::EvidenceRecord external_record;
        external_record.key.node_id = external_target;
        external_record.key.sector = ld::kPrimarySector;
        external_record.key.channel = ld::EvidenceChannel::Safe;
        external_record.key.endpoint_source = rbf::EndpointSource::IFK;
        external_record.key.payload_kind = ld::EvidencePayloadKind::EndpointEnvelope;
        external_record.payload.assign(12, 0.0f);
        require(external_database.put_evidence(std::move(external_record)));

        require(active_database.checkpoint());
        require(external_database.checkpoint());
    }

    auto reopened_active = ld::LectDatabase::open_existing(active_dir, false, &reason);
    auto reopened_external = ld::LectDatabase::open_existing(external_dir, true, &reason);
    require(reopened_active.has_value());
    require(reopened_external.has_value());

    rbf::EndpointSourceConfig endpoint_config;
    endpoint_config.source = rbf::EndpointSource::IFK;
    rbf::EnvelopeTypeConfig envelope_config;
    envelope_config.type = rbf::EnvelopeType::LinkIAABB;
    rbf::OracleValidationConfig validation_config;
    validation_config.external_evidence_materialization = true;
    validation_config.external_evidence_backfill_active = false;

    const std::vector<rbf::Obstacle> obstacles = {rbf::Obstacle(-10.0f, -10.0f, -10.0f, 10.0f, 10.0f, 10.0f)};
    rbf::DatabaseBoxOracle oracle(
        robot,
        *reopened_active,
        rbf::Scene(obstacles),
        endpoint_config,
        envelope_config,
        validation_config,
        &*reopened_external);

    const auto target_box = oracle.node_intervals(static_cast<rbf::OracleNodeId>(active_target));
    (void)oracle.validate_node(static_cast<rbf::OracleNodeId>(active_target),
                               target_box,
                               reopened_active->topology(active_parent).split_dim);
    require(oracle.counters().materialization_reused_external_evidence == 1);
    require(oracle.counters().materializations == 0);
    require(reopened_active->evidence_count() == 0);
    require(oracle.last_validation_detail().reused_external_evidence);

    std::filesystem::remove_all(active_dir);
    std::filesystem::remove_all(external_dir);
}

void test_external_child_hull_reuses_unified_envelope_evidence() {
    const auto active_dir = std::filesystem::temp_directory_path() / "lectdb_sbf_active_child_hull_test";
    const auto external_dir = std::filesystem::temp_directory_path() / "lectdb_sbf_external_child_hull_test";
    std::filesystem::remove_all(active_dir);
    std::filesystem::remove_all(external_dir);

    auto robot = make_toy_robot();
    const auto root = robot.joint_limits().limits;

    auto active_config = make_test_db_config(
        active_dir,
        robot,
        root,
        "planning_database_oracle_ifk",
        "planning_database_oracle_link_iaabb");
    auto external_config = make_test_db_config(
        external_dir,
        robot,
        root,
        "planning_database_oracle_ifk",
        "planning_database_oracle_link_iaabb");

    ld::LectDatabase active_database;
    ld::LectDatabase external_database;
    std::string reason;
    require(active_database.open(active_config, &reason));
    require(external_database.open(external_config, &reason));

    ld::EvidenceRecord external_record;
    external_record.key.node_id = 0;
    external_record.key.sector = ld::kPrimarySector;
    external_record.key.channel = ld::EvidenceChannel::Safe;
    external_record.key.endpoint_source = rbf::EndpointSource::IFK;
    external_record.key.payload_kind = ld::EvidencePayloadKind::EndpointEnvelope;
    external_record.child_hull = true;
    external_record.payload.assign(12, 0.0f);
    require(external_database.put_evidence(std::move(external_record)));

    rbf::EndpointSourceConfig endpoint_config;
    endpoint_config.source = rbf::EndpointSource::IFK;
    rbf::EnvelopeTypeConfig envelope_config;
    envelope_config.type = rbf::EnvelopeType::LinkIAABB;
    rbf::OracleValidationConfig validation_config;
    validation_config.external_evidence_materialization = true;
    validation_config.external_evidence_backfill_active = false;

    const std::vector<rbf::Obstacle> obstacles = {rbf::Obstacle(-10.0f, -10.0f, -10.0f, 10.0f, 10.0f, 10.0f)};
    rbf::DatabaseBoxOracle oracle(
        robot,
        active_database,
        rbf::Scene(obstacles),
        endpoint_config,
        envelope_config,
        validation_config,
        &external_database);

    const auto root_box = oracle.root_intervals();
    (void)oracle.validate_node(oracle.root_node(), root_box, -1);
    require(oracle.counters().materialization_reused_external_evidence == 1);
    require(oracle.counters().materializations == 0);
    require(active_database.evidence_count() == 0);
    require(oracle.last_validation_detail().reused_external_evidence);

    std::filesystem::remove_all(active_dir);
    std::filesystem::remove_all(external_dir);
}

}  // namespace

int main() {
    test_database_box_oracle_topology_and_cache();
    test_database_box_oracle_hifk_reuses_depth_aligned_split_policy();
    test_database_box_oracle_sessions_commit_and_remap();
    test_database_box_oracle_supports_deep_path_keys();
    test_external_evidence_reuses_when_handles_differ();
    test_external_child_hull_reuses_unified_envelope_evidence();
    return 0;
}