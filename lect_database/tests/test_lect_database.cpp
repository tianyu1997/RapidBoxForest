#include <rbf/lect_database.h>

#ifdef NDEBUG
#undef NDEBUG
#endif

#include <cassert>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace ld = rbf::lect_database;

namespace {

constexpr std::uint64_t kBinaryEvidenceStoreMagic = 0x3156454242464652ull;
constexpr std::uint64_t kBinaryEvidenceIndexSidecarMagic = 0x3158444945424652ull;

#pragma pack(push, 1)
struct BinaryEvidenceIndexSidecarHeader {
    std::uint64_t magic = 0;
    std::uint32_t version = 0;
    std::uint32_t header_size = 0;
    std::uint64_t evidence_file_size = 0;
    std::uint64_t entry_count = 0;
    std::uint64_t path_blob_offset = 0;
};

struct BinaryEvidenceIndexSidecarEntry {
    std::uint64_t node_id = 0;
    std::int32_t sector = 0;
    std::uint32_t channel = 0;
    std::uint32_t endpoint_source = 0;
    std::uint32_t payload_kind = 0;
    std::uint32_t flags = 0;
    std::uint32_t size = 0;
    std::uint32_t path_word_count = 0;
    std::uint32_t path_bit_count = 0;
    std::uint64_t path_blob_offset = 0;
    std::uint64_t offset = 0;
    std::uint64_t generation = 0;
    std::uint64_t checksum = 0;
};
#pragma pack(pop)

static_assert(sizeof(BinaryEvidenceIndexSidecarHeader) == 40);
static_assert(sizeof(BinaryEvidenceIndexSidecarEntry) == 72);

std::vector<rbf::Interval> root2() {
    return {{0.0, 2.0}, {0.0, 2.0}};
}

ld::SplitPolicyDescriptor split_policy() {
    ld::SplitPolicyDescriptor descriptor;
    descriptor.strategy = ld::SplitStrategy::RoundRobin;
    descriptor.min_width = 0.0;
    descriptor.midpoint = true;
    descriptor.deterministic_tie_break = true;
    return descriptor;
}

ld::LectDatabaseIdentity identity_for(const std::vector<rbf::Interval>& root,
                                      const ld::SplitPolicyDescriptor& split) {
    ld::LectDatabaseIdentity identity;
    identity.robot_fingerprint = 42;
    identity.root_domain_fingerprint = ld::fingerprint_intervals(root);
    identity.split_policy_hash = ld::split_policy_hash(split);
    identity.split_policy_descriptor = ld::split_policy_descriptor(split);
    identity.endpoint_descriptor = "test_endpoint";
    identity.envelope_descriptor = "test_envelope";
    identity.payload_layout = "endpoint_envelope_v1";
    return identity;
}

ld::LectDatabaseConfig config_for(const std::filesystem::path& dir) {
    ld::LectDatabaseConfig config;
    config.path = dir;
    config.root_intervals = root2();
    config.split_policy = split_policy();
    config.identity = identity_for(config.root_intervals, config.split_policy);
    config.page_size_bytes = 256;
    config.max_resident_pages = 2;
    return config;
}

ld::LectDatabase make_database(const std::filesystem::path& dir) {
    std::filesystem::remove_all(dir);
    ld::LectDatabase database;
    std::string reason;
    assert(database.open(config_for(dir), &reason));
    assert(reason.empty());
    return database;
}

std::uint64_t payload_checksum(const std::vector<float>& payload) {
    std::uint64_t hash = 1469598103934665603ull;
    if (!payload.empty()) {
        hash = ld::stable_hash_append(hash, payload.data(), payload.size() * sizeof(float));
    }
    return hash;
}

std::string serialize_legacy_payload(const std::vector<float>& payload) {
    std::ostringstream out;
    out << payload.size();
    for (float value : payload) {
        out << ',' << value;
    }
    return out.str();
}

std::string serialize_legacy_evidence_record(const ld::EvidenceRecord& record) {
    std::ostringstream out;
    out << record.key.node_id << '|'
        << record.key.sector << '|'
        << static_cast<int>(record.key.channel) << '|'
        << static_cast<int>(record.key.endpoint_source) << '|'
        << static_cast<int>(record.key.payload_kind) << '|'
        << (record.child_hull ? 1 : 0) << '|'
        << (record.unavailable ? 1 : 0) << '|'
        << record.generation << '|'
        << record.checksum << '|'
        << serialize_legacy_payload(record.payload);
    return out.str();
}

void test_identity_rejection() {
    const auto dir = std::filesystem::temp_directory_path() / "rbf_lect_database_identity";
    auto database = make_database(dir);
    assert(database.checkpoint());

    auto config = config_for(dir);
    config.identity.endpoint_descriptor = "different_endpoint";
    ld::LectDatabase rejected;
    std::string reason;
    assert(!rejected.open(config, &reason));
    assert(reason.find("endpoint descriptor") != std::string::npos);
    std::filesystem::remove_all(dir);
}

void test_max_tree_depth_limit() {
    const auto dir = std::filesystem::temp_directory_path() / "rbf_lect_database_max_tree_depth";
    std::filesystem::remove_all(dir);

    auto config = config_for(dir);
    config.max_tree_depth = 2;
    ld::LectDatabase database;
    std::string reason;
    assert(database.open(config, &reason));
    assert(database.max_tree_depth() == 2);

    const auto root_children = database.split_leaf(database.root_node());
    assert(root_children.first == 1);
    assert(root_children.second == 2);
    assert(!ld::valid_node_id(database.split_leaf(root_children.first).first));
    assert(!database.ensure_depth(2));
    assert(database.checkpoint());

    auto reopened = ld::LectDatabase::open_existing(dir, true, &reason);
    assert(reopened.has_value());
    assert(reopened->max_tree_depth() == 2);

    auto invalid_config = config_for(dir / "invalid");
    invalid_config.max_tree_depth = 0;
    ld::LectDatabase invalid_database;
    reason.clear();
    assert(!invalid_database.open(invalid_config, &reason));
    assert(reason.find("positive") != std::string::npos);

    auto deep_config = config_for(dir / "deep");
    deep_config.max_tree_depth = 80;
    ld::LectDatabase deep_database;
    reason.clear();
    assert(deep_database.open(deep_config, &reason));
    ld::NodeId cursor = deep_database.root_node();
    for (int depth = 0; depth < 70; ++depth) {
        const auto children = deep_database.split_leaf(cursor);
        assert(ld::valid_node_id(children.first));
        assert(ld::valid_node_id(children.second));
        cursor = children.first;
        const auto topology = deep_database.topology(cursor);
        assert(topology.depth == depth + 1);
        assert(topology.path.bit_count == depth + 1);
    }
    assert(deep_database.topology(cursor).path.words.size() >= 2);
    assert(deep_database.verify(true).ok);
    std::filesystem::remove_all(dir);
}

void test_topology_box_and_range_queries() {
    const auto dir = std::filesystem::temp_directory_path() / "rbf_lect_database_topology";
    {
        auto database = make_database(dir);
        assert(database.ensure_depth(2));
        assert(database.node_count() == 7);
        assert(database.layer_nodes(0).size() == 1);
        assert(database.layer_nodes(1).size() == 2);
        assert(database.layer_nodes(2).size() == 4);

        const auto root_children = database.children(database.root_node());
        assert(root_children.first == 1);
        assert(root_children.second == 2);
        assert(database.parent(1) == database.root_node());
        assert(database.sibling(1) == 2);
        assert(database.is_ancestor(database.root_node(), 6));
        assert(database.lca(3, 4) == 1);
        assert(database.lca(3, 6) == database.root_node());

        const auto node3_box = database.node_box(3);
        assert(node3_box.has_value());
        assert((*node3_box)[0].lo == 0.0 && (*node3_box)[0].hi == 1.0);
        assert((*node3_box)[1].lo == 0.0 && (*node3_box)[1].hi == 1.0);
        const auto lookup = database.box_to_node_exact(database.make_box_key(*node3_box));
        assert(lookup.found && lookup.node_id == 3);

        auto wrong_policy_key = database.make_box_key(*node3_box);
        wrong_policy_key.split_policy_hash ^= 123u;
        assert(!database.box_to_node_exact(wrong_policy_key).found);

        auto off_grid = database.make_box_key({{0.25, 0.75}, {0.0, 1.0}});
        assert(!database.box_to_node_exact(off_grid).found);

        const std::vector<rbf::Interval> query = {{0.25, 0.75}, {0.25, 0.75}};
        ld::LectDatabaseStats stats;
        const auto covering = database.range_query(query, ld::RangeQueryMode::CoveringFrontier, &stats);
        assert(covering.size() == 1 && covering.front() == 3);
        assert(stats.range_nodes_visited > 0);
        assert(stats.range_nodes_visited < database.node_count());
        const auto containing = database.range_query(query, ld::RangeQueryMode::Containing);
        assert(containing.size() == 3);
        const auto root_contained = database.range_query(root2(), ld::RangeQueryMode::ContainedBy);
        assert(root_contained.size() == database.node_count());

        auto session = database.make_query_session();
        assert(session.node_box(3).has_value());
        assert(session.node_box(3).has_value());
        assert(session.stats().query_path_cache_hits == 1);

        const auto verify = database.verify(true);
        assert(verify.ok);
    }
    std::filesystem::remove_all(dir);
}

void test_depth_synchronous_split_dimensions() {
    const auto dir = std::filesystem::temp_directory_path() / "rbf_lect_database_depth_sync";
    {
        auto database = make_database(dir);
        assert(database.ensure_depth(4));

        for (int depth = 0; depth < 4; ++depth) {
            const int expected_dim = depth % 2;
            for (const auto node_id : database.layer_nodes(depth)) {
                const auto node = database.node(node_id);
                assert(node.has_value());
                assert(!node->is_leaf());
                assert(node->split_dim == expected_dim);
            }
        }
    }
    std::filesystem::remove_all(dir);
}

void test_split_to_box_and_persistence() {
    const auto dir = std::filesystem::temp_directory_path() / "rbf_lect_database_persist";
    {
        auto database = make_database(dir);
        const auto target = database.make_box_key({{1.0, 2.0}, {1.0, 2.0}});
        const auto split = database.split_to_box(target, 4);
        assert(split.found);
        const auto split_box = database.node_box(split.node_id);
        assert(split_box.has_value());
        assert((*split_box)[0].lo == 1.0 && (*split_box)[0].hi == 2.0);
        assert((*split_box)[1].lo == 1.0 && (*split_box)[1].hi == 2.0);
        assert(database.checkpoint());

        std::string reason;
        auto reopened = ld::LectDatabase::open_existing(dir, true, &reason);
        assert(reopened.has_value());
        assert(reopened->node_count() == database.node_count());
        const auto lookup = reopened->box_to_node_exact(target);
        assert(lookup.found && lookup.node_id == split.node_id);
        assert(reopened->verify(true).ok);
    }
    std::filesystem::remove_all(dir);
}

void test_path_keys_are_split_order_independent() {
    const auto dir_a = std::filesystem::temp_directory_path() / "rbf_lect_database_path_keys_a";
    const auto dir_b = std::filesystem::temp_directory_path() / "rbf_lect_database_path_keys_b";
    ld::NodeId target_a = ld::kInvalidNodeId;
    {
        auto database = make_database(dir_a);
        const auto root_children = database.split_leaf(database.root_node());
        assert(root_children.first == 1 && root_children.second == 2);
        const auto right_children = database.split_leaf(root_children.second);
        target_a = right_children.first;
        assert(database.node_count() == 5);

        ld::EvidenceRecord record;
        record.key.node_id = target_a;
        record.key.sector = ld::kPrimarySector;
        record.key.channel = ld::EvidenceChannel::Safe;
        record.key.endpoint_source = rbf::EndpointSource::IFK;
        record.key.payload_kind = ld::EvidencePayloadKind::EndpointEnvelope;
        record.payload = {3.0f, 4.0f, 5.0f, 6.0f};
        assert(database.put_evidence(record));

        assert(database.checkpoint());
        std::string reason;
        auto reopened = ld::LectDatabase::open_existing(dir_a, true, &reason);
        assert(reopened.has_value());
        assert(reopened->node_count() == 5);
        assert(reopened->node(target_a).has_value());
        assert(reopened->verify(true).ok);
    }
    {
        auto database_a = ld::LectDatabase::open_existing(dir_a, true).value();
        auto database_b = make_database(dir_b);
        const auto root_children = database_b.split_leaf(database_b.root_node());
        assert(root_children.first == 1 && root_children.second == 2);
        const auto left_children = database_b.split_leaf(root_children.first);
        assert(left_children.first == 3 && left_children.second == 4);
        const auto right_children = database_b.split_leaf(root_children.second);
        assert(right_children.first == 5 && right_children.second == 6);
        const ld::NodeId target_b = right_children.first;
        assert(target_a != target_b);

        ld::EvidenceKey cross_db_key;
        cross_db_key.node_id = target_b;
        cross_db_key.node_path = database_b.topology(target_b).path;
        cross_db_key.node_path_valid = true;
        cross_db_key.sector = ld::kPrimarySector;
        cross_db_key.channel = ld::EvidenceChannel::Safe;
        cross_db_key.endpoint_source = rbf::EndpointSource::IFK;
        cross_db_key.payload_kind = ld::EvidencePayloadKind::EndpointEnvelope;

        const auto reused = database_a.evidence(cross_db_key);
        assert(reused.has_value());
        assert(reused->key.node_id == target_a);
        assert(reused->payload.size() == 4);
        assert(reused->payload[0] == 3.0f);
        assert(reused->payload[3] == 6.0f);
    }
    std::filesystem::remove_all(dir_a);
    std::filesystem::remove_all(dir_b);
}

void test_journal_replay_truncated_tail() {
    const auto dir = std::filesystem::temp_directory_path() / "rbf_lect_database_journal";
    {
        auto database = make_database(dir);
        assert(database.split_leaf(database.root_node()).first == 1);
    }

    {
        std::ofstream out(dir / "journal.log", std::ios::app);
        out << "begin|999\n";
        out << "split|1|3|4|1|1.0\n";
    }

    std::string reason;
    auto reopened = ld::LectDatabase::open_existing(dir, true, &reason);
    assert(reopened.has_value());
    assert(reopened->node_count() == 3);
    assert(reopened->verify(true).ok);
    std::filesystem::remove_all(dir);
}

void test_evidence_journal_replay_without_checkpoint() {
    const auto dir = std::filesystem::temp_directory_path() / "rbf_lect_database_evidence_journal";
    ld::EvidenceKey left_key;
    {
        auto database = make_database(dir);
        const auto children = database.split_leaf(database.root_node());
        assert(children.first == 1 && children.second == 2);

        left_key.node_id = children.first;
        left_key.sector = ld::kPrimarySector;
        left_key.channel = ld::EvidenceChannel::Safe;
        left_key.endpoint_source = rbf::EndpointSource::IFK;
        left_key.payload_kind = ld::EvidencePayloadKind::EndpointEnvelope;

        ld::EvidenceRecord left;
        left.key = left_key;
        left.payload = {1.0f, 2.0f, 3.0f, 4.0f};
        assert(database.put_evidence(left));

        ld::EvidenceRecord right;
        right.key = left_key;
        right.key.node_id = children.second;
        right.payload = {-2.0f, 3.0f, 2.5f, 9.0f};
        assert(database.put_evidence(right));
    }

    std::string reason;
    auto reopened = ld::LectDatabase::open_existing(dir, true, &reason);
    assert(reopened.has_value());
    const auto left_replayed = reopened->evidence(left_key);
    assert(left_replayed.has_value());
    auto parent_key = left_key;
    parent_key.node_id = reopened->root_node();
    const auto parent_replayed = reopened->evidence(parent_key);
    assert(parent_replayed.has_value());
    assert(parent_replayed->child_hull);
    assert(parent_replayed->payload[0] == -2.0f);
    assert(parent_replayed->payload[1] == 3.0f);
    assert(reopened->verify(true).ok);
    std::filesystem::remove_all(dir);
}

void test_evidence_parent_hull_and_exact_box_lookup() {
    const auto dir = std::filesystem::temp_directory_path() / "rbf_lect_database_evidence";
    auto database = make_database(dir);
    const auto children = database.split_leaf(database.root_node());
    assert(children.first == 1 && children.second == 2);

    ld::EvidenceKey key;
    key.node_id = children.first;
    key.sector = ld::kPrimarySector;
    key.channel = ld::EvidenceChannel::Safe;
    key.endpoint_source = rbf::EndpointSource::IFK;
    key.payload_kind = ld::EvidencePayloadKind::EndpointEnvelope;

    ld::EvidenceRecord left;
    left.key = key;
    left.payload = {0.0f, 1.0f, 2.0f, 3.0f};
    assert(database.put_evidence(left));

    ld::EvidenceRecord right;
    right.key = key;
    right.key.node_id = children.second;
    right.payload = {-1.0f, 2.0f, 1.0f, 4.0f};
    assert(database.put_evidence(right));

    auto parent_key = key;
    parent_key.node_id = database.root_node();
    auto parent = database.evidence(parent_key);
    assert(parent.has_value());
    assert(parent->child_hull);
    assert(parent->payload.size() == 4);
    assert(parent->payload[0] == -1.0f);
    assert(parent->payload[1] == 2.0f);
    assert(parent->payload[2] == 1.0f);
    assert(parent->payload[3] == 4.0f);

    ld::EvidenceRecord stale_direct;
    stale_direct.key = parent_key;
    stale_direct.payload = {9.0f, 9.0f, 9.0f, 9.0f};
    assert(database.put_evidence(stale_direct));
    parent = database.evidence(parent_key);
    assert(parent.has_value());
    assert(parent->child_hull);
    assert(parent->payload[0] == -1.0f);

    auto left_box = database.node_box(children.first);
    assert(left_box.has_value());
    auto by_box = database.endpoint_for_box_exact(database.make_box_key(*left_box), key);
    assert(by_box.has_value());
    assert(by_box->key.node_id == children.first);

    assert(database.checkpoint());
    std::string reason;
    auto reopened = ld::LectDatabase::open_existing(dir, true, &reason);
    assert(reopened.has_value());
    assert(reopened->evidence(parent_key).has_value());
    assert(reopened->verify(true).ok);
    std::filesystem::remove_all(dir);
}

void test_endpoint_payload_parent_hull_layout() {
    const auto dir = std::filesystem::temp_directory_path() / "rbf_lect_database_endpoint_layout";
    {
        auto database = make_database(dir);
        const auto children = database.split_leaf(database.root_node());
        assert(children.first == 1 && children.second == 2);

        ld::EvidenceKey key;
        key.node_id = children.first;
        key.sector = ld::kPrimarySector;
        key.channel = ld::EvidenceChannel::Safe;
        key.endpoint_source = rbf::EndpointSource::IFK;
        key.payload_kind = ld::EvidencePayloadKind::EndpointEnvelope;

        ld::EvidenceRecord left;
        left.key = key;
        left.payload = {
            0.0f, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f,
            6.0f, 7.0f, 8.0f, 9.0f, 10.0f, 11.0f,
        };
        assert(database.put_evidence(left));

        ld::EvidenceRecord right;
        right.key = key;
        right.key.node_id = children.second;
        right.payload = {
            -1.0f, -2.0f, 1.5f, 2.5f, 6.0f, 7.5f,
            5.5f, 6.5f, -3.0f, 10.0f, 9.5f, 12.0f,
        };
        assert(database.put_evidence(right));

        auto parent_key = key;
        parent_key.node_id = database.root_node();
        const auto parent = database.evidence(parent_key);
        assert(parent.has_value());
        assert(parent->child_hull);
        assert((std::vector<float>(parent->payload.begin(), parent->payload.end()) == std::vector<float>{
            -1.0f, -2.0f, 1.5f, 3.0f, 6.0f, 7.5f,
            5.5f, 6.5f, -3.0f, 10.0f, 10.0f, 12.0f,
        }));
        assert(database.verify(true).ok);
    }
    std::filesystem::remove_all(dir);
}

void test_compact_rewrites_evidence_store() {
    const auto dir = std::filesystem::temp_directory_path() / "rbf_lect_database_compact_rewrite";
    {
        auto database = make_database(dir);
        assert(database.ensure_depth(3));

        std::vector<ld::EvidenceKey> inserted_keys;
        inserted_keys.reserve(database.node_count());
        for (ld::NodeId node_id = 0; node_id < database.node_count(); ++node_id) {
            ld::EvidenceRecord record;
            record.key.node_id = node_id;
            record.key.sector = ld::kPrimarySector;
            record.key.channel = ld::EvidenceChannel::Safe;
            record.key.endpoint_source = rbf::EndpointSource::IFK;
            record.key.payload_kind = ld::EvidencePayloadKind::EndpointEnvelope;
            record.payload = {
                static_cast<float>(node_id),
                static_cast<float>(node_id) + 0.5f,
                static_cast<float>(node_id) + 1.0f,
                static_cast<float>(node_id) + 1.5f,
            };
            assert(database.put_evidence(record));
            inserted_keys.push_back(record.key);
        }
        assert(database.checkpoint());

        const auto evidence_path = dir / "evidence.pages";
        const auto size_before_delete = std::filesystem::file_size(evidence_path);
        for (ld::NodeId node_id = 0; node_id < database.node_count(); node_id += 4) {
            database.delete_node_payloads(node_id);
        }
        assert(database.checkpoint());
        const auto size_after_delete = std::filesystem::file_size(evidence_path);
        assert(size_after_delete > size_before_delete);

        assert(database.compact());
        const auto size_after_compact = std::filesystem::file_size(evidence_path);
        assert(size_after_compact < size_after_delete);

        std::string reason;
        auto reopened = ld::LectDatabase::open_existing(dir, true, &reason);
        assert(reopened.has_value());
        assert(reopened->verify(true).ok);
        for (ld::NodeId node_id = 0; node_id < reopened->node_count(); ++node_id) {
            const auto evidence = reopened->evidence(inserted_keys[static_cast<std::size_t>(node_id)]);
            if (node_id % 4 == 0) {
                assert(!evidence.has_value());
            } else {
                assert(evidence.has_value());
                assert(evidence->payload.size() == 4);
            }
        }
    }
    std::filesystem::remove_all(dir);
}

void test_lru_node_page_swap_and_reopen() {
    const auto dir = std::filesystem::temp_directory_path() / "rbf_lect_database_lru";
    auto config = config_for(dir);
    config.page_size_bytes = 160;
    config.max_resident_pages = 2;
    std::filesystem::remove_all(dir);
    ld::LectDatabase database;
    std::string reason;
    assert(database.open(config, &reason));
    assert(database.ensure_depth(4));
    assert(database.node_count() == 31);

    for (ld::NodeId id = 0; id < database.node_count(); ++id) {
        assert(database.node(id).has_value());
    }
    const auto& stats = database.stats();
    assert(stats.node_page_cache_misses > 0);
    assert(stats.node_page_evictions > 0);
    assert(stats.node_page_dirty_flushes > 0);
    assert(stats.resident_node_pages <= config.max_resident_pages);
    assert(stats.max_resident_node_pages <= config.max_resident_pages);
    assert(database.verify(true).ok);
    assert(database.checkpoint());

    auto reopened = ld::LectDatabase::open_existing(dir, true, &reason);
    assert(reopened.has_value());
    assert(reopened->node_count() == 31);
    assert(reopened->verify(true).ok);
    const auto node30_box = reopened->node_box(30);
    assert(node30_box.has_value());
    const auto lookup = reopened->box_to_node_exact(reopened->make_box_key(*node30_box));
    assert(lookup.found && lookup.node_id == 30);
    assert(reopened->stats().resident_node_pages <= config.max_resident_pages);
    std::filesystem::remove_all(dir);
}

void test_legacy_text_evidence_store_is_rejected() {
    const auto dir = std::filesystem::temp_directory_path() / "rbf_lect_database_legacy_evidence_rejected";
    ld::EvidenceKey key;
    {
        auto database = make_database(dir);
        const auto children = database.split_leaf(database.root_node());
        assert(children.first == 1 && children.second == 2);
        assert(database.checkpoint());

        key.node_id = children.first;
        key.sector = ld::kPrimarySector;
        key.channel = ld::EvidenceChannel::Safe;
        key.endpoint_source = rbf::EndpointSource::IFK;
        key.payload_kind = ld::EvidencePayloadKind::EndpointEnvelope;
    }

    ld::EvidenceRecord legacy_record;
    legacy_record.key = key;
    legacy_record.generation = 7;
    legacy_record.payload = {1.25f, -2.5f, 3.75f, 4.5f};
    legacy_record.checksum = payload_checksum(legacy_record.payload);

    {
        std::ofstream out(dir / "evidence.pages", std::ios::binary | std::ios::trunc);
        out << serialize_legacy_evidence_record(legacy_record) << '\n';
    }
    std::filesystem::remove(dir / "evidence.index");

    std::string reason;
    auto reopened = ld::LectDatabase::open_existing(dir, false, &reason);
    assert(!reopened.has_value());
    assert(reason == "evidence store format is unsupported; rebuild the database");
    std::filesystem::remove_all(dir);
}

void test_binary_evidence_index_sidecar_sorted_and_unsorted_fallback() {
    const auto dir = std::filesystem::temp_directory_path() / "rbf_lect_database_binary_sidecar_sorted";
    ld::EvidenceKey left_key;
    {
        auto database = make_database(dir);
        const auto children = database.split_leaf(database.root_node());
        assert(children.first == 1 && children.second == 2);

        left_key.node_id = children.first;
        left_key.sector = ld::kPrimarySector;
        left_key.channel = ld::EvidenceChannel::Safe;
        left_key.endpoint_source = rbf::EndpointSource::IFK;
        left_key.payload_kind = ld::EvidencePayloadKind::EndpointEnvelope;

        ld::EvidenceRecord left;
        left.key = left_key;
        left.payload = {0.0f, 1.0f, 2.0f, 3.0f};
        assert(database.put_evidence(left));

        ld::EvidenceRecord right;
        right.key = left_key;
        right.key.node_id = children.second;
        right.payload = {-1.0f, 2.0f, 1.0f, 4.0f};
        assert(database.put_evidence(right));
        assert(database.checkpoint());
    }

    BinaryEvidenceIndexSidecarHeader header;
    std::vector<BinaryEvidenceIndexSidecarEntry> entries;
    {
        std::ifstream input(dir / "evidence.index", std::ios::binary);
        input.read(reinterpret_cast<char*>(&header), static_cast<std::streamsize>(sizeof(header)));
        assert(static_cast<bool>(input));
        assert(header.magic == kBinaryEvidenceIndexSidecarMagic);
        assert(header.header_size == sizeof(BinaryEvidenceIndexSidecarHeader));
        entries.resize(static_cast<std::size_t>(header.entry_count));
        if (!entries.empty()) {
            input.read(reinterpret_cast<char*>(entries.data()),
                       static_cast<std::streamsize>(entries.size() * sizeof(BinaryEvidenceIndexSidecarEntry)));
            assert(static_cast<bool>(input));
        }
    }
    assert(header.path_blob_offset == sizeof(BinaryEvidenceIndexSidecarHeader) +
           entries.size() * sizeof(BinaryEvidenceIndexSidecarEntry));
    assert(entries.size() >= 2);
    for (std::size_t index = 1; index < entries.size(); ++index) {
        assert(entries[index - 1].offset <= entries[index].offset);
    }

    std::swap(entries.front(), entries.back());
    {
        std::ofstream out(dir / "evidence.index", std::ios::binary | std::ios::trunc);
        out.write(reinterpret_cast<const char*>(&header), static_cast<std::streamsize>(sizeof(header)));
        out.write(reinterpret_cast<const char*>(entries.data()),
                  static_cast<std::streamsize>(entries.size() * sizeof(BinaryEvidenceIndexSidecarEntry)));
        assert(static_cast<bool>(out));
    }

    std::string reason;
    auto reopened = ld::LectDatabase::open_existing(dir, true, &reason);
    assert(reopened.has_value());
    const auto left = reopened->evidence(left_key);
    assert(left.has_value());
    assert(left->payload.size() == 4);
    assert(reopened->verify(true).ok);
    std::filesystem::remove_all(dir);
}

void test_binary_evidence_index_sidecar_pathcode_recovers_from_bad_node_id_hint() {
    const auto dir = std::filesystem::temp_directory_path() / "rbf_lect_database_binary_sidecar_bad_hint";
    ld::EvidenceKey left_key;
    {
        auto database = make_database(dir);
        const auto children = database.split_leaf(database.root_node());
        assert(children.first == 1 && children.second == 2);

        left_key.node_id = children.first;
        left_key.sector = ld::kPrimarySector;
        left_key.channel = ld::EvidenceChannel::Safe;
        left_key.endpoint_source = rbf::EndpointSource::IFK;
        left_key.payload_kind = ld::EvidencePayloadKind::EndpointEnvelope;

        ld::EvidenceRecord left;
        left.key = left_key;
        left.payload = {4.0f, 3.0f, 2.0f, 1.0f};
        assert(database.put_evidence(left));
        assert(database.checkpoint());
    }

    BinaryEvidenceIndexSidecarHeader header;
    std::vector<BinaryEvidenceIndexSidecarEntry> entries;
    std::vector<char> path_blob;
    {
        const auto sidecar_path = dir / "evidence.index";
        std::ifstream input(sidecar_path, std::ios::binary);
        input.read(reinterpret_cast<char*>(&header), static_cast<std::streamsize>(sizeof(header)));
        assert(static_cast<bool>(input));
        entries.resize(static_cast<std::size_t>(header.entry_count));
        if (!entries.empty()) {
            input.read(reinterpret_cast<char*>(entries.data()),
                       static_cast<std::streamsize>(entries.size() * sizeof(BinaryEvidenceIndexSidecarEntry)));
            assert(static_cast<bool>(input));
        }
        const auto sidecar_size = std::filesystem::file_size(sidecar_path);
        const auto path_blob_size = static_cast<std::size_t>(sidecar_size - header.path_blob_offset);
        path_blob.resize(path_blob_size);
        if (!path_blob.empty()) {
            input.read(path_blob.data(), static_cast<std::streamsize>(path_blob.size()));
            assert(static_cast<bool>(input));
        }
    }
    assert(entries.size() == 1);
    assert(entries.front().node_id == left_key.node_id);
    entries.front().node_id += 99;

    {
        std::ofstream out(dir / "evidence.pages", std::ios::binary | std::ios::app);
        const char garbage[] = {'x', 'v', '3', '!'};
        out.write(garbage, static_cast<std::streamsize>(sizeof(garbage)));
        assert(static_cast<bool>(out));
    }
    header.evidence_file_size = std::filesystem::file_size(dir / "evidence.pages");

    {
        std::ofstream out(dir / "evidence.index", std::ios::binary | std::ios::trunc);
        out.write(reinterpret_cast<const char*>(&header), static_cast<std::streamsize>(sizeof(header)));
        out.write(reinterpret_cast<const char*>(entries.data()),
                  static_cast<std::streamsize>(entries.size() * sizeof(BinaryEvidenceIndexSidecarEntry)));
        if (!path_blob.empty()) {
            out.write(path_blob.data(), static_cast<std::streamsize>(path_blob.size()));
        }
        assert(static_cast<bool>(out));
    }

    std::string reason;
    auto reopened = ld::LectDatabase::open_existing(dir, true, &reason);
    assert(reopened.has_value());
    const auto left = reopened->evidence(left_key);
    assert(left.has_value());
    assert(left->key.node_id == left_key.node_id);
    assert(left->payload.size() == 4);
    assert(left->payload[0] == 4.0f);
    assert(left->payload[3] == 1.0f);
    std::filesystem::remove_all(dir);
}

}  // namespace

int main() {
    test_identity_rejection();
    test_max_tree_depth_limit();
    test_topology_box_and_range_queries();
    test_depth_synchronous_split_dimensions();
    test_split_to_box_and_persistence();
    test_path_keys_are_split_order_independent();
    test_journal_replay_truncated_tail();
    test_evidence_journal_replay_without_checkpoint();
    test_evidence_parent_hull_and_exact_box_lookup();
    test_endpoint_payload_parent_hull_layout();
    test_compact_rewrites_evidence_store();
    test_lru_node_page_swap_and_reopen();
    test_legacy_text_evidence_store_is_rejected();
    test_binary_evidence_index_sidecar_sorted_and_unsorted_fallback();
    test_binary_evidence_index_sidecar_pathcode_recovers_from_bad_node_id_hint();
    return 0;
}
