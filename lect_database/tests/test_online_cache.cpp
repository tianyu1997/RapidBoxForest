#include <LECTDatabase/online_cache.h>

#ifdef NDEBUG
#undef NDEBUG
#endif

#include <cassert>
#include <filesystem>
#include <vector>

namespace ld = rbf::lect_database;

namespace {

std::vector<rbf::Interval> root2() {
    return {{0.0, 2.0}, {0.0, 2.0}};
}

ld::SplitPolicyDescriptor split_policy() {
    ld::SplitPolicyDescriptor descriptor;
    descriptor.strategy = ld::SplitStrategy::RoundRobin;
    descriptor.midpoint = true;
    return descriptor;
}

ld::LectDatabaseConfig config_for(const std::filesystem::path& dir) {
    ld::LectDatabaseConfig config;
    config.path = dir;
    config.root_intervals = root2();
    config.split_policy = split_policy();
    config.identity.robot_fingerprint = 7;
    config.identity.root_domain_fingerprint = ld::fingerprint_intervals(config.root_intervals);
    config.identity.split_policy_hash = ld::split_policy_hash(config.split_policy);
    config.identity.split_policy_descriptor = ld::split_policy_descriptor(config.split_policy);
    config.identity.endpoint_descriptor = "online_cache_test_endpoint";
    config.identity.envelope_descriptor = "online_cache_test_envelope";
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

ld::EvidenceRecord record_for(ld::NodeId node_id, float seed) {
    ld::EvidenceRecord record;
    record.key.node_id = node_id;
    record.key.channel = ld::EvidenceChannel::Safe;
    record.key.endpoint_source = rbf::EndpointSource::IFK;
    record.key.payload_kind = ld::EvidencePayloadKind::EndpointEnvelope;
    record.payload = {seed, seed + 1.0f, seed + 2.0f, seed + 3.0f};
    return record;
}

void test_split_uses_database_topology() {
    const auto dir = std::filesystem::temp_directory_path() / "rbf_online_cache_split";
    auto database = make_database(dir);
    ld::OnlineEnvelopeCacheTree cache(database);
    const auto children = cache.split_leaf(cache.root_node());
    assert(children.first == 1);
    assert(children.second == 2);
    assert(!cache.is_leaf(cache.root_node()));
    assert(cache.stats().split_requests == 1);
    assert(cache.stats().parent_union_pending == 1);
    std::filesystem::remove_all(dir);
}

void test_payload_cache_and_backfill() {
    const auto dir = std::filesystem::temp_directory_path() / "rbf_online_cache_payload";
    auto database = make_database(dir);
    ld::OnlineEnvelopeCacheConfig config;
    config.max_payload_bytes = 24;
    ld::OnlineEnvelopeCacheTree cache(database, config);

    auto first = record_for(database.root_node(), 1.0f);
    const auto first_key = first.key;
    assert(cache.put_evidence(first));
    assert(cache.has_cached_payload(first_key));
    assert(database.has_evidence(first_key));

    auto second = record_for(database.root_node(), 10.0f);
    second.key.sector = 1;
    const auto second_key = second.key;
    assert(cache.put_evidence(second, false));
    assert(!cache.has_cached_payload(first_key));
    assert(cache.has_cached_payload(second_key));
    assert(cache.stats().lru_evictions == 1);

    auto reloaded = cache.evidence(first_key);
    assert(reloaded.has_value());
    assert(cache.has_cached_payload(first_key));
    assert(cache.stats().database_loads == 1);
    std::filesystem::remove_all(dir);
}

}  // namespace

int main() {
    test_split_uses_database_topology();
    test_payload_cache_and_backfill();
    return 0;
}
