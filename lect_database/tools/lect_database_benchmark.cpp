#include <rbf/lect_database.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;
namespace ld = rbf::lect_database;

volatile float g_payload_probe_sink = 0.0f;

struct Options {
    std::filesystem::path db_path = "outputs/lect_database_benchmark/db";
    std::filesystem::path csv_path;
    int dims = 6;
    int depth = 8;
    std::uint64_t queries = 20000;
    std::uint64_t evidence_records = 20000;
    std::uint64_t compact_delete_ops = 0;
    int payload_floats = 84;
    std::uint32_t page_size_bytes = 4096;
    std::uint32_t max_resident_pages = 4;
    std::uint32_t seed = 42;
    bool fresh = true;
    bool existing_db = false;
    bool verify = true;
    bool read_stages_only = false;
    bool snapshot = false;
    bool reuse_snapshot = false;
    std::filesystem::path snapshot_path;
};

struct StageRow {
    std::string stage;
    std::uint64_t operations = 0;
    double elapsed_ms = 0.0;
    std::size_t nodes = 0;
    std::size_t evidence = 0;
    ld::LectDatabaseStats delta;
    bool ok = true;
};

struct BenchmarkFixture {
    std::uint64_t node_count = 0;
    std::vector<ld::NodeId> node_ids;
    std::vector<ld::NodeId> random_nodes;
    std::vector<std::vector<rbf::Interval>> sampled_boxes;
    std::vector<std::vector<rbf::Interval>> query_boxes;
    std::vector<ld::EvidenceRecord> evidence_samples;
};

ld::LectDatabaseStats diff_stats(const ld::LectDatabaseStats& after, const ld::LectDatabaseStats& before) {
    ld::LectDatabaseStats delta;
    delta.node_page_reads = after.node_page_reads - before.node_page_reads;
    delta.node_page_writes = after.node_page_writes - before.node_page_writes;
    delta.node_page_cache_hits = after.node_page_cache_hits - before.node_page_cache_hits;
    delta.node_page_cache_misses = after.node_page_cache_misses - before.node_page_cache_misses;
    delta.node_page_evictions = after.node_page_evictions - before.node_page_evictions;
    delta.node_page_dirty_evictions = after.node_page_dirty_evictions - before.node_page_dirty_evictions;
    delta.node_page_dirty_flushes = after.node_page_dirty_flushes - before.node_page_dirty_flushes;
    delta.evidence_reads = after.evidence_reads - before.evidence_reads;
    delta.evidence_writes = after.evidence_writes - before.evidence_writes;
    delta.journal_transactions = after.journal_transactions - before.journal_transactions;
    delta.range_nodes_visited = after.range_nodes_visited - before.range_nodes_visited;
    delta.query_path_cache_hits = after.query_path_cache_hits - before.query_path_cache_hits;
    delta.query_path_cache_misses = after.query_path_cache_misses - before.query_path_cache_misses;
    delta.resident_node_pages = after.resident_node_pages;
    delta.max_resident_node_pages = after.max_resident_node_pages;
    return delta;
}

void usage() {
    std::cout
        << "lect_database_benchmark [options]\n"
        << "  --db PATH                 database directory (default outputs/lect_database_benchmark/db)\n"
        << "  --csv PATH                optional CSV output\n"
        << "  --dims N                  synthetic root dimensionality (default 6)\n"
        << "  --depth D                 ensured kd-tree depth (default 8)\n"
        << "  --queries N               read/query operations per read stage (default 20000)\n"
        << "  --evidence-records N      evidence insert/update/read operations (default 20000)\n"
        << "  --compact-delete-ops N    optional sampled node payload tombstones before compact (default node_count/4)\n"
        << "  --payload-floats N        floats per endpoint payload (default 84)\n"
        << "  --page-size N             node page size in bytes (default 4096)\n"
        << "  --resident-pages N        LRU resident node page cap (default 4)\n"
        << "  --seed N                  random seed (default 42)\n"
        << "  --existing-db             benchmark an already-persisted database instead of building synthetic data\n"
        << "  --snapshot               build and benchmark a read snapshot from the database\n"
        << "  --snapshot-path PATH      snapshot directory (default DB/lect_snapshot)\n"
        << "  --reuse-snapshot         reuse an existing snapshot if it opens cleanly\n"
        << "  --reuse                   do not delete an existing database first\n"
        << "  --read-stages-only        run only open/query/evidence read stages\n"
        << "  --no-verify               skip final strict verify\n"
        << "  --help                    print this help\n";
}

Options parse_args(int argc, char** argv) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto next = [&]() -> std::string {
            if (i + 1 >= argc) {
                throw std::invalid_argument("missing value for " + arg);
            }
            return argv[++i];
        };
        if (arg == "--db") options.db_path = next();
        else if (arg == "--csv") options.csv_path = next();
        else if (arg == "--dims") options.dims = std::stoi(next());
        else if (arg == "--depth") options.depth = std::stoi(next());
        else if (arg == "--queries") options.queries = static_cast<std::uint64_t>(std::stoull(next()));
        else if (arg == "--evidence-records") options.evidence_records = static_cast<std::uint64_t>(std::stoull(next()));
        else if (arg == "--compact-delete-ops") options.compact_delete_ops = static_cast<std::uint64_t>(std::stoull(next()));
        else if (arg == "--payload-floats") options.payload_floats = std::stoi(next());
        else if (arg == "--page-size") options.page_size_bytes = static_cast<std::uint32_t>(std::stoul(next()));
        else if (arg == "--resident-pages") options.max_resident_pages = static_cast<std::uint32_t>(std::stoul(next()));
        else if (arg == "--seed") options.seed = static_cast<std::uint32_t>(std::stoul(next()));
        else if (arg == "--existing-db") options.existing_db = true;
        else if (arg == "--snapshot") options.snapshot = true;
        else if (arg == "--snapshot-path") options.snapshot_path = next();
        else if (arg == "--reuse-snapshot") options.reuse_snapshot = true;
        else if (arg == "--reuse") options.fresh = false;
        else if (arg == "--read-stages-only") options.read_stages_only = true;
        else if (arg == "--no-verify") options.verify = false;
        else if (arg == "--help") {
            usage();
            std::exit(0);
        } else {
            throw std::invalid_argument("unknown argument: " + arg);
        }
    }
    options.dims = std::max(1, options.dims);
    options.depth = std::max(0, options.depth);
    options.queries = std::max<std::uint64_t>(1, options.queries);
    options.evidence_records = std::max<std::uint64_t>(1, options.evidence_records);
    options.payload_floats = std::max(1, options.payload_floats);
    options.page_size_bytes = std::max<std::uint32_t>(128, options.page_size_bytes);
    options.max_resident_pages = std::max<std::uint32_t>(1, options.max_resident_pages);
    if (options.snapshot && options.snapshot_path.empty()) {
        options.snapshot_path = ld::LectReadSnapshot::default_snapshot_path(options.db_path);
    }
    return options;
}

std::vector<rbf::Interval> root_intervals(int dims) {
    std::vector<rbf::Interval> root;
    root.reserve(static_cast<std::size_t>(dims));
    for (int dim = 0; dim < dims; ++dim) {
        root.push_back({0.0, 1.0});
    }
    return root;
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
    identity.robot_fingerprint = 0xBEEFBEEFull;
    identity.root_domain_fingerprint = ld::fingerprint_intervals(root);
    identity.split_policy_hash = ld::split_policy_hash(split);
    identity.split_policy_descriptor = ld::split_policy_descriptor(split);
    identity.endpoint_descriptor = "benchmark_endpoint";
    identity.envelope_descriptor = "benchmark_envelope";
    identity.payload_layout = "float_payload";
    identity.builder_version = "lect_database_benchmark_snapshot";
    return identity;
}

ld::LectDatabaseConfig make_config(const Options& options) {
    ld::LectDatabaseConfig config;
    config.path = options.db_path;
    config.root_intervals = root_intervals(options.dims);
    config.split_policy = split_policy();
    config.identity = identity_for(config.root_intervals, config.split_policy);
    config.page_size_bytes = options.page_size_bytes;
    config.max_resident_pages = options.max_resident_pages;
    return config;
}

std::vector<float> make_payload(std::uint64_t index, int payload_floats) {
    std::vector<float> payload(static_cast<std::size_t>(payload_floats));
    for (int i = 0; i < payload_floats; ++i) {
        payload[static_cast<std::size_t>(i)] = static_cast<float>((index + static_cast<std::uint64_t>(i)) % 997u) / 997.0f;
    }
    return payload;
}

ld::EvidenceKey evidence_key(std::uint64_t index, ld::NodeId node_id, std::uint64_t node_count) {
    ld::EvidenceKey key;
    key.node_id = node_id;
    key.sector = static_cast<int>(index / std::max<std::uint64_t>(1, node_count));
    key.channel = ld::EvidenceChannel::Safe;
    key.endpoint_source = rbf::EndpointSource::IFK;
    key.payload_kind = ld::EvidencePayloadKind::EndpointEnvelope;
    return key;
}

std::vector<ld::EvidenceKey> fixture_key_templates() {
    std::vector<ld::EvidenceKey> key_templates;
    key_templates.reserve(24);
    for (auto channel : {ld::EvidenceChannel::Safe, ld::EvidenceChannel::Rapid}) {
        for (auto endpoint_source : {rbf::EndpointSource::IFK,
                                     rbf::EndpointSource::CritSample,
                                     rbf::EndpointSource::Analytical,
                                     rbf::EndpointSource::GCPC,
                                     rbf::EndpointSource::MC,
                                     rbf::EndpointSource::HIFK}) {
            for (auto payload_kind : {ld::EvidencePayloadKind::EndpointEnvelope,
                                      ld::EvidencePayloadKind::LinkEnvelope}) {
                ld::EvidenceKey key;
                key.channel = channel;
                key.endpoint_source = endpoint_source;
                key.payload_kind = payload_kind;
                key_templates.push_back(key);
            }
        }
    }
    return key_templates;
}

std::vector<rbf::Interval> random_query_box(int dims, std::mt19937& rng) {
    std::uniform_real_distribution<double> lo_dist(0.0, 0.9);
    std::uniform_real_distribution<double> width_dist(0.02, 0.1);
    std::vector<rbf::Interval> box;
    box.reserve(static_cast<std::size_t>(dims));
    for (int dim = 0; dim < dims; ++dim) {
        const double lo = lo_dist(rng);
        const double hi = std::min(1.0, lo + width_dist(rng));
        box.push_back({lo, hi});
    }
    return box;
}

std::vector<rbf::Interval> expanded_query_box(const std::vector<rbf::Interval>& box) {
    std::vector<rbf::Interval> query = box;
    for (auto& interval : query) {
        const double width = std::max(1e-9, interval.hi - interval.lo);
        const double pad = std::max(1e-6, width * 0.25);
        interval.lo -= pad;
        interval.hi += pad;
    }
    return query;
}

void perturb_payload(ld::EvidenceRecord* record, std::uint64_t ordinal) {
    if (record == nullptr) {
        return;
    }
    record->generation += 1;
    record->checksum = 0;
    record->unavailable = false;
    if (record->payload.empty()) {
        record->payload.push_back(0.0f);
    }
    const float delta = 1.0e-6f * static_cast<float>((ordinal % 17u) + 1u);
    record->payload.front() = std::nextafter(record->payload.front() + delta,
                                             std::numeric_limits<float>::infinity());
}

ld::EvidenceRecord materialize_record(const ld::EvidenceRecordView& view) {
    ld::EvidenceRecord record;
    record.key = view.key;
    record.child_hull = view.child_hull;
    record.unavailable = view.unavailable;
    record.generation = view.generation;
    record.checksum = view.checksum;
    record.payload.assign(view.payload.begin(), view.payload.end());
    return record;
}

float payload_probe_sum(const ld::EvidenceRecordView& view) {
    if (view.payload.empty()) {
        return 0.0f;
    }
    const auto mid = view.payload.size() / 2u;
    return view.payload.front() + view.payload[mid] + view.payload.back();
}

std::size_t hot_working_set_size(std::size_t size, std::size_t preferred = 2u) {
    return std::max<std::size_t>(1u, std::min(size, preferred));
}

template <typename Fn>
StageRow measure(const std::string& stage,
                 std::uint64_t operations,
                 ld::LectDatabase& database,
                 Fn&& fn) {
    const auto before = database.stats();
    const auto begin = Clock::now();
    const bool ok = fn();
    const auto elapsed = std::chrono::duration<double, std::milli>(Clock::now() - begin).count();
    StageRow row;
    row.stage = stage;
    row.operations = operations;
    row.elapsed_ms = elapsed;
    row.nodes = database.node_count();
    row.evidence = database.evidence_count();
    row.delta = diff_stats(database.stats(), before);
    row.ok = ok;
    return row;
}

double avg_us_per_op(const StageRow& row) {
    return row.operations == 0 ? 0.0 : (row.elapsed_ms * 1000.0 / static_cast<double>(row.operations));
}

std::filesystem::path sibling_stage_path(const std::filesystem::path& root, const std::string& suffix) {
    const auto name = root.filename().string();
    return root.parent_path() / (name + "_" + suffix);
}

bool copy_database_tree(const std::filesystem::path& source, const std::filesystem::path& target) {
    std::error_code error;
    std::filesystem::remove_all(target, error);
    error.clear();
    std::filesystem::create_directories(target.parent_path(), error);
    if (error) {
        return false;
    }
    std::filesystem::copy(source, target, std::filesystem::copy_options::recursive, error);
    return !error;
}

StageRow failed_stage(const std::string& stage, std::uint64_t operations) {
    StageRow row;
    row.stage = stage;
    row.operations = operations;
    row.ok = false;
    return row;
}

StageRow measure_open_existing(const std::string& stage,
                               const std::filesystem::path& path,
                               bool read_only,
                               bool verify_after_open) {
    StageRow row;
    row.stage = stage;
    row.operations = 1;
    std::string reason;
    const auto begin = Clock::now();
    auto database = ld::LectDatabase::open_existing(path, read_only, &reason);
    row.elapsed_ms = std::chrono::duration<double, std::milli>(Clock::now() - begin).count();
    row.ok = database.has_value() && (!verify_after_open || database->verify(true).ok);
    if (database) {
        row.nodes = database->node_count();
        row.evidence = database->evidence_count();
        row.delta = database->stats();
    }
    return row;
}

StageRow measure_snapshot_build(const std::filesystem::path& legacy_path,
                          const std::filesystem::path& snapshot_path) {
    StageRow row;
    row.stage = "snapshot.build";
    row.operations = 1;
    std::string reason;
    const auto begin = Clock::now();
    row.ok = ld::LectReadSnapshot::build_from_legacy(legacy_path, snapshot_path, &reason);
    row.elapsed_ms = std::chrono::duration<double, std::milli>(Clock::now() - begin).count();
    if (row.ok) {
        ld::LectReadSnapshot snapshot;
        row.ok = snapshot.open(snapshot_path, &reason);
        if (row.ok) {
            row.nodes = snapshot.node_count();
            row.evidence = snapshot.evidence_count();
        }
    }
    if (!row.ok && !reason.empty()) {
        std::cerr << "snapshot build failed: " << reason << '\n';
    }
    return row;
}

StageRow measure_snapshot_open(const std::string& stage,
                         const std::filesystem::path& snapshot_path) {
    StageRow row;
    row.stage = stage;
    row.operations = 1;
    std::string reason;
    ld::LectReadSnapshot snapshot;
    const auto begin = Clock::now();
    row.ok = snapshot.open(snapshot_path, &reason);
    row.elapsed_ms = std::chrono::duration<double, std::milli>(Clock::now() - begin).count();
    if (row.ok) {
        row.nodes = snapshot.node_count();
        row.evidence = snapshot.evidence_count();
    } else if (!reason.empty()) {
        std::cerr << "snapshot open failed: " << reason << '\n';
    }
    return row;
}

void prepare_snapshot_rows(const Options& options, std::vector<StageRow>* rows) {
    if (rows == nullptr) {
        return;
    }
    if (options.reuse_snapshot && std::filesystem::exists(options.snapshot_path / "manifest.bin")) {
        rows->push_back(measure_snapshot_open("snapshot.load.open_read_only", options.snapshot_path));
        if (rows->back().ok) {
            return;
        }
        rows->pop_back();
    }
    rows->push_back(measure_snapshot_build(options.db_path, options.snapshot_path));
    rows->push_back(measure_snapshot_open("snapshot.load.open_read_only", options.snapshot_path));
}

template <typename Fn>
StageRow measure_reopened_snapshot(const std::string& stage,
                                   std::uint64_t operations,
                                   const std::filesystem::path& snapshot_path,
                                   Fn&& fn) {
    std::string reason;
    ld::LectReadSnapshot snapshot;
    if (!snapshot.open(snapshot_path, &reason)) {
        if (!reason.empty()) {
            std::cerr << "snapshot open failed for " << stage << ": " << reason << '\n';
        }
        return failed_stage(stage, operations);
    }
    StageRow row;
    row.stage = stage;
    row.operations = operations;
    row.nodes = snapshot.node_count();
    row.evidence = snapshot.evidence_count();
    const auto begin = Clock::now();
    row.ok = fn(snapshot, row.delta);
    row.elapsed_ms = std::chrono::duration<double, std::milli>(Clock::now() - begin).count();
    return row;
}

template <typename Fn>
StageRow measure_reopened_db(const std::string& stage,
                             std::uint64_t operations,
                             const std::filesystem::path& path,
                             bool read_only,
                             Fn&& fn) {
    std::string reason;
    auto database = ld::LectDatabase::open_existing(path, read_only, &reason);
    if (!database) {
        return failed_stage(stage, operations);
    }
    return measure(stage, operations, *database, [&]() { return fn(*database); });
}

template <typename Fn>
StageRow measure_reopened_db_session(const std::string& stage,
                                     std::uint64_t operations,
                                     const std::filesystem::path& path,
                                     bool read_only,
                                     Fn&& fn) {
    std::string reason;
    auto database = ld::LectDatabase::open_existing(path, read_only, &reason);
    if (!database) {
        return failed_stage(stage, operations);
    }

    auto session = database->make_query_session();
    const auto before = database->stats();
    const auto begin = Clock::now();
    const bool ok = fn(*database, session);
    const auto elapsed = std::chrono::duration<double, std::milli>(Clock::now() - begin).count();

    StageRow row;
    row.stage = stage;
    row.operations = operations;
    row.elapsed_ms = elapsed;
    row.nodes = database->node_count();
    row.evidence = database->evidence_count();
    row.delta = diff_stats(database->stats(), before);
    row.delta.query_path_cache_hits = session.stats().query_path_cache_hits;
    row.delta.query_path_cache_misses = session.stats().query_path_cache_misses;
    row.ok = ok;
    return row;
}

bool build_baseline_database(const Options& options,
                            BenchmarkFixture* fixture,
                            std::string* reason) {
    ld::LectDatabase database;
    const auto config = make_config(options);
    if (!database.open(config, reason)) {
        return false;
    }
    if (!database.ensure_depth(options.depth)) {
        if (reason) *reason = "ensure_depth failed";
        return false;
    }

    fixture->node_count = static_cast<std::uint64_t>(database.node_count());
    fixture->node_ids = database.node_ids();
    std::mt19937 rng(options.seed);
    std::uniform_int_distribution<std::uint64_t> node_dist(0, fixture->node_count == 0 ? 0 : fixture->node_count - 1);

    fixture->random_nodes.clear();
    fixture->random_nodes.reserve(static_cast<std::size_t>(options.queries));
    for (std::uint64_t i = 0; i < options.queries; ++i) {
        fixture->random_nodes.push_back(static_cast<ld::NodeId>(node_dist(rng)));
    }

    fixture->sampled_boxes.clear();
    fixture->sampled_boxes.reserve(static_cast<std::size_t>(std::min<std::uint64_t>(options.queries, 4096)));
    for (std::uint64_t i = 0; i < options.queries && fixture->sampled_boxes.size() < 4096; ++i) {
        auto box = database.node_box(fixture->random_nodes[static_cast<std::size_t>(i)]);
        if (!box) {
            if (reason) *reason = "node_box failed during baseline setup";
            return false;
        }
        fixture->sampled_boxes.push_back(std::move(*box));
    }

    fixture->query_boxes.clear();
    fixture->query_boxes.reserve(static_cast<std::size_t>(options.queries));
    for (std::uint64_t i = 0; i < options.queries; ++i) {
        fixture->query_boxes.push_back(random_query_box(options.dims, rng));
    }

    fixture->evidence_samples.clear();
    fixture->evidence_samples.reserve(static_cast<std::size_t>(options.evidence_records));
    for (std::uint64_t i = 0; i < options.evidence_records; ++i) {
        const auto node_id = static_cast<ld::NodeId>(i % fixture->node_count);
        auto key = evidence_key(i, node_id, fixture->node_count);
        ld::EvidenceRecord record;
        record.key = key;
        record.payload = make_payload(i, options.payload_floats);
        if (!database.put_evidence(std::move(record))) {
            if (reason) *reason = "put_evidence failed during baseline setup";
            return false;
        }
        ld::EvidenceRecord sample;
        sample.key = key;
        sample.payload = make_payload(i, options.payload_floats);
        fixture->evidence_samples.push_back(std::move(sample));
    }

    if (!database.checkpoint()) {
        if (reason) *reason = "checkpoint failed during baseline setup";
        return false;
    }
    return true;
}

bool load_snapshot_fixture(const Options& options,
                           const std::filesystem::path& snapshot_path,
                           BenchmarkFixture* fixture,
                           std::string* reason) {
    ld::LectReadSnapshot snapshot;
    if (!snapshot.open(snapshot_path, reason)) {
        return false;
    }
    fixture->node_count = static_cast<std::uint64_t>(snapshot.node_count());
    fixture->node_ids.clear();
    if (fixture->node_count == 0) {
        if (reason) *reason = "snapshot fixture has no nodes";
        return false;
    }

    std::mt19937 rng(options.seed);
    std::uniform_int_distribution<std::uint64_t> node_dist(0, fixture->node_count - 1u);
    fixture->random_nodes.clear();
    fixture->random_nodes.reserve(static_cast<std::size_t>(options.queries));
    fixture->sampled_boxes.clear();
    fixture->sampled_boxes.reserve(static_cast<std::size_t>(std::min<std::uint64_t>(options.queries, 4096)));

    auto try_add_node = [&](ld::NodeId node_id) {
        auto box = snapshot.node_box(node_id);
        if (!box) {
            return false;
        }
        fixture->random_nodes.push_back(node_id);
        if (fixture->sampled_boxes.size() < 4096) {
            fixture->sampled_boxes.push_back(std::move(*box));
        }
        return true;
    };

    const std::uint64_t max_random_attempts = std::max<std::uint64_t>(options.queries * 4u, 1024u);
    for (std::uint64_t attempt = 0; attempt < max_random_attempts && fixture->random_nodes.size() < options.queries; ++attempt) {
        try_add_node(static_cast<ld::NodeId>(node_dist(rng)));
    }
    for (std::uint64_t node_id = 0; node_id < fixture->node_count && fixture->random_nodes.size() < options.queries; ++node_id) {
        try_add_node(static_cast<ld::NodeId>(node_id));
    }
    if (fixture->random_nodes.empty() || fixture->sampled_boxes.empty()) {
        if (reason) *reason = "snapshot fixture could not sample node boxes";
        return false;
    }
    const auto random_seed_size = fixture->random_nodes.size();
    for (std::uint64_t i = static_cast<std::uint64_t>(random_seed_size); i < options.queries; ++i) {
        fixture->random_nodes.push_back(fixture->random_nodes[static_cast<std::size_t>(i % random_seed_size)]);
    }

    fixture->query_boxes.clear();
    fixture->query_boxes.reserve(static_cast<std::size_t>(options.queries));
    for (std::uint64_t i = 0; i < options.queries; ++i) {
        const auto& box = fixture->sampled_boxes[static_cast<std::size_t>(i % fixture->sampled_boxes.size())];
        fixture->query_boxes.push_back(expanded_query_box(box));
    }

    fixture->evidence_samples.clear();
    fixture->evidence_samples.reserve(static_cast<std::size_t>(options.evidence_records));
    const auto key_templates = fixture_key_templates();
    auto try_collect_record = [&](ld::NodeId node_id) {
        for (const auto& key_template : key_templates) {
            auto key = key_template;
            key.node_id = node_id;
            const auto view = snapshot.evidence(key);
            if (!view || view->unavailable || view->payload.empty()) {
                continue;
            }
            fixture->evidence_samples.push_back(materialize_record(*view));
            return true;
        }
        return false;
    };

    for (const auto node_id : fixture->random_nodes) {
        if (try_collect_record(node_id) && fixture->evidence_samples.size() >= static_cast<std::size_t>(options.evidence_records)) {
            break;
        }
    }
    for (std::uint64_t node_id = 0; node_id < fixture->node_count && fixture->evidence_samples.size() < static_cast<std::size_t>(options.evidence_records); ++node_id) {
        try_collect_record(static_cast<ld::NodeId>(node_id));
    }
    if (fixture->evidence_samples.empty()) {
        if (reason) *reason = "snapshot fixture could not load sampled evidence";
        return false;
    }
    const auto evidence_seed_size = fixture->evidence_samples.size();
    for (std::uint64_t i = static_cast<std::uint64_t>(evidence_seed_size); i < options.evidence_records; ++i) {
        fixture->evidence_samples.push_back(fixture->evidence_samples[static_cast<std::size_t>(i % evidence_seed_size)]);
    }
    return true;
}

bool load_existing_fixture(const Options& options,
                           BenchmarkFixture* fixture,
                           std::string* reason,
                           StageRow* open_stage = nullptr) {
    const auto begin = Clock::now();
    auto database = ld::LectDatabase::open_existing(options.db_path, true, reason);
    const auto elapsed = std::chrono::duration<double, std::milli>(Clock::now() - begin).count();
    if (open_stage != nullptr) {
        open_stage->stage = "load.open_read_only";
        open_stage->operations = 1;
        open_stage->elapsed_ms = elapsed;
        open_stage->ok = database.has_value();
        if (database) {
            open_stage->nodes = database->node_count();
            open_stage->evidence = database->evidence_count();
            open_stage->delta = database->stats();
        }
    }
    if (!database) {
        return false;
    }

    fixture->node_ids = database->node_ids();
    fixture->node_count = static_cast<std::uint64_t>(fixture->node_ids.size());
    if (fixture->node_ids.empty()) {
        if (reason) *reason = "existing database has no nodes";
        return false;
    }

    std::mt19937 rng(options.seed);
    std::uniform_int_distribution<std::size_t> node_dist(0, fixture->node_ids.size() - 1);
    fixture->random_nodes.clear();
    fixture->random_nodes.reserve(static_cast<std::size_t>(options.queries));
    for (std::uint64_t i = 0; i < options.queries; ++i) {
        fixture->random_nodes.push_back(fixture->node_ids[node_dist(rng)]);
    }

    fixture->sampled_boxes.clear();
    fixture->sampled_boxes.reserve(static_cast<std::size_t>(std::min<std::uint64_t>(options.queries, 4096)));
    for (std::uint64_t i = 0; i < options.queries && fixture->sampled_boxes.size() < 4096; ++i) {
        auto box = database->node_box(fixture->random_nodes[static_cast<std::size_t>(i)]);
        if (!box) {
            if (reason) *reason = "node_box failed while preparing existing-db fixture";
            return false;
        }
        fixture->sampled_boxes.push_back(std::move(*box));
    }
    if (fixture->sampled_boxes.empty()) {
        if (reason) *reason = "existing database fixture has no sampled boxes";
        return false;
    }

    fixture->query_boxes.clear();
    fixture->query_boxes.reserve(static_cast<std::size_t>(options.queries));
    for (std::uint64_t i = 0; i < options.queries; ++i) {
        const auto& box = fixture->sampled_boxes[static_cast<std::size_t>(i % fixture->sampled_boxes.size())];
        fixture->query_boxes.push_back(expanded_query_box(box));
    }

    fixture->evidence_samples.clear();
    fixture->evidence_samples.reserve(static_cast<std::size_t>(options.evidence_records));

    std::vector<ld::EvidenceKey> key_templates;
    key_templates.reserve(24);
    for (auto channel : {ld::EvidenceChannel::Safe, ld::EvidenceChannel::Rapid}) {
        for (auto endpoint_source : {rbf::EndpointSource::IFK,
                                     rbf::EndpointSource::CritSample,
                                     rbf::EndpointSource::Analytical,
                                     rbf::EndpointSource::GCPC,
                                     rbf::EndpointSource::MC,
                                     rbf::EndpointSource::HIFK}) {
            for (auto payload_kind : {ld::EvidencePayloadKind::EndpointEnvelope,
                                      ld::EvidencePayloadKind::LinkEnvelope}) {
                ld::EvidenceKey key;
                key.channel = channel;
                key.endpoint_source = endpoint_source;
                key.payload_kind = payload_kind;
                key_templates.push_back(key);
            }
        }
    }

    auto try_collect_record = [&](ld::NodeId node_id) {
        for (const auto& key_template : key_templates) {
            auto key = key_template;
            key.node_id = node_id;
            const auto view = database->evidence(key);
            if (!view || view->unavailable || view->payload.empty()) {
                continue;
            }
            fixture->evidence_samples.push_back(materialize_record(*view));
            return true;
        }
        return false;
    };

    for (const auto node_id : fixture->random_nodes) {
        if (try_collect_record(node_id) && fixture->evidence_samples.size() >= static_cast<std::size_t>(options.evidence_records)) {
            break;
        }
    }
    for (const auto node_id : fixture->node_ids) {
        if (fixture->evidence_samples.size() >= static_cast<std::size_t>(options.evidence_records)) {
            break;
        }
        try_collect_record(node_id);
    }

    if (fixture->evidence_samples.empty()) {
        const auto available_evidence = database->evidence_records();
        if (available_evidence.empty()) {
            if (reason) *reason = "existing database fixture has no readable evidence records";
            return false;
        }
        std::uniform_int_distribution<std::size_t> evidence_dist(0, available_evidence.size() - 1);
        for (std::uint64_t i = 0; i < options.evidence_records; ++i) {
            const auto& record = available_evidence[evidence_dist(rng)];
            if (record.unavailable || record.payload.empty()) {
                continue;
            }
            fixture->evidence_samples.push_back(record);
        }
    }
    if (fixture->evidence_samples.empty()) {
        if (reason) *reason = "existing database fixture could not load sampled evidence";
        return false;
    }
    return true;
}

void print_rows(const std::vector<StageRow>& rows) {
    std::cout << std::left << std::setw(28) << "stage"
              << std::right << std::setw(12) << "ops"
              << std::setw(12) << "ms"
              << std::setw(14) << "avg_us/op"
              << std::setw(14) << "ops/s"
              << std::setw(10) << "nodes"
              << std::setw(10) << "evidence"
              << std::setw(10) << "reads"
              << std::setw(10) << "writes"
              << std::setw(10) << "hits"
              << std::setw(10) << "misses"
              << std::setw(10) << "evict"
              << std::setw(10) << "dirty"
              << '\n';
    for (const auto& row : rows) {
        const double ops_per_sec = row.elapsed_ms > 0.0
            ? static_cast<double>(row.operations) * 1000.0 / row.elapsed_ms
            : 0.0;
        std::cout << std::left << std::setw(28) << row.stage
                  << std::right << std::setw(12) << row.operations
                  << std::setw(12) << std::fixed << std::setprecision(3) << row.elapsed_ms
                  << std::setw(14) << std::fixed << std::setprecision(3) << avg_us_per_op(row)
                  << std::setw(14) << std::fixed << std::setprecision(1) << ops_per_sec
                  << std::setw(10) << row.nodes
                  << std::setw(10) << row.evidence
                  << std::setw(10) << row.delta.node_page_reads
                  << std::setw(10) << row.delta.node_page_writes
                  << std::setw(10) << row.delta.node_page_cache_hits
                  << std::setw(10) << row.delta.node_page_cache_misses
                  << std::setw(10) << row.delta.node_page_evictions
                  << std::setw(10) << row.delta.node_page_dirty_flushes
                  << (row.ok ? "" : "  FAILED")
                  << '\n';
    }
}

bool write_csv(const std::filesystem::path& path, const std::vector<StageRow>& rows) {
    if (path.empty()) {
        return true;
    }
    const auto parent = path.parent_path();
    if (!parent.empty()) {
        std::filesystem::create_directories(parent);
    }
    std::ofstream out(path, std::ios::trunc);
    if (!out) {
        return false;
    }
    out << "stage,operations,elapsed_ms,avg_us_per_op,ops_per_sec,nodes,evidence,page_reads,page_writes,cache_hits,cache_misses,evictions,dirty_evictions,dirty_flushes,evidence_reads,evidence_writes,journal_transactions,range_nodes_visited,resident_pages,max_resident_pages,ok\n";
    for (const auto& row : rows) {
        const double ops_per_sec = row.elapsed_ms > 0.0
            ? static_cast<double>(row.operations) * 1000.0 / row.elapsed_ms
            : 0.0;
        out << row.stage << ','
            << row.operations << ','
            << std::setprecision(17) << row.elapsed_ms << ','
            << avg_us_per_op(row) << ','
            << ops_per_sec << ','
            << row.nodes << ','
            << row.evidence << ','
            << row.delta.node_page_reads << ','
            << row.delta.node_page_writes << ','
            << row.delta.node_page_cache_hits << ','
            << row.delta.node_page_cache_misses << ','
            << row.delta.node_page_evictions << ','
            << row.delta.node_page_dirty_evictions << ','
            << row.delta.node_page_dirty_flushes << ','
            << row.delta.evidence_reads << ','
            << row.delta.evidence_writes << ','
            << row.delta.journal_transactions << ','
            << row.delta.range_nodes_visited << ','
            << row.delta.resident_node_pages << ','
            << row.delta.max_resident_node_pages << ','
            << (row.ok ? 1 : 0) << '\n';
    }
    return static_cast<bool>(out);
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const Options options = parse_args(argc, argv);
        if (options.fresh && !options.existing_db) {
            std::filesystem::remove_all(options.db_path);
        }
        const auto db_parent = options.db_path.parent_path();
        if (!db_parent.empty()) {
            std::filesystem::create_directories(db_parent);
        }

        std::string reason;
        BenchmarkFixture fixture;
        StageRow initial_open_row;
        std::vector<StageRow> rows;
        if (options.existing_db) {
            if (options.snapshot) {
                prepare_snapshot_rows(options, &rows);
                const bool snapshot_ready = std::all_of(rows.begin(), rows.end(), [](const StageRow& row) { return row.ok; });
                if (!snapshot_ready) {
                    print_rows(rows);
                    if (!options.csv_path.empty()) {
                        write_csv(options.csv_path, rows);
                    }
                    return 1;
                }
                if (!load_snapshot_fixture(options, options.snapshot_path, &fixture, &reason)) {
                    std::cerr << "failed to load snapshot fixture: " << reason << '\n';
                    return 1;
                }
            } else {
                if (!load_existing_fixture(options, &fixture, &reason, &initial_open_row)) {
                    std::cerr << "failed to load existing database fixture: " << reason << '\n';
                    return 1;
                }
            }
        } else {
            if (!build_baseline_database(options, &fixture, &reason)) {
                std::cerr << "failed to build baseline database: " << reason << '\n';
                return 1;
            }
        }

        if (!options.snapshot) {
        if (options.existing_db) {
            rows.push_back(initial_open_row);
        } else {
            rows.push_back(measure_open_existing("load.open_read_only", options.db_path, true, false));
        }
        rows.push_back(measure_reopened_db("read.node_box_disk",
                                           options.queries,
                                           options.db_path,
                                           true,
                                           [&](ld::LectDatabase& database) {
                                               for (std::uint64_t i = 0; i < options.queries; ++i) {
                                                   auto box = database.node_box(fixture.random_nodes[static_cast<std::size_t>(i)]);
                                                   if (!box) {
                                                       return false;
                                                   }
                                               }
                                               return true;
                                           }));
        rows.push_back(measure_reopened_db_session("read.node_box_session",
                                                   options.queries,
                                                   options.db_path,
                                                   true,
                                                   [&](ld::LectDatabase&, ld::LectDbQuerySession& session) {
                                                       for (std::uint64_t i = 0; i < options.queries; ++i) {
                                                           auto box = session.node_box(fixture.random_nodes[static_cast<std::size_t>(i)]);
                                                           if (!box) {
                                                               return false;
                                                           }
                                                       }
                                                       return true;
                                                   }));
        rows.push_back(measure_reopened_db("read.exact_box_lookup_disk",
                                           options.queries,
                                           options.db_path,
                                           true,
                                           [&](ld::LectDatabase& database) {
                                               for (std::uint64_t i = 0; i < options.queries; ++i) {
                                                   const auto& box = fixture.sampled_boxes[static_cast<std::size_t>(i % fixture.sampled_boxes.size())];
                                                   const auto lookup = database.box_to_node_exact(database.make_box_key(box));
                                                   if (!lookup.found) {
                                                       return false;
                                                   }
                                               }
                                               return true;
                                           }));
        rows.push_back(measure_reopened_db_session("read.exact_box_lookup_session",
                                                   options.queries,
                                                   options.db_path,
                                                   true,
                                                   [&](ld::LectDatabase& database, ld::LectDbQuerySession& session) {
                                                       for (std::uint64_t i = 0; i < options.queries; ++i) {
                                                           const auto& box = fixture.sampled_boxes[static_cast<std::size_t>(i % fixture.sampled_boxes.size())];
                                                           const auto lookup = session.box_to_node_exact(database.make_box_key(box));
                                                           if (!lookup.found) {
                                                               return false;
                                                           }
                                                       }
                                                       return true;
                                                   }));
        rows.push_back(measure_reopened_db("read.endpoint_for_box_exact_disk",
                                           options.queries,
                                           options.db_path,
                                           true,
                                           [&](ld::LectDatabase& database) {
                                               for (std::uint64_t i = 0; i < options.queries; ++i) {
                                                   const auto index = static_cast<std::size_t>(i % fixture.sampled_boxes.size());
                                                   const auto& box = fixture.sampled_boxes[index];
                                                   auto key_template = fixture.evidence_samples[static_cast<std::size_t>(i % fixture.evidence_samples.size())].key;
                                                   const auto endpoint = database.endpoint_for_box_exact(database.make_box_key(box), key_template);
                                                   if (!endpoint) {
                                                       return false;
                                                   }
                                               }
                                               return true;
                                           }));
        rows.push_back(measure_reopened_db("read.range_query_disk",
                                           options.queries,
                                           options.db_path,
                                           true,
                                           [&](ld::LectDatabase& database) {
                                               ld::LectDatabaseStats range_stats;
                                               for (const auto& box : fixture.query_boxes) {
                                                   const auto ids = database.range_query(box,
                                                                                         ld::RangeQueryMode::Intersecting,
                                                                                         &range_stats);
                                                   if (ids.empty() && database.node_count() > 0) {
                                                       return false;
                                                   }
                                               }
                                               return range_stats.range_nodes_visited > 0;
                                           }));
        rows.push_back(measure_reopened_db("read.evidence_disk",
                                           static_cast<std::uint64_t>(fixture.evidence_samples.size()),
                                           options.db_path,
                                           true,
                                           [&](ld::LectDatabase& database) {
                                               for (const auto& record : fixture.evidence_samples) {
                                                   if (!database.evidence(record.key)) {
                                                       return false;
                                                   }
                                               }
                                               return true;
                                           }));
        }

        if (options.snapshot) {
            if (!options.existing_db) {
                prepare_snapshot_rows(options, &rows);
            }
            rows.push_back(measure_reopened_snapshot("snapshot.read.node_box",
                                               options.queries,
                                               options.snapshot_path,
                                               [&](const ld::LectReadSnapshot& snapshot, ld::LectDatabaseStats&) {
                                                   for (std::uint64_t i = 0; i < options.queries; ++i) {
                                                       auto box = snapshot.node_box(fixture.random_nodes[static_cast<std::size_t>(i)]);
                                                       if (!box) {
                                                           return false;
                                                       }
                                                   }
                                                   return true;
                                               }));
            rows.push_back(measure_reopened_snapshot("snapshot.read.exact_box_lookup",
                                               options.queries,
                                               options.snapshot_path,
                                               [&](const ld::LectReadSnapshot& snapshot, ld::LectDatabaseStats&) {
                                                   for (std::uint64_t i = 0; i < options.queries; ++i) {
                                                       const auto& box = fixture.sampled_boxes[static_cast<std::size_t>(i % fixture.sampled_boxes.size())];
                                                       const auto lookup = snapshot.box_to_node_exact(snapshot.make_box_key(box));
                                                       if (!lookup.found) {
                                                           return false;
                                                       }
                                                   }
                                                   return true;
                                               }));
            rows.push_back(measure_reopened_snapshot("snapshot.read.endpoint_for_box_exact",
                                               options.queries,
                                               options.snapshot_path,
                                               [&](const ld::LectReadSnapshot& snapshot, ld::LectDatabaseStats&) {
                                                   for (std::uint64_t i = 0; i < options.queries; ++i) {
                                                       const auto index = static_cast<std::size_t>(i % fixture.sampled_boxes.size());
                                                       const auto& box = fixture.sampled_boxes[index];
                                                       auto key_template = fixture.evidence_samples[static_cast<std::size_t>(i % fixture.evidence_samples.size())].key;
                                                       const auto endpoint = snapshot.endpoint_for_box_exact(snapshot.make_box_key(box), key_template);
                                                       if (!endpoint) {
                                                           return false;
                                                       }
                                                   }
                                                   return true;
                                               }));
            rows.push_back(measure_reopened_snapshot("snapshot.read.endpoint_for_box_exact_hot2",
                                               options.queries,
                                               options.snapshot_path,
                                               [&](const ld::LectReadSnapshot& snapshot, ld::LectDatabaseStats&) {
                                                   const auto hot = hot_working_set_size(std::min(fixture.sampled_boxes.size(),
                                                                                                 fixture.evidence_samples.size()));
                                                   for (std::uint64_t i = 0; i < options.queries; ++i) {
                                                       const auto index = static_cast<std::size_t>(i % hot);
                                                       const auto& box = fixture.sampled_boxes[index];
                                                       auto key_template = fixture.evidence_samples[index].key;
                                                       const auto endpoint = snapshot.endpoint_for_box_exact(snapshot.make_box_key(box), key_template);
                                                       if (!endpoint) {
                                                           return false;
                                                       }
                                                   }
                                                   return true;
                                               }));
            rows.push_back(measure_reopened_snapshot("snapshot.read.endpoint_for_box_exact_touch_payload",
                                               options.queries,
                                               options.snapshot_path,
                                               [&](const ld::LectReadSnapshot& snapshot, ld::LectDatabaseStats&) {
                                                   float payload_accum = 0.0f;
                                                   for (std::uint64_t i = 0; i < options.queries; ++i) {
                                                       const auto index = static_cast<std::size_t>(i % fixture.sampled_boxes.size());
                                                       const auto& box = fixture.sampled_boxes[index];
                                                       auto key_template = fixture.evidence_samples[static_cast<std::size_t>(i % fixture.evidence_samples.size())].key;
                                                       const auto endpoint = snapshot.endpoint_for_box_exact(snapshot.make_box_key(box), key_template);
                                                       if (!endpoint) {
                                                           return false;
                                                       }
                                                       payload_accum += payload_probe_sum(*endpoint);
                                                   }
                                                   g_payload_probe_sink = payload_accum;
                                                   return true;
                                               }));
            rows.push_back(measure_reopened_snapshot("snapshot.read.range_query",
                                               options.queries,
                                               options.snapshot_path,
                                               [&](const ld::LectReadSnapshot& snapshot, ld::LectDatabaseStats& delta) {
                                                   ld::LectDatabaseStats range_stats;
                                                   for (const auto& box : fixture.query_boxes) {
                                                       const auto ids = snapshot.range_query(box,
                                                                                             ld::RangeQueryMode::Intersecting,
                                                                                             &range_stats);
                                                       if (ids.empty() && snapshot.node_count() > 0) {
                                                           return false;
                                                       }
                                                   }
                                                   delta.range_nodes_visited = range_stats.range_nodes_visited;
                                                   return range_stats.range_nodes_visited > 0;
                                               }));
            rows.push_back(measure_reopened_snapshot("snapshot.read.evidence",
                                               static_cast<std::uint64_t>(fixture.evidence_samples.size()),
                                               options.snapshot_path,
                                               [&](const ld::LectReadSnapshot& snapshot, ld::LectDatabaseStats& delta) {
                                                   for (const auto& record : fixture.evidence_samples) {
                                                       if (!snapshot.evidence(record.key)) {
                                                           return false;
                                                       }
                                                   }
                                                   delta.evidence_reads = static_cast<std::uint64_t>(fixture.evidence_samples.size());
                                                   return true;
                                               }));
            rows.push_back(measure_reopened_snapshot("snapshot.read.evidence_hot2",
                                               options.queries,
                                               options.snapshot_path,
                                               [&](const ld::LectReadSnapshot& snapshot, ld::LectDatabaseStats& delta) {
                                                   const auto hot = hot_working_set_size(fixture.evidence_samples.size());
                                                   for (std::uint64_t i = 0; i < options.queries; ++i) {
                                                       const auto& record = fixture.evidence_samples[static_cast<std::size_t>(i % hot)];
                                                       if (!snapshot.evidence(record.key)) {
                                                           return false;
                                                       }
                                                   }
                                                   delta.evidence_reads = options.queries;
                                                   return true;
                                               }));
            rows.push_back(measure_reopened_snapshot("snapshot.read.evidence_touch_payload",
                                               static_cast<std::uint64_t>(fixture.evidence_samples.size()),
                                               options.snapshot_path,
                                               [&](const ld::LectReadSnapshot& snapshot, ld::LectDatabaseStats& delta) {
                                                   float payload_accum = 0.0f;
                                                   for (const auto& record : fixture.evidence_samples) {
                                                       const auto view = snapshot.evidence(record.key);
                                                       if (!view) {
                                                           return false;
                                                       }
                                                       payload_accum += payload_probe_sum(*view);
                                                   }
                                                   g_payload_probe_sink = payload_accum;
                                                   delta.evidence_reads = static_cast<std::uint64_t>(fixture.evidence_samples.size());
                                                   return true;
                                               }));
        }

        if (!options.snapshot && !options.read_stages_only) {
            const auto checkpoint_db = sibling_stage_path(options.db_path, "checkpoint_dirty");
            if (!copy_database_tree(options.db_path, checkpoint_db)) {
                rows.push_back(failed_stage("write.checkpoint_dirty", 1));
            } else {
                auto database = ld::LectDatabase::open_existing(checkpoint_db, false, &reason);
                if (!database) {
                    rows.push_back(failed_stage("write.checkpoint_dirty", 1));
                } else {
                    bool prepared = true;
                    for (std::uint64_t i = 0; i < fixture.evidence_samples.size(); ++i) {
                        auto record = fixture.evidence_samples[static_cast<std::size_t>(i)];
                        perturb_payload(&record, i + 13);
                        if (!database->put_evidence(std::move(record))) {
                            prepared = false;
                            break;
                        }
                    }
                    rows.push_back(prepared
                        ? measure("write.checkpoint_dirty", 1, *database, [&]() { return database->checkpoint(); })
                        : failed_stage("write.checkpoint_dirty", 1));
                }
            }

            const auto compact_db = sibling_stage_path(options.db_path, "compact_tombstones");
            const std::uint64_t delete_ops = options.compact_delete_ops > 0
                ? std::max<std::uint64_t>(1, std::min<std::uint64_t>(options.compact_delete_ops, fixture.node_ids.size()))
                : std::max<std::uint64_t>(1, fixture.node_count / 4);
            if (!copy_database_tree(options.db_path, compact_db)) {
                rows.push_back(failed_stage("write.compact_tombstones", 1));
            } else {
                bool prepared = false;
                {
                    auto database = ld::LectDatabase::open_existing(compact_db, false, &reason);
                    if (database) {
                        prepared = true;
                        for (std::uint64_t i = 0; i < delete_ops; ++i) {
                            const auto node_id = fixture.node_ids[static_cast<std::size_t>((i * 4) % fixture.node_ids.size())];
                            database->delete_node_payloads(node_id);
                        }
                        prepared = prepared && database->checkpoint();
                    }
                }
                rows.push_back(prepared
                    ? measure_reopened_db("write.compact_tombstones",
                                          1,
                                          compact_db,
                                          false,
                                          [&](ld::LectDatabase& database) { return database.compact(); })
                    : failed_stage("write.compact_tombstones", 1));
            }

            rows.push_back(measure_open_existing(options.verify ? "read.reopen_verify"
                                                               : "read.reopen_read_only",
                                                 options.db_path,
                                                 true,
                                                 options.verify));
        }

        print_rows(rows);
        if (!options.csv_path.empty() && !write_csv(options.csv_path, rows)) {
            std::cerr << "failed to write CSV: " << options.csv_path.string() << '\n';
            return 1;
        }
        const bool ok = std::all_of(rows.begin(), rows.end(), [](const StageRow& row) { return row.ok; });
        return ok ? 0 : 1;
    } catch (const std::exception& ex) {
        std::cerr << "error: " << ex.what() << '\n';
        return 1;
    }
}
