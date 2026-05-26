#include <rbf/lect_database.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;
namespace ld = rbf::lect_database;

struct Options {
    std::filesystem::path db_path = "outputs/lect_database_benchmark/db";
    std::filesystem::path csv_path;
    int dims = 6;
    int depth = 8;
    std::uint64_t queries = 20000;
    std::uint64_t evidence_records = 20000;
    int payload_floats = 84;
    std::uint32_t page_size_bytes = 4096;
    std::uint32_t max_resident_pages = 4;
    std::uint32_t seed = 42;
    bool fresh = true;
    bool verify = true;
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
        << "  --payload-floats N        floats per endpoint payload (default 84)\n"
        << "  --page-size N             node page size in bytes (default 4096)\n"
        << "  --resident-pages N        LRU resident node page cap (default 4)\n"
        << "  --seed N                  random seed (default 42)\n"
        << "  --reuse                   do not delete an existing database first\n"
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
        else if (arg == "--payload-floats") options.payload_floats = std::stoi(next());
        else if (arg == "--page-size") options.page_size_bytes = static_cast<std::uint32_t>(std::stoul(next()));
        else if (arg == "--resident-pages") options.max_resident_pages = static_cast<std::uint32_t>(std::stoul(next()));
        else if (arg == "--seed") options.seed = static_cast<std::uint32_t>(std::stoul(next()));
        else if (arg == "--reuse") options.fresh = false;
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
    identity.builder_version = "lect_database_benchmark_v1";
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

void print_rows(const std::vector<StageRow>& rows) {
    std::cout << std::left << std::setw(24) << "stage"
              << std::right << std::setw(12) << "ops"
              << std::setw(12) << "ms"
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
        std::cout << std::left << std::setw(24) << row.stage
                  << std::right << std::setw(12) << row.operations
                  << std::setw(12) << std::fixed << std::setprecision(3) << row.elapsed_ms
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
    out << "stage,operations,elapsed_ms,ops_per_sec,nodes,evidence,page_reads,page_writes,cache_hits,cache_misses,evictions,dirty_evictions,dirty_flushes,evidence_reads,evidence_writes,journal_transactions,range_nodes_visited,resident_pages,max_resident_pages,ok\n";
    for (const auto& row : rows) {
        const double ops_per_sec = row.elapsed_ms > 0.0
            ? static_cast<double>(row.operations) * 1000.0 / row.elapsed_ms
            : 0.0;
        out << row.stage << ','
            << row.operations << ','
            << std::setprecision(17) << row.elapsed_ms << ','
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
        if (options.fresh) {
            std::filesystem::remove_all(options.db_path);
        }
        const auto db_parent = options.db_path.parent_path();
        if (!db_parent.empty()) {
            std::filesystem::create_directories(db_parent);
        }

        ld::LectDatabase database;
        std::string reason;
        const auto config = make_config(options);
        if (!database.open(config, &reason)) {
            std::cerr << "failed to open database: " << reason << '\n';
            return 1;
        }

        std::vector<StageRow> rows;
        rows.push_back(measure("add.ensure_depth", 1, database, [&]() {
            return database.ensure_depth(options.depth);
        }));

        const std::uint64_t node_count = static_cast<std::uint64_t>(database.node_count());
        std::mt19937 rng(options.seed);
        std::uniform_int_distribution<std::uint64_t> node_dist(0, node_count == 0 ? 0 : node_count - 1);

        std::vector<ld::NodeId> random_nodes;
        random_nodes.reserve(static_cast<std::size_t>(options.queries));
        for (std::uint64_t i = 0; i < options.queries; ++i) {
            random_nodes.push_back(static_cast<ld::NodeId>(node_dist(rng)));
        }

        std::vector<std::vector<rbf::Interval>> sampled_boxes;
        rows.push_back(measure("read.node_topology", options.queries, database, [&]() {
            for (std::uint64_t i = 0; i < options.queries; ++i) {
                const auto node_id = static_cast<ld::NodeId>(i % node_count);
                if (!database.node(node_id) || !ld::valid_node_id(database.topology(node_id).id)) {
                    return false;
                }
            }
            return true;
        }));

        rows.push_back(measure("read.node_box", options.queries, database, [&]() {
            sampled_boxes.clear();
            sampled_boxes.reserve(static_cast<std::size_t>(std::min<std::uint64_t>(options.queries, 4096)));
            for (std::uint64_t i = 0; i < options.queries; ++i) {
                auto box = database.node_box(random_nodes[static_cast<std::size_t>(i)]);
                if (!box) {
                    return false;
                }
                if (sampled_boxes.size() < 4096) {
                    sampled_boxes.push_back(std::move(*box));
                }
            }
            return true;
        }));

        rows.push_back(measure("read.exact_box_lookup", options.queries, database, [&]() {
            for (std::uint64_t i = 0; i < options.queries; ++i) {
                const auto& box = sampled_boxes[static_cast<std::size_t>(i % sampled_boxes.size())];
                const auto lookup = database.box_to_node_exact(database.make_box_key(box));
                if (!lookup.found) {
                    return false;
                }
            }
            return true;
        }));

        rows.push_back(measure("read.range_query", options.queries, database, [&]() {
            ld::LectDatabaseStats range_stats;
            for (std::uint64_t i = 0; i < options.queries; ++i) {
                const auto ids = database.range_query(random_query_box(options.dims, rng),
                                                      ld::RangeQueryMode::Intersecting,
                                                      &range_stats);
                if (ids.empty() && database.node_count() > 0) {
                    return false;
                }
            }
            return range_stats.range_nodes_visited > 0;
        }));

        std::vector<ld::EvidenceKey> evidence_keys;
        evidence_keys.reserve(static_cast<std::size_t>(options.evidence_records));
        rows.push_back(measure("write.evidence_insert", options.evidence_records, database, [&]() {
            for (std::uint64_t i = 0; i < options.evidence_records; ++i) {
                const auto node_id = static_cast<ld::NodeId>(i % node_count);
                auto key = evidence_key(i, node_id, node_count);
                ld::EvidenceRecord record;
                record.key = key;
                record.payload = make_payload(i, options.payload_floats);
                if (!database.put_evidence(std::move(record))) {
                    return false;
                }
                evidence_keys.push_back(key);
            }
            return true;
        }));

        rows.push_back(measure("read.evidence", options.evidence_records, database, [&]() {
            for (const auto& key : evidence_keys) {
                if (!database.evidence(key)) {
                    return false;
                }
            }
            return true;
        }));

        rows.push_back(measure("write.evidence_update", options.evidence_records, database, [&]() {
            for (std::uint64_t i = 0; i < evidence_keys.size(); ++i) {
                ld::EvidenceRecord record;
                record.key = evidence_keys[static_cast<std::size_t>(i)];
                record.payload = make_payload(i + 13, options.payload_floats);
                if (!database.put_evidence(std::move(record))) {
                    return false;
                }
            }
            return true;
        }));

        rows.push_back(measure("write.checkpoint_live_evidence", 1, database, [&]() {
            return database.checkpoint();
        }));

        rows.push_back(measure("write.compact_live_evidence", 1, database, [&]() {
            return database.compact();
        }));

        const std::uint64_t delete_ops = std::min<std::uint64_t>(node_count, options.evidence_records);
        rows.push_back(measure("delete.node_payloads", delete_ops, database, [&]() {
            for (std::uint64_t i = 0; i < delete_ops; ++i) {
                database.delete_node_payloads(static_cast<ld::NodeId>(i));
            }
            return true;
        }));

        rows.push_back(measure("write.checkpoint_after_delete", 1, database, [&]() {
            return database.checkpoint();
        }));

        rows.push_back(measure("write.compact_after_delete", 1, database, [&]() {
            return database.compact();
        }));

        rows.push_back(measure("read.verify", 1, database, [&]() {
            return !options.verify || database.verify(true).ok;
        }));

        StageRow reopen_row;
        reopen_row.stage = "read.reopen_verify";
        reopen_row.operations = 1;
        const auto reopen_begin = Clock::now();
        auto reopened = ld::LectDatabase::open_existing(options.db_path, true, &reason);
        reopen_row.elapsed_ms = std::chrono::duration<double, std::milli>(Clock::now() - reopen_begin).count();
        reopen_row.ok = reopened.has_value() && (!options.verify || reopened->verify(true).ok);
        if (reopened) {
            reopen_row.nodes = reopened->node_count();
            reopen_row.evidence = reopened->evidence_count();
            reopen_row.delta = reopened->stats();
        }
        rows.push_back(reopen_row);

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
