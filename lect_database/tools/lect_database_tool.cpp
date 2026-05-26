#include <rbf/lect_database.h>

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <sstream>
#include <string>
#include <vector>

namespace {

std::vector<std::string> split(const std::string& text, char delimiter) {
    std::vector<std::string> parts;
    std::stringstream stream(text);
    std::string item;
    while (std::getline(stream, item, delimiter)) {
        parts.push_back(item);
    }
    return parts;
}

std::vector<rbf::Interval> parse_box(const std::string& text) {
    std::vector<rbf::Interval> intervals;
    for (const auto& part : split(text, ',')) {
        const auto bounds = split(part, ':');
        if (bounds.size() != 2) {
            throw std::runtime_error("box intervals must use lo:hi,lo:hi syntax");
        }
        intervals.push_back({std::stod(bounds[0]), std::stod(bounds[1])});
    }
    return intervals;
}

rbf::lect_database::RangeQueryMode parse_mode(const std::string& text) {
    using rbf::lect_database::RangeQueryMode;
    if (text == "containing") return RangeQueryMode::Containing;
    if (text == "contained-by") return RangeQueryMode::ContainedBy;
    if (text == "intersecting") return RangeQueryMode::Intersecting;
    if (text == "covering-frontier") return RangeQueryMode::CoveringFrontier;
    throw std::runtime_error("unknown range-query mode");
}

void usage() {
    std::cout << "usage:\n"
              << "  lect_database_tool create-demo <db>\n"
              << "  lect_database_tool inspect <db>\n"
              << "  lect_database_tool verify <db>\n"
              << "  lect_database_tool stats <db>\n"
              << "  lect_database_tool dump-node <db> <node_id>\n"
              << "  lect_database_tool dump-box <db> <lo:hi,lo:hi,...>\n"
              << "  lect_database_tool range-query <db> <lo:hi,lo:hi,...> [containing|contained-by|intersecting|covering-frontier]\n"
              << "  lect_database_tool checkpoint <db>\n"
              << "  lect_database_tool compact <db>\n";
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 3) {
        usage();
        return 2;
    }
    const std::string command = argv[1];
    const std::filesystem::path path = argv[2];
    std::string reason;
    if (command == "create-demo") {
        if (std::filesystem::exists(path)) {
            std::cerr << "target already exists: " << path.string() << '\n';
            return 1;
        }
        rbf::lect_database::SplitPolicyDescriptor split_policy;
        split_policy.strategy = rbf::lect_database::SplitStrategy::RoundRobin;
        std::vector<rbf::Interval> root = {{0.0, 2.0}, {0.0, 2.0}};
        rbf::lect_database::LectDatabaseIdentity identity;
        identity.robot_fingerprint = 1;
        identity.root_domain_fingerprint = rbf::lect_database::fingerprint_intervals(root);
        identity.split_policy_hash = rbf::lect_database::split_policy_hash(split_policy);
        identity.split_policy_descriptor = rbf::lect_database::split_policy_descriptor(split_policy);
        identity.endpoint_descriptor = "demo_endpoint";
        identity.envelope_descriptor = "demo_envelope";

        rbf::lect_database::LectDatabaseConfig config;
        config.path = path;
        config.root_intervals = std::move(root);
        config.split_policy = split_policy;
        config.identity = std::move(identity);
        rbf::lect_database::LectDatabase database;
        if (!database.open(std::move(config), &reason)) {
            std::cerr << "failed to create database: " << reason << '\n';
            return 1;
        }
        if (!database.ensure_depth(2) || !database.checkpoint()) {
            std::cerr << "failed to initialize demo database\n";
            return 1;
        }
        std::cout << database.inspect_summary();
        return 0;
    }
    const bool write_mode = command == "checkpoint" || command == "compact";
    auto database = rbf::lect_database::LectDatabase::open_existing(path, !write_mode, &reason);
    if (!database) {
        std::cerr << "failed to open database: " << reason << '\n';
        return 1;
    }

    try {
        if (command == "inspect") {
            std::cout << database->inspect_summary();
            return 0;
        }
        if (command == "verify") {
            const auto result = database->verify(true);
            std::cout << (result.ok ? "ok" : "failed") << '\n';
            for (const auto& error : result.errors) {
                std::cout << "error: " << error << '\n';
            }
            for (const auto& warning : result.warnings) {
                std::cout << "warning: " << warning << '\n';
            }
            return result.ok ? 0 : 1;
        }
        if (command == "stats") {
            const auto& stats = database->stats();
            std::cout << "nodes=" << database->node_count() << '\n'
                      << "evidence=" << database->evidence_count() << '\n'
                      << "generation=" << database->generation() << '\n'
                      << "node_page_reads=" << stats.node_page_reads << '\n'
                      << "node_page_writes=" << stats.node_page_writes << '\n'
                      << "node_page_cache_hits=" << stats.node_page_cache_hits << '\n'
                      << "node_page_cache_misses=" << stats.node_page_cache_misses << '\n'
                      << "node_page_evictions=" << stats.node_page_evictions << '\n'
                      << "node_page_dirty_evictions=" << stats.node_page_dirty_evictions << '\n'
                      << "node_page_dirty_flushes=" << stats.node_page_dirty_flushes << '\n'
                      << "resident_node_pages=" << stats.resident_node_pages << '\n'
                      << "max_resident_node_pages=" << stats.max_resident_node_pages << '\n'
                      << "evidence_reads=" << stats.evidence_reads << '\n'
                      << "evidence_writes=" << stats.evidence_writes << '\n'
                      << "journal_transactions=" << stats.journal_transactions << '\n';
            return 0;
        }
        if (command == "dump-node") {
            if (argc < 4) {
                usage();
                return 2;
            }
            const auto node_id = static_cast<rbf::lect_database::NodeId>(std::stoull(argv[3]));
            const auto topology = database->topology(node_id);
            auto box = database->node_box(node_id);
            if (!box) {
                std::cerr << "node not found\n";
                return 1;
            }
            std::cout << "node=" << topology.id
                      << " parent=" << topology.parent
                      << " left=" << topology.left
                      << " right=" << topology.right
                      << " sibling=" << topology.sibling
                      << " depth=" << topology.depth
                      << " split_dim=" << topology.split_dim
                      << " split_value=" << topology.split_value
                      << " leaf=" << topology.leaf << '\n';
            for (std::size_t dim = 0; dim < box->size(); ++dim) {
                std::cout << "  dim" << dim << "=[" << (*box)[dim].lo << "," << (*box)[dim].hi << "]\n";
            }
            return 0;
        }
        if (command == "dump-box") {
            if (argc < 4) {
                usage();
                return 2;
            }
            const auto lookup = database->box_to_node_exact(database->make_box_key(parse_box(argv[3])));
            if (!lookup.found) {
                std::cerr << "box not found: " << lookup.reason << '\n';
                return 1;
            }
            std::cout << "node=" << lookup.node_id << '\n';
            return 0;
        }
        if (command == "range-query") {
            if (argc < 4) {
                usage();
                return 2;
            }
            const auto mode = argc >= 5 ? parse_mode(argv[4]) : rbf::lect_database::RangeQueryMode::Intersecting;
            rbf::lect_database::LectDatabaseStats stats;
            const auto ids = database->range_query(parse_box(argv[3]), mode, &stats);
            std::cout << "count=" << ids.size() << " visited=" << stats.range_nodes_visited << '\n';
            for (auto id : ids) {
                std::cout << id << '\n';
            }
            return 0;
        }
        if (command == "checkpoint") {
            return database->checkpoint() ? 0 : 1;
        }
        if (command == "compact") {
            return database->compact() ? 0 : 1;
        }
    } catch (const std::exception& ex) {
        std::cerr << "error: " << ex.what() << '\n';
        return 1;
    }

    usage();
    return 2;
}
