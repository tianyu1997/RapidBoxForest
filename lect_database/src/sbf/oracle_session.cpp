#include <LECTDatabase/sbf/oracle.h>

#include "oracle_support.h"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <system_error>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <unistd.h>
#endif

namespace rbf {
namespace {

lect_database::LectDatabaseConfig make_worker_database_config(
    const DatabaseBoxOracle& master,
    const std::filesystem::path& path,
    const std::vector<Interval>& root_intervals,
    int root_depth) {
    lect_database::LectDatabaseConfig config;
    config.path = path;
    config.root_intervals = root_intervals;
    config.split_policy = master.database().split_policy_descriptor();
    config.identity = master.database().identity();
    config.identity.root_domain_fingerprint = lect_database::fingerprint_intervals(root_intervals);
    config.open.read_only = false;
    config.open.create_if_missing = true;
    config.open.verify_identity = true;
    config.open.replay_journal = true;
    config.root_depth = root_depth;
    return config;
}

lect_database::NodeId to_database_node(OracleNodeId node) {
    if (node < 0 || static_cast<std::uint64_t>(node) > std::numeric_limits<lect_database::NodeId>::max()) {
        return lect_database::kInvalidNodeId;
    }
    return static_cast<lect_database::NodeId>(node);
}

OracleNodeId from_database_node(lect_database::NodeId node) {
    return lect_database::valid_node_id(node) ? static_cast<OracleNodeId>(node) : kInvalidOracleNodeId;
}

OracleNodeId remap_lookup(const std::unordered_map<OracleNodeId, OracleNodeId>& node_remap,
                          OracleNodeId worker_node) {
    const auto it = node_remap.find(worker_node);
    return it == node_remap.end() ? kInvalidOracleNodeId : it->second;
}

}  // namespace

DatabaseBoxOracleSession::DatabaseBoxOracleSession(DatabaseBoxOracle& master,
                                                   const OracleSessionConfig& config)
    : master_(master),
      master_domain_root_(config.domain_root >= 0 ? config.domain_root : master.root_node()),
      read_only_(config.read_only),
      temp_dir_(make_temp_dir()) {
    if (master_domain_root_ < 0 || !lect_database::valid_node_id(to_database_node(master_domain_root_)) ||
        !master.database().node(to_database_node(master_domain_root_))) {
        throw std::out_of_range("LECTDatabase oracle session domain root is out of range");
    }
    const auto worker_root = master.node_intervals(master_domain_root_);
    if (worker_root.empty()) {
        throw std::runtime_error("LECTDatabase oracle session domain root has no intervals");
    }
    std::error_code ec;
    std::filesystem::remove_all(temp_dir_, ec);
    ec = {};
    std::filesystem::create_directories(temp_dir_, ec);
    if (ec) {
        throw std::runtime_error("LECTDatabase oracle session failed to create temp directory");
    }

    std::string reason;
    worker_database_.emplace();
    auto worker_config = make_worker_database_config(master_,
                                                     temp_dir_,
                                                     worker_root,
                                                     master_.depth(master_domain_root_));
    if (!worker_database_->open(std::move(worker_config), &reason)) {
        throw std::runtime_error("LECTDatabase oracle session failed to open worker database: " + reason);
    }
    auto worker_validation_config = master_.validation_config();
    worker_validation_config.store_endpoint_evidence_cache = false;
    worker_validation_config.external_evidence_backfill_active = false;
    worker_oracle_ = std::make_unique<DatabaseBoxOracle>(master_.robot(),
                                                         *worker_database_,
                                                         master_.scene(),
                                                         master_.endpoint_config(),
                                                         master_.envelope_config(),
                                                         worker_validation_config,
                                                         master_.external_evidence_source(),
                                                         master_.direct_external_evidence_database());
    if (worker_validation_config.enable_worker_shared_endpoint_cache) {
        worker_oracle_->set_shared_endpoint_cache(master_.shared_endpoint_cache());
    }
    worker_oracle_->best_tighten_depth_dims_ = master_.best_tighten_depth_dims_;
    worker_oracle_->best_tighten_recent_dims_ = master_.best_tighten_recent_dims_;
    worker_oracle_->best_tighten_reference_volumes_ = master_.best_tighten_reference_volumes_;
    node_remap_.emplace(worker_oracle_->root_node(), master_domain_root_);
}

DatabaseBoxOracleSession::~DatabaseBoxOracleSession() {
    worker_oracle_.reset();
    worker_database_.reset();
    std::error_code ec;
    std::filesystem::remove_all(temp_dir_, ec);
}

bool DatabaseBoxOracleSession::commit() {
    if (read_only_) {
        committed_ = true;
        return true;
    }
    if (committed_) {
        return true;
    }
    if (!replay_structure(worker_oracle_->root_node(), master_domain_root_)) {
        return false;
    }
    if (!copy_worker_leaf_evidence()) {
        return false;
    }
    for (const auto& [worker_node, visit_count] : worker_oracle_->visit_counts_) {
        if (visit_count == 0) {
            continue;
        }
        const OracleNodeId master_node = remap_lookup(node_remap_, worker_node);
        if (master_node >= 0) {
            master_.visit_counts_[master_node] += visit_count;
        }
    }
    committed_ = true;
    return true;
}

OracleNodeId DatabaseBoxOracleSession::map_node_to_master(OracleNodeId worker_node) const {
    return remap_lookup(node_remap_, worker_node);
}

bool DatabaseBoxOracleSession::replay_structure(OracleNodeId worker_node, OracleNodeId master_node) {
    node_remap_[worker_node] = master_node;
    const auto worker_topology = worker_database_->topology(to_database_node(worker_node));
    if (worker_topology.leaf) {
        return true;
    }

    auto master_topology = master_.database().topology(to_database_node(master_node));
    if (master_topology.leaf) {
        const auto children = master_.database().split_leaf(to_database_node(master_node),
                                                            worker_topology.split_dim,
                                                            worker_topology.split_value);
        if (!lect_database::valid_node_id(children.first) || !lect_database::valid_node_id(children.second)) {
            return false;
        }
        master_topology = master_.database().topology(to_database_node(master_node));
    } else if (master_topology.split_dim != worker_topology.split_dim ||
               std::abs(master_topology.split_value - worker_topology.split_value) > 1e-12) {
        return false;
    }

    const OracleNodeId master_left = from_database_node(master_topology.left);
    const OracleNodeId master_right = from_database_node(master_topology.right);
    if (master_left < 0 || master_right < 0) {
        return false;
    }
    node_remap_[from_database_node(worker_topology.left)] = master_left;
    node_remap_[from_database_node(worker_topology.right)] = master_right;
    return replay_structure(from_database_node(worker_topology.left), master_left) &&
           replay_structure(from_database_node(worker_topology.right), master_right);
}

bool DatabaseBoxOracleSession::copy_worker_leaf_evidence() {
    for (const auto& record : worker_database_->evidence_records()) {
        const auto topology = worker_database_->topology(record.key.node_id);
        if (!topology.leaf) {
            continue;
        }
        const OracleNodeId mapped_node = remap_lookup(node_remap_, from_database_node(record.key.node_id));
        if (mapped_node < 0) {
            return false;
        }
        auto replay = record;
        replay.key.node_id = to_database_node(mapped_node);
        const auto mapped_topology = master_.database().topology(replay.key.node_id);
        replay.key.node_path = mapped_topology.path;
        replay.key.node_path_valid = lect_database::valid_node_id(mapped_topology.id);
        if (!master_.database().put_evidence(std::move(replay))) {
            return false;
        }
    }
    return true;
}

std::filesystem::path DatabaseBoxOracleSession::make_temp_dir() {
    const auto process_id =
#ifdef _WIN32
        static_cast<unsigned long long>(::GetCurrentProcessId());
#else
        static_cast<unsigned long long>(::getpid());
#endif
    return std::filesystem::temp_directory_path() /
        ("lectdb_sbf_session_" + std::to_string(process_id) + "_" + std::to_string(next_session_id()));
}

}  // namespace rbf
