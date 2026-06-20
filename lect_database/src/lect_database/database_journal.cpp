#include <rbf/lect_database/database.h>

#include "database_evidence_codec.h"
#include "database_file_layout.h"

#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace rbf::lect_database {

namespace {

using database_file::journal_path;
using database_file::split;

}  // namespace

void LectDatabase::replay_journal() {
    std::ifstream input(journal_path(config_.path));
    if (!input) {
        return;
    }
    std::vector<std::string> pending;
    std::string line;
    while (std::getline(input, line)) {
        if (line.rfind("begin|", 0) == 0) {
            pending.clear();
            continue;
        }
        if (line.rfind("commit|", 0) == 0) {
            if (!pending.empty()) {
                pending_changes_ = true;
            }
            for (const auto& record : pending) {
                if (record.rfind("split|", 0) == 0) {
                    const auto parts = split(record, '|');
                    if (parts.size() < 6) {
                        continue;
                    }
                    const NodeId parent_id = static_cast<NodeId>(std::stoull(parts[1]));
                    const NodeId left_id = static_cast<NodeId>(std::stoull(parts[2]));
                    const NodeId right_id = static_cast<NodeId>(std::stoull(parts[3]));
                    const int split_dim = std::stoi(parts[4]);
                    const double split_value = std::stod(parts[5]);
                    if (!valid_node_id(left_id) || !valid_node_id(right_id) || left_id == right_id) {
                        continue;
                    }
                    const auto parent_record = read_node(parent_id);
                    if (parent_record) {
                        const auto left_existing = read_node(left_id);
                        const auto right_existing = read_node(right_id);
                        const bool already_applied = left_existing && right_existing &&
                            parent_record->left == left_id && parent_record->right == right_id &&
                            parent_record->split_dim == split_dim && parent_record->split_value == split_value;
                        if (already_applied) {
                            continue;
                        }
                        ++generation_;
                        const int depth = parent_record->depth + 1;
                        PathCode left_path = parent_record->path;
                        PathCode right_path = left_path;
                        left_path.push_child(false);
                        right_path.push_child(true);
                        if (!read_node(left_id)) {
                            NodeRecord left_record;
                            left_record.id = left_id;
                            left_record.parent = parent_id;
                            left_record.depth = depth;
                            left_record.path = std::move(left_path);
                            left_record.generation = generation_;
                            write_node_record(std::move(left_record));
                        }
                        if (!read_node(right_id)) {
                            NodeRecord right_record;
                            right_record.id = right_id;
                            right_record.parent = parent_id;
                            right_record.depth = depth;
                            right_record.path = std::move(right_path);
                            right_record.generation = generation_;
                            write_node_record(std::move(right_record));
                        }
                        if (auto* parent_node = mutable_node(parent_id)) {
                            parent_node->left = left_id;
                            parent_node->right = right_id;
                            parent_node->split_dim = split_dim;
                            parent_node->split_value = split_value;
                            parent_node->dirty = true;
                        }
                    }
                } else if (record.rfind("evidence|", 0) == 0) {
                    auto parsed = parse_evidence_record(record.substr(9));
                    if (parsed) {
                        if (!normalize_evidence_key(&parsed->key)) {
                            continue;
                        }
                        auto shared_record = std::make_shared<EvidenceRecord>(std::move(*parsed));
                        evidence_[shared_record->key] = shared_record;
                        remember_evidence_metadata(*shared_record);
                        if (!config_.open.read_only) {
                            append_evidence_record_to_store(*shared_record);
                        }
                        if (config_.propagate_parent_hulls) {
                            propagate_parent_hulls(shared_record->key.node_id, shared_record->key);
                        }
                    }
                }
            }
            pending.clear();
            ++stats_.journal_transactions;
            continue;
        }
        if (!line.empty()) {
            pending.push_back(line);
        }
    }
    rebuild_layer_index();
    assign_page_ids();
}

bool LectDatabase::ensure_journal_append_stream() {
    if (config_.open.read_only || config_.path.empty()) {
        return false;
    }
    if (journal_append_stream_.is_open()) {
        return static_cast<bool>(journal_append_stream_);
    }
    std::filesystem::create_directories(config_.path);
    journal_append_stream_.open(journal_path(config_.path), std::ios::app);
    return static_cast<bool>(journal_append_stream_);
}

void LectDatabase::close_journal_append_stream() {
    if (journal_append_stream_.is_open()) {
        journal_append_stream_.flush();
        journal_append_stream_.close();
    }
    journal_append_stream_.clear();
}

void LectDatabase::append_committed_transaction(const LectDbTransaction& transaction) {
    if (bulk_prewarm_mode_ || streaming_prewarm_mode_) {
        return;
    }
    if (config_.path.empty() || !ensure_journal_append_stream()) {
        return;
    }
    journal_append_stream_ << "begin|" << transaction.generation << '\n';
    for (const auto& record : transaction.records) {
        journal_append_stream_ << record << '\n';
    }
    if (transaction.committed) {
        journal_append_stream_ << "commit|" << transaction.generation << '\n';
    }
    journal_append_stream_.flush();
    ++stats_.journal_transactions;
}


}  // namespace rbf::lect_database
