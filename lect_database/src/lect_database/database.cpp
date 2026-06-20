#include <rbf/lect_database/database.h>

#include "database_evidence_codec.h"
#include "database_file_layout.h"

#include <algorithm>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <system_error>

namespace rbf::lect_database {

namespace {

using database_file::get_double;
using database_file::get_int;
using database_file::get_u64;
using database_file::get_value;
using database_file::journal_path;
using database_file::kCurrentEvidenceEncoding;
using database_file::kCurrentNodeIdScheme;
using database_file::manifest_path;
using database_file::node_page_path;
using database_file::node_pages_path;
using database_file::nodes_path;
using database_file::parse_depth_dimensions;
using database_file::parse_node_record;
using database_file::read_key_values;
using database_file::replace_file;
using database_file::serialize_depth_dimensions;
using database_file::serialize_node_record;
using database_file::split;

bool valid_tree_depth_limit(int max_tree_depth) {
    return max_tree_depth >= 1;
}

bool valid_root_depth(int root_depth, int max_tree_depth) {
    return root_depth >= 0 && root_depth < max_tree_depth;
}

}  // namespace


LectDatabase::~LectDatabase() {
    close_evidence_streams();
    close_journal_append_stream();
}

LectDatabase::LectDatabase(LectDatabase&&) noexcept = default;

LectDatabase& LectDatabase::operator=(LectDatabase&&) noexcept = default;

std::optional<LectDatabase> LectDatabase::open_existing(const std::filesystem::path& path,
                                                        bool read_only,
                                                        std::string* reason) {
    LectDatabase database;
    LectDatabaseConfig config;
    config.path = path;
    config.open.read_only = read_only;
    config.open.create_if_missing = false;
    config.open.verify_identity = false;
    if (!database.open(std::move(config), reason)) {
        return std::nullopt;
    }
    return database;
}

bool LectDatabase::open(LectDatabaseConfig config, std::string* reason) {
    close_journal_append_stream();
    close_evidence_streams();
    config_ = std::move(config);
    opened_ = false;
    node_count_ = 0;
    max_node_id_ = 0;
    next_node_id_ = 0;
    node_ids_.clear();
    node_path_index_.clear();
    node_pages_.clear();
    node_page_clock_ = 0;
    layer_index_.clear();
    evidence_.clear();
    clear_evidence_index();
    evidence_mapped_file_.reset();
    evidence_mapping_stale_ = false;
    evidence_read_buffer_.clear();
    evidence_append_offset_ = 0;
    evidence_appends_since_flush_ = 0;
    generation_ = 0;
    deferred_parent_hull_writes_.clear();
    pending_changes_ = false;
    stats_ = {};

    if (config_.path.empty()) {
        if (reason) *reason = "database path is empty";
        return false;
    }
    if (!valid_tree_depth_limit(config_.max_tree_depth)) {
        if (reason) *reason = "max tree depth must be positive";
        return false;
    }
    if (!valid_root_depth(config_.root_depth, config_.max_tree_depth)) {
        if (reason) *reason = "root depth must be non-negative and less than max tree depth";
        return false;
    }
    std::error_code ignored;
    if (config_.open.create_if_missing) {
        std::filesystem::create_directories(config_.path, ignored);
    }

    const bool has_manifest = std::filesystem::is_regular_file(manifest_path(config_.path));
    LectDatabaseIdentity requested_identity = config_.identity;
    const bool has_requested_identity = requested_identity.robot_fingerprint != 0 ||
                                        requested_identity.root_domain_fingerprint != 0 ||
                                        requested_identity.split_policy_hash != 0;
    if (has_manifest) {
        if (!load_manifest(reason)) {
            return false;
        }
        identity_.schema_version = std::max(identity_.schema_version, kLectDatabaseSchemaVersion);
        if (config_.open.verify_identity && has_requested_identity) {
            std::string mismatch;
            if (!identity_compatible(identity_, requested_identity, &mismatch)) {
                const bool allow_prefix_split_reuse =
                    mismatch == "split policy differs" &&
                    split_policy_prefix_compatible(
                        split_policy_.descriptor(),
                        config_.split_policy,
                        config_.max_tree_depth);
                if (!allow_prefix_split_reuse) {
                    if (reason) *reason = mismatch;
                    return false;
                }
            }
        }
        if (config_.open.metadata_only) {
            // Manifest loaded and identity verified; skip the expensive node /
            // evidence / index materialization. The handle is intended to be
            // discarded after this probe and must not service queries.
            opened_ = true;
            return true;
        }
        if (!load_nodes(reason)) {
            return false;
        }
        if (!load_evidence(reason)) {
            return false;
        }
        if (config_.open.replay_journal) {
            replay_journal();
        }
        rebuild_layer_index();
        assign_page_ids();
        opened_ = true;
        return true;
    }

    if (!config_.open.create_if_missing || config_.open.read_only) {
        if (reason) *reason = "database manifest does not exist";
        return false;
    }
    if (config_.root_intervals.empty()) {
        if (reason) *reason = "new database requires root intervals";
        return false;
    }

    root_intervals_ = config_.root_intervals;
    coverage_intervals_ = config_.coverage_intervals.empty() ? root_intervals_ : config_.coverage_intervals;
    if (coverage_intervals_.size() != root_intervals_.size()) {
        if (reason) *reason = "coverage intervals must have the same dimensionality as root intervals";
        return false;
    }
    split_policy_ = SplitPolicy(config_.split_policy);
    identity_ = config_.identity;
    identity_.schema_version = std::max(identity_.schema_version, kLectDatabaseSchemaVersion);
    if (identity_.root_domain_fingerprint == 0) {
        identity_.root_domain_fingerprint = fingerprint_intervals(root_intervals_);
    }
    if (identity_.split_policy_hash == 0) {
        identity_.split_policy_hash = split_policy_.hash();
    }
    if (identity_.split_policy_descriptor.empty()) {
        identity_.split_policy_descriptor = ::rbf::lect_database::split_policy_descriptor(split_policy_.descriptor());
    }

    NodeRecord root;
    root.id = 0;
    root.depth = config_.root_depth;
    write_node_record(std::move(root));
    rebuild_layer_index();
    assign_page_ids();
    opened_ = true;
    const bool saved = save_manifest() && save_nodes() && save_evidence();
    if (saved) {
        pending_changes_ = false;
    }
    return saved;
}

LectDbQuerySession LectDatabase::make_query_session() const {
    return LectDbQuerySession(*this);
}

VerificationResult LectDatabase::verify(bool strict) const {
    VerificationResult result;
    if (!opened_) {
        result.add_error("database is not open");
        return result;
    }
    const auto load_uncached_evidence = [this](const EvidenceKey& raw_key) -> std::optional<EvidenceRecord> {
        EvidenceKey normalized_key = raw_key;
        if (!normalize_evidence_key(&normalized_key)) {
            return std::nullopt;
        }
        const auto* index_entry = find_evidence_index(normalized_key);
        if (index_entry == nullptr || index_entry->size == 0) {
            return std::nullopt;
        }
        const auto bytes_view = load_evidence_bytes(index_entry->offset, index_entry->size);
        if (!bytes_view) {
            return std::nullopt;
        }
        auto record = parse_binary_evidence_record(*bytes_view);
        if (!record || !normalize_evidence_key(&record->key)) {
            return std::nullopt;
        }
        return record;
    };
    if (identity_.root_domain_fingerprint != fingerprint_intervals(root_intervals_)) {
        result.add_error("root domain fingerprint does not match stored root intervals");
    }
    if (identity_.split_policy_hash != split_policy_.hash()) {
        result.add_error("split policy hash does not match descriptor");
    }
    const auto root_record = read_node(root_node());
    if (node_count_ == 0 || !root_record || root_record->id != 0 || valid_node_id(root_record->parent)) {
        result.add_error("root node is missing or malformed");
    }
    for (NodeId node_id : sorted_node_ids()) {
        const auto item_opt = read_node(node_id);
        if (!item_opt) {
            result.add_error("node page is missing a row");
            continue;
        }
        const auto& item = *item_opt;
        if (item.id != node_id) {
            result.add_error("node id does not match table position");
        }
        if (valid_node_id(item.parent)) {
            const auto parent_node = read_node(item.parent);
            if (!parent_node) {
                result.add_error("node has invalid parent id");
            } else {
                if (parent_node->left != item.id && parent_node->right != item.id) {
                    result.add_error("parent does not reference child");
                }
            }
        }
        if (!item.is_leaf()) {
            const auto left_node = read_node(item.left);
            const auto right_node = read_node(item.right);
            if (!left_node || !right_node) {
                result.add_error("internal node has invalid child id");
            } else if (left_node->parent != item.id || right_node->parent != item.id) {
                result.add_error("child parent pointer mismatch");
            }
        }
        const auto key = make_box_key(node_box_unchecked(item.id));
        const auto lookup = box_to_node_exact(key);
        if (!lookup.found || lookup.node_id != item.id) {
            result.add_error("node box does not round-trip through exact box lookup");
        }
    }
    for (const auto& [depth, ids] : layer_index_) {
        for (NodeId id : ids) {
            const auto item = read_node(id);
            if (!item || item->depth != depth) {
                result.add_error("layer index contains an invalid node");
            }
        }
    }
    std::vector<float> expected_payload;
    for (const auto& slot : evidence_index_) {
        if (slot.key.node_id == kInvalidNodeId || slot.entry.unavailable) {
            continue;
        }
        const auto record_storage = load_uncached_evidence(slot.key);
        if (!record_storage || record_storage->unavailable) {
            result.add_error("evidence row could not be materialized");
            continue;
        }
        const auto& record = *record_storage;
        if (record.checksum != payload_checksum(record.payload)) {
            result.add_error("evidence payload checksum mismatch");
        }
        if (strict && record.child_hull) {
            const auto parent_opt = read_node(record.key.node_id);
            if (!parent_opt) {
                continue;
            }
            const auto& parent_node = *parent_opt;
            if (parent_node.is_leaf()) {
                continue;
            }

            EvidenceKey left_key = record.key;
            EvidenceKey right_key = record.key;
            left_key.node_id = parent_node.left;
            right_key.node_id = parent_node.right;
            const auto left_record = load_uncached_evidence(left_key);
            const auto right_record = load_uncached_evidence(right_key);
            if (!left_record || !right_record || left_record->unavailable || right_record->unavailable) {
                continue;
            }
            if (left_record->payload.size() != right_record->payload.size()) {
                continue;
            }

            expected_payload.clear();
            if (!merge_payload_hull(record.key.payload_kind,
                                    left_record->payload,
                                    right_record->payload,
                                    expected_payload)) {
                continue;
            }
            if (!payloads_equal(expected_payload, record.payload)) {
                result.add_error("parent hull evidence does not match child hull");
            }
        }
    }
    if (!strict && !result.ok) {
        result.add_warning("quick verification found errors; run strict verification for details");
    }
    return result;
}

bool LectDatabase::checkpoint() {
    if (config_.open.read_only || !opened_) {
        return false;
    }
    if (!drain_deferred_parent_hulls()) {
        return false;
    }
    if (!pending_changes_) {
        return true;
    }
    close_journal_append_stream();
    const bool ok_manifest = save_manifest();
    const bool ok_nodes = save_nodes();
    const bool ok_evidence = save_evidence();
    if (ok_manifest && ok_nodes && ok_evidence) {
        close_evidence_streams();
        std::ofstream clear(journal_path(config_.path), std::ios::trunc);
        const bool ok = static_cast<bool>(clear);
        if (ok) {
            pending_changes_ = false;
        }
        return ok;
    }
    return false;
}

bool LectDatabase::compact() {
    return checkpoint();
}

std::string LectDatabase::inspect_summary() const {
    std::ostringstream out;
    out << "lect_database\n"
        << "  path=" << config_.path.string() << "\n"
        << "  identity_hash=" << identity_hash(identity_) << "\n"
        << "  identity=" << identity_descriptor(identity_) << "\n"
        << "  root=" << interval_descriptor(root_intervals_) << "\n"
        << "  split_policy=" << ::rbf::lect_database::split_policy_descriptor(split_policy_.descriptor()) << "\n"
        << "  generation=" << generation_ << "\n"
        << "  nodes=" << node_count_ << "\n"
        << "  resident_node_pages=" << node_pages_.size() << "\n"
        << "  evidence=" << evidence_count() << "\n";
    return out.str();
}

bool LectDatabase::normalize_evidence_key(EvidenceKey* key) const {
    if (key == nullptr) {
        return false;
    }
    if (key->node_path_valid) {
        const auto found = node_path_index_.find(key->node_path);
        if (found == node_path_index_.end()) {
            return false;
        }
        key->node_id = found->second;
        return true;
    }
    if (!valid_node_id(key->node_id)) {
        return false;
    }
    const auto node_item = read_node(key->node_id);
    if (!node_item) {
        return false;
    }
    key->node_path = node_item->path;
    key->node_path_valid = true;
    return true;
}

EvidenceKey LectDatabase::evidence_key_for_node(NodeId node_id, const EvidenceKey* key_template) const {
    EvidenceKey key = key_template == nullptr ? EvidenceKey{} : *key_template;
    key.node_id = node_id;
    key.node_path_valid = false;
    key.node_path = {};
    normalize_evidence_key(&key);
    return key;
}

bool LectDatabase::load_manifest(std::string* reason) {
    const auto values = read_key_values(manifest_path(config_.path));
    if (values.empty()) {
        if (reason) *reason = "manifest is empty or unreadable";
        return false;
    }
    const std::string node_id_scheme = get_value(values, "node_id_scheme");
    if (node_id_scheme != kCurrentNodeIdScheme) {
        if (reason) *reason = "node id scheme is missing or unsupported; rebuild the database";
        return false;
    }
    identity_.schema_version = static_cast<std::uint32_t>(get_u64(values, "schema_version", kLectDatabaseSchemaVersion));
    if (identity_.schema_version != kLectDatabaseSchemaVersion) {
        if (reason) *reason = "schema version is unsupported; rebuild the database";
        return false;
    }
    if (get_value(values, "evidence_encoding") != kCurrentEvidenceEncoding) {
        if (reason) *reason = "evidence encoding is missing or unsupported; rebuild the database";
        return false;
    }
    identity_.robot_fingerprint = get_u64(values, "robot_fingerprint");
    identity_.root_domain_fingerprint = get_u64(values, "root_domain_fingerprint");
    identity_.split_policy_hash = get_u64(values, "split_policy_hash");
    identity_.symmetry_hash = get_u64(values, "symmetry_hash");
    identity_.canonical_mode = get_int(values, "canonical_mode") != 0;
    identity_.symmetry_descriptor = get_value(values, "symmetry_descriptor");
    identity_.split_policy_descriptor = get_value(values, "split_policy_descriptor");
    identity_.endpoint_descriptor = get_value(values, "endpoint_descriptor", identity_.endpoint_descriptor);
    identity_.envelope_descriptor = get_value(values, "envelope_descriptor", identity_.envelope_descriptor);
    identity_.payload_layout = get_value(values, "payload_layout", identity_.payload_layout);
    identity_.builder_version = get_value(values, "builder_version");

    SplitPolicyDescriptor descriptor;
    descriptor.strategy = static_cast<SplitStrategy>(get_int(values, "split_strategy"));
    descriptor.min_width = get_double(values, "split_min_width");
    descriptor.midpoint = get_int(values, "split_midpoint", 1) != 0;
    descriptor.deterministic_tie_break = get_int(values, "split_deterministic_tie_break", 1) != 0;
    descriptor.dimension_schedule_hash = get_value(values, "split_dimension_schedule_hash");
    descriptor.depth_dimensions = parse_depth_dimensions(get_value(values, "split_depth_dimensions"));
    split_policy_ = SplitPolicy(descriptor);
    generation_ = get_u64(values, "generation");
    config_.page_size_bytes = static_cast<std::uint32_t>(get_u64(values, "page_size_bytes", config_.page_size_bytes));
    config_.max_resident_pages = static_cast<std::uint32_t>(get_u64(values, "max_resident_pages", config_.max_resident_pages));
    config_.root_depth = get_int(values, "root_depth", config_.root_depth);
    config_.max_tree_depth = get_int(values, "max_tree_depth", config_.max_tree_depth);
    if (!valid_tree_depth_limit(config_.max_tree_depth)) {
        if (reason) *reason = "manifest max_tree_depth must be positive";
        return false;
    }
    if (!valid_root_depth(config_.root_depth, config_.max_tree_depth)) {
        if (reason) *reason = "manifest root_depth must be non-negative and less than max_tree_depth";
        return false;
    }
    node_count_ = static_cast<NodeId>(get_u64(values, "node_count", node_count_));
    max_node_id_ = static_cast<NodeId>(get_u64(values, "max_node_id", max_node_id_));

    const int dims = get_int(values, "root_dims");
    root_intervals_.clear();
    root_intervals_.reserve(static_cast<std::size_t>(std::max(0, dims)));
    for (int dim = 0; dim < dims; ++dim) {
        root_intervals_.push_back({get_double(values, "root_" + std::to_string(dim) + "_lo"),
                                   get_double(values, "root_" + std::to_string(dim) + "_hi")});
    }
    const int coverage_dims = get_int(values, "coverage_dims", dims);
    coverage_intervals_.clear();
    coverage_intervals_.reserve(static_cast<std::size_t>(std::max(0, coverage_dims)));
    for (int dim = 0; dim < coverage_dims; ++dim) {
        const std::string lo_key = "coverage_" + std::to_string(dim) + "_lo";
        const std::string hi_key = "coverage_" + std::to_string(dim) + "_hi";
        if (values.find(lo_key) == values.end() || values.find(hi_key) == values.end()) {
            coverage_intervals_.clear();
            break;
        }
        coverage_intervals_.push_back({get_double(values, lo_key),
                                       get_double(values, hi_key)});
    }
    if (coverage_intervals_.empty()) {
        coverage_intervals_ = root_intervals_;
    }
    if (coverage_intervals_.size() != root_intervals_.size()) {
        if (reason) *reason = "manifest coverage intervals have incompatible dimensionality";
        return false;
    }
    config_.root_intervals = root_intervals_;
    config_.coverage_intervals = coverage_intervals_;
    config_.split_policy = descriptor;
    return true;
}

bool LectDatabase::save_manifest() const {
    std::filesystem::create_directories(config_.path);
    const auto path = manifest_path(config_.path);
    const auto tmp = path.string() + ".tmp";
    std::ofstream out(tmp, std::ios::trunc);
    if (!out) {
        return false;
    }
    const auto& descriptor = split_policy_.descriptor();
    out << "schema_version=" << identity_.schema_version << '\n'
        << "robot_fingerprint=" << identity_.robot_fingerprint << '\n'
        << "root_domain_fingerprint=" << identity_.root_domain_fingerprint << '\n'
        << "split_policy_hash=" << identity_.split_policy_hash << '\n'
        << "symmetry_hash=" << identity_.symmetry_hash << '\n'
        << "canonical_mode=" << (identity_.canonical_mode ? 1 : 0) << '\n'
        << "symmetry_descriptor=" << identity_.symmetry_descriptor << '\n'
        << "split_policy_descriptor=" << identity_.split_policy_descriptor << '\n'
        << "endpoint_descriptor=" << identity_.endpoint_descriptor << '\n'
        << "envelope_descriptor=" << identity_.envelope_descriptor << '\n'
        << "payload_layout=" << identity_.payload_layout << '\n'
        << "builder_version=" << identity_.builder_version << '\n'
        << "node_id_scheme=" << kCurrentNodeIdScheme << '\n'
        << "evidence_encoding=" << kCurrentEvidenceEncoding << '\n'
        << "split_strategy=" << static_cast<int>(descriptor.strategy) << '\n'
        << "split_min_width=" << std::setprecision(17) << descriptor.min_width << '\n'
        << "split_midpoint=" << (descriptor.midpoint ? 1 : 0) << '\n'
        << "split_deterministic_tie_break=" << (descriptor.deterministic_tie_break ? 1 : 0) << '\n'
        << "split_dimension_schedule_hash=" << descriptor.dimension_schedule_hash << '\n'
        << "split_depth_dimensions=" << serialize_depth_dimensions(descriptor.depth_dimensions) << '\n'
        << "root_dims=" << root_intervals_.size() << '\n'
        << "coverage_dims=" << coverage_intervals_.size() << '\n'
        << "page_size_bytes=" << config_.page_size_bytes << '\n'
        << "max_resident_pages=" << config_.max_resident_pages << '\n'
        << "root_depth=" << config_.root_depth << '\n'
        << "max_tree_depth=" << config_.max_tree_depth << '\n'
        << "node_count=" << node_count_ << '\n'
        << "max_node_id=" << max_node_id_ << '\n'
        << "generation=" << generation_ << '\n';
    for (std::size_t dim = 0; dim < root_intervals_.size(); ++dim) {
        out << "root_" << dim << "_lo=" << std::setprecision(17) << root_intervals_[dim].lo << '\n'
            << "root_" << dim << "_hi=" << std::setprecision(17) << root_intervals_[dim].hi << '\n';
    }
    for (std::size_t dim = 0; dim < coverage_intervals_.size(); ++dim) {
        out << "coverage_" << dim << "_lo=" << std::setprecision(17) << coverage_intervals_[dim].lo << '\n'
            << "coverage_" << dim << "_hi=" << std::setprecision(17) << coverage_intervals_[dim].hi << '\n';
    }
    out.close();
    return static_cast<bool>(out) && replace_file(tmp, path);
}

bool LectDatabase::load_nodes(std::string* reason) {
    node_pages_.clear();
    node_page_clock_ = 0;
    node_ids_.clear();
    node_path_index_.clear();
    node_count_ = 0;
    max_node_id_ = 0;
    next_node_id_ = 0;
    const bool has_page_dir = std::filesystem::is_directory(node_pages_path(config_.path));
    if (has_page_dir) {
        for (const auto& entry : std::filesystem::directory_iterator(node_pages_path(config_.path))) {
            if (!entry.is_regular_file()) {
                continue;
            }
            std::ifstream page_input(entry.path());
            std::string line;
            while (std::getline(page_input, line)) {
                if (line.empty()) {
                    continue;
                }
                auto record = parse_node_record(line);
                if (record && valid_node_id(record->id)) {
                    if (!remember_node_record(*record)) {
                        if (reason) *reason = "node path index is malformed";
                        return false;
                    }
                }
            }
        }
        update_resident_page_stats();
        return true;
    }

    std::ifstream input(nodes_path(config_.path));
    if (!input) {
        if (reason) *reason = "nodes.pages is missing";
        return false;
    }
    std::string line;
    while (std::getline(input, line)) {
        if (line.empty()) {
            continue;
        }
        auto record = parse_node_record(line);
        if (!record) {
            if (reason) *reason = "node row is malformed";
            return false;
        }
        write_node_record(std::move(*record));
    }
    if (!config_.open.read_only) {
        flush_all_node_pages();
    }
    return true;
}

bool LectDatabase::save_nodes() const {
    if (!flush_all_node_pages()) {
        return false;
    }
    const auto path = nodes_path(config_.path);
    const auto tmp = path.string() + ".tmp";
    std::ofstream out(tmp, std::ios::trunc);
    if (!out) {
        return false;
    }
    for (NodeId node_id : sorted_node_ids()) {
        const auto record = read_node(node_id);
        if (!record) {
            return false;
        }
        out << serialize_node_record(*record) << '\n';
    }
    out.close();
    return static_cast<bool>(out) && replace_file(tmp, path);
}

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

bool LectDatabase::propagate_parent_hulls(NodeId node_id, EvidenceKey key_template) {
    return propagate_parent_hulls_from(parent(node_id), std::move(key_template), nullptr);
}

bool LectDatabase::propagate_parent_hulls_from(NodeId parent_id,
                                               EvidenceKey key_template,
                                               std::shared_ptr<const EvidenceRecord> child_record) {
    NodeId cursor = parent_id;
    while (has_node(cursor)) {
        const auto parent_node = read_node(cursor);
        if (!parent_node) {
            break;
        }
        auto parent_record = child_record != nullptr
            ? build_parent_hull_from_child(*parent_node, *child_record, key_template)
            : build_parent_hull_from_node(*parent_node, key_template);
        if (!parent_record) {
            break;
        }
        parent_record->generation = generation_;
        quantize_payload_outward(parent_record->key.payload_kind, parent_record->payload);
        parent_record->checksum = payload_checksum(parent_record->payload);
        auto shared_parent_record = std::make_shared<EvidenceRecord>(std::move(*parent_record));
        auto [parent_it, parent_inserted] = evidence_.insert_or_assign(shared_parent_record->key,
                                                                       shared_parent_record);
        (void)parent_inserted;
        remember_evidence_metadata(*parent_it->second);
        if (!config_.open.read_only && !append_evidence_record_to_store(*parent_it->second)) {
            return false;
        }
        child_record = parent_it->second;
        cursor = parent_node->parent;
    }
    return true;
}

bool LectDatabase::drain_deferred_parent_hulls() {
    if (deferred_parent_hull_writes_.empty()) {
        return true;
    }
    if (!config_.propagate_parent_hulls) {
        deferred_parent_hull_writes_.clear();
        return true;
    }

    std::vector<DeferredParentHullWrite> pending;
    pending.swap(deferred_parent_hull_writes_);
    for (const auto& item : pending) {
        const auto node_item = read_node(item.key.node_id);
        if (!node_item) {
            return false;
        }
        auto stored = evidence(item.key);
        if (!stored) {
            continue;
        }
        std::shared_ptr<const EvidenceRecord> propagated_child = stored->storage;
        if (!stored->child_hull && !node_item->is_leaf()) {
            if (auto child_hull = build_parent_hull_from_node(*node_item, item.key)) {
                child_hull->generation = generation_;
                quantize_payload_outward(child_hull->key.payload_kind, child_hull->payload);
                child_hull->checksum = payload_checksum(child_hull->payload);
                auto child_hull_record = std::make_shared<EvidenceRecord>(std::move(*child_hull));
                auto [child_hull_it, child_hull_inserted] = evidence_.insert_or_assign(child_hull_record->key,
                                                                                       child_hull_record);
                (void)child_hull_inserted;
                if (!append_evidence_record_to_store(*child_hull_it->second)) {
                    return false;
                }
                propagated_child = child_hull_it->second;
                pending_changes_ = true;
            }
        }
        if (!propagate_parent_hulls_from(node_item->parent, item.key, propagated_child)) {
            return false;
        }
    }
    return true;
}

std::optional<EvidenceRecord> LectDatabase::build_parent_hull_from_child(const NodeRecord& parent_node,
                                                                         const EvidenceRecord& child_record,
                                                                         const EvidenceKey& key_template) const {
    if (parent_node.is_leaf() || child_record.unavailable) {
        return std::nullopt;
    }
    const bool child_is_left = child_record.key.node_id == parent_node.left;
    const bool child_is_right = child_record.key.node_id == parent_node.right;
    if (!child_is_left && !child_is_right) {
        return build_parent_hull_from_node(parent_node, key_template);
    }

    EvidenceKey sibling_key = evidence_key_for_node(child_is_left ? parent_node.right : parent_node.left,
                                                    &key_template);
    const auto sibling_record = evidence(sibling_key);
    if (!sibling_record || sibling_record->payload.size() != child_record.payload.size()) {
        return std::nullopt;
    }

    const auto left_payload = child_is_left ? std::span<const float>(child_record.payload)
                                            : sibling_record->payload;
    const auto right_payload = child_is_left ? sibling_record->payload
                                             : std::span<const float>(child_record.payload);
    EvidenceRecord out;
    out.key = evidence_key_for_node(parent_node.id, &key_template);
    out.child_hull = true;
    if (!merge_payload_hull(key_template.payload_kind,
                            left_payload,
                            right_payload,
                            out.payload)) {
        return std::nullopt;
    }
    return out;
}

std::optional<EvidenceRecord> LectDatabase::build_parent_hull_from_node(const NodeRecord& parent_node,
                                                                        const EvidenceKey& key_template) const {
    if (parent_node.is_leaf()) {
        return std::nullopt;
    }
    EvidenceKey left_key = evidence_key_for_node(parent_node.left, &key_template);
    EvidenceKey right_key = evidence_key_for_node(parent_node.right, &key_template);
    const auto left_record = evidence(left_key);
    const auto right_record = evidence(right_key);
    if (!left_record || !right_record || left_record->payload.size() != right_record->payload.size()) {
        return std::nullopt;
    }
    EvidenceRecord out;
    out.key = evidence_key_for_node(parent_node.id, &key_template);
    out.child_hull = true;
    if (!merge_payload_hull(key_template.payload_kind,
                            left_record->payload,
                            right_record->payload,
                            out.payload)) {
        return std::nullopt;
    }
    return out;
}

std::optional<EvidenceRecord> LectDatabase::build_parent_hull(NodeId parent_id,
                                                              const EvidenceKey& key_template) const {
    const auto parent_node = read_node(parent_id);
    if (!parent_node) {
        return std::nullopt;
    }
    return build_parent_hull_from_node(*parent_node, key_template);
}

std::size_t LectDatabase::materialize_internal_parent_hulls_bottom_up(
    int deepest_depth,
    const EvidenceKey& key_template,
    const std::function<void(int depth, std::size_t built)>& layer_progress) {
    std::size_t built = 0;
    for (int depth = deepest_depth - 1; depth >= 0; --depth) {
        for (NodeId node_id : layer_nodes(depth)) {
            const auto parent_node = read_node(node_id);
            if (!parent_node || parent_node->is_leaf()) {
                continue;
            }
            auto parent_record = build_parent_hull_from_node(*parent_node, key_template);
            if (!parent_record) {
                // Children not (yet) materialized (e.g. sector-boundary straddle
                // leaf with no stored evidence); leave this ancestor uncached.
                continue;
            }
            parent_record->generation = generation_;
            quantize_payload_outward(parent_record->key.payload_kind, parent_record->payload);
            parent_record->checksum = payload_checksum(parent_record->payload);
            auto shared_parent_record = std::make_shared<EvidenceRecord>(std::move(*parent_record));
            auto [parent_it, parent_inserted] = evidence_.insert_or_assign(shared_parent_record->key,
                                                                           shared_parent_record);
            (void)parent_inserted;
            remember_evidence_metadata(*parent_it->second);
            if (!config_.open.read_only && !append_evidence_record_to_store(*parent_it->second)) {
                return built;
            }
            pending_changes_ = true;
            ++built;
        }
        if (layer_progress) {
            layer_progress(depth, built);
        }
    }
    return built;
}

LectDbQuerySession::LectDbQuerySession(const LectDatabase& database)
    : database_(&database) {}

std::optional<std::vector<Interval>> LectDbQuerySession::node_box(NodeId node_id) {
    if (database_ == nullptr) {
        return std::nullopt;
    }
    if (node_id == cached_node_) {
        ++stats_.query_path_cache_hits;
        return cached_box_;
    }
    ++stats_.query_path_cache_misses;
    auto box = database_->node_box(node_id);
    if (box) {
        cached_node_ = node_id;
        cached_box_ = *box;
    }
    return box;
}

BoxLookupResult LectDbQuerySession::box_to_node_exact(const BoxKey& box) {
    if (database_ == nullptr) {
        return {false, kInvalidNodeId, "query session is not attached"};
    }
    if (database_->has_node(cached_node_) && database_->intervals_equal(cached_box_, box.intervals, box.tolerance)) {
        ++stats_.query_path_cache_hits;
        return {true, cached_node_, {}};
    }
    ++stats_.query_path_cache_misses;
    auto result = database_->box_to_node_exact(box);
    if (result.found) {
        cached_node_ = result.node_id;
        cached_box_ = box.intervals;
    }
    return result;
}

}  // namespace rbf::lect_database
