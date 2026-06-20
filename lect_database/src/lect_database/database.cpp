#include <rbf/lect_database/database.h>

#include "database_evidence_codec.h"
#include "database_file_layout.h"
#include "database_mapped_file.h"

#include <algorithm>
#include <cstddef>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <system_error>

namespace rbf::lect_database {

namespace {

constexpr std::size_t kBufferedIoBytes = 1u << 20;
constexpr std::uint64_t kEvidenceAppendsPerFlush = 1024;

using database_file::evidence_index_path;
using database_file::evidence_path;
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

bool LectDatabase::put_evidence(EvidenceRecord record) {
    if (config_.open.read_only) {
        return false;
    }
    if (!normalize_evidence_key(&record.key)) {
        return false;
    }
    const EvidenceKey key = record.key;
    const auto node_item = read_node(key.node_id);
    if (!node_item) {
        return false;
    }

    ++generation_;
    pending_changes_ = true;
    record.generation = generation_;
    quantize_payload_outward(record.key.payload_kind, record.payload);
    record.checksum = payload_checksum(record.payload);
    // The text journal is a write-ahead log for crash recovery between
    // checkpoints. During bulk/streaming prewarm we skip it entirely: the final
    // checkpoint truncates the journal anyway and persists the authoritative
    // binary store, so per-record journaling is pure write amplification.
    const std::string journal_record =
        (bulk_prewarm_mode_ || streaming_prewarm_mode_)
            ? std::string()
            : ("evidence|" + serialize_evidence_record(record));
    const bool direct_evidence = !record.child_hull;
    const bool node_is_internal = !node_item->is_leaf();
    const NodeId parent_id = node_item->parent;
    const bool streaming_append_only = streaming_prewarm_mode_ &&
        !config_.propagate_parent_hulls &&
        streaming_resident_cap_ > 0 &&
        evidence_.size() >= streaming_resident_cap_;

    if (streaming_append_only) {
        if (!append_evidence_record_to_store(record)) {
            return false;
        }
    } else {
        auto stored_record = std::make_shared<EvidenceRecord>(std::move(record));
        auto [stored_it, stored_inserted] = evidence_.insert_or_assign(key, stored_record);
        (void)stored_inserted;
        if (!append_evidence_record_to_store(*stored_it->second)) {
            return false;
        }
        if (config_.propagate_parent_hulls) {
            if (config_.defer_parent_hull_writes) {
                deferred_parent_hull_writes_.push_back(DeferredParentHullWrite{key});
            } else {
                std::shared_ptr<const EvidenceRecord> propagated_child = stored_it->second;
                if (direct_evidence && node_is_internal) {
                    if (auto child_hull = build_parent_hull_from_node(*node_item, key)) {
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
                    }
                }
                if (!propagate_parent_hulls_from(parent_id, key, propagated_child)) {
                    return false;
                }
            }
        }
    }

    LectDbTransaction transaction;
    transaction.generation = generation_;
    transaction.records.push_back(journal_record);
    transaction.committed = true;
    append_committed_transaction(transaction);
    ++stats_.evidence_writes;
    if (!maybe_flush_incremental_storage()) {
        return false;
    }
    trim_evidence_cache();
    return true;
}

std::optional<EvidenceRecordView> LectDatabase::evidence(const EvidenceKey& key) const {
    ++stats_.evidence_reads;
    EvidenceKey normalized_key = key;
    if (!normalize_evidence_key(&normalized_key)) {
        return std::nullopt;
    }
    auto it = evidence_.find(normalized_key);
    if (it == evidence_.end()) {
        auto loaded = load_indexed_evidence(normalized_key);
        if (!loaded) {
            return std::nullopt;
        }
        if (loaded->unavailable) {
            return std::nullopt;
        }
        trim_evidence_cache();
        return make_evidence_view(std::move(loaded));
    }
    if (it->second == nullptr || it->second->unavailable) {
        return std::nullopt;
    }
    auto cached_record = it->second;
    trim_evidence_cache();
    return make_evidence_view(cached_record);
}

std::optional<std::uint64_t> LectDatabase::evidence_offset(const EvidenceKey& key) const {
    const auto* index_entry = find_evidence_index(key);
    if (index_entry == nullptr || index_entry->unavailable) {
        return std::nullopt;
    }
    return index_entry->offset;
}

bool LectDatabase::has_evidence(const EvidenceKey& key) const {
    EvidenceKey normalized_key = key;
    if (!normalize_evidence_key(&normalized_key)) {
        return false;
    }
    const auto loaded = evidence_.find(normalized_key);
    if (loaded != evidence_.end()) {
        return loaded->second != nullptr && !loaded->second->unavailable;
    }
    const auto* indexed = find_evidence_index(normalized_key);
    return indexed != nullptr && !indexed->unavailable;
}

std::vector<EvidenceRecord> LectDatabase::evidence_records() const {
    std::vector<EvidenceRecord> records;
    records.reserve(evidence_count());
    for (const auto& slot : evidence_index_) {
        if (slot.key.node_id == kInvalidNodeId || slot.entry.unavailable) {
            continue;
        }
        if (auto record = evidence(slot.key)) {
            records.push_back(clone_evidence_record(*record));
        }
    }
    return records;
}

std::optional<EvidenceRecordView> LectDatabase::endpoint_for_box_exact(const BoxKey& box,
                                                                       EvidenceKey key_template) const {
    const auto lookup = box_to_node_exact(box);
    if (!lookup.found) {
        return std::nullopt;
    }
    key_template.node_id = lookup.node_id;
    key_template.node_path = {};
    key_template.node_path_valid = false;
    key_template.payload_kind = EvidencePayloadKind::EndpointEnvelope;
    return evidence(key_template);
}

std::size_t LectDatabase::delete_node_payloads(NodeId node_id) {
    if (config_.open.read_only) {
        return 0;
    }
    std::vector<EvidenceKey> keys;
    for (const auto& slot : evidence_index_) {
        if (slot.key.node_id == kInvalidNodeId) {
            continue;
        }
        if (slot.key.node_id == node_id && !slot.entry.unavailable) {
            keys.push_back(slot.key);
        }
    }
    for (const auto& [key, record] : evidence_) {
        if (record != nullptr && key.node_id == node_id && !record->unavailable &&
            std::find(keys.begin(), keys.end(), key) == keys.end()) {
            keys.push_back(key);
        }
    }
    std::size_t erased = 0;
    for (const auto& key : keys) {
        EvidenceRecord tombstone;
        tombstone.key = key;
        tombstone.unavailable = true;
        tombstone.generation = ++generation_;
        tombstone.checksum = 0;
        if (!append_evidence_record_to_store(tombstone)) {
            break;
        }
        evidence_.erase(key);
        ++erased;
    }
    if (erased > 0) {
        pending_changes_ = true;
        maybe_flush_incremental_storage();
    }
    return erased;
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

bool LectDatabase::load_evidence(std::string* reason) {
    evidence_.clear();
    clear_evidence_index();
    close_evidence_streams();
    evidence_append_offset_ = 0;
    evidence_appends_since_flush_ = 0;
    evidence_index_sidecar_dirty_ = false;
    const auto path = evidence_path(config_.path);
    std::error_code error;
    const bool evidence_exists = std::filesystem::exists(path, error);
    if (error || !evidence_exists) {
        return true;
    }
    const std::uint64_t evidence_file_size = std::filesystem::file_size(path, error);
    if (error) {
        if (reason) *reason = "failed to size evidence file";
        return false;
    }

    std::ifstream input(path, std::ios::binary);
    if (!input) {
        if (reason) *reason = "failed to open evidence file";
        return false;
    }
    EvidenceStoreFileHeader store_header;
    const bool binary_store = read_evidence_store_file_header(input, &store_header);
    if (binary_store) {
        evidence_append_offset_ = evidence_file_size;
    }
    if (!binary_store) {
        if (reason) *reason = "evidence store format is unsupported; rebuild the database";
        return false;
    }

    if (binary_store && load_evidence_index_sidecar(evidence_file_size)) {
        return true;
    }

    if (!scan_binary_evidence_store(input, evidence_file_size, reason)) {
        return false;
    }
    evidence_index_sidecar_dirty_ = true;
    if (!config_.open.read_only && !save_evidence_index_sidecar(evidence_append_offset_)) {
        evidence_index_sidecar_dirty_ = true;
    }
    prefetch_indexed_evidence_ranges();
    return true;
}

bool LectDatabase::save_evidence() const {
    const auto path = evidence_path(config_.path);
    std::filesystem::create_directories(config_.path);
    if (evidence_append_stream_.is_open()) {
        evidence_append_stream_.flush();
        if (!evidence_append_stream_) {
            return false;
        }
    }
    if (!ensure_binary_evidence_store_file()) {
        return false;
    }
    if (evidence_index_sidecar_dirty_ && !save_evidence_index_sidecar(evidence_append_offset_)) {
        return false;
    }
    return true;
}

bool LectDatabase::ensure_evidence_mapped_file() const {
    if (evidence_append_offset_ == 0 || config_.path.empty()) {
        return false;
    }
    if (evidence_append_stream_.is_open()) {
        evidence_append_stream_.flush();
        if (!evidence_append_stream_) {
            return false;
        }
    }
    if (evidence_mapped_file_ != nullptr && evidence_mapped_file_->is_open() && !evidence_mapping_stale_ &&
        evidence_mapped_file_->size() >= static_cast<std::size_t>(evidence_append_offset_)) {
        return true;
    }

    if (evidence_mapped_file_ == nullptr) {
        evidence_mapped_file_ = std::make_shared<EvidenceMappedFile>();
    }
    if (!evidence_mapped_file_->open_read_only(evidence_path(config_.path))) {
        evidence_mapped_file_.reset();
        return false;
    }
    evidence_mapping_stale_ = false;
    return evidence_mapped_file_->size() >= static_cast<std::size_t>(evidence_append_offset_);
}

std::optional<std::span<const std::byte>> LectDatabase::load_evidence_bytes(std::uint64_t offset,
                                                                            std::uint32_t size) const {
    if (size == 0) {
        return std::span<const std::byte>{};
    }
    const auto end = offset + static_cast<std::uint64_t>(size);
    if (end > evidence_append_offset_) {
        return std::nullopt;
    }

    if (ensure_evidence_mapped_file()) {
        const auto mapped = evidence_mapped_file_->bytes();
        if (end <= mapped.size()) {
            return mapped.subspan(static_cast<std::size_t>(offset), size);
        }
    }

    evidence_read_buffer_.resize(size);
    std::ifstream input(evidence_path(config_.path), std::ios::binary);
    if (!input) {
        evidence_read_buffer_.clear();
        return std::nullopt;
    }
    input.seekg(static_cast<std::streamoff>(offset), std::ios::beg);
    if (!input) {
        evidence_read_buffer_.clear();
        return std::nullopt;
    }
    input.read(reinterpret_cast<char*>(evidence_read_buffer_.data()), static_cast<std::streamsize>(size));
    if (!input) {
        evidence_read_buffer_.clear();
        return std::nullopt;
    }
    return std::span<const std::byte>(evidence_read_buffer_.data(), evidence_read_buffer_.size());
}

void LectDatabase::prefetch_indexed_evidence_ranges() const {
    if (evidence_index_count_ == 0) {
        return;
    }
    if (!ensure_evidence_mapped_file()) {
        return;
    }

    struct EvidenceRange {
        std::uint64_t offset = 0;
        std::uint32_t size = 0;
    };

    std::vector<EvidenceRange> ranges;
    ranges.reserve(evidence_index_count_);
    for (const auto& slot : evidence_index_) {
        if (slot.key.node_id == kInvalidNodeId || slot.entry.unavailable || slot.entry.size == 0) {
            continue;
        }
        ranges.push_back({slot.entry.offset, slot.entry.size});
    }
    if (ranges.empty()) {
        return;
    }

    std::sort(ranges.begin(), ranges.end(), [](const EvidenceRange& lhs, const EvidenceRange& rhs) {
        return lhs.offset < rhs.offset;
    });

    const auto merge_gap = std::max<std::uint64_t>(4096u, config_.page_size_bytes);
    std::uint64_t range_begin = ranges.front().offset;
    std::uint64_t range_end = range_begin + ranges.front().size;
    for (std::size_t index = 1; index < ranges.size(); ++index) {
        const auto entry_begin = ranges[index].offset;
        const auto entry_end = entry_begin + ranges[index].size;
        if (entry_begin <= range_end + merge_gap) {
            range_end = std::max(range_end, entry_end);
            continue;
        }
        evidence_mapped_file_->prefetch(range_begin, range_end - range_begin);
        range_begin = entry_begin;
        range_end = entry_end;
    }
    evidence_mapped_file_->prefetch(range_begin, range_end - range_begin);
}

bool LectDatabase::load_evidence_index_sidecar(std::uint64_t evidence_file_size) {
    const auto path = evidence_index_path(config_.path);
    EvidenceMappedFile mapped_sidecar;
    if (!mapped_sidecar.open_read_only(path)) {
        return false;
    }
    const auto bytes = mapped_sidecar.bytes();
    if (bytes.size() < sizeof(EvidenceIndexSidecarHeader)) {
        return false;
    }

    EvidenceIndexSidecarHeader header;
    std::memcpy(&header, bytes.data(), sizeof(header));
    if (header.magic != kEvidenceIndexSidecarMagic ||
        header.version != kEvidenceIndexSidecarSchemaVersion ||
        header.header_size != sizeof(EvidenceIndexSidecarHeader) ||
        header.evidence_file_size != evidence_file_size) {
        return false;
    }
    const auto expected_count = static_cast<std::size_t>(header.entry_count);
    const auto entries_bytes = expected_count * sizeof(EvidenceIndexSidecarEntry);
    const auto entries_offset = static_cast<std::size_t>(header.header_size);
    const auto expected_path_blob_offset = entries_offset + entries_bytes;
    if (entries_bytes / sizeof(EvidenceIndexSidecarEntry) != expected_count ||
        header.path_blob_offset != expected_path_blob_offset ||
        bytes.size() < expected_path_blob_offset) {
        return false;
    }
    const auto* raw_entry_data = reinterpret_cast<const EvidenceIndexSidecarEntry*>(
        bytes.data() + entries_offset);
    const std::span<const EvidenceIndexSidecarEntry> raw_entries(raw_entry_data, expected_count);
    if (!evidence_sidecar_offsets_sorted(raw_entries)) {
        return false;
    }
    const auto path_blob = bytes.subspan(static_cast<std::size_t>(header.path_blob_offset));

    clear_evidence_index();
    reserve_evidence_index(expected_count);
    for (const auto& raw_entry : raw_entries) {
        EvidenceKey key;
        key.node_id = raw_entry.node_id;
        const auto path_bytes = static_cast<std::size_t>(path_code_storage_bytes(raw_entry.path_word_count));
        if (raw_entry.path_blob_offset > path_blob.size() || path_bytes > path_blob.size() - raw_entry.path_blob_offset) {
            return false;
        }
        const auto path = parse_path_code_blob(
            path_blob.subspan(static_cast<std::size_t>(raw_entry.path_blob_offset), path_bytes),
            raw_entry.path_word_count,
            raw_entry.path_bit_count);
        if (!path) {
            return false;
        }
        key.node_path = *path;
        key.node_path_valid = true;
        key.sector = raw_entry.sector;
        key.channel = static_cast<EvidenceChannel>(raw_entry.channel);
        key.endpoint_source = static_cast<EndpointSource>(raw_entry.endpoint_source);
        key.payload_kind = static_cast<EvidencePayloadKind>(raw_entry.payload_kind);
        if (!normalize_evidence_key(&key)) {
            return false;
        }
        EvidenceIndexEntry entry;
        entry.offset = raw_entry.offset;
        entry.size = raw_entry.size;
        entry.child_hull = (raw_entry.flags & kEvidenceIndexFlagChildHull) != 0;
        entry.unavailable = (raw_entry.flags & kEvidenceIndexFlagUnavailable) != 0;
        entry.generation = raw_entry.generation;
        entry.checksum = raw_entry.checksum;
        upsert_evidence_index(key, entry);
    }
    if (evidence_index_count_ != expected_count) {
        return false;
    }
    if (ensure_evidence_mapped_file()) {
        const auto merge_gap = std::max<std::uint64_t>(4096u, config_.page_size_bytes);
        bool have_range = false;
        std::uint64_t range_begin = 0;
        std::uint64_t range_end = 0;
        for (const auto& raw_entry : raw_entries) {
            if ((raw_entry.flags & kEvidenceIndexFlagUnavailable) != 0 || raw_entry.size == 0) {
                continue;
            }
            const auto entry_begin = raw_entry.offset;
            const auto entry_end = entry_begin + raw_entry.size;
            if (!have_range) {
                range_begin = entry_begin;
                range_end = entry_end;
                have_range = true;
                continue;
            }
            if (entry_begin <= range_end + merge_gap) {
                range_end = std::max(range_end, entry_end);
                continue;
            }
            evidence_mapped_file_->prefetch(range_begin, range_end - range_begin);
            range_begin = entry_begin;
            range_end = entry_end;
        }
        if (have_range) {
            evidence_mapped_file_->prefetch(range_begin, range_end - range_begin);
        }
    }
    evidence_index_sidecar_dirty_ = false;
    return true;
}

bool LectDatabase::scan_binary_evidence_store(std::ifstream& input,
                                              std::uint64_t evidence_file_size,
                                              std::string* reason) {
    EvidenceStoreFileHeader header;
    if (!read_evidence_store_file_header(input, &header)) {
        if (reason) *reason = "evidence store header is malformed";
        return false;
    }

    clear_evidence_index();
    std::uint64_t offset = sizeof(EvidenceStoreFileHeader);
    while (offset < evidence_file_size) {
        EvidenceStoreRecordHeader record_header;
        input.read(reinterpret_cast<char*>(&record_header), static_cast<std::streamsize>(sizeof(record_header)));
        if (!input) {
            if (reason) *reason = "evidence store record header is truncated";
            return false;
        }

        const auto path_bytes = path_code_storage_bytes(record_header.path_word_count);
        const auto payload_bytes = static_cast<std::uint64_t>(record_header.payload_count) * sizeof(std::uint16_t);
        const auto expected_record_size = sizeof(EvidenceStoreRecordHeader) + path_bytes + payload_bytes;
        if (record_header.record_size != expected_record_size ||
            record_header.record_size < sizeof(EvidenceStoreRecordHeader) ||
            offset + record_header.record_size > evidence_file_size) {
            if (reason) *reason = "evidence store record is malformed";
            return false;
        }

        EvidenceKey key;
        key.node_id = record_header.node_id;
        std::vector<std::byte> path_storage(static_cast<std::size_t>(path_bytes));
        if (path_bytes > 0) {
            input.read(reinterpret_cast<char*>(path_storage.data()), static_cast<std::streamsize>(path_bytes));
            if (!input) {
                if (reason) *reason = "evidence store path payload is truncated";
                return false;
            }
        }
        const auto path = parse_path_code_blob(path_storage, record_header.path_word_count, record_header.path_bit_count);
        if (!path) {
            if (reason) *reason = "evidence store path payload is malformed";
            return false;
        }
        key.node_path = *path;
        key.node_path_valid = true;
        key.sector = record_header.sector;
        key.channel = static_cast<EvidenceChannel>(record_header.channel);
        key.endpoint_source = static_cast<EndpointSource>(record_header.endpoint_source);
        key.payload_kind = static_cast<EvidencePayloadKind>(record_header.payload_kind);
        if (!normalize_evidence_key(&key)) {
            if (reason) *reason = "evidence store references an unknown node path";
            return false;
        }

        EvidenceIndexEntry entry;
        entry.offset = offset;
        entry.size = record_header.record_size;
        entry.child_hull = (record_header.flags & kEvidenceIndexFlagChildHull) != 0;
        entry.unavailable = (record_header.flags & kEvidenceIndexFlagUnavailable) != 0;
        entry.generation = record_header.generation;
        entry.checksum = record_header.checksum;
        upsert_evidence_index(key, entry);

        if (payload_bytes > 0) {
            input.seekg(static_cast<std::streamoff>(payload_bytes), std::ios::cur);
            if (!input) {
                if (reason) *reason = "evidence store payload is truncated";
                return false;
            }
        }
        offset += record_header.record_size;
    }
    if (offset != evidence_file_size) {
        if (reason) *reason = "evidence store has trailing bytes";
        return false;
    }
    return true;
}

bool LectDatabase::ensure_binary_evidence_store_file() const {
    if (config_.path.empty()) {
        return false;
    }
    std::filesystem::create_directories(config_.path);
    const auto path = evidence_path(config_.path);
    std::error_code error;
    const bool exists = std::filesystem::exists(path, error);
    if (error) {
        return false;
    }
    const auto file_size = exists ? std::filesystem::file_size(path, error) : 0;
    if (error) {
        return false;
    }
    if (!exists || file_size == 0) {
        std::ofstream out(path, std::ios::binary | std::ios::trunc);
        if (!out || !write_evidence_store_file_header(out)) {
            return false;
        }
        evidence_append_offset_ = sizeof(EvidenceStoreFileHeader);
        return true;
    }

    std::ifstream input(path, std::ios::binary);
    EvidenceStoreFileHeader header;
    if (!input || !read_evidence_store_file_header(input, &header)) {
        return false;
    }
    evidence_append_offset_ = file_size;
    return true;
}

bool LectDatabase::save_evidence_index_sidecar(std::uint64_t evidence_file_size) const {
    if (config_.path.empty()) {
        return false;
    }
    std::filesystem::create_directories(config_.path);
    const auto path = evidence_index_path(config_.path);
    const auto tmp = path.string() + ".tmp";
    std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
    if (!out) {
        return false;
    }
    std::vector<EvidenceIndexSidecarEntry> raw_entries;
    raw_entries.reserve(evidence_index_count_);
    std::vector<std::byte> path_blob;
    for (const auto& slot : evidence_index_) {
        if (slot.key.node_id == kInvalidNodeId) {
            continue;
        }
        EvidenceKey key = slot.key;
        if (!normalize_evidence_key(&key) || !key.node_path_valid || !path_code_storage_valid(key.node_path)) {
            return false;
        }
        EvidenceIndexSidecarEntry raw_entry;
        raw_entry.node_id = key.node_id;
        raw_entry.sector = static_cast<std::int32_t>(key.sector);
        raw_entry.channel = static_cast<std::uint32_t>(key.channel);
        raw_entry.endpoint_source = static_cast<std::uint32_t>(key.endpoint_source);
        raw_entry.payload_kind = static_cast<std::uint32_t>(key.payload_kind);
        raw_entry.flags = (slot.entry.child_hull ? kEvidenceIndexFlagChildHull : 0u) |
                          (slot.entry.unavailable ? kEvidenceIndexFlagUnavailable : 0u);
        raw_entry.size = slot.entry.size;
        raw_entry.path_word_count = path_word_count_for_bits(key.node_path.bit_count);
        raw_entry.path_bit_count = static_cast<std::uint32_t>(key.node_path.bit_count);
        raw_entry.path_blob_offset = path_blob.size();
        raw_entry.offset = slot.entry.offset;
        raw_entry.generation = slot.entry.generation;
        raw_entry.checksum = slot.entry.checksum;
        raw_entries.push_back(raw_entry);
        const auto* path_bytes = reinterpret_cast<const std::byte*>(key.node_path.words.data());
        path_blob.insert(path_blob.end(), path_bytes, path_bytes + path_code_storage_bytes(raw_entry.path_word_count));
    }
    std::sort(raw_entries.begin(), raw_entries.end(), evidence_sidecar_entry_less);
    const EvidenceIndexSidecarHeader header{
        kEvidenceIndexSidecarMagic,
        kEvidenceIndexSidecarSchemaVersion,
        sizeof(EvidenceIndexSidecarHeader),
        evidence_file_size,
        static_cast<std::uint64_t>(raw_entries.size()),
        sizeof(EvidenceIndexSidecarHeader) + raw_entries.size() * sizeof(EvidenceIndexSidecarEntry),
    };
    out.write(reinterpret_cast<const char*>(&header), static_cast<std::streamsize>(sizeof(header)));
    for (const auto& raw_entry : raw_entries) {
        out.write(reinterpret_cast<const char*>(&raw_entry), static_cast<std::streamsize>(sizeof(raw_entry)));
    }
    if (!path_blob.empty()) {
        out.write(reinterpret_cast<const char*>(path_blob.data()), static_cast<std::streamsize>(path_blob.size()));
    }
    out.close();
    if (!static_cast<bool>(out) || !replace_file(tmp, path)) {
        return false;
    }
    evidence_index_sidecar_dirty_ = false;
    return true;
}

void LectDatabase::remember_evidence_metadata(const EvidenceRecord& record) {
    EvidenceIndexEntry entry;
    if (const auto* existing = find_evidence_index(record.key)) {
        entry = *existing;
    }
    entry.child_hull = record.child_hull;
    entry.unavailable = record.unavailable;
    entry.generation = record.generation;
    entry.checksum = record.checksum;
    upsert_evidence_index(record.key, entry);
    evidence_index_sidecar_dirty_ = true;
}

bool LectDatabase::append_evidence_record_to_store(const EvidenceRecord& record) {
    if (config_.open.read_only) {
        return false;
    }
    if (!ensure_evidence_append_stream()) {
        return false;
    }
    const auto header = make_evidence_store_record_header(record);
    if (header.record_size == 0) {
        return false;
    }
    const std::uint64_t offset = evidence_append_offset_;
    evidence_append_stream_.write(reinterpret_cast<const char*>(&header),
                                  static_cast<std::streamsize>(sizeof(header)));
    if (header.path_word_count > 0) {
        evidence_append_stream_.write(reinterpret_cast<const char*>(record.key.node_path.words.data()),
                                      static_cast<std::streamsize>(path_code_storage_bytes(header.path_word_count)));
    }
    if (!record.payload.empty()) {
        std::vector<std::uint16_t> halves(record.payload.size());
        for (std::size_t i = 0; i < record.payload.size(); ++i) {
            halves[i] = f16_from_f32_nearest(record.payload[i]);
        }
        evidence_append_stream_.write(reinterpret_cast<const char*>(halves.data()),
                                      static_cast<std::streamsize>(halves.size() * sizeof(std::uint16_t)));
    }
    if (!evidence_append_stream_) {
        return false;
    }
    EvidenceIndexEntry entry;
    entry.offset = offset;
    entry.size = header.record_size;
    entry.child_hull = record.child_hull;
    entry.unavailable = record.unavailable;
    entry.generation = record.generation;
    entry.checksum = record.checksum;
    upsert_evidence_index(record.key, entry);
    evidence_append_offset_ += header.record_size;
    evidence_mapping_stale_ = true;
    ++evidence_appends_since_flush_;
    evidence_index_sidecar_dirty_ = true;
    return true;
}

std::shared_ptr<const EvidenceRecord> LectDatabase::load_indexed_evidence(const EvidenceKey& key) const {
    EvidenceKey normalized_key = key;
    if (!normalize_evidence_key(&normalized_key)) {
        return {};
    }
    const auto* index_entry = find_evidence_index(normalized_key);
    if (index_entry == nullptr) {
        return {};
    }
    const auto cached = evidence_.find(normalized_key);
    if (cached != evidence_.end()) {
        return cached->second;
    }
    if (index_entry->size == 0) {
        return {};
    }
    const auto bytes_view = load_evidence_bytes(index_entry->offset, index_entry->size);
    if (!bytes_view) {
        return {};
    }
    auto record = parse_binary_evidence_record(*bytes_view);
    if (!record) {
        return {};
    }
    if (!normalize_evidence_key(&record->key)) {
        return {};
    }
    auto shared_record = std::make_shared<EvidenceRecord>(std::move(*record));
    auto [it, inserted] = evidence_.insert_or_assign(shared_record->key, shared_record);
    (void)inserted;
    return it->second;
}

bool LectDatabase::ensure_all_evidence_loaded() const {
    for (const auto& slot : evidence_index_) {
        if (slot.key.node_id == kInvalidNodeId) {
            continue;
        }
        if (evidence_.find(slot.key) == evidence_.end()) {
            if (!load_indexed_evidence(slot.key)) {
                return false;
            }
        }
    }
    return true;
}

bool LectDatabase::ensure_evidence_append_stream() const {
    if (config_.open.read_only || config_.path.empty()) {
        return false;
    }
    if (evidence_append_stream_.is_open()) {
        return static_cast<bool>(evidence_append_stream_);
    }
    if (!ensure_binary_evidence_store_file()) {
        return false;
    }
    evidence_append_stream_.open(evidence_path(config_.path), std::ios::binary | std::ios::app);
    return static_cast<bool>(evidence_append_stream_);
}

void LectDatabase::close_evidence_streams() const {
    if (evidence_append_stream_.is_open()) {
        evidence_append_stream_.flush();
        evidence_append_stream_.close();
    }
    evidence_append_stream_.clear();
    evidence_mapped_file_.reset();
    evidence_mapping_stale_ = false;
    evidence_read_buffer_.clear();
}

bool LectDatabase::flush_incremental_storage() const {
    if (!save_evidence()) {
        return false;
    }
    if (!flush_all_node_pages()) {
        return false;
    }
    if (!save_manifest()) {
        return false;
    }
    evidence_appends_since_flush_ = 0;
    return true;
}

bool LectDatabase::maybe_flush_incremental_storage() const {
    // Prewarm defers the heavy incremental flush (index sidecar + manifest +
    // node pages) to the final checkpoint. Records are already appended to the
    // durable store and the read path flushes the append stream on demand, so
    // skipping the periodic full-sidecar rewrite keeps total store writes
    // O(records) instead of O(records^2 / kEvidenceAppendsPerFlush).
    if (bulk_prewarm_mode_ || streaming_prewarm_mode_) {
        return true;
    }
    if (evidence_appends_since_flush_ < kEvidenceAppendsPerFlush) {
        return true;
    }
    return flush_incremental_storage();
}

void LectDatabase::trim_evidence_cache() const {
    // Streaming prewarm: bound the resident cache to streaming_resident_cap_
    // records. The records are already in the append-only store, so eviction is
    // cheap -- flush the append stream so the bytes are durable, then drop
    // resident copies that have a committed on-disk index entry. The index
    // sidecar is written once at the final checkpoint (not here), so this is
    // O(evicted) with no full-store or sidecar rewrite. Any evicted child record
    // needed by the bottom-up parent sweep is reloaded on demand from the store.
    if (streaming_prewarm_mode_) {
        if (evidence_.size() <= streaming_resident_cap_) {
            return;
        }
        if (evidence_append_stream_.is_open()) {
            evidence_append_stream_.flush();
        }
        for (auto it = evidence_.begin();
             it != evidence_.end() && evidence_.size() > streaming_resident_cap_;) {
            const auto* index_entry = find_evidence_index(it->first);
            if (index_entry != nullptr && index_entry->size > 0) {
                it = evidence_.erase(it);
            } else {
                ++it;
            }
        }
        return;
    }
    // During bulk prewarm keep every materialized record resident so the store
    // is consolidated once at checkpoint instead of being fully rewritten each
    // time the resident set crosses kMaxResidentEvidenceRecords.
    const std::size_t cap = bulk_prewarm_mode_
        ? std::max<std::size_t>(bulk_prewarm_resident_cap_, kMaxResidentEvidenceRecords)
        : kMaxResidentEvidenceRecords;
    if (evidence_.size() <= cap) {
        return;
    }
    save_evidence();
    for (auto it = evidence_.begin(); it != evidence_.end();) {
        const auto* index_entry = find_evidence_index(it->first);
        if (index_entry != nullptr && index_entry->size > 0) {
            it = evidence_.erase(it);
        } else {
            ++it;
        }
    }
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
