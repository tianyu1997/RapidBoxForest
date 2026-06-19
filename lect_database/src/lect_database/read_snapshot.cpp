#include <rbf/lect_database/read_snapshot.h>

#include "read_snapshot_builder.h"
#include "read_snapshot_format.h"
#include "read_snapshot_evidence.h"
#include "read_snapshot_mapped_file.h"
#include "read_snapshot_paths.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <filesystem>
#include <limits>
#include <memory>
#include <mutex>
#include <span>
#include <unordered_map>
#include <vector>

namespace rbf::lect_database {
namespace {

struct SnapshotBoxIndexKey {
    std::uint64_t primary = 0;
    std::uint64_t secondary = 0;

    bool operator==(const SnapshotBoxIndexKey& other) const noexcept {
        return primary == other.primary && secondary == other.secondary;
    }
};

struct SnapshotBoxIndexKeyHash {
    std::size_t operator()(const SnapshotBoxIndexKey& key) const noexcept {
        return static_cast<std::size_t>(key.primary ^ (key.secondary + 0x9e3779b97f4a7c15ull + (key.primary << 6u) + (key.primary >> 2u)));
    }
};

SnapshotBoxIndexKey make_snapshot_box_index_key(const std::vector<Interval>& intervals) {
    SnapshotBoxIndexKey key;
    key.primary = fingerprint_intervals(intervals);
    std::uint64_t secondary = 1099511628211ull;
    for (const auto& interval : intervals) {
        secondary = stable_hash_append(secondary, &interval.hi, sizeof(interval.hi));
        secondary = stable_hash_append(secondary, &interval.lo, sizeof(interval.lo));
    }
    key.secondary = secondary;
    return key;
}

bool intervals_equal(const std::vector<Interval>& lhs,
                     const std::vector<Interval>& rhs,
                     double tolerance) {
    if (lhs.size() != rhs.size()) {
        return false;
    }
    for (std::size_t dim = 0; dim < lhs.size(); ++dim) {
        if (std::abs(lhs[dim].lo - rhs[dim].lo) > tolerance ||
            std::abs(lhs[dim].hi - rhs[dim].hi) > tolerance) {
            return false;
        }
    }
    return true;
}

bool interval_contains(const Interval& outer, const Interval& inner, double tolerance) {
    return outer.lo <= inner.lo + tolerance && outer.hi + tolerance >= inner.hi;
}

bool box_contains(const std::vector<Interval>& outer,
                  const std::vector<Interval>& inner,
                  double tolerance) {
    if (outer.size() != inner.size()) {
        return false;
    }
    for (std::size_t dim = 0; dim < outer.size(); ++dim) {
        if (!interval_contains(outer[dim], inner[dim], tolerance)) {
            return false;
        }
    }
    return true;
}

bool box_overlaps(const std::vector<Interval>& lhs,
                  const std::vector<Interval>& rhs,
                  double tolerance) {
    if (lhs.size() != rhs.size()) {
        return false;
    }
    for (std::size_t dim = 0; dim < lhs.size(); ++dim) {
        if (lhs[dim].hi + tolerance < rhs[dim].lo || rhs[dim].hi + tolerance < lhs[dim].lo) {
            return false;
        }
    }
    return true;
}

template <typename T>
std::span<const T> span_after_header(std::span<const std::byte> bytes, std::size_t header_size, std::size_t count) {
    const auto payload_bytes = count * sizeof(T);
    if (payload_bytes / sizeof(T) != count || bytes.size() != header_size + payload_bytes) {
        return {};
    }
    return std::span<const T>(reinterpret_cast<const T*>(bytes.data() + header_size), count);
}

}  // namespace

struct LectReadSnapshot::Impl {
    std::filesystem::path path;
    SnapshotManifestHeader manifest;
    std::vector<Interval> root;
    std::shared_ptr<ReadSnapshotMappedFile> manifest_file;
    std::shared_ptr<ReadSnapshotMappedFile> nodes_file;
    std::shared_ptr<ReadSnapshotMappedFile> direct_evidence_file;
    std::shared_ptr<ReadSnapshotMappedFile> evidence_table_file;
    std::shared_ptr<ReadSnapshotMappedFile> payload_file;
    std::span<const SnapshotNodeRow> nodes;
    std::span<const SnapshotDirectEvidenceEntry> direct_evidence;
    std::span<const SnapshotEvidenceSlot> evidence_slots;
    // Populated lazily on endpoint/box exact lookups; avoids O(node_count) work at open.
    mutable std::unordered_map<SnapshotBoxIndexKey, NodeId, SnapshotBoxIndexKeyHash> exact_box_index;
    mutable std::mutex exact_box_index_mutex;

    bool has_node(NodeId node_id) const noexcept {
        const auto index = static_cast<std::size_t>(node_id);
        return node_id != kInvalidNodeId && index < nodes.size() &&
               (nodes[index].flags & kSnapshotNodePresent) != 0;
    }

    const SnapshotNodeRow* node(NodeId node_id) const noexcept {
        return has_node(node_id) ? &nodes[static_cast<std::size_t>(node_id)] : nullptr;
    }

    static bool valid_cached_node(NodeId node_id) noexcept {
        return node_id != kInvalidNodeId;
    }

    std::optional<NodeId> cached_exact_box_node(const SnapshotBoxIndexKey& key) const {
        std::lock_guard<std::mutex> lock(exact_box_index_mutex);
        const auto found = exact_box_index.find(key);
        if (found == exact_box_index.end() || !valid_cached_node(found->second)) {
            return std::nullopt;
        }
        return found->second;
    }

    void remember_exact_box_node(const SnapshotBoxIndexKey& key, NodeId node_id) const {
        if (!valid_cached_node(node_id)) {
            return;
        }
        std::lock_guard<std::mutex> lock(exact_box_index_mutex);
        auto [it, inserted] = exact_box_index.emplace(key, node_id);
        if (!inserted && it->second != node_id) {
            it->second = kInvalidNodeId;
        }
    }

    std::optional<NodeId> locate_exact_box_node(const std::vector<Interval>& box_intervals,
                                                  double tolerance) const {
        if (!has_node(0) || box_intervals.size() != root.size()) {
            return std::nullopt;
        }
        NodeId cursor = 0;
        auto intervals = root;
        while (has_node(cursor)) {
            if (intervals_equal(intervals, box_intervals, tolerance)) {
                return cursor;
            }
            const auto* row = node(cursor);
            if (row == nullptr || (row->left == kInvalidNodeId && row->right == kInvalidNodeId) ||
                row->split_dim < 0 || row->split_dim >= static_cast<int>(intervals.size())) {
                break;
            }
            const auto dim = static_cast<std::size_t>(row->split_dim);
            if (box_intervals[dim].hi <= row->split_value + tolerance && has_node(row->left)) {
                intervals[dim].hi = row->split_value;
                cursor = row->left;
            } else if (box_intervals[dim].lo + tolerance >= row->split_value && has_node(row->right)) {
                intervals[dim].lo = row->split_value;
                cursor = row->right;
            } else {
                break;
            }
        }
        return std::nullopt;
    }

};

std::optional<EvidenceRecordView> lookup_endpoint_exact_uncached(std::span<const SnapshotNodeRow> nodes,
                                                                 const std::vector<Interval>& root,
                                                                 std::span<const SnapshotDirectEvidenceEntry> direct_evidence,
                                                                 std::span<const SnapshotEvidenceSlot> evidence_slots,
                                                                 const std::shared_ptr<ReadSnapshotMappedFile>& payload_file,
                                                                 const std::vector<Interval>& box_intervals,
                                                                 double tolerance,
                                                                 EvidenceKey key_template) {
    auto has_node = [&](NodeId node_id) {
        const auto index = static_cast<std::size_t>(node_id);
        return node_id != kInvalidNodeId && index < nodes.size() &&
               (nodes[index].flags & kSnapshotNodePresent) != 0;
    };
    auto node = [&](NodeId node_id) -> const SnapshotNodeRow* {
        return has_node(node_id) ? &nodes[static_cast<std::size_t>(node_id)] : nullptr;
    };

    NodeId cursor = 0;
    auto intervals = root;
    while (has_node(cursor)) {
        if (intervals_equal(intervals, box_intervals, tolerance)) {
            key_template.node_id = cursor;
            key_template.node_path = {};
            key_template.node_path_valid = false;
            return lookup_snapshot_evidence_uncached(direct_evidence, evidence_slots, key_template, payload_file);
        }
        const auto* row = node(cursor);
        if (row == nullptr || (row->left == kInvalidNodeId && row->right == kInvalidNodeId) ||
            row->split_dim < 0 || row->split_dim >= static_cast<int>(intervals.size())) {
            break;
        }
        const auto dim = static_cast<std::size_t>(row->split_dim);
        if (box_intervals[dim].hi <= row->split_value + tolerance && has_node(row->left)) {
            intervals[dim].hi = row->split_value;
            cursor = row->left;
        } else if (box_intervals[dim].lo + tolerance >= row->split_value && has_node(row->right)) {
            intervals[dim].lo = row->split_value;
            cursor = row->right;
        } else {
            break;
        }
    }
    return std::nullopt;
}

LectReadSnapshot::LectReadSnapshot() : impl_(std::make_unique<Impl>()) {}
LectReadSnapshot::~LectReadSnapshot() = default;
LectReadSnapshot::LectReadSnapshot(LectReadSnapshot&&) noexcept = default;
LectReadSnapshot& LectReadSnapshot::operator=(LectReadSnapshot&&) noexcept = default;

std::filesystem::path LectReadSnapshot::default_snapshot_path(const std::filesystem::path& legacy_root) {
    return legacy_root / "lect_snapshot";
}

bool LectReadSnapshot::build_from_legacy(const std::filesystem::path& legacy_root,
                                           const std::filesystem::path& snapshot_path,
                                           std::string* reason) {
    return build_read_snapshot_from_legacy(legacy_root, snapshot_path, reason);
}

bool LectReadSnapshot::open(const std::filesystem::path& snapshot_path, std::string* reason) {
    close();
    impl_->path = snapshot_path;
    impl_->manifest_file = std::make_shared<ReadSnapshotMappedFile>();
    impl_->nodes_file = std::make_shared<ReadSnapshotMappedFile>();
    impl_->direct_evidence_file = std::make_shared<ReadSnapshotMappedFile>();
    impl_->evidence_table_file = std::make_shared<ReadSnapshotMappedFile>();
    impl_->payload_file = std::make_shared<ReadSnapshotMappedFile>();

    if (!impl_->manifest_file->open_read_only(snapshot_manifest_path(snapshot_path))) {
        if (reason) *reason = "failed to mmap snapshot manifest";
        close();
        return false;
    }
    const auto manifest_bytes = impl_->manifest_file->bytes();
    if (manifest_bytes.size() < sizeof(SnapshotManifestHeader)) {
        if (reason) *reason = "snapshot manifest is truncated";
        close();
        return false;
    }
    std::memcpy(&impl_->manifest, manifest_bytes.data(), sizeof(SnapshotManifestHeader));
    if (impl_->manifest.magic != kSnapshotManifestMagic || impl_->manifest.version != kSnapshotFormatVersion ||
        impl_->manifest.header_size != sizeof(SnapshotManifestHeader)) {
        if (reason) *reason = "snapshot manifest header is incompatible";
        close();
        return false;
    }
    const auto root_bytes = static_cast<std::size_t>(impl_->manifest.root_dims) * sizeof(Interval);
    if (manifest_bytes.size() != sizeof(SnapshotManifestHeader) + root_bytes) {
        if (reason) *reason = "snapshot manifest root interval payload is malformed";
        close();
        return false;
    }
    impl_->root.assign(reinterpret_cast<const Interval*>(manifest_bytes.data() + sizeof(SnapshotManifestHeader)),
                       reinterpret_cast<const Interval*>(manifest_bytes.data() + sizeof(SnapshotManifestHeader) + root_bytes));

    if (!impl_->nodes_file->open_read_only(snapshot_nodes_path(snapshot_path))) {
        if (reason) *reason = "failed to mmap snapshot nodes";
        close();
        return false;
    }
    const auto node_bytes = impl_->nodes_file->bytes();
    if (node_bytes.size() < sizeof(SnapshotNodesHeader)) {
        if (reason) *reason = "snapshot nodes file is truncated";
        close();
        return false;
    }
    SnapshotNodesHeader nodes_header;
    std::memcpy(&nodes_header, node_bytes.data(), sizeof(nodes_header));
    if (nodes_header.magic != kSnapshotNodesMagic || nodes_header.version != kSnapshotFormatVersion ||
        nodes_header.header_size != sizeof(SnapshotNodesHeader) ||
        nodes_header.node_count != impl_->manifest.node_count ||
        nodes_header.max_node_id != impl_->manifest.max_node_id) {
        if (reason) *reason = "snapshot nodes header is incompatible";
        close();
        return false;
    }
    impl_->nodes = span_after_header<SnapshotNodeRow>(node_bytes, sizeof(SnapshotNodesHeader),
                                                static_cast<std::size_t>(nodes_header.row_count));
    if (impl_->nodes.empty() && nodes_header.row_count != 0) {
        if (reason) *reason = "snapshot nodes rows are malformed";
        close();
        return false;
    }
    if (nodes_header.row_count != 0 && !impl_->has_node(0)) {
        if (reason) *reason = "snapshot root node is missing";
        close();
        return false;
    }

    if (!impl_->direct_evidence_file->open_read_only(snapshot_direct_evidence_path(snapshot_path))) {
        if (reason) *reason = "failed to mmap snapshot direct evidence";
        close();
        return false;
    }
    const auto direct_bytes = impl_->direct_evidence_file->bytes();
    if (direct_bytes.size() < sizeof(SnapshotDirectEvidenceHeader)) {
        if (reason) *reason = "snapshot direct evidence file is truncated";
        close();
        return false;
    }
    SnapshotDirectEvidenceHeader direct_header;
    std::memcpy(&direct_header, direct_bytes.data(), sizeof(direct_header));
    if (direct_header.magic != (kSnapshotEvidenceTableMagic ^ 0x444952454354ull) ||
        direct_header.version != kSnapshotFormatVersion ||
        direct_header.header_size != sizeof(SnapshotDirectEvidenceHeader) ||
        direct_header.row_count != nodes_header.row_count ||
        direct_header.evidence_count != impl_->manifest.evidence_count ||
        direct_header.payload_file_size != impl_->manifest.payload_file_size) {
        if (reason) *reason = "snapshot direct evidence header is incompatible";
        close();
        return false;
    }
    impl_->direct_evidence = span_after_header<SnapshotDirectEvidenceEntry>(direct_bytes,
                                                                      sizeof(SnapshotDirectEvidenceHeader),
                                                                      static_cast<std::size_t>(direct_header.row_count));
    if (impl_->direct_evidence.empty() && direct_header.row_count != 0) {
        if (reason) *reason = "snapshot direct evidence rows are malformed";
        close();
        return false;
    }

    if (!impl_->evidence_table_file->open_read_only(snapshot_evidence_table_path(snapshot_path))) {
        if (reason) *reason = "failed to mmap snapshot evidence table";
        close();
        return false;
    }
    const auto evidence_bytes = impl_->evidence_table_file->bytes();
    if (evidence_bytes.size() < sizeof(SnapshotEvidenceTableHeader)) {
        if (reason) *reason = "snapshot evidence table is truncated";
        close();
        return false;
    }
    SnapshotEvidenceTableHeader evidence_header;
    std::memcpy(&evidence_header, evidence_bytes.data(), sizeof(evidence_header));
    if (evidence_header.magic != kSnapshotEvidenceTableMagic || evidence_header.version != kSnapshotFormatVersion ||
        evidence_header.header_size != sizeof(SnapshotEvidenceTableHeader) ||
        evidence_header.evidence_count != impl_->manifest.evidence_count ||
        evidence_header.payload_file_size != impl_->manifest.payload_file_size ||
        evidence_header.slot_count == 0 || (evidence_header.slot_count & (evidence_header.slot_count - 1u)) != 0) {
        if (reason) *reason = "snapshot evidence table header is incompatible";
        close();
        return false;
    }
    impl_->evidence_slots = span_after_header<SnapshotEvidenceSlot>(evidence_bytes, sizeof(SnapshotEvidenceTableHeader),
                                                              static_cast<std::size_t>(evidence_header.slot_count));
    if (impl_->evidence_slots.empty()) {
        if (reason) *reason = "snapshot evidence table slots are malformed";
        close();
        return false;
    }

    if (!impl_->payload_file->open_read_only(snapshot_payload_path(snapshot_path)) ||
        impl_->payload_file->size() != impl_->manifest.payload_file_size) {
        if (reason) *reason = "failed to mmap snapshot payload";
        close();
        return false;
    }
    return true;
}

void LectReadSnapshot::close() {
    if (!impl_) {
        return;
    }
    impl_->nodes = {};
    impl_->direct_evidence = {};
    impl_->evidence_slots = {};
    impl_->exact_box_index.clear();
    impl_->root.clear();
    impl_->manifest = SnapshotManifestHeader{};
    impl_->manifest_file.reset();
    impl_->nodes_file.reset();
    impl_->direct_evidence_file.reset();
    impl_->evidence_table_file.reset();
    impl_->payload_file.reset();
    impl_->path.clear();
}

bool LectReadSnapshot::is_open() const noexcept {
        return impl_ && impl_->manifest_file && impl_->nodes_file && impl_->direct_evidence_file &&
            impl_->evidence_table_file && impl_->payload_file;
}

std::size_t LectReadSnapshot::node_count() const noexcept {
    return impl_ ? static_cast<std::size_t>(impl_->manifest.node_count) : 0;
}

std::size_t LectReadSnapshot::evidence_count() const noexcept {
    return impl_ ? static_cast<std::size_t>(impl_->manifest.evidence_count) : 0;
}

std::uint64_t LectReadSnapshot::generation() const noexcept {
    return impl_ ? impl_->manifest.generation : 0;
}

const std::vector<Interval>& LectReadSnapshot::root_intervals() const noexcept {
    return impl_->root;
}

BoxKey LectReadSnapshot::make_box_key(std::vector<Interval> intervals) const {
    BoxKey key;
    key.intervals = std::move(intervals);
    key.root_domain_fingerprint = impl_->manifest.root_domain_fingerprint;
    key.split_policy_hash = impl_->manifest.split_policy_hash;
    key.tolerance = 1e-12;
    return key;
}

std::optional<std::vector<Interval>> LectReadSnapshot::node_box(NodeId node_id) const {
    if (!impl_->has_node(node_id)) {
        return std::nullopt;
    }
    if (node_id == 0) {
        return impl_->root;
    }
    std::array<NodeId, 256> inline_lineage;
    std::vector<NodeId> overflow_lineage;
    NodeId cursor = node_id;
    std::size_t lineage_size = 0;
    while (cursor != 0) {
        const auto* row = impl_->node(cursor);
        if (row == nullptr || !impl_->has_node(row->parent)) {
            return std::nullopt;
        }
        if (lineage_size < inline_lineage.size()) {
            inline_lineage[lineage_size] = cursor;
        } else {
            if (overflow_lineage.empty()) {
                overflow_lineage.assign(inline_lineage.begin(), inline_lineage.end());
            }
            overflow_lineage.push_back(cursor);
        }
        ++lineage_size;
        cursor = row->parent;
    }
    auto intervals = impl_->root;
    NodeId parent_id = 0;
    auto child_at = [&](std::size_t reverse_index) {
        const auto index = lineage_size - reverse_index - 1u;
        return overflow_lineage.empty() ? inline_lineage[index] : overflow_lineage[index];
    };
    for (std::size_t index = 0; index < lineage_size; ++index) {
        const NodeId child_id = child_at(index);
        const auto* parent = impl_->node(parent_id);
        if (parent == nullptr || parent->split_dim < 0 ||
            parent->split_dim >= static_cast<int>(intervals.size())) {
            return std::nullopt;
        }
        auto& interval = intervals[static_cast<std::size_t>(parent->split_dim)];
        if (child_id == parent->left) {
            interval.hi = parent->split_value;
        } else if (child_id == parent->right) {
            interval.lo = parent->split_value;
        } else {
            return std::nullopt;
        }
        parent_id = child_id;
    }
    return intervals;
}

BoxLookupResult LectReadSnapshot::box_to_node_exact(const BoxKey& box) const {
    BoxLookupResult result;
    if (box.root_domain_fingerprint != impl_->manifest.root_domain_fingerprint) {
        result.reason = "root domain fingerprint differs";
        return result;
    }
    if (box.split_policy_hash != impl_->manifest.split_policy_hash) {
        result.reason = "split policy hash differs";
        return result;
    }
    if (box.intervals.size() != impl_->root.size()) {
        result.reason = "dimension mismatch";
        return result;
    }
    const auto box_index_key = make_snapshot_box_index_key(box.intervals);
    if (const auto cached = impl_->cached_exact_box_node(box_index_key)) {
        result.found = true;
        result.node_id = *cached;
        return result;
    }
    const auto located = impl_->locate_exact_box_node(box.intervals, box.tolerance);
    if (!located) {
        result.reason = "box does not match a stored node";
        return result;
    }
    impl_->remember_exact_box_node(box_index_key, *located);
    result.found = true;
    result.node_id = *located;
    return result;
}

std::vector<NodeId> LectReadSnapshot::range_query(const std::vector<Interval>& box,
                                                    RangeQueryMode mode,
                                                    LectDatabaseStats* stats) const {
    std::vector<NodeId> out;
    if (box.size() != impl_->root.size() || !impl_->has_node(0)) {
        return out;
    }
    struct StackItem {
        NodeId node_id = kInvalidNodeId;
        std::vector<Interval> intervals;
    };
    std::vector<StackItem> stack;
    stack.push_back({0, impl_->root});
    while (!stack.empty()) {
        auto item = std::move(stack.back());
        stack.pop_back();
        if (stats != nullptr) {
            ++stats->range_nodes_visited;
        }
        const bool match = [&]() {
            switch (mode) {
                case RangeQueryMode::Containing:
                    return box_contains(item.intervals, box, 1e-12);
                case RangeQueryMode::ContainedBy:
                    return box_contains(box, item.intervals, 1e-12);
                case RangeQueryMode::Intersecting:
                case RangeQueryMode::CoveringFrontier:
                    return box_overlaps(item.intervals, box, 1e-12);
            }
            return false;
        }();
        if (!match) {
            continue;
        }
        out.push_back(item.node_id);
        const auto* row = impl_->node(item.node_id);
        if (row == nullptr || row->split_dim < 0 || row->split_dim >= static_cast<int>(item.intervals.size())) {
            continue;
        }
        const auto dim = static_cast<std::size_t>(row->split_dim);
        if (impl_->has_node(row->right)) {
            auto right = item.intervals;
            right[dim].lo = row->split_value;
            stack.push_back({row->right, std::move(right)});
        }
        if (impl_->has_node(row->left)) {
            auto left = std::move(item.intervals);
            left[dim].hi = row->split_value;
            stack.push_back({row->left, std::move(left)});
        }
    }
    return out;
}

std::optional<EvidenceRecordView> LectReadSnapshot::evidence(const EvidenceKey& key) const {
    if (!impl_->has_node(key.node_id)) {
        return std::nullopt;
    }
    return lookup_snapshot_evidence_uncached(impl_->direct_evidence,
                                             impl_->evidence_slots,
                                             key,
                                             impl_->payload_file);
}

std::optional<EvidenceRecordView> LectReadSnapshot::endpoint_for_box_exact(const BoxKey& box,
                                                                             EvidenceKey key_template) const {
    if (box.root_domain_fingerprint != impl_->manifest.root_domain_fingerprint ||
        box.split_policy_hash != impl_->manifest.split_policy_hash ||
        box.intervals.size() != impl_->root.size()) {
        return std::nullopt;
    }
    return endpoint_for_box_exact(box.intervals, key_template, box.tolerance);
}

std::optional<EvidenceRecordView> LectReadSnapshot::endpoint_for_box_exact(const std::vector<Interval>& intervals,
                                                                             EvidenceKey key_template,
                                                                             double tolerance) const {
    if (intervals.size() != impl_->root.size()) {
        return std::nullopt;
    }
    const auto box_index_key = make_snapshot_box_index_key(intervals);
    if (const auto cached = impl_->cached_exact_box_node(box_index_key)) {
        key_template.node_id = *cached;
        key_template.node_path = {};
        key_template.node_path_valid = false;
        if (auto view = lookup_snapshot_evidence_uncached(impl_->direct_evidence,
                                                          impl_->evidence_slots,
                                                          key_template,
                                                          impl_->payload_file)) {
            return view;
        }
    }
    if (const auto located = impl_->locate_exact_box_node(intervals, tolerance)) {
        key_template.node_id = *located;
        key_template.node_path = {};
        key_template.node_path_valid = false;
        if (auto view = lookup_snapshot_evidence_uncached(impl_->direct_evidence,
                                                          impl_->evidence_slots,
                                                          key_template,
                                                          impl_->payload_file)) {
            impl_->remember_exact_box_node(box_index_key, *located);
            return view;
        }
    }
    if (auto view = lookup_endpoint_exact_uncached(impl_->nodes,
                                                   impl_->root,
                                                   impl_->direct_evidence,
                                                   impl_->evidence_slots,
                                                   impl_->payload_file,
                                                   intervals,
                                                   tolerance,
                                                   key_template)) {
        if (valid_node_id(key_template.node_id)) {
            impl_->remember_exact_box_node(box_index_key, key_template.node_id);
        }
        return view;
    }
    return std::nullopt;
}

}  // namespace rbf::lect_database
