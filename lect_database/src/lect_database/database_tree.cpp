#include <rbf/lect_database/database.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace rbf::lect_database {

namespace {

template <typename T>
std::string to_text(const T& value) {
    std::ostringstream out;
    out << std::setprecision(17) << value;
    return out.str();
}

}  // namespace

std::pair<NodeId, NodeId> LectDatabase::split_leaf(NodeId node_id) {
    const auto parent_record = read_node(node_id);
    if (config_.open.read_only || !parent_record) {
        return {kInvalidNodeId, kInvalidNodeId};
    }
    if (!parent_record->is_leaf()) {
        return {parent_record->left, parent_record->right};
    }
    const auto intervals = node_box_unchecked(node_id);
    const int parent_depth = parent_record->depth;
    const int split_dim = split_policy_.choose_dimension(root_intervals_, intervals, parent_depth);
    if (split_dim < 0 || split_dim >= static_cast<int>(intervals.size())) {
        return {kInvalidNodeId, kInvalidNodeId};
    }
    const double split_value = split_policy_.choose_split_value(intervals[static_cast<std::size_t>(split_dim)]);
    return split_leaf(node_id, split_dim, split_value);
}

std::pair<NodeId, NodeId> LectDatabase::split_leaf(NodeId node_id, int split_dim, double split_value) {
    const auto parent_record = read_node(node_id);
    if (config_.open.read_only || !parent_record) {
        return {kInvalidNodeId, kInvalidNodeId};
    }
    if (!parent_record->is_leaf()) {
        return {parent_record->left, parent_record->right};
    }
    const auto intervals = node_box_unchecked(node_id);
    const int parent_depth = parent_record->depth;
    const PathCode parent_path = parent_record->path;
    if (parent_depth + 1 >= config_.max_tree_depth) {
        return {kInvalidNodeId, kInvalidNodeId};
    }
    if (split_dim < 0 || split_dim >= static_cast<int>(intervals.size())) {
        return {kInvalidNodeId, kInvalidNodeId};
    }
    if (split_value <= intervals[static_cast<std::size_t>(split_dim)].lo ||
        split_value >= intervals[static_cast<std::size_t>(split_dim)].hi) {
        return {kInvalidNodeId, kInvalidNodeId};
    }

    ++generation_;
    pending_changes_ = true;
    PathCode left_path = parent_path;
    PathCode right_path = parent_path;
    left_path.push_child(false);
    right_path.push_child(true);

    const NodeId left = allocate_node_id();
    const NodeId right = allocate_node_id();
    if (!valid_node_id(left) || !valid_node_id(right) || left == right) {
        return {kInvalidNodeId, kInvalidNodeId};
    }

    LectDbTransaction transaction;
    transaction.generation = generation_;
    transaction.records.push_back("split|" + std::to_string(node_id) + "|" + std::to_string(left) + "|" +
                                  std::to_string(right) + "|" + std::to_string(split_dim) + "|" +
                                  to_text(split_value));
    transaction.committed = true;
    append_committed_transaction(transaction);

    NodeRecord left_record;
    left_record.id = left;
    left_record.parent = node_id;
    left_record.depth = parent_depth + 1;
    left_record.path = std::move(left_path);
    left_record.generation = generation_;
    left_record.dirty = true;
    write_node_record(std::move(left_record));

    NodeRecord right_record;
    right_record.id = right;
    right_record.parent = node_id;
    right_record.depth = parent_depth + 1;
    right_record.path = std::move(right_path);
    right_record.generation = generation_;
    right_record.dirty = true;
    write_node_record(std::move(right_record));

    if (auto* item = mutable_node(node_id)) {
        item->left = left;
        item->right = right;
        item->split_dim = split_dim;
        item->split_value = split_value;
        item->generation = generation_;
        item->dirty = true;
    }
    layer_index_[parent_depth + 1].push_back(left);
    layer_index_[parent_depth + 1].push_back(right);
    return {left, right};
}

bool LectDatabase::ensure_depth(int target_depth) {
    if (target_depth < 0) {
        return false;
    }
    if (target_depth >= config_.max_tree_depth) {
        return false;
    }
    bool changed = false;
    for (int depth = config_.root_depth; depth < target_depth; ++depth) {
        const auto layer = layer_nodes(depth);
        for (NodeId id : layer) {
            const auto node_record = read_node(id);
            if (node_record && node_record->is_leaf()) {
                const auto children_pair = split_leaf(id);
                if (!valid_node_id(children_pair.first) || !valid_node_id(children_pair.second)) {
                    return false;
                }
                changed = true;
            }
        }
    }
    if (changed && !config_.open.read_only) {
        return flush_all_node_pages() && save_manifest();
    }
    return true;
}

BoxLookupResult LectDatabase::split_to_box(const BoxKey& box, int max_depth) {
    BoxLookupResult result;
    if (config_.open.read_only) {
        result.reason = "database is read-only";
        return result;
    }
    if (box.root_domain_fingerprint != identity_.root_domain_fingerprint ||
        box.split_policy_hash != identity_.split_policy_hash) {
        return box_to_node_exact(box);
    }
    if (!box_contains(root_intervals_, box.intervals, box.tolerance)) {
        result.reason = "box is outside root domain";
        return result;
    }

    NodeId cursor = root_node();
    auto current_box = root_intervals_;
    bool changed = false;
    while (has_node(cursor)) {
        if (intervals_equal(current_box, box.intervals, box.tolerance)) {
            if (changed && (!flush_all_node_pages() || !save_manifest())) {
                result.reason = "exact box was split but persistence failed";
                return result;
            }
            result.found = true;
            result.node_id = cursor;
            return result;
        }
        auto current = read_node(cursor);
        if (!current) {
            break;
        }
        const int effective_max_depth = std::min(max_depth, config_.max_tree_depth - 1);
        if (current->depth >= effective_max_depth) {
            result.reason = "max split depth reached before exact box";
            return result;
        }
        if (current->is_leaf()) {
            const auto children_pair = split_leaf(cursor);
            if (!valid_node_id(children_pair.first) || !valid_node_id(children_pair.second)) {
                result.reason = "leaf could not be split toward exact box";
                return result;
            }
            changed = true;
            current = read_node(cursor);
            if (!current) {
                break;
            }
        }
        const int dim = current->split_dim;
        if (dim < 0 || dim >= static_cast<int>(box.intervals.size())) {
            result.reason = "stored split dimension is invalid";
            return result;
        }
        const auto& target = box.intervals[static_cast<std::size_t>(dim)];
        if (target.hi <= current->split_value + box.tolerance) {
            current_box[static_cast<std::size_t>(dim)].hi = current->split_value;
            cursor = current->left;
        } else if (target.lo >= current->split_value - box.tolerance) {
            current_box[static_cast<std::size_t>(dim)].lo = current->split_value;
            cursor = current->right;
        } else {
            std::ostringstream out;
            out << "box is not representable: crosses split dimension " << dim << " at " << current->split_value;
            result.reason = out.str();
            return result;
        }
    }
    result.reason = "descended to invalid node id";
    return result;
}


NodeId LectDatabase::append_child(NodeId parent_id, bool right_child, int depth, PathCode path) {
    const NodeId child_id = allocate_node_id();
    if (!valid_node_id(child_id)) {
        return kInvalidNodeId;
    }
    NodeRecord child;
    child.id = child_id;
    child.parent = parent_id;
    child.depth = depth;
    child.path = std::move(path);
    child.generation = generation_;
    child.dirty = true;
    write_node_record(std::move(child));
    if (auto* parent_node = mutable_node(parent_id)) {
        if (right_child) {
            parent_node->right = child_id;
        } else {
            parent_node->left = child_id;
        }
        parent_node->dirty = true;
    }
    return child_id;
}

NodeId LectDatabase::allocate_node_id() {
    while (valid_node_id(next_node_id_) && has_node(next_node_id_)) {
        if (next_node_id_ == kInvalidNodeId - 1) {
            next_node_id_ = kInvalidNodeId;
            break;
        }
        ++next_node_id_;
    }
    if (!valid_node_id(next_node_id_)) {
        return kInvalidNodeId;
    }
    const NodeId allocated = next_node_id_;
    if (next_node_id_ == kInvalidNodeId - 1) {
        next_node_id_ = kInvalidNodeId;
    } else {
        ++next_node_id_;
    }
    return allocated;
}


std::vector<Interval> LectDatabase::node_box_unchecked(NodeId node_id) const {
    const auto node_record = read_node(node_id);
    if (!node_record) {
        return root_intervals_;
    }
    if (node_id == root_node()) {
        return root_intervals_;
    }

    std::vector<Interval> intervals = root_intervals_;
    std::vector<NodeId> lineage;
    lineage.reserve(static_cast<std::size_t>(std::max(0, node_record->depth)));

    NodeId cursor = node_id;
    while (valid_node_id(cursor) && cursor != root_node()) {
        const auto current = read_node(cursor);
        if (!current || !valid_node_id(current->parent)) {
            return node_box_from_path(node_record->path);
        }
        lineage.push_back(cursor);
        cursor = current->parent;
    }
    std::reverse(lineage.begin(), lineage.end());

    NodeId parent_id = root_node();
    for (NodeId child_id : lineage) {
        const auto parent = read_node(parent_id);
        if (!parent || parent->split_dim < 0 ||
            parent->split_dim >= static_cast<int>(intervals.size())) {
            return node_box_from_path(node_record->path);
        }
        if (child_id == parent->left) {
            intervals[static_cast<std::size_t>(parent->split_dim)].hi = parent->split_value;
        } else if (child_id == parent->right) {
            intervals[static_cast<std::size_t>(parent->split_dim)].lo = parent->split_value;
        } else {
            return node_box_from_path(node_record->path);
        }
        parent_id = child_id;
    }
    return intervals;
}

std::vector<Interval> LectDatabase::node_box_from_path(const PathCode& path) const {
    auto intervals = root_intervals_;
    for (int depth = 0; depth < path.bit_count; ++depth) {
        const int dim = split_policy_.choose_dimension(root_intervals_, intervals, config_.root_depth + depth);
        if (dim < 0 || dim >= static_cast<int>(intervals.size())) {
            return intervals;
        }
        const double split_value = split_policy_.choose_split_value(intervals[static_cast<std::size_t>(dim)]);
        if (path.bit(depth)) {
            intervals[static_cast<std::size_t>(dim)].lo = split_value;
        } else {
            intervals[static_cast<std::size_t>(dim)].hi = split_value;
        }
    }
    return intervals;
}

bool LectDatabase::intervals_equal(const std::vector<Interval>& lhs,
                                   const std::vector<Interval>& rhs,
                                   double tolerance) const {
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

bool LectDatabase::interval_contains(const Interval& outer, const Interval& inner, double tolerance) const {
    return outer.lo <= inner.lo + tolerance && outer.hi >= inner.hi - tolerance;
}

bool LectDatabase::box_contains(const std::vector<Interval>& outer,
                                const std::vector<Interval>& inner,
                                double tolerance) const {
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

bool LectDatabase::box_overlaps(const std::vector<Interval>& lhs,
                                const std::vector<Interval>& rhs,
                                double tolerance) const {
    if (lhs.size() != rhs.size()) {
        return false;
    }
    for (std::size_t dim = 0; dim < lhs.size(); ++dim) {
        if (!lhs[dim].overlaps(rhs[dim], tolerance)) {
            return false;
        }
    }
    return true;
}

void LectDatabase::rebuild_layer_index() {
    layer_index_.clear();
    for (NodeId node_id : sorted_node_ids()) {
        const auto node_record = read_node(node_id);
        if (node_record) {
            layer_index_[node_record->depth].push_back(node_record->id);
        }
    }
}

void LectDatabase::assign_page_ids() {
    for (NodeId node_id : sorted_node_ids()) {
        auto node_record = read_node(node_id);
        if (!node_record) {
            continue;
        }
        const auto expected_page_id = page_id_for_node(node_id);
        if (node_record->page_id != expected_page_id) {
            if (auto* item = mutable_node(node_id)) {
                item->page_id = expected_page_id;
                item->dirty = true;
            }
        }
    }
}


}  // namespace rbf::lect_database
