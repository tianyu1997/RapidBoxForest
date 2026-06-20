#include <rbf/lect_database/database.h>

#include <algorithm>
#include <sstream>

namespace rbf::lect_database {

BoxKey LectDatabase::make_box_key(std::vector<Interval> intervals) const {
    BoxKey key;
    key.intervals = std::move(intervals);
    key.root_domain_fingerprint = identity_.root_domain_fingerprint;
    key.split_policy_hash = identity_.split_policy_hash;
    key.tolerance = config_.exact_box_tolerance;
    return key;
}

std::optional<NodeRecord> LectDatabase::node(NodeId node_id) const {
    return read_node(node_id);
}

NodeTopology LectDatabase::topology(NodeId node_id) const {
    NodeTopology out;
    const auto item = read_node(node_id);
    if (!item) {
        return out;
    }
    out.id = item->id;
    out.parent = item->parent;
    out.left = item->left;
    out.right = item->right;
    out.depth = item->depth;
    out.split_dim = item->split_dim;
    out.split_value = item->split_value;
    out.leaf = item->is_leaf();
    out.path = item->path;
    out.sibling = sibling(node_id);
    return out;
}

NodeId LectDatabase::parent(NodeId node_id) const {
    const auto item = read_node(node_id);
    return item ? item->parent : kInvalidNodeId;
}

std::pair<NodeId, NodeId> LectDatabase::children(NodeId node_id) const {
    const auto item = read_node(node_id);
    return item ? std::pair<NodeId, NodeId>{item->left, item->right}
                : std::pair<NodeId, NodeId>{kInvalidNodeId, kInvalidNodeId};
}

NodeId LectDatabase::sibling(NodeId node_id) const {
    const auto item = read_node(node_id);
    if (!item) {
        return kInvalidNodeId;
    }
    const NodeId parent_id = item->parent;
    const auto parent_node = read_node(parent_id);
    if (!parent_node) {
        return kInvalidNodeId;
    }
    return parent_node->left == node_id ? parent_node->right : parent_node->left;
}

bool LectDatabase::is_ancestor(NodeId ancestor, NodeId node_id) const {
    const auto a = read_node(ancestor);
    const auto n = read_node(node_id);
    if (!a || !n) {
        return false;
    }
    return a->path.is_prefix_of(n->path);
}

NodeId LectDatabase::lca(NodeId lhs, NodeId rhs) const {
    if (!has_node(lhs) || !has_node(rhs)) {
        return kInvalidNodeId;
    }
    NodeId left = lhs;
    NodeId right = rhs;
    auto left_record = read_node(left);
    auto right_record = read_node(right);
    while (left_record && right_record && left_record->depth > right_record->depth) {
        left = parent(left);
        left_record = read_node(left);
    }
    while (left_record && right_record && right_record->depth > left_record->depth) {
        right = parent(right);
        right_record = read_node(right);
    }
    while (left != right && has_node(left) && has_node(right)) {
        left = parent(left);
        right = parent(right);
    }
    return left == right ? left : kInvalidNodeId;
}

std::vector<NodeId> LectDatabase::node_ids() const {
    return sorted_node_ids();
}

std::vector<NodeId> LectDatabase::layer_nodes(int depth) const {
    const auto it = layer_index_.find(depth);
    return it == layer_index_.end() ? std::vector<NodeId>{} : it->second;
}

std::optional<std::vector<Interval>> LectDatabase::node_box(NodeId node_id) const {
    if (!has_node(node_id)) {
        return std::nullopt;
    }
    return node_box_unchecked(node_id);
}

BoxLookupResult LectDatabase::box_to_node_exact(const BoxKey& box) const {
    BoxLookupResult result;
    if (box.root_domain_fingerprint != identity_.root_domain_fingerprint) {
        result.reason = "root domain fingerprint differs";
        return result;
    }
    if (box.split_policy_hash != identity_.split_policy_hash) {
        result.reason = "split policy hash differs";
        return result;
    }
    if (box.intervals.size() != root_intervals_.size()) {
        result.reason = "dimension count differs";
        return result;
    }
    if (!box_contains(root_intervals_, box.intervals, box.tolerance)) {
        result.reason = "box is outside root domain";
        return result;
    }

    NodeId cursor = root_node();
    auto current_box = root_intervals_;
    while (has_node(cursor)) {
        if (intervals_equal(current_box, box.intervals, box.tolerance)) {
            result.found = true;
            result.node_id = cursor;
            return result;
        }
        const auto current = read_node(cursor);
        if (!current) {
            break;
        }
        if (current->is_leaf()) {
            result.reason = "tree has not split far enough for exact box";
            return result;
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
            out << "box crosses split dimension " << dim << " at " << current->split_value;
            result.reason = out.str();
            return result;
        }
    }
    result.reason = "descended to invalid node id";
    return result;
}

std::vector<NodeId> LectDatabase::range_query(const std::vector<Interval>& box,
                                              RangeQueryMode mode,
                                              LectDatabaseStats* stats) const {
    std::vector<NodeId> out;
    if (box.size() != root_intervals_.size()) {
        return out;
    }
    const auto add_stats = [&]() {
        ++stats_.range_nodes_visited;
        if (stats != nullptr) {
            ++stats->range_nodes_visited;
        }
    };

    if (!valid_node_id(root_node())) {
        return out;
    }

    auto current_box = root_intervals_;
    const auto child_contributes = [&](const std::vector<Interval>& child_box) {
        return mode == RangeQueryMode::Containing
            ? box_contains(child_box, box, config_.exact_box_tolerance)
            : box_overlaps(child_box, box, config_.exact_box_tolerance);
    };

    const auto visit = [&](auto&& self, NodeId node_id) -> void {
        if (!has_node(node_id)) {
            return;
        }
        const auto node_record = read_node(node_id);
        if (!node_record) {
            return;
        }
        add_stats();
        bool keep = false;
        bool descend = false;
        switch (mode) {
        case RangeQueryMode::Containing:
            keep = box_contains(current_box, box, config_.exact_box_tolerance);
            descend = keep;
            break;
        case RangeQueryMode::ContainedBy:
            if (!box_overlaps(current_box, box, config_.exact_box_tolerance)) {
                return;
            }
            keep = box_contains(box, current_box, config_.exact_box_tolerance);
            descend = !node_record->is_leaf();
            break;
        case RangeQueryMode::Intersecting:
            keep = box_overlaps(current_box, box, config_.exact_box_tolerance);
            descend = keep;
            break;
        case RangeQueryMode::CoveringFrontier:
            if (!box_overlaps(current_box, box, config_.exact_box_tolerance)) {
                return;
            }
            keep = node_record->is_leaf();
            descend = !node_record->is_leaf();
            break;
        }
        if (keep) {
            out.push_back(node_record->id);
        }
        if (!descend || node_record->is_leaf()) {
            return;
        }
        const int dim = node_record->split_dim;
        if (dim < 0 || dim >= static_cast<int>(current_box.size())) {
            return;
        }

        if (valid_node_id(node_record->left)) {
            auto& left = current_box[static_cast<std::size_t>(dim)];
            const double saved_hi = left.hi;
            left.hi = node_record->split_value;
            if (child_contributes(current_box)) {
                self(self, node_record->left);
            }
            left.hi = saved_hi;
        }
        if (valid_node_id(node_record->right)) {
            auto& right = current_box[static_cast<std::size_t>(dim)];
            const double saved_lo = right.lo;
            right.lo = node_record->split_value;
            if (child_contributes(current_box)) {
                self(self, node_record->right);
            }
            right.lo = saved_lo;
        }
    };

    visit(visit, root_node());
    return out;
}

}  // namespace rbf::lect_database
