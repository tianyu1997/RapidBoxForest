#include <SBF/adaptive_grid_partition.h>
#include <SBF/segment_edge_types.h>

#include <algorithm>

namespace rbf {

void AdaptiveGridPartition::clear_overlay_edges() {
	overlay_edges_by_cell_.clear();
	overlay_edge_ids_.clear();
	reset_overlay_components();
	stats_.overlay_edges = 0;
}

bool AdaptiveGridPartition::append_segment_edge(const SegmentEdge& edge) {
	if (edge.id < 0 || overlay_edge_ids_.find(edge.id) != overlay_edge_ids_.end()) {
		return false;
	}
	const auto source_it = cell_by_box_id_.find(edge.source_box_id);
	const auto target_it = cell_by_box_id_.find(edge.target_box_id);
	if (source_it == cell_by_box_id_.end() || target_it == cell_by_box_id_.end()) {
		return false;
	}
	OverlayEdge forward;
	forward.edge_id = edge.id;
	forward.source_cell = source_it->second;
	forward.target_cell = target_it->second;
	forward.source_box_id = edge.source_box_id;
	forward.target_box_id = edge.target_box_id;
	forward.length = edge.length;
	forward.type = edge.type;
	forward.validation = edge.validation;
	forward.strict_audit_required = edge.strict_audit_required;
	forward.query_index = edge.query_index;
	forward.waypoints = edge.waypoints;
	OverlayEdge reverse = forward;
	reverse.source_cell = target_it->second;
	reverse.target_cell = source_it->second;
	reverse.source_box_id = edge.target_box_id;
	reverse.target_box_id = edge.source_box_id;
	std::reverse(reverse.waypoints.begin(), reverse.waypoints.end());
	overlay_edges_by_cell_[forward.source_cell].push_back(std::move(forward));
	overlay_edges_by_cell_[reverse.source_cell].push_back(std::move(reverse));
	overlay_edge_ids_.insert(edge.id);
	union_overlay_components(source_it->second, target_it->second);
	stats_.overlay_edges = static_cast<int>(overlay_edge_ids_.size());
	return true;
}

int AdaptiveGridPartition::sync_segment_edges(const SegmentEdgeList& edges) {
	int added = 0;
	for (const auto& edge : edges) {
		if (append_segment_edge(edge)) {
			added += 1;
		}
	}
	return added;
}

}  // namespace rbf
