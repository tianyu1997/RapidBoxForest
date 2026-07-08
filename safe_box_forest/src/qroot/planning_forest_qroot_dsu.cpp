#include "planning_forest_qroot_helpers.h"

#include <unordered_set>
#include <utility>

namespace rbf {

void BuildDisjointSet::add(int id) {
    if (parent.find(id) == parent.end()) {
        parent[id] = id;
        rank[id] = 0;
    }
}

int BuildDisjointSet::find(int id) {
    add(id);
    int p = parent[id];
    if (p != id) {
        p = find(p);
        parent[id] = p;
    }
    return p;
}

void BuildDisjointSet::unite(int lhs, int rhs) {
    int left = find(lhs);
    int right = find(rhs);
    if (left == right) {
        return;
    }
    if (rank[left] < rank[right]) {
        std::swap(left, right);
    }
    parent[right] = left;
    if (rank[left] == rank[right]) {
        rank[left] += 1;
    }
}

bool BuildDisjointSet::connected(int lhs, int rhs) {
    return find(lhs) == find(rhs);
}

int BuildDisjointSet::island_count() {
    std::unordered_set<int> roots;
    roots.reserve(parent.size());
    for (const auto& [id, _] : parent) {
        roots.insert(find(id));
    }
    return static_cast<int>(roots.size());
}

BuildDisjointSet make_dsu_from_graph(const std::vector<BoxNode>& boxes,
                                     const AdjacencyGraph& graph) {
    BuildDisjointSet dsu;
    for (const auto& box : boxes) {
        dsu.add(box.id);
    }
    for (const auto& [id, neighbors] : graph) {
        dsu.add(id);
        for (int neighbor : neighbors) {
            dsu.unite(id, neighbor);
        }
    }
    return dsu;
}

}  // namespace rbf
