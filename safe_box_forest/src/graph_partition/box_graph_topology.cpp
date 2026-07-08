#include <SBF/box_graph.h>

#include <algorithm>
#include <functional>
#include <queue>
#include <unordered_map>
#include <unordered_set>

namespace rbf {

std::vector<std::vector<int>> find_islands(const AdjacencyGraph& graph) {
    std::unordered_set<int> unseen;
    for (const auto& [id, _] : graph) {
        unseen.insert(id);
    }
    std::vector<std::vector<int>> islands;
    while (!unseen.empty()) {
        const int root = *unseen.begin();
        unseen.erase(root);
        islands.push_back({});
        std::queue<int> queue;
        queue.push(root);
        while (!queue.empty()) {
            const int current = queue.front();
            queue.pop();
            islands.back().push_back(current);
            auto it = graph.find(current);
            if (it == graph.end()) {
                continue;
            }
            for (int next : it->second) {
                if (unseen.erase(next) > 0) {
                    queue.push(next);
                }
            }
        }
    }
    return islands;
}

std::unordered_set<int> find_articulation_points(const AdjacencyGraph& graph) {
    std::unordered_map<int, int> disc;
    std::unordered_map<int, int> low;
    std::unordered_set<int> points;
    int time = 0;

    std::function<void(int, int)> dfs = [&](int u, int parent) {
        disc[u] = low[u] = ++time;
        int children = 0;
        auto it = graph.find(u);
        if (it == graph.end()) {
            return;
        }
        for (int v : it->second) {
            if (v == parent) {
                continue;
            }
            if (disc.find(v) == disc.end()) {
                children += 1;
                dfs(v, u);
                low[u] = std::min(low[u], low[v]);
                if (parent != -1 && low[v] >= disc[u]) {
                    points.insert(u);
                }
            } else {
                low[u] = std::min(low[u], disc[v]);
            }
        }
        if (parent == -1 && children > 1) {
            points.insert(u);
        }
    };

    for (const auto& [id, _] : graph) {
        if (disc.find(id) == disc.end()) {
            dfs(id, -1);
        }
    }
    return points;
}

} // namespace rbf
