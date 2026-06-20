#include "connector_internal.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_set>

namespace rbf {
namespace {

int center_broadphase_dimension(const std::unordered_map<int, const BoxNode*>& map,
                                const std::vector<int>& lhs_ids,
                                const std::vector<int>& rhs_ids) {
    if (lhs_ids.empty() && rhs_ids.empty()) {
        return -1;
    }
    const int first_id = lhs_ids.empty() ? rhs_ids.front() : lhs_ids.front();
    auto first_it = map.find(first_id);
    if (first_it == map.end()) {
        return -1;
    }
    const int nd = first_it->second->n_dims();
    int best_dim = 0;
    double best_span = -1.0;
    for (int dim = 0; dim < nd; ++dim) {
        double lo = std::numeric_limits<double>::infinity();
        double hi = -std::numeric_limits<double>::infinity();
        auto observe = [&](const std::vector<int>& ids) {
            for (int id : ids) {
                auto it = map.find(id);
                if (it == map.end()) {
                    continue;
                }
                const double value = it->second->center()[dim];
                lo = std::min(lo, value);
                hi = std::max(hi, value);
            }
        };
        observe(lhs_ids);
        observe(rhs_ids);
        const double span = hi - lo;
        if (span > best_span) {
            best_span = span;
            best_dim = dim;
        }
    }
    return best_dim;
}

int interval_broadphase_dimension(const std::unordered_map<int, const BoxNode*>& map,
                                  const std::vector<int>& lhs_ids,
                                  const std::vector<int>& rhs_ids) {
    if (lhs_ids.empty() && rhs_ids.empty()) {
        return -1;
    }
    const int first_id = lhs_ids.empty() ? rhs_ids.front() : lhs_ids.front();
    auto first_it = map.find(first_id);
    if (first_it == map.end()) {
        return -1;
    }
    const int nd = first_it->second->n_dims();
    int best_dim = 0;
    double best_span = -1.0;
    for (int dim = 0; dim < nd; ++dim) {
        double lo = std::numeric_limits<double>::infinity();
        double hi = -std::numeric_limits<double>::infinity();
        auto observe = [&](const std::vector<int>& ids) {
            for (int id : ids) {
                auto it = map.find(id);
                if (it == map.end()) {
                    continue;
                }
                const auto& interval = it->second->joint_intervals[static_cast<std::size_t>(dim)];
                lo = std::min(lo, interval.lo);
                hi = std::max(hi, interval.hi);
            }
        };
        observe(lhs_ids);
        observe(rhs_ids);
        const double span = hi - lo;
        if (span > best_span) {
            best_span = span;
            best_dim = dim;
        }
    }
    return best_dim;
}

std::uint64_t pair_key(int lhs, int rhs) {
    const std::uint32_t lo = static_cast<std::uint32_t>(std::min(lhs, rhs));
    const std::uint32_t hi = static_cast<std::uint32_t>(std::max(lhs, rhs));
    return (static_cast<std::uint64_t>(lo) << 32) | static_cast<std::uint64_t>(hi);
}

}  // namespace

std::vector<BridgePairTask> broadphase_bridge_pairs(const std::unordered_map<int, const BoxNode*>& map,
                                                    const std::vector<int>& lhs_ids,
                                                    const std::vector<int>& rhs_ids,
                                                    int result_limit,
                                                    int window_hint) {
    (void)window_hint;
    std::vector<BridgePairTask> out;
    if (lhs_ids.empty() || rhs_ids.empty()) {
        return out;
    }
    const int dim = center_broadphase_dimension(map, lhs_ids, rhs_ids);
    if (dim < 0) {
        return out;
    }
    std::vector<std::pair<double, int>> rhs_sorted;
    rhs_sorted.reserve(rhs_ids.size());
    for (int id : rhs_ids) {
        auto it = map.find(id);
        if (it != map.end()) {
            rhs_sorted.emplace_back(it->second->center()[dim], id);
        }
    }
    std::sort(rhs_sorted.begin(), rhs_sorted.end(), [](const auto& lhs, const auto& rhs) {
        return lhs.first < rhs.first;
    });
    if (rhs_sorted.empty()) {
        return out;
    }
    const int safe_limit = std::max(1, result_limit);
    out.reserve(static_cast<std::size_t>(safe_limit));
    std::unordered_set<std::uint64_t> seen;
    seen.reserve(static_cast<std::size_t>(safe_limit * 16));
    auto trim_to_limit = [&]() {
        if (static_cast<int>(out.size()) <= safe_limit) {
            return;
        }
        std::nth_element(out.begin(),
                         out.begin() + safe_limit,
                         out.end(),
                         [](const BridgePairTask& lhs, const BridgePairTask& rhs) {
                             return lhs.score < rhs.score;
                         });
        out.resize(static_cast<std::size_t>(safe_limit));
    };
    auto current_cutoff = [&]() {
        if (static_cast<int>(out.size()) < safe_limit) {
            return std::numeric_limits<double>::infinity();
        }
        double cutoff = 0.0;
        for (const auto& item : out) {
            cutoff = std::max(cutoff, item.score);
        }
        return cutoff;
    };
    for (int lhs_id : lhs_ids) {
        auto lhs_it = map.find(lhs_id);
        if (lhs_it == map.end()) {
            continue;
        }
        const double center_value = lhs_it->second->center()[dim];
        const double cutoff = current_cutoff();
        auto begin = rhs_sorted.begin();
        auto end = rhs_sorted.end();
        if (std::isfinite(cutoff)) {
            const double radius = std::sqrt(std::max(0.0, cutoff));
            begin = std::lower_bound(rhs_sorted.begin(),
                                     rhs_sorted.end(),
                                     std::make_pair(center_value - radius, std::numeric_limits<int>::min()),
                                     [](const auto& lhs, const auto& rhs) { return lhs.first < rhs.first; });
            end = std::upper_bound(rhs_sorted.begin(),
                                   rhs_sorted.end(),
                                   std::make_pair(center_value + radius, std::numeric_limits<int>::max()),
                                   [](const auto& lhs, const auto& rhs) { return lhs.first < rhs.first; });
        }
        for (auto it = begin; it != end; ++it) {
            const int rhs_id = it->second;
            const auto key = pair_key(lhs_id, rhs_id);
            if (!seen.insert(key).second) {
                continue;
            }
            const BoxNode& lhs = *lhs_it->second;
            const BoxNode& rhs = *map.at(rhs_id);
            out.push_back({static_cast<int>(out.size()), lhs_id, rhs_id, (lhs.center() - rhs.center()).squaredNorm()});
            trim_to_limit();
        }
    }
    std::sort(out.begin(), out.end(), [](const BridgePairTask& lhs, const BridgePairTask& rhs) {
        return lhs.score < rhs.score;
    });
    if (static_cast<int>(out.size()) > safe_limit) {
        out.resize(static_cast<std::size_t>(safe_limit));
    }
    for (int index = 0; index < static_cast<int>(out.size()); ++index) {
        out[static_cast<std::size_t>(index)].task_id = index;
    }
    return out;
}

std::vector<BridgePairTask> interval_gap_broadphase_pairs(const std::unordered_map<int, const BoxNode*>& map,
                                                          const std::vector<int>& lhs_ids,
                                                          const std::vector<int>& rhs_ids,
                                                          double gap_tolerance) {
    std::vector<BridgePairTask> out;
    if (lhs_ids.empty() || rhs_ids.empty()) {
        return out;
    }
    const int dim = interval_broadphase_dimension(map, lhs_ids, rhs_ids);
    if (dim < 0) {
        return out;
    }
    std::vector<int> rhs_sorted = rhs_ids;
    rhs_sorted.erase(std::remove_if(rhs_sorted.begin(), rhs_sorted.end(), [&](int id) {
        return map.find(id) == map.end();
    }), rhs_sorted.end());
    std::sort(rhs_sorted.begin(), rhs_sorted.end(), [&](int lhs, int rhs) {
        return map.at(lhs)->joint_intervals[static_cast<std::size_t>(dim)].lo <
               map.at(rhs)->joint_intervals[static_cast<std::size_t>(dim)].lo;
    });
    std::unordered_set<std::uint64_t> seen;
    seen.reserve(lhs_ids.size() * 4);
    for (int lhs_id : lhs_ids) {
        auto lhs_it = map.find(lhs_id);
        if (lhs_it == map.end()) {
            continue;
        }
        const auto& lhs_interval = lhs_it->second->joint_intervals[static_cast<std::size_t>(dim)];
        const double search_hi = lhs_interval.hi + gap_tolerance;
        for (int rhs_id : rhs_sorted) {
            const auto& rhs_interval = map.at(rhs_id)->joint_intervals[static_cast<std::size_t>(dim)];
            if (rhs_interval.lo > search_hi) {
                break;
            }
            if (rhs_interval.hi + gap_tolerance < lhs_interval.lo) {
                continue;
            }
            const auto key = pair_key(lhs_id, rhs_id);
            if (!seen.insert(key).second) {
                continue;
            }
            const BoxNode& lhs = *lhs_it->second;
            const BoxNode& rhs = *map.at(rhs_id);
            out.push_back({static_cast<int>(out.size()), lhs_id, rhs_id, interval_max_gap(lhs, rhs)});
        }
    }
    std::sort(out.begin(), out.end(), [](const BridgePairTask& lhs, const BridgePairTask& rhs) {
        if (lhs.score == rhs.score) {
            if (lhs.source_box_id == rhs.source_box_id) {
                return lhs.target_box_id < rhs.target_box_id;
            }
            return lhs.source_box_id < rhs.source_box_id;
        }
        return lhs.score < rhs.score;
    });
    for (int index = 0; index < static_cast<int>(out.size()); ++index) {
        out[static_cast<std::size_t>(index)].task_id = index;
    }
    return out;
}

}  // namespace rbf
