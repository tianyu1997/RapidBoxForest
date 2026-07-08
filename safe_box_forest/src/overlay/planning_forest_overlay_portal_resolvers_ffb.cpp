#include <SBF/safe_box_forest.h>

#include <SBF/box_graph.h>
#include <SBF/oracle.h>
#include <SBF/runtime.h>

#include <algorithm>
#include <functional>
#include <string>
#include <vector>

#include "../query_runtime/planning_forest_query_utils.h"

namespace rbf {

bool RBFPlanningForest::build_ffb_portal_corridor_chain(
    const BoxNode& source_box,
    const BoxNode& target_box,
    const std::vector<Eigen::VectorXd>& waypoint_path,
    const std::vector<Interval>& domain,
    const std::string& prefix,
    int requested_depth,
    int max_internal_boxes,
    int max_recursion_depth,
    double tol,
    bool online_portal,
    BuildProfile* profile,
    StageContext& context,
    std::vector<BoxNode>& internal_boxes,
    int& next_internal_id) {
    BuildProfile* out_profile = profile != nullptr ? profile : &last_build_;
    auto& diagnostics = out_profile->diagnostics;
    FindFreeBoxOptions ffb_options = config_.connector.pave.find_free_box;
    ffb_options.max_depth = std::max(1, std::min(requested_depth, config_.database.max_tree_depth));
    ffb_options.skip_existing_cover_check = true;
    ffb_options.reject_seed_collision = false;
    ffb_options.deadline_ms =
        std::max(0.0, last_adaptive_partition_config_.hipac_portal_ffb_deadline_ms);
    const int max_ffb_calls = online_portal
        ? std::max(0, last_adaptive_partition_config_.hipac_online_max_ffb_calls_per_portal)
        : -1;
    int ffb_calls = 0;
    int ffb_success = 0;
    int ffb_fail = 0;
    int non_adjacent = 0;
    int non_certified = 0;
    int recursion_splits = 0;

    std::function<bool(BoxNode&, const Eigen::VectorXd&, const Eigen::VectorXd&, int)> connect_to_point_ffb;
    connect_to_point_ffb = [&](BoxNode& current,
                               const Eigen::VectorXd& from,
                               const Eigen::VectorXd& to,
                               int depth) -> bool {
        if (current.contains(to, tol)) {
            return true;
        }
        if (target_box.contains(to, tol) && boxes_connected(current, target_box, tol)) {
            return true;
        }
        if (static_cast<int>(internal_boxes.size()) >= max_internal_boxes) {
            diagnostics[prefix + ".portal_corridor_internal_cap_hit"] += 1.0;
            return false;
        }
        if (max_ffb_calls >= 0 && ffb_calls >= max_ffb_calls) {
            diagnostics[prefix + ".portal_corridor_ffb_cap_hit"] += 1.0;
            return false;
        }
        ++ffb_calls;
        FindFreeBoxResult found = find_free_box_in_domain(to, domain, context, ffb_options);
        if (found.found) {
            BoxNode candidate = adaptive_make_box_from_intervals(found.intervals,
                                                                 found.node,
                                                                 next_internal_id--,
                                                                 BoxSafetyStatus::CertifiedFree,
                                                                 false);
            candidate.seed_config = to;
            if (candidate.safety_status != BoxSafetyStatus::CertifiedFree ||
                candidate.strict_audit_required) {
                ++non_certified;
            } else if (boxes_connected(current, candidate, tol)) {
                ++ffb_success;
                internal_boxes.push_back(candidate);
                current = internal_boxes.back();
                return true;
            } else {
                ++non_adjacent;
            }
        } else {
            ++ffb_fail;
        }
        if (depth <= 0 || from.size() != to.size()) {
            return false;
        }
        ++recursion_splits;
        const Eigen::VectorXd midpoint = 0.5 * (from + to);
        if (!connect_to_point_ffb(current, from, midpoint, depth - 1)) {
            return false;
        }
        return connect_to_point_ffb(current, midpoint, to, depth - 1);
    };

    bool chain_ok = true;
    BoxNode current = source_box;
    Eigen::VectorXd previous = source_box.center();
    for (const auto& waypoint : waypoint_path) {
        if (waypoint.size() != previous.size()) {
            chain_ok = false;
            break;
        }
    }
    if (chain_ok) {
        for (std::size_t index = 1; index < waypoint_path.size(); ++index) {
            if (!connect_to_point_ffb(current, previous, waypoint_path[index], max_recursion_depth)) {
                chain_ok = false;
                break;
            }
            previous = waypoint_path[index];
        }
    }
    if (chain_ok && !boxes_connected(current, target_box, tol)) {
        chain_ok = connect_to_point_ffb(current, previous, target_box.center(), max_recursion_depth) &&
                   boxes_connected(current, target_box, tol);
    }

    diagnostics[prefix + ".portal_corridor_ffb_calls"] += static_cast<double>(ffb_calls);
    diagnostics[prefix + ".portal_corridor_ffb_success"] += static_cast<double>(ffb_success);
    diagnostics[prefix + ".portal_corridor_ffb_fail"] += static_cast<double>(ffb_fail);
    diagnostics[prefix + ".portal_corridor_non_adjacent"] += static_cast<double>(non_adjacent);
    diagnostics[prefix + ".portal_corridor_non_certified"] += static_cast<double>(non_certified);
    diagnostics[prefix + ".portal_corridor_recursion_splits"] += static_cast<double>(recursion_splits);
    diagnostics[prefix + ".portal_corridor_internal_boxes"] +=
        static_cast<double>(internal_boxes.size());

    for (const auto& [key, value] : context.diagnostics().snapshot()) {
        diagnostics[prefix + ".portal_corridor_context." + key] += value;
    }
    return chain_ok;
}

}  // namespace rbf
