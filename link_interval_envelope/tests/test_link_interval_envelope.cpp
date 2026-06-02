#include <sbf/core/robot.h>
#include <sbf/envelope/envelope_collision.h>
#include <sbf/envelope/endpoint_source.h>
#include <sbf/envelope/ifk_aa_source.h>
#include <sbf/envelope/support_hull.h>
#include <sbf/envelope/envelope_type.h>
#include <link_interval_envelope/api.h>
#include <link_interval_envelope/batch.h>
#include <link_interval_envelope/incremental_context.h>

#include <cassert>
#include <cmath>
#include <string>
#include <type_traits>
#include <vector>

namespace {

void assert_close(const std::vector<float>& a, const std::vector<float>& b) {
    assert(a.size() == b.size());
    for (std::size_t i = 0; i < a.size(); ++i) {
        assert(std::abs(a[i] - b[i]) < 1e-6f);
    }
}

void assert_not_close(const std::vector<float>& a, const std::vector<float>& b) {
    assert(a.size() == b.size());
    bool differs = false;
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (std::abs(a[i] - b[i]) >= 1e-6f) {
            differs = true;
            break;
        }
    }
    assert(differs);
}

}  // namespace

int main() {
    const std::string robot_path = std::string(LIE_EXAMPLE_DATA_DIR) + "/2dof_planar.json";
    const rbf::Robot robot = rbf::Robot::from_json(robot_path);
    const link_interval_envelope::Robot facade_robot =
        link_interval_envelope::Robot::from_json(robot_path);
    static_assert(std::is_same_v<link_interval_envelope::Interval, rbf::Interval>);
    static_assert(std::is_same_v<link_interval_envelope::EndpointSourceConfig, rbf::EndpointSourceConfig>);
    static_assert(std::is_same_v<link_interval_envelope::EnvelopeTypeConfig, rbf::EnvelopeTypeConfig>);
    assert(facade_robot.n_joints() == robot.n_joints());
    assert(robot.n_joints() == 2);
    assert(robot.n_active_links() > 0);

    std::vector<rbf::Interval> intervals = {
        {-0.4, 0.4},
        {-0.2, 0.2},
    };

    rbf::EndpointSourceConfig endpoint_config;
    endpoint_config.source = rbf::EndpointSource::IFK;
    const auto endpoint = rbf::compute_endpoint_iaabb(robot, intervals, endpoint_config);
    assert(endpoint.is_safe);
    assert(endpoint.n_active_links == robot.n_active_links());
    assert(endpoint.endpoint_iaabbs.size() == static_cast<std::size_t>(robot.n_active_links() * 2 * 6));

    rbf::EnvelopeTypeConfig envelope_config;
    envelope_config.type = rbf::EnvelopeType::LinkIAABB;
    envelope_config.n_subdivisions = 4;
    const auto envelope = rbf::compute_link_envelope(
        endpoint.endpoint_iaabbs.data(),
        endpoint.n_active_links,
        robot.active_link_radii(),
        envelope_config);
    assert(envelope.n_subdivisions == 4);
    assert(envelope.link_iaabbs.size() == static_cast<std::size_t>(robot.n_active_links() * 4 * 6));

    envelope_config.type = rbf::EnvelopeType::SupportHull;
    envelope_config.n_subdivisions = 1;
    const auto shape_envelope = rbf::compute_link_envelope(
        endpoint.endpoint_iaabbs.data(),
        endpoint.n_active_links,
        robot.active_link_radii(),
        envelope_config);
    assert(shape_envelope.kdop_n_axes > 0);
    assert(!shape_envelope.kdop_intervals.empty());
    assert(!shape_envelope.support_hulls.empty());
    if (robot.active_link_radii() != nullptr) {
        assert(std::abs(shape_envelope.support_hulls[12] - static_cast<float>(robot.active_link_radii()[0])) < 1e-6f);
    }

    std::vector<float> helper_support_hulls = rbf::compute_support_hulls_from_aabbs({0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f});
    assert(helper_support_hulls.size() == static_cast<std::size_t>(rbf::kSupportHullRecordSize));
    for (int i = 0; i < 6; ++i) {
        assert(std::abs(helper_support_hulls[static_cast<std::size_t>(i)] - (i < 3 ? 0.0f : 1.0f)) < 1e-6f);
        assert(std::abs(helper_support_hulls[static_cast<std::size_t>(i + 6)] - (i < 3 ? 0.0f : 1.0f)) < 1e-6f);
    }
    assert(std::abs(helper_support_hulls[12]) < 1e-6f);

    std::vector<float> synthetic_endpoint_iaabbs = {
        0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f,
        1.0f, 1.0f, 0.0f, 2.0f, 2.0f, 1.0f,
    };
    rbf::EnvelopeTypeConfig synthetic_shape_config;
    synthetic_shape_config.type = rbf::EnvelopeType::SupportHull;
    synthetic_shape_config.n_subdivisions = 1;
    const auto synthetic_shape_envelope = rbf::compute_link_envelope(
        synthetic_endpoint_iaabbs.data(),
        1,
        nullptr,
        synthetic_shape_config);
    assert(synthetic_shape_envelope.support_hulls.size() ==
        static_cast<std::size_t>(rbf::kSupportHullRecordSize));
    assert(synthetic_shape_envelope.kdop_n_axes > 0);
    assert(!synthetic_shape_envelope.kdop_intervals.empty());
    for (int i = 0; i < 12; ++i) {
        assert(std::abs(synthetic_shape_envelope.support_hulls[static_cast<std::size_t>(i)] -
            synthetic_endpoint_iaabbs[static_cast<std::size_t>(i)]) < 1e-6f);
    }
    assert(std::abs(synthetic_shape_envelope.support_hulls[12]) < 1e-6f);

    rbf::EnvelopeTypeConfig synthetic_support_eps_config = synthetic_shape_config;
    synthetic_support_eps_config.support_hull_config.safety_epsilon = 0.5;
    const auto synthetic_support_eps_envelope = rbf::compute_link_envelope(
        synthetic_endpoint_iaabbs.data(),
        1,
        nullptr,
        synthetic_support_eps_config);
    for (int i = 0; i < 12; ++i) {
        assert(std::abs(synthetic_support_eps_envelope.support_hulls[static_cast<std::size_t>(i)] -
            synthetic_endpoint_iaabbs[static_cast<std::size_t>(i)]) < 1e-6f);
    }
    assert(std::abs(synthetic_support_eps_envelope.support_hulls[12]) < 1e-6f);

    const rbf::Obstacle diagonal_gap_obstacle(0.0f, 1.6f, 0.0f, 0.4f, 1.9f, 1.0f);
    rbf::EnvelopeTypeConfig synthetic_aabb_config;
    synthetic_aabb_config.type = rbf::EnvelopeType::LinkIAABB;
    synthetic_aabb_config.n_subdivisions = 1;
    const auto synthetic_aabb_envelope = rbf::compute_link_envelope(
        synthetic_endpoint_iaabbs.data(),
        1,
        nullptr,
        synthetic_aabb_config);
    rbf::EnvelopeCollisionStats aabb_stats;
    const auto aabb_kind = rbf::collide_envelope_aabbs(
        synthetic_aabb_envelope,
        &diagonal_gap_obstacle,
        1,
        {},
        &aabb_stats);
    assert(aabb_kind == rbf::CollisionResultKind::MaybeColliding);
    assert(aabb_stats.maybe_pairs == 1);

    rbf::EnvelopeTypeConfig synthetic_kdop_config;
    synthetic_kdop_config.type = rbf::EnvelopeType::KDOP;
    synthetic_kdop_config.n_subdivisions = 1;
    const auto synthetic_kdop_envelope = rbf::compute_link_envelope(
        synthetic_endpoint_iaabbs.data(),
        1,
        nullptr,
        synthetic_kdop_config);
    const float expected_diag_bound = 1.0f / std::sqrt(2.0f);
    const std::size_t diag_axis_offset = 8;
    assert(std::abs(synthetic_kdop_envelope.kdop_intervals[diag_axis_offset] + expected_diag_bound) < 1e-6f);
    assert(std::abs(synthetic_kdop_envelope.kdop_intervals[diag_axis_offset + 1] - expected_diag_bound) < 1e-6f);
    rbf::EnvelopeCollisionOptions kdop_options;
    kdop_options.mode = rbf::EnvelopeCollisionMode::KDOP;
    rbf::EnvelopeCollisionStats kdop_stats;
    const auto kdop_kind = rbf::collide_envelope_aabbs(
        synthetic_kdop_envelope,
        &diagonal_gap_obstacle,
        1,
        kdop_options,
        &kdop_stats);
    assert(kdop_kind == rbf::CollisionResultKind::DefinitelyFree);
    assert(kdop_stats.kdop_tests == 1);
    assert(kdop_stats.kdop_rejects == 1);
    assert(kdop_stats.maybe_pairs == 0);

    rbf::EnvelopeTypeConfig synthetic_support_config;
    synthetic_support_config.type = rbf::EnvelopeType::SupportHull;
    synthetic_support_config.n_subdivisions = 1;
    const auto synthetic_support_envelope = rbf::compute_link_envelope(
        synthetic_endpoint_iaabbs.data(),
        1,
        nullptr,
        synthetic_support_config);
    rbf::EnvelopeCollisionOptions support_options;
    support_options.mode = rbf::EnvelopeCollisionMode::GJK;
    rbf::EnvelopeCollisionStats support_stats;
    const auto support_kind = rbf::collide_envelope_aabbs(
        synthetic_support_envelope,
        &diagonal_gap_obstacle,
        1,
        support_options,
        &support_stats);
    assert(support_kind == rbf::CollisionResultKind::DefinitelyFree);
    assert(support_stats.gjk_tests == 1);
    assert(support_stats.gjk_rejects == 1);
    assert(support_stats.maybe_pairs == 0);

    const rbf::Obstacle epsilon_only_obstacle(1.2f, 0.2f, 0.2f, 1.3f, 0.3f, 0.3f);
    rbf::EnvelopeCollisionOptions support_eps_options;
    support_eps_options.mode = rbf::EnvelopeCollisionMode::GJK;
    support_eps_options.safety_epsilon = 0.5;
    rbf::EnvelopeCollisionStats support_eps_stats;
    const auto support_eps_kind = rbf::collide_envelope_aabbs(
        synthetic_support_eps_envelope,
        &epsilon_only_obstacle,
        1,
        support_eps_options,
        &support_eps_stats);
    assert(support_eps_kind == rbf::CollisionResultKind::MaybeColliding);
    assert(support_eps_stats.link_aabb_tests == 1);
    assert(support_eps_stats.gjk_tests == 1);
    assert(support_eps_stats.maybe_pairs == 1);

    std::vector<rbf::Interval> parent_intervals = {
        {-0.4, 0.4},
        {-0.2, 0.2},
    };
    std::vector<rbf::Interval> child_intervals = {
        {-0.4, 0.4},
        {-0.1, 0.3},
    };
    const auto ifk_endpoint = rbf::compute_endpoint_iaabb(robot, intervals, endpoint_config);
    assert(ifk_endpoint.is_safe);
    assert(ifk_endpoint.source == rbf::EndpointSource::IFK);
    assert(std::string(rbf::endpoint_source_name(ifk_endpoint.source)) == "IFK");

    rbf::EndpointSourceConfig hifk_config;
    hifk_config.source = rbf::EndpointSource::HIFK;
    hifk_config.hifk_max_depth = 0;
    const auto hifk_endpoint = rbf::compute_endpoint_iaabb(robot, intervals, hifk_config);
    assert(hifk_endpoint.is_safe);
    assert(hifk_endpoint.source == rbf::EndpointSource::HIFK);
    assert(std::string(rbf::endpoint_source_name(hifk_endpoint.source)) == "HIFK");
    assert_close(ifk_endpoint.endpoint_iaabbs, hifk_endpoint.endpoint_iaabbs);

    std::vector<rbf::Interval> narrow_intervals = {
        {-0.025, 0.025},
        {-0.02, 0.02},
    };
    std::vector<rbf::Interval> medium_intervals = {
        {-0.1, 0.1},
        {-0.05, 0.05},
    };
    std::vector<rbf::Interval> wide_intervals = {
        {-0.25, 0.25},
        {-0.25, 0.25},
    };
    std::vector<rbf::Interval> wide_joint0_intervals = {
        {-0.25, 0.25},
        {-0.02, 0.02},
    };
    std::vector<rbf::Interval> wide_joint1_intervals = {
        {-0.02, 0.02},
        {-0.25, 0.25},
    };

    rbf::EndpointSourceConfig hifk_rr_config = hifk_config;
    hifk_rr_config.hifk_max_depth = 1;
    const auto hifk_rr_endpoint = rbf::compute_endpoint_iaabb(robot, wide_intervals, hifk_rr_config);

    rbf::EndpointSourceConfig hifk_fixed_joint0 = hifk_rr_config;
    hifk_fixed_joint0.hifk_split_strategy = rbf::HifkSplitStrategy::FixedDepthSchedule;
    hifk_fixed_joint0.hifk_depth_dimensions = {0};
    const auto hifk_fixed_joint0_endpoint = rbf::compute_endpoint_iaabb(robot, wide_intervals, hifk_fixed_joint0);
    assert_close(hifk_rr_endpoint.endpoint_iaabbs, hifk_fixed_joint0_endpoint.endpoint_iaabbs);

    rbf::EndpointSourceConfig hifk_fixed_joint1 = hifk_rr_config;
    hifk_fixed_joint1.hifk_split_strategy = rbf::HifkSplitStrategy::FixedDepthSchedule;
    hifk_fixed_joint1.hifk_depth_dimensions = {1};
    const auto hifk_fixed_joint1_endpoint = rbf::compute_endpoint_iaabb(robot, wide_intervals, hifk_fixed_joint1);
    assert_not_close(hifk_rr_endpoint.endpoint_iaabbs, hifk_fixed_joint1_endpoint.endpoint_iaabbs);

    assert(rbf::recommend_hifk_depth(robot, narrow_intervals) == 0);
    assert(rbf::recommend_hifk_depth(robot, medium_intervals) == 3);
    assert(rbf::recommend_hifk_depth(robot, wide_intervals) == 5);
    assert(rbf::recommend_hifk_depth(robot, wide_joint0_intervals) == 5);
    assert(rbf::recommend_hifk_depth(robot, wide_joint1_intervals) == 3);

    rbf::EndpointSourceConfig auto_hifk_config = hifk_config;
    auto_hifk_config.hifk_max_depth = -1;
    const auto auto_hifk_endpoint = rbf::compute_endpoint_iaabb(robot, wide_intervals, auto_hifk_config);
    rbf::EndpointSourceConfig explicit_hifk_config = hifk_config;
    explicit_hifk_config.hifk_max_depth = rbf::recommend_hifk_depth(robot, wide_intervals);
    const auto explicit_hifk_endpoint = rbf::compute_endpoint_iaabb(robot, wide_intervals, explicit_hifk_config);
    assert_close(auto_hifk_endpoint.endpoint_iaabbs, explicit_hifk_endpoint.endpoint_iaabbs);

    std::vector<rbf::Interval> sampled_schedule_root = {
        {-1.2, 0.3},
        {-0.9, 0.6},
    };
    const auto default_aafk_schedule = rbf::aafk_volume_min_depth_schedule(robot, sampled_schedule_root, 5);
    const auto single_path_aafk_schedule = rbf::aafk_volume_min_depth_schedule(robot, sampled_schedule_root, 5, 1);
    const auto sampled_aafk_schedule = rbf::aafk_volume_min_depth_schedule(robot, sampled_schedule_root, 5, 4);
    assert((single_path_aafk_schedule == std::vector<int>{0, 1, 0, 0, 1}));
    assert((sampled_aafk_schedule == std::vector<int>{0, 1, 0, 1, 0}));
    assert(single_path_aafk_schedule != sampled_aafk_schedule);
    assert(default_aafk_schedule == sampled_aafk_schedule);

    link_interval_envelope::IncrementalEnvelopeContext context(
        robot, endpoint_config, envelope_config);
    auto first = context.compute(parent_intervals);
    assert(first.endpoint.is_safe);
    assert(!first.used_incremental_fk);
    assert(!first.used_source_incremental_state);
    assert(!context.has_valid_fk());
    auto second = context.compute(child_intervals);
    assert(second.changed_dim == 1);
    assert(!second.used_incremental_fk);
    assert(second.used_source_incremental_state);
    auto full_child_envelope = rbf::compute_link_envelope(
        rbf::compute_endpoint_iaabb(robot, child_intervals, endpoint_config).endpoint_iaabbs.data(),
        robot.n_active_links(),
        robot.active_link_radii(),
        envelope_config);
    const auto full_endpoint = rbf::compute_endpoint_iaabb(robot, child_intervals, endpoint_config);
    assert_close(second.endpoint.endpoint_iaabbs, full_endpoint.endpoint_iaabbs);
    assert_close(second.envelope.link_iaabbs, full_child_envelope.link_iaabbs);

    auto third = context.compute(child_intervals);
    assert(!third.reused_fk);
    assert(third.used_source_incremental_state);
    assert_close(third.endpoint.endpoint_iaabbs, full_endpoint.endpoint_iaabbs);

    rbf::EndpointSourceConfig hifk_incremental_config = hifk_config;
    hifk_incremental_config.hifk_max_depth = 1;
    hifk_incremental_config.hifk_split_strategy = rbf::HifkSplitStrategy::FixedDepthSchedule;
    hifk_incremental_config.hifk_depth_dimensions = {1};
    link_interval_envelope::IncrementalEnvelopeContext hifk_context(
        robot, hifk_incremental_config, envelope_config);
    auto hifk_first = hifk_context.compute(parent_intervals);
    assert(hifk_first.endpoint.is_safe);
    assert(!hifk_first.used_source_incremental_state);
    auto hifk_second = hifk_context.compute(child_intervals);
    assert(hifk_second.changed_dim == 1);
    assert(hifk_second.used_source_incremental_state);
    const auto full_hifk_child_endpoint = rbf::compute_endpoint_iaabb(
        robot, child_intervals, hifk_incremental_config);
    assert_close(hifk_second.endpoint.endpoint_iaabbs,
                 full_hifk_child_endpoint.endpoint_iaabbs);
    auto hifk_third = hifk_context.compute(child_intervals);
    assert(hifk_third.used_source_incremental_state);
    assert_close(hifk_third.endpoint.endpoint_iaabbs,
                 full_hifk_child_endpoint.endpoint_iaabbs);

    rbf::EndpointSourceConfig crit_config;
    crit_config.source = rbf::EndpointSource::CritSample;
    crit_config.n_threads = 1;
    envelope_config.type = rbf::EnvelopeType::LinkIAABB;
    envelope_config.n_subdivisions = 4;

    link_interval_envelope::IncrementalEnvelopeContext crit_context(
        robot, crit_config, envelope_config);
    auto crit_first = crit_context.compute(parent_intervals);
    assert(!crit_first.used_source_incremental_state);
    assert(!crit_first.endpoint.is_safe);
    assert(crit_context.has_valid_fk());
    auto crit_second = crit_context.compute(child_intervals);
    assert(crit_second.used_source_incremental_state);
    assert(crit_second.changed_dim == 1);
    assert(crit_second.endpoint.candidate_dirty_count >= 1);
    assert(crit_second.endpoint.predh_rebuild_count >= 1);
    const auto crit_full_endpoint = rbf::compute_endpoint_iaabb(
        robot, child_intervals, crit_config);
    const auto crit_full_envelope = rbf::compute_link_envelope(
        crit_full_endpoint.endpoint_iaabbs.data(),
        crit_full_endpoint.n_active_links,
        robot.active_link_radii(),
        envelope_config);
    assert_close(crit_second.endpoint.endpoint_iaabbs, crit_full_endpoint.endpoint_iaabbs);
    assert_close(crit_second.envelope.link_iaabbs, crit_full_envelope.link_iaabbs);
    auto crit_third = crit_context.compute(child_intervals);
    assert(crit_third.reused_endpoint_cache);
    assert(crit_third.endpoint.endpoint_cache_reused);
    assert_close(crit_third.endpoint.endpoint_iaabbs, crit_full_endpoint.endpoint_iaabbs);

    const std::string iiwa14_path = std::string(LIE_EXAMPLE_DATA_DIR) + "/iiwa14.json";
    const rbf::Robot iiwa14 = rbf::Robot::from_json(iiwa14_path);
    std::vector<rbf::Interval> old_seq986_intervals = {
        {-0.37085, 0.0},
        {0.6544375, 0.7853249999999999},
        {0.37085, 0.7417},
        {-0.261775, 0.0},
        {0.0, 0.7417},
        {-1.0471, 0.0},
        {0.0, 1.52705},
    };
    std::vector<rbf::Interval> old_node1_intervals = {
        {-2.9668, 2.9668},
        {-2.0942, 2.0942},
        {-2.9668, 2.9668},
        {-2.0942, 2.0942},
        {-2.9668, 2.9668},
        {-2.0942, 2.0942},
        {-3.0541, 0.0},
    };

    link_interval_envelope::IncrementalEnvelopeContext crit_guard_context(
        iiwa14, crit_config, envelope_config);
    auto old_prev = crit_guard_context.compute(old_seq986_intervals, -1);
    assert(!old_prev.used_source_incremental_state);
    auto guarded_node1 = crit_guard_context.compute(old_node1_intervals, 6);
    assert(guarded_node1.changed_dim == -1);
    assert(!guarded_node1.used_source_incremental_state);
    assert(!guarded_node1.reused_endpoint_cache);

    const auto old_node1_full_endpoint = rbf::compute_endpoint_iaabb(
        iiwa14, old_node1_intervals, crit_config);
    const auto old_node1_full_envelope = rbf::compute_link_envelope(
        old_node1_full_endpoint.endpoint_iaabbs.data(),
        old_node1_full_endpoint.n_active_links,
        iiwa14.active_link_radii(),
        envelope_config);
    assert_close(guarded_node1.endpoint.endpoint_iaabbs,
                 old_node1_full_endpoint.endpoint_iaabbs);
    assert_close(guarded_node1.envelope.link_iaabbs,
                 old_node1_full_envelope.link_iaabbs);

    rbf::EndpointSourceConfig crit_parallel_config = crit_config;
    crit_parallel_config.n_threads = 2;
    crit_parallel_config.parallel_min_combos = 1;
    const auto crit_serial_endpoint = rbf::compute_endpoint_iaabb(
        robot, child_intervals, crit_config);
    const auto crit_parallel_endpoint = rbf::compute_endpoint_iaabb(
        robot, child_intervals, crit_parallel_config);
    assert(crit_serial_endpoint.combo_count == crit_parallel_endpoint.combo_count);
    assert(crit_serial_endpoint.combo_count > 0);
    assert(crit_parallel_endpoint.enumerate_threads == 2);
    assert(crit_parallel_endpoint.parallel_min_combos_used == 1);
    assert(crit_parallel_endpoint.enumerate_chunk_count >= 1);
    assert_close(crit_serial_endpoint.endpoint_iaabbs,
                 crit_parallel_endpoint.endpoint_iaabbs);

    std::vector<std::vector<rbf::Interval>> batch_boxes = {
        parent_intervals,
        child_intervals,
    };
    for (const auto source : {rbf::EndpointSource::IFK,
                              rbf::EndpointSource::CritSample,
                              rbf::EndpointSource::HIFK}) {
        rbf::EndpointSourceConfig batch_endpoint_config;
        batch_endpoint_config.source = source;
        if (source == rbf::EndpointSource::HIFK) {
            batch_endpoint_config.hifk_max_depth = 0;
        }
        auto batch_single = link_interval_envelope::compute_envelope_batch(
            robot, batch_boxes, batch_endpoint_config, envelope_config, 1);
        auto batch_parallel = link_interval_envelope::compute_envelope_batch(
            robot, batch_boxes, batch_endpoint_config, envelope_config, 2);
        assert(batch_single.size() == batch_boxes.size());
        assert(batch_parallel.size() == batch_boxes.size());
        for (std::size_t i = 0; i < batch_boxes.size(); ++i) {
            const auto full_batch_endpoint = rbf::compute_endpoint_iaabb(
                robot, batch_boxes[i], batch_endpoint_config);
            assert(batch_single[i].is_safe == full_batch_endpoint.is_safe);
            assert(batch_parallel[i].is_safe == full_batch_endpoint.is_safe);
            const auto full_batch_envelope = rbf::compute_link_envelope(
                full_batch_endpoint.endpoint_iaabbs.data(),
                full_batch_endpoint.n_active_links,
                robot.active_link_radii(),
                envelope_config);
            assert_close(batch_single[i].endpoint_iaabbs,
                         full_batch_endpoint.endpoint_iaabbs);
            assert_close(batch_parallel[i].endpoint_iaabbs,
                         full_batch_endpoint.endpoint_iaabbs);
            assert_close(batch_single[i].envelope.link_iaabbs,
                         full_batch_envelope.link_iaabbs);
            assert_close(batch_parallel[i].envelope.link_iaabbs,
                         full_batch_envelope.link_iaabbs);
        }
    }
    return 0;
}
