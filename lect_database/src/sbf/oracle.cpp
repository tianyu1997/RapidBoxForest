#include <LECTDatabase/sbf/oracle.h>

#include <sbf/core/joint_symmetry.h>
#include <sbf/envelope/endpoint_source.h>
#include <sbf/envelope/ifk_aa_source.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <system_error>

namespace rbf {
namespace {

using lect_database::NodeId;
using Clock = std::chrono::steady_clock;

double elapsed_us(Clock::time_point start) {
    return std::chrono::duration<double, std::micro>(Clock::now() - start).count();
}

std::uint64_t make_envelope_cache_key(lect_database::NodeId node_id, int sector) {
    std::uint64_t key = static_cast<std::uint64_t>(node_id);
    const std::uint64_t sector_bits = static_cast<std::uint64_t>(static_cast<std::uint32_t>(sector));
    key ^= sector_bits + 0x9e3779b97f4a7c15ull + (key << 6) + (key >> 2);
    return key;
}

std::uint64_t next_session_id() {
    static std::atomic<std::uint64_t> counter{0};
    return counter.fetch_add(1, std::memory_order_relaxed);
}

void record_envelope_collision(OracleCounters& counters, const EnvelopeCollisionStats& stats) {
    counters.envelope_collision_envelope_aabb_tests += stats.envelope_aabb_tests;
    counters.envelope_collision_envelope_aabb_rejects += stats.envelope_aabb_rejects;
    counters.envelope_collision_link_union_aabb_tests += stats.link_union_aabb_tests;
    counters.envelope_collision_link_union_aabb_rejects += stats.link_union_aabb_rejects;
    counters.envelope_collision_link_aabb_tests += stats.link_aabb_tests;
    counters.envelope_collision_link_aabb_rejects += stats.link_aabb_rejects;
    counters.envelope_collision_kdop_tests += stats.kdop_tests;
    counters.envelope_collision_kdop_rejects += stats.kdop_rejects;
    counters.envelope_collision_kdop_axes_tested += stats.kdop_axes_tested;
    counters.envelope_collision_gjk_tests += stats.gjk_tests;
    counters.envelope_collision_gjk_rejects += stats.gjk_rejects;
    counters.envelope_collision_gjk_iterations += stats.gjk_iterations;
}

lect_database::EvidenceChannel database_channel_for_endpoint(EndpointSource source) {
    return source_channel(source) == 0 ? lect_database::EvidenceChannel::Safe
                                       : lect_database::EvidenceChannel::Rapid;
}

EndpointSourceConfig hifk_config_for_materialization(const DatabaseBoxOracle& oracle,
                                                     OracleNodeId node,
                                                     EndpointSourceConfig config) {
    if (config.source != EndpointSource::HIFK) {
        return config;
    }

    const auto& split_policy = oracle.database().split_policy_descriptor();
    config.hifk_depth_offset = oracle.depth(node);
    config.hifk_min_split_width = split_policy.min_width;
    config.hifk_depth_dimensions.clear();
    config.hifk_root_intervals.clear();

    switch (split_policy.strategy) {
    case lect_database::SplitStrategy::RoundRobin:
        config.hifk_split_strategy = HifkSplitStrategy::RoundRobin;
        break;
    case lect_database::SplitStrategy::WidestRoot:
        config.hifk_split_strategy = HifkSplitStrategy::WidestRoot;
        config.hifk_root_intervals = oracle.root_intervals();
        break;
    case lect_database::SplitStrategy::AAFKVolumeMin:
        config.hifk_split_strategy = HifkSplitStrategy::FixedDepthSchedule;
        config.hifk_depth_dimensions = split_policy.depth_dimensions;
        break;
    }

    return config;
}

rbf::lect_database::LectDatabaseConfig make_worker_database_config(const DatabaseBoxOracle& master,
                                                                   const std::filesystem::path& path,
                                                                   const std::vector<Interval>& root_intervals) {
    lect_database::LectDatabaseConfig config;
    config.path = path;
    config.root_intervals = root_intervals;
    config.split_policy = master.database().split_policy_descriptor();
    config.identity = master.database().identity();
    config.identity.root_domain_fingerprint = lect_database::fingerprint_intervals(root_intervals);
    config.open.read_only = false;
    config.open.create_if_missing = true;
    config.open.verify_identity = true;
    config.open.replay_journal = true;
    return config;
}

lect_database::NodeId to_database_node(OracleNodeId node) {
    if (node < 0 || static_cast<std::uint64_t>(node) > std::numeric_limits<lect_database::NodeId>::max()) {
        return lect_database::kInvalidNodeId;
    }
    return static_cast<lect_database::NodeId>(node);
}

OracleNodeId from_database_node(lect_database::NodeId node) {
    return lect_database::valid_node_id(node) ? static_cast<OracleNodeId>(node) : kInvalidOracleNodeId;
}

OracleNodeId remap_lookup(const std::unordered_map<OracleNodeId, OracleNodeId>& node_remap,
                          OracleNodeId worker_node) {
    const auto it = node_remap.find(worker_node);
    return it == node_remap.end() ? kInvalidOracleNodeId : it->second;
}

bool database_uses_canonical_symmetry(const lect_database::LectDatabase& database) {
    const auto& identity = database.identity();
    return identity.canonical_mode && lect_database::uses_joint_symmetry_native(identity.symmetry_descriptor);
}

std::optional<JointSymmetry> primary_database_symmetry(const Robot& robot,
                                                       const lect_database::LectDatabase& database) {
    if (!database_uses_canonical_symmetry(database)) {
        return std::nullopt;
    }
    auto symmetries = detect_joint_symmetries(robot);
    if (symmetries.empty()) {
        return std::nullopt;
    }
    JointSymmetry symmetry = symmetries.front();
    if (symmetry.type == JointSymmetryType::NONE || symmetry.period <= 0.0 || symmetry.joint_index < 0) {
        return std::nullopt;
    }
    return symmetry;
}

int normalize_sector(lect_database::SectorId sector) {
    return ((static_cast<int>(sector) % 4) + 4) % 4;
}

double normalized_joint_value(double value, double origin) {
    double normalized = std::fmod(value - origin, TWO_PI);
    if (normalized < 0.0) {
        normalized += TWO_PI;
    }
    return normalized + origin;
}

double canonical_value_in_sector(const JointSymmetry& symmetry,
                                 double value,
                                 lect_database::SectorId sector) {
    return normalized_joint_value(value, symmetry.canonical_lo) -
        static_cast<double>(normalize_sector(sector)) * symmetry.period;
}

Interval canonical_interval_for_sector(const JointSymmetry& symmetry,
                                       const Interval& interval,
                                       lect_database::SectorId sector) {
    const double lo = canonical_value_in_sector(symmetry, interval.lo, sector);
    const double hi = canonical_value_in_sector(symmetry, interval.hi, sector);
    Interval canonical{std::min(lo, hi), std::max(lo, hi)};
    canonical.lo = std::max(canonical.lo, symmetry.canonical_lo);
    canonical.hi = std::min(canonical.hi, symmetry.canonical_hi);
    return canonical;
}

Interval map_canonical_interval_to_sector(const JointSymmetry& symmetry,
                                          const Interval& canonical,
                                          lect_database::SectorId sector,
                                          const Interval& limit,
                                          double reference_value) {
    double lo = 0.0;
    double hi = 0.0;
    symmetry.map_interval(canonical.lo, canonical.hi, normalize_sector(sector), lo, hi);
    while (hi > limit.hi + 1e-12) {
        lo -= TWO_PI;
        hi -= TWO_PI;
    }
    while (lo < limit.lo - 1e-12) {
        lo += TWO_PI;
        hi += TWO_PI;
    }
    const double center = 0.5 * (lo + hi);
    const double lower_shift_center = center - TWO_PI;
    const double upper_shift_center = center + TWO_PI;
    if (std::abs(reference_value - lower_shift_center) < std::abs(reference_value - center) &&
        lo - TWO_PI >= limit.lo - 1e-12) {
        lo -= TWO_PI;
        hi -= TWO_PI;
    } else if (std::abs(reference_value - upper_shift_center) < std::abs(reference_value - center) &&
               hi + TWO_PI <= limit.hi + 1e-12) {
        lo += TWO_PI;
        hi += TWO_PI;
    }
    return {std::max(lo, limit.lo), std::min(hi, limit.hi)};
}

struct CanonicalEvidenceFrame {
    std::vector<Interval> lookup_intervals;
    lect_database::SectorId sector = lect_database::kPrimarySector;
    std::optional<JointSymmetry> symmetry;
};

CanonicalEvidenceFrame canonical_evidence_frame_for_intervals(const Robot& robot,
                                                              const lect_database::LectDatabase& database,
                                                              const std::vector<Interval>& query_intervals) {
    CanonicalEvidenceFrame frame;
    frame.lookup_intervals = query_intervals;
    frame.symmetry = primary_database_symmetry(robot, database);
    if (!frame.symmetry || query_intervals.empty()) {
        return frame;
    }
    const JointSymmetry& symmetry = *frame.symmetry;
    const std::size_t joint_index = static_cast<std::size_t>(symmetry.joint_index);
    if (joint_index >= query_intervals.size()) {
        frame.symmetry.reset();
        return frame;
    }
    double canonical_center = query_intervals[joint_index].center();
    frame.sector = symmetry.canonicalize(canonical_center, canonical_center);
    frame.lookup_intervals[joint_index] = canonical_interval_for_sector(
        symmetry,
        query_intervals[joint_index],
        frame.sector);
    return frame;
}

// A query box whose symmetry-joint interval spans a sector boundary cannot be
// represented faithfully by a single canonical sector (the canonical cache is
// keyed per sector). Detect this so we can fall back to computing the envelope
// directly on the native query intervals.
bool intervals_straddle_sector_boundary(const JointSymmetry& symmetry,
                                        const std::vector<Interval>& intervals) {
    const std::size_t joint_index = static_cast<std::size_t>(symmetry.joint_index);
    if (joint_index >= intervals.size()) {
        return false;
    }
    double dummy = 0.0;
    const int sector_lo = symmetry.canonicalize(intervals[joint_index].lo, dummy);
    const int sector_hi = symmetry.canonicalize(intervals[joint_index].hi, dummy);
    return normalize_sector(sector_lo) != normalize_sector(sector_hi);
}

}  // namespace

DatabaseBoxOracle::DatabaseBoxOracle(Robot robot,
                                     lect_database::LectDatabase& database,
                                     Scene scene,
                                     EndpointSourceConfig endpoint_config,
                                     EnvelopeTypeConfig envelope_config,
                         OracleValidationConfig validation_config,
                                                                                                                                                 const lect_database::LectExternalEvidenceSource* external_evidence_source,
                                                                                                                                                 const lect_database::LectDatabase* direct_external_evidence_database)
    : robot_(std::move(robot)),
      database_(database),
        external_evidence_source_(external_evidence_source),
        direct_external_evidence_database_(direct_external_evidence_database),
      endpoint_config_(std::move(endpoint_config)),
      envelope_config_(std::move(envelope_config)),
      validation_config_(std::move(validation_config)),
      scene_(std::move(scene)),
      checker_(robot_, scene_) {}

DatabaseBoxOracle::DatabaseBoxOracle(Robot robot,
                         lect_database::OnlineEnvelopeCacheTree& online_cache,
                         Scene scene,
                         EndpointSourceConfig endpoint_config,
                         EnvelopeTypeConfig envelope_config,
                         OracleValidationConfig validation_config,
                                                                         const lect_database::LectExternalEvidenceSource* external_evidence_source,
                                                                         const lect_database::LectDatabase* direct_external_evidence_database)
    : robot_(std::move(robot)),
    database_(online_cache.database()),
    online_cache_(&online_cache),
    external_evidence_source_(external_evidence_source),
    direct_external_evidence_database_(direct_external_evidence_database),
    endpoint_config_(std::move(endpoint_config)),
    envelope_config_(std::move(envelope_config)),
    validation_config_(std::move(validation_config)),
    scene_(std::move(scene)),
    checker_(robot_, scene_) {}

int DatabaseBoxOracle::n_dims() const {
    return static_cast<int>(database_.root_intervals().size());
}

int DatabaseBoxOracle::max_tree_depth() const {
    return database_.max_tree_depth();
}

const std::vector<Interval>& DatabaseBoxOracle::root_intervals() const {
    return database_.root_intervals();
}

std::vector<Interval> DatabaseBoxOracle::node_intervals(OracleNodeId node) const {
    if (online_cache_ != nullptr) {
        auto box = online_cache_->node_intervals(to_database_node(node));
        return box ? std::move(*box) : database_.root_intervals();
    }
    auto box = database_.node_box(to_database_node(node));
    return box ? std::move(*box) : database_.root_intervals();
}

Eigen::VectorXd DatabaseBoxOracle::tree_configuration_for_query(const Eigen::Ref<const Eigen::VectorXd>& q) const {
    Eigen::VectorXd tree_q = q;
    if (database_uses_canonical_symmetry(database_) && tree_q.size() > 0) {
        lect_database::canonicalize_configuration_for_robot(
            robot_,
            true,
            database_.identity().symmetry_descriptor,
            std::span<double>(tree_q.data(), static_cast<std::size_t>(tree_q.size())));
    }
    return tree_q;
}

std::vector<Interval> DatabaseBoxOracle::query_intervals_for_node(OracleNodeId node,
                                                                  const std::vector<Interval>& tree_intervals,
                                                                  const Eigen::Ref<const Eigen::VectorXd>& q) const {
    (void)node;
    auto symmetry = primary_database_symmetry(robot_, database_);
    if (!symmetry || tree_intervals.empty() || q.size() <= symmetry->joint_index) {
        return tree_intervals;
    }
    const auto& limits = robot_.joint_limits().limits;
    const std::size_t joint_index = static_cast<std::size_t>(symmetry->joint_index);
    if (joint_index >= tree_intervals.size() || joint_index >= limits.size()) {
        return tree_intervals;
    }
    double canonical_value = q[static_cast<int>(joint_index)];
    const lect_database::SectorId sector = symmetry->canonicalize(canonical_value, canonical_value);
    std::vector<Interval> query_intervals = tree_intervals;
    query_intervals[joint_index] = map_canonical_interval_to_sector(
        *symmetry,
        tree_intervals[joint_index],
        sector,
        limits[joint_index],
        q[static_cast<int>(joint_index)]);
    return query_intervals;
}

bool DatabaseBoxOracle::contains_point(OracleNodeId node, const Eigen::Ref<const Eigen::VectorXd>& q) const {
    const Eigen::VectorXd tree_q = tree_configuration_for_query(q);
    const auto box = database_.node_box(to_database_node(node));
    if (!box || tree_q.size() != static_cast<int>(box->size())) {
        return false;
    }
    for (int dim = 0; dim < tree_q.size(); ++dim) {
        if (!(*box)[static_cast<std::size_t>(dim)].contains(tree_q[dim])) {
            return false;
        }
    }
    return true;
}

bool DatabaseBoxOracle::is_leaf(OracleNodeId node) const {
    if (online_cache_ != nullptr) {
        return online_cache_->is_leaf(to_database_node(node));
    }
    return database_.topology(to_database_node(node)).leaf;
}

int DatabaseBoxOracle::depth(OracleNodeId node) const {
    if (online_cache_ != nullptr) {
        return online_cache_->depth(to_database_node(node));
    }
    return database_.topology(to_database_node(node)).depth;
}

int DatabaseBoxOracle::split_dim(OracleNodeId node) const {
    return database_.topology(to_database_node(node)).split_dim;
}

double DatabaseBoxOracle::split_value(OracleNodeId node) const {
    return database_.topology(to_database_node(node)).split_value;
}

OracleNodeId DatabaseBoxOracle::left_child(OracleNodeId node) const {
    const auto id = database_.topology(to_database_node(node)).left;
    return from_database_node(id);
}

OracleNodeId DatabaseBoxOracle::right_child(OracleNodeId node) const {
    const auto id = database_.topology(to_database_node(node)).right;
    return from_database_node(id);
}

SplitNodeResult DatabaseBoxOracle::split_node(OracleNodeId node,
                                              const std::vector<Interval>& intervals,
                                              int changed_dim,
                                              const OracleSplitOptions& options) {
    (void)intervals;
    (void)changed_dim;
    (void)options;
    SplitNodeResult result;
    const auto children = online_cache_ != nullptr
        ? online_cache_->split_leaf(to_database_node(node))
        : database_.split_leaf(to_database_node(node));
    if (!lect_database::valid_node_id(children.first) || !lect_database::valid_node_id(children.second)) {
        return result;
    }
    const auto topology = database_.topology(to_database_node(node));
    result.split = true;
    result.node = node;
    result.left = from_database_node(children.first);
    result.right = from_database_node(children.second);
    result.split_dim = topology.split_dim;
    result.split_value = topology.split_value;
    return result;
}

SplitNodeResult DatabaseBoxOracle::split_node_at(OracleNodeId node, int split_dim, double split_value) {
    SplitNodeResult result;
    const auto children = database_.split_leaf(to_database_node(node), split_dim, split_value);
    if (!lect_database::valid_node_id(children.first) || !lect_database::valid_node_id(children.second)) {
        return result;
    }
    const auto topology = database_.topology(to_database_node(node));
    result.split = true;
    result.node = node;
    result.left = from_database_node(children.first);
    result.right = from_database_node(children.second);
    result.split_dim = topology.split_dim;
    result.split_value = topology.split_value;
    return result;
}

bool DatabaseBoxOracle::point_in_collision(const Eigen::Ref<const Eigen::VectorXd>& q) const {
    return checker_.check_config(q);
}

lect_database::EvidenceKey DatabaseBoxOracle::endpoint_key(OracleNodeId node) const {
    lect_database::EvidenceKey key;
    key.node_id = to_database_node(node);
    const auto topology = database_.topology(key.node_id);
    key.node_path = topology.path;
    key.node_path_valid = lect_database::valid_node_id(topology.id);
    key.sector = lect_database::kPrimarySector;
    key.channel = database_channel_for_endpoint(endpoint_config_.source);
    key.endpoint_source = endpoint_config_.source;
    key.payload_kind = lect_database::EvidencePayloadKind::EndpointEnvelope;
    return key;
}

std::optional<DatabaseBoxOracle::EndpointPayload> DatabaseBoxOracle::endpoint_payload_for_node(
    OracleNodeId node,
    const std::vector<Interval>& intervals,
    int changed_dim) {
    const auto key = endpoint_key(node);
    const auto evidence_frame = canonical_evidence_frame_for_intervals(robot_, database_, intervals);
    const int normalized_sector = normalize_sector(evidence_frame.sector);
    const std::uint64_t envelope_cache_key = make_envelope_cache_key(key.node_id, normalized_sector);
    if (evidence_frame.symmetry &&
        intervals_straddle_sector_boundary(*evidence_frame.symmetry, intervals)) {
        // The query box spans a sector boundary; a single canonical sector cannot
        // represent it. Compute the envelope directly on the native query box so
        // validation never has to remember a sector for boundary boxes.
        const auto endpoint_start = Clock::now();
        EndpointSourceConfig materialization_config =
            hifk_config_for_materialization(*this, node, endpoint_config_);
        EndpointIAABBResult endpoint =
            compute_endpoint_iaabb(robot_, intervals, materialization_config, nullptr, changed_dim);
        counters_.materialization_endpoint_wall_time_us += elapsed_us(endpoint_start);
        if (endpoint.endpoint_iaabbs.empty()) {
            return std::nullopt;
        }
        EndpointPayload payload;
        payload.owned_payload = std::move(endpoint.endpoint_iaabbs);
        payload.payload = payload.owned_payload;
        return payload;
    }
    auto reflect_payload = [&](EndpointPayload payload) -> EndpointPayload {
        if (!evidence_frame.symmetry || normalize_sector(evidence_frame.sector) == 0 || payload.payload.empty()) {
            return payload;
        }
        const int n_endpoint_boxes = static_cast<int>(payload.payload.size() / 6u);
        if (n_endpoint_boxes <= 0 || payload.payload.size() != static_cast<std::size_t>(n_endpoint_boxes) * 6u) {
            return payload;
        }
        EndpointPayload reflected;
        reflected.owned_payload.resize(payload.payload.size());
        evidence_frame.symmetry->transform_all_endpoint_iaabbs(
            payload.payload.data(),
            n_endpoint_boxes,
            normalize_sector(evidence_frame.sector),
            reflected.owned_payload.data());
        reflected.payload = reflected.owned_payload;
        reflected.envelope_cache_key = payload.envelope_cache_key;
        reflected.envelope_cacheable = payload.envelope_cacheable;
        return reflected;
    };
    const bool use_endpoint_evidence_cache = validation_config_.enable_endpoint_evidence_cache;
    const bool store_endpoint_evidence_cache =
        use_endpoint_evidence_cache && validation_config_.store_endpoint_evidence_cache;
    const bool read_local_endpoint_cache =
        use_endpoint_evidence_cache && (online_cache_ != nullptr || store_endpoint_evidence_cache);
    auto lookup_external_payload = [&]() -> std::optional<EndpointPayload> {
        if (!validation_config_.external_evidence_materialization || external_evidence_source_ == nullptr) {
            return std::nullopt;
        }
        const auto external_lookup_start = Clock::now();
        std::optional<lect_database::EvidenceRecordView> cached;
        const bool direct_external_keys_match =
            direct_external_evidence_database_ != nullptr &&
            database_.identity().root_domain_fingerprint == direct_external_evidence_database_->identity().root_domain_fingerprint &&
            database_.identity().split_policy_hash == direct_external_evidence_database_->identity().split_policy_hash;
        if (direct_external_keys_match) {
            cached = direct_external_evidence_database_->evidence(key);
        }
        if (!cached) {
            cached = external_evidence_source_->endpoint_for_box_exact(evidence_frame.lookup_intervals, key);
        }
        counters_.materialization_external_lookup_time_us += elapsed_us(external_lookup_start);
        if (!cached) {
            return std::nullopt;
        }
        counters_.materialization_reused_external_evidence += 1;
        last_validation_detail_.reused_external_evidence = true;
        if (validation_config_.external_evidence_backfill_active && store_endpoint_evidence_cache) {
            lect_database::EvidenceRecord backfill;
            backfill.key = key;
            backfill.child_hull = cached->child_hull;
            backfill.unavailable = cached->unavailable;
            backfill.payload.assign(cached->payload.begin(), cached->payload.end());
            if (online_cache_ != nullptr) {
                online_cache_->put_evidence(backfill);
            } else {
                database_.put_evidence(std::move(backfill));
            }
        }
        const auto read_start = Clock::now();
        EndpointPayload payload;
        payload.record_storage = cached->storage;
        payload.storage_owner = cached->storage_owner;
        payload.payload = cached->payload;
        payload.envelope_cache_key = envelope_cache_key;
        payload.envelope_cacheable = true;
        counters_.materialization_external_read_time_us += elapsed_us(read_start);
        return reflect_payload(std::move(payload));
    };
    const bool prefer_external_first =
        validation_config_.external_evidence_materialization && external_evidence_source_ != nullptr &&
        !validation_config_.external_evidence_backfill_active;
    if (prefer_external_first) {
        if (auto payload = lookup_external_payload()) {
            return payload;
        }
    }
    if (read_local_endpoint_cache) {
        if (online_cache_ != nullptr) {
            const auto lookup_start = Clock::now();
            auto cached = online_cache_->evidence(key);
            counters_.materialization_cache_lookup_time_us += elapsed_us(lookup_start);
            if (cached) {
                const auto read_start = Clock::now();
                EndpointPayload payload;
                payload.owned_payload = std::move(cached->payload);
                payload.payload = payload.owned_payload;
                payload.envelope_cache_key = envelope_cache_key;
                payload.envelope_cacheable = true;
                counters_.materialization_cache_read_time_us += elapsed_us(read_start);
                counters_.materialization_reused_endpoint_cache += 1;
                return reflect_payload(std::move(payload));
            }
        }
        const auto database_lookup_start = Clock::now();
        auto local_cached = database_.evidence(key);
        counters_.materialization_cache_lookup_time_us += elapsed_us(database_lookup_start);
        if (local_cached) {
            const auto read_start = Clock::now();
            EndpointPayload payload;
            payload.record_storage = local_cached->storage;
            payload.storage_owner = local_cached->storage_owner;
            payload.payload = local_cached->payload;
            payload.envelope_cache_key = envelope_cache_key;
            payload.envelope_cacheable = true;
            counters_.materialization_cache_read_time_us += elapsed_us(read_start);
            counters_.materialization_reused_endpoint_cache += 1;
            return reflect_payload(std::move(payload));
        }
    } else {
        counters_.materialization_skipped_endpoint_cache += 1;
    }
    if (!prefer_external_first) {
        if (auto payload = lookup_external_payload()) {
            return payload;
        }
    }

    // Cross-task shared endpoint cache (interval-keyed, thread-safe). Lets a
    // worker reuse an endpoint another worker (or an earlier batch) already
    // computed for the identical canonical box, the way a persistent oracle
    // reuses evidence across queries.
    const bool use_shared_endpoint_cache =
        shared_endpoint_cache_ != nullptr &&
        validation_config_.enable_endpoint_evidence_cache &&
        validation_config_.enable_worker_shared_endpoint_cache;
    if (use_shared_endpoint_cache) {
        const auto shared_lookup_start = Clock::now();
        auto shared_cached =
            shared_endpoint_cache_->endpoint_for_box_exact(evidence_frame.lookup_intervals, key);
        counters_.materialization_cache_lookup_time_us += elapsed_us(shared_lookup_start);
        if (shared_cached) {
            counters_.materialization_reused_shared_endpoint_cache += 1;
            last_validation_detail_.reused_endpoint_cache = true;
            EndpointPayload payload;
            payload.record_storage = shared_cached->storage;
            payload.storage_owner = shared_cached->storage_owner;
            payload.payload = shared_cached->payload;
            payload.envelope_cache_key = envelope_cache_key;
            payload.envelope_cacheable = true;
            return reflect_payload(std::move(payload));
        }
    }

    const auto endpoint_start = Clock::now();
    EndpointSourceConfig materialization_config = hifk_config_for_materialization(*this, node, endpoint_config_);
    EndpointIAABBResult endpoint =
        materialization_config.source == EndpointSource::IFK
            ? compute_endpoint_iaabb_ifk_aa_stateful(robot_, evidence_frame.lookup_intervals, aa_fk_prefix_state_)
            : compute_endpoint_iaabb(robot_, evidence_frame.lookup_intervals, materialization_config, nullptr, changed_dim);
    counters_.materialization_endpoint_wall_time_us += elapsed_us(endpoint_start);
    if (endpoint.endpoint_iaabbs.empty()) {
        return std::nullopt;
    }
    EndpointPayload payload;
    payload.owned_payload = std::move(endpoint.endpoint_iaabbs);
    payload.payload = payload.owned_payload;
    payload.envelope_cache_key = envelope_cache_key;
    payload.envelope_cacheable = true;
    lect_database::EvidenceRecord record;
    record.key = key;
    record.payload = payload.owned_payload;
    record.child_hull = false;
    if (store_endpoint_evidence_cache) {
        if (online_cache_ != nullptr) {
            online_cache_->put_evidence(record);
        } else {
            database_.put_evidence(record);
        }
    }
    if (use_shared_endpoint_cache) {
        shared_endpoint_cache_->put(evidence_frame.lookup_intervals, key,
                                    payload.owned_payload, /*child_hull=*/false,
                                    /*unavailable=*/false);
        counters_.materialization_stored_shared_endpoint_cache += 1;
    }
    counters_.materializations += 1;
    if (store_endpoint_evidence_cache) {
        counters_.materialization_stored_endpoint += 1;
    }
    counters_.materialization_endpoint_time_us += endpoint.enumerate_time_us;
    return reflect_payload(std::move(payload));
}

BoxValidation DatabaseBoxOracle::classify_payload(OracleNodeId node,
                                                  const std::vector<Interval>& intervals,
                                                  const EndpointPayload& endpoint_payload) {
    (void)intervals;
    const auto envelope_read_start = Clock::now();
    const LinkEnvelope* envelope = nullptr;
    if (endpoint_payload.envelope_cacheable) {
        const auto cache_it = envelope_cache_.find(endpoint_payload.envelope_cache_key);
        counters_.materialization_envelope_read_time_us += elapsed_us(envelope_read_start);
        if (cache_it != envelope_cache_.end()) {
            counters_.materialization_reused_cached_envelope += 1;
            envelope = &cache_it->second;
        }
    } else {
        counters_.materialization_envelope_read_time_us += elapsed_us(envelope_read_start);
    }

    LinkEnvelope computed_envelope;
    double envelope_compute_us = 0.0;
    if (envelope == nullptr) {
        const auto envelope_start = Clock::now();
        computed_envelope = compute_link_envelope(endpoint_payload.payload.data(),
                                                  robot_.n_active_links(),
                                                  robot_.active_link_radii(),
                                                  envelope_config_);
        envelope_compute_us = elapsed_us(envelope_start);
        counters_.materialization_envelope_compute_time_us += envelope_compute_us;
        if (endpoint_payload.envelope_cacheable) {
            auto [cache_it, inserted] = envelope_cache_.try_emplace(endpoint_payload.envelope_cache_key,
                                                                    std::move(computed_envelope));
            if (!inserted) {
                cache_it->second = std::move(computed_envelope);
            }
            envelope = &cache_it->second;
        } else {
            envelope = &computed_envelope;
        }
    }
    EnvelopeCollisionStats collision_stats;
    const auto collision_start = Clock::now();
    const CollisionResultKind collision = collide_envelope_aabbs(*envelope,
                                                                 scene_.obstacles().data(),
                                                                 scene_.n_obstacles(),
                                                                 {},
                                                                 &collision_stats);
    const double collision_us = elapsed_us(collision_start);
    counters_.materialization_envelope_collision_time_us += collision_us;
    counters_.materialization_envelope_time_us += envelope_compute_us + collision_us;
    record_envelope_collision(counters_, collision_stats);
    counters_.envelope_collision_queries += 1;
    last_validation_detail_.node = node;
    last_validation_detail_.depth = depth(node);
    last_validation_detail_.mode = validation_config_.mode;
    last_validation_detail_.endpoint_source = endpoint_config_.source;
    last_validation_detail_.endpoint_safety_level = endpoint_source_default_safety(endpoint_config_.source);
    last_validation_detail_.endpoint_is_safe = endpoint_safety_is_certified(last_validation_detail_.endpoint_safety_level);
    last_validation_detail_.materialized = true;
    last_validation_detail_.aabb_overlap = collision != CollisionResultKind::DefinitelyFree;
    if (collision == CollisionResultKind::DefinitelyFree) {
        counters_.certified_free += 1;
        counters_.envelope_collision_free += 1;
        last_validation_detail_.validation = BoxValidation::Free;
        last_validation_detail_.safety_status = BoxSafetyStatus::CertifiedFree;
        last_validation_detail_.collision_possible = false;
        return BoxValidation::Free;
    }
    counters_.collision_possible += 1;
    counters_.envelope_collision_maybe += 1;
    last_validation_detail_.validation = BoxValidation::Unknown;
    last_validation_detail_.safety_status = BoxSafetyStatus::Unknown;
    last_validation_detail_.collision_possible = true;
    return BoxValidation::Unknown;
}

BoxValidation DatabaseBoxOracle::validate_node(OracleNodeId node,
                                               const std::vector<Interval>& intervals,
                                               int changed_dim) {
    const auto total_start = Clock::now();
    counters_.node_validations += 1;
    last_validation_detail_ = {};
    last_validation_detail_.node = node;
    last_validation_detail_.depth = depth(node);
    last_validation_detail_.mode = validation_config_.mode;
    const double preamble_us = elapsed_us(total_start);
    counters_.validate_node_preamble_time_us += preamble_us;
    if (scene_.empty()) {
        counters_.certified_free += 1;
        last_validation_detail_.validation = BoxValidation::Free;
        last_validation_detail_.safety_status = BoxSafetyStatus::CertifiedFree;
        last_validation_detail_.collision_possible = false;
        const double total_us = elapsed_us(total_start);
        counters_.validate_node_total_time_us += total_us;
        counters_.validate_node_overhead_time_us += std::max(0.0, total_us - preamble_us);
        return BoxValidation::Free;
    }
    const auto endpoint_path_start = Clock::now();
    auto payload = endpoint_payload_for_node(node, intervals, changed_dim);
    const double endpoint_path_us = elapsed_us(endpoint_path_start);
    counters_.validate_node_endpoint_path_time_us += endpoint_path_us;
    if (!payload) {
        counters_.collision_possible += 1;
        last_validation_detail_.validation = BoxValidation::Unknown;
        last_validation_detail_.safety_status = BoxSafetyStatus::Unknown;
        last_validation_detail_.collision_possible = true;
        const double total_us = elapsed_us(total_start);
        counters_.validate_node_total_time_us += total_us;
        counters_.validate_node_overhead_time_us += std::max(0.0, total_us - preamble_us - endpoint_path_us);
        return BoxValidation::Unknown;
    }
    const auto classify_start = Clock::now();
    const BoxValidation result = classify_payload(node, intervals, *payload);
    const double classify_us = elapsed_us(classify_start);
    counters_.validate_node_classify_time_us += classify_us;
    const double total_us = elapsed_us(total_start);
    counters_.validate_node_total_time_us += total_us;
    counters_.validate_node_overhead_time_us +=
        std::max(0.0, total_us - preamble_us - endpoint_path_us - classify_us);
    return result;
}

bool DatabaseBoxOracle::validate_intervals(const std::vector<Interval>& intervals) {
    counters_.interval_validations += 1;
    return !checker_.check_box(intervals);
}

bool DatabaseBoxOracle::is_reserved(OracleNodeId node) const {
    return node_to_box_.find(node) != node_to_box_.end();
}

std::optional<int> DatabaseBoxOracle::reservation_owner(OracleNodeId node) const {
    const auto it = node_to_box_.find(node);
    return it == node_to_box_.end() ? std::nullopt : std::optional<int>(it->second);
}

void DatabaseBoxOracle::reserve_node(OracleNodeId node, int box_id) {
    node_to_box_[node] = box_id;
    box_to_node_[box_id] = node;
}

void DatabaseBoxOracle::release_node(OracleNodeId node) {
    const auto it = node_to_box_.find(node);
    if (it == node_to_box_.end()) {
        return;
    }
    box_to_node_.erase(it->second);
    node_to_box_.erase(it);
}

void DatabaseBoxOracle::release_box(int box_id) {
    const auto it = box_to_node_.find(box_id);
    if (it == box_to_node_.end()) {
        return;
    }
    node_to_box_.erase(it->second);
    box_to_node_.erase(it);
}

void DatabaseBoxOracle::clear_reservations() {
    node_to_box_.clear();
    box_to_node_.clear();
}

OracleNodeId DatabaseBoxOracle::select_unexplored_node() const {
    for (lect_database::NodeId node_id : database_.node_ids()) {
        const auto topology = database_.topology(node_id);
        const OracleNodeId node = from_database_node(node_id);
        if (topology.leaf && !is_reserved(node)) {
            return node;
        }
    }
    return kInvalidOracleNodeId;
}

int DatabaseBoxOracle::common_ancestor_depth(OracleNodeId lhs_node, OracleNodeId rhs_node) const {
    const auto lhs = to_database_node(lhs_node);
    const auto rhs = to_database_node(rhs_node);
    if (lhs_node < 0 || rhs_node < 0 || !database_.node(lhs) || !database_.node(rhs)) {
        return -1;
    }
    const auto ancestor = database_.lca(lhs, rhs);
    if (!lect_database::valid_node_id(ancestor)) {
        return -1;
    }
    return database_.topology(ancestor).depth;
}

std::unique_ptr<BoxOracleSession> DatabaseBoxOracle::make_session(const OracleSessionConfig& config) {
    return std::make_unique<DatabaseBoxOracleSession>(*this, config);
}

std::shared_ptr<lect_database::SharedEndpointEvidenceCache> DatabaseBoxOracle::shared_endpoint_cache() {
    if (!shared_endpoint_cache_) {
        shared_endpoint_cache_ = std::make_shared<lect_database::SharedEndpointEvidenceCache>(
            validation_config_.shared_endpoint_cache_max_entries,
            validation_config_.shared_endpoint_cache_max_bytes);
    }
    return shared_endpoint_cache_;
}

void DatabaseBoxOracle::set_scene(Scene scene) {
    scene_ = std::move(scene);
    checker_.set_scene(scene_);
}

DatabaseBoxOracleSession::DatabaseBoxOracleSession(DatabaseBoxOracle& master,
                                                   const OracleSessionConfig& config)
    : master_(master),
      master_domain_root_(config.domain_root >= 0 ? config.domain_root : master.root_node()),
      read_only_(config.read_only),
      temp_dir_(make_temp_dir()) {
    if (master_domain_root_ < 0 || !lect_database::valid_node_id(to_database_node(master_domain_root_)) ||
        !master.database().node(to_database_node(master_domain_root_))) {
        throw std::out_of_range("LECTDatabase oracle session domain root is out of range");
    }
    const auto worker_root = master.node_intervals(master_domain_root_);
    if (worker_root.empty()) {
        throw std::runtime_error("LECTDatabase oracle session domain root has no intervals");
    }
    std::error_code ec;
    std::filesystem::create_directories(temp_dir_, ec);
    if (ec) {
        throw std::runtime_error("LECTDatabase oracle session failed to create temp directory");
    }

    std::string reason;
    worker_database_.emplace();
    auto worker_config = make_worker_database_config(master_, temp_dir_, worker_root);
    if (!worker_database_->open(std::move(worker_config), &reason)) {
        throw std::runtime_error("LECTDatabase oracle session failed to open worker database: " + reason);
    }
    auto worker_validation_config = master_.validation_config();
    worker_validation_config.store_endpoint_evidence_cache = false;
    worker_validation_config.external_evidence_backfill_active = false;
    worker_oracle_ = std::make_unique<DatabaseBoxOracle>(master_.robot(),
                                                         *worker_database_,
                                                         master_.scene(),
                                                         master_.endpoint_config(),
                                                         master_.envelope_config(),
                                                         worker_validation_config,
                                                         master_.external_evidence_source(),
                                                         master_.direct_external_evidence_database());
    if (worker_validation_config.enable_worker_shared_endpoint_cache) {
        worker_oracle_->set_shared_endpoint_cache(master_.shared_endpoint_cache());
    }
    node_remap_.emplace(worker_oracle_->root_node(), master_domain_root_);
}

DatabaseBoxOracleSession::~DatabaseBoxOracleSession() {
    worker_oracle_.reset();
    worker_database_.reset();
    std::error_code ec;
    std::filesystem::remove_all(temp_dir_, ec);
}

bool DatabaseBoxOracleSession::commit() {
    if (read_only_) {
        committed_ = true;
        return true;
    }
    if (committed_) {
        return true;
    }
    if (!replay_structure(worker_oracle_->root_node(), master_domain_root_)) {
        return false;
    }
    if (!copy_worker_leaf_evidence()) {
        return false;
    }
    committed_ = true;
    return true;
}

OracleNodeId DatabaseBoxOracleSession::map_node_to_master(OracleNodeId worker_node) const {
    return remap_lookup(node_remap_, worker_node);
}

bool DatabaseBoxOracleSession::replay_structure(OracleNodeId worker_node, OracleNodeId master_node) {
    node_remap_[worker_node] = master_node;
    const auto worker_topology = worker_database_->topology(to_database_node(worker_node));
    if (worker_topology.leaf) {
        return true;
    }

    auto master_topology = master_.database().topology(to_database_node(master_node));
    if (master_topology.leaf) {
        const auto children = master_.database().split_leaf(to_database_node(master_node),
                                                            worker_topology.split_dim,
                                                            worker_topology.split_value);
        if (!lect_database::valid_node_id(children.first) || !lect_database::valid_node_id(children.second)) {
            return false;
        }
        master_topology = master_.database().topology(to_database_node(master_node));
    } else if (master_topology.split_dim != worker_topology.split_dim ||
               std::abs(master_topology.split_value - worker_topology.split_value) > 1e-12) {
        return false;
    }

    const OracleNodeId master_left = from_database_node(master_topology.left);
    const OracleNodeId master_right = from_database_node(master_topology.right);
    if (master_left < 0 || master_right < 0) {
        return false;
    }
    node_remap_[from_database_node(worker_topology.left)] = master_left;
    node_remap_[from_database_node(worker_topology.right)] = master_right;
    return replay_structure(from_database_node(worker_topology.left), master_left) &&
           replay_structure(from_database_node(worker_topology.right), master_right);
}

bool DatabaseBoxOracleSession::copy_worker_leaf_evidence() {
    for (const auto& record : worker_database_->evidence_records()) {
        const auto topology = worker_database_->topology(record.key.node_id);
        if (!topology.leaf) {
            continue;
        }
        const OracleNodeId mapped_node = remap_lookup(node_remap_, from_database_node(record.key.node_id));
        if (mapped_node < 0) {
            return false;
        }
        auto replay = record;
        replay.key.node_id = to_database_node(mapped_node);
        const auto mapped_topology = master_.database().topology(replay.key.node_id);
        replay.key.node_path = mapped_topology.path;
        replay.key.node_path_valid = lect_database::valid_node_id(mapped_topology.id);
        if (!master_.database().put_evidence(std::move(replay))) {
            return false;
        }
    }
    return true;
}

std::filesystem::path DatabaseBoxOracleSession::make_temp_dir() {
    return std::filesystem::temp_directory_path() /
        ("lectdb_sbf_session_" + std::to_string(next_session_id()));
}

std::unique_ptr<BoxOracleSession> DatabaseBoxOracleFactory::make_session(const OracleSessionConfig& config) {
    return master_.make_session(config);
}

}  // namespace rbf
