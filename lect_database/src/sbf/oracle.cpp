#include <LECTDatabase/sbf/oracle.h>

#include <sbf/envelope/endpoint_source.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <system_error>

namespace rbf {
namespace {

using lect_database::NodeId;

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
    return endpoint_safety_is_certified(endpoint_source_default_safety(source))
        ? lect_database::EvidenceChannel::Safe
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

}  // namespace

DatabaseBoxOracle::DatabaseBoxOracle(Robot robot,
                                     lect_database::LectDatabase& database,
                                     Scene scene,
                                     EndpointSourceConfig endpoint_config,
                                                                         EnvelopeTypeConfig envelope_config,
                                                                         OracleValidationConfig validation_config,
                                                                         const lect_database::LectDatabase* external_evidence_database)
    : robot_(std::move(robot)),
      database_(database),
      endpoint_config_(std::move(endpoint_config)),
      envelope_config_(std::move(envelope_config)),
      validation_config_(std::move(validation_config)),
      scene_(std::move(scene)),
            checker_(robot_, scene_) {
        set_external_evidence_database(external_evidence_database);
}

DatabaseBoxOracle::DatabaseBoxOracle(Robot robot,
                                                                         lect_database::LectDatabase& database,
                                                                         Scene scene,
                                                                         EndpointSourceConfig endpoint_config,
                                                                         EnvelopeTypeConfig envelope_config,
                                                                         OracleValidationConfig validation_config,
                                                                         const lect_database::LectExternalEvidenceSource* external_evidence_source)
        : robot_(std::move(robot)),
            database_(database),
            external_evidence_source_(external_evidence_source),
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
                                                                         const lect_database::LectDatabase* external_evidence_database)
        : robot_(std::move(robot)),
            database_(online_cache.database()),
            online_cache_(&online_cache),
            endpoint_config_(std::move(endpoint_config)),
            envelope_config_(std::move(envelope_config)),
            validation_config_(std::move(validation_config)),
            scene_(std::move(scene)),
            checker_(robot_, scene_) {
        set_external_evidence_database(external_evidence_database);
}

DatabaseBoxOracle::DatabaseBoxOracle(Robot robot,
                                                                         lect_database::OnlineEnvelopeCacheTree& online_cache,
                                                                         Scene scene,
                                                                         EndpointSourceConfig endpoint_config,
                                                                         EnvelopeTypeConfig envelope_config,
                                                                         OracleValidationConfig validation_config,
                                                                         const lect_database::LectExternalEvidenceSource* external_evidence_source)
        : robot_(std::move(robot)),
            database_(online_cache.database()),
            online_cache_(&online_cache),
            external_evidence_source_(external_evidence_source),
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

bool DatabaseBoxOracle::contains_point(OracleNodeId node, const Eigen::Ref<const Eigen::VectorXd>& q) const {
    const auto box = database_.node_box(to_database_node(node));
    if (!box || q.size() != static_cast<int>(box->size())) {
        return false;
    }
    for (int dim = 0; dim < q.size(); ++dim) {
        if (!(*box)[static_cast<std::size_t>(dim)].contains(q[dim])) {
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

std::optional<std::vector<float>> DatabaseBoxOracle::endpoint_payload_for_node(
    OracleNodeId node,
    const std::vector<Interval>& intervals,
    int changed_dim) {
    const auto key = endpoint_key(node);
    if (online_cache_ != nullptr) {
        bool reused_external_evidence = false;
        const auto* exact_intervals =
            validation_config_.external_evidence_materialization ? &intervals : nullptr;
        const auto* external_source =
            validation_config_.external_evidence_materialization ? external_evidence_source_ : nullptr;
        if (auto cached = online_cache_->evidence(key, exact_intervals, external_source, &reused_external_evidence)) {
            if (reused_external_evidence) {
                counters_.materialization_reused_external_evidence += 1;
                last_validation_detail_.reused_external_evidence = true;
            } else {
                counters_.materialization_reused_endpoint_cache += 1;
            }
            return std::move(cached->payload);
        }
    } else if (auto cached = database_.evidence(key)) {
        counters_.materialization_reused_endpoint_cache += 1;
        return std::vector<float>(cached->payload.begin(), cached->payload.end());
    } else if (validation_config_.external_evidence_materialization && external_evidence_source_ != nullptr) {
        auto cached = external_evidence_source_->endpoint_for_box_exact(intervals, key);
        if (!cached && key.node_path_valid) {
            cached = external_evidence_source_->evidence(key);
        }
        if (cached) {
            counters_.materialization_reused_external_evidence += 1;
            last_validation_detail_.reused_external_evidence = true;
            if (validation_config_.external_evidence_backfill_active) {
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
            return std::vector<float>(cached->payload.begin(), cached->payload.end());
        }
    }

    return materialize_endpoint_payload_for_node(node, intervals, changed_dim);
}

std::optional<std::vector<float>> DatabaseBoxOracle::materialize_endpoint_payload_for_node(
    OracleNodeId node,
    const std::vector<Interval>& intervals,
    int changed_dim) {
    const auto key = endpoint_key(node);

    const EndpointSourceConfig materialization_endpoint_config =
        hifk_config_for_materialization(*this, node, endpoint_config_);
    EndpointIAABBResult endpoint =
        compute_endpoint_iaabb(robot_, intervals, materialization_endpoint_config, nullptr, changed_dim);
    if (endpoint.endpoint_iaabbs.empty()) {
        return std::nullopt;
    }
    lect_database::EvidenceRecord record;
    record.key = key;
    record.payload = std::move(endpoint.endpoint_iaabbs);
    record.child_hull = false;
    if (online_cache_ != nullptr) {
        online_cache_->put_evidence(record, false);
    } else {
        database_.put_evidence(record);
    }
    counters_.materializations += 1;
    counters_.materialization_stored_endpoint += 1;
    counters_.materialization_endpoint_time_us += endpoint.enumerate_time_us;
    if (online_cache_ != nullptr) {
        if (auto stored = online_cache_->evidence(key)) {
            return std::move(stored->payload);
        }
        return record.payload;
    }
    if (auto stored = database_.evidence(key)) {
        return std::vector<float>(stored->payload.begin(), stored->payload.end());
    }
    return std::nullopt;
}

BoxValidation DatabaseBoxOracle::classify_payload(OracleNodeId node,
                                                  const std::vector<Interval>& intervals,
                                                  const std::vector<float>& endpoint_payload) {
    (void)intervals;
    LinkEnvelope envelope = compute_link_envelope(endpoint_payload.data(),
                                                  robot_.n_active_links(),
                                                  robot_.active_link_radii(),
                                                  envelope_config_);
    EnvelopeCollisionStats collision_stats;
    const CollisionResultKind collision = collide_envelope_aabbs(envelope,
                                                                 scene_.obstacles().data(),
                                                                 scene_.n_obstacles(),
                                                                 {},
                                                                 &collision_stats);
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
    counters_.node_validations += 1;
    last_validation_detail_ = {};
    last_validation_detail_.node = node;
    last_validation_detail_.depth = depth(node);
    last_validation_detail_.mode = validation_config_.mode;
    if (scene_.empty()) {
        counters_.certified_free += 1;
        last_validation_detail_.validation = BoxValidation::Free;
        last_validation_detail_.safety_status = BoxSafetyStatus::CertifiedFree;
        last_validation_detail_.collision_possible = false;
        return BoxValidation::Free;
    }
    auto payload = endpoint_payload_for_node(node, intervals, changed_dim);
    if (!payload) {
        counters_.collision_possible += 1;
        last_validation_detail_.validation = BoxValidation::Unknown;
        last_validation_detail_.safety_status = BoxSafetyStatus::Unknown;
        last_validation_detail_.collision_possible = true;
        return BoxValidation::Unknown;
    }
    auto validation = classify_payload(node, intervals, *payload);
    if (validation == BoxValidation::Unknown &&
        last_validation_detail_.reused_external_evidence &&
        validation_config_.external_evidence_materialization) {
        auto refined_payload = materialize_endpoint_payload_for_node(node, intervals, changed_dim);
        if (refined_payload) {
            last_validation_detail_.reused_external_evidence = false;
            validation = classify_payload(node, intervals, *refined_payload);
        }
    }
    return validation;
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
    worker_oracle_ = std::make_unique<DatabaseBoxOracle>(master_.robot(),
                                                         *worker_database_,
                                                         master_.scene(),
                                                         master_.endpoint_config(),
                                                         master_.envelope_config(),
                                                         master_.validation_config(),
                                                         master_.external_evidence_source());
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
    const auto now = std::chrono::system_clock::now().time_since_epoch().count();
    return std::filesystem::temp_directory_path() /
        ("lectdb_sbf_session_" + std::to_string(now) + "_" + std::to_string(next_session_id()));
}

std::unique_ptr<BoxOracleSession> DatabaseBoxOracleFactory::make_session(const OracleSessionConfig& config) {
    return master_.make_session(config);
}

}  // namespace rbf
