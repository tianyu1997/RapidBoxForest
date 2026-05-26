#pragma once
/// @file incremental_context.h
/// @brief Stateful envelope computation with reusable source-local caches.

#include <sbf/core/fk_state.h>
#include <sbf/core/robot.h>
#include <sbf/envelope/crit_source.h>
#include <sbf/envelope/endpoint_source.h>
#include <sbf/envelope/envelope_type.h>

#include <vector>

namespace link_interval_envelope {

struct IncrementalEnvelopeResult {
    rbf::EndpointIAABBResult endpoint;
    rbf::LinkEnvelope envelope;
    double endpoint_time_us = 0.0;
    double envelope_time_us = 0.0;
    int changed_dim = -1;
    bool used_incremental_fk = false;
    bool used_source_incremental_state = false;
    bool reused_fk = false;
    bool reused_endpoint_cache = false;
};

/// Stateful context that keeps the previous v6-compatible rbf::FKState for
/// sources that need it internally, plus reusable CritSample state.
///
/// The FKState layout is intentionally the same as `cpp/v6/include/rbf/core/fk_state.h`
/// in this repository snapshot. C++ callers can pass the exposed `fk_state()` to
/// low-level functions that expect this standalone package's `rbf::FKState`.
class IncrementalEnvelopeContext {
public:
    IncrementalEnvelopeContext(
        rbf::Robot robot,
        rbf::EndpointSourceConfig endpoint_config = {},
        rbf::EnvelopeTypeConfig envelope_config = {});

    IncrementalEnvelopeResult compute(
        const std::vector<rbf::Interval>& intervals,
        int changed_dim = -1);

    void reset();

    const rbf::Robot& robot() const { return robot_; }
    const rbf::EndpointSourceConfig& endpoint_config() const { return endpoint_config_; }
    const rbf::EnvelopeTypeConfig& envelope_config() const { return envelope_config_; }
    const rbf::FKState& fk_state() const { return fk_state_; }
    rbf::FKState& fk_state() { return fk_state_; }
    const std::vector<rbf::Interval>& last_intervals() const { return last_intervals_; }
    bool has_valid_fk() const { return fk_state_.valid; }

private:
    rbf::Robot robot_;
    rbf::EndpointSourceConfig endpoint_config_;
    rbf::EnvelopeTypeConfig envelope_config_;
    rbf::FKState fk_state_;
    rbf::CritSampleState crit_state_;
    std::vector<rbf::Interval> last_intervals_;

    int infer_changed_dim(const std::vector<rbf::Interval>& intervals) const;
    rbf::EndpointIAABBResult endpoint_from_current_fk() const;
    rbf::EndpointIAABBResult endpoint_from_crit_cache() const;
};

}  // namespace link_interval_envelope
