#include "planning_forest_adaptive_validation.h"

#include <SBF/runtime.h>
#include <LECTDatabase/sbf/oracle.h>

#include <algorithm>
#include <exception>

namespace rbf {

AdaptiveFrontierValidationSession::AdaptiveFrontierValidationSession(
    DatabaseBoxOracle& primary_oracle,
    StageContext& context,
    int adaptive_threads,
    int validation_batch_limit,
    bool collect_overlap_ratio,
    std::unordered_map<std::string, double>& diagnostics)
    : primary_oracle_(primary_oracle),
      context_(context),
      validation_batch_limit_(std::max(1, validation_batch_limit)),
      diagnostics_(diagnostics) {
    if (validation_batch_limit_ <= 1) {
        return;
    }
    const int worker_count = std::max(1, adaptive_threads);
    worker_oracles_.reserve(static_cast<std::size_t>(worker_count));
    for (int worker = 0; worker < worker_count; ++worker) {
        auto worker_validation_config = primary_oracle_.validation_config();
        worker_validation_config.store_endpoint_evidence_cache = false;
        worker_validation_config.external_evidence_backfill_active = false;
        worker_validation_config.collect_full_overlap_stats = collect_overlap_ratio;
        auto worker_oracle = std::make_unique<DatabaseBoxOracle>(
            primary_oracle_.robot(),
            primary_oracle_.database(),
            primary_oracle_.scene(),
            primary_oracle_.endpoint_config(),
            primary_oracle_.envelope_config(),
            worker_validation_config,
            primary_oracle_.external_evidence_source(),
            nullptr);
        worker_oracle->set_envelope_cache_enabled(primary_oracle_.envelope_cache_enabled());
        if (worker_validation_config.enable_worker_shared_endpoint_cache) {
            worker_oracle->set_shared_endpoint_cache(primary_oracle_.shared_endpoint_cache());
        }
        worker_oracles_.push_back(std::move(worker_oracle));
    }
    diagnostics_["adaptive.parallel_validation_sessions"] =
        static_cast<double>(worker_oracles_.size());
}

AdaptiveFrontierValidationSession::~AdaptiveFrontierValidationSession() = default;

std::vector<AdaptiveValidationOutcome> AdaptiveFrontierValidationSession::validate_batch(
    const std::vector<AdaptiveFrontierItem>& items) {
    std::vector<AdaptiveValidationOutcome> outcomes(items.size());
    if (items.empty()) {
        return outcomes;
    }
    if (validation_batch_limit_ <= 1 || items.size() == 1) {
        for (std::size_t i = 0; i < items.size(); ++i) {
            try {
                outcomes[i].validation =
                    primary_oracle_.validate_node(items[i].node,
                                                  items[i].intervals,
                                                  items[i].changed_dim);
                outcomes[i].detail = primary_oracle_.last_validation_detail();
            } catch (const std::exception&) {
                outcomes[i].validation = BoxValidation::Unknown;
                outcomes[i].exception = true;
            }
        }
        return outcomes;
    }
    context_.executor().parallel_for(0,
                                     static_cast<int>(items.size()),
                                     [&](int local_index) {
        const int worker_id = std::max(0, current_worker_id());
        DatabaseBoxOracle* worker_oracle =
            worker_id < static_cast<int>(worker_oracles_.size()) &&
                    worker_oracles_[static_cast<std::size_t>(worker_id)]
                ? worker_oracles_[static_cast<std::size_t>(worker_id)].get()
                : &primary_oracle_;
        auto& outcome = outcomes[static_cast<std::size_t>(local_index)];
        const auto& item = items[static_cast<std::size_t>(local_index)];
        try {
            outcome.validation =
                worker_oracle->validate_node(item.node,
                                             item.intervals,
                                             item.changed_dim);
            outcome.detail = worker_oracle->last_validation_detail();
        } catch (const std::exception&) {
            outcome.validation = BoxValidation::Unknown;
            outcome.exception = true;
        }
    });
    diagnostics_["adaptive.parallel_validation_batches"] += 1.0;
    diagnostics_["adaptive.parallel_validation_items"] += static_cast<double>(items.size());
    return outcomes;
}

}  // namespace rbf
