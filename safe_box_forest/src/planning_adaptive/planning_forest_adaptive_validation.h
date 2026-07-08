#pragma once

#include "planning_forest_adaptive_cover_utils.h"

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace rbf {

class DatabaseBoxOracle;
class StageContext;

struct AdaptiveValidationOutcome {
    BoxValidation validation = BoxValidation::Unknown;
    OracleValidationDetail detail;
    bool exception = false;
};

class AdaptiveFrontierValidationSession {
public:
    AdaptiveFrontierValidationSession(DatabaseBoxOracle& primary_oracle,
                                      StageContext& context,
                                      int adaptive_threads,
                                      int validation_batch_limit,
                                      bool collect_overlap_ratio,
                                      std::unordered_map<std::string, double>& diagnostics);
    ~AdaptiveFrontierValidationSession();

    std::vector<AdaptiveValidationOutcome> validate_batch(
        const std::vector<AdaptiveFrontierItem>& items);

private:
    DatabaseBoxOracle& primary_oracle_;
    StageContext& context_;
    int validation_batch_limit_ = 1;
    std::unordered_map<std::string, double>& diagnostics_;
    std::vector<std::unique_ptr<DatabaseBoxOracle>> worker_oracles_;
};

}  // namespace rbf
