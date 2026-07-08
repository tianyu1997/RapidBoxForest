#include "leaf_sweep_grower_internal.h"

#include <SBF/oracle.h>
#include <SBF/runtime.h>

namespace rbf {

void add_counter(LeafSweepResult& result,
				 StageContext& context,
				 const std::string& key,
				 double value) {
	context.diagnostics().add_counter(key, value);
	result.diagnostics[key] += value;
}

void set_value(LeafSweepResult& result,
			   StageContext& context,
			   const std::string& key,
			   double value) {
	context.diagnostics().set_value(key, value);
	result.diagnostics[key] = value;
}

void record_timing(LeafSweepResult& result,
				   StageContext& context,
				   const std::string& key,
				   double milliseconds) {
	context.diagnostics().record_timing(key, milliseconds);
	result.diagnostics[key + ".total_ms"] += milliseconds;
	result.diagnostics[key + ".count"] += 1.0;
}

ScopedOracleEnvelopeCache::ScopedOracleEnvelopeCache(DatabaseBoxOracle& oracle, bool enabled)
	: oracle_(oracle), previous_(oracle.envelope_cache_enabled()) {
	oracle_.set_envelope_cache_enabled(enabled);
}

ScopedOracleEnvelopeCache::~ScopedOracleEnvelopeCache() {
	oracle_.set_envelope_cache_enabled(previous_);
}

ScopedOracleFullOverlapStats::ScopedOracleFullOverlapStats(DatabaseBoxOracle& oracle, bool enabled)
	: oracle_(oracle), previous_(oracle.validation_config().collect_full_overlap_stats) {
	if (enabled) {
		oracle_.set_collect_full_overlap_stats(true);
	}
}

ScopedOracleFullOverlapStats::~ScopedOracleFullOverlapStats() {
	oracle_.set_collect_full_overlap_stats(previous_);
}

}  // namespace rbf
