#pragma once

#include <LECTDatabase/sbf/oracle.h>

#include <sbf/envelope/crit_source.h>
#include <sbf/envelope/endpoint_source.h>
#include <sbf/envelope/ifk_aa_source.h>

namespace rbf {

struct DatabaseBoxOracle::Impl {
    // Incremental AA-backed endpoint state reused along a single-threaded
    // parent->child descent. IFK reuses a single AA-FK chain prefix; HIFK
    // reuses per-leaf AA-FK states when the split schedule is deterministic.
    // Not shared across threads (each worker owns its own oracle).
    AaFkPrefixState aa_fk_prefix_state;
    CritSampleState crit_sample_state;
    HifkAaState hifk_aa_state;
};

}  // namespace rbf
