#include <SBF/safe_box_forest.h>

namespace rbf {

void RBFPlanningForest::initialize_optional_collision_cache() {
#if defined(SBF_DIAGNOSTIC_API) && SBF_DIAGNOSTIC_API
    initialize_dynamic_collision_cache();
#endif
}

void RBFPlanningForest::clear_optional_collision_cache() {
#if defined(SBF_DIAGNOSTIC_API) && SBF_DIAGNOSTIC_API
    clear_dynamic_collision_cache();
#endif
}

void RBFPlanningForest::populate_optional_collision_cache_from_leaf_sweep(const LeafSweepResult& result,
                                                                    int obstacle_count) {
#if defined(SBF_DIAGNOSTIC_API) && SBF_DIAGNOSTIC_API
    populate_dynamic_collision_cache(result, obstacle_count);
#else
    (void)result;
    (void)obstacle_count;
#endif
}

void RBFPlanningForest::record_optional_collision_cache_box_count(
    BuildProfile& profile,
    const char* diagnostic_key) const {
#if defined(SBF_DIAGNOSTIC_API) && SBF_DIAGNOSTIC_API
    if (diagnostic_key != nullptr && diagnostic_key[0] != '\0') {
        profile.diagnostics[diagnostic_key] = static_cast<double>(dynamic_collision_cache_box_count());
    }
#else
    (void)profile;
    (void)diagnostic_key;
#endif
}

}  // namespace rbf
