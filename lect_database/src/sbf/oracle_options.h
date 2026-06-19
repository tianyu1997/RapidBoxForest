#pragma once

#include <cstdlib>

namespace rbf {

inline bool oracle_best_tighten_debug_enabled() {
    return std::getenv("SBF_BT_DEBUG") != nullptr;
}

inline bool oracle_canonical_debug_enabled() {
    const char* value = std::getenv("RBF_CANONICAL_DEBUG");
    return value != nullptr && value[0] == '1';
}

inline bool oracle_envelope_debug_enabled() {
    const char* value = std::getenv("RBF_ENV_DEBUG");
    return value != nullptr && value[0] == '1';
}

}  // namespace rbf
