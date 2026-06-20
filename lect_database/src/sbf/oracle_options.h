#pragma once

#include <cstdlib>

namespace rbf {

enum class OracleDebugEnvMode {
    Present,
    EqualsOne,
};

inline bool oracle_debug_env_enabled(const char* name, OracleDebugEnvMode mode) {
    const char* value = std::getenv(name);
    if (value == nullptr) {
        return false;
    }
    switch (mode) {
        case OracleDebugEnvMode::Present:
            return true;
        case OracleDebugEnvMode::EqualsOne:
            return value[0] == '1';
    }
    return false;
}

inline bool oracle_best_tighten_debug_enabled() {
    return oracle_debug_env_enabled("SBF_BT_DEBUG", OracleDebugEnvMode::Present);
}

inline bool oracle_canonical_debug_enabled() {
    return oracle_debug_env_enabled("RBF_CANONICAL_DEBUG", OracleDebugEnvMode::EqualsOne);
}

inline bool oracle_envelope_debug_enabled() {
    return oracle_debug_env_enabled("RBF_ENV_DEBUG", OracleDebugEnvMode::EqualsOne);
}

}  // namespace rbf
