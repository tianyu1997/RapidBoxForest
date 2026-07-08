#pragma once

#include <cstdint>

namespace rbf {

enum class ExecutionMode : std::uint8_t {
    Inline = 0,
    Parallel = 1,
};

struct RuntimeConfig {
    ExecutionMode mode = ExecutionMode::Inline;
    int n_threads = 1;
    int batch_size = 0;
    int parallel_threshold = 0;
    bool deterministic_reduce = true;
};

}  // namespace rbf
