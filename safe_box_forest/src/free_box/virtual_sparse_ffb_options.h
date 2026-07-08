#pragma once

namespace rbf::detail {

struct VirtualSparseFfbOptions {
    int binary_probe_depth = -1;
};

inline VirtualSparseFfbOptions virtual_sparse_ffb_options(int binary_probe_depth) {
    VirtualSparseFfbOptions options;
    options.binary_probe_depth = binary_probe_depth;
    return options;
}

inline int binary_probe_depth(int start_depth,
                              int effective_max_depth,
                              const VirtualSparseFfbOptions& options) {
    if (options.binary_probe_depth >= start_depth &&
        options.binary_probe_depth < effective_max_depth) {
        return options.binary_probe_depth;
    }
    const int span = effective_max_depth - start_depth;
    return span >= 4 ? start_depth + span / 2 : -1;
}

}  // namespace rbf::detail
