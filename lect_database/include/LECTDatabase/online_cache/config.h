#pragma once

#include <cstddef>

namespace rbf::lect_database {

struct OnlineEnvelopeCacheConfig {
    std::size_t max_nodes = 0;
    std::size_t max_payload_bytes = 64u * 1024u * 1024u;
    bool allow_database_backfill = true;
};

}  // namespace rbf::lect_database
