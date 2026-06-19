#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

namespace rbf::lect_database {

std::shared_ptr<std::vector<float>> decode_half_payload(const std::uint16_t* base,
                                                        std::size_t count);

}  // namespace rbf::lect_database
