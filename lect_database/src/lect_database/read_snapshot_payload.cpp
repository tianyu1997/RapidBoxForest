#include "read_snapshot_payload.h"

#include <cstring>

namespace rbf::lect_database {
namespace {

// Decode an IEEE binary16 (half) value to float32. Mirrors the encoder used by
// the authoritative evidence store (lect_database/database.cpp). Snapshot
// payloads are stored as outward-rounded halves (schema v4); decode on read.
float f32_from_f16(std::uint16_t h) {
    const std::uint32_t sign = static_cast<std::uint32_t>(h & 0x8000u) << 16;
    const std::uint32_t exp = (h >> 10) & 0x1fu;
    const std::uint32_t mant = h & 0x3ffu;
    std::uint32_t bits;
    if (exp == 0) {
        if (mant == 0) {
            bits = sign;  // +/- zero
        } else {
            // Subnormal half -> normalized float.
            std::uint32_t m = mant;
            std::int32_t e = -1;
            do {
                m <<= 1;
                ++e;
            } while ((m & 0x400u) == 0);
            m &= 0x3ffu;
            const std::uint32_t fexp = static_cast<std::uint32_t>(127 - 15 - e);
            bits = sign | (fexp << 23) | (m << 13);
        }
    } else if (exp == 0x1fu) {
        bits = sign | 0x7f800000u | (mant << 13);  // inf / nan
    } else {
        const std::uint32_t fexp = exp + (127u - 15u);
        bits = sign | (fexp << 23) | (mant << 13);
    }
    float out;
    std::memcpy(&out, &bits, sizeof(out));
    return out;
}

}  // namespace

std::shared_ptr<std::vector<float>> decode_half_payload(const std::uint16_t* base,
                                                        std::size_t count) {
    auto buffer = std::make_shared<std::vector<float>>();
    buffer->resize(count);
    for (std::size_t i = 0; i < count; ++i) {
        std::uint16_t h;
        std::memcpy(&h, base + i, sizeof(h));
        (*buffer)[i] = f32_from_f16(h);
    }
    return buffer;
}

}  // namespace rbf::lect_database
