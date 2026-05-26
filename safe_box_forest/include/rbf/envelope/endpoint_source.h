#pragma once

#include <sbf/envelope/endpoint_source.h>

#include <cstdint>

namespace rbf {

struct SourceQualityMetadata {
    std::uint8_t hifk_max_depth = 0;
    std::uint8_t max_phase_analytical = 0;
    std::uint16_t n_threads = 1;
};

inline SourceQualityMetadata make_quality_metadata(const EndpointSourceConfig& cfg) {
    SourceQualityMetadata metadata;
    metadata.hifk_max_depth = static_cast<std::uint8_t>(cfg.hifk_max_depth);
    metadata.max_phase_analytical = static_cast<std::uint8_t>(cfg.max_phase_analytical);
    metadata.n_threads = static_cast<std::uint16_t>(cfg.n_threads);
    return metadata;
}

inline bool cached_source_dominates(EndpointSource cached_source,
                                    const SourceQualityMetadata& cached_meta,
                                    EndpointSource requested_source,
                                    const EndpointSourceConfig& requested_cfg) {
    if (!source_can_serve(cached_source, requested_source)) {
        return false;
    }
    if (cached_source == EndpointSource::HIFK && requested_source == EndpointSource::HIFK) {
        return cached_meta.hifk_max_depth >= static_cast<std::uint8_t>(requested_cfg.hifk_max_depth);
    }
    if (cached_source == EndpointSource::Analytical && requested_source == EndpointSource::Analytical) {
        return cached_meta.max_phase_analytical >= static_cast<std::uint8_t>(requested_cfg.max_phase_analytical);
    }
    if (cached_source == EndpointSource::CritSample && requested_source == EndpointSource::CritSample) {
        return cached_meta.n_threads >= static_cast<std::uint16_t>(requested_cfg.n_threads);
    }
    return true;
}

}  // namespace rbf
