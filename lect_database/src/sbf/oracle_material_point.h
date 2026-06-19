#pragma once

#include <LECTDatabase/sbf/oracle.h>

#include <optional>
#include <vector>

namespace rbf {

std::optional<MaterialPointOccupiedWitness> try_material_point_occupied_witness(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    const LinkEnvelope& envelope,
    const Scene& scene,
    const OccupiedCertificateConfig& config);

}  // namespace rbf
