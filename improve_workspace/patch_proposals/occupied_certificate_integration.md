# Occupied Certificate Production Integration Proposal

This note tracks production integration for `docs/improve.md` Section 2.2.
The default-disabled oracle scaffold and classifier branch have been applied;
the remaining production work is a real signed-distance/material-point witness
source.

## Current Production State

Relevant source anchors:

- `lect_database/include/LECTDatabase/sbf/oracle.h`
  - `BoxValidation` contains `Occupied`.
  - `OccupiedCertificateConfig` is available and disabled by default.
  - `MaterialPointOccupiedWitness` records the material-point witness fields.
  - `OracleValidationDetail` and `OracleCounters` expose occupied-certificate
    diagnostics.
- `lect_database/src/sbf/oracle.cpp`
  - `DatabaseBoxOracle::classify_payload(...)` has a guarded
    `BoxValidation::Occupied` branch.
  - The current production witness provider returns no witness because the
    obstacle interface exposes AABBs but not reliable SDF/material-point
    penetration witnesses.
- `link_interval_envelope/include/sbf/envelope/envelope_collision.h`
  - Provides envelope collision and overlap statistics, but not a material-point
    signed-distance witness.

Therefore the source anchor is wired, but current runs still produce no
occupied cells unless a future witness provider supplies a sound certificate.

## Required New Types

Implemented disabled-by-default witness config:

```cpp
struct OccupiedCertificateConfig {
    bool enabled = false;
    double numerical_epsilon = 1e-9;
    double min_penetration_margin = 0.0;
};
```

Implemented `OracleValidationConfig` extension:

```cpp
OccupiedCertificateConfig occupied_certificate;
```

Implemented witness record, mirroring the sidecar `MaterialWitness`:

```cpp
struct MaterialPointOccupiedWitness {
    int link_id = -1;
    int obstacle_id = -1;
    Eigen::Vector3d link_point;
    double center_signed_distance = 0.0;
    double motion_bound = 0.0;
    double epsilon_num = 1e-9;
};
```

## Remaining Required Checker Interface

Do not infer occupied cells from AABB, KDOP, support-hull, or GJK overlap alone.
The production checker needs an explicit SDF/material-point witness source:

```cpp
std::optional<MaterialPointOccupiedWitness>
try_material_point_occupied_witness(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    const LinkEnvelope& envelope,
    const Scene& scene,
    const OccupiedCertificateConfig& config);
```

The predicate is:

```cpp
center_signed_distance + motion_bound + epsilon_num < 0.0
```

where `motion_bound` is a conservative bound on workspace displacement of the
same material point over the whole joint box.

## Oracle Integration Point

`DatabaseBoxOracle::classify_payload(...)` now calls the occupied-certificate
hook after `collide_envelope_aabbs` returns `Maybe` and before returning
`Unknown`:

```cpp
if (validation_config_.occupied_certificate.enabled) {
    auto witness = try_material_point_occupied_witness(...);
    if (witness && witness->center_signed_distance +
                   witness->motion_bound +
                   witness->epsilon_num < 0.0) {
        last_validation_detail_.validation = BoxValidation::Occupied;
        last_validation_detail_.collision_possible = true;
        last_validation_detail_.strict_audit_required = false;
        counters_.certified_occupied += 1;
        return BoxValidation::Occupied;
    }
}
```

`certified_occupied` is a counter distinct from `collision_possible`.
Occupied cells must not enter the free forest; they can only be used as
terminal prune domains.

## Safety Rules

- Default config is `enabled=false`.
- If no reliable SDF/material-point witness exists, return `Unknown`.
- Never classify occupied from broadphase or support-hull overlap alone.
- The existing conservative free certificate remains unchanged.
- The occupied-pruning lemma should be documented as optional, not required by
  the main soundness theorem.

## Tests

Current and remaining C++ unit tests:

- AABB/support-hull overlap without witness returns `Unknown`: implemented in
  `lect_database/tests/test_sbf_adapter.cpp`.
- Strong witness returns `BoxValidation::Occupied`: pending real/test witness
  provider.
- Weak witness returns `BoxValidation::Unknown`: pending real/test witness
  provider.
- `enabled=false` never returns `Occupied`: covered by default behavior and
  should be made explicit when a test witness provider is added.
- Occupied terminal cells are not inserted into `boxes_`: pending production
  adaptive terminal-cell integration.

## Experiment Use

For paper experiments, report occupied-certificate pruning only as an optional
ablation unless the production witness source is available for every tested
robot/scene pair.
