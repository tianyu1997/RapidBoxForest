#pragma once
/// @file aa_fk.h
/// @brief Affine-Arithmetic Forward Kinematics (AA-FK) for endpoint iAABB.
///
/// AA-FK propagates a 4x4 DH chain using first-order affine arithmetic.
/// The low-level module is named `aa_fk`, while the public standalone endpoint
/// sources built on top of it are `IFK` and `HIFK`.

#include <sbf/core/types.h>
#include <sbf/core/robot.h>

#include <algorithm>
#include <cmath>

namespace rbf {

struct AffineScalar {
    double ctr = 0.0;
    double lin[MAX_JOINTS] = {};
    double rem_lo = 0.0;
    double rem_hi = 0.0;

    void to_interval(int n_joints, double& lo, double& hi) const {
        double radius = 0.0;
        for (int joint = 0; joint < n_joints; ++joint) {
            radius += std::abs(lin[joint]);
        }
        lo = ctr - radius + rem_lo;
        hi = ctr + radius + rem_hi;
    }
};

struct AffineMatrix4 {
    AffineScalar m[16];
};

void compute_endpoint_iaabb_aa_fk_raw(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    float* out);

void aa_fk_update_prefix_from(
    AffineMatrix4* prefix,
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int from_joint,
    int n_joints);

void aa_fk_extend_prefix(
    AffineMatrix4* prefix,
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int from_joint,
    int to_joint,
    int n_joints);

void aa_fk_extract_from_prefix(
    const AffineMatrix4* prefix,
    const int* active_link_map,
    int n_active_links,
    int n_joints,
    float* out);

/// Stateful AA-FK chain prefix enabling incremental endpoint recomputation
/// along a parent->child descent. When exactly one joint interval changes
/// between two consecutive calls, only the affected suffix of the kinematic
/// chain is recomputed; the produced endpoint iAABBs are bit-identical to a
/// full AA-FK pass.
struct AaFkPrefixState {
    AffineMatrix4 prefix[MAX_TF];
    std::vector<Interval> intervals;
    int n_joints = 0;
    bool valid = false;
};

/// Full AA-FK endpoint pass that also records the chain prefix in @p state so a
/// subsequent single-dimension change can be served incrementally.
void aa_fk_endpoint_full(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    float* out,
    AaFkPrefixState& state);

/// Incremental AA-FK endpoint pass. When @p state is valid and @p intervals
/// differ from the cached intervals in exactly one dimension (bit-exact on all
/// others), only the affected suffix of the chain is recomputed and @p out is
/// filled with a result bit-identical to a full pass; returns true. Otherwise
/// @p out is left untouched and the function returns false.
bool aa_fk_endpoint_incremental(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    float* out,
    AaFkPrefixState& state);

}  // namespace rbf