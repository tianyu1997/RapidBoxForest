# Modified-DH Symmetry Compression Traits for RBF/LECT

This document records symmetry and compression traits discovered for the
current RBF/LECT implementation.  It is based on the actual
`build_dh_joint()` convention in `link_interval_envelope/src/core/interval_math.cpp`:

```text
A_i = Tx(a_i) Rx(alpha_i) Rz(q_i) Tz(d_i)
```

The primary robot scope is `iiwa14`, `ur5`, and `panda`.  Traits are
robot-specific and must be enabled by robot fingerprint or an explicit
symmetry manifest.  Only exact or conservative relations may be used for
certified pruning; approximate similarities may only guide scheduling or
appendix ablations.

## Current Baseline

The current implementation only supports a joint-0 `Z4_ROTATION`:

```text
q0 -> q0 + k*pi/2
workspace transform = Rz(k*pi/2)
k = 0,1,2,3
```

This is exact for AABB payloads because 90-degree rotations preserve AABB
axis alignment up to coordinate permutation and sign changes.  It is a
restricted discrete version of a broader continuous base-yaw symmetry.

## Exact Traits

### 1. Continuous Base-Yaw Evidence Reuse

For `iiwa14`, `ur5`, and `panda`, changing `q0` by an arbitrary angle rotates
all active endpoint geometry by the same workspace yaw:

```text
FK(q0 + delta, q1, ..., qn) = Rz(delta) FK(q0, q1, ..., qn)
```

Modified-DH point screening residuals for `delta = 0.23, 0.71, 1.0` were
machine precision:

| Robot | max residual |
|---|---:|
| IIWA14 | `3.6e-16` |
| UR5 | `2.8e-16` |
| Panda | `3.4e-16` |

Use:

- exact evidence reuse for support-hull payloads if arbitrary orthogonal
  transforms are supported;
- conservative evidence reuse for AABBs if transformed hulls are re-AABB'd;
- collision label reuse only if the obstacle set is also invariant under the
  same rotation.

### 2. Joint-0 `pi` Shift

The special case

```text
q0 -> q0 + pi
workspace transform = Rz(pi)
```

is exact for all three primary robots and is already contained in the current
Z4 implementation.  For UR5 and Panda, the equivalent action may also flip the
terminal inert joint without changing active endpoint geometry.

### 3. Terminal Spin Quotient

The last revolute joint is position-inert for the current active endpoint /
capsule envelope model:

| Robot | inert joint | note |
|---|---:|---|
| IIWA14 | `q6` | active endpoint geometry unchanged by `q6` |
| UR5 | `q5` | active endpoint geometry unchanged by `q5` |
| Panda | `q6` | active endpoint geometry unchanged by `q6` |

Use:

- avoid splitting the terminal spin dimension for position-only capsule
  evidence;
- widen that interval to the full joint range when validating endpoint/capsule
  envelopes;
- include the tool/collision-geometry identity in the fingerprint, because a
  non-axisymmetric tool or orientation-sensitive model invalidates this trait.

### 4. IIWA Sign-Reflection Actions

Under the current IIWA14 Modified-DH model and active endpoint geometry, the
following pure sign actions are exact.  `q6` is inert, so its sign is irrelevant.

| Joint sign pattern | workspace transform |
|---|---|
| `(-q0,-q1,-q2,-q3,-q4,-q5, +/-q6)` | `Mx = diag(-1, 1, 1)` |
| `(-q0,+q1,-q2,+q3,-q4,+q5, +/-q6)` | `My = diag(1, -1, 1)` |
| `(+q0,-q1,+q2,-q3,+q4,-q5, +/-q6)` | `Mxy = diag(-1, -1, 1)` |
| `(q0,q1,q2,q3,q4,q5, -q6)` | identity, from terminal spin quotient |

Use:

- exact endpoint/support evidence reuse if reflection transforms are supported;
- exact AABB reflection by min/max swap on reflected axes;
- collision labels only when the obstacle set is also symmetric under the same
  reflection.

This is a stronger IIWA-specific family than the current Z4-only
canonicalization, but it must be implemented as robot-specific evidence
canonicalization, not as a planner-level native-space transform.

Special action anchored at `q0 -> q0 + pi, q1 -> -q1`:

```text
q0' = q0 + pi
q1' = -q1
q2' =  q2
q3' = -q3
q4' =  q4
q5' = -q5
q6' = +/-q6
```

This is exact for IIWA active endpoint geometry.  In the screening basis it
appears as `Rz(pi) * Mxy`, which equals the identity workspace transform.  In
other words, the `q0 + pi` base rotation cancels the `xy` sign-reflection
action.  This is not a generic UR/Panda action.

Why this fails for UR5 and Panda:

- UR5 first fails at active link `frame2 -> frame3`.  The distal endpoint error
  reaches `0.85 m`, exactly `2 * |a2|` for `a2 = -0.425`.  The candidate action
  flips the local x direction of that link, but UR5 has a real nonzero
  `Tx(a2)` offset, so the endpoint moves to the opposite side instead of
  matching.
- Panda first fails at active link `frame3 -> frame4`.  The distal endpoint
  error reaches `0.165 m`, exactly `2 * |a3|` for `a3 = 0.0825`; later
  nonzero offsets `a4 = -0.0825` and `a6 = 0.088` accumulate additional error.
- IIWA14 has `a_i = 0` for all joints in the current model.  The same
  orientation flips therefore do not create lateral `Tx(a)` displacement of
  active endpoints.  Frame orientations still differ, but the current
  active-endpoint/capsule envelope is position-based and remains identical.
- If the real IIWA lateral offsets are restored, this exact identity is lost.
  A controlled test with `a2 = +0.0825` and `a4 = -0.0825` gave:

  | Active link | first/second endpoint error under the action |
  |---|---:|
  | `frame2 -> frame3` | `0 / 0.165 m` |
  | `frame4 -> frame5` | up to `0.165 / 0.328 m` |
  | later active links | up to `0.328 m` |

  The two offsets can partially or even fully cancel at the final endpoint for
  special configurations, but the first affected active link already has a
  fixed `2*|a2| = 0.165 m` endpoint discrepancy.  Since RBF validates the
  union of active link envelopes, final-endpoint cancellation is insufficient
  for certified collision reuse.

Leaf-sweep use:

- For IIWA14 and the current position/capsule envelope, this identity action
  can safely reuse validation results because the native workspace geometry is
  unchanged; fixed obstacles do not need to be symmetric.
- This statement is exact only for the current simplified IIWA JSON where all
  lateral `a_i` are zero.  With nonzero lateral offsets it becomes an
  approximate similarity and must not be used for certified pruning.
- It should first be implemented as robot-specific evidence/state reuse:
  validate one interval, then reuse or clone the result for the mapped interval.
- It should not simply delete the mirrored branch from the planner graph unless
  the online planner is also made quotient-aware.  Queries and joint-space
  paths still live in native C-space, so mirrored boxes may need to be
  materialized or virtually represented.
- The `q0 -> q0 + pi` interval must remain inside joint limits.  For IIWA14
  limits `[-2.9668, 2.9668]`, only the pairable subrange
  `q0 in [-2.9668, 2.9668 - pi]` maps forward by `+pi`; the central band near
  zero is unpaired.  A full-period `[-pi, pi]` internal root would make this
  pairing cleaner, but then joint-limit clipping must be explicit.

### 5. Full-Period Boundary Wrapping

For robots whose joint limits are exactly `[-pi, pi]`, periodic boundary cells
at `-pi` and `pi` represent adjacent physical configurations for revolute
joints.

Use:

- topology/neighbor compression near periodic boundaries;
- optional cache lookup normalization for exact full-period joints.

Guardrails:

- do not apply to IIWA limits such as `[-2.9668, 2.9668]`;
- do not merge cache keys across a boundary unless interval wrapping and
  native collision validation are explicit.

## Prefix Contraction Ladder

The user-proposed contraction ladder should be evaluated under the implemented
Modified-DH convention:

```text
P0 = {q0}
P1 = {q0,q1}
P2 = {q0,q1,q2}
...
```

Result:

- `P0`: valid as workspace yaw transform.
- `P1`: preserving `q0 + q1` is not a native identity.
- `P2` and longer prefixes: not valid cumulative-sum quotients.

Random zero-sum perturbations inside each prefix gave:

| Robot | `{q0,q1}` zero-sum err | `{q0,q1,q2}` zero-sum err | Conclusion |
|---|---:|---:|---|
| IIWA14 | `0.379` | `0.337` | fail |
| UR5 | `0.320` | `0.304` | fail |
| Panda | `0.292` | `0.284` | fail |

Therefore, under the current code, there is no certified `(q0,q1)` sum
quotient and no `q0+q1+q2` quotient.  Any previous analysis based on standard
DH multiplication order is invalid for this codebase.

## Active-Link Prefix Dependency Masks

For a serial chain, early active link endpoints do not depend on suffix joints.
This is an exact evidence-materialization compression even though it does not
by itself certify a full C-space box.

Modified-DH active endpoint perturbation gives:

| Robot | Active link map | Effective per-link dependency dimensions |
|---|---|---|
| IIWA14 | `[2, 4, 6, 7]` | `{q0,q1}`, `{q0..q3}`, `{q0..q5}`, `{q0..q5}` |
| UR5 | `[2, 3, 4, 5]` | `{q0,q1}`, `{q0..q2}`, `{q0..q3}`, `{q0..q4}` |
| Panda | `[2, 3, 4, 6, 7]` | `{q0,q1}`, `{q0..q2}`, `{q0..q3}`, `{q0..q5}`, `{q0..q5}` |

Use:

- cache each active link's endpoint/support evidence by its effective
  dependency prefix instead of the full configuration interval;
- reuse earlier-link evidence across boxes that differ only in suffix joints;
- combine with terminal spin quotient where the terminal joint is outside the
  effective dependency mask.

Guardrails:

- final collision classification still needs all active links;
- support-hull payload layout must track which link evidence was reused.

## Conservative Non-Equality Traits

### 1. Transformed-Envelope Dominance

If a transformed cached envelope conservatively contains the live envelope,
then a free cached envelope proves the live interval free:

```text
Envelope(I_live) subset Transform(Envelope(I_cached))
```

This is safe only in the free direction.  If the containing envelope collides
or is unknown, no conclusion follows.

### 2. Scene Symmetry

If the obstacle set is exactly invariant under a workspace transform, collision
labels can be reused as well as envelope payloads.

Examples:

- synthetic fourfold rotational scenes around the base z axis;
- mirror-symmetric scenes under `x -> -x` or `y -> -y`.

Shelf and random scenes should be treated as non-symmetric unless a detector
proves invariance within strict tolerance.

## Explicitly Rejected Candidates

These must not be used for certified pruning:

- `(q0,q1)` sum quotient under the current Modified-DH implementation.
- `q0 -> q0 + pi`, `q1 -> -q1`, with or without later sign flips and `pi`
  offsets, as a general symmetry.  The IIWA-specific alternating-sign action
  listed above is the exact exception; UR5 and Panda failed this anchored
  search.
- `q0,q1 -> -q0,-q1` as a general IIWA symmetry.
- Panda `coupled_pairs` metadata: current code reads it into the robot
  fingerprint only; it is not used by FK, envelope generation, or LECT
  canonicalization.
- Any visual or AABB-volume similarity that does not preserve the actual
  active endpoint/support-hull set.

The `q0 -> q0 + pi`, `q1 -> -q1` family can look similar on special intervals
such as full-period `q0` or symmetric `q1`, but this is an interval-set
coincidence, not a general LECT cell symmetry.

## Screening Summary

| Robot | Continuous `q0` yaw | Terminal spin quotient | Extra pure sign actions | Prefix-sum quotient |
|---|---:|---:|---|---|
| IIWA14 | yes | `q6` | IIWA reflection family above | no |
| UR5 | yes | `q5` | terminal spin only | no |
| Panda | yes | `q6` | terminal spin only | no |

## Recommended Implementation Priority

1. Add a robot-specific symmetry discovery harness:
   - use the implemented Modified-DH matrix;
   - enumerate sign/reflection actions;
   - test active endpoint equality and transformed equality;
   - emit a robot fingerprint-bound symmetry manifest.

2. Implement terminal spin quotient:
   - avoid splitting the inert terminal joint for endpoint/capsule evidence;
   - invalidate when tool/collision geometry becomes orientation-sensitive.

3. Extend evidence transforms beyond Z4:
   - support arbitrary `Rz(delta)` for support hull evidence;
   - support reflections for IIWA sign actions;
   - transform evidence back to native workspace before obstacle collision.

4. Add per-link prefix evidence keys:
   - cache active-link evidence with the effective dependency mask;
   - assemble full envelope classification from reused per-link records.

5. Add conservative transformed-envelope dominance:
   - certify only when a transformed/free superset contains the live envelope.

## Verification Required Before Certified Use

- point FK / active endpoint equality residual below tolerance;
- interval endpoint/envelope equality or conservative containment verified;
- support-hull/GJK transform semantics verified for rotations and reflections;
- transformed payload collision checked in native workspace;
- no planner/experiment layer canonicalizes native query points;
- strict audit remains 100% on Exp.4/Exp.6 smoke runs.
