#include <sbf/core/aa_fk.h>
#include <sbf/core/interval_math.h>

#include <algorithm>
#include <cmath>
#include <cstring>

namespace rbf {

namespace {

static void affine_identity(AffineMatrix4& matrix, int n_joints) {
    for (int index = 0; index < 16; ++index) {
        matrix.m[index].ctr = 0.0;
        matrix.m[index].rem_lo = 0.0;
        matrix.m[index].rem_hi = 0.0;
        for (int joint = 0; joint < n_joints; ++joint) {
            matrix.m[index].lin[joint] = 0.0;
        }
    }
    matrix.m[0].ctr = 1.0;
    matrix.m[5].ctr = 1.0;
    matrix.m[10].ctr = 1.0;
    matrix.m[15].ctr = 1.0;
}

static void affine_build_dh_joint(
    const Robot& robot,
    int joint_idx,
    const Interval& interval,
    AffineMatrix4& matrix,
    int n_joints)
{
    const auto& dh = robot.dh_params()[joint_idx];

    for (int index = 0; index < 16; ++index) {
        matrix.m[index].ctr = 0.0;
        matrix.m[index].rem_lo = 0.0;
        matrix.m[index].rem_hi = 0.0;
        for (int joint = 0; joint < n_joints; ++joint) {
            matrix.m[index].lin[joint] = 0.0;
        }
    }
    matrix.m[15].ctr = 1.0;

    const double ca = std::cos(dh.alpha);
    const double sa = std::sin(dh.alpha);

    if (dh.joint_type == 0) {
        const double theta_mid = (interval.lo + interval.hi) * 0.5 + dh.theta;
        const double delta = (interval.hi - interval.lo) * 0.5;

        const double ct0 = std::cos(theta_mid);
        const double st0 = std::sin(theta_mid);
        const double d_ct_lin = -st0 * delta;
        const double d_st_lin = ct0 * delta;
        const double trig_rem = delta * delta * 0.5;
        const double d_val = dh.d;

        matrix.m[0].ctr = ct0;
        matrix.m[0].lin[joint_idx] = d_ct_lin;
        matrix.m[0].rem_lo = -trig_rem;
        matrix.m[0].rem_hi = trig_rem;

        matrix.m[1].ctr = -st0;
        matrix.m[1].lin[joint_idx] = -d_st_lin;
        matrix.m[1].rem_lo = -trig_rem;
        matrix.m[1].rem_hi = trig_rem;
        matrix.m[3].ctr = dh.a;

        matrix.m[4].ctr = st0 * ca;
        matrix.m[4].lin[joint_idx] = d_st_lin * ca;
        matrix.m[4].rem_lo = -trig_rem * std::abs(ca);
        matrix.m[4].rem_hi = trig_rem * std::abs(ca);

        matrix.m[5].ctr = ct0 * ca;
        matrix.m[5].lin[joint_idx] = d_ct_lin * ca;
        matrix.m[5].rem_lo = -trig_rem * std::abs(ca);
        matrix.m[5].rem_hi = trig_rem * std::abs(ca);

        matrix.m[6].ctr = -sa;
        matrix.m[7].ctr = -d_val * sa;

        matrix.m[8].ctr = st0 * sa;
        matrix.m[8].lin[joint_idx] = d_st_lin * sa;
        matrix.m[8].rem_lo = -trig_rem * std::abs(sa);
        matrix.m[8].rem_hi = trig_rem * std::abs(sa);

        matrix.m[9].ctr = ct0 * sa;
        matrix.m[9].lin[joint_idx] = d_ct_lin * sa;
        matrix.m[9].rem_lo = -trig_rem * std::abs(sa);
        matrix.m[9].rem_hi = trig_rem * std::abs(sa);

        matrix.m[10].ctr = ca;
        matrix.m[11].ctr = d_val * ca;
    } else {
        const double theta = dh.theta;
        const double ct0 = std::cos(theta);
        const double st0 = std::sin(theta);
        const double d_mid = (interval.lo + interval.hi) * 0.5 + dh.d;
        const double delta = (interval.hi - interval.lo) * 0.5;

        matrix.m[0].ctr = ct0;
        matrix.m[1].ctr = -st0;
        matrix.m[3].ctr = dh.a;

        matrix.m[4].ctr = st0 * ca;
        matrix.m[5].ctr = ct0 * ca;
        matrix.m[6].ctr = -sa;
        matrix.m[7].ctr = -d_mid * sa;
        matrix.m[7].lin[joint_idx] = -delta * sa;

        matrix.m[8].ctr = st0 * sa;
        matrix.m[9].ctr = ct0 * sa;
        matrix.m[10].ctr = ca;
        matrix.m[11].ctr = d_mid * ca;
        matrix.m[11].lin[joint_idx] = delta * ca;
    }
}

static void affine_mat_mul(
    const AffineMatrix4& lhs,
    const AffineMatrix4& rhs,
    AffineMatrix4& out,
    int n_joints)
{
    out.m[12].ctr = 0.0;
    out.m[13].ctr = 0.0;
    out.m[14].ctr = 0.0;
    out.m[15].ctr = 1.0;
    for (int index = 12; index < 16; ++index) {
        out.m[index].rem_lo = 0.0;
        out.m[index].rem_hi = 0.0;
        for (int joint = 0; joint < n_joints; ++joint) {
            out.m[index].lin[joint] = 0.0;
        }
    }

    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 4; ++col) {
            AffineScalar& result = out.m[row * 4 + col];
            result.ctr = 0.0;
            result.rem_lo = 0.0;
            result.rem_hi = 0.0;
            for (int joint = 0; joint < n_joints; ++joint) {
                result.lin[joint] = 0.0;
            }

            for (int k = 0; k < 3; ++k) {
                const AffineScalar& te = lhs.m[row * 4 + k];
                const AffineScalar& ae = rhs.m[k * 4 + col];

                result.ctr += te.ctr * ae.ctr;
                for (int joint = 0; joint < n_joints; ++joint) {
                    result.lin[joint] += te.ctr * ae.lin[joint] + te.lin[joint] * ae.ctr;
                }

                double te_rad = 0.0;
                for (int joint = 0; joint < n_joints; ++joint) {
                    te_rad += std::abs(te.lin[joint]);
                }
                const double te_rem_rad = std::max(std::abs(te.rem_lo), std::abs(te.rem_hi));

                double ae_rad = 0.0;
                for (int joint = 0; joint < n_joints; ++joint) {
                    ae_rad += std::abs(ae.lin[joint]);
                }
                const double ae_rem_rad = std::max(std::abs(ae.rem_lo), std::abs(ae.rem_hi));

                const double cross = te_rad * ae_rad;
                const double rem_extra = cross
                    + te_rem_rad * (std::abs(ae.ctr) + ae_rad + ae_rem_rad)
                    + te_rad * ae_rem_rad
                    + std::abs(te.ctr) * ae_rem_rad
                    + te_rem_rad * ae_rem_rad;

                result.rem_lo -= rem_extra;
                result.rem_hi += rem_extra;
            }

            if (col == 3) {
                const AffineScalar& te3 = lhs.m[row * 4 + 3];
                result.ctr += te3.ctr;
                for (int joint = 0; joint < n_joints; ++joint) {
                    result.lin[joint] += te3.lin[joint];
                }
                result.rem_lo += te3.rem_lo;
                result.rem_hi += te3.rem_hi;
            }
        }
    }
}

static void affine_extract_endpoint_iaabbs(
    const AffineMatrix4* prefix,
    const int* active_link_map,
    int n_active_links,
    int n_joints,
    float* out)
{
    for (int link = 0; link < n_active_links; ++link) {
        const int link_idx = active_link_map[link];

        float* proximal = out + (link * 2) * 6;
        {
            double lo = 0.0;
            double hi = 0.0;
            prefix[link_idx].m[3].to_interval(n_joints, lo, hi);
            proximal[0] = static_cast<float>(lo);
            proximal[3] = static_cast<float>(hi);
            prefix[link_idx].m[7].to_interval(n_joints, lo, hi);
            proximal[1] = static_cast<float>(lo);
            proximal[4] = static_cast<float>(hi);
            prefix[link_idx].m[11].to_interval(n_joints, lo, hi);
            proximal[2] = static_cast<float>(lo);
            proximal[5] = static_cast<float>(hi);
        }

        float* distal = out + (link * 2 + 1) * 6;
        {
            double lo = 0.0;
            double hi = 0.0;
            prefix[link_idx + 1].m[3].to_interval(n_joints, lo, hi);
            distal[0] = static_cast<float>(lo);
            distal[3] = static_cast<float>(hi);
            prefix[link_idx + 1].m[7].to_interval(n_joints, lo, hi);
            distal[1] = static_cast<float>(lo);
            distal[4] = static_cast<float>(hi);
            prefix[link_idx + 1].m[11].to_interval(n_joints, lo, hi);
            distal[2] = static_cast<float>(lo);
            distal[5] = static_cast<float>(hi);
        }
    }
}

}  // namespace

void compute_endpoint_iaabb_aa_fk_raw(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    float* out)
{
    const int n_joints = robot.n_joints();
    const int n_active_links = robot.n_active_links();
    const int* active_link_map = robot.active_link_map();

    AffineMatrix4 prefix[MAX_TF];
    AffineMatrix4 joint_matrix;

    affine_identity(prefix[0], n_joints);

    for (int joint = 0; joint < n_joints; ++joint) {
        affine_build_dh_joint(robot, joint, intervals[joint], joint_matrix, n_joints);
        affine_mat_mul(prefix[joint], joint_matrix, prefix[joint + 1], n_joints);
    }

    if (robot.has_tool()) {
        const auto& tool = *robot.tool_frame();
        affine_identity(joint_matrix, n_joints);
        const double ca = std::cos(tool.alpha);
        const double sa = std::sin(tool.alpha);
        const double ct = std::cos(tool.theta);
        const double st = std::sin(tool.theta);
        joint_matrix.m[0].ctr = ct;
        joint_matrix.m[1].ctr = -st;
        joint_matrix.m[3].ctr = tool.a;
        joint_matrix.m[4].ctr = st * ca;
        joint_matrix.m[5].ctr = ct * ca;
        joint_matrix.m[6].ctr = -sa;
        joint_matrix.m[7].ctr = -tool.d * sa;
        joint_matrix.m[8].ctr = st * sa;
        joint_matrix.m[9].ctr = ct * sa;
        joint_matrix.m[10].ctr = ca;
        joint_matrix.m[11].ctr = tool.d * ca;
        affine_mat_mul(prefix[n_joints], joint_matrix, prefix[n_joints + 1], n_joints);
    }

    affine_extract_endpoint_iaabbs(prefix, active_link_map, n_active_links, n_joints, out);
}

void aa_fk_extend_prefix(
    AffineMatrix4* prefix,
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int from_joint,
    int to_joint,
    int n_joints)
{
    AffineMatrix4 joint_matrix;
    for (int joint = from_joint; joint < to_joint; ++joint) {
        affine_build_dh_joint(robot, joint, intervals[joint], joint_matrix, n_joints);
        affine_mat_mul(prefix[joint], joint_matrix, prefix[joint + 1], n_joints);
    }
}

void aa_fk_update_prefix_from(
    AffineMatrix4* prefix,
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int from_joint,
    int n_joints)
{
    aa_fk_extend_prefix(prefix, robot, intervals, from_joint, n_joints, n_joints);

    if (robot.has_tool()) {
        const auto& tool = *robot.tool_frame();
        AffineMatrix4 joint_matrix;
        affine_identity(joint_matrix, n_joints);
        const double ca = std::cos(tool.alpha);
        const double sa = std::sin(tool.alpha);
        const double ct = std::cos(tool.theta);
        const double st = std::sin(tool.theta);
        joint_matrix.m[0].ctr = ct;
        joint_matrix.m[1].ctr = -st;
        joint_matrix.m[3].ctr = tool.a;
        joint_matrix.m[4].ctr = st * ca;
        joint_matrix.m[5].ctr = ct * ca;
        joint_matrix.m[6].ctr = -sa;
        joint_matrix.m[7].ctr = -tool.d * sa;
        joint_matrix.m[8].ctr = st * sa;
        joint_matrix.m[9].ctr = ct * sa;
        joint_matrix.m[10].ctr = ca;
        joint_matrix.m[11].ctr = tool.d * ca;
        affine_mat_mul(prefix[n_joints], joint_matrix, prefix[n_joints + 1], n_joints);
    }
}

void aa_fk_extract_from_prefix(
    const AffineMatrix4* prefix,
    const int* active_link_map,
    int n_active_links,
    int n_joints,
    float* out)
{
    affine_extract_endpoint_iaabbs(prefix, active_link_map, n_active_links, n_joints, out);
}

}  // namespace rbf