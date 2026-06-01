"""
ρ_k / Φ_k 坐标变形 vs midpoint 切分 —— robot-only 深度压缩验证实验 (PoC)

目标
----
检验一个"环境无关、缓存安全"的廉价杠杆：把每个关节的切分值从 midpoint 改成
"体积均衡变形" Φ_k 下的中点。维度顺序、二叉结构、运行期切分公式完全不变，
只是在 materialization 时按预计算表挑选切分值。

度量
----
认证深度 (certification depth): 对随机目标构型 q*，沿规范树向 q* 单路径下降，
记录"包住 q* 的盒子的真实 robot-only 包络尺寸 E < τ"首次成立的深度。
深度越小 => 认证盒越大 => 覆盖空间所需盒子越少 (盒数 ~ 2^depth)。

公平性
------
midpoint 与 warped 使用同一组 q*、同一维度调度、同一包络度量。
变形表 Φ_k 仅由机器人本体在标称构型下的扫掠尺寸积分得到，与障碍/场景无关。

包络
----
crit (CritSample) 内界 —— 与论文中用于认证的紧包络一致 (用户关注的量)。
尺寸标量 E(box) = max over (link,endpoint) 的 L1 直径 sum(hi-lo over xyz)。
"""
from __future__ import annotations

import json
import math
import os
import time
from pathlib import Path

import numpy as np

import link_interval_envelope as lie

HERE = Path(__file__).resolve().parent
OUT = HERE / "outputs"
OUT.mkdir(exist_ok=True)
ROBOT_JSON = HERE.parent.parent / "build-rbf-only-exec/python/sbf/data/iiwa14.json"

# iiwa14 关节限位 (来自 sbf.Robot.joint_limits)
LIMITS = [
    (-2.9668, 2.9668),
    (-2.0942, 2.0942),
    (-2.9668, 2.9668),
    (-2.0942, 2.0942),
    (-2.9668, 2.9668),
    (-2.0942, 2.0942),
    (-3.0541, 3.0541),
]
NJ = len(LIMITS)

# robot-only 体积最小维度调度 (来自 sbf.aafk_volume_min_depth_schedule, 近端关节优先)
SCHEDULE = [5, 2, 4, 3, 0, 1, 5, 2, 1, 3, 0, 4, 2, 0, 1, 3, 4, 2, 0, 1,
            3, 5, 2, 0, 4, 3, 1, 5, 3, 2, 1, 4, 3, 5, 0, 2, 1, 4, 3, 5]

CRIT_SAMPLES = 1500
WARP_GRID = 64          # Φ_k 网格点数
WARP_HALFWIDTH = 0.15   # 标称扫掠窗口半宽 (rad)


def make_robot():
    return lie.load_robot(str(ROBOT_JSON))


def envelope_size(robot, box, cfg) -> float:
    """E(box) = max over (link,endpoint) 的 L1 直径 (sum of xyz extents)."""
    info = lie.compute_endpoint_iaabb_info(robot, box, endpoint_config=cfg,
                                           output_mode="full")
    ia = info["endpoint_iaabbs"]  # flat, shape (L,2,6)
    n = len(ia) // 6
    worst = 0.0
    for i in range(n):
        b = ia[i * 6:i * 6 + 6]
        d = (b[3] - b[0]) + (b[4] - b[1]) + (b[5] - b[2])
        if d > worst:
            worst = d
    return worst


# --------------------------------------------------------------------------
# Φ_k 构造: ρ_k(θ) = 标称构型 (其余关节=0) 下, 关节 k 在 [θ-h,θ+h] 窗口的包络尺寸
# 这反映"轴对齐 AABB 随关节角的三角非线性" —— 沿坐标轴方向时增长慢, 45° 时增长快.
# Φ_k = ρ_k 的归一化累计积分; 逆变形 W_k(u) 用线性插值.
# --------------------------------------------------------------------------
def build_warp(robot, cfg):
    warps = []
    for k in range(NJ):
        lo, hi = LIMITS[k]
        thetas = np.linspace(lo, hi, WARP_GRID)
        rho = np.zeros(WARP_GRID)
        for j, th in enumerate(thetas):
            box = []
            for d in range(NJ):
                if d == k:
                    a = max(lo, th - WARP_HALFWIDTH)
                    b = min(hi, th + WARP_HALFWIDTH)
                    box.append([a, b])
                else:
                    box.append([0.0, 0.0])
            rho[j] = envelope_size(robot, box, cfg)
        # 累计积分 -> Φ
        cdf = np.concatenate([[0.0], np.cumsum(0.5 * (rho[1:] + rho[:-1]) * np.diff(thetas))])
        if cdf[-1] <= 0:
            cdf = thetas - lo  # 退化 -> 恒等
        phi = cdf / cdf[-1]
        warps.append({"theta": thetas, "phi": phi, "rho": rho.copy()})
    return warps


def phi_of(warp, theta):
    return float(np.interp(theta, warp["theta"], warp["phi"]))


def inv_phi(warp, u):
    return float(np.interp(u, warp["phi"], warp["theta"]))


def split_value(warp, lo, hi, mode):
    if mode == "midpoint":
        return 0.5 * (lo + hi)
    # warped: 在 Φ 坐标下取中点
    um = 0.5 * (phi_of(warp, lo) + phi_of(warp, hi))
    m = inv_phi(warp, um)
    # 数值保护
    if not (lo < m < hi):
        m = 0.5 * (lo + hi)
    return m


# --------------------------------------------------------------------------
# 单路径下降: 沿 SCHEDULE 向 q* 细分, 记录 E<τ 的首次深度
# --------------------------------------------------------------------------
def certification_depth(robot, cfg, warps, q, mode, taus, max_depth):
    box = [list(LIMITS[d]) for d in range(NJ)]
    results = {tau: None for tau in taus}
    remaining = set(taus)
    for depth in range(max_depth):
        dim = SCHEDULE[depth % len(SCHEDULE)]
        lo, hi = box[dim]
        m = split_value(warps[dim], lo, hi, mode)
        if q[dim] <= m:
            box[dim][1] = m
        else:
            box[dim][0] = m
        E = envelope_size(robot, box, cfg)
        done = [tau for tau in remaining if E < tau]
        for tau in done:
            results[tau] = depth + 1
            remaining.discard(tau)
        if not remaining:
            break
    return results


def main():
    rng = np.random.default_rng(12345)
    robot = make_robot()
    cfg = lie.make_endpoint_config("crit", n_samples_crit=CRIT_SAMPLES)

    print("[1/3] 构造 robot-only 变形表 Φ_k ...")
    t0 = time.time()
    warps = build_warp(robot, cfg)
    print(f"    完成, 用时 {time.time()-t0:.2f}s")
    for k in range(NJ):
        w = warps[k]
        # 报告变形非线性强度: warped 中点相对 midpoint 的偏移 (在全范围根盒上)
        lo, hi = LIMITS[k]
        wm = inv_phi(w, 0.5)
        mid = 0.5 * (lo + hi)
        print(f"    joint{k}: rho[min,max]=({w['rho'].min():.3f},{w['rho'].max():.3f}) "
              f"warp-median={wm:+.3f} vs midpoint={mid:+.3f} 偏移={wm-mid:+.3f}")

    # 根包络尺寸用于选 τ
    root_box = [list(LIMITS[d]) for d in range(NJ)]
    root_E = envelope_size(robot, root_box, cfg)
    print(f"[info] 根盒 crit 包络尺寸 E0 = {root_E:.3f}")
    taus = [0.40, 0.25, 0.15, 0.08]

    S = 80
    max_depth = 36
    print(f"[2/3] 单路径下降 ({S} 个随机 q*, max_depth={max_depth}) ...")
    rows = {"midpoint": {tau: [] for tau in taus}, "warped": {tau: [] for tau in taus}}
    t0 = time.time()
    for s in range(S):
        q = np.array([rng.uniform(lo, hi) for (lo, hi) in LIMITS])
        for mode in ("midpoint", "warped"):
            res = certification_depth(robot, cfg, warps, q, mode, taus, max_depth)
            for tau in taus:
                rows[mode][tau].append(res[tau])
        if (s + 1) % 20 == 0:
            print(f"    {s+1}/{S}  ({time.time()-t0:.1f}s)")

    print("[3/3] 结果汇总 (认证深度, 越小越好):")
    summary = {"taus": taus, "n_samples": S, "crit_samples": CRIT_SAMPLES,
               "root_E": root_E, "by_tau": {}}
    print(f"\n  {'tau':>6} | {'midpoint depth':>26} | {'warped depth':>26} | {'Δmed':>6} | {'box×':>6}")
    print("  " + "-" * 86)
    for tau in taus:
        def stats(vals):
            arr = [v for v in vals if v is not None]
            miss = len(vals) - len(arr)
            if not arr:
                return None, None, None, miss
            return float(np.median(arr)), float(np.mean(arr)), float(np.percentile(arr, 90)), miss
        m_med, m_mean, m_p90, m_miss = stats(rows["midpoint"][tau])
        w_med, w_mean, w_p90, w_miss = stats(rows["warped"][tau])
        if m_med is None or w_med is None:
            print(f"  {tau:>6.2f} | (未达标过多)  miss mid={m_miss} warp={w_miss}")
            continue
        dmed = w_med - m_med
        boxfac = 2.0 ** (m_med - w_med)  # warped 相对 midpoint 的盒数比 (<1 = 更少)
        print(f"  {tau:>6.2f} | med={m_med:>5.1f} mean={m_mean:>5.1f} p90={m_p90:>5.1f}"
              f" miss={m_miss:>2} | med={w_med:>5.1f} mean={w_mean:>5.1f} p90={w_p90:>5.1f}"
              f" miss={w_miss:>2} | {dmed:>+5.1f} | {boxfac:>5.2f}×")
        summary["by_tau"][f"{tau:.2f}"] = {
            "midpoint": {"median": m_med, "mean": m_mean, "p90": m_p90, "miss": m_miss},
            "warped": {"median": w_med, "mean": w_mean, "p90": w_p90, "miss": w_miss},
            "delta_median_depth": dmed,
            "box_ratio_warped_over_midpoint": boxfac,
        }

    (OUT / "warp_vs_midpoint_result.json").write_text(json.dumps(summary, indent=2))
    print(f"\n[saved] {OUT/'warp_vs_midpoint_result.json'}")
    print("\n说明: box× = 2^(mid_med - warp_med) = warped 相对 midpoint 的覆盖盒数比, <1 表示更少.")


if __name__ == "__main__":
    main()
