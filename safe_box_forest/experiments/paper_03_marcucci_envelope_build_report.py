#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = ROOT.parents[1]


PIPELINE_NAMES = {
    "crit_link_coverage": "LinkIAABB hot path",
    "coverage_hybrid": "Hull/Grid pipeline",
    "ifk_strict": "IFK strict",
}


def esc(value: Any) -> str:
    text = str(value)
    replacements = {
        "\\": r"\textbackslash{}",
        "&": r"\&",
        "%": r"\%",
        "$": r"\$",
        "#": r"\#",
        "_": r"\_",
        "{": r"\{",
        "}": r"\}",
        "~": r"\textasciitilde{}",
        "^": r"\textasciicircum{}",
    }
    for old, new in replacements.items():
        text = text.replace(old, new)
    return text


def fmt(value: Any, digits: int = 3, null: str = "--") -> str:
    if value is None:
        return null
    try:
        number = float(value)
    except (TypeError, ValueError):
        return esc(value)
    if abs(number) >= 1000.0 or (0.0 < abs(number) < 0.001):
        return f"{number:.{digits}e}"
    return f"{number:.{digits}f}"


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def path_for_tex(path: Path) -> str:
    resolved = path.resolve()
    try:
        resolved = resolved.relative_to(REPO_ROOT)
    except ValueError:
        pass
    return str(resolved).replace("\\", "/")


def query_sr_from_v6(v6: dict[str, Any]) -> float | None:
    queries = v6.get("queries", [])
    if not queries:
        return None
    return sum(float(row.get("sr", 0.0)) for row in queries) / len(queries)


def unique_cross_scene_rows(payload: dict[str, Any]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    seen: set[tuple[Any, Any, Any]] = set()
    for trial in payload.get("trials", []):
        if trial.get("protocol") != "cross_scene" or not trial.get("prewarm"):
            continue
        metadata = trial["prewarm"].get("scene_metadata", {})
        key = (metadata.get("scene_kind"), metadata.get("seed"), metadata.get("difficulty"))
        if key in seen:
            continue
        seen.add(key)
        rows.append({
            "source_scene": trial["prewarm"].get("scene"),
            "kind": metadata.get("scene_kind", trial["prewarm"].get("scene")),
            "difficulty": metadata.get("difficulty", "--"),
            "seed": metadata.get("seed", "--"),
            "obstacles": metadata.get("obstacle_count", "--"),
            "attempts": metadata.get("placement_attempts", "--"),
            "endpoints": metadata.get("endpoint_count", "--"),
            "clearance": metadata.get("clearance", "--"),
            "ok": metadata.get("endpoint_clearance_ok", False),
        })
    return rows


def make_summary_table(payload: dict[str, Any]) -> str:
    lines = [
        r"\begin{table}[h]",
        r"\centering",
        r"\caption{Standalone Exp.3 cache protocol summary.}",
        r"\scriptsize",
        r"\resizebox{\linewidth}{!}{%",
        r"\begin{tabular}{llrrrrrrrrr}",
        r"\toprule",
        r"Pipeline & Protocol & Build s & Prewarm s & Boxes & Vol. & Audit SR & Raw SR & $t_{\mathrm{read}}$ $\mu$s & Grid mats & Cache MB \\",
        r"\midrule",
    ]
    summaries = payload.get("summaries", {})
    for variant, protocols in summaries.items():
        for protocol, row in protocols.items():
            cache_mb = None if row.get("cache_file_bytes_mean") is None else float(row.get("cache_file_bytes_mean", 0.0)) / 1_000_000.0
            lines.append(
                f"{esc(PIPELINE_NAMES.get(variant, variant))} & {esc(protocol)} & "
                f"{fmt(row.get('build_mean_s'))} & {fmt(row.get('prewarm_build_mean_s'))} & "
                f"{fmt(row.get('box_count_mean'), 1)} & {fmt(row.get('volume_sum_mean'), 1)} & "
                f"{fmt(row.get('audited_query_sr'))} & {fmt(row.get('raw_query_sr'))} & "
                f"{fmt(row.get('t_read_us_mean'))} & {fmt(row.get('lazy_grid_materializations_mean'), 1)} & "
                f"{fmt(cache_mb)} \\\\")
    lines.extend([r"\bottomrule", r"\end{tabular}%", r"}", r"\end{table}"])
    return "\n".join(lines)


def make_v6_table(payload: dict[str, Any], v6: dict[str, Any] | None) -> str:
    comparison = payload.get("comparison_to_v6") or {}
    v6_build = comparison.get("v6_build_mean_s")
    v6_boxes = None
    v6_volume = None
    v6_query_sr = None
    if v6 is not None:
        build = v6.get("build", {})
        v6_build = build.get("mean_s", v6_build)
        v6_boxes = build.get("mean_unique_box_count")
        v6_volume = build.get("mean_dedup_box_volume_sum")
        v6_query_sr = query_sr_from_v6(v6)
    lines = [
        r"\begin{table}[h]",
        r"\centering",
        r"\caption{Reference against the v6 paper-facing Exp.3 artifact.}",
        r"\scriptsize",
        r"\begin{tabular}{lrrrr}",
        r"\toprule",
        r"Source & Build s & Boxes & Volume & Query SR \\",
        r"\midrule",
        f"v6 retained LinkIAABB S=4 & {fmt(v6_build)} & {fmt(v6_boxes, 1)} & {fmt(v6_volume, 1)} & {fmt(v6_query_sr)} \\\\",
    ]
    summaries = payload.get("summaries", {})
    for variant, protocols in summaries.items():
        cold = protocols.get("cold")
        if cold is None:
            continue
        lines.append(
            f"Standalone {esc(PIPELINE_NAMES.get(variant, variant))} cold & "
            f"{fmt(cold.get('build_mean_s'))} & {fmt(cold.get('box_count_mean'), 1)} & "
            f"{fmt(cold.get('volume_sum_mean'), 1)} & {fmt(cold.get('audited_query_sr'))} \\\\")
    lines.extend([r"\bottomrule", r"\end{tabular}", r"\end{table}"])
    return "\n".join(lines)


def make_cross_scene_table(payload: dict[str, Any]) -> str:
    rows = unique_cross_scene_rows(payload)
    lines = [
        r"\begin{table}[h]",
        r"\centering",
        r"\caption{Random cross-scene source validation. Clearance means endpoint collision-free after obstacle inflation by the listed margin.}",
        r"\scriptsize",
        r"\begin{tabular}{llrrrrr}",
        r"\toprule",
        r"Kind & Difficulty & Seed & Obs. & Attempts & Clearance & Endpoints clear \\",
        r"\midrule",
    ]
    if not rows:
        lines.append(r"-- & -- & -- & -- & -- & -- & -- \\")
    for row in rows:
        lines.append(
            f"{esc(row['kind'])} & {esc(row['difficulty'])} & {esc(row['seed'])} & "
            f"{esc(row['obstacles'])} & {esc(row['attempts'])} & {fmt(row['clearance'], 2)} & "
            f"{esc('yes' if row['ok'] else 'no')} \\\\")
    lines.extend([r"\bottomrule", r"\end{tabular}", r"\end{table}"])
    return "\n".join(lines)


def make_tex(payload: dict[str, Any], v6: dict[str, Any] | None, json_path: Path) -> str:
    params = payload.get("params", {})
    variants = ", ".join(PIPELINE_NAMES.get(item, item) for item in params.get("variants", []))
    protocols = ", ".join(params.get("protocols", []))
    json_display_path = path_for_tex(json_path)
    target_scene = (params.get("target_scene_metadata") or {}).get("scene_kind", params.get("target_scene"))
    return rf"""\documentclass[UTF8,11pt]{{ctexart}}
\usepackage[landscape,a4paper,margin=1.15cm]{{geometry}}
\usepackage{{booktabs}}
\usepackage{{array}}
\usepackage{{graphicx}}
\usepackage{{hyperref}}
\usepackage{{xcolor}}
\usepackage{{url}}
\hypersetup{{colorlinks=true,linkcolor=blue,urlcolor=blue}}
\setlength{{\tabcolsep}}{{4pt}}
\renewcommand{{\arraystretch}}{{1.12}}

\title{{Standalone SBF Exp.3 Cache and Grid Pipeline Report}}
\author{{Standalone SBF}}
\date{{2026-05-05}}

\begin{{document}}
\maketitle

\section{{实验口径}}
本报告整合 standalone SBF Exp.3 的 cache protocol 结果，并以 v6 paper-facing artifact 作为参考。数据源为 \path{{{json_display_path}}}；v6 reference 为 \path{{cpp/v6/experiments/results_paper/marcucci_combined.json}}。

本次 standalone 协议包含 \texttt{{cold}}、\texttt{{warm}} 与 \texttt{{cross\_scene}}。\texttt{{cold}} 使用 fresh LECT cache namespace 构建 v6 对齐的 Marcucci combined scene；\texttt{{warm}} 在同一 namespace 下重复 Marcucci combined scene，用来测同场景 cache amortization；\texttt{{cross\_scene}} 先在随机生成源场景预热 cache，再回到 Marcucci combined target scene 构建，用来测跨场景迁移。为了避免 warm/cross 已分裂 LECT 树改变 root selection，本脚本在 Exp.3 cache protocol 下禁用 \texttt{{root\_seed\_max\_lca\_depth}}。

Pipelines: {esc(variants)}。Protocols: {esc(protocols)}。目标场景为 \texttt{{{esc(target_scene)}}}；cross-scene source 为 \texttt{{{esc(params.get('cross_source_scene'))}}}；grid pad policy 为 \texttt{{{esc(params.get('grid_pad_policy'))}}}；storage profile 为 \texttt{{{esc(params.get('storage_profile'))}}}。

\section{{v6 参考}}
v6 Exp.3 retained build variant 是 CritSample + LinkIAABB($S=4$) 的 non-grid hot path。Standalone 的 \texttt{{crit\_link\_coverage}} 对齐这一方向，但当前 grower/connector 仍是独立实现；\texttt{{coverage\_hybrid}} 是 CritSample + Hull/Grid 的 grid pipeline，本次默认使用 conservative half-diagonal grid pad。

{make_v6_table(payload, v6)}

\section{{Cache Protocol 主结果}}
{make_summary_table(payload)}

\section{{随机 Cross-Scene 与 Clearance}}
随机 cross-scene 不再使用 shelves/table 等固定场景，而是在 Marcucci IIWA robot 下生成随机 AABB 障碍物。每个候选障碍物加入前都会将全部障碍物按 clearance margin 膨胀，再检查 AS、TS、CS、LB、RB 五个 query endpoints 是否仍 collision-free。该检查保证 start/goal endpoints 对随机源场景有指定 margin 的 conservative AABB clearance；它不声明整条路径有同等 clearance。

{make_cross_scene_table(payload)}

\section{{结论}}
\paragraph{{Match-route 的定位。}}
Match-route 仍然有诊断价值，用来隔离同一路径、同一 split route 下 cache hit 是否生效；但 standalone Exp.3 的主问题是 cache amortization 与跨场景迁移，所以 cold/warm/cross-scene 更适合作为主表。随机 cross-scene 进一步避免了 shelves 到 combined 的人工场景相关性。

\paragraph{{Grid pipeline。}}
Hull/Grid pipeline 已纳入同一 Exp.3 protocol。与 LinkIAABB hot path 相比，它给出 grid materialization、grid collision 与 $t_{{\mathrm{{read}}}}$ 统计；若 warm 明显快于 cold，说明 cache 中的 LECT split/evidence/grid payload 对同场景复用有效。Cross-scene 若没有相似加速，则说明当前 cache transfer 主要依赖目标场景的实际 occupancy/evidence replay，而非单纯 robot fingerprint。

\paragraph{{注意事项。}}
Standalone query SR 和 v6 paper cached-query SR 不是完全相同口径：v6 artifact 包含 paper-facing cached-query/bridge policy，standalone runner 只验证当前独立 SBF build/query path。因此本报告优先解读 build/cache/grid 统计，query SR 作为 sanity check。

\end{{document}}
"""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate TeX report for standalone Exp.3 cache/grid results.")
    parser.add_argument("--json", type=Path, default=ROOT / "outputs" / "paper" / "marcucci_envelope_build_standalone.json")
    parser.add_argument("--v6-json", type=Path, default=REPO_ROOT / "cpp" / "v6" / "experiments" / "results_paper" / "marcucci_combined.json")
    parser.add_argument("--out-tex", type=Path, default=ROOT / "outputs" / "paper" / "marcucci_envelope_build_report.tex")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    payload = load_json(args.json)
    v6 = load_json(args.v6_json) if args.v6_json.exists() else None
    tex = make_tex(payload, v6, args.json)
    args.out_tex.parent.mkdir(parents=True, exist_ok=True)
    args.out_tex.write_text(tex, encoding="utf-8")
    print(json.dumps({"out_tex": str(args.out_tex)}, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())