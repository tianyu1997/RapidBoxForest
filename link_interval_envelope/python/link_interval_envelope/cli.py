from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from . import GcpcCache, compute_envelope, compute_from_endpoint_iaabbs, write_json
from .visualize import save_html


def _load_json_payload(path: str | Path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def _load_intervals(args: argparse.Namespace):
    if args.intervals_json:
        payload = json.loads(args.intervals_json)
    elif args.intervals_file:
        payload = _load_json_payload(args.intervals_file)
    else:
        raise SystemExit("provide --intervals-json or --intervals-file")
    if isinstance(payload, dict):
        payload = payload.get("intervals")
    if not isinstance(payload, list):
        raise SystemExit("interval payload must be a list or an object with an 'intervals' list")
    return payload


def _load_endpoint_iaabbs(path: str | Path):
    payload = _load_json_payload(path)
    if isinstance(payload, dict):
        if isinstance(payload.get("endpoint"), dict):
            endpoint_payload = payload["endpoint"]
            if "endpoint_iaabbs_flat" in endpoint_payload:
                return endpoint_payload["endpoint_iaabbs_flat"]
            if "endpoint_iaabbs" in endpoint_payload:
                return endpoint_payload["endpoint_iaabbs"]
        for key in ("endpoint_iaabbs_flat", "endpoint_iaabbs", "data"):
            if key in payload:
                payload = payload[key]
                break
    return payload


def _parse_hifk_depth(value: str) -> int:
    key = value.strip().lower()
    if key == "auto":
        return -1
    return int(value)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Compute and visualize link interval envelopes")
    sub = parser.add_subparsers(dest="command")
    compute = sub.add_parser("compute", help="compute one interval envelope")
    compute.add_argument("--robot", required=True, help="robot JSON path")
    compute.add_argument("--intervals-json", help="JSON list like [[lo,hi], ...]")
    compute.add_argument("--intervals-file", help="JSON file with intervals")
    compute.add_argument("--endpoint-iaabbs-file", help="precomputed endpoint iAABB JSON/flat list")
    compute.add_argument("--endpoint-source", default="ifk", choices=["ifk", "hifk", "critsample", "crit", "analytical", "gcpc", "mc"])
    compute.add_argument("--n-samples-crit", type=int, default=1000)
    compute.add_argument("--endpoint-threads", type=int, default=1, help="internal CritSample enumeration threads")
    compute.add_argument("--parallel-min-combos", type=int, default=0, help="minimum CritSample combos before internal parallelism; <=0 selects an automatic threshold")
    compute.add_argument("--max-phase-analytical", type=int, default=3)
    compute.add_argument("--bypass-narrow-skip", action="store_true")
    compute.add_argument("--gcpc-match-analytical", action="store_true")
    compute.add_argument("--hifk-max-depth", type=_parse_hifk_depth, default=9, help="HIFK total bisection depth, or 'auto' for interval-aware scheduling")
    compute.add_argument("--hifk-n-threads", type=int, default=1, help="reserved HIFK worker count")
    compute.add_argument("--hifk-vol-ratio-thresh", type=float, default=0.0, help="adaptive HIFK split threshold; 0 uses fixed-depth mode")
    compute.add_argument("--gcpc-cache", help="GCPC cache file for --endpoint-source gcpc")
    compute.add_argument("--env", default="link_iaabb", choices=["link_iaabb", "kdop", "support_hull"])
    compute.add_argument("--n-sub", type=int, default=1)
    compute.add_argument("--out-json", help="write result JSON")
    compute.add_argument("--out-html", help="write interactive HTML visualization")
    compute.add_argument("--view", default="inflated", choices=["raw", "inflated"])
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.command is None:
        args.command = "compute"

    gcpc_cache = GcpcCache.load(args.gcpc_cache) if args.gcpc_cache else None
    if args.endpoint_iaabbs_file:
        endpoint_iaabbs = _load_endpoint_iaabbs(args.endpoint_iaabbs_file)
        result = compute_from_endpoint_iaabbs(
            args.robot,
            endpoint_iaabbs,
            envelope_type=args.env,
            n_subdivisions=args.n_sub,
        )
    else:
        intervals = _load_intervals(args)
        result = compute_envelope(
            args.robot,
            intervals,
            endpoint_source=args.endpoint_source,
            envelope_type=args.env,
            n_subdivisions=args.n_sub,
            n_samples_crit=args.n_samples_crit,
            endpoint_threads=args.endpoint_threads,
            parallel_min_combos=args.parallel_min_combos,
            max_phase_analytical=args.max_phase_analytical,
            bypass_narrow_skip=args.bypass_narrow_skip,
            gcpc_match_analytical=args.gcpc_match_analytical,
            hifk_max_depth=args.hifk_max_depth,
            hifk_n_threads=args.hifk_n_threads,
            hifk_vol_ratio_thresh=args.hifk_vol_ratio_thresh,
            gcpc_cache=gcpc_cache,
        )

    if args.out_json:
        write_json(result, args.out_json)
    else:
        print(json.dumps(result, indent=2))
    if args.out_html:
        save_html(result, args.out_html, view=args.view)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
