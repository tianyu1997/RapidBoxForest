from __future__ import annotations

import hashlib
import json
import re
import time
from pathlib import Path
from typing import Any


def slugify(value: Any, *, limit: int = 96) -> str:
    text = re.sub(r"[^A-Za-z0-9_.-]+", "_", str(value)).strip("_")
    if not text:
        text = "row"
    return text[:limit]


def planned_row_fingerprint(row: dict[str, Any]) -> str:
    relevant = {
        "method": row.get("method"),
        "robot": row.get("robot"),
        "difficulty": row.get("difficulty"),
        "scene_seed": row.get("scene_seed"),
        "stage_id": row.get("stage_id"),
        "budget_s": row.get("budget_s"),
        "deep_max_boxes": row.get("deep_max_boxes"),
        "queries_per_scene": row.get("queries_per_scene"),
        "audit_segment_step": row.get("audit_segment_step"),
        "audit_collision_tolerance": row.get("audit_collision_tolerance"),
        "ompl_simplify_time_s": row.get("ompl_simplify_time_s"),
        "prm_config": row.get("prm_config"),
        "bitstar_config": row.get("bitstar_config"),
        "rbf_default_profile": row.get("rbf_default_profile"),
    }
    encoded = json.dumps(relevant, sort_keys=True, default=str, separators=(",", ":"))
    return hashlib.sha1(encoded.encode("utf-8")).hexdigest()[:16]


def planned_row_part_path(checkpoint_dir: Path, row: dict[str, Any]) -> Path:
    stem = "_".join([
        slugify(row.get("method", "method"), limit=24),
        slugify(row.get("robot", "robot"), limit=16),
        slugify(row.get("difficulty", "difficulty"), limit=16),
        f"s{int(row.get('scene_seed', 0))}",
        slugify(row.get("stage_id", "stage"), limit=72),
        planned_row_fingerprint(row),
    ])
    return checkpoint_dir / f"{stem}.json"


def load_result_part(path: Path) -> list[dict[str, Any]] | None:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return None
    if not isinstance(payload, dict) or payload.get("status") != "complete":
        return None
    rows = payload.get("rows")
    if not isinstance(rows, list):
        return None
    return [dict(row) for row in rows if isinstance(row, dict)]


def write_result_part(path: Path, planned_row: dict[str, Any], rows: list[dict[str, Any]]) -> None:
    payload = {
        "status": "complete",
        "fingerprint": planned_row_fingerprint(planned_row),
        "planned_row": planned_row,
        "row_count": len(rows),
        "rows": rows,
        "written_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(payload, indent=2, sort_keys=True, default=str), encoding="utf-8")
    tmp.replace(path)
