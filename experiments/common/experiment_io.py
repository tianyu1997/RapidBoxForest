from __future__ import annotations

import argparse
import importlib.util
import json
import os
import platform
import socket
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Sequence


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_OUTPUT_ROOT = REPO_ROOT / "outputs" / "new_experiments"


def csv_list(raw: str) -> list[str]:
    return [item.strip() for item in str(raw).split(",") if item.strip()]


def csv_ints(raw: str) -> list[int]:
    return [int(item) for item in csv_list(raw)]


def as_jsonable(value: Any) -> Any:
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {str(key): as_jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [as_jsonable(item) for item in value]
    return value


def namespace_dict(args: argparse.Namespace) -> dict[str, Any]:
    return {key: as_jsonable(value) for key, value in vars(args).items()}


def write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")


def append_jsonl(path: Path, row: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(row, sort_keys=True) + "\n")


def git_sha() -> str:
    try:
        result = subprocess.run(
            ["git", "rev-parse", "--short=12", "HEAD"],
            cwd=str(REPO_ROOT),
            text=True,
            capture_output=True,
            check=True,
        )
    except Exception:
        return "unknown"
    return result.stdout.strip() or "unknown"


def run_id(prefix: str) -> str:
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return f"{prefix}_{stamp}_{git_sha()}"


def proc_status() -> dict[str, int]:
    status_path = Path("/proc/self/status")
    if not status_path.exists():
        return {}
    wanted = {"VmRSS", "VmHWM", "VmSize", "VmData"}
    out: dict[str, int] = {}
    for line in status_path.read_text(encoding="utf-8", errors="replace").splitlines():
        key, sep, rest = line.partition(":")
        if sep and key in wanted:
            parts = rest.strip().split()
            if parts:
                out[f"{key}_kb"] = int(float(parts[0]))
    return out


def environment_metadata() -> dict[str, Any]:
    thread_env = {
        key: os.environ.get(key)
        for key in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS")
        if os.environ.get(key) is not None
    }
    return {
        "git_sha": git_sha(),
        "python": sys.version.split()[0],
        "platform": platform.platform(),
        "machine": platform.machine(),
        "processor": platform.processor(),
        "hostname": socket.gethostname(),
        "cwd": str(Path.cwd()),
        "repo_root": str(REPO_ROOT),
        "thread_env": thread_env,
        "proc_status": proc_status(),
    }


def default_sbf_subprocess_env() -> dict[str, str]:
    env: dict[str, str] = {}
    candidates = [
        REPO_ROOT / "build-rbf-only-exec",
        REPO_ROOT / "build-rbf-python-current",
        REPO_ROOT / "build-consolidated-python",
    ]
    build_dir = next((candidate for candidate in candidates if (candidate / "python" / "sbf").exists()), None)
    if build_dir is not None:
        env["SBF_BUILD_DIR"] = str(build_dir)
    repo_parent = REPO_ROOT.parent
    existing_pythonpath = os.environ.get("PYTHONPATH", "")
    entries = [str(repo_parent)]
    if existing_pythonpath:
        entries.append(existing_pythonpath)
    env["PYTHONPATH"] = os.pathsep.join(entries)
    return env


def command_record(command: Sequence[str], *, cwd: Path | None = None) -> dict[str, Any]:
    return {"command": [str(part) for part in command], "cwd": str(cwd or REPO_ROOT)}


def run_command(
    command: Sequence[str],
    *,
    cwd: Path | None = None,
    dry_run: bool = False,
    extra_env: dict[str, str] | None = None,
) -> dict[str, Any]:
    command_text = [str(part) for part in command]
    record = command_record(command_text, cwd=cwd)
    record["dry_run"] = bool(dry_run)
    if dry_run:
        record["returncode"] = None
        record["wall_s"] = 0.0
        return record
    env = os.environ.copy()
    if extra_env:
        env.update({str(key): str(value) for key, value in extra_env.items()})
    before = proc_status()
    t0 = time.perf_counter()
    completed = subprocess.run(
        command_text,
        cwd=str(cwd or REPO_ROOT),
        env=env,
        text=True,
        capture_output=True,
        check=False,
    )
    wall_s = time.perf_counter() - t0
    after = proc_status()
    record.update({
        "returncode": int(completed.returncode),
        "wall_s": float(wall_s),
        "stdout_tail": completed.stdout[-8000:],
        "stderr_tail": completed.stderr[-8000:],
        "proc_status_before": before,
        "proc_status_after": after,
    })
    return record


def load_module_from_path(module_name: str, path: Path) -> Any:
    spec = importlib.util.spec_from_file_location(module_name, str(path))
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load module {module_name!r} from {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module
