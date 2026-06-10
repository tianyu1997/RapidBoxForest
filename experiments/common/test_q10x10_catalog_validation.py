#!/usr/bin/env python3
from __future__ import annotations

import unittest

from experiments.common.assemble_q10x10_prefix_catalog import validate_records


def record(difficulty: str, scene_seed: int, obstacles: list[list[float]], query_shift: float = 0.0) -> dict:
    query = {
        "start": [query_shift, 0.0],
        "goal": [1.0 + query_shift, 0.0],
        "canonical_start": [query_shift, 0.0],
        "canonical_goal": [1.0 + query_shift, 0.0],
    }
    return {
        "schema": "tro2026_random_scene_catalog_v7",
        "robot": "iiwa",
        "difficulty": difficulty,
        "scene_seed": scene_seed,
        "obstacles": obstacles,
        "queries": [query for _ in range(10)],
        "difficulty_probe": {
            "policy": "distribution_separation_v1",
            "ok": True,
            "rrtconnect": {"success_fraction": 1.0, "median_first_success_s": 0.1},
            "bitstar": {"success_fraction": 1.0, "median_first_success_s": 0.1},
        },
    }


class Q10x10CatalogValidationTest(unittest.TestCase):
    def test_accepts_strict_prefix_and_shared_queries(self) -> None:
        records = [
            record("easy", 0, []),
            record("medium", 0, [[0.0, 1.0]]),
            record("hard", 0, [[0.0, 1.0], [1.0, 2.0]]),
        ]
        report = validate_records(
            records,
            robots=["iiwa"],
            difficulties=["easy", "medium", "hard"],
            scene_seeds=1,
            queries_per_scene=10,
            allow_extra_records=False,
        )
        self.assertTrue(report["ok"], report["errors"])

    def test_rejects_non_prefix_obstacles(self) -> None:
        records = [
            record("easy", 0, []),
            record("medium", 0, [[0.0, 1.0]]),
            record("hard", 0, [[2.0, 3.0], [1.0, 2.0]]),
        ]
        report = validate_records(
            records,
            robots=["iiwa"],
            difficulties=["easy", "medium", "hard"],
            scene_seeds=1,
            queries_per_scene=10,
            allow_extra_records=False,
        )
        self.assertFalse(report["ok"])
        self.assertTrue(any("not an extension" in error for error in report["errors"]))

    def test_rejects_difficulty_query_mismatch(self) -> None:
        records = [
            record("easy", 0, []),
            record("medium", 0, [[0.0, 1.0]], query_shift=0.1),
            record("hard", 0, [[0.0, 1.0], [1.0, 2.0]]),
        ]
        report = validate_records(
            records,
            robots=["iiwa"],
            difficulties=["easy", "medium", "hard"],
            scene_seeds=1,
            queries_per_scene=10,
            allow_extra_records=False,
        )
        self.assertFalse(report["ok"])
        self.assertTrue(any("queries differ" in error for error in report["errors"]))


if __name__ == "__main__":
    unittest.main()
