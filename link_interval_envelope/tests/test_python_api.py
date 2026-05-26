from __future__ import annotations

import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from link_interval_envelope import (
    IncrementalEnvelopeComputer,
    compute_envelope,
    compute_envelope_batch,
    compute_from_endpoint_iaabbs,
    recommend_hifk_depth,
)


ROOT = Path(__file__).resolve().parents[1]
ROBOT = ROOT / "examples" / "data" / "2dof_planar.json"


class LinkIntervalEnvelopePythonTests(unittest.TestCase):
    def test_one_shot_schema(self) -> None:
        result = compute_envelope(
            ROBOT,
            [[-0.4, 0.4], [-0.2, 0.2]],
            endpoint_source="ifk",
            envelope_type="link_iaabb",
            n_subdivisions=4,
        )
        self.assertEqual(result["schema"], "link_interval_envelope.v1")
        self.assertEqual(result["robot"]["n_joints"], 2)
        self.assertEqual(result["envelope"]["shape"], [2, 4, 6])
        self.assertTrue(result["endpoint"]["is_safe"])
        self.assertEqual(result["endpoint"]["source"], "IFK")

    def test_hifk_depth_zero_matches_ifk(self) -> None:
        intervals = [[-0.4, 0.4], [-0.2, 0.2]]
        ifk_result = compute_envelope(
            ROBOT,
            intervals,
            endpoint_source="ifk",
            envelope_type="link_iaabb",
            n_subdivisions=4,
        )
        hifk_result = compute_envelope(
            ROBOT,
            intervals,
            endpoint_source="hifk",
            hifk_max_depth=0,
            envelope_type="link_iaabb",
            n_subdivisions=4,
        )
        self.assertTrue(hifk_result["endpoint"]["is_safe"])
        self.assertEqual(hifk_result["endpoint"]["source"], "HIFK")
        self.assertEqual(
            ifk_result["endpoint"]["endpoint_iaabbs_flat"],
            hifk_result["endpoint"]["endpoint_iaabbs_flat"],
        )

    def test_recommend_hifk_depth_schedule(self) -> None:
        self.assertEqual(recommend_hifk_depth(ROBOT, [[-0.025, 0.025], [-0.02, 0.02]]), 0)
        self.assertEqual(recommend_hifk_depth(ROBOT, [[-0.1, 0.1], [-0.05, 0.05]]), 3)
        self.assertEqual(recommend_hifk_depth(ROBOT, [[-0.25, 0.25], [-0.25, 0.25]]), 5)

    def test_recommend_hifk_depth_distinguishes_split_dimension_effect(self) -> None:
        depth_joint0 = recommend_hifk_depth(ROBOT, [[-0.25, 0.25], [-0.02, 0.02]])
        depth_joint1 = recommend_hifk_depth(ROBOT, [[-0.02, 0.02], [-0.25, 0.25]])
        self.assertEqual(depth_joint0, 5)
        self.assertEqual(depth_joint1, 3)

    def test_hifk_auto_depth_matches_recommended_explicit_depth(self) -> None:
        intervals = [[-0.25, 0.25], [-0.25, 0.25]]
        scheduled_depth = recommend_hifk_depth(ROBOT, intervals)
        auto_result = compute_envelope(
            ROBOT,
            intervals,
            endpoint_source="hifk",
            hifk_max_depth="auto",
            envelope_type="link_iaabb",
            n_subdivisions=4,
        )
        explicit_result = compute_envelope(
            ROBOT,
            intervals,
            endpoint_source="hifk",
            hifk_max_depth=scheduled_depth,
            envelope_type="link_iaabb",
            n_subdivisions=4,
        )
        self.assertEqual(auto_result["endpoint"]["source"], "HIFK")
        self.assertEqual(
            auto_result["endpoint"]["endpoint_iaabbs_flat"],
            explicit_result["endpoint"]["endpoint_iaabbs_flat"],
        )

    def test_incremental_matches_full_recompute(self) -> None:
        first = [[-0.4, 0.4], [-0.2, 0.2]]
        second = [[-0.4, 0.4], [-0.1, 0.3]]
        computer = IncrementalEnvelopeComputer(
            ROBOT,
            endpoint_source="ifk",
            envelope_type="link_iaabb",
            n_subdivisions=4,
        )
        cold = computer.compute(first)
        self.assertFalse(cold["incremental"]["used_incremental_fk"])
        self.assertFalse(computer.has_valid_fk())
        incremental = computer.compute(second)
        full = compute_envelope(
            ROBOT,
            second,
            endpoint_source="ifk",
            envelope_type="link_iaabb",
            n_subdivisions=4,
        )
        self.assertEqual(incremental["incremental"]["changed_dim"], 1)
        self.assertFalse(incremental["incremental"]["used_incremental_fk"])
        self.assertEqual(
            incremental["endpoint"]["endpoint_iaabbs_flat"],
            full["endpoint"]["endpoint_iaabbs_flat"],
        )
        self.assertEqual(
            incremental["envelope"]["link_iaabbs_flat"],
            full["envelope"]["link_iaabbs_flat"],
        )

    def test_critsample_incremental_matches_full_recompute(self) -> None:
        first = [[-0.4, 0.4], [-0.2, 0.2]]
        second = [[-0.4, 0.4], [-0.1, 0.3]]
        computer = IncrementalEnvelopeComputer(
            ROBOT,
            endpoint_source="critsample",
            envelope_type="link_iaabb",
            n_subdivisions=4,
        )
        cold = computer.compute(first)
        self.assertFalse(cold["incremental"]["used_source_incremental_state"])
        self.assertTrue(computer.has_valid_fk())
        incremental = computer.compute(second)
        full = compute_envelope(
            ROBOT,
            second,
            endpoint_source="critsample",
            envelope_type="link_iaabb",
            n_subdivisions=4,
        )
        self.assertEqual(incremental["incremental"]["changed_dim"], 1)
        self.assertTrue(incremental["incremental"]["used_source_incremental_state"])
        self.assertGreaterEqual(incremental["diagnostics"]["candidate_dirty_count"], 1)
        self.assertGreaterEqual(incremental["diagnostics"]["predh_rebuild_count"], 1)
        self.assertEqual(
            incremental["endpoint"]["endpoint_iaabbs_flat"],
            full["endpoint"]["endpoint_iaabbs_flat"],
        )
        self.assertEqual(
            incremental["envelope"]["link_iaabbs_flat"],
            full["envelope"]["link_iaabbs_flat"],
        )
        cached = computer.compute(second)
        self.assertTrue(cached["incremental"]["reused_endpoint_cache"])
        self.assertTrue(cached["diagnostics"]["endpoint_cache_reused"])
        self.assertEqual(
            cached["endpoint"]["endpoint_iaabbs_flat"],
            full["endpoint"]["endpoint_iaabbs_flat"],
        )

    def test_endpoint_reuse_from_result_json(self) -> None:
        result = compute_envelope(
            ROBOT,
            [[-0.3, 0.3], [-0.1, 0.1]],
            endpoint_source="ifk",
            envelope_type="link_iaabb",
            n_subdivisions=2,
        )
        reused = compute_from_endpoint_iaabbs(
            ROBOT,
            result["endpoint"]["endpoint_iaabbs_flat"],
            envelope_type="link_iaabb",
            n_subdivisions=2,
        )
        self.assertEqual(
            reused["envelope"]["link_iaabbs_flat"],
            result["envelope"]["link_iaabbs_flat"],
        )

    def test_critsample_parallel_diagnostics(self) -> None:
        intervals = [[-0.4, 0.4], [-0.1, 0.3]]
        serial = compute_envelope(
            ROBOT,
            intervals,
            endpoint_source="critsample",
            envelope_type="link_iaabb_grid",
            n_subdivisions=4,
            endpoint_threads=1,
        )
        parallel = compute_envelope(
            ROBOT,
            intervals,
            endpoint_source="critsample",
            envelope_type="link_iaabb_grid",
            n_subdivisions=4,
            endpoint_threads=2,
            parallel_min_combos=1,
        )
        self.assertGreater(serial["diagnostics"]["combo_count"], 0)
        self.assertEqual(parallel["diagnostics"]["enumerate_threads"], 2)
        self.assertEqual(parallel["diagnostics"]["parallel_min_combos_used"], 1)
        self.assertGreaterEqual(parallel["diagnostics"]["enumerate_chunk_count"], 1)
        self.assertEqual(
            serial["endpoint"]["endpoint_iaabbs_flat"],
            parallel["endpoint"]["endpoint_iaabbs_flat"],
        )
        grid = parallel["envelope"]["grid"]
        self.assertGreaterEqual(grid["fill_time_us"], 0.0)
        self.assertGreaterEqual(grid["capacity"], grid["n_bricks"])
        self.assertGreater(grid["range_write_count"], 0)
        self.assertGreater(grid["brick_write_count"], 0)

    def test_ifk_hifk_and_critsample_batch_matches_sequential(self) -> None:
        boxes = [
            [[-0.4, 0.4], [-0.2, 0.2]],
            [[-0.4, 0.4], [-0.1, 0.3]],
            [[-0.2, 0.5], [-0.25, 0.15]],
        ]
        for source in ("ifk", "critsample", "hifk"):
            sequential = [
                compute_envelope(
                    ROBOT,
                    box,
                    endpoint_source=source,
                    envelope_type="link_iaabb",
                    n_subdivisions=4,
                    hifk_max_depth=0,
                )
                for box in boxes
            ]
            for n_threads in (1, 2):
                batch = compute_envelope_batch(
                    ROBOT,
                    boxes,
                    endpoint_source=source,
                    envelope_type="link_iaabb",
                    n_subdivisions=4,
                    hifk_max_depth=0,
                    n_threads=n_threads,
                )
                self.assertEqual(len(batch), len(sequential))
                for got, expected in zip(batch, sequential):
                    self.assertEqual(
                        got["endpoint"]["endpoint_iaabbs_flat"],
                        expected["endpoint"]["endpoint_iaabbs_flat"],
                    )
                    self.assertEqual(
                        got["envelope"]["link_iaabbs_flat"],
                        expected["envelope"]["link_iaabbs_flat"],
                    )

    def test_cli_json_and_html(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            out_json = Path(tmp) / "lie.json"
            out_html = Path(tmp) / "lie.html"
            cmd = [
                sys.executable,
                "-m",
                "link_interval_envelope",
                "compute",
                "--robot",
                str(ROBOT),
                "--intervals-json",
                "[[-0.4, 0.4], [-0.2, 0.2]]",
                "--endpoint-source",
                "hifk",
                "--hifk-max-depth",
                "auto",
                "--hifk-vol-ratio-thresh",
                "0.0",
                "--env",
                "link_iaabb",
                "--n-sub",
                "4",
                "--out-json",
                str(out_json),
                "--out-html",
                str(out_html),
            ]
            subprocess.run(cmd, check=True, cwd=ROOT, env=os.environ.copy())
            payload = json.loads(out_json.read_text(encoding="utf-8"))
            self.assertEqual(payload["schema"], "link_interval_envelope.v1")
            self.assertEqual(payload["endpoint"]["source"], "HIFK")
            self.assertIn("Plotly.newPlot", out_html.read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()
