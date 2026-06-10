#!/usr/bin/env python3
from __future__ import annotations

import unittest

from experiments.common.generate_prefix_mapped_workspace_catalog import select_distribution_prefixes_from_scan


def row(count: int, composite: tuple[float, float, float], rrt: tuple[float, float, float], bitstar: tuple[float, float, float]) -> dict:
    def metrics(values: tuple[float, float, float]) -> dict:
        q25, median, q75 = values
        return {
            "times_s": [q25, median, q75],
            "success_fraction": 1.0,
            "median_s": median,
            "q25_s": q25,
            "q75_s": q75,
            "log_median": 0.0,
        }

    return {
        "count": count,
        "metrics": {
            "policy": "distribution_separation_v1",
            "selectable": True,
            "rrtconnect": metrics(rrt),
            "bitstar": metrics(bitstar),
            "composite": {
                "times_s": list(composite),
                "median_s": composite[1],
                "q25_s": composite[0],
                "q75_s": composite[2],
                "log_median": 0.0,
            },
        },
    }


class DistributionPrefixSelectorTest(unittest.TestCase):
    def test_selects_separated_ordered_prefixes(self) -> None:
        scan = [
            row(0, (0.005, 0.006, 0.007), (0.004, 0.005, 0.006), (0.005, 0.006, 0.007)),
            row(5, (0.040, 0.050, 0.060), (0.030, 0.050, 0.070), (0.040, 0.050, 0.060)),
            row(9, (0.130, 0.160, 0.190), (0.120, 0.160, 0.200), (0.130, 0.160, 0.190)),
        ]
        selected = select_distribution_prefixes_from_scan(scan)
        self.assertEqual(selected["prefix_counts"], {"easy": 0, "medium": 5, "hard": 9})

    def test_nonmonotonic_prefix_times_can_be_skipped(self) -> None:
        scan = [
            row(0, (0.005, 0.006, 0.007), (0.004, 0.005, 0.006), (0.005, 0.006, 0.007)),
            row(1, (0.200, 0.250, 0.300), (0.200, 0.250, 0.300), (0.200, 0.250, 0.300)),
            row(2, (0.040, 0.050, 0.060), (0.030, 0.050, 0.070), (0.040, 0.050, 0.060)),
            row(3, (0.130, 0.160, 0.190), (0.120, 0.160, 0.200), (0.130, 0.160, 0.190)),
        ]
        selected = select_distribution_prefixes_from_scan(scan)
        self.assertEqual(selected["prefix_counts"], {"easy": 0, "medium": 2, "hard": 3})

    def test_selection_is_count_prefix_nested_even_with_unsorted_input(self) -> None:
        scan = [
            row(9, (0.130, 0.160, 0.190), (0.120, 0.160, 0.200), (0.130, 0.160, 0.190)),
            row(0, (0.005, 0.006, 0.007), (0.004, 0.005, 0.006), (0.005, 0.006, 0.007)),
            row(5, (0.040, 0.050, 0.060), (0.030, 0.050, 0.070), (0.040, 0.050, 0.060)),
        ]
        selected = select_distribution_prefixes_from_scan(scan)
        counts = selected["prefix_counts"]
        self.assertLess(counts["easy"], counts["medium"])
        self.assertLess(counts["medium"], counts["hard"])
        self.assertTrue(selected["criteria"]["strict_prefix_nesting"])

    def test_rejects_overlapping_distributions(self) -> None:
        scan = [
            row(0, (0.005, 0.020, 0.060), (0.005, 0.020, 0.060), (0.005, 0.020, 0.060)),
            row(1, (0.020, 0.030, 0.070), (0.020, 0.030, 0.070), (0.020, 0.030, 0.070)),
            row(2, (0.050, 0.070, 0.090), (0.050, 0.070, 0.090), (0.050, 0.070, 0.090)),
        ]
        with self.assertRaises(RuntimeError):
            select_distribution_prefixes_from_scan(scan, medium_ratio=1.1, hard_ratio=1.1, require_strong_planner=False)

    def test_rejects_hard_that_is_faster_for_a_reference_planner(self) -> None:
        scan = [
            row(0, (0.005, 0.006, 0.007), (0.004, 0.005, 0.006), (0.005, 0.006, 0.007)),
            row(5, (0.040, 0.050, 0.060), (0.030, 0.050, 0.070), (0.040, 0.050, 0.060)),
            row(9, (0.130, 0.160, 0.190), (0.010, 0.020, 0.030), (0.130, 0.160, 0.190)),
        ]
        with self.assertRaises(RuntimeError):
            select_distribution_prefixes_from_scan(
                scan,
                hard_not_faster_factor=1.0,
                require_strong_planner=False,
            )


if __name__ == "__main__":
    unittest.main()
