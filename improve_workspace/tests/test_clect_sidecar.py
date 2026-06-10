from __future__ import annotations

import unittest

from improve_workspace.clect_sidecar import (
    AdaptiveLeafSweep,
    AdaptiveSweepConfig,
    ComponentBox,
    ConnectivityAction,
    ConnectivityContext,
    DyadicAddress,
    Interval,
    Portal,
    PortalCorridor,
    PortalGraph,
    BoundaryBox,
    MaterialWitness,
    SparseNodeMap,
    ValidationReport,
    occupied_report_from_witness,
    detect_portals,
    classify_connectivity_dominance,
    greedy_portal_search,
    revolute_motion_bound,
    jump_cell_containing,
)
from improve_workspace.clect_sidecar.dyadic import boxes_touch_or_overlap
from improve_workspace.clect_sidecar.reports import Blocker, FailStage, ValidationStatus
from improve_workspace.clect_sidecar.synthetic import Region, SyntheticValidator, unit_root


class DyadicAddressTest(unittest.TestCase):
    def test_jump_cell_contains_point(self) -> None:
        root = unit_root(2)
        cell = jump_cell_containing(root, (0.99, 0.01), (3, 2))
        self.assertTrue(cell.contains_point(root, (0.99, 0.01)))
        self.assertEqual(cell.levels, (3, 2))

    def test_split_and_lca(self) -> None:
        root = DyadicAddress.root(2)
        left, right = root.split(0)
        self.assertTrue(root.is_ancestor_of(left))
        self.assertTrue(root.is_ancestor_of(right))
        self.assertEqual(left.lca(right), root)

    def test_mixed_depth_adjacency_uses_intervals(self) -> None:
        root_box = unit_root(2)
        coarse_left, coarse_right = DyadicAddress.root(2).split(0)
        child, _ = coarse_right.split(1)
        self.assertTrue(
            boxes_touch_or_overlap(coarse_left.interval_box(root_box), child.interval_box(root_box))
        )


class AdaptiveSweepTest(unittest.TestCase):
    def test_early_free_accept_reduces_evaluations(self) -> None:
        root = unit_root(2)
        validator = SyntheticValidator(
            free_regions=[Region((Interval(0.0, 0.5), Interval(0.0, 1.0)), "free_half")]
        )
        result = AdaptiveLeafSweep(
            root,
            validator,
            AdaptiveSweepConfig(start_depth=2, max_depth=8, max_evaluations=10000),
        ).run()
        fixed_count = (1 << 2) * (1 << (8 - 2))
        self.assertLess(result.evaluated, fixed_count)
        self.assertGreater(len(result.free), 0)

    def test_no_good_defers_repeated_blocker(self) -> None:
        root = unit_root(1)

        def always_fail(address, box):
            blocker = Blocker(0, 0, FailStage.GJK, margin=-1.0, overlap_score=1.0, affected_joints=(0,))
            return ValidationReport.fail([blocker], overlap_score=1.0)

        result = AdaptiveLeafSweep(
            root,
            always_fail,
            AdaptiveSweepConfig(start_depth=0, max_depth=10, no_good_chain=2, max_evaluations=100),
        ).run()
        self.assertGreater(len(result.deferred), 0)
        self.assertTrue(any(cell.reason == "no_good" for cell in result.deferred))


class ConnectivityDominanceTest(unittest.TestCase):
    def test_covered_cell_is_pruned_without_validation_priority(self) -> None:
        cell = (Interval(0.25, 0.5), Interval(0.25, 0.5))
        accepted = [ComponentBox("big", "c0", (Interval(0.0, 1.0), Interval(0.0, 1.0)))]
        decision = classify_connectivity_dominance(cell, accepted)
        self.assertEqual(decision.action, ConnectivityAction.COVERED)
        self.assertEqual(decision.covered_by, "big")

    def test_cell_touching_two_components_is_prioritized(self) -> None:
        cell = (Interval(0.4, 0.6),)
        accepted = [
            ComponentBox("left", "c0", (Interval(0.0, 0.4),)),
            ComponentBox("right", "c1", (Interval(0.6, 1.0),)),
        ]
        decision = classify_connectivity_dominance(cell, accepted)
        self.assertEqual(decision.action, ConnectivityAction.PRIORITIZE_CONNECTOR)
        self.assertEqual(decision.adjacent_components, frozenset({"c0", "c1"}))

    def test_single_component_interior_cell_defers_unless_protected(self) -> None:
        cell = (Interval(0.4, 0.6),)
        accepted = [ComponentBox("left", "c0", (Interval(0.0, 0.4),))]
        decision = classify_connectivity_dominance(cell, accepted)
        self.assertEqual(decision.action, ConnectivityAction.DEFER_CONNECTIVITY)

        protected = classify_connectivity_dominance(
            cell,
            accepted,
            ConnectivityContext(on_portal_boundary=True),
        )
        self.assertEqual(protected.action, ConnectivityAction.RELEVANT)

    def test_isolated_cell_is_low_priority_unless_anchor_protected(self) -> None:
        cell = (Interval(0.4, 0.6),)
        accepted = [ComponentBox("far", "c0", (Interval(0.0, 0.2),))]
        decision = classify_connectivity_dominance(cell, accepted)
        self.assertEqual(decision.action, ConnectivityAction.LOW_PRIORITY_DEFER)

        protected = classify_connectivity_dominance(
            cell,
            accepted,
            ConnectivityContext(offline_anchor_cell=True),
        )
        self.assertEqual(protected.action, ConnectivityAction.RELEVANT)


class PortalGraphTest(unittest.TestCase):
    def test_conservative_portal_edge_expands_chain(self) -> None:
        root = unit_root(1)
        left, right = DyadicAddress.root(1).split(0)
        mid_left, mid_right = right.split(0)
        pin = Portal("domain", "left_box", "c0")
        pout = Portal("domain", "right_box", "c1")
        corridor = PortalCorridor(
            "corridor",
            "domain",
            pin,
            pout,
            [mid_left],
            [mid_left.interval_box(root)],
            [ValidationReport.free("cert")],
        )
        graph = PortalGraph()
        graph.add_portal_corridor(corridor, left.interval_box(root), mid_right.interval_box(root))
        expanded = graph.expand_edge("corridor")
        self.assertEqual(expanded[0], "left_box")
        self.assertEqual(expanded[-1], "right_box")
        self.assertIn("corridor:cell:0", expanded)

    def test_invalid_portal_certificate_rejected(self) -> None:
        root = unit_root(1)
        left, right = DyadicAddress.root(1).split(0)
        pin = Portal("domain", "left_box", "c0")
        pout = Portal("domain", "right_box", "c1")
        corridor = PortalCorridor(
            "bad",
            "domain",
            pin,
            pout,
            [right],
            [right.interval_box(root)],
            [ValidationReport(status=ValidationStatus.FAIL)],
        )
        with self.assertRaises(ValueError):
            PortalGraph().add_portal_corridor(corridor, left.interval_box(root), right.interval_box(root))

    def test_greedy_portal_search_builds_internal_chain(self) -> None:
        root = unit_root(1)
        left, right = DyadicAddress.root(1).split(0)
        c0, c1 = right.split(0)
        pin = Portal("domain", "left_box", "c0")
        pout = Portal("domain", "right_box", "c1")
        reports = {c0: ValidationReport.free("c0")}
        corridor = greedy_portal_search(
            "pc",
            "domain",
            right.interval_box(root),
            pin,
            pout,
            left.interval_box(root),
            c1.interval_box(root),
            [(c0, c0.interval_box(root))],
            reports,
        )
        self.assertIsNotNone(corridor)
        assert corridor is not None
        self.assertTrue(corridor.validate_certificate(left.interval_box(root), c1.interval_box(root)))

    def test_detect_portals_filters_touching_boundary_boxes(self) -> None:
        root = unit_root(1)
        left, right = DyadicAddress.root(1).split(0)
        portals = detect_portals(
            "domain",
            right.interval_box(root),
            [
                BoundaryBox("left", "c0", left.interval_box(root)),
                BoundaryBox("right", "c1", right.interval_box(root)),
            ],
        )
        self.assertEqual({p.boundary_box_id for p in portals}, {"left", "right"})


class OccupiedCertificateTest(unittest.TestCase):
    def test_signed_distance_witness_certifies_only_when_motion_bound_fits(self) -> None:
        witness = MaterialWitness(0, 1, center_signed_distance=-0.10, motion_bound=0.02, epsilon_num=1e-6)
        report = occupied_report_from_witness(witness)
        self.assertIsNotNone(report)
        self.assertEqual(report.status, ValidationStatus.CERT_OCCUPIED)

        weak = MaterialWitness(0, 1, center_signed_distance=-0.01, motion_bound=0.02, epsilon_num=1e-6)
        self.assertIsNone(occupied_report_from_witness(weak))

    def test_revolute_motion_bound_is_no_larger_than_linearized_for_small_angles(self) -> None:
        exact = revolute_motion_bound([2.0, 1.0], [0.05, 0.10])
        linear = 2.0 * 0.05 + 1.0 * 0.10
        self.assertLessEqual(exact, linear)


class SparseTreeTest(unittest.TestCase):
    def test_jump_materialize_does_not_create_ancestors(self) -> None:
        root = unit_root(2)
        tree = SparseNodeMap(2)
        node = tree.jump_materialize(root, (0.25, 0.75), (20, 20), evidence_ref="e", terminal=True)
        self.assertEqual(len(tree), 1)
        self.assertEqual(node.address.levels, (20, 20))
        self.assertEqual(len(tree.terminal_nodes()), 1)
        self.assertEqual(tree.ancestors_present(node.address), [])


if __name__ == "__main__":
    unittest.main()
