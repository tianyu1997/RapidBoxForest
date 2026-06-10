from __future__ import annotations

import unittest

from improve_workspace.clect_sidecar import (
    Blocker,
    DyadicAddress,
    FailStage,
    HiPaCCell,
    HiPaCCellState,
    HiPaCConfig,
    HiPaCOfflineMode,
    HierarchicalPartitionConnectivity,
    Interval,
    ValidationReport,
)
from improve_workspace.clect_sidecar.synthetic import unit_root


def corridor_validator(address: DyadicAddress, box):
    """Coarse middle cell is mixed; deeper children reveal a free corridor."""

    lo = box[0].lo
    hi = box[0].hi
    if hi <= 0.25 + 1e-12:
        return ValidationReport.free("left")
    if lo >= 0.5 - 1e-12:
        return ValidationReport.free("right")
    if lo >= 0.25 - 1e-12 and hi <= 0.5 + 1e-12 and address.depth >= 3:
        return ValidationReport.free("hidden_mid")
    blocker = Blocker(0, 0, FailStage.GJK, margin=-0.1, overlap_score=0.5, affected_joints=(0,))
    return ValidationReport.fail([blocker], overlap_score=0.5)


class HiPaCSidecarTest(unittest.TestCase):
    def test_build_resolves_mixed_cell_as_certified_portal_edge(self) -> None:
        planner = HierarchicalPartitionConnectivity(
            unit_root(1),
            corridor_validator,
            HiPaCConfig(coarse_depth=2, max_depth=4, max_refinement_iterations=4, local_refine_budget=8),
        )
        result = planner.build()
        self.assertGreaterEqual(result.resolved_portal_pairs, 1)
        self.assertEqual(result.certified_components, 1)
        self.assertTrue(planner.portal_corridors)
        self.assertTrue(any(edge.kind == "portal" for edges in planner.cert_adj.values() for edge in edges))
        hidden_ids = {
            f"{corridor.corridor_id}:cell:{index}"
            for corridor in planner.portal_corridors.values()
            for index, _ in enumerate(corridor.internal_cells)
        }
        self.assertFalse(hidden_ids & set(planner.cert_adj))

    def test_mixed_cells_only_enter_optimistic_graph(self) -> None:
        planner = HierarchicalPartitionConnectivity(
            unit_root(1),
            corridor_validator,
            HiPaCConfig(coarse_depth=2, max_depth=4, max_refinement_iterations=0),
        )
        result = planner.build()
        self.assertEqual(result.resolved_portal_pairs, 0)
        mixed_ids = {cell.cell_id for cell in planner.cells.values() if cell.state == HiPaCCellState.MIXED}
        self.assertTrue(mixed_ids)
        self.assertTrue(mixed_ids & set(planner.opt_adj))
        self.assertFalse(mixed_ids & set(planner.cert_adj))

    def test_query_refines_first_unresolved_mixed_cell_and_expands_portal(self) -> None:
        planner = HierarchicalPartitionConnectivity(
            unit_root(1),
            corridor_validator,
            HiPaCConfig(coarse_depth=2, max_depth=4, max_refinement_iterations=0, local_refine_budget=8),
        )
        planner.build()
        result = planner.query((0.1,), (0.8,), online_budget=4)
        self.assertTrue(result.success, result.status)
        self.assertGreaterEqual(result.refined_mixed_cells, 1)
        self.assertGreaterEqual(result.used_portal_edges, 1)
        self.assertTrue(any(":cell:" in item for item in result.expanded_path))

    def test_workload_anchor_pair_metrics_use_connected_components(self) -> None:
        planner = HierarchicalPartitionConnectivity(
            unit_root(1),
            corridor_validator,
            HiPaCConfig(
                coarse_depth=2,
                max_depth=4,
                max_refinement_iterations=4,
                local_refine_budget=8,
                offline_mode=HiPaCOfflineMode.WORKLOAD_AWARE_COMPONENT_CONNECTIVITY,
            ),
            workload_anchors=((0.1,), (0.8,)),
            workload_anchor_pairs=((0, 1),),
        )
        planner.build()
        metrics = planner.metrics(
            probe_points=((0.1,), (0.3,), (0.8,)),
            query_pairs=(((0.1,), (0.8,)),),
        )
        self.assertEqual(metrics.connected_anchor_pairs, 1)
        self.assertEqual(metrics.anchor_pairs, 1)
        self.assertEqual(metrics.p_attach, 1.0)
        self.assertEqual(metrics.p_samecomp, 1.0)

    def test_blocker_affected_joint_drives_anisotropic_split(self) -> None:
        root = unit_root(2)
        report = ValidationReport.fail(
            [Blocker(0, 0, FailStage.GJK, margin=-1.0, overlap_score=3.0, affected_joints=(1,))],
            overlap_score=3.0,
        )
        cell = HiPaCCell(
            "root",
            DyadicAddress.root(2),
            root,
            HiPaCCellState.MIXED,
            report,
        )
        planner = HierarchicalPartitionConnectivity(root, lambda address, box: report)
        self.assertEqual(planner._choose_split_dimension(cell), 1)

    def test_cert_occupied_cell_does_not_enter_optimistic_graph(self) -> None:
        def validator(address, box):
            if box[0].hi <= 0.5:
                return ValidationReport.free("left")
            return ValidationReport.cert_occupied("occupied")

        planner = HierarchicalPartitionConnectivity(
            (Interval(0.0, 1.0),),
            validator,
            HiPaCConfig(coarse_depth=1, max_refinement_iterations=0),
        )
        result = planner.build()
        self.assertEqual(result.cell_state_counts[HiPaCCellState.CERT_OCCUPIED.value], 1)
        occupied = [cell.cell_id for cell in planner.cells.values() if cell.state == HiPaCCellState.CERT_OCCUPIED]
        self.assertFalse(set(occupied) & set(planner.opt_adj))


if __name__ == "__main__":
    unittest.main()
