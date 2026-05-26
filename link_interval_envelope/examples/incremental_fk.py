from __future__ import annotations

from pathlib import Path

from link_interval_envelope import IncrementalEnvelopeComputer, write_json
from link_interval_envelope.visualize import save_html


ROOT = Path(__file__).resolve().parents[1]


def main() -> None:
    computer = IncrementalEnvelopeComputer(
        ROOT / "examples" / "data" / "2dof_planar.json",
        endpoint_source="ifk",
        envelope_type="link_iaabb",
        n_subdivisions=4,
    )
    computer.compute([[-0.4, 0.4], [-0.2, 0.2]])
    result = computer.compute([[-0.4, 0.4], [-0.1, 0.3]])
    out_dir = ROOT / "examples" / "out"
    write_json(result, out_dir / "2dof_incremental.json")
    save_html(result, out_dir / "2dof_incremental.html")
    print(result["incremental"])


if __name__ == "__main__":
    main()
