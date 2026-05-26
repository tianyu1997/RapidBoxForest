from __future__ import annotations

from pathlib import Path

from link_interval_envelope import compute_envelope, write_json
from link_interval_envelope.visualize import save_html


ROOT = Path(__file__).resolve().parents[1]


def main() -> None:
    result = compute_envelope(
        ROOT / "examples" / "data" / "2dof_planar.json",
        [[-0.4, 0.4], [-0.2, 0.2]],
        endpoint_source="ifk",
        envelope_type="link_iaabb",
        n_subdivisions=4,
    )
    out_dir = ROOT / "examples" / "out"
    write_json(result, out_dir / "2dof_linkiaabb_s4.json")
    save_html(result, out_dir / "2dof_linkiaabb_s4.html")
    print(out_dir / "2dof_linkiaabb_s4.json")
    print(out_dir / "2dof_linkiaabb_s4.html")


if __name__ == "__main__":
    main()
