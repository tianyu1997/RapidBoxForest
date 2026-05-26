from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Mapping


_COLORS = [
    "#2563eb", "#dc2626", "#16a34a", "#9333ea", "#ea580c",
    "#0891b2", "#be123c", "#4f46e5", "#65a30d", "#0f766e",
]


def _box_edges(aabb: list[list[float]]):
    x0, x1 = aabb[0]
    y0, y1 = aabb[1]
    z0, z1 = aabb[2]
    none = None
    x = [x0, x1, none, x0, x1, none, x0, x1, none, x0, x1, none,
         x0, x0, none, x1, x1, none, x1, x1, none, x0, x0, none,
         x0, x0, none, x1, x1, none, x1, x1, none, x0, x0]
    y = [y0, y0, none, y1, y1, none, y0, y0, none, y1, y1, none,
         y0, y1, none, y0, y1, none, y0, y1, none, y0, y1, none,
         y0, y0, none, y0, y0, none, y1, y1, none, y1, y1]
    z = [z0, z0, none, z0, z0, none, z1, z1, none, z1, z1, none,
         z0, z0, none, z0, z0, none, z1, z1, none, z1, z1, none,
         z0, z1, none, z0, z1, none, z0, z1, none, z0, z1]
    return x, y, z


def build_traces(result: Mapping[str, Any], *, view: str = "inflated") -> list[dict[str, Any]]:
    traces: list[dict[str, Any]] = []
    key = "inflated_aabb" if view == "inflated" else "raw_aabb"
    seen_links: set[int] = set()

    for item in result.get("envelope", {}).get("links", []):
        link_idx = int(item.get("active_link_idx", item.get("link_idx", 0)))
        color = _COLORS[link_idx % len(_COLORS)]
        x, y, z = _box_edges(item[key])
        traces.append({
            "type": "scatter3d",
            "mode": "lines",
            "x": x,
            "y": y,
            "z": z,
            "line": {"color": color, "width": 3},
            "name": f"link {item.get('link_idx', link_idx)}",
            "showlegend": link_idx not in seen_links,
        })
        seen_links.add(link_idx)

    for link in result.get("robot", {}).get("midpoint_links", []):
        p = link.get("proximal", [0, 0, 0])
        d = link.get("distal", [0, 0, 0])
        traces.append({
            "type": "scatter3d",
            "mode": "lines+markers",
            "x": [p[0], d[0]],
            "y": [p[1], d[1]],
            "z": [p[2], d[2]],
            "line": {"color": "#111827", "width": 6},
            "marker": {"size": 3, "color": "#111827"},
            "name": "midpoint robot",
            "showlegend": False,
        })

    centres = result.get("envelope", {}).get("grid", {}).get("centres", [])
    if centres:
        traces.append({
            "type": "scatter3d",
            "mode": "markers",
            "x": [p[0] for p in centres],
            "y": [p[1] for p in centres],
            "z": [p[2] for p in centres],
            "marker": {"size": 2, "opacity": 0.35, "color": "#0f766e"},
            "name": "voxels",
        })
    return traces


def make_html(result: Mapping[str, Any], *, view: str = "inflated") -> str:
    title = f"{result.get('robot', {}).get('name', 'robot')} {result.get('envelope', {}).get('type', '')}"
    traces = build_traces(result, view=view)
    layout = {
        "title": title,
        "scene": {
            "xaxis": {"title": "X (m)"},
            "yaxis": {"title": "Y (m)"},
            "zaxis": {"title": "Z (m)"},
            "aspectmode": "data",
        },
        "margin": {"l": 0, "r": 0, "b": 0, "t": 42},
        "legend": {"x": 0.01, "y": 0.99},
    }
    return f"""<!doctype html>
<html lang=\"en\">
<head>
  <meta charset=\"utf-8\" />
  <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\" />
  <title>{title}</title>
  <script src=\"https://cdn.plot.ly/plotly-2.35.2.min.js\"></script>
  <style>html, body, #plot {{ width: 100%; height: 100%; margin: 0; }}</style>
</head>
<body>
  <div id=\"plot\"></div>
  <script>
    const traces = {json.dumps(traces)};
    const layout = {json.dumps(layout)};
    Plotly.newPlot('plot', traces, layout, {{responsive: true}});
  </script>
</body>
</html>
"""


def save_html(result: Mapping[str, Any], path: str | Path, *, view: str = "inflated") -> None:
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text(make_html(result, view=view), encoding="utf-8")


def load_result(path: str | Path) -> dict[str, Any]:
    return json.loads(Path(path).read_text(encoding="utf-8"))
