"""Prepare reviewable Start/B drafts from existing geometry; no field certification."""

import hashlib
import json
from pathlib import Path

import numpy as np
import yaml


def main():
    root = Path(__file__).resolve().parents[1]
    reconstruction = root / "data/map-reviews/0914_fr_v3-20260914-142008/reconstruction"
    source = reconstruction / "start_B_short_fine_v1/official_policy_v1/bundle/course_full.yaml"
    edges_file = reconstruction / "B_structured_repair_v1/delivery/repair_report.json"
    original = yaml.safe_load(source.read_text())["waypoints"][:13]
    b = np.array([p["position"] for p in original[4:]])
    station = np.r_[0.0, np.linalg.norm(np.diff(b[:, :2], axis=0), axis=1).cumsum()]
    edges = json.loads(edges_file.read_text())["optimized_edges"]
    # Provisional centreline margins for entire-body clearance and pre-entry stopping.
    # They must be checked against real chassis geometry, turn angle and localization.
    margin = 0.70
    landings = [(edges[a]["station"] + margin, edges[a + 1]["station"] - margin) for a in (11, 18)]
    upper_clear = edges[31]["station"] + margin
    transitions = [x for pair in landings for x in pair] + [upper_clear]
    assert all(a < b for a, b in landings)
    stations = sorted([*station, *transitions])
    points = [dict(p, kind="flat", b_station=None) for p in original[:4]]
    for i, s in enumerate(stations):
        xyz = [float(np.interp(s, station, b[:, dim])) for dim in range(3)]
        middle = 0 if i == 0 else (stations[i - 1] + s) / 2
        flat = i == 0 or middle >= upper_clear or any(a <= middle <= z for a, z in landings)
        points.append(
            {
                "position": xyz,
                "kind": "flat" if flat else "stairs",
                "b_station": float(s),
                "staging_candidate": s in transitions,
            }
        )
    metadata = {
        "map_id": "0914_fr_v3-20260914-142008",
        "z_reference": "ground",
        "status": "DRAFT; field XYZ reference, corridor and staging clearance unverified",
        "scope": "Start through Area B upper landing; no post-B extension",
        "kind_semantics": "gait for incoming segment; switch at previous target",
        "body_clearance_margin_m": margin,
        "landing_flat_station_intervals": landings,
        "upper_flat_starts_at_station": upper_clear,
        "ground_z_method": "interpolate simulation ground heights; field check required",
        "sources": {
            str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in (source, edges_file)
        },
    }
    target = Path(__file__).with_name("config")
    for name, subset in [("start", points[:5]), ("start_b", points), ("b", points[4:])]:
        data = {"metadata": metadata, "waypoints": [dict(p, index=i) for i, p in enumerate(subset)]}
        (target / f"{name}.draft.json").write_text(json.dumps(data, indent=2) + "\n")
        print(name, len(subset), "targets", [p["kind"] for p in subset])


if __name__ == "__main__":
    main()
