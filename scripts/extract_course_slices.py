#!/usr/bin/env python3
"""Extract exact MuJoCo collision meshes around selected waypoint segments."""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

WAYPOINT_RE = re.compile(r"position:\s*\[\s*([^,]+),\s*([^,]+),\s*([^\]]+)\]")


def load_waypoints(path: Path) -> list[np.ndarray]:
    points = []
    for line in path.read_text(encoding="utf-8").splitlines():
        match = WAYPOINT_RE.search(line)
        if match:
            points.append(np.array([float(value) for value in match.groups()], dtype=float))
    if not points:
        raise RuntimeError(f"No waypoints found in {path}")
    return points


def segment_route(points: list[np.ndarray], start_wp: int, end_wp: int, extension: float) -> np.ndarray:
    start_idx = start_wp - 1
    end_idx = end_wp - 1
    if end_idx != start_idx + 1 or start_idx < 1 or end_idx + 1 >= len(points):
        raise ValueError("Expected adjacent waypoints with a previous and next waypoint")

    start, end = points[start_idx], points[end_idx]
    approach_dir = start[:2] - points[start_idx - 1][:2]
    exit_dir = points[end_idx + 1][:2] - end[:2]
    approach_dir /= np.linalg.norm(approach_dir)
    exit_dir /= np.linalg.norm(exit_dir)

    approach = start.copy()
    approach[:2] -= extension * approach_dir
    exit_point = end.copy()
    exit_point[:2] += extension * exit_dir
    return np.stack((approach, start, end, exit_point))


def aabb_hits_corridor(center: np.ndarray, half: np.ndarray, route: np.ndarray, half_width: float) -> bool:
    for first, second in zip(route[:-1], route[1:]):
        delta = second[:2] - first[:2]
        length = float(np.linalg.norm(delta))
        if length == 0.0:
            continue
        forward = delta / length
        lateral = np.array([-forward[1], forward[0]])
        offset = center[:2] - first[:2]
        forward_radius = float(np.dot(np.abs(forward), half[:2]))
        lateral_radius = float(np.dot(np.abs(lateral), half[:2]))
        along = float(np.dot(offset, forward))
        across = abs(float(np.dot(offset, lateral)))
        if along + forward_radius >= 0.0 and along - forward_radius <= length and across <= half_width + lateral_radius:
            return True
    return False


def scene_assets(scene_path: Path) -> tuple[dict[str, ET.Element], ET.Element]:
    root = ET.parse(scene_path).getroot()
    meshes = {element.attrib["name"]: element for element in root.findall("./asset/mesh")}
    body = root.find("./worldbody/body")
    if body is None:
        raise RuntimeError(f"No terrain body found in {scene_path}")
    return meshes, body


def selected_meshes(track_xml: Path, route: np.ndarray, half_width: float) -> list[str]:
    model = mujoco.MjModel.from_xml_path(str(track_xml))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    selected: set[str] = set()
    for geom_id in range(model.ngeom):
        if model.geom_type[geom_id] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        mesh_id = int(model.geom_dataid[geom_id])
        mesh_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MESH, mesh_id)
        if not mesh_name or not mesh_name.startswith("mesh_"):
            continue
        local_center = model.geom_aabb[geom_id, :3]
        local_half = model.geom_aabb[geom_id, 3:]
        rotation = data.geom_xmat[geom_id].reshape(3, 3)
        world_center = data.geom_xpos[geom_id] + rotation @ local_center
        world_half = np.abs(rotation) @ local_half
        if aabb_hits_corridor(world_center, world_half, route, half_width):
            selected.add(mesh_name)
    return sorted(selected, key=lambda name: int(name.split("_")[1]))


def add_route_markers(worldbody: ET.Element, route: np.ndarray) -> None:
    colors = ("0.1 0.45 1 1", "0.15 0.9 0.3 1", "1 0.35 0.1 1")
    for index, point in enumerate(route):
        color = colors[0] if index == 0 else colors[2] if index == len(route) - 1 else colors[1]
        ET.SubElement(
            worldbody,
            "geom",
            type="sphere",
            size="0.11",
            pos=" ".join(f"{value:.5f}" for value in (point + np.array([0.0, 0.0, 0.12]))),
            rgba=color,
            contype="0",
            conaffinity="0",
            group="2",
        )
    for first, second in zip(route[:-1], route[1:]):
        start = first + np.array([0.0, 0.0, 0.08])
        end = second + np.array([0.0, 0.0, 0.08])
        ET.SubElement(
            worldbody,
            "geom",
            type="capsule",
            size="0.025",
            fromto=" ".join(f"{value:.5f}" for value in np.concatenate((start, end))),
            rgba="0.15 0.9 0.3 0.8",
            contype="0",
            conaffinity="0",
            group="2",
        )


def write_slice(
    scene_path: Path,
    output_dir: Path,
    route: np.ndarray,
    names: list[str],
    start_wp: int,
    end_wp: int,
    extension: float,
    half_width: float,
) -> Path:
    assets, source_body = scene_assets(scene_path)
    output_dir.mkdir(parents=True, exist_ok=True)
    mesh_dir = output_dir / "meshes"
    mesh_dir.mkdir(exist_ok=True)

    root = ET.Element("mujoco", model=f"wp{start_wp}_wp{end_wp}_collision_slice")
    ET.SubElement(root, "compiler", angle="radian", meshdir="meshes")
    ET.SubElement(root, "option", gravity="0 0 -9.81")
    visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "global", offwidth="1200", offheight="500")
    asset_out = ET.SubElement(root, "asset")
    worldbody = ET.SubElement(root, "worldbody")
    ET.SubElement(worldbody, "light", pos="0 0 20", dir="0 0 -1", diffuse="0.9 0.9 0.9")
    body_out = ET.SubElement(worldbody, "body", name="collision_slice", pos=source_body.attrib.get("pos", "0 0 0"))

    source_geoms = {geom.attrib.get("mesh"): geom for geom in source_body.findall("geom") if geom.attrib.get("mesh")}
    copied = []
    for name in names:
        source_asset = assets[name]
        source_file = (scene_path.parent / source_asset.attrib["file"]).resolve()
        destination = mesh_dir / source_file.name
        shutil.copy2(source_file, destination)
        mesh_attributes = {"name": name, "file": destination.name}
        if "scale" in source_asset.attrib:
            mesh_attributes["scale"] = source_asset.attrib["scale"]
        ET.SubElement(asset_out, "mesh", **mesh_attributes)

        source_geom = source_geoms[name]
        geom_attributes = dict(source_geom.attrib)
        geom_attributes.setdefault("rgba", "0.62 0.65 0.70 1")
        ET.SubElement(body_out, "geom", **geom_attributes)
        copied.append(destination.name)

    add_route_markers(worldbody, route)
    ET.indent(root, space="  ")
    slice_path = output_dir / "slice.xml"
    ET.ElementTree(root).write(slice_path, encoding="utf-8", xml_declaration=True)
    mujoco.MjModel.from_xml_path(str(slice_path))

    manifest = {
        "source_scene": str(scene_path),
        "start_waypoint": start_wp,
        "end_waypoint": end_wp,
        "approach_m": extension,
        "exit_m": extension,
        "corridor_half_width_m": half_width,
        "route_points_xyz": route.tolist(),
        "mesh_count": len(names),
        "meshes": copied,
    }
    (output_dir / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    return slice_path


def render_slice(slice_path: Path, route: np.ndarray) -> Path:
    model = mujoco.MjModel.from_xml_path(str(slice_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    renderer = mujoco.Renderer(model, height=500, width=600)
    center = np.mean(route, axis=0)
    span = float(np.linalg.norm(route[-1, :2] - route[0, :2]))
    direction = route[-1, :2] - route[0, :2]
    heading = math.degrees(math.atan2(direction[1], direction[0]))

    panels = []
    for azimuth, elevation, distance in ((heading - 90.0, -88.0, max(7.0, span * 0.75)), (heading - 105.0, -32.0, max(7.0, span * 0.85))):
        camera = mujoco.MjvCamera()
        camera.type = mujoco.mjtCamera.mjCAMERA_FREE
        camera.lookat[:] = center
        camera.lookat[2] += 0.25
        camera.azimuth = azimuth
        camera.elevation = elevation
        camera.distance = distance
        renderer.update_scene(data, camera=camera)
        panels.append(renderer.render().copy())
    renderer.close()
    image = np.concatenate(panels, axis=1)
    output = slice_path.with_name("preview.png")
    subprocess.run(
        ["ffmpeg", "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{image.shape[1]}x{image.shape[0]}", "-i", "-", "-frames:v", "1", str(output)],
        input=image.tobytes(),
        check=True,
    )
    return output


def point_along_route(route: np.ndarray, distance: float) -> np.ndarray:
    remaining = distance
    for first, second in zip(route[:-1], route[1:]):
        length = float(np.linalg.norm(second[:2] - first[:2]))
        if remaining <= length:
            return first + (remaining / length) * (second - first)
        remaining -= length
    return route[-1].copy()


def stair_crop_route(profile_path: Path, route: np.ndarray, margin: float) -> np.ndarray:
    with profile_path.open(encoding="utf-8") as handle:
        samples = [(float(row["distance_m"]), float(row["collision_height_m"])) for row in csv.DictReader(handle)]
    rises = [distance for (previous_distance, previous), (distance, height) in zip(samples[:-1], samples[1:]) if height - previous > 0.035]
    if not rises:
        raise RuntimeError(f"No stair risers detected in {profile_path}")
    total = sum(float(np.linalg.norm(second[:2] - first[:2])) for first, second in zip(route[:-1], route[1:]))
    first_riser, final_riser = rises[0], rises[-1]
    distances = (max(0.0, first_riser - margin), first_riser, final_riser, min(total, final_riser + margin))
    return np.stack([point_along_route(route, distance) for distance in distances])


def write_profile(
    slice_path: Path,
    route: np.ndarray,
    spacing: float = 0.025,
    labels: tuple[str, str, str, str] | None = None,
) -> Path:
    model = mujoco.MjModel.from_xml_path(str(slice_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    distances: list[float] = []
    heights: list[float] = []
    cumulative = 0.0
    geomgroup = np.array([1, 1, 0, 1, 1, 1], dtype=np.uint8)
    for segment_index, (first, second) in enumerate(zip(route[:-1], route[1:])):
        length = float(np.linalg.norm(second[:2] - first[:2]))
        sample_count = max(2, math.ceil(length / spacing) + 1)
        for index, fraction in enumerate(np.linspace(0.0, 1.0, sample_count)):
            if segment_index and index == 0:
                continue
            xy = first[:2] + fraction * (second[:2] - first[:2])
            origin = np.array([xy[0], xy[1], 10.0])
            geom_id = np.array([-1], dtype=np.int32)
            ray_distance = mujoco.mj_ray(
                model, data, origin, np.array([0.0, 0.0, -1.0]), geomgroup, True, -1, geom_id
            )
            distances.append(cumulative + fraction * length)
            heights.append(float("nan") if ray_distance < 0 else 10.0 - ray_distance)
        cumulative += length

    csv_path = slice_path.with_name("profile.csv")
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(("distance_m", "collision_height_m"))
        writer.writerows(zip(distances, heights))

    finite = [(distance, height) for distance, height in zip(distances, heights) if math.isfinite(height)]
    x_values = [value[0] for value in finite]
    y_values = [value[1] for value in finite]
    x_min, x_max = min(x_values), max(x_values)
    y_min, y_max = min(y_values), max(y_values)
    y_pad = max(0.08, (y_max - y_min) * 0.12)
    y_min -= y_pad
    y_max += y_pad
    width, height = 1100, 360
    left, right, top, bottom = 72, 24, 28, 52

    def sx(value: float) -> float:
        return left + (value - x_min) / (x_max - x_min) * (width - left - right)

    def sy(value: float) -> float:
        return top + (y_max - value) / (y_max - y_min) * (height - top - bottom)

    path_points = " ".join(f"{sx(x):.1f},{sy(y):.1f}" for x, y in finite)
    boundaries = np.cumsum([0.0] + [float(np.linalg.norm(b[:2] - a[:2])) for a, b in zip(route[:-1], route[1:])])
    verticals = "".join(
        f'<line x1="{sx(value):.1f}" y1="{top}" x2="{sx(value):.1f}" y2="{height-bottom}" class="boundary"/>'
        for value in boundaries[1:-1]
    )
    if labels is None:
        labels = ("接近区起点", f"WP{slice_path.parent.name[2:4]}", f"WP{slice_path.parent.name[7:9]}", "退出区终点")
    label_elements = "".join(
        f'<text x="{sx(value):.1f}" y="{height-20}" text-anchor="middle">{label}</text>'
        for value, label in zip(boundaries, labels)
    )
    svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
<style>text{{font:14px system-ui;fill:#263238}} .axis{{stroke:#607d8b}} .boundary{{stroke:#90a4ae;stroke-dasharray:6 5}} .profile{{fill:none;stroke:#1565c0;stroke-width:3;stroke-linejoin:round}}</style>
<rect width="100%" height="100%" fill="#f7f9fb"/><line x1="{left}" y1="{height-bottom}" x2="{width-right}" y2="{height-bottom}" class="axis"/>
<line x1="{left}" y1="{top}" x2="{left}" y2="{height-bottom}" class="axis"/>{verticals}
<polyline points="{path_points}" class="profile"/>{label_elements}
<text x="18" y="{(top + height-bottom)/2:.1f}" transform="rotate(-90 18 {(top + height-bottom)/2:.1f})" text-anchor="middle">碰撞面高度 (m)</text>
<text x="{left}" y="18">{y_max-y_pad:.3f} m</text><text x="{left}" y="{height-bottom-8}">{y_min+y_pad:.3f} m</text>
</svg>'''
    svg_path = slice_path.with_name("profile.svg")
    svg_path.write_text(svg, encoding="utf-8")
    return svg_path


def self_test() -> None:
    route = np.array([[0.0, 0.0, 0.0], [2.0, 0.0, 0.0]])
    assert aabb_hits_corridor(np.array([1.0, 0.9, 0.0]), np.array([0.1, 0.1, 0.1]), route, 1.0)
    assert not aabb_hits_corridor(np.array([1.0, 1.2, 0.0]), np.array([0.1, 0.1, 0.1]), route, 1.0)


def main() -> None:
    parser = argparse.ArgumentParser()
    upstream_mjcf = Path("upstream/goai_embodied_future_material/src/S10_sdk_deploy/S10_description/s10_mjcf/mjcf")
    parser.add_argument("--track-xml", type=Path, default=upstream_mjcf / "S10_track.xml")
    parser.add_argument("--scene-xml", type=Path, default=upstream_mjcf / "scene.xml")
    parser.add_argument("--course", type=Path, default=Path("src/s10_bringup/config/course.yaml"))
    parser.add_argument("--output", type=Path, default=Path("artifacts/course_slices"))
    parser.add_argument("--half-width", type=float, default=1.5)
    parser.add_argument("--extension", type=float, default=1.0)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return

    points = load_waypoints(args.course)
    for start_wp, end_wp in ((7, 8), (18, 19)):
        route = segment_route(points, start_wp, end_wp, args.extension)
        names = selected_meshes(args.track_xml, route, args.half_width)
        if not names:
            raise RuntimeError(f"No collision meshes found for WP{start_wp} -> WP{end_wp}")
        output_dir = args.output / f"wp{start_wp:02d}_wp{end_wp:02d}"
        slice_path = write_slice(args.scene_xml, output_dir, route, names, start_wp, end_wp, args.extension, args.half_width)
        preview = render_slice(slice_path, route)
        profile = write_profile(slice_path, route)
        crop_route = stair_crop_route(profile.with_suffix(".csv"), route, args.extension)
        crop_names = selected_meshes(args.track_xml, crop_route, args.half_width)
        crop_dir = output_dir / "stair_crop"
        crop_slice = write_slice(
            args.scene_xml,
            crop_dir,
            crop_route,
            crop_names,
            start_wp,
            end_wp,
            args.extension,
            args.half_width,
        )
        crop_preview = render_slice(crop_slice, crop_route)
        crop_profile = write_profile(
            crop_slice,
            crop_route,
            labels=("接近区起点", "第一阶", "最后一阶", "退出区终点"),
        )
        print(
            f"WP{start_wp}->{end_wp}: overview={len(names)} meshes, stair_crop={len(crop_names)} meshes, "
            f"{slice_path}, {preview}, {profile}, {crop_slice}, {crop_preview}, {crop_profile}"
        )


if __name__ == "__main__":
    main()
