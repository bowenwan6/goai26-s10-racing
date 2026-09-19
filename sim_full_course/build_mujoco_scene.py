"""Full-course MuJoCo scene: hfield from course_terrain.npz + the packaged S10 robot.

Coarse terrain only (0.10 m hfield from the v3 cloud). The fine Start+B mesh scene in
deliverables/S10_v3_Map_MuJoCo_20260916 remains the reference for stair contact.

Outputs (ARTIFACTS/mujoco/, not committed):
  course_ground.bin               MuJoCo binary hfield (int32 nrow, int32 ncol, float32 data)
  course_ground_obst.bin          same with the obstacle layer raised (conservative walls)
  scene_full_course.xml           ground only
  scene_full_course_obstacles.xml ground + obstacle layer
"""

from __future__ import annotations

import argparse
import copy
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from sim_full_course.io_utils import ARTIFACTS, REPO

PACKAGE_SCENE = (
    REPO / "deliverables" / "S10_v3_Map_MuJoCo_20260916" / "mujoco" / "robot_scene" / "scene.xml"
)
BODY_Z = 0.4327  # package start_pose: base z 0.0028 over ground -0.4293
STAND_JOINTS = [0, -0.3, 0.6, 0] * 2 + [0, 0.3, -0.6, 0] * 2


def write_hfield_bin(path: Path, heights: np.ndarray) -> tuple[float, float]:
    """MuJoCo binary hfield; row index increases with +Y. Returns (zmin, zrange)."""
    h = np.asarray(heights, np.float64)
    zmin, zmax = float(h.min()), float(h.max())
    zr = max(zmax - zmin, 1e-3)
    norm = ((h - zmin) / zr).astype("<f4")
    with open(path, "wb") as f:
        np.array(h.shape, dtype="<i4").tofile(f)
        norm.tofile(f)
    return zmin, zr


def robot_parts(scene_path: Path = PACKAGE_SCENE):
    root = ET.parse(scene_path).getroot()
    robot_dir = scene_path.parent
    default = root.find("default")
    meshes = []
    for m in root.find("asset").findall("mesh"):
        f = m.get("file", "")
        if f.startswith("robot/"):
            m = copy.deepcopy(m)
            m.set("file", str((robot_dir / f).resolve()))
            meshes.append(m)
    mats = [copy.deepcopy(m) for m in root.find("asset").findall("material")
            if m.get("name") in ("default_material", "collision_material")]
    body = copy.deepcopy(root.find("worldbody").find("body[@name='base_link']"))
    sections = [copy.deepcopy(root.find(k)) for k in ("actuator", "contact", "sensor")]
    return default, meshes, mats, body, sections, root.find("option")


def build_scene(npz: Path, out_dir: Path, obstacles: bool, start_xy=(0.0, 0.5), start_yaw=0.0,
                course_pose=None) -> Path:
    t = np.load(npz)
    res = float(t["resolution"])
    x0, y0 = (float(v) for v in t["origin"])
    h = t["ground_filled"].astype(np.float64)
    if obstacles:
        h = h + np.where(t["obstacle"], t["obstacle_height"], 0.0)
    nrow, ncol = h.shape
    name = "course_ground_obst.bin" if obstacles else "course_ground.bin"
    zmin, zr = write_hfield_bin(out_dir / name, h)
    rx = (ncol - 1) * res / 2
    ry = (nrow - 1) * res / 2
    cx = x0 + res / 2 + rx
    cy = y0 + res / 2 + ry

    default, meshes, mats, body, sections, option = robot_parts()
    m = ET.Element("mujoco", model="S10 full course coarse hfield | simulation only")
    ET.SubElement(m, "compiler", angle="radian")
    m.append(copy.deepcopy(option))
    ET.SubElement(m, "size", memory="512M")
    m.append(copy.deepcopy(default))
    asset = ET.SubElement(m, "asset")
    ET.SubElement(asset, "hfield", name="course", file=str((out_dir / name).resolve()),
                  size=f"{rx:.6f} {ry:.6f} {zr:.6f} 1.0")
    ET.SubElement(asset, "texture", name="grid", type="2d", builtin="checker", rgb1=".45 .5 .45",
                  rgb2=".4 .45 .4", width="512", height="512")
    ET.SubElement(asset, "material", name="ground", texture="grid", texrepeat="200 130",
                  reflectance="0")
    for e in meshes + mats:
        asset.append(e)
    wb = ET.SubElement(m, "worldbody")
    ET.SubElement(wb, "light", pos=f"{cx} {cy} 60", dir="0 0 -1", directional="true")
    ET.SubElement(wb, "geom", name="course_hfield", type="hfield", hfield="course",
                  pos=f"{cx:.6f} {cy:.6f} {zmin:.6f}", material="ground", condim="3",
                  friction="1 0.01 0.001", contype="1", conaffinity="1", group="0")
    wb.append(body)
    vis = ET.SubElement(m, "visual")
    ET.SubElement(vis, "global", offwidth="1600", offheight="1000")
    ET.SubElement(vis, "map", zfar="400")
    ET.SubElement(m, "statistic", center=f"{cx} {cy} 3", extent=f"{max(rx, ry) * 2:.1f}")
    for s in sections:
        if s is not None:
            m.append(s)

    def ground_at(x, y):
        ix = int(np.clip(round((x - x0) / res - 0.5), 0, ncol - 1))
        iy = int(np.clip(round((y - y0) / res - 0.5), 0, nrow - 1))
        return float(h[iy, ix])

    kf = ET.SubElement(m, "keyframe")
    poses = [("start_pose", start_xy[0], start_xy[1], start_yaw)]
    if course_pose is not None:
        poses.append(("course_first_keyframe", *course_pose))
    for kname, x, y, yaw in poses:
        z = ground_at(x, y) + BODY_Z
        q = [np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)]
        qpos = [x, y, z, *q, *STAND_JOINTS]
        ET.SubElement(kf, "key", name=kname, qpos=" ".join(f"{v:.6g}" for v in qpos))
    ET.indent(m)
    xml = out_dir / ("scene_full_course_obstacles.xml" if obstacles else "scene_full_course.xml")
    xml.write_text(ET.tostring(m, encoding="unicode"))
    return xml


def verify(xml: Path, npz: Path, n_rays=400, seed=0) -> dict:
    import mujoco

    model = mujoco.MjModel.from_xml_path(str(xml))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    t = np.load(npz)
    res = float(t["resolution"])
    x0, y0 = t["origin"]
    known = t["known"]
    h = t["ground_filled"]
    if "obst" in xml.name:
        h = h + np.where(t["obstacle"], t["obstacle_height"], 0)
    rng = np.random.default_rng(seed)
    iy, ix = np.nonzero(known)
    pick = rng.choice(len(iy), n_rays, replace=False)
    geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "course_hfield")
    err = []
    for k in pick:
        x = x0 + (ix[k] + 0.5) * res
        y = y0 + (iy[k] + 0.5) * res
        gid = np.zeros(1, np.int32)
        d = mujoco.mj_ray(model, data, np.array([x, y, 50.0]), np.array([0, 0, -1.0]), None, 1,
                          -1, gid)
        if gid[0] == geom:
            err.append(50.0 - d - h[iy[k], ix[k]])
    err = np.abs(np.array(err))
    hid = 0
    for _ in range(2000):
        mujoco.mj_step(model, data)
    return {
        "xml": str(xml),
        "mujoco_version": mujoco.__version__,
        "hfield_nrow_ncol": [int(model.hfield_nrow[hid]), int(model.hfield_ncol[hid])],
        "hfield_size": [float(v) for v in model.hfield_size[hid]],
        "nq": int(model.nq),
        "nu": int(model.nu),
        "ray_vs_npz_abs_err_m": {"n": len(err), "max": float(err.max()),
                                 "p95": float(np.percentile(err, 95))},
        "keyframes": [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_KEY, i)
                      for i in range(model.nkey)],
        "passive_2s_base_z_drop_m": float(model.key_qpos[0][2] - data.qpos[2]),
        "note": "passive (zero-torque) 2 s drop test only; no locomotion controller",
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--npz", type=Path, default=ARTIFACTS / "terrain" / "course_terrain.npz")
    ap.add_argument("--out", type=Path, default=ARTIFACTS / "mujoco")
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    t = np.load(a.npz)
    c = t["course_trajectory"]
    yaw0 = float(np.arctan2(c[3, 2] - c[0, 2], c[3, 1] - c[0, 1]))
    course_pose = (float(c[0, 1]), float(c[0, 2]), yaw0)
    report = []
    for obst in (False, True):
        xml = build_scene(a.npz, a.out, obst, course_pose=course_pose)
        report.append(verify(xml, a.npz))
    (a.out / "mujoco_check.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
