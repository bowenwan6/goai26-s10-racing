"""Tiny 2.5-D kinematic sandbox for the route_v2 follower core (tests / harness smoke).

Not a physics simulator: a height field (ground function + axis-aligned boxes), a
kinematic body integrating ``(vx, vy, yaw_rate)`` in the body frame, and sensors that
mimic the native adapters' conventions:

* :func:`sense_height_grid` -- 13 x 9 grid, robot yaw frame, heights relative to the base,
  topmost value per cell (like ``real_transfer.geometry.height_grid``), all cells valid
  unless ``occluded`` says otherwise.
* :func:`sense_scan` -- 72 bins, range to the first point whose height is inside the body
  band [-0.25, 0.75] m relative to the base (like ``conservative_scan``), else ``max_range``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from s10_auto_nav.route_planner import HEIGHT_CELL, HEIGHT_XS, HEIGHT_YS


@dataclass
class Box:
    x0: float
    x1: float
    y0: float
    y1: float
    top: float  # absolute z of the top face


@dataclass
class HeightField:
    boxes: list[Box] = field(default_factory=list)
    ground: float = 0.0

    def height(self, xy: np.ndarray) -> np.ndarray:
        xy = np.asarray(xy, float).reshape(-1, 2)
        h = np.full(len(xy), self.ground)
        for b in self.boxes:
            inside = (xy[:, 0] >= b.x0) & (xy[:, 0] <= b.x1) & (xy[:, 1] >= b.y0) & (xy[:, 1] <= b.y1)
            h = np.where(inside, np.maximum(h, b.top), h)
        return h


def _rot(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]])


def sense_height_grid(field_: HeightField, pose, sub: int = 3, max_spread: float = 0.08):
    x, y, z, yaw = pose
    offs = (np.arange(sub) + 0.5) / sub * HEIGHT_CELL - HEIGHT_CELL / 2
    grid = np.zeros((len(HEIGHT_XS), len(HEIGHT_YS)))
    mask = np.ones(grid.shape, bool)
    rot = _rot(yaw)
    for i, gx in enumerate(HEIGHT_XS):
        for j, gy in enumerate(HEIGHT_YS):
            local = np.array([[gx + a, gy + b] for a in offs for b in offs])
            world = local @ rot.T + np.array([x, y])
            hs = field_.height(world)
            grid[i, j] = hs.max() - z
            # height_grid rejects cells whose returns span more than max_spread (mixed levels).
            mask[i, j] = hs.max() - hs.min() <= max_spread
    return grid, mask


def sense_scan(field_: HeightField, pose, bins: int = 72, max_range: float = 10.0,
               band=(-0.25, 0.75), step: float = 0.05):
    x, y, z, yaw = pose
    angles = -math.pi + (np.arange(bins) + 0.5) * (2 * math.pi / bins)
    dists = np.arange(0.25, max_range, step)
    ranges = np.full(bins, max_range)
    for k, a in enumerate(angles):
        th = yaw + a
        pts = np.array([x, y]) + dists[:, None] * np.array([math.cos(th), math.sin(th)])
        rel = field_.height(pts) - z
        hit = (rel >= band[0]) & (rel <= band[1])
        if hit.any():
            ranges[k] = dists[int(np.argmax(hit))]
    return ranges, angles


@dataclass
class KinematicBody:
    x: float
    y: float
    yaw: float
    body_z_offset: float = 0.42
    field_: HeightField = field(default_factory=HeightField)

    @property
    def pose(self):
        z = float(self.field_.height(np.array([[self.x, self.y]]))[0]) + self.body_z_offset
        return (self.x, self.y, z, self.yaw)

    def step(self, command, dt: float) -> None:
        vx, vy, wz = command
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        self.x += (vx * c - vy * s) * dt
        self.y += (vx * s + vy * c) * dt
        self.yaw = math.atan2(math.sin(self.yaw + wz * dt), math.cos(self.yaw + wz * dt))


def run_closed_loop(core, body: KinematicBody, *, duration: float, dt: float = 0.1):
    """Drive ``core`` (a RouteFollowerCore) in the sandbox; returns a list of dicts."""
    trace = []
    t = 0.0
    while t < duration:
        pose = body.pose
        grid, mask = sense_height_grid(body.field_, pose)
        ranges, angles = sense_scan(body.field_, pose)
        out = core.step(t, pose, grid, mask, ranges, angles)
        trace.append({"t": t, "pose": pose, "status": out.status, "reason": out.reason,
                      "s": out.s, "d": out.d, "command": out.command, "target": out.target_id})
        if out.finished:
            break
        body.step(out.command, dt)
        t += dt
    return trace
