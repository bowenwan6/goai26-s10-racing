"""Field configuration and input validation, shared by runtime and acceptance tests."""

from __future__ import annotations

import math

import numpy as np

from native_transfer.router import GAITS
from real_transfer.geometry import rigid_transform

VERIFICATIONS = (
    "map_and_global_status",
    "odom_reference",
    "cloud_frame_deskew_self_filter",
    "perception_projection",
    "route_and_staging_points",
    "operator_on_site",
    "native_command_timeout_and_stop",
)


def admission_reasons(config, *, policy_probe=False):
    reasons = [
        "unverified_" + k
        for k in VERIFICATIONS
        if config.get("verified", {}).get(k) is not True
        and not (policy_probe and k == "native_command_timeout_and_stop")
    ]
    if config.get("map_id") != "0914_fr_v3-20260914-142008":
        reasons.append("map_not_v3")
    if type(config.get("healthy_location_code")) is not int:
        reasons.append("unknown_location_status_semantics")
    offset = config.get("body_z_offset")
    if not isinstance(offset, (int, float)) or not math.isfinite(offset) or not 0 < offset < 1:
        reasons.append("unknown_body_z_offset")
    for key in ("odom_child_from_base", "base_from_cloud"):
        try:
            rigid_transform(config.get(key))
        except (ValueError, TypeError):
            reasons.append("invalid_" + key)
    frames = config.get("frames", {})
    if not all(k in frames and isinstance(frames[k], str) for k in ("map", "odom_child", "cloud")):
        reasons.append("missing_frame_contract")
    if not policy_probe and config.get("policy_call_acceptance") != "passed_on_048":
        reasons.append("official_policy_call_not_yet_accepted")
    return reasons


def validate_route(route):
    points = route.get("waypoints", [])
    if not points:
        raise ValueError("route must contain waypoints")
    for i, p in enumerate(points):
        xyz = np.asarray(p.get("position"), float)
        if xyz.shape != (3,) or not np.isfinite(xyz).all():
            raise ValueError("route requires finite XYZ")
        if p.get("index") != i or p.get("kind") not in GAITS:
            raise ValueError("route requires sequential indices and explicit incoming gait")
    return points


class SourceClock:
    """A repeated old source stamp cannot be refreshed by a DDS receipt."""

    def __init__(self):
        self.previous = {}

    def check(self, name, stamp, wall, *, max_age=0.35):
        if not math.isfinite(stamp) or not -0.03 <= wall - stamp <= max_age or stamp <= 0:
            raise ValueError(name + "_source_timestamp_invalid")
        if name in self.previous and stamp <= self.previous[name]:
            raise ValueError(name + "_source_replayed_or_reordered")
        self.previous[name] = stamp


def conservative_scan(points, bins=72):
    """Observed cloud extent, with unknown angular bins kept unknown.

    Ground returns provide a bounded visibility estimate; returns in the body-height
    obstacle band shorten it. This requires field validation of the cloud's filtering
    and occlusion behaviour. It is not a certified free-space/negative-obstacle map.
    """
    p = np.asarray(points, float)
    if p.ndim != 2 or p.shape[1] != 3:
        raise ValueError("cloud must be Nx3")
    p = p[np.isfinite(p).all(axis=1)]
    d = np.linalg.norm(p[:, :2], axis=1)
    keep = (d >= 0.25) & (d <= 10.0)
    p, d = p[keep], d[keep]
    ids = np.floor((np.arctan2(p[:, 1], p[:, 0]) + np.pi) / (2 * np.pi) * bins).astype(int)
    ids = np.clip(ids, 0, bins - 1)
    extent = np.zeros(bins)
    np.maximum.at(extent, ids, d)
    obstacle = np.full(bins, np.inf)
    band = (p[:, 2] >= -0.25) & (p[:, 2] <= 0.75)
    np.minimum.at(obstacle, ids[band], d[band])
    ranges = np.minimum(extent, obstacle)
    ranges[extent == 0] = np.nan
    return ranges
