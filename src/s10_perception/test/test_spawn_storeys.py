"""Every waypoint on the real course must resolve to its own storey, not to the one below.

Deliberately separate from ``test_segment_spawn.py``, which builds its own scenes so that a
failure there points at the module rather than at the track. This file is the opposite: it
loads the shipped ``course.yaml`` and the SDK's own MJCF and asks the question that actually
went wrong, on the actual geometry it went wrong on. Both are worth having, and confusing them
is how a bug ends up passing tests -- every synthetic scene in this repository was single
storey, and the bug was invisible on a single storey.

What it caught: with no storey reference, thirteen of the thirty-three waypoints -- 7 to 12,
18 to 20 and 23 to 32 -- resolved to 0.479 m, the open floor the upper decks are built over.
Waypoint 28 is at 3.750 m. That is not a near miss, and a sweep was collected against it.

Skips where the upstream SDK checkout is absent, since it is not vendored.
"""

from __future__ import annotations

import math
from pathlib import Path

import mujoco
import pytest
import yaml

from s10_perception.segment_spawn import STOREY_ABOVE, STOREY_BELOW, level_ground

REPO_ROOT = Path(__file__).resolve().parents[3]
COURSE = REPO_ROOT / "src/s10_bringup/config/course.yaml"
SCENE = (
    REPO_ROOT
    / "upstream/goai_embodied_future_material/src/S10_sdk_deploy"
    / "S10_description/s10_mjcf/mjcf/scene.xml"
)

#: Waypoint 0 is the start line, where the race sets its own pose and the spawn override is
#: never consulted. There is no group-0 terrain geom under it at all, so it has no storey to
#: be right or wrong about, and asserting one would be asserting something untrue.
START_LINE = 0


@pytest.fixture(scope="module")
def course_scene():
    if not SCENE.is_file():
        pytest.skip(f"upstream SDK checkout not present at {SCENE}")
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    waypoints = [w["position"] for w in yaml.safe_load(COURSE.read_text())["waypoints"]]
    return model, data, waypoints


def _approach_yaw(waypoints, index: int) -> float:
    """The heading the robot arrives on, which is what orients the footprint."""
    here = waypoints[index]
    nxt = waypoints[min(index + 1, len(waypoints) - 1)]
    return math.atan2(nxt[1] - here[1], nxt[0] - here[0])


def test_every_waypoint_stands_on_its_own_storey(course_scene):
    model, data, waypoints = course_scene
    wrong = []
    for index, (x, y, z) in enumerate(waypoints):
        if index == START_LINE:
            continue
        found = level_ground(model, data, x, y, _approach_yaw(waypoints, index), z)
        # NaN is allowed here and is not the failure this guards against: several waypoints
        # sit hard against a wall or a barrier, and SpawnOverride.pose backs off along the
        # approach until it finds somewhere legal. A number from the wrong storey is the
        # failure, because nothing downstream can tell it from a right one.
        if math.isfinite(found) and not -STOREY_BELOW <= found - z <= STOREY_ABOVE:
            wrong.append(f"WP{index:02d} wants z={z:.3f}, got {found:.3f}")
    assert not wrong, "\n".join(wrong)


def test_the_reference_is_what_makes_the_difference(course_scene):
    """Guards the fix rather than the symptom: without the storey these waypoints move.

    If this ever starts failing because ``z_ref=0`` gives the right answer everywhere, the
    course has changed shape and the assumption behind the whole spawn path is worth
    re-reading before the test is deleted.
    """
    model, data, waypoints = course_scene
    moved = 0
    for index, (x, y, z) in enumerate(waypoints):
        if index == START_LINE:
            continue
        yaw = _approach_yaw(waypoints, index)
        ground_floor = level_ground(model, data, x, y, yaw, 0.0)
        own_storey = level_ground(model, data, x, y, yaw, z)
        both_found = math.isfinite(ground_floor) and math.isfinite(own_storey)
        if both_found and abs(ground_floor - own_storey) > 0.5:
            moved += 1
    assert moved >= 10, f"only {moved} waypoints differ between storeys; the fix is untested"


def test_the_upper_decks_are_where_the_course_says_they_are(course_scene):
    """Named cases, so a regression reads as a place rather than as a count.

    Waypoint 23 is on the deck a metre and a half above the open floor and 28 on the one
    above that. These are the two the void sweep was run on.
    """
    model, data, waypoints = course_scene
    for index in (23, 28):
        x, y, z = waypoints[index]
        yaw = _approach_yaw(waypoints, index)
        assert level_ground(model, data, x, y, yaw, z) == pytest.approx(z, abs=0.15)
        assert level_ground(model, data, x, y, yaw, 0.0) == pytest.approx(0.479, abs=0.05), (
            "the storey underneath is still there; this test would be vacuous without it"
        )
