"""Locate and import the contest simulator.

The upstream contest material is distributed as a private repository and is not vendored
here. ``scripts/setup_upstream.sh`` clones it into ``upstream/`` at the workspace root;
this module finds that checkout and imports its simulator module by file path, so we can
subclass the simulator rather than fork it.
"""

from __future__ import annotations

import importlib.util
import os
import sys
from pathlib import Path
from types import ModuleType

UPSTREAM_ENV_VAR = "S10_UPSTREAM_DIR"
UPSTREAM_DIR_NAME = "goai_embodied_future_material"

_SIM_RELPATH = Path("src/S10_sdk_deploy/interface/robot/simulation/mujoco_simulation_ros2.py")


def _candidate_roots() -> list[Path]:
    candidates: list[Path] = []

    override = os.environ.get(UPSTREAM_ENV_VAR)
    if override:
        candidates.append(Path(override).expanduser().resolve())

    # Walk up from this file looking for a sibling `upstream/` directory. This resolves
    # both from a source checkout and from an installed share/ directory.
    here = Path(__file__).resolve()
    for parent in here.parents:
        candidates.append(parent / "upstream" / UPSTREAM_DIR_NAME)
        candidates.append(parent / UPSTREAM_DIR_NAME)

    return candidates


def find_upstream_root() -> Path:
    """Return the root of the upstream checkout, or raise with actionable guidance."""
    for candidate in _candidate_roots():
        if (candidate / _SIM_RELPATH).is_file():
            return candidate

    raise FileNotFoundError(
        "Could not locate the contest material. Run scripts/setup_upstream.sh, or set "
        f"{UPSTREAM_ENV_VAR} to the root of your goai_embodied_future_material checkout."
    )


def load_simulator_module() -> ModuleType:
    """Import upstream ``mujoco_simulation_ros2`` as a module named ``s10_upstream_sim``.

    The upstream package is not installed on ``sys.path``, so it is loaded from its file
    location. The module is cached in ``sys.modules`` under a stable name to keep repeated
    imports and pickling well behaved.
    """
    cached = sys.modules.get("s10_upstream_sim")
    if cached is not None:
        return cached

    sim_path = find_upstream_root() / _SIM_RELPATH
    spec = importlib.util.spec_from_file_location("s10_upstream_sim", sim_path)
    if spec is None or spec.loader is None:
        raise ImportError(f"Cannot build an import spec for {sim_path}")

    module = importlib.util.module_from_spec(spec)
    sys.modules["s10_upstream_sim"] = module
    spec.loader.exec_module(module)
    return module


def default_track_xml() -> Path:
    """Path to the upstream track scene, used as the base for our sensor-equipped MJCF."""
    return find_upstream_root() / "src/S10_sdk_deploy/S10_description/s10_mjcf/mjcf/S10_track.xml"
