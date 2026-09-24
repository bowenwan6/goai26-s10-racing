"""Compile and run the C++ ownership gate that decides what reaches sixteen actuators.

``integration/joint_command_owner.hpp`` is the only place in this project where a safety
property is written in C++, because it is the only place it can be written: ``/JOINTS_CMD`` is
created and published inside the contest SDK's own process and no external node can take it
away. A header that is exercised for the first time during a run is not a gate anybody should
trust, and it earned that suspicion -- the first time the checks below were compiled they
found that the gate could never hand ownership to a climb policy at all. It went climb, stale,
hold, climb, stale, hold, because completing a handover dropped the command buffer and the
same tick then read the empty buffer as a dropped message.

The checks themselves are in ``integration/test/joint_command_owner_test.cpp``, a plain
executable: adding a C++ test framework for one header would be the larger change. This module
compiles it against the SDK's own headers and ROS, runs it, and turns each ``ok``/``FAIL`` line
into something pytest reports. Nothing here starts a ROS daemon; the gate's ROS side is three
subscriptions calling the same public methods the test drives directly.

It skips, rather than fails, where the pieces are not present -- no compiler, no ROS, no
upstream checkout -- so a machine that can only run the Python suite still runs it.
"""

from __future__ import annotations

import shutil
import subprocess
import tempfile
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
INTEGRATION = REPO_ROOT / "robot" / "integration"
SOURCE = INTEGRATION / "test/joint_command_owner_test.cpp"
SDK = REPO_ROOT / "upstream/goai_embodied_future_material/src/S10_sdk_deploy"
ROS = Path("/opt/ros/jazzy")

#: Linked explicitly rather than through ament, because this is one translation unit and
#: standing up a colcon package to build it would put the gate's own test inside the build it
#: is meant to be independent of. ``tracetools`` is not optional: rclcpp's publish path
#: references it directly.
LIBRARIES = ["rclcpp", "rcutils", "rcl", "std_msgs__rosidl_typesupport_cpp", "tracetools"]


def _requirements() -> str | None:
    """Return why this cannot run here, or ``None`` if it can."""
    if shutil.which("g++") is None:
        return "no C++ compiler"
    if not SOURCE.is_file():
        return f"missing {SOURCE}"
    if not (SDK / "include/types/common_types.h").is_file():
        return "upstream SDK checkout not present"
    if not (ROS / "include").is_dir():
        return f"no ROS installation at {ROS}"
    return None


@pytest.fixture(scope="module")
def gate_test_output() -> str:
    reason = _requirements()
    if reason:
        pytest.skip(reason)

    # Jazzy installs each package's headers under include/<package>/, so the include path is
    # the set of those directories rather than include/ itself.
    includes = [f"-I{path}" for path in sorted((ROS / "include").iterdir()) if path.is_dir()]
    with tempfile.TemporaryDirectory() as tmp:
        binary = Path(tmp) / "joint_command_owner_test"
        compile_command = [
            "g++",
            "-std=c++17",
            "-O0",
            "-o",
            str(binary),
            str(SOURCE),
            f"-I{INTEGRATION}",
            f"-I{SDK / 'include/types'}",
            f"-I{SDK / 'third_party/eigen'}",
            *includes,
            f"-L{ROS / 'lib'}",
            *[f"-l{name}" for name in LIBRARIES],
        ]
        built = subprocess.run(compile_command, capture_output=True, text=True)
        assert built.returncode == 0, f"the gate does not compile:\n{built.stderr}"

        # The gate's transitions are timed in real seconds -- a handover is 0.25 s and a stale
        # command 0.2 s -- so the executable sleeps its way through them and takes a couple of
        # seconds. The timeout is a deadlock backstop, not a performance bound.
        run = subprocess.run([str(binary)], capture_output=True, text=True, timeout=120)
    assert run.returncode == 0, f"the gate failed its own checks:\n{run.stdout}\n{run.stderr}"
    return run.stdout


def test_every_check_passes(gate_test_output: str) -> None:
    failures = [line for line in gate_test_output.splitlines() if line.startswith("FAIL")]
    assert not failures, "\n".join(failures)


def test_the_checks_actually_ran(gate_test_output: str) -> None:
    """A gate that compiles and prints nothing would otherwise pass the test above.

    The count is a floor rather than an equality so that adding a check does not fail this,
    but deleting the body of the test does.
    """
    passed = [line for line in gate_test_output.splitlines() if line.startswith("ok")]
    assert len(passed) >= 20, gate_test_output


@pytest.mark.parametrize(
    "guarantee",
    [
        "the shipped policy owns the joints at start",
        "the climb policy takes over after the hold",
        "the shipped policy contributes nothing while the climb policy drives",
        "a stale climb command falls back to the hold",
        "and not silently handing back to the shipped policy",
        "a target submitted before the handover cannot be replayed after it",
        "and no NaN reaches the actuators",
    ],
)
def test_named_guarantee_holds(gate_test_output: str, guarantee: str) -> None:
    """Name the properties here as well, so a deleted C++ check fails a named Python test.

    These are the ones with a robot on the other end of them: single ownership, no step change
    across a handover, no silent hand-back, no late command, no NaN. Losing one of them
    silently because a line was removed from a .cpp file is the failure this guards against.
    """
    assert f"ok   {guarantee}" in gate_test_output, gate_test_output
