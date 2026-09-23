#!/usr/bin/env python3
"""Select the HIM SDK for simulation; the default Gate16 patcher is unchanged.

Use prepare_s10_sdk_copy.py --him-model for hardware (DDS entry point).
"""
import argparse
from pathlib import Path

from patch_upstream import DEFAULT_UPSTREAM, SDK, Edit, resolve_upstream
from patch_upstream import EDITS as MAIN_EDITS

ROOT = Path(__file__).resolve().parents[1]
HEADERS = (
    (ROOT / 'integration/ros_cmd_interface.hpp', 'interface/user_command/ros_cmd_interface.hpp'),
    (ROOT / 'integration/s10_policy_runner.hpp', 'run_policy/s10_policy_runner.hpp'),
    (ROOT / 'integration/rl_control_state.hpp', 'state_machine/quadruped_wheel/rl_control_state.hpp'),
    (ROOT / 'integration/standup_state.hpp', 'state_machine/quadruped_wheel/standup_state.hpp'),
)
# Share only the existing ROS bridge and build dependencies, not the Gate16 controller.
BRIDGE_MARKERS = {
    'kRosTopic', 'ros_cmd_interface.hpp', 'RosCmdInterface',
    'find_package(geometry_msgs', '  geometry_msgs\n', '<depend>geometry_msgs</depend>',
}
EDITS = [edit for edit in MAIN_EDITS if edit.marker in BRIDGE_MARKERS] + [
    Edit(
        path=SDK / 'state_machine/parameters/control_parameters.h',
        anchor='    float pre_height_, stand_height_;',
        addition='\n    VecXf policy_stand_pose_;  // HIM default pose; empty for legacy policies\n',
        marker='policy_stand_pose_',
    ),
    Edit(
        path=SDK / 'state_machine/quadruped_wheel/qw_state_machine.hpp',
        anchor='    void Stop(){\n        sc_ptr_->Stop();',
        addition='    void Stop(){\n        current_controller_->OnExit();  // Join controller-owned threads\n        sc_ptr_->Stop();',
        marker='Join controller-owned threads', mode='replace',
    ),
]


def install_headers(root):
    changed = False
    for source, relative in HEADERS:
        target = root / SDK / relative
        content = source.read_text(encoding='utf-8')
        if not target.is_file() or target.read_text(encoding='utf-8') != content:
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_text(content, encoding='utf-8')
            changed = True
    return changed


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--upstream', type=Path, default=DEFAULT_UPSTREAM)
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    root = resolve_upstream(args.upstream.resolve())
    if args.check:
        headers_ok = all((root / SDK / dest).is_file() and
                         (root / SDK / dest).read_text(encoding='utf-8') == source.read_text(encoding='utf-8')
                         for source, dest in HEADERS)
        edits_ok = all(edit.is_applied(root) for edit in EDITS)
        if not headers_ok or not edits_ok:
            raise SystemExit('HIM SDK is missing or stale; run scripts/patch_him_upstream.py and rebuild')
        print('HIM_SDK_PATCH_CHECK_OK')
        return
    install_headers(root)
    for edit in EDITS:
        edit.apply(root)
    print(f'HIM SDK installed at {root / SDK}; rebuild before running')


if __name__ == '__main__':
    main()
