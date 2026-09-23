"""Run on AGX: prepare an isolated copy only; never build, launch, or control motors."""
import argparse
import hashlib
import json
import shutil
import tempfile
from pathlib import Path


def adjust(main, interface, angle_unit):
    if angle_unit not in ('rad', 'deg'):
        raise ValueError('Measure IMU units first: rad or deg')
    old_qos = '"/IMU_DATA", 10,'
    new_qos = '"/IMU_DATA", rclcpp::SensorDataQoS(),'
    old_rpy = 'rpy_ = Vec3f(Deg2Rad(msg->data.roll), Deg2Rad(msg->data.pitch), Deg2Rad(msg->data.yaw));'
    new_rpy = 'rpy_ = Vec3f(msg->data.roll, msg->data.pitch, msg->data.yaw);'
    if old_qos in interface:
        interface = interface.replace(old_qos, new_qos)
    elif new_qos not in interface:
        raise ValueError('Different IMU subscription implementation; inspect manually')
    if old_rpy not in interface and new_rpy not in interface:
        raise ValueError('Different IMU angle implementation; inspect manually')
    if angle_unit == 'rad':
        interface = interface.replace(old_rpy, new_rpy)
    else:
        interface = interface.replace(new_rpy, old_rpy)
    if 'int main(){' in main and 'rclcpp::init(0, 0);' in main:
        main = main.replace('int main(){', 'int main(int argc, char** argv){')
        main = main.replace('rclcpp::init(0, 0);', 'rclcpp::init(argc, argv);')
    elif 'int main(int argc, char** argv)' not in main or 'rclcpp::init(argc, argv);' not in main:
        raise ValueError('Different main/ROS initialization; inspect manually')
    return main, interface


def him_headers():
    """Use the shared HIM implementation with the hardware SDK's original state set."""
    integration = Path(__file__).resolve().parents[1] / 'integration'
    return {
        'run_policy/s10_policy_runner.hpp': (integration / 's10_policy_runner.hpp').read_text(encoding='utf-8'),
        'state_machine/quadruped_wheel/rl_control_state.hpp': (integration / 'rl_control_state.hpp').read_text(encoding='utf-8'),
        'state_machine/quadruped_wheel/standup_state.hpp': (integration / 'standup_state.hpp').read_text(encoding='utf-8'),
    }


def prepare(source, angle_unit, him_model=None):
    source = source.resolve(strict=True)
    names = ('main.cpp', 'interface/robot/hardware/dds_interface.hpp', 'policy/policy.onnx')
    baseline = {name: hashlib.sha256((source / name).read_bytes()).hexdigest() for name in names}
    main, interface = adjust((source / names[0]).read_text(), (source / names[1]).read_text(), angle_unit)
    if not (source / 'third_party').is_dir():
        raise ValueError('Expected bundled third_party directory is missing')
    headers = {}
    if him_model is not None:
        him_model = him_model.resolve(strict=True)
        him_model.with_suffix('.json').resolve(strict=True)
        if him_model.suffix != '.onnx':
            raise ValueError('HIM requires an ONNX model and its same-stem JSON sidecar')
        if 'RemoteCommandType::kDDS' not in main or 'S10_MANUAL' in main or 'RemoteCommandType::kRosTopic' in main:
            raise ValueError('Use the verified hardware DDS entry point, not the simulation SDK')
        headers = him_headers()
        parameters = 'state_machine/parameters/control_parameters.h'
        text = (source / parameters).read_text(encoding='utf-8')
        if 'policy_stand_pose_' not in text:
            anchor = '    float pre_height_, stand_height_;'
            if text.count(anchor) != 1:
                raise ValueError('Unknown SDK control parameters; inspect before installing HIM')
            text = text.replace(anchor, anchor + '\n    VecXf policy_stand_pose_;  // HIM sidecar pose; empty preserves legacy height planning\n')
        headers[parameters] = text
    root = Path(tempfile.mkdtemp(prefix='s10-sdk-isolated-'))
    target = root / 'src/S10_sdk_deploy'
    shutil.copytree(source, target, ignore=shutil.ignore_patterns('third_party', 'S10_description', '__pycache__'))
    (target / 'third_party').symlink_to(source / 'third_party', target_is_directory=True)
    (target / names[0]).write_text(main)
    (target / names[1]).write_text(interface)
    if hashlib.sha256((target / names[2]).read_bytes()).hexdigest() != baseline[names[2]]:
        raise RuntimeError('Copied model differs from source')
    if any(hashlib.sha256((source / name).read_bytes()).hexdigest() != digest for name, digest in baseline.items()):
        raise RuntimeError('Source changed during preparation; inspect before using copy')
    for name, text in headers.items():
        (target / name).write_text(text, encoding='utf-8')
    if him_model is not None:
        shutil.copyfile(him_model, target / 'policy/policy.onnx')
        shutil.copyfile(him_model.with_suffix('.json'), target / 'policy/policy.json')
    (root / 'original-baseline.json').write_text(json.dumps(baseline, indent=2))
    (root / 'preparation.json').write_text(json.dumps(dict(source=str(source), angle_unit=angle_unit,
        him_model=str(him_model) if him_model else None,
        scope='Source/model preparation only; no build or motor control'), indent=2))
    print(root)


def check():
    main = 'int main(){ rclcpp::init(0, 0); }'
    interface = '"/IMU_DATA", 10,\nrpy_ = Vec3f(Deg2Rad(msg->data.roll), Deg2Rad(msg->data.pitch), Deg2Rad(msg->data.yaw));'
    fixed = adjust(main, interface, 'rad')
    assert 'init(argc, argv)' in fixed[0] and 'SensorDataQoS' in fixed[1]
    assert 'Deg2Rad' not in fixed[1]
    assert adjust(*fixed, 'rad') == fixed
    assert 'Deg2Rad' in adjust(*fixed, 'deg')[1]
    try:
        adjust('unknown main', interface, 'rad')
    except ValueError:
        pass
    else:
        raise AssertionError('Unknown source must not be patched blindly')
    headers = him_headers()
    rl = headers['state_machine/quadruped_wheel/rl_control_state.hpp']
    assert 'kObstacle' not in rl and 'ApplyHighSpeed' not in rl
    assert all(name in rl for name in ('BlendHimHandover(res,', 'GuardHimCommand(res,', 'policy_stand_pose_'))
    assert 'monotonic_s' in headers['run_policy/s10_policy_runner.hpp']
    print('PREPARE_COPY_CHECK_OK')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, help='Existing S10_sdk_deploy source package')
    parser.add_argument('--imu-angle-unit', choices=('rad', 'deg'), help='Unit verified on this robot')
    parser.add_argument('--him-model', type=Path, help='Install shared HIM headers and this ONNX + JSON into the isolated copy')
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    if args.check:
        check()
    elif args.source and args.imu_angle_unit:
        prepare(args.source, args.imu_angle_unit, args.him_model)
    else:
        parser.error('--source and --imu-angle-unit are required')
