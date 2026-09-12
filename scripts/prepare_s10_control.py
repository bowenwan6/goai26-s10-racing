"""Prepare a persistent drdds overlay from this robot's SDK; never send commands."""
import argparse
from pathlib import Path
import shutil
import tempfile


# Vendor interfaces: supplied drdds snapshot and S10 manual p.48 (MotionState).
# Existing definitions on the selected robot take precedence only if identical.
EXTRA = {
    'MotionInfo': 'MetaType header\nMotionInfoValue data\n',
    'MotionInfoValue': ('float32 vel_x\nfloat32 vel_y\nfloat32 vel_yaw\nfloat32 height\n'
                        'MotionStateValue motion_state\nGaitValue gait_state\n'
                        'float32 payload\nfloat32 remain_mile\n'),
    'MotionState': 'MetaType header\nMotionStateValue data\n',
    'MotionStateValue': 'int32 state\n',
    'GaitValue': 'uint32 gait\n',
}


def fields(text):
    return [line.split('#')[0].split() for line in text.splitlines()
            if line.split('#')[0].strip()]


def prepare(source, output):
    source = source.resolve(strict=True)
    output = output.resolve()
    if output.exists():
        raise ValueError(f'{output} already exists; rebuild it or select a new --output')
    expected = {'MetaType': 'uint64 frame_id\nbuiltin_interfaces/Time stamp\n', **EXTRA}
    for name, definition in expected.items():
        path = source / 'msg' / f'{name}.msg'
        if path.exists() and fields(path.read_text()) != fields(definition):
            raise ValueError(f'{path}: vendor interface differs; inspect before using this overlay')
    for name in ('MetaType', 'Steer', 'SteerValue', 'BatteryData', 'BatteryDataValue', 'StdMsgInt32'):
        if not (source / 'msg' / f'{name}.msg').is_file():
            raise ValueError(f'Missing SDK interface: {name}')
    cmake = (source / 'CMakeLists.txt').read_text()
    if 'GLOB_RECURSE MSG_FILES' not in cmake or '${MSG_FILES}' not in cmake:
        raise ValueError('Expected SDK CMake message glob; inspect this SDK version')
    target = output / 'src/drdds'
    shutil.copytree(source, target)
    for name, definition in EXTRA.items():
        path = target / 'msg' / f'{name}.msg'
        if not path.exists():
            path.write_text(definition, encoding='utf-8')
    print(output)


def check():
    with tempfile.TemporaryDirectory() as folder:
        root = Path(folder)
        source = root / 'sdk'
        (source / 'msg').mkdir(parents=True)
        (source / 'CMakeLists.txt').write_text('GLOB_RECURSE MSG_FILES ${MSG_FILES}')
        for name in ('Steer', 'SteerValue', 'BatteryData', 'BatteryDataValue', 'StdMsgInt32'):
            (source / 'msg' / f'{name}.msg').write_text('int32 value\n')
        (source / 'msg/MetaType.msg').write_text('uint64 frame_id\nbuiltin_interfaces/Time stamp\n')
        prepare(source, root / 'overlay')
        assert (root / 'overlay/src/drdds/msg/MotionState.msg').read_text() == EXTRA['MotionState']
        assert not (source / 'msg/MotionState.msg').exists()
        (source / 'msg/MotionInfo.msg').write_text('int32 incompatible\n')
        try:
            prepare(source, root / 'bad-overlay')
        except ValueError:
            assert not (root / 'bad-overlay').exists()
        else:
            raise AssertionError('Mismatched interfaces accepted')
    print('CONTROL_OVERLAY_CHECK_OK')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path,
                        default=Path.home() / 'goai_embodied_future_material/src/drdds')
    parser.add_argument('--output', type=Path, default=Path.home() / 's10_control_ws')
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    if args.check:
        check()
    else:
        prepare(args.source, args.output)
