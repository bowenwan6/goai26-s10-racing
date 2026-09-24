"""OFFLINE ONLY: review a noise-tolerant stationary candidate, never unlock maps.

This module is deliberately not imported by the app, ROS worker, or controller.
The 0.02 motion residual envelope is a provisional experiment, not a calibrated
stop threshold. Static recordings cannot establish rejection of real movement.
Published values, gravity, original bags and production gates are never changed.
"""
import argparse
import hashlib
import json
import math
import statistics
import zipfile
from pathlib import Path

LIMITS = dict(window_s=5., motion_xy_m_s=.02, motion_yaw_rad_s=.02,
              gyro_peak_rad_s=.08, gyro_mean_rad_s=.01, gyro_std_rad_s=.01,
              position_from_start_m=.01, attitude_from_start_deg=1.)


def finite(value):
    return type(value) in (float, int) and math.isfinite(value)


def vector(value, length):
    return isinstance(value, (list, tuple)) and len(value) == length and all(map(finite, value))


def rotation_degrees(a, b):
    if not vector(a, 4) or not vector(b, 4):
        return math.inf
    na, nb = math.sqrt(sum(x*x for x in a)), math.sqrt(sum(x*x for x in b))
    if not .99 <= na <= 1.01 or not .99 <= nb <= 1.01:
        return math.inf
    return math.degrees(2*math.acos(min(1., abs(sum(x*y for x, y in zip(a, b))/(na*nb)))))


def evaluate(rows, duration=5.):
    """Return diagnostic metrics only. activation_allowed is ALWAYS false."""
    reasons, warnings, metrics = [], [], {}
    grouped = {topic: [r for r in rows if r.get('topic') == topic]
               for topic in ('imu', 'odom', 'motion')}
    for topic, samples in grouped.items():
        if len(samples) < {'imu': 150*duration, 'odom': 5*duration, 'motion': 3}[topic]:
            reasons.append(topic+': independent sample count insufficient')
        times = [r.get('stamp') for r in samples]
        if not times or not all(map(finite, times)):
            reasons.append(topic+': missing/invalid source timestamps'); continue
        gaps = [b-a for a, b in zip(times, times[1:])]
        max_gap = 2. if topic == 'motion' else .1 if topic == 'imu' else .3
        if times[-1]-times[0] < duration*.65 or any(g <= 0 or g > max_gap for g in gaps):
            reasons.append(topic+': incomplete, repeated, backwards or gapped source time')
        for r in samples:
            wall = r.get('received_wall')
            if not finite(wall) or not -.05 <= wall-r['stamp'] <= (2. if topic == 'motion' else .5):
                reasons.append(topic+': stale or invalid measurement age'); break
        if len({r.get('session_id') for r in samples}) != 1:
            reasons.append(topic+': mixed sessions')
        if topic != 'motion' and len({r.get('frame') for r in samples}) != 1:
            reasons.append(topic+': mixed reference frames')

    imu = grouped['imu']
    if not imu or any(not vector(r.get('values'), 6) for r in imu):
        reasons.append('imu: invalid angular velocity/acceleration')
    else:
        gyro = list(zip(*(r['values'][:3] for r in imu)))
        metrics['gyro_peak_rad_s'] = [max(map(abs, axis)) for axis in gyro]
        metrics['gyro_mean_rad_s'] = [statistics.mean(axis) for axis in gyro]
        metrics['gyro_std_rad_s'] = [statistics.pstdev(axis) for axis in gyro]
        for name in ('peak', 'mean', 'std'):
            if any(abs(v) > LIMITS['gyro_'+name+'_rad_s'] for v in metrics['gyro_'+name+'_rad_s']):
                reasons.append('imu: '+name+' angular motion exceeds candidate envelope')
        if any(r.get('angular_velocity_covariance', [0])[0] == -1 for r in imu):
            reasons.append('imu: angular velocity unavailable')
        # Gravity and instantaneous source values are not subtracted or zeroed.
        imu_angle = max(rotation_degrees(imu[0].get('quaternion'), r.get('quaternion')) for r in imu)
        if not finite(imu_angle) or imu_angle > LIMITS['attitude_from_start_deg']:
            reasons.append('imu: orientation changes or is invalid')

    odom = grouped['odom']
    if not odom or any(not vector(r.get('values'), 3) for r in odom):
        reasons.append('odom: invalid position')
    else:
        metrics['position_from_start_m'] = max(math.dist(odom[0]['values'], r['values']) for r in odom)
        if metrics['position_from_start_m'] > LIMITS['position_from_start_m']:
            reasons.append('odom: translation exceeds candidate envelope')
        angle = max(rotation_degrees(odom[0].get('quaternion'), r.get('quaternion')) for r in odom)
        if not finite(angle) or angle > LIMITS['attitude_from_start_deg']:
            reasons.append('odom: rotation exceeds candidate envelope or is invalid')
        if len({r.get('child_frame') for r in odom}) != 1:
            reasons.append('odom: mixed child frames')

    motion = grouped['motion']
    invocations = set()
    for row in motion:
        n = row.get('navigation', {})
        invocations.add(n.get('invocation'))
        if n.get('fresh') is not True or n.get('goal_none') is not True or n.get('planner_code') != 999:
            reasons.append('planner: stale, active or has target')
        if not vector(n.get('command'), 3) or any(v != 0 for v in n['command']):
            reasons.append('planner: command must remain zero')
        if not vector(n.get('motion'), 3):
            reasons.append('motion: invalid feedback')
        elif any(abs(v) > limit for v, limit in zip(n['motion'], [.02, .02, .02])):
            reasons.append('motion: outside provisional residual envelope')
    if len(invocations) != 1 or not all(invocations):
        reasons.append('planner: missing or changed service invocation')
    for i in range(3):
        values = [r.get('navigation', {}).get('motion', [None]*3)[i] for r in motion
                  if vector(r.get('navigation', {}).get('motion'), 3)]
        if values and len(set(values)) == 1 and values[0] != 0:
            warnings.append('constant nonzero motion component: cached/invalid feedback not excluded')
    return dict(scope='OFFLINE_CANDIDATE_ONLY', activation_allowed=False,
                signal_candidate_pass=not reasons, reasons=sorted(set(reasons)),
                warnings=sorted(set(warnings)), metrics=metrics, limits=LIMITS,
                missing_acceptance=['verified live feedback validity', 'supervised real move-stop negative tests'])


def review(path):
    with zipfile.ZipFile(path) as archive:
        if archive.testzip() is not None:
            raise ValueError('archive CRC failed')
        status = json.loads(archive.read('status.json'))
        rows = [json.loads(line) for line in archive.open('samples.jsonl')]
    start = status['started_mono']
    windows = []
    for offset in range(0, int(status['elapsed_s'])-4, 5):
        selected = [r for r in rows if start+offset <= r['received'] < start+offset+5]
        windows.append(dict(start_s=offset, **evaluate(selected)))
    return dict(source=str(path), source_sha256=hashlib.sha256(Path(path).read_bytes()).hexdigest(),
                rows=len(rows), windows=windows, candidate_passes=sum(w['signal_candidate_pass'] for w in windows),
                activation_allowed=False, production_files_modified=False)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('archive', type=Path)
    args = parser.parse_args()
    print(json.dumps(review(args.archive), ensure_ascii=False, allow_nan=False, indent=2))
