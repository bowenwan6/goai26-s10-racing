"""Pure logic for the S10 teach page (新 SLAM 采集助手). No ROS imports; Python 3.8+.

Stillness statistics for 3 s marks, mark validation, path length, loop-closure
distance and session naming. Used by teach_worker.py and its tests.
"""
import math
import re
import time

WP_IDS = ['WP%02d' % i for i in range(1, 31)]
SAMPLED_KINDS = {'WP', 'SWIN', 'SWOUT'}          # need a 3 s still sample
INSTANT_KINDS = {'WP_PASS', 'PATH_START', 'PATH_END', 'NOTE'}
MODES = {'mapping': '建图采集', 'survey': '标点', 'path': '示教路径'}

SAMPLE_S = 3.0          # sampling window after the button press
STILL_XY_M = 0.02       # radial std limit for an accurate mark
STILL_YAW_DEG = 1.0
WARN_XY_M = 0.01        # above this: pass, but flagged "偏大"
MIN_SAMPLES = 10
MAX_GAP_S = 0.5
TRAIL_STEP_M = 0.05     # keep a trail point every 5 cm


class TeachError(ValueError):
    """Operator-facing error (Chinese message)."""


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def circ_mean(angles):
    return math.atan2(sum(math.sin(a) for a in angles), sum(math.cos(a) for a in angles))


def still_stats(samples, window_s=SAMPLE_S):
    """samples: list of dicts with t (s), x, y, z, yaw (rad). Returns the mark result."""
    reasons = []
    n = len(samples)
    if n < MIN_SAMPLES:
        return dict(n=n, passed=False, reasons=['位姿样本不足（%d 个，至少 %d 个）：检查定位是否在输出' % (n, MIN_SAMPLES)])
    ts = [s['t'] for s in samples]
    gaps = [b - a for a, b in zip(ts, ts[1:])]
    mx = sum(s['x'] for s in samples) / n
    my = sum(s['y'] for s in samples) / n
    mz = sum(s['z'] for s in samples) / n
    yaw = circ_mean([s['yaw'] for s in samples])
    std_xy = math.sqrt(sum((s['x'] - mx) ** 2 + (s['y'] - my) ** 2 for s in samples) / n)
    std_z = math.sqrt(sum((s['z'] - mz) ** 2 for s in samples) / n)
    yaw_std = math.degrees(math.sqrt(sum(wrap(s['yaw'] - yaw) ** 2 for s in samples) / n))
    max_gap = max(gaps) if gaps else 0.0
    if std_xy > STILL_XY_M:
        reasons.append('机器人没停稳：位置离散 %.1f cm（上限 %.0f cm）' % (std_xy * 100, STILL_XY_M * 100))
    if yaw_std > STILL_YAW_DEG:
        reasons.append('机器人没停稳：朝向离散 %.1f°（上限 %.0f°）' % (yaw_std, STILL_YAW_DEG))
    if max_gap > MAX_GAP_S:
        reasons.append('位姿中断 %.1f s：定位输出不连续' % max_gap)
    if ts[-1] - ts[0] < 0.8 * window_s:
        reasons.append('采样时长不足：定位输出不连续')
    return dict(n=n, duration_s=round(ts[-1] - ts[0], 3), mean=[round(mx, 4), round(my, 4), round(mz, 4)],
                yaw=round(yaw, 5), yaw_deg=round(math.degrees(yaw), 2), std_xy=round(std_xy, 4),
                std_z=round(std_z, 4), yaw_std_deg=round(yaw_std, 3), max_gap_s=round(max_gap, 3),
                passed=not reasons, warn=not reasons and std_xy > WARN_XY_M, reasons=reasons)


def validate_mark(kind, wp_id=None, note=None):
    if kind not in SAMPLED_KINDS | INSTANT_KINDS:
        raise TeachError('未知标记类型')
    if kind in ('WP', 'WP_PASS') and wp_id not in WP_IDS:
        raise TeachError('请选择 WP01–WP30')
    if note is not None:
        if not isinstance(note, str) or len(note) > 200:
            raise TeachError('备注最多 200 字')
    if kind == 'NOTE' and not (note or '').strip():
        raise TeachError('备注不能为空')


def path_length(points):
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(points, points[1:]))


def loop_offset(start, now):
    """start/now: dict x, y, yaw. Horizontal distance and heading difference."""
    return dict(dist=math.hypot(now['x'] - start['x'], now['y'] - start['y']),
                heading_deg=math.degrees(abs(wrap(now['yaw'] - start['yaw']))))


def session_id(label, now=None):
    safe = re.sub(r'[^A-Za-z0-9_-]+', '_', (label or '').strip())[:32].strip('_')
    stamp = time.strftime('%Y%m%d-%H%M%S', time.localtime(now))
    return stamp + ('-' + safe if safe else '')


SESSION_RE = re.compile(r'^\d{8}-\d{6}(-[A-Za-z0-9_-]{1,32})?$')
FILE_RE = re.compile(r'^[A-Za-z0-9_.-]{1,96}$')


def wp_board(marks):
    """Latest valid state per WP for the 30-button board: none | pass | warn | fail."""
    board = {w: 'none' for w in WP_IDS}
    for m in marks:
        if m.get('void') or m.get('kind') != 'WP':
            continue
        r = m.get('result') or {}
        board[m['wp_id']] = 'fail' if not r.get('passed') else ('warn' if r.get('warn') else 'pass')
    return board


def switch_pairs(marks):
    """Pair SWIN/SWOUT in order; returns list of dicts and whether a zone is open."""
    pairs, open_ = [], None
    for m in marks:
        if m.get('void') or m.get('kind') not in ('SWIN', 'SWOUT') or not (m.get('result') or {}).get('passed'):
            continue
        if m['kind'] == 'SWIN':
            if open_ is not None:
                pairs.append(dict(swin=open_, swout=None))
            open_ = m['seq']
        else:
            pairs.append(dict(swin=open_, swout=m['seq']))
            open_ = None
    if open_ is not None:
        pairs.append(dict(swin=open_, swout=None))
    return pairs
