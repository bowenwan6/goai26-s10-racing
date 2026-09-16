"""Check real ARM inference traces against the selected ONNX and its contract."""
import json
from pathlib import Path
import argparse
import numpy as np
import onnxruntime as ort

parser = argparse.ArgumentParser(description='Verify a recorded one-second, zero-command HIM takeover (no robot connection).')
parser.add_argument('run', type=Path)
parser.add_argument('--model', type=Path, required=True)
parser.add_argument('--partial', action='store_true', help='Allow an interrupted trial with at least one frame')
args = parser.parse_args()
run, model = args.run, args.model
cfg = json.loads(model.with_suffix('.json').read_text())
records = [json.loads(line) for line in (run / 'policy_trace.jsonl').read_text().splitlines()]
result_path = next(run.glob('result-*.json'))
result = json.loads(result_path.read_text())
partial = args.partial
assert partial or (result['completed'] and result['error'] is None), result['error']
assert result['policy'] == 'HIM checkpoint 1500'
assert (1 <= len(records) <= 70) if partial else (40 <= len(records) <= 70), len(records)
session = ort.InferenceSession(str(model), providers=['CPUExecutionProvider'])
max_error = 0.
wheel_commands = []
for index, record in enumerate(records):
    obs = np.asarray(record['observation'], dtype=np.float32)
    raw = np.asarray(record['raw_action_policy'], dtype=np.float32)
    clipped = np.clip(raw, -cfg['clip_actions'], cfg['clip_actions'])
    assert obs.shape == (342,) and raw.shape == (16,)
    assert np.isfinite(obs).all() and np.isfinite(raw).all()
    np.testing.assert_array_equal(record['command'], [0, 0, 0])
    if index == 0:
        np.testing.assert_array_equal(obs[41:], np.zeros(301))
    else:
        np.testing.assert_allclose(obs[57:], records[index-1]['observation'][:285], atol=1e-6, rtol=0)
        np.testing.assert_allclose(obs[41:57], records[index-1]['clipped_action_policy'], atol=1e-6, rtol=0)
    q = np.asarray(record['joint_pos_robot']) - cfg['default_dof_pos']
    q[3::4] = 0
    np.testing.assert_allclose(obs[9:25], q, atol=2e-6, rtol=0)
    np.testing.assert_allclose(obs[25:41], np.asarray(record['joint_vel_robot'])*.05, atol=2e-6, rtol=0)
    expected = np.asarray(cfg['default_dof_pos']) + clipped * cfg['action_scale']
    expected[3::4] = clipped[3::4] * cfg['vel_scale']
    np.testing.assert_allclose(record['decoded_command_robot'], expected, atol=2e-6, rtol=0)
    predicted = session.run(['actions'], {'obs': obs[None]})[0][0]
    max_error = max(max_error, float(np.max(np.abs(predicted-raw))))
    np.testing.assert_allclose(raw, predicted, atol=1e-4, rtol=1e-4)
    wheel_commands.append(expected[3::4])
dt = np.diff([r['monotonic_s'] for r in records])
assert not dt.size or (np.all(dt > 0) and np.max(dt) < .06), dt.tolist()
pd_samples = [s['data']['joint_commands']['values'] for s in result['samples']
              if s['state'] == 'rl_control' and 'joint_commands' in s['data']]
if partial and result['failure'] and result['failure']['state'] == 'rl_control':
    pd_samples.append(result['failure']['data']['joint_commands']['values'])
assert pd_samples
for command in pd_samples:
    np.testing.assert_allclose(np.asarray(command)[:,3], cfg['p_gains'], atol=1e-5)
    np.testing.assert_allclose(np.asarray(command)[:,4], cfg['d_gains'], atol=1e-5)
rl = [s for s in result['samples'] if s['state'] == 'rl_control']
rpy = [s['data']['imu']['values'][:3] for s in rl]
if partial and result['failure']:
    rpy.append(result['failure']['data']['imu']['values'][:3])
first = records[0]
error = np.asarray(first['decoded_command_robot']) - first['joint_pos_robot']
pd_torque = error * cfg['p_gains'] - np.asarray(first['joint_vel_robot']) * cfg['d_gains']
summary = dict(trace_contract_passed=True, trial_completed=result['completed'],
    trial_error=result['error'], dry=result['dry'], model=str(model), policy_frames=len(records),
    policy_hz=1/float(np.mean(dt)) if dt.size else None,
    max_policy_gap_ms=float(np.max(dt))*1000 if dt.size else None,
    arm_vs_local_onnx_max_abs_error=max_error,
    max_abs_roll_pitch_deg=np.max(np.abs(np.asarray(rpy)[:,:2]), axis=0).tolist(),
    max_abs_wheel_target_rad_s=float(np.max(np.abs(wheel_commands))),
    first_frame_unblended_leg_target_error_rad={cfg['dof_names'][i]: float(error[i]) for i in range(16) if i%4 != 3},
    first_frame_unblended_pd_torque_estimate_nm={cfg['dof_names'][i]: float(pd_torque[i]) for i in range(16) if i%4 != 3},
    final_state=result['final_state'], returncode=result['returncode'])
summary['max_abs_roll_pitch_deg'] = [v*180/np.pi for v in summary['max_abs_roll_pitch_deg']]
(run / 'verification.json').write_text(json.dumps(summary, indent=2))
print(json.dumps(summary, indent=2))
