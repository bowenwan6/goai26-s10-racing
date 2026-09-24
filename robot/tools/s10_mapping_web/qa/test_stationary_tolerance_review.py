"""Local candidate tests; no ROS, network, robot activation or production imports."""
import copy
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from stationary_tolerance_review import evaluate


def fixture():
    rows = []
    for topic, hz in [('imu', 200), ('odom', 10), ('motion', 1)]:
        for i in range(5*hz):
            t = 100+i/hz
            r = dict(topic=topic, stamp=t, received_wall=t+.02, session_id='same',
                     frame='map', child_frame='', quaternion=[0., 0., 0., 1.])
            if topic == 'imu':
                r['values'] = [.003*math.sin(i), .004*math.cos(i), .0005, 0., 0., 9.81]
            elif topic == 'odom':
                r['values'] = [.001*math.sin(i), 0., 0.]
            else:
                r['navigation'] = dict(fresh=True, goal_none=True, planner_code=999,
                                       invocation='one', command=[0., 0., 0.], motion=[.009, -.017, .007])
            rows.append(r)
    return rows


class CandidateTests(unittest.TestCase):
    def test_static_noise_candidate_does_not_unlock_or_modify_original(self):
        rows = fixture(); before = copy.deepcopy(rows); r = evaluate(rows)
        self.assertTrue(r['signal_candidate_pass']); self.assertFalse(r['activation_allowed'])
        self.assertEqual(rows, before); self.assertTrue(r['warnings'])

    def assert_rejected(self, change):
        rows = fixture(); change(rows)
        r = evaluate(rows)
        self.assertFalse(r['signal_candidate_pass'], r); self.assertFalse(r['activation_allowed'])

    def test_real_translation_with_zero_gyro_is_rejected(self):
        self.assert_rejected(lambda rows: [r.update(values=[.01*(r['stamp']-100), 0., 0.]) for r in rows if r['topic']=='odom'])

    def test_rotation_is_rejected(self):
        self.assert_rejected(lambda rows: [r.update(quaternion=[0., 0., math.sin(.015*(r['stamp']-100)), math.cos(.015*(r['stamp']-100))]) for r in rows if r['topic']=='odom'])

    def test_zero_mean_oscillation_not_mistaken_for_noise(self):
        self.assert_rejected(lambda rows: [r['values'].__setitem__(0, .035*math.sin(i)) for i,r in enumerate(rows) if r['topic']=='imu'])

    def test_nonzero_command_still_rejected(self):
        self.assert_rejected(lambda rows: [r['navigation']['command'].__setitem__(0,.001) for r in rows if r['topic']=='motion'])

    def test_stale_repeated_nan_and_missing_data_rejected(self):
        for change in [lambda rows: rows[0].update(received_wall=102),
                       lambda rows: rows[1].update(stamp=100),
                       lambda rows: rows[0]['values'].__setitem__(0,math.nan),
                       lambda rows: rows.__setitem__(slice(None),[r for r in rows if r['topic']!='motion'])]:
            self.assert_rejected(change)

    def test_active_goal_and_changed_service_rejected(self):
        self.assert_rejected(lambda rows: rows[-1]['navigation'].update(goal_none=False))
        self.assert_rejected(lambda rows: rows[-1]['navigation'].update(invocation='new'))

    def test_symmetric_envelope_does_not_accept_large_negative_values(self):
        for value in [-.1,.1]:
            self.assert_rejected(lambda rows: rows[-1]['navigation']['motion'].__setitem__(2,value))


if __name__ == '__main__':
    unittest.main(verbosity=2)
