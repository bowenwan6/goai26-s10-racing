"""Map-load gate tests. Vendor activation is always mocked; no ROS/network."""
import copy
import math
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import field_robot
from field_core import FieldError
from test_independent_core import healthy


class MockActivation(Exception):pass

def fixtures():
    snapshots=[]
    for i in range(51):
        s=healthy(100+i*.1)
        s.update(map_name='old-map',board_time=100+i*.1,
                 imu=dict(age=.02,stamp_age_s=.02,angular_velocity=[.004,-.003,.006]),
                 navigation=dict(idle=True,fresh=True,invocation='planner-one',stamp=100+i*.1,
                                 command=[0.,0.,0.],motion=[.003,-.018,.007]))
        s['pose']['xyz'][0]+=.001*math.sin(i)
        snapshots.append(s)
    return snapshots

def run_gate(snapshots):
    adapter=field_robot.RobotAdapter.__new__(field_robot.RobotAdapter)
    adapter.map_identity=lambda _: 'target-sha'
    adapter.snapshot=lambda: snapshots[0]
    def sample(seconds,progress):
        assert seconds==5
        return snapshots[1:]
    adapter.sample=sample
    with patch.object(field_robot.vendor,'run',side_effect=MockActivation) as called:
        try:adapter.load_map('target-map',lambda _:None)
        except MockActivation:return True
        except FieldError:
            called.assert_not_called();return False
        raise AssertionError('Unexpected return without mocked activation')

class MapStopToleranceTests(unittest.TestCase):
    def test_standing_noise_keeps_original_values(self):
        rows=fixtures();before=copy.deepcopy(rows)
        self.assertTrue(run_gate(rows));self.assertEqual(rows,before)

    def test_slow_translation_is_rejected_despite_small_feedback(self):
        for speed in [-.005,.005,-.02,.02]:
            rows=fixtures()
            for s in rows:s['pose']['xyz'][0]=1+speed*(s['board_time']-100)
            self.assertFalse(run_gate(rows))

    def test_rotation_in_window_and_endpoint_return_are_rejected(self):
        rows=fixtures()
        for i,s in enumerate(rows):
            a=math.radians(2)*math.sin(math.pi*i/50)
            s['pose']['quaternion']=[0.,0.,math.sin(a/2),math.cos(a/2)]
        self.assertFalse(run_gate(rows))

    def test_missing_frozen_invalid_and_stale_evidence_rejected(self):
        mutations=[lambda rows:rows[20]['imu'].update(age=.6),
                   lambda rows:rows[20]['imu'].update(angular_velocity=[math.nan,0.,0.]),
                   lambda rows:rows[20]['navigation'].update(idle=False),
                   lambda rows:rows[20]['navigation'].update(invocation='new-service'),
                   lambda rows:[s['pose'].update(stamp=100.) for s in rows],
                   lambda rows:rows[0]['pose'].update(quaternion=[0.,0.,0.,0.]),
                   lambda rows:rows[20]['pose'].update(child_frame='changed')]
        for mutate in mutations:
            with self.subTest(mutation=mutate):
                rows=fixtures();mutate(rows);self.assertFalse(run_gate(rows))

    def test_three_seconds_cannot_satisfy_five_second_gate(self):
        self.assertFalse(run_gate(fixtures()[:31]))

if __name__=='__main__':unittest.main()
