"""Implementation self-tests, separate from the independent reviewer's qa suite."""
import math
import struct
import tempfile
import time
import unittest
from pathlib import Path

from field_core import FieldError, Store, localization_reasons
from field_preview import read_pcd_preview
from field_robot import PREVIEW_INTERVALS_S, parse_navigation


class PreviewCadenceTests(unittest.TestCase):
    def test_cloud_preview_budget_keeps_gate_and_pose_imu_cadence(self):
        self.assertEqual(PREVIEW_INTERVALS_S, dict(pose=.1, imu=.1, cloud=.2, aligned_cloud=.2))
        self.assertLess(.13+PREVIEW_INTERVALS_S['cloud'], .5)
        # Increasing display rate must not turn stale source data into valid
        # evidence. The actual source-age gate still rejects 0.567 seconds.
        snap = dict(robot_id='test', boot_id='boot', map_identity='map-sha', invocation='inv',
                    mapping_active=False, localization_active=True, started_at=1.,
                    status=dict(fresh=True, code=0, mode='全局'),
                    pose=dict(frame='map', child_frame='', xyz=[0., 0., 0.], quaternion=[0., 0., 0., 1.],
                              stamp=2., age=.01, stamp_age_s=.567))
        self.assertTrue(any('过期' in reason for reason in localization_reasons(snap)))


class PreviewTests(unittest.TestCase):
    def test_binary_with_extra_field_and_nonfinite(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp)/'map.pcd'
            path.write_bytes(b'FIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nPOINTS 3\nDATA binary\n'+
                             struct.pack('<12f', 1, 2, 3, 10, 4, 5, 6, 20, math.nan, 0, 0, 0))
            self.assertEqual(read_pcd_preview(path), [[1, 2, 3], [4, 5, 6]])
            path.write_bytes(path.read_bytes()[:-1])
            with self.assertRaises(ValueError):
                read_pcd_preview(path)


class NavigationTests(unittest.TestCase):
    def block(self, when, goal='NO', cmd='0.000', motion='0.000', complete=True):
        stamp = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(when))+'.000.000'
        return (f'[{stamp}] [INFO ] [Basic]\n'
                '================ Planning Monitor ================\n'
                f'| Odom[OK] | Map[OK] | Goal[{goal}] | Cancel[NO] | Mode: ----\n'
                '| Planner Status : Code = 999 (Idle)\n'
                f'| Cmd Velocity : x_vel = {cmd} | y_vel = 0.000 | yaw_vel = 0.000\n'
                f'| Motion Vel : x_vel = {motion} | y_vel = 0.000 | yaw_vel = 0.000\n'+
                ('================ Planning Monitor ================\n' if complete else ''))

    def test_real_multiline_block(self):
        now = int(time.time())
        self.assertTrue(parse_navigation(self.block(now), now+.2, now-20, 'inv')['idle'])

    def test_latest_block_overrides_old_good(self):
        now = int(time.time())
        for kwargs in ({'goal': 'YES'}, {'cmd': '.1'}, {'motion': '.1'}, {'cmd': 'nan'}, {'complete': False}):
            text = self.block(now-1)+self.block(now, **kwargs)
            self.assertFalse(parse_navigation(text, now+.2, now-20, 'inv')['idle'], kwargs)

    def test_stale_future_old_invocation(self):
        now = int(time.time())
        for timestamp, started, invocation in ((now-10, now-20, 'inv'), (now+1, now-20, 'inv'), (now, now+1, 'inv'), (now, now-20, '')):
            self.assertFalse(parse_navigation(self.block(timestamp), now+.2, started, invocation)['idle'])


class RequestTests(unittest.TestCase):
    def test_no_arbitrary_actions_or_params(self):
        with tempfile.TemporaryDirectory() as temp:
            s = Store(temp)
            for req in ({'action': 'motion', 'key': 'a'*20, 'params': {}},
                        {'action': 'selfcheck', 'key': 'a'*20, 'params': {'target_map': '../../etc'}},
                        {'action': 'selfcheck', 'key': 'a'*20, 'params': {'target_map': 'a', 'shell': 'id'}}):
                with self.assertRaises(FieldError):
                    s.submit(req)


if __name__ == '__main__':
    unittest.main(verbosity=2)
