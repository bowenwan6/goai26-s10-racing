"""Pure Python adapter gates with mocked vendor commands; no robot access."""
import copy
import math
from pathlib import Path
import sys
import time
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from field_core import FieldError
import field_robot
from test_independent_core import healthy


class RobotGateTests(unittest.TestCase):
    def test_map_activation_rejects_invalid_imu_before_vendor_command(self):
        baseline = dict(mapping_active=False, map_name='old-map', invocation='old-invocation',
                        imu=dict(age=.01, stamp_age_s=.01, angular_velocity=[0., 0., 0.]))
        invalids = [dict(age=math.nan), dict(stamp_age_s=math.nan), dict(stamp_age_s=-10.),
                    dict(angular_velocity=[math.nan, 0., 0.]), dict(angular_velocity=[]),
                    dict(error='source time discontinuity')]
        for invalid in invalids:
            with self.subTest(invalid=invalid):
                before = copy.deepcopy(baseline); before['imu'].update(invalid)
                adapter = field_robot.RobotAdapter.__new__(field_robot.RobotAdapter)
                adapter.map_identity = lambda _: 'target-sha'
                adapter.snapshot = lambda: before
                with patch.object(field_robot.vendor, 'run', side_effect=AssertionError('vendor command must not run')) as called:
                    with self.assertRaises(FieldError):
                        adapter.load_map('target-map', lambda _: None)
                    called.assert_not_called()

    def test_zero_gyro_does_not_allow_translating_robot_map_switch(self):
        before = healthy(100.)
        before.update(map_name='old-map', board_time=100., imu=dict(age=.01, stamp_age_s=.01, angular_velocity=[0.,0.,0.]),
                      navigation=dict(idle=True, fresh=True, invocation='planner-one', stamp=100.))
        samples=[]
        for i in range(31):
            snap=copy.deepcopy(before);snap['pose']['stamp']=100.+i*.1
            snap['pose']['xyz'][0]+=i*.01;snap['board_time']=100.+i*.1;snap['navigation']['stamp']=snap['board_time'];samples.append(snap)
        adapter=field_robot.RobotAdapter.__new__(field_robot.RobotAdapter)
        adapter.map_identity=lambda _: 'target-sha';adapter.snapshot=lambda:before;adapter.sample=lambda *args:samples
        with patch.object(field_robot.vendor,'run',side_effect=AssertionError('moving robot must not trigger vendor command')) as called:
            with self.assertRaises(FieldError):adapter.load_map('target-map',lambda _:None)
            called.assert_not_called()

    def test_still_pose_with_unknown_navigation_does_not_switch_map(self):
        before=healthy(100.)
        before.update(map_name='old-map', board_time=100., imu=dict(age=.01,stamp_age_s=.01,angular_velocity=[0.,0.,0.]),
                      navigation=dict(idle=False,fresh=False,reason='unknown'))
        samples=[]
        for i in range(31):
            snap=copy.deepcopy(before);snap['pose']['stamp']=100.+i*.1;snap['board_time']=100.+i*.1;samples.append(snap)
        adapter=field_robot.RobotAdapter.__new__(field_robot.RobotAdapter)
        adapter.map_identity=lambda _:'target-sha';adapter.snapshot=lambda:before;adapter.sample=lambda *args:samples
        with patch.object(field_robot.vendor,'run',side_effect=AssertionError('unknown navigation must not trigger vendor command')) as called:
            with self.assertRaises(FieldError):adapter.load_map('target-map',lambda _:None)
            called.assert_not_called()

    def test_positive_still_robot_on_old_local_map_reaches_only_mock_vendor_activation(self):
        class ReachedMockVendor(Exception): pass
        before=healthy(100.)
        before['status'].update(code=3,mode='局部')
        before['pose']['child_frame']=''
        before.update(map_name='old-map',board_time=100.,imu=dict(age=.01,stamp_age_s=.01,angular_velocity=[0.,0.,0.]),
                      navigation=dict(idle=True,fresh=True,invocation='planner-one',stamp=100.))
        samples=[]
        for i in range(31):
            snap=copy.deepcopy(before);snap['pose']['stamp']=100.+i*.1;snap['board_time']=100.+i*.1
            snap['navigation']['stamp']=snap['board_time'];samples.append(snap)
        adapter=field_robot.RobotAdapter.__new__(field_robot.RobotAdapter)
        adapter.map_identity=lambda _:'target-sha';adapter.snapshot=lambda:before;adapter.sample=lambda *args:samples
        with patch.object(field_robot.vendor,'run',side_effect=ReachedMockVendor) as called:
            with self.assertRaises(ReachedMockVendor):adapter.load_map('target-map',lambda _:None)
            called.assert_called_once_with(['drmap','--format','json','map','activate','target-map'],timeout=120)


class NavigationParserTests(unittest.TestCase):
    def block(self, stamp=100., goal='NO', code=999, cmd='0.000', motion='0.000', ending=True):
        date=time.strftime('%Y-%m-%d %H:%M:%S',time.localtime(stamp))
        return (f'[{date}.000.000] [INFO] [Basic]\n'
                '================ Planning Monitor ================\n'
                f'| Odom[OK] | Map[OK] | Goal[{goal}] | Cancel[NO]\n'
                f'| Planner Status : Code = {code} (Idle/Reached Goal)\n'
                f'| Cmd Velocity : x_vel = {cmd} | y_vel = 0.000 | yaw_vel = 0.000\n'
                f'| Motion Vel : x_vel = {motion} | y_vel = 0.000 | yaw_vel = 0.000\n'+
                ('================ Planning Monitor ================\n' if ending else ''))

    def test_complete_same_block_idle_is_evidence(self):
        self.assertTrue(field_robot.parse_navigation(self.block(),100.1,90.,'planner-one')['idle'])

    def test_latest_incomplete_or_active_block_not_replaced_by_old_good(self):
        variants=[dict(ending=False),dict(goal='YES'),dict(code=1),dict(cmd='.1'),dict(motion='.1'),dict(cmd='nan'),dict(motion='inf')]
        for variant in variants:
            with self.subTest(variant=variant):
                self.assertFalse(field_robot.parse_navigation(self.block(99)+self.block(**variant),100.1,90.,'planner-one')['idle'])

    def test_velocity_lines_from_two_blocks_cannot_be_joined(self):
        first=self.block(99).replace('| Motion Vel : x_vel = 0.000 | y_vel = 0.000 | yaw_vel = 0.000\n','')
        second=self.block().replace('| Cmd Velocity : x_vel = 0.000 | y_vel = 0.000 | yaw_vel = 0.000\n','')
        self.assertFalse(field_robot.parse_navigation(first+second,100.1,90.,'planner-one')['idle'])

    def test_old_future_pre_invocation_or_unknown_times_are_not_idle(self):
        for now,started,invocation in [(104.,90.,'p'),(99.,90.,'p'),(100.1,101.,'p'),(100.1,90.,''),(math.nan,90.,'p'),(100.,math.nan,'p')]:
            with self.subTest(now=now,started=started,invocation=invocation):
                self.assertFalse(field_robot.parse_navigation(self.block(),now,started,invocation)['idle'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
