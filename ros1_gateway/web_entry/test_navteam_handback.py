"""Our additions to the adapter: stop must finish the hand-back when the robot was LEFT in a navigation gait / mode
by a run that is already gone (2026-09-21 15:17: MotionState 17, Gait 0x3003, Mode 0 -> every stop was refused)."""
import unittest

from test_asdu_navigation import AgentTest as _Base


class HandBack(_Base):
    def _wire(self):
        a, ev = self.agent, []
        a.external_active = lambda: False
        a.run_script = lambda name, *args: ev.append(name)
        def mode(value):
            ev.append('mode' + str(value)); self.robot.data['status']['Mode'] = value
        a.set_mode = mode
        original = self.robot.send
        def send(body, *args, **kwargs):
            items = body['PatrolDevice']['Items']
            if 'GaitParam' in items:
                ev.append('gait%#x' % items['GaitParam']); self.robot.data['status']['Gait'] = items['GaitParam']
            return original(body, *args, **kwargs)
        self.robot.send = send
        return a, ev

    def test_left_in_nav_stairs_gait_mode0(self):
        a, ev = self._wire()
        self.robot.data['status'].update(MotionState=17, Mode=0, Gait=0x3003, HES=0)
        a.stop()
        self.assertEqual(ev, ['gait0x1001']); self.assertEqual(a.phase, 'stopped'); self.assertTrue(a.return_verified)

    def test_left_in_mode1(self):
        a, ev = self._wire()
        self.robot.data['status'].update(MotionState=17, Mode=1, Gait=0x3002, HES=0)
        a.stop()
        self.assertEqual(ev, ['mode0', 'gait0x1001']); self.assertEqual(a.phase, 'stopped')

    def test_already_manual_sends_nothing(self):
        a, ev = self._wire()
        self.robot.data['status'].update(MotionState=17, Mode=0, Gait=0x1001, HES=0)
        a.stop()
        self.assertEqual(ev, []); self.assertEqual(a.phase, 'stopped')

    def test_hard_estop_sends_nothing(self):
        a, ev = self._wire()
        self.robot.data['status'].update(MotionState=17, Mode=0, Gait=0x3003, HES=1)
        a.wait = lambda *args, **kw: (_ for _ in ()).throw(ValueError('not verified'))
        with self.assertRaises(ValueError): a.stop()
        self.assertEqual(ev, [])


# only our cases
del _Base
if __name__ == '__main__':
    unittest.main()
