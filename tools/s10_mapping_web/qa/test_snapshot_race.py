"""Independent snapshot timing regressions; all clocks/vendor/identity are fake.

No ROS initialization, filesystem identity reads, SSH, or robot connections.
"""
import copy
import hashlib
import sys
import threading
import unittest
from contextlib import ExitStack
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import field_robot
from field_core import localization_reasons


class Clock:
    def __init__(self):
        self.mono, self.wall = 100.0, 1000.0

    def monotonic(self):
        return self.mono

    def time(self):
        return self.wall

    def advance(self, seconds):
        self.mono += seconds
        self.wall += seconds


class SnapshotRaceTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.adapter = field_robot.RobotAdapter.__new__(field_robot.RobotAdapter)
        self.adapter.config = dict(robot_id='qa-only', expected_machine_id_sha256=hashlib.sha256(b'qa-machine').hexdigest())
        self.adapter.lock = threading.Lock()
        self.adapter.error = None
        self.adapter.status_cache = None
        self.adapter.status_at = self.clock.mono
        self.adapter.map_identity = Mock(return_value='qa-map-sha')
        self.loc = dict(active_map='qa-map', service='active', invocation='qa-invocation', started_at=900.0,
                        status=dict(fresh=True, stamp=999.9, code=0, mode='全局'))
        self.inject_messages()
        self.patches = ExitStack()
        self.addCleanup(self.patches.close)
        self.patches.enter_context(patch.object(field_robot, 'time', self.clock))
        self.patches.enter_context(patch.object(field_robot, 'Path', side_effect=self.identity_path))
        self.vendor_loc = self.patches.enter_context(patch.object(field_robot.vendor, 'localization_status', return_value=self.loc))
        self.vendor_service = self.patches.enter_context(patch.object(field_robot.vendor, 'service', return_value={'ActiveState': 'inactive'}))
        self.patches.enter_context(patch.object(field_robot, 'navigation_status', return_value={'fresh': True, 'idle': True}))

    @staticmethod
    def identity_path(name):
        if name == '/etc/machine-id':
            return SimpleNamespace(read_bytes=lambda: b'qa-machine\n')
        if name == '/proc/sys/kernel/random/boot_id':
            return SimpleNamespace(read_text=lambda: 'qa-boot\n')
        raise AssertionError('Unexpected filesystem access: ' + str(name))

    def inject_messages(self):
        base = dict(received=self.clock.mono-.01, stamp=self.clock.wall-.01, frame='map')
        self.adapter.latest = dict(
            pose=dict(base, child_frame='base_link', xyz=[1., 2., 3.], quaternion=[0., 0., 0., 1.]),
            imu=dict(base, angular_velocity=[0., 0., 0.]),
            cloud=dict(base, points=[[1., 2., 3.]]),
            aligned_cloud=dict(base, points=[[1., 2., 3.]]))

    def cache_status(self):
        self.adapter.status_cache = dict(loc=copy.deepcopy(self.loc), mapping='inactive', navigation={'fresh': True, 'idle': True})
        self.adapter.status_at = self.clock.mono

    def concurrent_delivery_during_io(self):
        # Callback delivers new messages after the caller began its snapshot.
        self.clock.advance(.3)
        self.inject_messages()
        self.clock.advance(.04)

    def assert_consistent_fresh_streams(self, snap):
        self.assertEqual(snap['monotonic'], self.clock.mono)
        self.assertEqual(snap['board_time'], self.clock.wall)
        for key in ('pose', 'imu', 'cloud', 'aligned_cloud'):
            with self.subTest(stream=key):
                self.assertAlmostEqual(snap[key]['age'], .05)
                self.assertAlmostEqual(snap[key]['stamp_age_s'], .05)
                self.assertGreaterEqual(snap[key]['age'], 0)
        self.assertEqual(localization_reasons(snap), [])

    def test_uncached_vendor_io_new_messages_use_post_io_clocks(self):
        def slow_vendor():
            self.concurrent_delivery_during_io()
            return self.loc
        self.vendor_loc.side_effect = slow_vendor
        snap = self.adapter.snapshot()
        self.vendor_loc.assert_called_once_with()
        self.assert_consistent_fresh_streams(snap)

    def test_cached_vendor_info_still_samples_after_slow_map_identity(self):
        self.cache_status()
        def slow_map(_):
            self.concurrent_delivery_during_io()
            return 'qa-map-sha'
        self.adapter.map_identity.side_effect = slow_map
        snap = self.adapter.snapshot()
        self.vendor_loc.assert_not_called()
        self.vendor_service.assert_not_called()
        self.assert_consistent_fresh_streams(snap)

    def test_clocks_are_sampled_after_data_lock_is_acquired(self):
        self.cache_status()
        deliver = self.concurrent_delivery_during_io
        class ContendedLock:
            def __enter__(self):
                deliver()
            def __exit__(self, *args):
                return False
        self.adapter.lock = ContendedLock()
        self.assert_consistent_fresh_streams(self.adapter.snapshot())

    def test_genuine_future_received_time_remains_negative_and_rejected(self):
        for cached in (False, True):
            with self.subTest(cached=cached):
                if cached:
                    self.cache_status()
                else:
                    self.adapter.status_cache = None
                self.inject_messages()
                self.adapter.latest['pose']['received'] = self.clock.mono+.4
                snap = self.adapter.snapshot()
                self.assertAlmostEqual(snap['pose']['age'], -.4)
                self.assertGreater(snap['pose']['stamp_age_s'], 0)
                self.assertTrue(localization_reasons(snap))

    def test_genuine_future_source_time_remains_negative_and_rejected(self):
        for cached in (False, True):
            with self.subTest(cached=cached):
                if cached:
                    self.cache_status()
                else:
                    self.adapter.status_cache = None
                self.inject_messages()
                self.adapter.latest['pose']['stamp'] = self.clock.wall+.4
                snap = self.adapter.snapshot()
                self.assertAlmostEqual(snap['pose']['stamp_age_s'], -.4)
                self.assertGreater(snap['pose']['age'], 0)
                self.assertTrue(localization_reasons(snap))

    def test_status_expires_during_io_using_same_board_clock(self):
        self.loc['status']['stamp'] = self.clock.wall-4
        def slow_vendor():
            self.clock.advance(2)
            self.inject_messages()
            return self.loc
        self.vendor_loc.side_effect = slow_vendor
        snap = self.adapter.snapshot()
        self.assertEqual(snap['board_time'], 1002.0)
        self.assertFalse(snap['status']['fresh'])
        self.assertAlmostEqual(snap['pose']['stamp_age_s'], .01)
        self.assertTrue(localization_reasons(snap))


if __name__ == '__main__':
    unittest.main(verbosity=2)
