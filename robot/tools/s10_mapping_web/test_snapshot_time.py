"""Snapshot clock regression: delayed vendor reads concurrent with new ROS data."""
import hashlib
import threading
import unittest
from pathlib import Path
from unittest.mock import patch

import field_robot


class SnapshotTimeTests(unittest.TestCase):
    def snapshot(self, future_received=False):
        adapter = field_robot.RobotAdapter.__new__(field_robot.RobotAdapter)
        adapter.config = dict(robot_id='test-106', expected_machine_id_sha256=hashlib.sha256(b'test-machine').hexdigest())
        adapter.lock = threading.Lock()
        adapter.latest = {}
        adapter.status_cache, adapter.status_at, adapter.error = None, 0, None
        adapter.map_identity = lambda _: 'map-sha'

        def slow_vendor_query():
            # Equivalent to a callback arriving after snapshot's initial 100.0
            # scheduling clock, while the vendor/log RPC is still outstanding.
            with adapter.lock:
                for i, key in enumerate(('pose', 'imu', 'cloud')):
                    adapter.latest[key] = dict(received=100.1+i*.01+(1 if future_received else 0),
                                               stamp=1000.1+i*.01, frame='map')
            return dict(active_map='map-one', invocation='inv', started_at=999., service='active',
                        status=dict(fresh=True, code=0, mode='全局', stamp=1000.15))

        with patch.object(field_robot.time, 'monotonic', side_effect=[100., 100.2]), \
             patch.object(field_robot.time, 'time', return_value=1000.2) as wall, \
             patch.object(field_robot.vendor, 'localization_status', side_effect=slow_vendor_query), \
             patch.object(field_robot.vendor, 'service', return_value=dict(ActiveState='inactive')), \
             patch.object(field_robot, 'navigation_status', return_value=dict(fresh=False, idle=False)), \
             patch.object(Path, 'read_bytes', return_value=b'test-machine\n'), \
             patch.object(Path, 'read_text', return_value='test-boot'):
            result = adapter.snapshot()
            self.assertEqual(wall.call_count, 1, 'one captured wall time must cover all streams and snapshot metadata')
        self.assertEqual(adapter.status_at, 100., 'status refresh scheduling stays separate from data capture')
        return result

    def test_fresh_data_arriving_during_vendor_io_is_not_negative_age(self):
        result = self.snapshot()
        self.assertEqual(result['monotonic'], 100.2)
        self.assertEqual(result['board_time'], 1000.2)
        self.assertTrue(result['identity_verified'])
        for i, key in enumerate(('pose', 'imu', 'cloud')):
            self.assertAlmostEqual(result[key]['age'], .1-i*.01)
            self.assertAlmostEqual(result[key]['stamp_age_s'], .1-i*.01)
            self.assertGreater(result[key]['age'], 0)

    def test_real_future_received_timestamp_is_not_clamped(self):
        result = self.snapshot(future_received=True)
        for key in ('pose', 'imu', 'cloud'):
            self.assertLess(result[key]['age'], -.8, 'future evidence remains invalid, never forced to zero')


if __name__ == '__main__':
    unittest.main(verbosity=2)
