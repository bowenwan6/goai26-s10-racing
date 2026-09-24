"""Local guardian processes and mocked bag auditing; never starts ROS."""
import json
import os
import signal
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from field_recorder import supervise
from field_robot import validate_bag


class GuardianTests(unittest.TestCase):
    def exercise(self, disconnected=False, over_size=False):
        signals = {s: signal.getsignal(s) for s in (signal.SIGINT, signal.SIGTERM)}
        read_fd, write_fd = os.pipe()
        try:
            with tempfile.TemporaryDirectory(prefix='s10-guardian-qa-') as temp:
                root = Path(temp)
                if over_size: (root/'existing-partial').write_bytes(b'x'*33)
                if disconnected: os.close(write_fd); write_fd = None
                child = [sys.executable, '-c', 'import signal,time,sys; signal.signal(signal.SIGINT,lambda *a:sys.exit(0)); time.sleep(60)']
                code = supervise(child, root, 10, read_fd, max_bytes=32 if over_size else 1024*1024, reserve=0)
                report = json.loads((root/'recorder-result.json').read_text())
                self.assertIsNotNone(report['returncode'])
                self.assertLess(report['elapsed_s'], 13)
                return code, report
        finally:
            os.close(read_fd)
            if write_fd is not None: os.close(write_fd)
            for s, handler in signals.items(): signal.signal(s, handler)

    def test_worker_pipe_closure_stops_local_recorder(self):
        code, report = self.exercise(disconnected=True)
        self.assertEqual(code, 1)
        self.assertEqual(report['reason'], 'worker_disconnected')

    def test_total_folder_bytes_limit_stops_local_recorder(self):
        code, report = self.exercise(over_size=True)
        self.assertEqual(code, 1)
        self.assertEqual(report['reason'], 'size_limit')

    def test_real_ten_second_deadline_stops_local_recorder(self):
        code, report = self.exercise()
        self.assertEqual(code, 0)
        self.assertEqual(report['reason'], 'duration_complete')
        self.assertGreaterEqual(report['elapsed_s'], 10)


class BagAuditTests(unittest.TestCase):
    def audit(self, missing=False, corrupt=False, wrong_type=False, zero=False, backwards=False):
        class Reader:
            def __init__(self):
                stamps = [] if zero else [100+i*.1 for i in range(101)]
                if backwards: stamps[50] = stamps[49]-.2
                self.rows = iter([('/ODOM', NS(header=NS(frame_id='map', stamp=NS(sec=int(t), nanosec=round((t-int(t))*1e9)))), round(t*1e9)) for t in stamps])
                self.next_row = next(self.rows, None)
            def open(self, *args): pass
            def close(self): pass
            def get_all_topics_and_types(self):
                return [NS(name='/ODOM', type='wrong/msg/Type' if wrong_type else 'nav_msgs/msg/Odometry')]
            def has_next(self): return self.next_row is not None
            def read_next(self):
                result = self.next_row; self.next_row = next(self.rows, None); return result
        class Info:
            def read_metadata(self, *args):
                if corrupt: raise ValueError('corrupt metadata')
                return NS()
        modules = {'rosbag2_py':NS(SequentialReader=Reader, Info=Info,
                     StorageOptions=lambda **kw:NS(**kw), ConverterOptions=lambda *args:NS()),
                   'rclpy':NS(), 'rclpy.serialization':NS(deserialize_message=lambda raw, cls:raw),
                   'rosidl_runtime_py':NS(), 'rosidl_runtime_py.utilities':NS(get_message=lambda name:object)}
        with tempfile.TemporaryDirectory(prefix='s10-bag-audit-qa-') as temp, patch.dict(sys.modules, modules):
            if not missing: (Path(temp)/'metadata.yaml').write_text('mock metadata for native parser stub')
            return validate_bag(Path(temp), [dict(name='/ODOM', type='nav_msgs/msg/Odometry', required=True)], 10)

    def test_valid_sample_bag_can_pass(self): self.assertTrue(self.audit()['passed'])
    def test_missing_metadata_rejected(self): self.assertFalse(self.audit(missing=True)['passed'])
    def test_corrupt_metadata_rejected(self): self.assertFalse(self.audit(corrupt=True)['passed'])
    def test_wrong_actual_topic_type_rejected(self): self.assertFalse(self.audit(wrong_type=True)['passed'])
    def test_required_zero_messages_rejected(self): self.assertFalse(self.audit(zero=True)['passed'])
    def test_source_time_backwards_rejected(self): self.assertFalse(self.audit(backwards=True)['passed'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
