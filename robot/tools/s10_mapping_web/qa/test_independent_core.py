"""Independent failure-injection tests. Local temporary files only; no ROS/SSH."""
import copy
import math
import sys
import tempfile
import unittest
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from field_core import FieldError, Store, binding, localization_reasons, pose_summary


def healthy(stamp=100.0):
    return dict(robot_id='s10-48', boot_id='boot-one', map_identity='map-sha-one', invocation='loc-one',
                mapping_active=False, localization_active=True, started_at=90.0,
                status=dict(fresh=True, code=0, mode='全局'),
                pose=dict(frame='map', child_frame='base_link', xyz=[1., 2., 3.],
                          quaternion=[0., 0., 0., 1.], age=0.01, stamp_age_s=0.01, stamp=stamp))


class StoreTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='s10-independent-qa-')
        self.root = Path(self.tmp.name)
        self.store = Store(self.root / 'persist')

    def tearDown(self):
        self.tmp.cleanup()

    def request(self, key='independent-test-0001'):
        return dict(action='selfcheck', key=key, params=dict(target_map='test_map'))

    def test_duplicate_after_response_loss_is_same_job(self):
        first = self.store.submit(self.request())
        reopened = Store(self.store.root)
        self.assertEqual(reopened.submit(self.request())['id'], first['id'])
        self.assertEqual(len(reopened.jobs()), 1)

    def test_parallel_duplicate_is_single_job(self):
        with ThreadPoolExecutor(max_workers=8) as pool:
            results = list(pool.map(lambda _: self.store.submit(self.request()), range(12)))
        self.assertEqual(len({r['id'] for r in results}), 1)
        self.assertEqual(len(self.store.jobs()), 1)

    def test_same_key_other_payload_rejected(self):
        self.store.submit(self.request())
        changed = self.request(); changed['params']['target_map'] = 'other_map'
        with self.assertRaises(FieldError) as cm:
            self.store.submit(changed)
        self.assertEqual(cm.exception.code, 'conflict')

    def test_two_different_clients_excluded(self):
        self.store.submit(self.request())
        with self.assertRaises(FieldError) as cm:
            self.store.submit(self.request('independent-test-0002'))
        self.assertEqual(cm.exception.code, 'busy')

    def test_restart_interrupts_without_automatic_retry(self):
        job = self.store.submit(self.request())
        self.store.update(job['id'], state='RUNNING', stage='正在执行')
        reopened = Store(self.store.root); reopened.interrupt_pending()
        interrupted = reopened.job(job['id'])
        self.assertEqual(interrupted['state'], 'INTERRUPTED')
        self.assertEqual(reopened.submit(self.request())['state'], 'INTERRUPTED')

    def registered_file(self):
        job = self.store.submit(self.request())
        directory = self.store.root / 'jobs' / job['id']
        directory.mkdir(parents=True)
        file = directory / 'report.json'
        file.write_text('{"value":1}')
        return job, directory, file, self.store.artifact(job['id'], file)

    def test_artifact_direct_path_escape_rejected(self):
        job = self.store.submit(self.request())
        outside = self.root / 'outside.txt'; outside.write_text('private')
        with self.assertRaises(FieldError):
            self.store.artifact(job['id'], outside)

    def test_artifact_symlink_ancestor_escape_rejected(self):
        _, directory, file, aid = self.registered_file()
        outside = self.root / 'unrelated-private-directory'; outside.mkdir()
        (outside / file.name).write_text('{"value":9}')
        file.unlink(); directory.rmdir(); directory.symlink_to(outside, target_is_directory=True)
        with self.assertRaises((FieldError, ValueError, OSError)):
            self.store.artifact_info(aid)

    def test_artifact_same_size_mutation_rejected(self):
        _, _, file, aid = self.registered_file()
        file.write_text('{"value":9}')
        with self.assertRaises((FieldError, ValueError, OSError)):
            self.store.artifact_info(aid)


class PoseTests(unittest.TestCase):
    def test_valid_static_window(self):
        samples = [healthy(100.0 + i * .1) for i in range(31)]
        self.assertTrue(pose_summary(samples, 3.0, binding(samples[0]))['passed'])

    def test_stable_local_odometry_rejected(self):
        snap = healthy(); snap['status']['mode'] = '局部'
        self.assertTrue(localization_reasons(snap))

    def test_boolean_status_is_not_integer_zero(self):
        snap = healthy(); snap['status']['code'] = False
        self.assertTrue(localization_reasons(snap))

    def test_nonfinite_source_and_service_stamps_rejected(self):
        for target in ('stamp', 'started_at'):
            for invalid in (math.nan, math.inf, -math.inf):
                with self.subTest(target=target, value=invalid):
                    snap = healthy()
                    if target == 'stamp': snap['pose'][target] = invalid
                    else: snap[target] = invalid
                    self.assertTrue(localization_reasons(snap))

    def test_child_frame_change_does_not_average_different_reference_points(self):
        samples = [healthy(100.0 + i * .1) for i in range(31)]
        samples[15]['pose']['child_frame'] = 'lidar_link'
        self.assertFalse(pose_summary(samples, 3.0, binding(samples[0]))['passed'])

    def test_quaternion_wraparound_uses_short_average(self):
        samples = [healthy(100.0 + i * .1) for i in range(31)]
        for i, sample in enumerate(samples):
            a = math.radians(179 if i % 2 else -179)
            sample['pose']['quaternion'] = [0, 0, math.sin(a/2), math.cos(a/2)]
        result = pose_summary(samples, 3.0, binding(samples[0]))
        self.assertTrue(result['passed'])
        self.assertGreater(abs(result['yaw']), math.radians(178))

    def test_repeat_cached_pose_not_counted(self):
        snap = healthy()
        result = pose_summary([copy.deepcopy(snap) for _ in range(100)], 3.0, binding(snap))
        self.assertFalse(result['passed'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
