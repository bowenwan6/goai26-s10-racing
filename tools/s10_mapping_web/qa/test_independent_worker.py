"""Independent Engine contract tests, fake adapter only; no subprocess/ROS/SSH."""
import copy
import math
import sys
import tempfile
import time
import unittest
import uuid
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from field_core import Store, exclusive
from field_worker import Engine
from test_independent_core import healthy


class FakeEvidenceAdapter:
    demo = True

    def __init__(self):
        self.state = healthy(time.time())
        self.state.update(map_name='test_map', started_at=time.time()-60)
        self.state['aligned_cloud'] = dict(frame='map', points=[[1, 2, 3]], age=.01, stamp=time.time())
        self.profile = dict(verified=True, reference_frame='base_link', version='calibration-v1', evidence='test-only')
        self.effects = []

    def snapshot(self):
        s = copy.deepcopy(self.state)
        s['pose']['stamp'] = time.time()
        s['aligned_cloud']['stamp'] = time.time()
        return s

    def selfcheck(self, target):
        return dict(passed=True, target_map=target)

    def preview(self, target):
        return dict(map_identity=self.state['map_identity'], points=[[1, 2, 3]])

    def sample(self, duration, progress):
        now = time.time()
        samples = [self.snapshot() for _ in range(int(duration*10)+1)]
        for i, snap in enumerate(samples):
            snap['pose']['stamp'] = now+i*.1
        return samples

    def calibration(self):
        return copy.deepcopy(self.profile)

    def load_map(self, target, progress):
        self.effects.append('load_map'); self.state['map_name'] = target
        return dict(loaded=True)

    def record(self, root, kind, seconds, expected, progress):
        self.effects.append('record')
        return dict(passed=True, kind=kind, seconds=seconds, files=[])


class WorkerTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='s10-worker-independent-qa-')
        root = Path(self.tmp.name)
        self.store, self.adapter = Store(root/'persist'), FakeEvidenceAdapter()
        self.engine = Engine(self.store, self.adapter, root/'operation.lock')
        self.session_id = None
        self.perform('selfcheck', dict(target_map='test_map'))

    def tearDown(self):
        self.tmp.cleanup()

    def perform(self, action, params):
        request = dict(action=action, key=uuid.uuid4().hex, params=params)
        if self.session_id: request['session_id'] = self.session_id
        job = self.engine.rpc(dict(action='submit', request=request))
        self.session_id = job['session_id']
        return self.engine.execute(job['id'])

    def approve(self):
        checked = self.perform('localization_check', dict(stationary=True))
        self.assertEqual(checked['state'], 'SUCCEEDED')
        self.assertTrue(checked['result']['passed'])
        confirmed = self.perform('confirm_overlay', dict(check_id=checked['id'], confirmed=True))
        self.assertEqual(confirmed['state'], 'SUCCEEDED')
        return checked

    def test_no_confirmation_no_regular_recording(self):
        recorded = self.perform('record', dict(kind='stationary', seconds=10, remote_ready=True))
        self.assertEqual(recorded['state'], 'FAILED')
        self.assertEqual(self.adapter.effects, [])

    def test_wrong_map_boot_service_invalidates_approval(self):
        for key in ('map_identity', 'boot_id', 'invocation'):
            with self.subTest(key=key):
                self.approve()
                old = self.adapter.state[key]; self.adapter.state[key] = 'changed'
                recorded = self.perform('record', dict(kind='stationary', seconds=10, remote_ready=True))
                self.assertEqual(recorded['state'], 'FAILED')
                self.adapter.state[key] = old
        self.assertEqual(self.adapter.effects, [])

    def test_unknown_child_valid_numeric_samples_can_save_draft(self):
        self.adapter.state['pose']['child_frame'] = ''
        point = self.perform('waypoint', dict(name='draft', floor='1', stationary=True, draft=True))
        self.assertEqual(point['state'], 'SUCCEEDED')
        self.assertFalse(point['result']['navigation_valid'])
        self.assertTrue(point['result']['draft'])

    def test_empty_reference_profile_does_not_authorize_navigation(self):
        self.adapter.state['pose']['child_frame'] = ''
        self.adapter.profile['reference_frame'] = ''
        self.approve()
        point = self.perform('waypoint', dict(name='invalid', floor='1', stationary=True, draft=False))
        self.assertTrue(point['state'] == 'FAILED' or not point['result'].get('navigation_valid'))

    def test_nonfinite_live_cloud_age_cannot_be_confirmed(self):
        checked = self.perform('localization_check', dict(stationary=True))
        self.adapter.state['aligned_cloud']['age'] = math.nan
        confirmed = self.perform('confirm_overlay', dict(check_id=checked['id'], confirmed=True))
        self.assertEqual(confirmed['state'], 'FAILED')

    def test_live_cloud_source_error_cannot_be_confirmed(self):
        checked = self.perform('localization_check', dict(stationary=True))
        self.adapter.state['aligned_cloud']['error'] = 'source time discontinuity'
        confirmed = self.perform('confirm_overlay', dict(check_id=checked['id'], confirmed=True))
        self.assertEqual(confirmed['state'], 'FAILED')

    def test_changed_calibration_requires_new_review_before_navigation(self):
        self.approve()
        self.adapter.profile['version'] = 'calibration-v2'
        point = self.perform('waypoint', dict(name='invalid', floor='1', stationary=True, draft=False))
        self.assertTrue(point['state'] == 'FAILED' or not point['result'].get('navigation_valid'))

    def test_observed_localization_loss_requires_fresh_confirmation(self):
        self.approve()
        self.adapter.state['status']['code'] = 3
        failed = self.perform('record', dict(kind='stationary', seconds=10, remote_ready=True))
        self.assertEqual(failed['state'], 'FAILED')
        self.adapter.state['status']['code'] = 0
        recovered = self.perform('record', dict(kind='stationary', seconds=10, remote_ready=True))
        self.assertEqual(recovered['state'], 'FAILED')
        self.assertEqual(self.adapter.effects, [])

    def test_old_mapping_lock_blocks_new_reservation(self):
        with exclusive(self.engine.lock_path):
            with self.assertRaises(Exception):
                self.engine.rpc(dict(action='submit', request=dict(action='finish', key=uuid.uuid4().hex,
                                      session_id=self.session_id, params={})))

    def test_finish_exports_failure_evidence_even_when_ros_state_unavailable(self):
        def unavailable(): raise RuntimeError('simulated ROS disconnected')
        self.adapter.snapshot = unavailable
        finished = self.perform('finish', {})
        self.assertEqual(finished['state'], 'SUCCEEDED')
        self.assertFalse(finished['result']['passed'])
        self.assertTrue(finished['result']['reasons'])
        self.assertIn('field-report.json', [a['name'] for a in finished['artifacts']])


if __name__ == '__main__':
    unittest.main(verbosity=2)
