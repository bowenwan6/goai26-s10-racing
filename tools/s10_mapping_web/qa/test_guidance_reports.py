"""Operator guidance, full-session export and diagnostics; fake adapters only."""
import json
import unittest
import uuid

import test_independent_worker as fixture
from field_core import FieldError
from field_robot import parse_navigation
import test_independent_robot as robot_fixture


class GuidanceReportTests(unittest.TestCase):
    def setUp(self):
        self.h = fixture.WorkerTests()
        self.h.setUp()

    def tearDown(self):
        self.h.tearDown()

    def point(self, name='WP01'):
        return self.h.perform('waypoint', dict(name=name, floor='1F', draft=True, stationary=True))

    def artifact(self, job, name):
        aid = next(a['id'] for a in job['artifacts'] if a['name'] == name)
        _, path = self.h.store.artifact_info(aid)
        return json.loads(path.read_text())

    def test_report_keeps_early_drafts_beyond_100_jobs(self):
        for n in range(3):
            self.assertEqual(self.point('WP%02d' % (n+1))['state'], 'SUCCEEDED')
        for _ in range(103):
            self.h.perform('selfcheck', {})
        self.assertEqual(len(self.h.store.jobs(self.h.session_id)), 100)
        finished = self.h.perform('finish', {})
        report = self.artifact(finished, 'field-report.json')
        review = self.artifact(finished, 'waypoints-review.json')
        self.assertEqual(len(report['jobs']), 108)
        self.assertEqual(report['saved_waypoint_count'], 3)
        self.assertEqual(report['draft_waypoint_count'], 3)
        self.assertEqual([p['name'] for p in review['draft_waypoints']], ['WP01', 'WP02', 'WP03'])
        self.assertEqual(review['waypoints'], [])
        self.assertFalse(review['navigation_ready'])
        overview = self.h.engine.overview(self.h.session_id)
        self.assertEqual(len(overview['saved_points']), 3)
        self.assertIsNotNone(overview['selfcheck'])

    def test_finish_does_not_close_session_and_new_point_marks_old_report_outdated(self):
        self.point()
        old = self.h.perform('finish', {})
        old_data = self.artifact(old, 'field-report.json')
        self.assertFalse(self.h.engine.overview(self.h.session_id)['report_outdated'])
        self.point('WP02')
        self.assertTrue(self.h.engine.overview(self.h.session_id)['report_outdated'])
        self.assertEqual(self.artifact(old, 'field-report.json'), old_data)
        new = self.h.perform('finish', {})
        self.assertEqual(new['result']['saved_waypoint_count'], 2)
        self.assertFalse(self.h.engine.overview(self.h.session_id)['report_outdated'])

    def test_failed_waypoint_is_not_counted_and_same_name_is_preserved(self):
        self.point()
        self.point()
        self.h.adapter.state['map_name'] = 'wrong-map'
        self.assertEqual(self.point('WP02')['state'], 'FAILED')
        summary = self.h.engine.overview(self.h.session_id)
        self.assertEqual(summary['saved_waypoint_count'], 2)
        self.assertEqual(summary['duplicate_names'], ['WP01'])

    def test_filtered_session_still_sees_global_active_job(self):
        other = self.h.store.submit(dict(action='selfcheck', key=uuid.uuid4().hex, params=dict(target_map='other')))
        listed = self.h.engine.rpc(dict(action='list', session_id=self.h.session_id))
        self.assertEqual(listed['active_job']['id'], other['id'])
        self.assertNotIn(other['id'], [j['id'] for j in listed['jobs']])
        self.assertEqual(len(listed['sessions']), 2)

    def test_active_job_exposes_requested_duration_without_request_payload(self):
        for action, params, seconds in [
            ('localization_check', dict(stationary=True), 30),
            ('waypoint', dict(name='WP01', floor='1F', draft=True, stationary=True), 3),
            ('record', dict(kind='stationary', seconds=10, remote_ready=True), 10),
        ]:
            with self.subTest(action=action):
                job = self.h.store.submit(dict(action=action, key=uuid.uuid4().hex,
                                               session_id=self.h.session_id, params=params))
                active = self.h.store.active_job()
                self.assertEqual(active['id'], job['id'])
                self.assertEqual(active['duration_seconds'], seconds)
                self.assertNotIn('request', active)
                self.h.engine.execute(job['id'])

    def test_failed_gate_persists_diagnostics_without_executing_motion(self):
        def blocked(*args):
            raise FieldError('停稳反馈未归零', details=dict(gate='navigation_idle', vendor_activation_called=False))
        self.h.adapter.load_map = blocked
        job = self.h.perform('load_map', dict(stationary=True, remote_ready=True))
        self.assertEqual(job['state'], 'FAILED')
        self.assertFalse(self.artifact(job, 'failure-diagnostics.json')['vendor_activation_called'])
        self.assertEqual(self.h.adapter.effects, [])

    def test_dashboard_is_read_only_and_includes_current_session(self):
        before = len(self.h.store.jobs())
        result = self.h.engine.rpc(dict(action='dashboard', session_id=self.h.session_id))
        self.assertEqual(result['health']['ui_contract'], 2)
        self.assertEqual(result['listing']['session']['id'], self.h.session_id)
        self.assertEqual(result['live']['map_name'], 'test_map')
        self.assertEqual(len(self.h.store.jobs()), before)
        self.assertEqual(self.h.adapter.effects, [])

    def test_dashboard_can_export_history_without_live_ros(self):
        def down():
            raise RuntimeError('ROS unavailable')
        self.h.adapter.snapshot = down
        result = self.h.engine.rpc(dict(action='dashboard', session_id=self.h.session_id))
        self.assertIsNone(result['live'])
        self.assertIn('ROS unavailable', result['live_error'])
        self.assertEqual(result['listing']['overview']['total_jobs'], 1)


class DetailedNavigationTests(unittest.TestCase):
    def test_tolerated_feedback_preserves_values_and_requires_stop_window(self):
        text = robot_fixture.NavigationParserTests().block(motion='.018')
        result = parse_navigation(text, 100.1, 90., 'planner-one')
        self.assertTrue(result['idle'])
        self.assertEqual(result['command'], [0, 0, 0])
        self.assertEqual(result['motion'], [.018, 0, 0])
        self.assertIn('这不等于已停稳', result['reason'])
        self.assertEqual(result['zero_limit'], .0005)
        self.assertEqual(result['feedback_limit'], .02)
        self.assertEqual(result['stop_sample_seconds'], 5)

    def test_nonfinite_velocity_has_no_nonfinite_json_payload(self):
        result = parse_navigation(robot_fixture.NavigationParserTests().block(motion='nan'), 100.1, 90., 'p')
        self.assertFalse(result['idle'])
        self.assertIsNone(result['motion'])
        json.dumps(result, allow_nan=False)


if __name__ == '__main__':
    unittest.main(verbosity=2)
