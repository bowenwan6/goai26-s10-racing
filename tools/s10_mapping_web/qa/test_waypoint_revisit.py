"""Waypoint revisit semantics/persistence/failure injection. Fake evidence only."""
import copy
import json
import math
import unittest
import uuid

import test_independent_worker as fixture
from field_core import FieldError, Store
from field_worker import Engine


class RevisitTests(unittest.TestCase):
    def setUp(self):
        self.h=fixture.WorkerTests();self.h.setUp();self.h.approve()
        self.point=self.h.perform('waypoint',dict(name='WP0',floor='1F',draft=True,stationary=True,marker_note='胶带十字，同一机身标记，同一朝向站高'))
        self.assertEqual(self.point['state'],'SUCCEEDED')
    def tearDown(self):self.h.tearDown()
    def params(self):
        return dict(waypoint_id=self.point['id'],stationary=True,remote_ready=True,
                    physical_confirmed=True,reference_note='对准原十字，同一机身参考和站姿')
    def revisit(self,params=None):return self.h.perform('waypoint_revisit',params or self.params())
    def artifact(self,job,name):
        aid=next(a['id'] for a in job['artifacts'] if a['name']==name)
        return self.h.store.artifact_info(aid)[1]

    def test_offset_saved_separately_and_source_immutable(self):
        original=copy.deepcopy(self.h.store.job(self.point['id']))
        self.h.adapter.state['pose']['xyz']=[1.03,2.04,3.02]
        job=self.revisit();self.assertEqual(job['state'],'SUCCEEDED',job.get('error'))
        r=job['result'];self.assertAlmostEqual(r['metrics']['horizontal_m'],.05)
        self.assertAlmostEqual(r['metrics']['vertical_m'],.02)
        self.assertTrue(r['within_reference']);self.assertFalse(r['navigation_ready'])
        self.assertFalse(r['absolute_accuracy_verified']);self.assertIsNone(r['ruler_offset_cm'])
        self.assertEqual(original,self.h.store.job(self.point['id']))
        self.assertEqual(self.h.adapter.effects,[])
        self.assertTrue(json.loads(self.artifact(job,'waypoint-revisit.json').read_text())['demo'])
        self.assertIn('演示',self.artifact(job,'revisit-xy.svg').read_text())

    def test_outside_reference_is_saved_not_mislabeled_task_failure(self):
        self.h.adapter.state['pose']['xyz'][0]+=.35
        job=self.revisit();self.assertEqual(job['state'],'SUCCEEDED')
        self.assertFalse(job['result']['within_reference'])
        self.assertTrue(job['result']['measurement_valid'])

    def test_yaw_wrap_and_quaternion_sign(self):
        self.h.adapter.state['pose']['quaternion']=[0,0,math.sin(math.radians(179)/2),math.cos(math.radians(179)/2)]
        self.point=self.h.perform('waypoint',dict(name='turn',floor='1',draft=True,stationary=True))
        self.h.adapter.state['pose']['quaternion']=[0,0,math.sin(math.radians(-179)/2),math.cos(math.radians(-179)/2)]
        r=self.revisit()['result'];self.assertAlmostEqual(r['metrics']['yaw_deg'],2.)

    def test_human_confirmation_and_optional_ruler_validation(self):
        for key in ('physical_confirmed','stationary','remote_ready','waypoint_id'):
            p=self.params();p.pop(key)
            with self.subTest(missing=key),self.assertRaises(FieldError):self.revisit(p)
        for value in (True,-1,float('nan'),float('inf'),'5',1001):
            p=self.params();p['ruler_offset_cm']=value
            with self.subTest(value=value),self.assertRaises(FieldError):self.revisit(p)
        p=self.params();p['ruler_offset_cm']=0
        self.assertEqual(self.revisit(p)['result']['ruler_offset_cm'],0)

    def test_old_approval_and_wrong_map_do_not_allow_review(self):
        self.h.store.revoke_checks(self.h.session_id)
        self.assertEqual(self.revisit()['state'],'FAILED')
        self.h.approve();self.h.adapter.state['map_identity']='changed-map'
        self.assertEqual(self.revisit()['state'],'FAILED')

    def test_new_localization_run_requires_new_approval_but_keeps_old_reference(self):
        self.h.adapter.state['invocation']='new-localizer';self.h.adapter.state['boot_id']='new-boot'
        self.assertEqual(self.revisit()['state'],'FAILED')
        self.h.perform('selfcheck',{});self.h.approve()
        job=self.revisit();self.assertEqual(job['state'],'SUCCEEDED',job.get('error'))
        self.assertTrue(job['result']['cross_localization_run'])

    def test_calibration_or_child_frame_change_is_not_comparable(self):
        self.h.adapter.profile['version']='changed'
        self.h.perform('selfcheck',{});self.h.approve()
        self.assertEqual(self.revisit()['state'],'FAILED')
        self.h.adapter.profile['version']='calibration-v1';self.h.adapter.state['pose']['child_frame']='lidar'
        self.h.approve()
        self.assertEqual(self.revisit()['state'],'FAILED')

    def test_unknown_reference_stays_diagnostic_only(self):
        self.h.adapter.profile['verified']=False
        self.h.adapter.state['pose']['child_frame']=''
        self.h.approve()
        self.point=self.h.perform('waypoint',dict(name='unknown',floor='1',draft=True,stationary=True))
        job=self.revisit();self.assertEqual(job['state'],'SUCCEEDED',job.get('error'))
        self.assertFalse(job['result']['reference_verified'])
        self.assertTrue(any('尚未核实' in w for w in job['result']['warnings']))

    def test_source_is_resolved_by_id_not_name_or_other_session(self):
        _second=self.h.perform('waypoint',dict(name='WP0',floor='2',draft=True,stationary=True))
        job=self.revisit();self.assertEqual(job['result']['source_waypoint_id'],self.point['id'])
        other=self.h.store.submit(dict(action='selfcheck',key=uuid.uuid4().hex,params=dict(target_map='test_map')))
        self.h.engine.execute(other['id']);self.h.session_id=other['session_id'];self.h.approve()
        self.assertEqual(self.revisit()['state'],'FAILED')

    def test_motion_stale_pose_and_mid_window_loss_save_evidence_not_metrics(self):
        original=self.h.adapter.sample
        for kind in ('motion','stale','loss','calibration','frame','whole_window_frame','cached'):
            self.h.adapter.sample=original
            self.h.adapter.profile['version']='calibration-v1'
            self.h.approve()
            def sample(seconds,progress):
                rows=original(seconds,progress)
                if kind=='motion':
                    for i,s in enumerate(rows):s['pose']['xyz'][0]+=i*.01
                elif kind=='stale':rows[20]['pose']['age']=1
                elif kind=='loss':rows[20]['status']['code']=3
                elif kind=='calibration':self.h.adapter.profile['version']='changed'
                elif kind=='frame':rows[20]['pose']['child_frame']='changed'
                elif kind=='whole_window_frame':
                    for s in rows:s['pose']['child_frame']='changed'
                elif kind=='cached':
                    for s in rows:s['pose']['stamp']=rows[0]['pose']['stamp']
                return rows
            self.h.adapter.sample=sample
            with self.subTest(kind=kind):
                job=self.revisit();self.assertEqual(job['state'],'FAILED',job)
                self.assertIsNone(job['result']);self.assertTrue(self.artifact(job,'revisit-samples.json').is_file())
            self.h.adapter.profile['version']='calibration-v1'
        self.h.adapter.sample=original

    def test_retry_same_key_and_restore_export_multiple_trials(self):
        request=dict(action='waypoint_revisit',key=uuid.uuid4().hex,session_id=self.h.session_id,params=self.params())
        first=self.h.engine.rpc(dict(action='submit',request=request))
        self.assertEqual(self.h.store.active_job()['duration_seconds'],5)
        self.assertEqual(self.h.engine.rpc(dict(action='submit',request=request))['id'],first['id'])
        self.h.engine.execute(first['id']);self.revisit()
        self.assertEqual(self.h.engine.rpc(dict(action='submit',request=request))['id'],first['id'])
        restored=Engine(Store(self.h.store.root),self.h.adapter,self.h.engine.lock_path)
        overview=restored.overview(self.h.session_id)
        self.assertEqual(overview['revisit_count'],2);self.assertEqual(overview['saved_waypoint_count'],1)
        report=self.h.perform('finish',{})
        export=json.loads(self.artifact(report,'waypoint-revisits.json').read_text())
        self.assertEqual(len(export['trials']),2);self.assertFalse(export['absolute_accuracy_verified'])
        self.revisit();self.assertTrue(self.h.engine.overview(self.h.session_id)['report_outdated'])

    def test_svg_escapes_untrusted_labels(self):
        self.point=self.h.perform('waypoint',dict(name='<script>alert(1)</script>',floor='1',draft=True,stationary=True))
        svg=self.artifact(self.revisit(),'revisit-xy.svg').read_text()
        self.assertNotIn('<script>',svg);self.assertIn('&lt;script&gt;',svg)

if __name__=='__main__':unittest.main()
