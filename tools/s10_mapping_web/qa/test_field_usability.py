"""Field workflow fix: diagnose first, never turn lost localization into approval."""
import copy
import math
import unittest
import test_independent_worker as fixture


class FieldUsabilityTests(unittest.TestCase):
    def setUp(self):self.h=fixture.WorkerTests();self.h.setUp()
    def tearDown(self):self.h.tearDown()

    def test_fresh_but_lost_can_inspect_and_save_failed_quality(self):
        self.h.adapter.state['status'].update(code=3,mode='局部',label='定位丢失')
        job=self.h.perform('localization_check',dict(stationary=True))
        self.assertEqual(job['state'],'SUCCEEDED')
        self.assertFalse(job['result']['passed']);self.assertTrue(job['result']['inspection_only'])
        self.assertIn('localization-samples.json',[a['name'] for a in job['artifacts']])
        self.assertTrue(any('Code3' in r for r in job['result']['reasons']))
        self.assertFalse(any('位姿接收或源时间过期' in r for r in job['result']['reasons']))
        self.assertIsNone(self.h.store.session(self.h.session_id).get('eligible_check'))
        confirmed=self.h.perform('confirm_overlay',dict(check_id=job['id'],confirmed=True))
        self.assertEqual(confirmed['state'],'FAILED');self.assertEqual(self.h.adapter.effects,[])

    def test_stale_or_missing_pose_can_be_diagnosed_without_success(self):
        for kind in ('stale','missing'):
            original=copy.deepcopy(self.h.adapter.state)
            if kind=='stale':self.h.adapter.state['pose']['age']=1
            else:self.h.adapter.state['pose']['xyz']=None
            job=self.h.perform('localization_check',dict(stationary=True))
            self.assertEqual(job['state'],'SUCCEEDED');self.assertFalse(job['result']['passed'])
            self.h.adapter.state=original

    def test_wrong_map_and_run_still_block_diagnostic_collection(self):
        for key in ('map_identity','boot_id','invocation','map_name'):
            original=self.h.adapter.state[key];self.h.adapter.state[key]='changed'
            job=self.h.perform('localization_check',dict(stationary=True))
            self.assertEqual(job['state'],'FAILED');self.h.adapter.state[key]=original

    def test_loss_immediately_after_window_cannot_publish_eligible_check(self):
        sample=self.h.adapter.sample
        def flip(*args):
            rows=sample(*args);self.h.adapter.state['status']['code']=3;return rows
        self.h.adapter.sample=flip
        job=self.h.perform('localization_check',dict(stationary=True))
        self.assertFalse(job['result']['passed'])
        self.assertIsNone(self.h.store.session(self.h.session_id).get('eligible_check'))

    def test_different_return_heading_and_approximate_point_can_be_saved(self):
        self.h.approve()
        p=self.h.perform('waypoint',dict(name='WP0',floor='1F',draft=True,stationary=True))
        original=copy.deepcopy(p)
        self.h.adapter.state['pose']['quaternion']=[0,0,math.sin(math.pi/4),math.cos(math.pi/4)]
        self.h.adapter.state['pose']['xyz'][0]+=.04
        job=self.h.perform('waypoint_revisit',dict(waypoint_id=p['id'],stationary=True,remote_ready=True,
                                                physical_confirmed=True,alignment_mode='nearby'))
        self.assertEqual(job['state'],'SUCCEEDED',job.get('error'))
        r=job['result'];self.assertAlmostEqual(r['metrics']['yaw_deg'],90)
        self.assertTrue(r['within_reference']);self.assertFalse(r['same_heading_required'])
        self.assertFalse(r['position_error_isolated']);self.assertFalse(r['exact_alignment_claimed'])
        self.assertEqual(self.h.store.job(p['id']),original)

    def test_same_stationary_quality_rules_for_waypoint_and_revisit(self):
        self.h.approve();sample=self.h.adapter.sample
        def jitter(seconds,progress):
            rows=sample(seconds,progress)
            for i,row in enumerate(rows):row['pose']['xyz'][0]+=.035 if i%2 else -.035
            return rows
        self.h.adapter.sample=jitter
        p=self.h.perform('waypoint',dict(name='WP0',floor='1F',draft=True,stationary=True))
        self.assertEqual(p['state'],'SUCCEEDED')
        job=self.h.perform('waypoint_revisit',dict(waypoint_id=p['id'],stationary=True,remote_ready=True,physical_confirmed=True))
        self.assertEqual(job['state'],'SUCCEEDED',job.get('error'))
        self.assertFalse(job['result']['sampling_precision_sufficient'])

if __name__=='__main__':unittest.main()
