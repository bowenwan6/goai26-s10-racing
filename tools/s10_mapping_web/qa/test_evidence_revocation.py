"""Regression for re-confirming pre-loss evidence; local fake only, no robot."""
import unittest

import test_independent_worker as fixture


class EvidenceRevocationTests(unittest.TestCase):
    def setUp(self):
        self.h = fixture.WorkerTests()
        self.h.setUp()

    def tearDown(self):
        self.h.tearDown()

    def confirm(self, checked):
        return self.h.perform('confirm_overlay', dict(check_id=checked['id'], confirmed=True))

    def lose_and_recover(self):
        self.h.adapter.state['status']['code'] = 3
        self.h.engine.invalidate_on_health()
        self.h.adapter.state['status']['code'] = 0

    def test_loss_before_human_confirmation_invalidates_static_check(self):
        checked = self.h.perform('localization_check', dict(stationary=True))
        self.lose_and_recover()
        self.assertEqual(self.confirm(checked)['state'], 'FAILED')

    def test_loss_after_approval_cannot_reconfirm_old_check_or_record(self):
        checked = self.h.approve()
        self.lose_and_recover()
        self.assertIsNone(self.h.store.session(self.h.session_id)['eligible_check'])
        self.assertEqual(self.confirm(checked)['state'], 'FAILED')
        record = self.h.perform('record', dict(kind='stationary', seconds=10, remote_ready=True))
        self.assertEqual(record['state'], 'FAILED')
        self.assertEqual(self.h.adapter.effects, [])

    def test_new_static_check_and_confirmation_after_recovery_can_proceed(self):
        self.h.approve()
        self.lose_and_recover()
        self.h.approve()
        record = self.h.perform('record', dict(kind='stationary', seconds=10, remote_ready=True))
        self.assertEqual(record['state'], 'SUCCEEDED')
        self.assertEqual(self.h.adapter.effects, ['record'])

    def test_failed_new_static_result_cannot_fall_back_to_previous_pass(self):
        checked = self.h.approve()
        sample = self.h.adapter.sample
        def bad_sample(*args):
            rows = sample(*args)
            rows[-1]['status']['code'] = 3
            return rows
        self.h.adapter.sample = bad_sample
        failed = self.h.perform('localization_check', dict(stationary=True))
        self.assertFalse(failed['result']['passed'])
        self.assertEqual(self.confirm(checked)['state'], 'FAILED')

    def test_new_static_exception_also_revokes_previous_pass(self):
        checked = self.h.approve()
        def broken(*args):
            raise RuntimeError('simulated sampling failure')
        self.h.adapter.sample = broken
        self.assertEqual(self.h.perform('localization_check', dict(stationary=True))['state'], 'FAILED')
        self.assertEqual(self.confirm(checked)['state'], 'FAILED')

    def test_new_passing_check_supersedes_previous_check(self):
        old = self.h.approve()
        new = self.h.perform('localization_check', dict(stationary=True))
        self.assertEqual(self.confirm(old)['state'], 'FAILED')
        self.assertEqual(self.confirm(new)['state'], 'SUCCEEDED')

    def test_restart_revocation_prevents_reusing_old_check(self):
        checked = self.h.approve()
        # The worker startup calls exactly this persistent revocation operation.
        self.h.store.revoke_checks(self.h.session_id)
        self.assertEqual(self.confirm(checked)['state'], 'FAILED')

    def test_revocation_during_preview_cannot_race_with_confirmation_commit(self):
        checked = self.h.perform('localization_check', dict(stationary=True))
        preview = self.h.adapter.preview
        def revoke_during_io(target):
            self.h.store.revoke_checks(self.h.session_id)
            return preview(target)
        self.h.adapter.preview = revoke_during_io
        result = self.confirm(checked)
        self.assertEqual(result['state'], 'FAILED')
        self.assertIsNone(self.h.store.session(self.h.session_id)['approved_check'])

    def test_frontend_eligibility_list_is_revoked_after_loss(self):
        checked = self.h.approve()
        listed = self.h.engine.rpc(dict(action='list'))
        self.assertEqual(listed['eligible_checks'][self.h.session_id], checked['id'])
        self.lose_and_recover()
        self.assertIsNone(self.h.engine.rpc(dict(action='list'))['eligible_checks'][self.h.session_id])


if __name__ == '__main__':
    unittest.main(verbosity=2)
