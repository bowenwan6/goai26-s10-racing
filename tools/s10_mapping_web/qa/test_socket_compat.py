"""Local-only persistent-socket deployment compatibility; no ROS/SSH or /run edits."""
import contextlib
import inspect
import io
import json
import os
import socket
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import field_worker
import robot_backend
import server as web
from field_core import DEFAULT_ROOT


class SocketDefaultTests(unittest.TestCase):
    def test_client_cli_defaults_unit_and_guide_agree(self):
        expected=DEFAULT_ROOT/'worker.sock'
        self.assertEqual(field_worker.SOCKET,expected)
        self.assertEqual(inspect.signature(field_worker.rpc_call).parameters['socket_path'].default,expected)
        self.assertIn("p.add_argument('--socket', type=Path, default=SOCKET)",inspect.getsource(field_worker))
        root=Path(field_worker.__file__).parent
        unit=(root/'s10-field-worker.service').read_text()
        self.assertNotIn('RuntimeDirectory=',unit)
        self.assertNotIn('RuntimeDirectoryMode=',unit)
        guide=(root/'FIELD_GUIDE_ZH.md').read_text()
        self.assertIn(str(expected),guide)
        self.assertNotIn('/run/s10-field/worker.sock',guide)
        self.assertLessEqual(len(os.fsencode(expected)),100)

    def test_forced_backend_and_agx_use_same_default_rpc_path(self):
        inner={'action':'health'};out=io.StringIO()
        with patch.object(field_worker,'rpc_call',return_value={'worker':True}) as rpc, \
             patch.object(robot_backend.sys,'stdin',io.StringIO(json.dumps({'action':'field','request':inner})+'\n')), \
             contextlib.redirect_stdout(out):
            robot_backend.main()
        rpc.assert_called_once_with(inner)
        self.assertTrue(json.loads(out.getvalue().split('S10_RESULT ',1)[1])['ok'])
        with patch.object(web,'field_transport',None),patch.object(web,'call',return_value={'worker':True}) as forwarded:
            web.field_call(inner)
        forwarded.assert_called_once_with('field',request=inner)


class PersistentSocketProcessTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory(prefix='s10-sock-',dir='/tmp')
        self.root=Path(self.tmp.name);self.data=self.root/'persist'
        self.data.mkdir(mode=0o700)
        self.socket_path=self.data/'worker.sock';self.process=None

    def tearDown(self):
        if self.process is not None:
            self.process.terminate()
            try:self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:self.process.kill();self.process.wait(timeout=3)
            self.process.stderr.close()
        self.tmp.cleanup()

    def spawn(self):
        return subprocess.Popen([sys.executable,'-B',str(Path(__file__).with_name('worker_fixture.py')),str(self.root),'--socket-under-root'],
                                stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)

    def start(self):
        self.process=self.spawn();deadline=time.monotonic()+5
        while time.monotonic()<deadline:
            if self.process.poll() is not None:self.fail(self.process.stderr.read().decode())
            try:
                result=field_worker.rpc_call({'action':'health'},self.socket_path)
                if result['demo']:return
            except (OSError,ValueError):time.sleep(.05)
        self.fail('local persistent socket did not become ready')

    def rejected_start(self):
        p=self.spawn()
        try:
            _,err=p.communicate(timeout=5)
            self.assertNotEqual(p.returncode,0,'worker must reject unsafe pre-existing socket path')
            return err.decode()
        except subprocess.TimeoutExpired:
            p.terminate();p.wait(timeout=3)
            self.fail('worker unexpectedly started or hung')
        finally:
            if p.stderr:p.stderr.close()

    def test_stale_socket_replaced_and_permissions_remain_private(self):
        with socket.socket(socket.AF_UNIX) as stale:stale.bind(str(self.socket_path))
        self.assertTrue(self.socket_path.is_socket())
        self.start()
        self.assertEqual(os.stat(self.socket_path).st_mode&0o777,0o600)
        self.assertEqual(os.stat(self.data).st_mode&0o777,0o700)

    def test_second_instance_does_not_unlink_live_socket(self):
        self.start();inode=self.socket_path.stat().st_ino
        self.rejected_start()
        self.assertEqual(self.socket_path.stat().st_ino,inode)
        self.assertTrue(field_worker.rpc_call({'action':'health'},self.socket_path)['worker'])

    def test_regular_file_preserved_and_start_rejected(self):
        self.socket_path.write_text('preserve this local fixture')
        self.rejected_start()
        self.assertEqual(self.socket_path.read_text(),'preserve this local fixture')

    def test_symlink_to_socket_preserved_and_start_rejected(self):
        target=self.root/'other.sock'
        with socket.socket(socket.AF_UNIX) as unrelated:
            unrelated.bind(str(target))
            self.socket_path.symlink_to(target)
            self.rejected_start()
            self.assertTrue(self.socket_path.is_symlink())
            self.assertTrue(target.is_socket())

    def test_dangling_symlink_preserved_and_start_rejected(self):
        target=self.root/'not-created'
        self.socket_path.symlink_to(target)
        self.rejected_start()
        self.assertTrue(self.socket_path.is_symlink())
        self.assertEqual(self.socket_path.readlink(),target)

    def test_nonprivate_parent_rejected_without_chmod_fallback(self):
        os.chmod(self.data,0o755)
        self.rejected_start()
        self.assertEqual(os.stat(self.data).st_mode&0o777,0o755)
        self.assertFalse(self.socket_path.exists())

    def test_configured_default_length_binds_in_local_fixture(self):
        length=len(os.fsencode(field_worker.SOCKET))
        filler=length-len(os.fsencode(self.root))-len('/worker.sock')-1
        self.assertGreater(filler,0)
        directory=self.root/('p'*filler);directory.mkdir(mode=0o700)
        candidate=directory/'worker.sock'
        self.assertEqual(len(os.fsencode(candidate)),length)
        with socket.socket(socket.AF_UNIX) as sock:sock.bind(str(candidate))
        self.assertTrue(candidate.is_socket())


if __name__=='__main__':unittest.main(verbosity=2)
