"""Explicit LOCAL SYNTHETIC UI harness. Cannot access ROS or robot SSH."""
import hashlib
import json
import math
import sys
import tempfile
import threading
import time
from http.server import ThreadingHTTPServer
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import server as web
from imu_diag import Diagnostic


def main():
    tmp=tempfile.TemporaryDirectory(prefix='s10-imu-synthetic-')
    d=Diagnostic(Path(tmp.name),reserve=0)
    web.config=dict(login_hash=hashlib.sha256(b'demo:imu-local-test').hexdigest())
    web.imu_thread=object()
    web.field_transport=lambda r:d.rpc(r['request'])
    def forbidden():raise RuntimeError('LOCAL DEMO: no SSH or robot allowed')
    web.connect=forbidden
    def feed():
        while True:
            t=time.monotonic();d.rpc(dict(action='live'))
            d.ingest(dict(topic='imu',received=t,received_wall=time.time(),stamp=time.time(),frame='SYNTHETIC_NOT_ROBOT',values=[.002+math.sin(t*10)*.001,math.cos(t*8)*.001,.003+math.sin(t*9)*.002,0,0,9.81],quaternion=[0,0,0,1],orientation_covariance=[0]*9))
            d.ingest(dict(topic='body_motion',received=t,stamp=time.time(),motion_state=17,values=[.01,-.017,.001*math.sin(t*9)]))
            if int(t*200)%20==0:
                d.ingest(dict(topic='motion',received=t,received_wall=time.time(),stamp=time.time(),values=[.001,.002,.003,0,0,0],navigation=dict(fresh=True,goal_none=True,planner_code=999,command=[0,0,0],invocation='SYNTHETIC')))
                row=d.rpc(dict(action='live'));row['error']='本机演示：全部为合成数据，不是机器人读数或校准证据。'
                with web.guard:web.imu_state=dict(row,online=True,received=t)
            time.sleep(.005)
    threading.Thread(target=feed,daemon=True).start()
    http=ThreadingHTTPServer(('127.0.0.1',0),web.Handler)
    print(json.dumps(dict(url=f'http://127.0.0.1:{http.server_port}/imu-check',username='demo',password='imu-local-test',synthetic=True)),flush=True)
    try:http.serve_forever()
    finally:d.shutdown.set();http.server_close();tmp.cleanup()

if __name__=='__main__':main()
