import math
from pathlib import Path
import struct
import sys
import tempfile
import threading
import time
from types import SimpleNamespace as NS
sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'tools/s10_gait_capture'))
from server import Recorder, create_app, POINT_TOPICS
with tempfile.TemporaryDirectory(prefix='s10-overlay-check-') as root:
    recorder = Recorder(root, demo=True)
    recorder.start_worker()
    def publish():
        while not recorder.quit.wait(.1):
            for side, sign in [('front', 1), ('rear', -1)]:
                points = [(sign*(2.4+(i%10)*.025), (i//10)/30-2, .3+math.sin(i)*.2) for i in range(1200)]
                points += [(sign*(.8+(i%100)/50), -1.5+(i//100)*.02, .1) for i in range(600)]
                msg = NS(width=len(points),height=1,point_step=12,row_step=len(points)*12,
                    data=b''.join(struct.pack('<fff',*p) for p in points),is_bigendian=False,
                    fields=[NS(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate(('x','y','z'))],
                    header=NS(frame_id='base_link',stamp=NS(sec=1,nanosec=0)))
                with recorder.lock: recorder.point_frames[POINT_TOPICS[side]]=(msg,time.monotonic())
    threading.Thread(target=publish,daemon=True).start()
    try: create_app(recorder,token='preview-test').run(host='127.0.0.1',port=8097,use_reloader=False)
    finally: recorder.quit.set()
