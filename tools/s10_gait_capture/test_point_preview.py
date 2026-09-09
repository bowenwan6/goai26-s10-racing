"""Run with python tools/s10_gait_capture/test_point_preview.py; no ROS needed."""
import struct
import tempfile
import time
from types import SimpleNamespace as NS

from server import Recorder, create_app, preview_points


def check():
    for endian in ('<', '>'):
        # Organized cloud with row padding, reordered fields and a nonfinite point.
        raw = b''.join(struct.pack(endian+'fff', *p)+b'PAD!' for p in
                       [(3, 1, 2), (6, 4, 5), (9, float('nan'), 8)])
        msg = NS(width=1, height=3, point_step=12, row_step=16, data=raw,
                 is_bigendian=endian=='>', fields=[NS(name=n, offset=o, datatype=7, count=1)
                 for n, o in [('x', 4), ('y', 8), ('z', 0)]],
                 header=NS(frame_id='base_link', stamp=NS(sec=1, nanosec=2)))
        assert preview_points(msg)['points'] == [[1, 2, 3], [4, 5, 6]]
        assert len(preview_points(msg, limit=1)['points']) <= 1
    with tempfile.TemporaryDirectory() as root:
        recorder = Recorder(root, demo=True)
        client = create_app(recorder, token='test').test_client()
        assert client.get('/api/points/front').status_code == 401
        client.post('/api/login', json={'token': 'test'})
        assert client.get('/api/points/front').json['fresh'] is False
        assert client.get('/api/points/unknown').status_code == 400
        recorder.point_frames['/rslidar_front/points'] = (msg, time.monotonic())
        assert client.get('/api/points/front').json['points'] == [[1, 2, 3], [4, 5, 6]]
        recorder.point_frames['/rslidar_front/points'] = (msg, time.monotonic()-3)
        assert client.get('/api/points/front').json['points'] == []
        msg.data = b''
        try:
            preview_points(msg)
        except ValueError:
            pass
        else:
            raise AssertionError('Truncated data accepted')
    print('Point preview checks passed')


if __name__ == '__main__':
    check()
