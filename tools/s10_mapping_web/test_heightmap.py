"""python tools/s10_mapping_web/test_heightmap.py — no ROS or robot required."""
import json
from types import SimpleNamespace as NS
from heightmap import decode_grid


def check():
    nx, ny = 2, 3
    def array(values):
        return NS(layout=NS(dim=[NS(label='column_index', size=ny, stride=nx*ny),
                                 NS(label='row_index', size=nx, stride=nx)], data_offset=1), data=[999, *values])
    msg = NS(info=NS(resolution=.5, length_x=1, length_y=1.5,
                    pose=NS(position=NS(x=10, y=20, z=0), orientation=NS(x=0, y=0, z=0, w=1))),
             layers=['elevation', 'variance'], basic_layers=['elevation', 'variance'],
             data=[array([1, 4, 2, 5, 3, 6]), array([1]*6)], outer_start_index=0, inner_start_index=0)
    assert decode_grid(msg)['heights'] == [1, 2, 3, 4, 5, 6]
    # Non-square grid catches transposition; nonzero buffer origin catches rolling-map shifts.
    msg.outer_start_index, msg.inner_start_index = 1, 2
    assert decode_grid(msg)['heights'] == [6, 4, 5, 3, 1, 2]
    msg.data[1].data[-1] = float('nan')
    decoded = decode_grid(msg)
    assert decoded['heights'][0] is None and decoded['valid'] == 5
    assert decoded['center'] == [10, 20]
    json.dumps(decoded, allow_nan=False)
    msg.data[0].data.pop()
    try:
        decode_grid(msg)
    except ValueError:
        pass
    else:
        raise AssertionError('Truncated grid accepted')
    print('HEIGHTMAP_CHECK_OK')


if __name__ == '__main__':
    check()
