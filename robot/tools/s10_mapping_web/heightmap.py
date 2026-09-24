"""Read the vendor's GridMap for the phone preview; no publishers or service changes."""
import json
import math
import time


def decode_grid(msg):
    info = msg.info
    r, lx, ly = info.resolution, info.length_x, info.length_y
    p, q = info.pose.position, info.pose.orientation
    if not all(math.isfinite(v) for v in (r, lx, ly, p.x, p.y, p.z, q.x, q.y, q.z, q.w)) or min(r, lx, ly) <= 0:
        raise ValueError('高程图几何无效')
    if max(abs(q.x), abs(q.y), abs(q.z), abs(abs(q.w)-1)) > 1e-5:
        raise ValueError('不支持旋转的 GridMap')
    nx, ny = round(lx/r), round(ly/r)
    if min(nx, ny) < 1 or nx*ny > 250000 or abs(nx*r-lx) > 1e-5 or abs(ny*r-ly) > 1e-5:
        raise ValueError('高程图尺寸无效')
    sx, sy = msg.outer_start_index, msg.inner_start_index
    if not (0 <= sx < nx and 0 <= sy < ny) or len(msg.layers) != len(msg.data):
        raise ValueError('高程图缓冲区无效')
    arrays = {}
    for name in dict.fromkeys(['elevation', *msg.basic_layers]):
        a = msg.data[list(msg.layers).index(name)]
        dims = a.layout.dim
        # ponytail: the vendor publishes Eigen column-major arrays; reject other layouts.
        if len(dims) != 2 or [(d.label, d.size, d.stride) for d in dims] != [
                ('column_index', ny, nx*ny), ('row_index', nx, nx)]:
            raise ValueError('高程图矩阵布局不匹配')
        offset = a.layout.data_offset
        if offset < 0 or len(a.data) < offset+nx*ny:
            raise ValueError('高程图数据被截断')
        arrays[name] = a.data[offset:offset+nx*ny]
    heights = []
    for i in range(nx):
        for j in range(ny):
            k = (j+sy) % ny * nx + (i+sx) % nx
            value = arrays['elevation'][k]
            heights.append(round(float(value), 4) if all(math.isfinite(a[k]) for a in arrays.values()) else None)
    valid = [v for v in heights if v is not None]
    return dict(nx=nx, ny=ny, resolution=r, length_x=lx, length_y=ly,
                center=[p.x, p.y], heights=heights, valid=len(valid),
                low=min(valid) if valid else None, high=max(valid) if valid else None)


def stream():
    import rclpy
    from grid_map_msgs.msg import GridMap
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.serialization import deserialize_message
    from robot_backend import localization_status, measurement_time, service
    from std_msgs.msg import String

    rclpy.init()
    node = rclpy.create_node('s10_heightmap_preview', enable_rosout=False, start_parameter_services=False)
    latest, received, counts = {}, {}, {}

    def receive(key, raw, kind):
        now = time.monotonic()
        counts[key] = counts.get(key, 0)+1
        if now-received.get(key, -10) < 1:
            return
        received[key] = now
        try:
            msg = deserialize_message(raw, kind)
            if key == 'telemetry':
                latest[key] = dict(text=msg.data[:6000], received=now)
                return
            row = dict(received=now, frame=msg.header.frame_id,
                       **measurement_time(msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,
                                          now, time.time(), latest.get(key) if 'stamp' in latest.get(key, {}) else None))
            if key == 'grid':
                row.update(decode_grid(msg))
            else:
                p, q = msg.pose.pose.position, msg.pose.pose.orientation
                if not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
                    raise ValueError('定位数值无效')
                row.update(xyz=[p.x, p.y, p.z], yaw=math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
            latest[key] = row
        except Exception as exc:
            latest[key] = dict(received=now, error=str(exc))

    for key, topic, kind in [('grid', '/elevation_map_raw', GridMap), ('pose', '/ODOM', Odometry),
                              ('telemetry', '/traversability_estimator_log', String)]:
        node.create_subscription(kind, topic, lambda raw, k=key, t=kind: receive(k, raw, t),
                                 qos_profile_sensor_data, raw=True)
    tick, emitted, previous_counts = 0, time.monotonic(), {}
    try:
        while True:
            rclpy.spin_once(node, timeout_sec=.1)
            now = time.monotonic()
            if now-emitted < 1:
                continue
            if tick % 3 == 0:
                status = dict(service=service('traversability_estimation')['ActiveState'],
                              localization=localization_status())
            data = {k: dict(v, age=round(now-v['received'], 3),
                            stamp_age_s=round(time.time()-v['stamp'], 3) if 'stamp' in v else None)
                    for k, v in latest.items()}
            hz = (counts.get('grid', 0)-previous_counts.get('grid', 0))/(now-emitted)
            print('S10_RESULT '+json.dumps(dict(**status, streams=data, source_hz=round(hz, 1),
                                                board_time=time.time()), allow_nan=False), flush=True)
            tick += 1
            emitted, previous_counts = now, counts.copy()
    except (BrokenPipeError, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
