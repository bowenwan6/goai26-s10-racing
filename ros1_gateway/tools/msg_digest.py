"""Per-message digests shared by the ROS 2 and ROS 1 dump tools (pure Python 3.8+).

Works on rclpy (ROS 2) and genpy (ROS 1) message objects: the field names of
PointCloud2, Imu and Odometry are identical except the header stamp.
"""
import hashlib
import struct

# PointField datatype -> struct code
_CODES = {1: 'b', 2: 'B', 3: 'h', 4: 'H', 5: 'i', 6: 'I', 7: 'f', 8: 'd'}


def stamp_ns(header):
    st = header.stamp
    if hasattr(st, 'nanosec'):
        return int(st.sec) * 1000000000 + int(st.nanosec)
    return int(st.secs) * 1000000000 + int(st.nsecs)


def _floats(values):
    return [float(v) for v in values]


def cloud(msg):
    data = bytes(msg.data)
    fields = [[f.name, int(f.offset), int(f.datatype), int(f.count)] for f in msg.fields]
    n = int(msg.width) * int(msg.height)
    endian = '>' if msg.is_bigendian else '<'
    samples = []
    if n:
        seed = stamp_ns(msg.header)
        idx = sorted({0, n // 4, n // 2, (3 * n) // 4, n - 1} | {(seed // 7919 * k) % n for k in range(1, 6)})
        for i in idx:
            row, col = divmod(i, int(msg.width))
            base = row * int(msg.row_step) + col * int(msg.point_step)
            point = {}
            for name, off, dt, _count in fields:
                code = _CODES.get(dt)
                if code:
                    point[name] = struct.unpack_from(endian + code, data, base + off)[0]
            samples.append([i, point])
    return dict(width=int(msg.width), height=int(msg.height), point_step=int(msg.point_step),
                row_step=int(msg.row_step), is_dense=bool(msg.is_dense), is_bigendian=bool(msg.is_bigendian),
                fields=fields, data_bytes=len(data), data_sha256=hashlib.sha256(data).hexdigest(),
                samples=samples)


def imu(msg):
    o, a, l = msg.orientation, msg.angular_velocity, msg.linear_acceleration
    return dict(orientation=_floats([o.x, o.y, o.z, o.w]), angular_velocity=_floats([a.x, a.y, a.z]),
                linear_acceleration=_floats([l.x, l.y, l.z]),
                orientation_covariance=_floats(msg.orientation_covariance),
                angular_velocity_covariance=_floats(msg.angular_velocity_covariance),
                linear_acceleration_covariance=_floats(msg.linear_acceleration_covariance))


def odom(msg):
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    tl, ta = msg.twist.twist.linear, msg.twist.twist.angular
    return dict(child_frame_id=msg.child_frame_id,
                pose=_floats([p.x, p.y, p.z, q.x, q.y, q.z, q.w]), pose_covariance=_floats(msg.pose.covariance),
                twist=_floats([tl.x, tl.y, tl.z, ta.x, ta.y, ta.z]), twist_covariance=_floats(msg.twist.covariance))


KINDS = {'PointCloud2': cloud, 'Imu': imu, 'Odometry': odom}


def digest(topic, type_name, msg, bag_time_ns=None, recv_time_ns=None):
    """type_name may be 'sensor_msgs/msg/PointCloud2' or 'sensor_msgs/PointCloud2'."""
    kind = type_name.rsplit('/', 1)[-1]
    row = dict(topic=topic, kind=kind, stamp_ns=stamp_ns(msg.header), frame_id=msg.header.frame_id)
    if bag_time_ns is not None:
        row['bag_time_ns'] = int(bag_time_ns)
    if recv_time_ns is not None:
        row['recv_time_ns'] = int(recv_time_ns)
    row.update(KINDS[kind](msg))
    return row
