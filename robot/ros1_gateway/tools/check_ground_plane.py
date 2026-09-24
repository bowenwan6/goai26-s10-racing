#!/usr/bin/env python3
"""Read a few /LIDAR/POINTS clouds (ROS 1) and fit the ground plane near the robot: reports the
plane height below the cloud origin and its tilt. On flat ground with the robot level, tilt
should be ~0 deg (cloud frame = body frame) and the height is the body reference height
(use it as body_z_offset if x_nav's pose is this same frame)."""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'nav'))
import nav_core as core
import rospy
from sensor_msgs.msg import PointCloud2


def main():
    rospy.init_node('check_ground_plane', anonymous=True)
    topic = sys.argv[1] if len(sys.argv) > 1 else '/LIDAR/POINTS'
    res = []
    for _ in range(5):
        m = rospy.wait_for_message(topic, PointCloud2, timeout=5)
        p = core.decode_pointcloud2(m)
        p = p[np.isfinite(p).all(axis=1)]
        r = np.linalg.norm(p[:, :2], axis=1)
        near = p[(r > 0.5) & (r < 2.5) & (p[:, 2] < 0.0)]
        if len(near) < 200:
            print('few ground points', len(near)); continue
        # robust: keep the lowest dense band, then least squares z = ax + by + c, 3 rounds
        q = near[near[:, 2] < np.percentile(near[:, 2], 60)]
        for _ in range(3):
            A = np.c_[q[:, 0], q[:, 1], np.ones(len(q))]
            coef, *_ = np.linalg.lstsq(A, q[:, 2], rcond=None)
            resid = q[:, 2] - A @ coef
            q = q[np.abs(resid) < max(0.02, 2 * resid.std())]
        a, b, c = coef
        res.append((math.degrees(math.atan(a)), math.degrees(math.atan(b)), -c, len(p), len(q), float(resid.std())))
        print(f'frame {m.header.frame_id!r} points {len(p)} ground {len(q)}: pitch-slope {res[-1][0]:+.2f} deg, roll-slope {res[-1][1]:+.2f} deg, ground {res[-1][2]:.3f} m below origin, rms {res[-1][5]*100:.1f} cm')
    if res:
        r = np.array(res)
        print(f'MEAN slope_x {r[:,0].mean():+.2f} deg slope_y {r[:,1].mean():+.2f} deg height {r[:,2].mean():.3f} m')

if __name__ == '__main__':
    main()
