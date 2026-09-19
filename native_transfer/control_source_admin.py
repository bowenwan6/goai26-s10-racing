"""Inspect native owners; optionally stop only the documented 103 handler unit.

This never publishes speed, gait, stand or emergency-release commands. Run on 106
with the production ROS environment. Restore/start is intentionally not automatic.
"""
import argparse
import json
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from drdds.msg import MotionInfo, NodeCtlCmd, StdMsgInt32
from drdds.srv import NodeCtlQuery
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stop-handler', action='store_true')
    args = parser.parse_args()
    rclpy.init()
    node = Node('goai_control_source_admin', enable_rosout=False, start_parameter_services=False)
    motion, hes, odom, cloud = [], [], [], []
    def sample_motion(m):
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        motion.append(dict(received=time.monotonic(), age=time.time()-stamp,
                           velocity=[m.data.vel_x,m.data.vel_y,m.data.vel_yaw],state=m.data.motion_state.state))
    node.create_subscription(MotionInfo, '/MOTION_INFO', sample_motion, qos_profile_sensor_data)
    node.create_subscription(StdMsgInt32, '/HES_STATUS', lambda m:hes.append((time.monotonic(),m.value)), qos_profile_sensor_data)
    node.create_subscription(Odometry, '/ODOM', lambda m:odom.append(time.monotonic()), qos_profile_sensor_data)
    node.create_subscription(PointCloud2, '/NAV_POINTS', lambda m:cloud.append((time.monotonic(),m.width*m.height)), qos_profile_sensor_data)
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
    def query(soc,module):
        c=node.create_client(NodeCtlQuery,f'/NODECTL_QUERY_{soc}')
        try:
            if not c.wait_for_service(timeout_sec=3):raise RuntimeError('native module query unavailable')
            f=c.call_async(NodeCtlQuery.Request(module_name=module))
            rclpy.spin_until_future_complete(node,f,timeout_sec=3)
            if not f.done() or f.exception():raise RuntimeError('native module query failed')
            m=f.result();return dict(state=m.state,pid=m.pid)
        finally:node.destroy_client(c)
    def snapshot():
        modules={f'{soc}/{module}':query(soc,module) for soc,module in [(103,'handler.service'),(103,'motion_master.service'),(103,'robot_server.service'),(106,'planner.service'),(106,'localization.service')]}
        return dict(wall_time=time.time(),modules=modules,
                    nav_publishers=[dict(name=e.node_name,gid=list(e.endpoint_gid)) for e in node.get_publishers_info_by_topic('/NAV_CMD')],
                    motion=motion[-1] if motion else None,
                    hes=hes[-1][1] if hes else None,
                    odom_age=None if not odom else time.monotonic()-odom[-1],
                    cloud_age=None if not cloud else time.monotonic()-cloud[-1][0],
                    cloud_points=None if not cloud else cloud[-1][1])
    try:
        spin(3)
        before=snapshot();print(json.dumps(dict(stage='before',**before)),flush=True)
        if args.stop_handler:
            recent=[m for m in motion if time.monotonic()-m['received']<2]
            if len(recent)<10 or time.monotonic()-motion[-1]['received']>.35:
                raise RuntimeError('fresh stationary feedback unavailable')
            if any(not -.03<=m['age']<=.35 or any(not math.isfinite(v) or abs(v)>.03 for v in m['velocity']) for m in recent):
                raise RuntimeError('robot is not confirmed stationary')
            if not hes or time.monotonic()-hes[-1][0]>.35:
                raise RuntimeError('emergency-state feedback unavailable')
            if before['modules']['103/handler.service']['state'] != 0:
                raise RuntimeError('handler is not active; no command sent')
            if any(before['modules'][k]['state']!=0 for k in ['103/motion_master.service','103/robot_server.service','106/localization.service']):
                raise RuntimeError('native control/localization not healthy; no command sent')
            pub=node.create_publisher(NodeCtlCmd,'/NODECTL_CMD_103',10)
            spin(1)
            if node.count_subscribers('/NODECTL_CMD_103')!=1:
                raise RuntimeError('ambiguous native module-control receiver')
            pub.publish(NodeCtlCmd(action='stop',module_name='handler.service'))
            spin(4)
            # Vendor DDS endpoints can outlive process shutdown in discovery caches.
            # Wait for withdrawal; never resend a stop merely because the cache lags.
            until=time.monotonic()+25
            expected=len(before['nav_publishers'])-1
            while node.count_publishers('/NAV_CMD')!=expected and time.monotonic()<until:
                spin(.5)
            after=snapshot();print(json.dumps(dict(stage='after_handler_stop',**after)),flush=True)
            if after['modules']['103/handler.service']['state']!=4:
                raise RuntimeError('handler stop not acknowledged; do not repeat blindly')
            if len(after['nav_publishers'])!=len(before['nav_publishers'])-1:
                raise RuntimeError('unexpected publisher change after stopping handler')
        print(json.dumps(dict(completed=True,motion_commands_sent=False,handler_stop_requested=args.stop_handler)),flush=True)
    finally:
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
