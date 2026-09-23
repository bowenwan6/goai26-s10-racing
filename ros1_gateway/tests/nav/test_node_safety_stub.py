import json
import math
import sys
import time
import types

pub_log = []
class Pub:
    def __init__(self, name, *a, **k): self.name = name
    def publish(self, m): pub_log.append((self.name, m))
rospy = types.ModuleType("rospy")
rospy.Publisher = Pub; rospy.Subscriber = lambda *a, **k: None; rospy.AnyMsg = object
rospy.loginfo = rospy.logwarn = lambda *a, **k: None
rospy.logwarn_throttle = rospy.logerr_throttle = lambda *a, **k: None
rospy.is_shutdown = lambda: False
class ROSException(Exception): pass
rospy.ROSException = ROSException
sys.modules["rospy"] = rospy
class Msg:
    def __init__(self, data=None): self.data = data
class Tw:
    def __init__(self):
        self.linear = types.SimpleNamespace(x=0, y=0, z=0); self.angular = types.SimpleNamespace(x=0, y=0, z=0)
for mod, names in {"geometry_msgs.msg": {"Twist": Tw}, "sensor_msgs.msg": {"PointCloud2": object, "Imu": object},
                   "std_msgs.msg": {"Bool": Msg, "Float32": Msg, "String": Msg}}.items():
    m = types.ModuleType(mod); m.__dict__.update(names); sys.modules[mod] = m
    sys.modules[mod.split(".")[0]] = types.ModuleType(mod.split(".")[0])
sys.path.insert(0, "nav")
import s10_rl_nav_ros1 as n

route = sys.argv[1]
def mk(shadow):
    a = types.SimpleNamespace(config="config/nav.yaml", route_dir=route, transform="", shadow=shadow, autostart=True)
    node = n.Node(a); node.args_route_dir = route; return node
def feed(node, x, y, yaw=0.0):
    with node.lock:
        t = node.now()
        if node.pose is not None and t - node.pose_t < 1.0:
            pass
    # emulate on_pose's jump logic through a fake Odometry path: call the inner code by building a message is heavy;
    # use the same maths via a tiny shim
    class P: pass
    hdr = {"type": "geometry_msgs/PoseStamped"}
    class M:
        _connection_header = hdr; _buff = b""
    class PS:
        def __init__(s): s.pose = types.SimpleNamespace(position=types.SimpleNamespace(x=x, y=y, z=0.27),
                          orientation=types.SimpleNamespace(x=0, y=0, z=math.sin(yaw/2), w=math.cos(yaw/2)))
        def deserialize(s, b): pass
    node.pose_type = "geometry_msgs/PoseStamped"; node.pose_cls = PS
    node.on_pose(M())
ok = True
def check(name, cond):
    global ok; ok &= bool(cond); print(("PASS " if cond else "FAIL ") + name)
wp0 = n.core.RouteV2.load(n.core.RouteBundle.from_dir(route).route_path).waypoints[0]
x0, y0 = float(wp0.xy[0]), float(wp0.xy[1])
# shadow: no cmd_vel, no gait request
node = mk(True); feed(node, x0, y0); pub_log.clear(); node.tick()
names = [p[0] for p in pub_log]
check("shadow publishes no /rl_nav/cmd_vel", "/rl_nav/cmd_vel" not in names)
check("shadow publishes no gait request", "/rl_nav/gait_request" not in names)
check("shadow publishes status", "/rl_nav/status" in names)
# armed: normal tick publishes cmd + gait
node = mk(False); feed(node, x0, y0); pub_log.clear(); node.tick()
names = [p[0] for p in pub_log]
check("normal tick publishes cmd_vel and gait", "/rl_nav/cmd_vel" in names and "/rl_nav/gait_request" in names)
# pose jump -> hold, zero, not running; start clears
time.sleep(0.05); feed(node, x0 + 2.0, y0); pub_log.clear(); node.tick()
st = json.loads([p for p in pub_log if p[0] == "/rl_nav/status"][-1][1].data)
tw = [p for p in pub_log if p[0] == "/rl_nav/cmd_vel"][-1][1]
check("pose jump pauses", node.running is False and "pose jump" in (st.get("reason") or ""))
check("pose jump commands zero", (tw.linear.x, tw.linear.y, tw.angular.z) == (0.0, 0.0, 0.0))
node.on_cmd(Msg("start")); check("start clears the jump", node.jump is None and node.running)
# tick exception -> zero twist, paused, node alive
node.core.step = lambda *a, **k: (_ for _ in ()).throw(RuntimeError("boom"))
time.sleep(0.05); feed(node, x0 + 2.0, y0); pub_log.clear(); node.tick()
tw = [p for p in pub_log if p[0] == "/rl_nav/cmd_vel"]
check("tick error -> zero twist and paused", tw and tw[-1][1].linear.x == 0.0 and node.running is False)
print("STUB_TEST", "PASS" if ok else "FAIL")
