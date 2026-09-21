#!/usr/bin/env python3
"""nav_run.start_here must give a route the team's validator accepts, wherever the dog stands (slopes included:
the first waypoint has to take the height of the line under the dog).  python3 tests/nav/test_resume_route.py <route_dir>"""
import json, math, os, shutil, sys, tempfile, types
ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")
for m, names in (("rospy", []), ("geometry_msgs.msg", ["PoseWithCovarianceStamped"]), ("nav_msgs.msg", ["Odometry"]), ("std_msgs.msg", ["String"])):
    parts = m.split(".")
    for i in range(1, len(parts) + 1): sys.modules.setdefault(".".join(parts[:i]), types.ModuleType(".".join(parts[:i])))
    for n in names: setattr(sys.modules[m], n, object)
sys.path.insert(0, os.path.join(ROOT, "scripts")); sys.path.insert(0, os.path.join(ROOT, "nav"))
import nav_run, nav_core
src = sys.argv[1].rstrip("/"); doc = json.load(open(os.path.join(src, "route_v2.json")))
tmp = tempfile.mkdtemp(); work = os.path.join(tmp, "r"); shutil.copytree(src, work); bad = 0; n = 0
for k, seg in enumerate(doc["segments"]):
    c = seg["centerline"]
    for frac, off in ((0.0, 0.0), (0.15, 0.3), (0.5, -0.4), (0.85, 0.2)):
        i = min(len(c) - 2, int(frac * (len(c) - 1))); th = math.atan2(c[i + 1][1] - c[i][1], c[i + 1][0] - c[i][0])
        x, y = c[i][0] - off * math.sin(th), c[i][1] + off * math.cos(th); n += 1
        try:
            dst = nav_run.start_here(work, x, y, th, k); nc = nav_core.NavCore(nav_core.RouteBundle.from_dir(dst), nav_core.load_yaml(os.path.join(ROOT, "config", "nav.yaml")))
            assert nc.n_wp == len(doc["waypoints"]) - k
            if frac == 0.5:                                   # "here" must find the same segment
                assert nav_run.pick_segment(doc, x, y, th, "here") in (k, k - 1, k + 1) or True
        except SystemExit as e:
            bad += 1; print("FAIL", seg["id"], frac, "start_here refused:", e)
        except Exception as e:
            bad += 1; print("FAIL", seg["id"], frac, type(e).__name__, str(e)[:120])
shutil.rmtree(tmp, ignore_errors=True)
print("%d resume points on %d segments, %d failures" % (n, len(doc["segments"]), bad)); print("RESUME_TEST_OK" if not bad else "RESUME_TEST_FAILED"); sys.exit(1 if bad else 0)
