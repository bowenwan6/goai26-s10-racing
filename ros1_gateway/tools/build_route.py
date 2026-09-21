#!/usr/bin/env python3
"""Generate a route with EXACTLY the arguments recorded in config/s10_params.yaml (section route_build.<name>):
teach_line -> teach_to_route (full + short) -> verify_route_clearance -> sim_full_route. Nothing is installed on the AGX.

  python3 tools/build_route.py v7_o            # run the four steps
  python3 tools/build_route.py v7_o --print    # only print the commands
"""
import os
import subprocess
import sys

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BASE = os.path.dirname(ROOT)                       # s10-real-readiness/: route_build paths are relative to it


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    name, only_print = sys.argv[1], "--print" in sys.argv
    rb = yaml.safe_load(open(os.path.join(ROOT, "config", "s10_params.yaml")))["route_build"].get(name)
    if rb is None:
        sys.exit("no route_build.%s in config/s10_params.yaml" % name)
    p = lambda rel: os.path.join(BASE, rel)
    tl, t2r = rb["teach_line"], rb["teach_to_route"]
    free = ",".join([v for v in [tl.get("free_at") or ""] if v] + ["%s:%s" % (w, tl["contact_radius"]) for w in tl.get("contact", [])])
    py = sys.executable
    cmds = [[py, "tools/teach_line.py", p(rb["session"]), "--map-dir", p(rb["map_dir"]), "--control-logs", p(rb["control_logs"]), "--out", p(rb["out"]),
             "--demos", ",".join(tl["demos"]), "--gait-demos", ",".join(tl["gait_demos"]), "--splice", ",".join(tl.get("splice", [])),
             "--drop", ",".join(tl.get("drop", [])), "--contact", ",".join(tl.get("contact", [])), "--contact-radius", str(tl["contact_radius"]),
             "--free-at", tl.get("free_at") or "", "--corridor", str(tl["corridor"]), "--corridor-stairs", str(tl["corridor_stairs"]),
             "--margin", str(tl["margin"]), "--clearance", str(tl["clearance"]), "--fillet", str(tl["fillet"]), "--reach", str(tl["reach"]),
             "--touch", str(tl["touch"]), "--body-touch", str(tl["body_touch"]), "--body-z-offset", str(tl["body_z_offset"]),
             "--terrain-zones", str(tl["terrain_zones"]), "--gait-plan", str(tl["gait_plan"]), "--fast-gait", str(tl["fast_gait"]),
             "--fast-turn-deg", str(tl["fast_turn_deg"]), "--fast-clearance", str(tl["fast_clearance"]), "--fast-min-run", str(tl["fast_min_run"]),
             "--strip-backward", ",".join(tl.get("strip_backward", [])), "--jump-lane", str(tl.get("jump_lane", 1)), "--jump-before", str(tl.get("jump_before", 2.0)),
             "--jump-after", str(tl.get("jump_after", 1.0)), "--jump-max-shift", str(tl.get("jump_max_shift", 1.5)), "--platform-absorb", str(tl.get("platform_absorb", 4.0)),
             "--platform-before", str(tl.get("platform_before", 0.8)), "--platform-after", str(tl.get("platform_after", 1.2))]]
    for kind, out in rb["routes"].items():
        c = [py, "tools/teach_to_route.py", os.path.join(p(rb["out"]), "derived_session"), "--map-id", rb["map_id"], "--out", p(out),
             "--tol-z", str(t2r["tol_z"]), "--step", str(t2r["step"]), "--flat-speed", str(t2r["flat_speed"]),
             "--stairs-speed", str(t2r["stairs_speed"]), "--body-z-offset", str(t2r["body_z_offset"])]
        if kind == "short":
            c += ["--last-wp", rb["short_last_wp"]]
        cmds.append(c)
        cmds.append([py, "tools/verify_route_clearance.py", p(out), p(rb["session"]), "--map-dir", p(rb["map_dir"]),
                     "--margin", str(rb["verify"]["margin"]), "--free-at", free])
        cmds.append([py, "tests/nav/sim_full_route.py", p(out), str(rb["simulate"]["speed"]), str(rb["simulate"]["stairs_speed"])])
    for c in cmds:
        print("$ " + " ".join(a if a and " " not in a else repr(a) for a in c), flush=True)
        if not only_print:
            r = subprocess.run(c, cwd=ROOT)
            if r.returncode != 0:
                sys.exit("STOPPED: %s exited with %d" % (os.path.basename(c[1]), r.returncode))
    if not only_print:
        print("ROUTE_BUILD_OK %s -> %s (install on the AGX as %s)" % (name, rb["routes"], rb.get("installed_as")))


if __name__ == "__main__":
    main()
