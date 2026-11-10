# v7_o route checks before field runs (2026-09-21 to 09-22)

[中文](README.zh.md)

Every version of the v7_o outdoor route that we built from teaching recordings on 21 and 22 September, with the checks each one went through before it was installed on the robot, and the data needed to run those checks again. Times are Beijing time (UTC+8).

## Three things to know first

1. The route is not merged into the map. The v7_o map stays as x_nav built it. Teaching recordings are compiled into a separate route bundle (`route_v2.json` and friends) in the map's frame.
2. The simulation here is kinematic, not MuJoCo. [`sim_full_route.py`](../../../../robot/ros1_gateway/tests/nav/sim_full_route.py) drives the real navigation core along the whole route, emulates gait switches the way the controller does them (still for more than a second, then switch) and feeds it flat-ground perception. It checks route geometry, waypoint order, gait zones, follower logic and timing. It does not model terrain, contact or falls. The MuJoCo runs in the paper use the v3 map and a different reference route; these taught routes never ran in MuJoCo.
3. The clearance check only arrived on 21 September at about 08:48. Before that, routes had only been through the kinematic simulation. The new check failed all three routes then installed on the AGX: the first straightened route had 107 violations and the route built straight from the raw teaching had 29 violating cells. All three were withdrawn. From then on every version had to pass both checks before it went onto the AGX.

## The checks

The procedure is written down in section 2.3 of [`PIPELINE_AND_PLANNING_ZH.md`](../../../../robot/ros1_gateway/docs/PIPELINE_AND_PLANNING_ZH.md). Later on [`build_route.py`](../../../../robot/ros1_gateway/tools/build_route.py) chained the steps and stopped at the first failure without writing a route.

1. [`audit_teach_session.py`](../../../../robot/ros1_gateway/tools/audit_teach_session.py) reports which teaching drives are complete and whether the robot fell.
2. [`teach_line.py`](../../../../robot/ros1_gateway/tools/teach_line.py) splices the drives into one line, straightens it, sweeps it with the body outline itself, and exits if it cannot repair it.
3. [`teach_to_route.py`](../../../../robot/ros1_gateway/tools/teach_to_route.py) compiles the route bundle: `route_v2.json`, `maneuvers.json`, `terrain.json`, `report.json`.
4. [`verify_route_clearance.py`](../../../../robot/ros1_gateway/tools/verify_route_clearance.py) is the hard gate. It sweeps a 0.9 × 0.5 m body along the route over the map; the body sitting on an obstacle that the operator's body never went over is a violation (HARD). The result goes into `clearance.json` in the route directory, and the navigation node refuses a route with violations.
5. `sim_full_route.py` runs the kinematic whole-route simulation and has to print `SIM_FULL_ROUTE_OK`.

s5 also passed the jump-lane (`test_jump_lane.py`) and zone-entry (`test_zone_entry.py`) tests on the Mac.

## Results by version

| Version | Clearance check | Kinematic simulation (flat/stairs speed, m/s) | What the checks stopped | Field run |
|---|---|---|---|---|
| Raw taught line `v7_o-teachline` | 8 violations between 08:49 and 09:08 on 09-21 while the line was being repaired; passes from 09:07 once the area round WP06 is ignored (2.5 m, confirmed by the operator) | 0.8/0.5: 528, 505, 490, 500 s, 29/29 | 8 failures from 01:56 to 02:24. The first four stopped 18 to 24 m into the route (between WP3 and WP5) with "centreline blocked, detour not allowed here". Passed at 02:25 after the fix | not recorded separately |
| s1 | passed 12:12 | 1.0/0.7: 413, 404, 411 s | none | not recorded separately |
| s2 | passed 14:21 | 1.0/0.7: 445 s; 393 to 443 s at 1.4 in the end | at 1.4 m/s, three stops at 28/29, 268 m into the route, in detour state; revision `20260921c` stopped once at 24/29 | not recorded separately |
| s3 | passed 15:40 | 1.0, 1.4, 1.67: 420, 373, 356 s; fastest 297 s | none | the 16:53 run on 09-21 |
| s4 | passed 21:30 | 1.67/1.67: 304, 317 s | first run at 20:41 stopped at 24/29, 239 m into the route, in detour state | 23:52 on 09-21: whole course, 29/29, 386 s |
| s5 | 3 violations from 01:08 to 01:13 on 09-22, passed at 01:15 after the fix | 1.67/1.67: 317 s | a thin obstacle between WP21 and WP22 lay under the line: the generator had joined separate survey pieces into one track and skipped it. The clearance check caught it | never ran on the robot |

The record holds 13 failed kinematic simulations, 11 clearance violations, and 3 stops of `build_route.py` because of them. Each of s1 to s5 passed both checks before it was installed on the AGX.

`free_at` in `clearance.json` lists map obstacles that the operator confirmed could be ignored. The raw taught line ignores 2.5 m round WP06, where the map has points 0.2 to 1.4 m high, most likely left by someone standing next to the robot during the survey. s1 to s3 ignore 1.5 m round WP17 and WP24. s4 and s5 ignore nothing.

## Rerun on 2026-11-10

The simulation results were only ever printed to the terminal. On 10 November we ran all 12 route bundles again with the current code and parameters, 15 runs in all. Every run passed. The output is in `rerun_20261110/`.

| Route | Speed | Time |
|---|---|---|
| Raw taught line | 0.8/0.5 | 398 s |
| s1, s2 | 1.0/0.7 | 371 s |
| s3 | 1.0/0.7, 1.67/1.67 | 369 s, 308 s |
| s4 | 1.0/0.7, 1.67/1.67 | 380 s, 315 s |
| s5 | 1.0/0.7, 1.67/1.67 | 380 s, 317 s |
| short routes | as above | 10 to 18 s |

s5 at 1.67 m/s takes 317 s, the same as in September. The earlier versions differ from their September numbers because the navigation core and parameters kept changing until 22 September; this rerun uses the final version.

We also reran the clearance check on s5. It matches the archived `clearance.json`.

## Contents

```
routes/            12 route bundles: route_v2.json, maneuvers.json, terrain.json,
                   report.json (compiler report), clearance.json and clearance.png
teach_lines/       the straightened line for each version: teach_line.json (body sweep,
                   gate radii, gait plan, parameters), figures, and derived_session/
                   (the input to teach_to_route)
audit/             teaching audit
raw/session_0921/  the teaching trails and waypoint marks used by s1 to s5
raw/logs_all/      control event logs, where the gait switches come from
history/           the September results, recovered from the development session logs
  check_events_20260921-22.csv       one row per check: time, type, route, speed, sim time,
                                     waypoints reached, result, stop reason
  check_output_lines_20260921-22.md  the raw output lines
rerun_20261110/    output of the November rerun; summary.csv collects it
```

Paths inside the JSON files start with `<s10-real-readiness>/`, the field workspace on the team laptop. The parameters and their approval record for every version are in [`s10_params.yaml`](../../../../robot/ros1_gateway/config/s10_params.yaml) and [`s10_params.lock`](../../../../robot/ros1_gateway/config/s10_params.lock). The v7_o map is not in the repository (228 MB); the team shares it separately.

## Reproducing

Run from `robot/ros1_gateway/`. The navigation core lives in `src/s10_auto_nav`, hence the `PYTHONPATH`. Both commands were tried on 10 November with the code on `main`, and s5 matched the archive.

```bash
PKG=../../artifacts/evidence/runs/v7o-route-checks-20260921-22
```

Kinematic simulation, no map needed, about a minute:

```bash
PYTHONPATH=../../src/s10_auto_nav python3 tests/nav/sim_full_route.py $PKG/routes/v7_o-s5-full 1.67 1.67
```

Clearance check. It rewrites `clearance.json` in the route directory, so run it on a copy. `MAP` is the v7_o map directory:

```bash
cp -R $PKG/routes/v7_o-s5-full /tmp/ && PYTHONPATH=../../src/s10_auto_nav python3 tools/verify_route_clearance.py /tmp/v7_o-s5-full $PKG/raw/session_0921 --map-dir $MAP
```

The raw taught line and s1 to s3 need the same `--free-at` values as in their `clearance.json`.

## Limits

- `history/` was recovered from session logs, not saved at the time. Checks run by hand on the AGX are not in those logs, so a few results may be missing.
- The kinematic simulation sees flat ground. Passing it means the route and the follower logic work together; it says nothing about whether the robot stays upright on the stairs.
- The three routes installed between the evening of 20 September and the morning of 21 September never passed the clearance check. They were found to hit obstacles and withdrawn.
