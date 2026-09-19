# 2026-09-17 app integration evidence

Deployed to the existing 048 app. No physical motion test was run.

Local checks:

- `python3.11 qa/test_native_nav.py`: 12 passed.
- `node qa/test_native_nav_ui.cjs`: all listed UI assertions passed at 320/390/1280px; no JS errors, no external resources, no real robot I/O in browser fixtures.
- `python3.11 test_server.py`: MAPPING_WEB_CHECK_OK.
- `node test_localization.cjs`: LOCALIZATION_VIEW_CHECK_OK.
- Real-readiness `PYTHONPATH=.:src/s10_auto_nav:src/s10_perception .venv/bin/python -m pytest -q tests_real/test_native_router.py tests_real/test_geometry.py tests_real/test_shadow.py`: 121 passed. Initial invocations missing PYTHONPATH packages failed collection/import; corrected invocation passed without source changes.

106 isolated wire test:

- `ROS_DOMAIN_ID=211`, `ROS_LOCALHOST_ONLY=1`, vendor DDS profile unset.
- `NATIVE_TEST_APP_CONTROL=1`, real WaypointFollowerNode and NativeGaitRouter.
- Ordinary navigation gait request 12290, 33 synthetic native velocity messages, expired app lease followed by zero outputs.
- `robot_actuated:false`; log in `app-lease-wire.log`.

Live read-only app backend:

- App `server.native_call` on 102 used the existing restricted SSH/forced-command channel to new 106 RPC.
- Read-only run `17773751123e40dfb9269631b16098e5`: completed, 148 samples; all observer, no native command publisher, no accepted gait, no published movement decision.
- Read-only run `d2dc9ba58b234b898fb560bffc1b3fc0`: repeat request returned same ID; distinct concurrent request rejected busy; cancel accepted and process ended; 24 samples, no native publisher.
- Snapshots in `live-readonly-status.json`, `live-cancel-status.json`.
- Current technical blockers: 2 competing NAV_CMD publishers; LocationStatus 3 plus fresh native local-mode log; native motion state 0; engineering config remains pending.
- Actual HTTP on 10.21.33.102:8080 and hotspot 10.21.41.1:8080: homepage/new unauthenticated page 200 with new entry/login; native status API 401 without login. Production account credentials were not extracted or changed. HTTP authentication was exercised locally; real backend transport was exercised separately.
- Original localization, planner, field worker remained active. Localization InvocationID remained `975155295050497a85ac86e9b09196f4`.

Deployment hashes are in `deployment-sha256.json`. New service starts idle, does not restart unfinished tests. Full Start/B route and official-policy physical response remain unaccepted; webpage checkboxes cannot overwrite engineering verification.
