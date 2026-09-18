# Successful WP0→WP32 run — Mac seed 8

Archived: 2026-08-20 23:19:29 Asia/Shanghai  
Code: `main` at `3660b81e8244dfa238633c161673596fe91650f6`  
Platform: Mac arm64, clean image `s10-racing:submission-3660b81-clean`, Docker volume
`s10-submission-mac-3660b81-215224`, ROS domain 203

## Verified result

- Waypoints 0 through 32 passed in strict order (`waypoint_pass_count=33`).
- Official WP32 `sim_time`: **392.258 s**.
- Official stopped-course `elapsed`: **392.257 s**.
- Recorder wall time: **674.01 s**; this is not the course score.
- WP28: 345.309 s; WP29: 353.676 s; official WP28→WP29 time: **8.367 s**.
- Final goal distance: 0.177 m; distance travelled: 249.94 m; maximum recorded tilt: 44.6°;
  stalls: 2.
- No skipped gate, fall, timeout, or router abort occurred.
- Actual joint owner at recorder completion: `official`. The strict 0.18 m recorder stopped the
  launch immediately, before a later router `done` status could be latched.

Gate16 logs show stable-fallback policy entry, four-wheel clearance verification, return to
the official joint owner, and follower resumption. During WP28→29, the same-level corridor
activated and no `CLIMB` or `DETOUR` mode was logged.

## Contents

- `raw/`: full ROS/simulator log, per-tick telemetry CSV, per-run and aggregate JSON,
  generated course YAML, and the exact replay NPZ captured during this successful run.
- `video/`: 1280×720 H.264 MP4 rendered from that replay on the official simulation-time
  timeline.

## Verified video

- File: `video/wp0_to_wp32_mac_seed8_720p_official_perception.mp4`
- Codec/profile: H.264 Main
- Resolution: 1280×720
- Pixel format: yuv420p
- Frame rate: 30 fps constant
- Frame count: 11,768
- MP4 duration: 392.267 s
- Official stopped-course elapsed: 392.257 s
- Duration difference: +0.010 s (less than one 30 fps frame)
- Size: 94,405,246 bytes
- SHA-256: `405f786f47aae9b91da04f04ac50908fb535bc531899a54d755258aa65383949`

FFprobe parsed the complete container successfully. FFmpeg decoded frames at both the start
and near the end without errors. A middle frame was visually inspected and showed the robot,
course, elapsed-time counter, current target, active policy, joint-owner overlays, exact horizontal
lidar rays/hit points, and the exact 13×9 relative-height scatter panel correctly.
