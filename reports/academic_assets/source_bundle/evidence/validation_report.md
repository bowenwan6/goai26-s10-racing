# Accepted seed-8 validation report — 2026-08-20

## Run outcome

- Code: public `main` commit `3660b81e8244dfa238633c161673596fe91650f6`.
- Platform: Mac arm64; clean image `s10-racing:submission-3660b81-clean`; Docker volume
  `s10-submission-mac-3660b81-215224`; ROS domain 203; seed 8.
- All 33 waypoints passed in strict order.
- Upstream timer: WP0 0.001 s, WP32 392.258 s, elapsed **392.257 s**.
- Recorder wall time: 674.01 s; not the course score.
- Distance: 249.94 m; final WP32 distance: 0.177 m; maximum tilt: 44.6°; stalls: 2.
- No skipped waypoint, fall, timeout or router abort; actual joint owner at recorder completion was
  `official`.

## Policy and navigation evidence

The full log records `stable_fallback entry envelope held`, Gate16 joint ownership, residual arm,
the 174D policy start, a four-wheel clearance candidate, verified clearance, residual disarm, safe
hold, official-owner acknowledgement and follower-state reset. WP16 passed at 152.871 s and WP17
at 155.488 s.

After WP28 passed at 345.309 s, the log records activation of the verified same-level corridor
toward WP29. WP28→WP29 contains no `CLIMB` or `DETOUR`, and WP29 passed at 353.676 s: 8.367 s.

## Package and test verification

- Offline upstream reset/reapply from retained Git metadata: passed.
- Clean isolated ARM64 Docker build: five packages built.
- `scripts/verify_install.sh`: passed pinned upstream, applied patch, course, three model hashes and
  four required ROS-package checks.
- Source suite excluding the separately tracked joint-owner fixture: 378/378 passed.
- Training/observation contract suite: 29/29 passed.
- Joint-owner fixture: compiled, but five checks for the unused adaptive in-process Gate16 path
  failed, causing nine pytest setup errors. Stable fallback runtime ownership succeeded in the
  accepted full run; the fixture issue remains disclosed as a risk.
- Secret/private-key scan, broken-symlink scan and generated-junk scan: passed.

## Video verification

`wp0_to_wp32_mac_seed8_720p_official_perception.mp4` is H.264 Main, yuv420p, 1280×720, constant
30 fps, 11,768 frames and 392.267 s. It differs from the official elapsed by +0.010 s. Its right
panels show exact per-frame horizontal lidar rays/hit points and 13×9 relative heightmap samples.
FFprobe parsed the
file and the source run's SHA-256 verification passed. Digest:
`405f786f47aae9b91da04f04ac50908fb535bc531899a54d755258aa65383949`.
