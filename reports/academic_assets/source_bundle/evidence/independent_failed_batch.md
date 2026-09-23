# macOS submission retest — 2026-08-20

## Tested delivery and clean-room setup

- Submission source: `<workspace-legacy>/resources/submission_20260820_397s_main3660b81 3`
- Team commit recorded by the delivery: `3660b81e8244dfa238633c161673596fe91650f6`
- Upstream commit: `13dd084be6cb5e2514098bc87e586d00dfe580b2`
- Submission archive SHA-256: `e5bf6be441128656aa86346ebad552b1efed4b603a6f1477b354129ab07abfec`
- All entries in the delivery's `SHA256SUMS` passed before extraction.
- The source was freshly extracted without `build/`, `install/`, `log/`, `.pytest_cache`, or Python cache directories.
- A no-cache arm64 image was built as `s10-racing:submission-3660b81-clean` (`sha256:33144dd5cdcffdd68589a83959222e4e388f18bf43a745b368f598ea3f558ba0`).
- Build outputs used a new named volume: `s10-submission-mac-3660b81-215224`.
- `scripts/verify_install.sh` passed the upstream revision, patched-source check, course extraction check, all three policy assets, and the required ROS packages. See `preflight_verify.log`.

The test did not reuse an existing build tree or running test container. Unrelated stopped Docker containers were left untouched.

## Test method

- Five sequential full-stack runs, seed order: `6, 8, 6, 8, 8`.
- Independent ROS domain IDs: `190, 191, 192, 193, 194`.
- Exact router configuration: `src/s10_bringup/config/strategy_gate16.yaml`.
- Full course: WP0 through WP32.
- Runtime capture requested at 10 Hz. The failed seed-6 runs retained their PNG state captures; the startup-aborted seed-8 runs ended before the first scheduled captured frame.
- The evaluator in this delivery reported `radius=0.200m` and `distance_mode=xy`; no evaluator or production setting was changed.

## Results

| Attempt | Seed | Result | Last official TRACK pass | Failure evidence | Raw capture |
|---:|---:|---|---|---|---|
| 1 | 6 | FAIL | WP4, `sim_time=43.957s` | router abort at 61.1° tilt while approaching WP5 | 800 PNG frames, CSV/JSON/log |
| 2 | 8 | FAIL | WP0 timer started; WP1 not passed | sensor data stopped updating; router abort after 3.0 s stale | CSV/JSON/log |
| 3 | 6 | FAIL | WP9, `sim_time=141.444s` | router abort at 64.7° tilt on WP9→WP10 | 1,435 PNG frames, CSV/JSON/log |
| 4 | 8 | FAIL | WP0 timer started; WP1 not passed | sensor data stopped updating; router abort after 3.0 s stale | CSV/JSON/log |
| 5 | 8 | FAIL | WP0 timer started; WP1 not passed | sensor data stopped updating; router abort after 3.0 s stale | CSV/JSON/log |

The `elapsed_s` fields in failure JSON files are recorder/wall-clock elapsed values, not official course completion times. None of these attempts reached WP32, so none has an official completion time.

## Video decision

No run met both required conditions (WP32 success and official time below 400 s). Therefore no 720p MP4 was generated. In particular, the retained failure frames were not presented as a successful course video. For a future qualifying run, perception visualization must be added to the 720p replay from the exact 8×64 lidar rays and 13×9 heightmap samples, and the encoded duration must be trimmed to the `[TRACK] Final waypoint ... elapsed=` value.

## Evidence locations

- Full logs, CSV, JSON, and captured frames: `raw/`
- Attempt lifecycle and ROS domains: `attempts.tsv`
- Batch console: `batch_console.log`
- Clean environment verification: `preflight_verify.log`
- Exact source used by the test: `source/goai26-s10-racing/`

## Main warning

All three seed-8 attempts reproduced an identical startup failure: MuJoCo loaded and announced lidar/heightmap dimensions, but the follower received no subsequent perception timestamp updates and aborted at 3.0 s stale. This is a deterministic startup/runtime issue in this Mac batch, not evidence of a navigation failure later on the course. The two seed-6 attempts did receive live perception and drove normally until separate high-tilt failures.
