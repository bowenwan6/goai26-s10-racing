# Gate 16 v1.5 Bowen integration

## Scope

This branch adds a confidence-selected entry contract on top of Bowen's current `main`.
It does not replace either ONNX graph or the SDK-local Gate16 runner. In particular, it
keeps measured actuator-state history seeding, explicit residual arm, joint ownership,
physical wheel-centre support detection, and four-wheel exit verification.

The selected contract is fixed at the far staging point for the rest of the attempt:

| Contract | Distance (m) | Forward speed (m/s) | Heading | Lateral | Yaw rate | Dwell | Command |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `fast_profile` | 0.60-0.65 | 0.23-0.27 | <= 2.5 deg | <= 0.08 m | <= 0.05 rad/s | 0.00 s | 0.25 m/s |
| `stable_fallback` | 0.62-0.70 | 0.08-0.20 | <= 6.0 deg | <= 0.25 m | <= 0.10 rad/s | 0.10 s | 0.15 m/s |

If the staging pose satisfies both contracts, `fast_profile` wins. A fallback entry does
not extrapolate the fast command profile: its unmatched adapter command stays at 0.15 m/s.
Both modes still arm the same frozen base+residual actor and use physical wheel positions
for the front-support phase change.

Set `gate16_fallback_enabled: false` in `strategy_gate16.yaml` to recover the current-main
adaptive-v3 entry behavior without reverting code.

## Why this is not a runner transplant

Bella's standalone v1.5 uses a height-map confidence gate around the release runner.
Bowen's `main` already has a stronger full-stack boundary: the router controls entry and
the ROS adapter waits for measured front wheel centres above and beyond the deck edge.
Copying the standalone C++ runner would remove those protections and reintroduce the
heightmap-only early-tuck failure seen in the `yaw18` ablation.

## Same-state A/B

Compare these three revisions without changing seed, spawn pose, waypoint radii, or router
parameters other than those committed on the branch:

1. `bw-test-gate16-stable-integration` at `45ebb49` (stable ver1)
2. `main` at `1854a0b` (adaptive-v3)
3. `bw-test-gate16-v1-5-confidence-fallback` (this branch)

Run at least six Gate16 attempts per revision. Record:

- Gate16 success and full WP0-WP32 completion;
- time from `CLIMB_READY` to four-wheel verification;
- selected `gate16_entry_mode` and command profile;
- staging distance, forward speed, heading, lateral error, and yaw rate;
- first failed invariant or timeout reason.

The decision should use success count first, then median successful climb time. Do not pool
fast and fallback runs: the mode split shows whether v1.5 is gaining tolerance or merely
slowing every attempt.

Example full-stack invocation from the repository root:

```bash
CONTAINER_NAME=s10-v15-ab docker/run.sh scripts/run_segment.py \
  --start 0 --end 32 --seeds 6 --max-time 2400 \
  --out /ws/results/wp0_32_v15 --tag v15-ab --router \
  --router-params /ws/src/s10_bringup/config/strategy_gate16.yaml
```

Expected log markers:

```text
fast_profile entry envelope held
stable_fallback entry envelope held
Gate16 command profile selected: ... mode=fast_profile ...
Gate16 command profile selected: unmatched mode=stable_fallback ...
```

## Verification completed before handoff

- Python compile check passed.
- `304 passed, 9 skipped` across `src/s10_auto_nav/test`.
- Three asset-dependent tests were deselected because the Windows review clone uses sparse
  checkout and does not contain `policy/gate16`. No ONNX, manifest, profile JSON, or C++
  runner file changed in this branch.
- Full-stack simulation of this integration branch is still required before merging.
