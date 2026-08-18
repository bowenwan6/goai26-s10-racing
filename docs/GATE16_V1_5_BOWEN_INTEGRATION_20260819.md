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
| `stable_fallback` | 0.62-0.70 | 0.08-0.20 | <= 6.0 deg | <= 0.25 m | <= 0.10 rad/s | 0.10 s | 0.18 m/s |

If the staging pose satisfies both contracts, `fast_profile` wins. A fallback entry does
not extrapolate the fast command profile: its unmatched adapter command stays at 0.18 m/s.
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
- `339 passed, 12 skipped` across the navigation and perception test suites.
- A real MuJoCo scan test covers the 3.3.1 `mj_multiRay` signature used by full-stack runs.
- Three asset-dependent tests were deselected because the Windows review clone uses sparse
  checkout and does not contain `policy/gate16`. No ONNX, manifest, profile JSON, or C++
  runner file changed in this branch.

Focused production-stack evidence from WP15 to WP17 is recorded in
`evaluation/gate16_v15_bowen_fullstack_20260819.csv`. With identical seeds 6-10, the stable
fallback at 0.15 m/s reached WP17 in 3/5 runs. Raising only its persistent command to
0.18 m/s reached WP17 in 4/5, rescued seed 9's straddle, reduced median successful elapsed
time from 37.25 s to 34.80 s, and did not increase the 44 deg worst successful tilt.
Every entry selected `stable_fallback` at about -5 deg yaw; the fast path still needs a
same-state focused trial that actually reaches its stricter staging contract.

Seed 11 never reached either entry contract. A longer alignment arc, a fallback-aware
straightening threshold, and one safe retry all failed, so none of those experiments is in
this branch. This avoids making the three proven successes depend on an unvalidated arrival
controller change.
