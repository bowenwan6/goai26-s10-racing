## Summary

<!-- What changes, and why. -->

## Effect on lap time

<!-- Delete if not applicable. Same course, note whether the run was headless. -->

| | Lap time | Waypoints reached |
|---|---|---|
| Before | | / 33 |
| After | | / 33 |

## Checklist

- [ ] `ruff check .` and `ruff format --check .` pass
- [ ] `colcon test` passes
- [ ] `scripts/extract_waypoints.py --check` passes, or the course was regenerated
- [ ] Observation layout unchanged, or `observation.py`, the ONNX export and the SDK's
      `observation_dim` were all updated together
- [ ] No edits made directly inside `upstream/`
- [ ] No new GPL-licensed dependencies
