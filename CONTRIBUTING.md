# Contributing

## Getting set up

```bash
scripts/setup_upstream.sh    # clone and patch the contest SDK
scripts/build.sh             # build the overlay workspace
```

You need Ubuntu 24.04 with ROS 2 Jazzy. The contest material is a private repository, so
`setup_upstream.sh` needs git credentials with access to it. If your checkout lives
elsewhere, point `S10_UPSTREAM_DIR` at it instead of cloning.

## Ground rules

**Never edit `upstream/` directly.** It is a pristine checkout that gets deleted and
re-cloned. Changes to the SDK belong in `integration/` and are applied by
`scripts/patch_upstream.py`, which keeps every modification visible in one place and
reversible with `--revert`.

**Never hand-edit `config/course.yaml`.** It is generated from the track scene by
`scripts/extract_waypoints.py`. Editing it by hand lets the course we follow drift from
the course we are scored on.

**Change the observation layout in one place.** `training/s10_rl/observation.py` is the
source of truth. If you change it, the ONNX export and the SDK's `observation_dim` must
both follow — `export_onnx.py` prints the constant you need.

## Before opening a pull request

```bash
ruff check . && ruff format --check .
colcon test --packages-select s10_auto_nav s10_perception
scripts/extract_waypoints.py --check
scripts/patch_upstream.py --check
```

CI runs the same checks.

## Conventions

- Python targets 3.12, 100-column lines, formatted and linted with `ruff`.
- C++ follows `.clang-format` (Google style, 100 columns).
- Comments explain *why*, not *what*. The tuning constants in this codebase are the result
  of physical reasoning about the course — record that reasoning next to the number.
- Tests live beside the package they cover, in `test/` for ROS packages and `tests/` for
  `training/`.

## Tuning changes

Lap time is the metric, so a tuning change is a claim about performance. Include the
before and after times from the same course, and note whether the run was headless — the
viewer changes timing behaviour.

Raising `max_forward` without a correspondingly retrained policy drives the policy out of
distribution. If you raise it, say what the policy was trained on.

## Competition rules

Per the participant handbook, teams may not share code privately with other teams, and
substantially similar submissions can void results. Keep this repository restricted to the
registered team until the required open-sourcing.

Do not introduce GPL-licensed dependencies. This project is BSD-3-Clause to match upstream
and must stay redistributable under those terms.
