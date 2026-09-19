# Third-party dependencies and data sources

This document states the runtime, model and data boundaries of the GOAI 2026 Track 4 Challenge 2
entry. It is a disclosure record, not a replacement for each upstream project's license text.

## Organizer material

| Material | Source/version | License or access boundary | Use |
|---|---|---|---|
| GOAI S10 SDK, robot model, MuJoCo course and waypoints | `DeepRoboticsLab/goai_embodied_future_material` at `13dd084be6cb5e2514098bc87e586d00dfe580b2` | BSD-3-Clause; repository access is granted by the organizer | Physics, official 57D locomotion policy runner, robot interfaces and official waypoint timer |

Organizer material is cloned into the ignored `upstream/` directory and is not redistributed by
the public Git-derived package script. The complete judge-facing offline ZIP includes the team's
authorized pinned checkout because the requested delivery must build without another download;
it should be shared only through the competition submission channel unless the organizer permits
wider redistribution. `scripts/setup_upstream.sh` checks out the pinned revision and applies the
submission's integration patch. Course coordinates in `src/s10_bringup/config/course.yaml` are a
generated configuration derived from the official scene and are verified against that checkout.

## Policy assets

| Asset | Provenance | Runtime role | Disclosure |
|---|---|---|---|
| `policy/gate16/policy.onnx` | `belsun/goai-s10-gate16-policy`, integration source `216b77affa550359e73f4e71944d2853c64959ef`, asset bundle `b6535a48bf3d72f3ab2f3e37f8b555eb15aa3e64` | Frozen 174D-to-16D Gate 16 base model | Included, checksum in its manifest |
| `policy/gate16/climb_residual.onnx` | Same Gate 16 bundle | Heightmap-gated residual used only on WP15-to-WP16 | Included, checksum in its manifest |
| `policy/stairs57/policy.onnx` | Team-supplied `s10_stairs_up_57d_model1800_teammate(1)` bundle | Experimental 57D-to-16D stair model | Included for interface traceability but `stairs57_enabled: false`; never used in the accepted run |

The Gate 16 source bundle and teammate stairs bundle did not contain a standalone license file
when integrated. They are recorded as team-contributed competition assets, not claimed as original
work of the navigation repository. Before distribution beyond the registered team/judges, the
team must obtain and retain written contributor permission or add the contributor-approved license
and attribution. This is the only unresolved model-license item in the current package.

No commercial API, hosted model, external online service or paid inference service is called at
runtime.

## Runtime dependencies

| Dependency | Validated version | License | Purpose |
|---|---:|---|---|
| Ubuntu | 24.04 LTS | Ubuntu component licenses | Base operating system |
| ROS 2 Jazzy | pinned `ros:jazzy-ros-base` image digest | Apache-2.0 and component licenses | Messaging, launch and package runtime |
| Python | 3.12.3 in the validated image | PSF | Python nodes and tooling |
| NumPy | 1.26.4 | BSD-3-Clause | Arrays and geometry |
| MuJoCo | 3.11.0 | Apache-2.0 | Official course simulation |
| SciPy | 1.17.1 | BSD-3-Clause | Dependency of the upstream simulator |
| ONNX Runtime | 1.28.0 | MIT | Gate 16 and optional stairs model inference |
| PyYAML | 6.0.1 from Ubuntu/ROS image | MIT | Course and parameter configuration |
| Docker Engine / Docker Desktop with Compose v2 | current supported release | Apache-2.0 / Docker terms | Portable evaluator environment; not needed for native Ubuntu |
| Mesa/OSMesa | Ubuntu 24.04 package | MIT and component licenses | Headless software rendering |
| FFmpeg | host-provided, optional | LGPL/GPL by build | Evidence-video encoding only; not used by autonomy |

The direct Python packages pull the following permissively licensed runtime dependencies, all
explicitly pinned in `docker/requirements.lock`: `absl-py`, `etils`, `flatbuffers`, `fsspec`,
`glfw`, `protobuf`, `PyOpenGL` and `typing_extensions`. Their upstream licenses are combinations
of Apache-2.0, BSD, MIT and PSF-style terms. ROS and Ubuntu packages come from the digest-pinned
multi-architecture base image. ROS package-level dependencies are declared in each
`src/*/package.xml`.

## Data and telemetry

- Scene geometry, robot state and waypoint locations originate from the organizer's official S10
  MuJoCo material.
- Lidar and height-map observations are generated online by ray casts against that scene; there is
  no external perception dataset in the competition runtime.
- The Gate 16 model was delivered by a team contributor after training against the official
  simulator. Its source checkpoint paths and hashes are preserved in the bundle manifest.
- Self-test logs, telemetry, replay state and MP4 files are retained under the team's external
  `resources` evidence directory and are not part of the public code license or Git history.
- No personal data, user content or external web data is consumed by the autonomy stack.
