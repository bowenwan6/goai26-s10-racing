# Open-source plan

## Current scope

The competition integration is published under BSD-3-Clause. The open scope includes perception,
height-map generation, waypoint navigation, local planning, terrain handling, strategy routing,
safe single-owner controller handoff, reproducible experiment tooling, Docker/ROS deployment,
configuration, tests and documentation.

The repository is intended to remain runnable and reviewable after the competition. Changes are
developed on `bw-test-*` branches, reviewed through small commits and merged into the protected
`main` release line. Model and release manifests carry immutable source revisions and SHA-256
checksums.

## Boundaries

- The official S10 SDK, robot model and competition scene remain in the organizer repository and
  are not vendored. Setup scripts fetch/check the authorized pinned revision.
- Team-contributed Gate 16 and stairs model files are distributed for competition evaluation. Their
  contributor permission/license must be confirmed before broader redistribution; see
  `THIRD_PARTY.md`.
- Raw trials, high-volume replay frames and videos remain external evidence, with selected hashes
  and results documented in the repository.
- No credentials, private conversations, private SDK copies or unrelated research data are
  published.

## After the final

Subject to the organizer's rules and contributor approval, the team plans to:

1. tag the exact judged release and retain its dependency/model checksums;
2. publish the core perception-navigation-router stack and reproducible evaluation tools;
3. add a public issue template for reproducibility reports and hardware-porting feedback;
4. document the simulation-to-hardware odometry and safety interface required for a physical S10;
5. publish aggregate robustness results, including failed seeds, without exposing restricted
   organizer assets;
6. upstream generally useful fixes to the official SDK when the organizer accepts contributions.

Maintenance will prioritize reproducible builds, safety regressions, dependency updates and clear
separation between validated competition defaults and experimental policies. Material changes to
model licensing or the organizer's required open-source scope will be reflected in release notes.
