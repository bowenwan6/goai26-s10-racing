# ROS 2 → ROS 1 sensor conversion: measured evidence (2026-09-19)

Captured on dog 048 with the gateway on the AGX (102) and the read-only tap on the
vendor perception board (106). These are the raw artifacts behind the gateway numbers
quoted in the root [README](../../../README.md).

The conversion itself has not changed since 2026-09-19, so this audit still describes
the code on `main`.

## 1. Byte-equality audit — `live-20260919-184451/`

A 59.2 s recording, compared field by field against an independent ROS 2 reference
taken at the same time.

| | `/LIDAR/POINTS` | `/IMU` |
|---|---|---|
| Matched | **592 / 592** | **11 845 / 11 845** |
| Field mismatches | **0** | **0** |
| In ROS 1 but not ROS 2 | 0 | 0 |
| Coverage of the shared window | **1.000** | 0.99983 |
| Rate from stamps | 9.999 999 7 Hz | 199.966 Hz |
| Backwards / duplicate stamps | 0 / 0 | 0 / 0 |
| Message md5 | `1158d486…` | `6a62c6da…` |
| md5 matches stock Noetic | **yes** | **yes** |

`ok: true`, `problems: []` for both topics. Publisher `callerid` is `/s10_ros1_gateway`
on both. The IMU coverage is 0.99983 rather than 1.000 because two of the 11 847 ROS 2
messages inside the shared window fall outside it by less than one sample period — not
a dropped message.

`agx_rosbag_info.txt` is `rosbag info` on the ROS 1 bag the audit read: 809.2 MB,
12 437 messages, 593/593 chunks, no compression.

**Not included here:** `ros1.jsonl` and `ros2_ref.jsonl`, the per-message digests the
audit compared (20 MB together), and the 809.2 MB bag, which was not kept. The summary
above is reproducible from the digests; ask if you need them.

## 2. Ten-minute soak — `soak-20260919-185311/`

18:53–19:03, 600 samples. Two samplers, one per board — the numbers are **not**
comparable across files because they describe different processes.

**AGX (102), `resagx.txt` / `resagx.csv`** — the gateway, pid 11395:
cpu mean **30.3 %** of one core (p95 34.9 %, max 36.9 %), rss max **67 MiB**, 599 samples.

**Perception board (106), `res106_summary_transcribed.txt`** — the tap, pid 91466:
cpu mean **8.4 %** of one core (p95 9.0 %, max 10.0 %), rss max **185 MiB**.

The point of the 106 sampler is the line under it: the vendor lidar driver ran at
**13.4 %** mean with the tap attached against **14.1–14.6 %** before it. Adding the tap
did not measurably load the vendor driver — which is the whole claim the read-only
design has to support.

`health.log` is the gateway's own minute-by-minute output over the same window:
**13 621 / 13 621 frames received = sent, 0 dropped, 0 errors, 0 rejected**, 19.3 GB
forwarded over a single connection, holding 9.96–10.00 Hz lidar and 199.79–199.99 Hz
IMU with 0 backwards stamps throughout.

> [!NOTE]
> `res106_summary_transcribed.txt` is a **transcription** of the sampler's output, read
> at 19:09 — not the raw file. The raw `res106.csv` was deleted by a
> `robot_session.sh down/up` test at 19:35 before it could be copied. The AGX side
> (`resagx.csv`) is raw. Treat the 106 figures as reported, not as primary data.

## 3. What this evidence does and does not cover

**Measured:** the one audit and the one soak above. That is the whole of the measured
record for the conversion.

**Operational, not measured:** the same tap and gateway ran continuously underneath
every field session on 2026-09-20, 09-21 and 09-22 on dog 048 — roughly 20+ navigation
runs, including the full 272 m course at 29/29 waypoints on 2026-09-21. No run ever
failed on sensor data. There are no per-run frame counts for those sessions: the
gateway's health lines stayed in `~/ros1_gateway/logs/gateway-*.log` on the AGX and were
never copied off. So this is an accurate operational claim and not a measurement, and it
is recorded here as such rather than as a second audit.

## 4. Redactions

The robot-LAN address of the perception board is replaced by `<PERCEPTION_BOARD>` in
`health.log`, and the three absolute path fields in `live_audit.json` are rewritten to
`<artifact-dir>/`. The network and DDS survey dumps from the same campaign are **not**
published: they are raw interface, MAC, SNMP and discovery data for the robot's internal
network, and none of it is needed to substantiate the numbers above.
