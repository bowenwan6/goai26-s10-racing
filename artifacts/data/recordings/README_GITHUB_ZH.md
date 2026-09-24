# GitHub 中的录制索引

本目录提交录制说明、原始文件 SHA-256、rosbag 元数据、配置参考和审计证据。

`slam_test_20260917_170443/raw_bag/` 中的 8 个 MCAP 分卷未随 Git 上传，仍位于原 GOAI 工作区及机器人端。克隆仓库后不能直接回放这次录制；需要另外准备完整 MCAP 文件，再按 `MANIFEST.json` 校验。

- [录制说明](slam_test_20260917_170443/README_先读我.md)
- [原始交付清单](slam_test_20260917_170443/MANIFEST.json)
- [消息与时间审计](slam_test_20260917_170443/evidence/bag_audit.json)

当前可从仓库取得的全场点云位于 [v3 场景包](../deliverables/S10_v3_Map_MuJoCo_20260916/README.md)，它与原始逐帧 LiDAR/IMU 录制用途不同。
