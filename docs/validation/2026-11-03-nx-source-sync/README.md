# NX 源码同步（2026-11-03）

仅更新本人 `zyh-nav2` 容器内 `/work/zyh_ws` 对应文件。SSH 已断开。容器同步前后均停止，未启动容器、未编译、未启动或重启任何 ROS 节点。未修改宿主机配置、网络、其他容器或运动接口。

15 个相关源码、测试及行为树文件通过 Docker 归档接口逐个核对，SHA-256 与本机一致，不重复覆盖。

本轮实际更新仅为 `src/nav2/navigation_bringup/config/nav2_mppi_params.yaml` 中的两项 Shim 参数：

- `angular_dist_threshold: 0.3490658503988659`（20°）。
- `angular_disengage_threshold: 0.3490658503988659`（20°）。

基于 NX 原文件定点合并，保留其本地代价地图膨胀半径 0.25 及其余参数。未将本机默认 0.35 覆盖过去。目标文件更新前后保留 UID/GID 1000 和原权限，并验证新文件及备份内容哈希。

容器内备份：`/root/zyh_sync_backups/shim-thresholds-20261103-58a14535`。

当前 `PathAlignCritic.offset_from_furthest` 仍为 20，PathFollow 和 GoalCritic 的 `threshold_to_consider` 仍为 1.2；之前的 5/0.4/0.4 只是建议，没有实施。昨天再次复核发现的四类取消分支也尚未修复，本轮未包含这些修复。

同步后的源配置需由用户编译安装、重新启动相关 launch 后用于运行；本轮没有更新 install/build 或运行时参数。

证据：`existing_code_verification.json`、`parameter_remote_before.json`、`nav2_mppi_params.remote-before.yaml`、`nav2_mppi_params.nx-merged.yaml`、`sync_plan.json`、`sync_receipt.json`。
