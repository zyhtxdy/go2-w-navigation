# 2026-11-06 NX 源码同步

按用户要求只同步代码，未编译、未启动导航或下发运动指令。

- 目标：`unitree@10.56.61.177` 的 `zyh-nav2` 容器，`/work/zyh_ws`。
- 同步范围：11-05 绕行协调及重规划起点修复的 17 个文件，涉及 `astar_3d_planner`、`navigation_bringup`。
- 两份 YAML 仅新增 `local_detour_max_deviation: 0.75`、`local_detour_progress_timeout: 4.0`、`local_detour_timeout: 12.0`；用解析后的结构比较确认其他既有参数未改变。
- NX 自调的速度、Shim、MPPI 采样及速度平滑器参数均保留，没有用笔记本默认值覆盖。
- 17 个文件复制后 SHA256 全部匹配计划结果；已有文件的 UID、GID 和权限保持不变；两个包中其他文件未改变。
- 通过 Docker 复制，并只读核对该容器 `/work` 绑定挂载中的实际文件，校验一致。
- 容器最初停止，同步前由外部启动；本次没有启动、停止或重启容器。复制前后均为运行状态，检查时未发现导航节点。
- 未修改安装目录或系统配置，未操作其他容器。源码使用前需重新编译。

远端原文件备份：`/work/zyh_ws/.codex_sync_backups/source_20261106_094904.tar.gz`。

`planned_sync.json` 记录预期哈希和保留的 NX 参数；`sync_result.json` 记录复制后的校验结果。`before/` 和两份包的 `*_before.tar` 保存同步前文件；`*.applied.diff` 为实际应用的配置增量。
