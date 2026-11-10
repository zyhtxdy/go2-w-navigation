# 2026-10-26 目标支撑搜索修复与 NX 功耗检查

## 刚发生的目标失败

当前容器日志记录：目标约为 `(0.18, -1.51)`，规划反复报 `goal_no_support_xy`，最终顶层任务为 `Goal failed`。这次在目标支撑候选筛选阶段就结束，没有进入路线搜索。不是当前这条记录中的矩形碰撞或控制器性能超时。

核对容器 `/work/zyh_ws/maps/lab_v1_navigation_guarded_20261026/support.csv`，最近观察支撑点为 `(0.20, -1.40, -0.33861)`，距取整后的目标 XY 约 0.112 m。容器实际启动参数的 `goal_projection_xy_radius` 为 0.2 m。

本机 `docs/validation/2026-10-26-live-path-obstacles/navigation_candidate` 的 support/obstacles 文件 SHA256 与容器地图一致；本次没有改地图。

## 修复

原 `OccupyMap::surfaceCandidatesUnlocked` 用整数格子编号差做圆形筛选。目标在格内偏离中心时，会漏掉实际半径内的支撑列，也会纳入实际半径外的列。

例如原始支撑 `(0.20,-1.40)` 对应支撑格中心 `(0.25,-1.35)`。目标 `(0.18,-1.51)` 到这个格中心约 0.175 m，在 0.2 m 内；格子编号却相差 `(1,2)`，被原来的两格半径筛掉。

现在按查询点到候选格中心的真实 XY 距离筛选和排序。保留查询自身所在列的原语义，使零半径的本列查询继续有效。没有增加支撑点、扩大搜索半径、缩小车身或删除障碍。高度选择、地面连通、矩形扫掠与动态障碍失效保护继续参与后续检查。

修复前，新增的“半径内漏选”和“半径外误选”回归案例均失败，完整规划插件的边界目标案例也失败；修复后均通过。插件案例使用构造的平地和对应的目标/支撑格关系，不是完整恢复当时的实时障碍和姿态，不能据此声称真机路线已经成功。

## 功耗模式与采样数

只读检查 NX 宿主机得到：

- 型号：NVIDIA Orin NX Developer Kit；Jetson Linux R35.3.1。
- 当前模式：`MAXN`，模式编号 0。
- 8 个 CPU 核心在线。
- 读取瞬间，两组 CPU 策略的当前频率与最大频率均为 1,984,000 kHz。
- CPU 温度约 70.7 °C，为单次读取值；没有据此判定发生温度降频。
- 配置中的可选模式为 MAXN、10W、15W、25W；没有修改或切换模式。

用户希望保留采样数。本机最终配置保留 `batch_size: 1500`、`time_steps: 50`、`model_dt: 0.05`、控制频率 20 Hz。中途尝试的 1000 已恢复，未同步到 NX。配置改动仅更新过时的说明注释。

MAXN 已打开，不能再通过从低功耗切换 MAXN 获得提升。锁频与切功耗模式是不同操作；当前 CPU 瞬时频率已到配置上限，不能保证锁频能解决整套节点的超时。标准 Nav2 MPPI 是 CPU 实现，提高 GPU 功耗或 GPU 时钟不会直接把它变成 GPU 计算。

参考：[NVIDIA R35.3.1 功耗管理文档](https://docs.nvidia.com/jetson/archives/r35.3.1/DeveloperGuide/text/SD/PlatformPowerAndPerformance/JetsonOrinNanoSeriesJetsonOrinNxSeriesAndJetsonAgxOrinSeries.html)、[Nav2 Humble MPPI 说明](https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_mppi_controller/README.md)。

## 验证和范围

本机 Release 编译 `astar_3d_planner`、`navigation_bringup` 成功。最终测试：规划器 190 项、导航插件加载 1 项，全部通过，无失败/错误/禁用项。

首轮完整检查有两个失败：一项旧诊断断言原本要求“生成超出半径的邻居，再在扩展时拒绝”，现改为验证候选筛选阶段已经排除该邻居，同时保留连接半径不足失败、足够后成功的检查；另一项插件加载失败来自未 source 工作空间。补齐工作空间环境后的最终完整检查全部通过。

修改文件：

- `src/nav2/astar_3d_planner/src/occupy_map.cpp`
- `src/nav2/astar_3d_planner/test/test_astar_3d.cpp`
- `src/nav2/astar_3d_planner/test/test_offline_goal_selection.cpp`
- `src/nav2/astar_3d_planner/test/test_start_diagnostics.cpp`
- `src/nav2/navigation_bringup/config/nav2_mppi_params.yaml`（仅说明注释，采样与控制参数最终保留）

本机验证阶段只在 NX 上读取日志、自己容器的参数/地图与宿主机功耗状态，没有修改远端或下发运动。后续按用户明确要求同步源码，范围和状态见下文。

之前记录的实时障碍矩形碰撞、起步连接失败以及控制周期超时，不能由这个目标候选修复一并宣称解决。小偏移导致假想转向的问题仍需带失败时刻完整路径与机器人姿态的数据验证；当前姿态的真实碰撞不能豁免。

证据见 `followup_logs.json`、`goal_support_check.json`、`goal_parameters.json`、`nx_power_readonly.json`、`regression_before.log`、`plugin_before.log`、`tests_final.log`、`navigation_tests_final.log` 与 `test_counts.json`。

## 后续源码同步

已按用户要求同步到 `zyh-nav2:/work/zyh_ws`，仅更新规划器实现和三个配套测试文件，共四个文件。逐一校验 SHA256 与本机通过测试的版本一致，保留原 UID/GID 1000。

容器的 `nav2_mppi_params.yaml` 有既有参数差异，本轮没有整份覆盖；其文件 SHA256 同步前后完全一致，采样数仍为 1500。没有更改地图、其他容器或宿主机设置。

备份为 `/root/zyh_sync_backups/metric-support-20261026-e5eafba8`。没有在 NX 上编译或重启，当前运行中的规划库不会因此自动更新；修复需要后续编译并重新启动导航才会生效。SSH 已断开。

同步证据见 `sync_remote_before.json`、`sync_receipt.json`；前述 `changes.diff` 为本机变更，不能当作远端全参数覆盖记录。
