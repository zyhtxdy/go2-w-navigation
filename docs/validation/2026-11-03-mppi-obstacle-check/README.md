# 2026-11-03 MPPI 动态避障链路检查

仅读取本人 zyh-nav2 容器的日志、参数和低带宽话题。未修改参数、未启动导航、未下发目标或运动指令，SSH 已关闭。

## 已确认

- 当前 primary_controller 为 nav2_mppi_controller::MPPIController，CostCritic.enabled=true、consider_footprint=true、collision_cost=1000000。
- /terrain/obstacles 发布者为 /terrain_costmap，/local_costmap/local_costmap 正在订阅。5 秒被动观察收到 19 帧障碍与 19 帧局部代价地图。
- 当前局部障碍 base_link 点云帧有 5742 点，参数高度/距离筛选通过 5742 点；当前局部地图 80×80，包含 142 个致命障碍格、1574 个内切代价格、0 个中间软代价格。此为一个当前快照，不是当时行人所在位置的历史地图。
- NX 局部 inflation_radius=0.25 m，footprint 为 ±0.40、±0.25 m，footprint_padding 约 0.01 m；带 padding 的内切半径约 0.26 m。膨胀范围没有越过内切半径，当前图中缺少外围连续衰减的软代价梯度。
- MPPI 时间窗口为 50×0.05=2.5 s，局部地图 4×4 m。全局检查覆盖剩余路径，会比有限时域的局部优化更早判定远处路径被挡住。

## 最近事件（北京时间）

| 时间 | 事件 |
|---|---|
| 16:24:26.610 | 第一次全局路径 footprint_collision |
| 16:24:27.828 | 连续三次无效，触发重规划 |
| 16:24:27.889 / 28.036 | 新规划 start_footprint_bridges_blocked |
| 16:24:28.132 | FollowPath 子任务被取消并停车 |
| 16:24:33.414 | 控制器重新收到跟踪任务 |
| 16:24:42.707 | 整个导航目标成功 |
| 16:25:00.133 | 下一次任务触发重规划 |
| 16:25:00.486 | FollowPath 子任务取消，随后反复清图、规划失败 |
| 16:25:17.859 | 整个目标 Goal failed（ABORTED，不是 CANCELED） |
| 16:25:52.729 | 再下一次任务触发重规划 |
| 16:25:53.084 | 直接向运行中的控制器传入新路径 |
| 16:26:04.212 | 整个目标成功 |

前两次是全局重规划失败/恢复过程停止了局部跟踪。不能笼统声称每一次全局重规划都会中断 MPPI，第三次为新路径更新。

## 判断与限制

MPPI 的避障是每周期持续参与轨迹打分，没有单独的“触发避障”开关。当前链路连通不代表已证明当时行人进入了局部地图，也不代表已证明 MPPI 能在当时空间和约束下绕过去。

现有证据支持：全局路径校验/恢复流程在两次事件中中断 FollowPath；局部软膨胀梯度配置缺失可能不利于提前形成平滑绕行。后者是配置与当前快照支持的风险，尚非当时没有绕行的唯一已证实原因。

原转向包没有录 /terrain/obstacles、/local_costmap/costmap、控制器归属或 MPPI 轨迹评分，无法完整回溯当时局部规划决策。下一次针对性记录应补齐这两个地图/障碍话题及 /plan、/odom、各级 cmd_vel、/tf、/tf_static，并保留控制日志。

建议先实现动态障碍重规划窗口与跟踪路径裁剪，再独立验证局部 inflation_radius 0.55 m（建议值，尚未修改；进一步版本核对发现中心零代价会提前跳过 footprint 检查，需覆盖约 0.486 m 的配置外接半径，详见 cost_critic_inflation_audit.md）；矩形 footprint 与碰撞检查保持。静态碰撞、TF/定位失效不能按普通动态避障窗口放行。

## 证据

- latest_events.json：最近导航、规划、地形和控制日志摘要。
- runtime_snapshot.json：实际运行参数、局部数据快照和订阅关系。
- controller_interruption.json：局部跟踪取消/恢复原文及 padding 参数。

## 后续版本核对更新

进一步确认了 CostCritic 中心代价为零时的提前跳过逻辑，问题不止缺少软梯度。此前 0.40～0.45 m 的建议不足以覆盖当前配置的外接半径，已修正为先独立验证 0.55 m。详见 cost_critic_inflation_audit.md。
