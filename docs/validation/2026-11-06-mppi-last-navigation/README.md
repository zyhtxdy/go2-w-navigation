# 2026-11-06 最近一次未绕行事件只读检查

只通过 SSH 读取 `zyh-nav2` 容器的日志、配置、已加载库，并查询运行参数。未修改远端配置或源码，未重启节点、启动导航、下发目标或速度。临时参数查询节点及 SSH 均已退出。

## 对应导航

以下时间按 NX 日志时间转换为 UTC+8，未与笔记本时钟重新对时。

- 最近一趟：10:06:56.901，从 `(4.51, -0.51)` 到 `(-0.16, -0.33)`。
- 初始全局路径为约 4.675m 的直线。
- 10:07:01.251：参考路线检查一次 `footprint_collision`。
- 10:07:02.501：`Keeping reference path while the local controller detours`，证明新协调服务已经参与当前运行。
- 10:07:06.797：MPPI `Optimizer fail to compute path`，当前 FollowPath 中止；随后清局部地图并重新规划。
- 10:07:06.950：全局规划起点报 `start_footprint_bridges_blocked`，诊断来源为 `/terrain/obstacles_measured_global` 的实测回波。此时日志所用全局障碍消息约 0.384s 前更新。
- 10:07:07～10:07:14：间歇出现 `current_footprint_collision`，部分检查又允许保留原参考线。
- 10:07:14.221：第二次 MPPI 求解失败，再次进入清图与重试。
- 10:07:24.301：目标最终返回成功。该成功不能证明机器人绕过了当时的行人。

第一回局部失败前，控制器日志没有新的 `Passing new path to controller` 事件；不能把该次失败解释为全局先换路打断局部绕行。真正的转向候选、实际角速度及绕行幅度没有历史记录，不能由日志还原。

## 已核实的运行参数

使用 `/get_parameters` 只读服务查询，区别于仅查看源 YAML：

- MPPI：1500 条候选，50×0.05s，`wz_std=0.45`，`vx_std=0.15`，`wz_max=0.6`，`vx_max=0.6`。
- PathAlign 权重 6，PathFollow 权重 6。
- CostCritic 使用矩形，权重 3.81，`near_goal_distance=1.0`。
- 局部地图为 odom 下 4×4m、0.05m 栅格；矩形 x±0.40m、y±0.25m，padding=0，膨胀半径 0.5m。
- 新 reference 路径检查参数为 0.75m、4s、12s；启动日志也确认新服务已注册。

## 能确定和不能确定的原因

结合已核对的 Humble MPPI 实现和当前启用的 critics，`Optimizer fail to compute path` 表示碰撞评分将一批候选全部判为不可行，重试后仍失败。此次证据支持“局部控制器未找到其认为无碰撞的绕行序列”，不支持“只是最终目标被取消”或“完全没有检测到障碍”。

全局当前矩形碰撞检查与 MPPI 使用不同地图/检查器，不能因两者接近同时失败就断言它们命中同一个体素，或断言真的已经发生物理接触。也没有当时的局部 costmap、采样轨迹和位姿同步快照，无法区分车体起始姿态已经被判占用、保守/残留障碍，或当前运动模型与采样未找到可行侧绕。不能认定增大标准差就能解决。

整个最新启动会话控制器日志包含 521 条控制周期超时警告；这是日志计数，不是丢包率。最新目标范围内的计数另见 `latest_goal_timeline.json`。这可能影响反应及时性，但仅凭警告不能确定避障失败的主因。

障碍观测过期与重定位超时警告集中在 10:08:06 之后，晚于该目标结束，不能倒推为 10:07:06/14 两次失败的原因。

## 文件

- `remote_logs.json`：读取时的最近日志及配置快照。
- `runtime_parameters.json`：运行参数、ROS 图和已加载库校验值。
- `latest_goal_timeline.json`：目标期间的控制器、行为树、全局规划事件时间线。
- 单独的 `*_2673_*.log`、`*_2683_*.log`、`*_2697_*.log`：对应当前会话原始日志。

参考版本源码：

- https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_mppi_controller/src/optimizer.cpp
- https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_mppi_controller/src/critics/cost_critic.cpp
