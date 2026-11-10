# 导航取消链二次复核（2026-11-02）

本轮只读复核本机源码、已安装 Nav2 1.1.20 对应本地源码和已保存日志；未连接 NX、未启动 ROS 节点、未下发控制、未修改生产源码。当前 845 项回归和 15 项录包执行验证不能覆盖下列新增边界，不能据此宣称所有误取消已排除。

## 确认仍存在的四类父目标取消入口

| 编号 | 场景 | 现有贯穿路径 | 处理建议 |
| --- | --- | --- | --- |
| 1 | adapter 因自身断流重置确认，上游却仍 valid，恢复第一条正常结果 | try_accept 只有 1/3 帧，报 confirming_alignment_or_correction；已有 holding 中仍按硬故障 pause | 区分正常重确认与超限位姿修正；正常重确认沿原始 10 秒窗口零速等待，不续期 |
| 2 | 上游收到单条坏、旧或未来点云，以及坏里程计 | relocalization_node 回调直接 valid=false；adapter 的恢复表不包含这些原因，合法 false 立即触发 pause | 上游丢弃坏/旧样本并保留仍新鲜的 last-good；拆分真实时钟故障，不能用坏帧刷新新鲜度 |
| 3 | 上游收到单条落后超过 0.2 秒的里程计 | 作为 odometry_discontinuity 清历史并撤销定位，最终硬取消 | 普通乱序丢弃；真实 ROS 时钟回退与空间跳变继续保留保护 |
| 4 | 单次前台配准或后台验证点数/质量未通过 | insufficient_points、registration_failed、low_overlap、high_rmse、low_quality、degenerate_geometry 均撤销定位并硬取消 | 这些是真实质量拒绝，不能一概称无关紧要；可先停车进行有界恢复，歧义、超限修正仍硬保护 |

编号 1 已用实际源码的 try_accept/set_ready 方法 AST 提取进行无 ROS 故障复现：单位变换、LIO/EKF 完全一致、鲜活结果、gate.reset 后 holding=True；第一条 valid 结果使 count=1，输出 confirming_alignment_or_correction，holding=False、blocked=True。没有真实位姿跳变也会进入父目标取消。

关键位置：

- src/go2_offline_navigation/go2_offline_navigation/localization_adapter.py:443，正常未满确认帧与位姿修正共用原因。
- 同文件 :519，只有 awaiting_distinct_consistent_frames 在 holding 中可恢复；confirming_alignment_or_correction 不在该分支。
- src/go2_relocalization/src/relocalization_node.cpp:243，单条坏里程计直接 invalidate。
- 同文件 :249，落后超过 0.2 秒的单条里程计被当作 discontinuity。
- 同文件 :262，单条坏点云直接 invalidate；:266 旧或未来点云也直接 invalidate，先于 :270 的重复旧戳过滤。
- 同文件 :638，单次后台验证拒绝；:674 单次前台配准拒绝；均 resetStateLocked 并发布 false。

## 不应与父目标取消混为一谈

- 路径匹配 0.30 m 阈值、连续 3 次几何失败：触发重规划，本身不直接取消父目标。
- 输入/RPC 暂不可用：行为树停止 FollowPath 子任务，父树 RUNNING 等待，生产 XML 12 秒。
- 速度桥接 0.3 秒超时：停车，不直接调用导航 cancel 服务。
- controller 的单次 TF getRobotPose 失败、MPPI 两轮均无有效结果，可能跳过 PlannerException 的 1 秒耐心，使 FollowPath 提前失败；行为树仍有清图、重规划及恢复。它们是潜在停顿/重试来源，不等于父 NavigateToPose CANCELED。
- 控制器日志 Goal was canceled 可能只是行为树停止 FollowPath 子任务。父目标最终 FAILED/ABORTED 与 CANCELED 需按 NavigateToPose 状态和 adapter 取消日志区分。

真实地图身份冲突、明确歧义定位、超限位姿修正、真实 ROS 时钟回退仍应保留硬保护。上述代码路径不等于已证明最近每次真机取消都命中了这些原因；本轮没有新的真机记录。

## 状态

以上四类仍待修改；本轮没有修复或部署它们。上次 NX 同步已核实的 14 个文件仍只是上一轮取消策略版本。Shim 的 20°/20°更改仅在本机；PathAlign/PathFollow/GoalCritic 建议值也尚未改动。
