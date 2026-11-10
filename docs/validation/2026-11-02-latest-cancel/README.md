# 2026-11-02 最新一次取消：通用定位不可用反馈

只读本人 zyh-nav2 的现有日志、已安装 Python 源码及启动生成的参数文件。未修改 NX、未重启节点、未发送目标或运动指令，SSH 已断开。

## 对应任务

最新启动会话约始于 epoch 1793587860；第一目标在 1793587913.547 成功。本次查询对应随后从 (2.19,-0.55) 到 (3.77,-0.96) 的目标：

| epoch | 事件 |
| --- | --- |
| 1793587917.600 | BT 开始任务 |
| 1793587917.760 | 全局规划返回 34 点、1.632 m 直线，报告计算 0.020 s |
| 1793587924.652 | 适配器收到 global_obstacle_localization_unavailable，立即停止并请求取消 |
| 1793587924.706 | BT Goal canceled |
| 1793587925.281 | 适配器重新 ready，距离失效约 0.630 s |

## 已确认与不能确认

直接取消来自通用定位不可用反馈。当前会话重定位日志只有加载和 aligned_cloud 的 QoS 不匹配告警，没有此前那种 global_verification_timeout、tracking_timeout 或过期全局结果告警。没有证据把本次归因于 12 秒复核期限到达、全局路径失败或物体碰撞。

已安装 adapter 哈希与本机修复版一致，包含 pending_localization_reflection；启动生成的规划参数为全局输入 2.0 s，定位安装参数为后台结果 6 s／全局复核 12 s，当前输入／跟踪 3 s。不存在简单认定‘忘了编译’的依据。

## 上轮覆盖不足的状态

C++ alignmentReady 仍要求 `/offline_localization/localization_available` 的接收心跳小于 **0.5 s**。它把‘值为 false’和‘接收心跳过期’都折叠成 false；发布 `/planner/live_obstacle_status` 时统一写为 localization_unavailable。

适配器此次的修复只允许：有已知底层可恢复原因，或已有零速度等待并等待反馈更新。若适配器自判定位正常、尚未开始等待，却先收到规划器这条通用失效报告，当前代码仍立即取消。源码中 `test_unexplained_planner_localization_loss_is_not_assumed_recoverable` 明确覆盖了这个保守分支；本次现场暴露该分支会误伤短暂的状态同步延迟。

这说明上轮修复还未覆盖‘规划器首先丢失心跳、适配器本身定位仍正常’的情况。应先分离 availability=false 与 heartbeat_stale，并为可验证的短暂通信／调度失效采用有界零速度等待；真实定位硬故障仍取消。

现有日志没有取消瞬间的接收心跳年龄或原始 Bool 事件，不能追溯证明一定是 >0.5 s 心跳延迟，也不能直接归因 Wi-Fi。依据上游无失效记录、短时恢复及代码门限，心跳／回调调度延迟是优先排查方向。

## 旁支告警

- 本次启动局部膨胀 0.25 m 小于模型内切半径 0.26 m，控制器日志有配置告警；本目标取消没有碰撞拒绝证据，不将它当本次直接原因。
- RViz 对 aligned_cloud 的可靠性要求与发布者不兼容，会影响该显示；它不等于此次取消的触发条件。
- 第一目标出现 robot_not_near_remaining_path 并重规划，随后成功；不与当前第二目标混算。

原始日志在 raw/，启动参数与版本核对在 running_version.json，关键结论与时间在 summary.json。本轮没有进行代码修复或部署。
