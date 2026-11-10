# 2026-11-02 第二次导航停顿与取消排查

只读本人 zyh-nav2 的现有日志、当前参数及 2 秒负载；未改 NX 文件或参数，未启停节点，未发送目标、速度或运动请求。SSH 已断开。

## 目标与结果

第二次目标：从约 (2.15,-0.74) 到 (0.79,-0.91)，开始 epoch 1793584983.889689888，取消 epoch 1793585003.290467808，持续约 19.401 秒。第一目标已成功，不将后续第三及其他任务混算。

全局规划器于 1793584984.105830944 返回 29 点、1.375 m 直线路径，报告计算 0.011 秒。此次没有全局搜索失败或路径几何碰撞的对应日志。

## 停顿与恢复

| 相对本目标开始 | 输入门控原因 | 等待到复核通过 |
| --- | --- | --- |
| +1.863 s | global_obstacle_source_stale | 0.669 s |
| +2.989 s | global_obstacle_source_stale | 1.176 s |
| +7.568 s | global_obstacle_source_stale | 0.296 s |
| +12.171 s | global_obstacle_source_stale | 1.745 s |

日志中有四次短暂的输入门控恢复，不等同于四次肉眼明显停顿。它们表示规划器已消费障碍消息的源时间过期，不表示真的检测到了障碍；恢复后日志为 current path revalidated，沿任务继续跟踪。本项源时间门限为 1 秒。最后还有 +18.869 s 的过期，随后进入取消链。

## 最终取消链

1. 1793585002.539：后台匹配结果被前台消费时判过期，reason=foreground_consumed_stale_result；后台计算约 3935.9 ms，日志保存的结果年龄约 3.994 s，接近 4 s 接收预算。日志的年龄快照不等于每个后续检查的精确时刻。
2. 1793585002.781：global_verification_timeout；全局复核年龄 8.043 s，超过 8 s。此时轻量跟踪年龄仍约 0.343 s，匹配结果 RMSE 约 0.0951 m、overlap 约 0.9983；不能直接说看不清环境或地图无法匹配。
3. 1793585003.234：适配器收到 global_obstacle_localization_unavailable，日志明确表示停止速度并请求取消任务。
4. 1793585003.290：BT 收到客户端取消并记录 Goal canceled。这里的客户端取消与适配器请求时间吻合，不要求操作人员点击取消。

该取消链在 NX 内部发生。笔记本 Wi-Fi 抖动与源端发布/调度仍可能有间接关系，但没有故障期间的逐包和逐帧记录，不能认定无线链路是全部根因。

## 已复现的恢复状态缺口

LocalizationAdapter.set_ready 将 localization_global_verification_timeout 视为可恢复，但 on_obstacle_health 会直接采用规划器转报的 global_obstacle_localization_unavailable；后者不在 recoverable_reasons 中，导致原本的等待恢复立刻转为取消。

生产 set_ready 方法使用假发布器和假 action guard 的本机复现见 recovery_reason_repro.json。没有初始化 ROS、创建节点或调用真实服务。

修复应保留真实底层原因，使已确认的可恢复超时共享原来的有限恢复窗口。不能把所有通用 localization_unavailable 一律视为可恢复，因为它也可能来自非法坐标、歧义或其他硬故障；失效时仍必须保持零速度，恢复后仍需定位连续性及路径复核。

## 计算预算与证据边界

第二目标区间 controller 共记录 155 条 20 Hz 控制循环未达期望频率告警。后台单次复核约 3.9 秒、候选约 49 个；实时障碍又间歇过期，说明应重点检查重定位计算和队列/调度时效，而不是继续只调全局碰撞参数。

目标结束后读取的 2 秒当前观察为：8 CPU、loadavg 约 20.67、重定位约 153.5% CPU。容器无 CPU quota，允许 CPU 0-7；当时 CPU 频率约 1.984 GHz。此为之后的一次观察，不能反推第二次目标每一秒的 CPU 或归因其他容器。

地形日志本目标期间报告的 analysis_ms 约 11～29 ms，pose_wait_expired 维持启动时的 2 次；没有本次每帧点云接收/排队/消费时间，尚不能把源过期进一步定性为雷达丢帧、DDS 排队或具体锁等待。

后续应优先缩短重定位复核长尾，并补齐上述恢复原因传递。当前没有修改线上时效阈值或代码。

## 附带确认本机可视化恢复

前一项发现笔记本缺少到 192.168.123.18 的专用路由，已仅在笔记本补回：

```bash
sudo ip route add 192.168.123.18/32 via 10.56.61.177 dev wlo1
```

随后一次 12 秒只读订阅实际收到 /odom、/cloud_registered_viz 与 /offline_map/support，且 map→base_link 可查询。可视化点云是分块消息，消息数不能直接当作雷达帧率；当时运动 ready=false，原因为 navigation_input_recovery_wait，说明可收到显示数据与允许运动是不同状态。

## 文件

- summary.json：第二目标的关键指标和取消链。
- second_goal_timeline.json：按 epoch 排序的源日志及行号；显示时间为 NX 日志换算北京时间，未校正笔记本与 NX 的时钟偏差。
- raw/：已读取的节点原日志，使用本次 PID 的文件名与上述索引对应。
- runtime_load_and_params.json：当前观察及实际生效的 YAML/launch 参数覆盖。
- recovery_reason_repro.json：不使用真实 ROS 端点的逻辑复现。
