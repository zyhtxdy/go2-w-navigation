# 全局实时障碍消费超时保护

2026-10-26。本轮仅在本机修改、编译和测试，未连接或同步 NX，未运行真机控制。

## 已完成的行为

离线规划器记录最后一次**成功转换并写入内部地图**的实时障碍点云时间。
源时间年龄、成功处理的稳态时间年龄均采用 1 秒阈值；实际检测另受线程调度、100ms 健康发布周期、
适配器 50ms 检查周期及通信延迟影响，不是严格的实时停车期限。

- 新鲜的空点云是正常更新，可以清除动态占用。
- 重复或倒序的旧时间戳、未来时间戳、转换失败、无效数据及已失效的地图 epoch 不续期。
- 即使 ROS 回放时钟暂停，成功处理时间也会按稳态时钟过期。
- 超时后拒绝新全局规划，路径有效性服务也返回无效；保留最后一帧占用，不把断流当作空场景。
- 适配器失去就绪后输出零速，并通过现有 NavigationGuard 取消活动导航任务。
- 数据恢复后仍须完成旧任务取消及输入安静期；原任务不会自动续跑，需要重新下发目标。

此前的 0.6 秒参数控制障碍点的历史保留；本轮的 1 秒阈值检查数据链路是否持续更新，两者用途不同。
机器人外廓、碰撞距离、支撑提取和正常路径搜索规则没有放宽。
在线入口不启用本轮离线健康门控；共享点云解码额外拒绝“空尺寸但带载荷”的异常数据，空云也须有有效坐标变换。

## 启动与恢复依赖

| 话题 | 含义 |
| --- | --- |
| /offline_localization/localization_available | 定位和里程计有效，允许处理实时障碍并显示 map TF |
| /planner/live_obstacle_status | 规划器成功消费障碍的健康心跳，每约 100ms 一次 |
| /offline_localization/ready | 最终导航准入：包含定位、地图、本地地形、全局障碍消费与取消保护 |

规划器使用 localization_available 处理输入，不依赖最终 ready，因此不会形成
“等 ready 才处理障碍、等障碍才 ready”的启动循环。
最终 ready=false 不会在只有障碍断流时清掉有效的定位 TF。
定位失效或时钟回退会使旧输入 epoch 失效，已在转换途中的旧点云不能恢复旧状态。

原有离线启动命令保持。只启动定位、不启动规划器时，最终 ready=false 属于预期，已验证的地图 TF 仍可显示。

## 验证结果

本轮两个修改包共 **385 项测试通过，无失败、无跳过**：

| 包 | 测试数 |
| --- | ---: |
| astar_3d_planner | 174 |
| go2_offline_navigation | 211 |

包含实际规划器的单路断流、重复旧戳、坐标转换失败、空快照恢复、拒绝规划及路径有效性检查；
还覆盖单独全局故障时的零速、取消未完成前禁止恢复、ROS 时钟暂停、旧 epoch、未知状态及无效状态。
现有在线相关几何测试和离线完整软件控制链路测试也通过；这些运行于本机隔离域，不存在机器人 API 发布者。

新增并发测试执行 20000 次健康快照读取，并与另一线程的成功更新并行，验证不会把正常更新误判成未来处理时间。

最终完整录包回放：**18/18 次普通规划请求生成有效路径，3/3 次地图外负对照被拒绝**。
没有超时、跳过、挂起或新增健康保护导致的规划拒绝。正常数据期间就绪链路保持可用；录包结束后的里程计超时属于预期。
本轮没有启动回放控制器、行为树或速度桥接，物理 API 发布者为 0。

输入为 bags/go2_inputs_20261010_094309、maps/lab_v1_database、maps/lab_v1_navigation_guarded_20261025，
目标清单沿用上轮 guarded_cases.yaml。结果只代表这一段录包中的软件请求，不能算成真机到点成功率。
录包持续播放，实际起点与障碍窗口随时间变化，不能当成冻结输入的严格 A/B。

## 中间发现与修复

第一版回放出现 14/18 次普通规划成功，其中 4 次误拒绝来自新增检查的并发时序问题。
检查线程在加锁前取得“当前时间”，随后另一线程成功写入更晚的处理时间，健康检查因而误判。
现已在同一把锁内读取状态并采样两种时钟，健康报告的 ready、源时间与处理年龄来自同一快照；
定位心跳也先读取接收时间，再采样当前时间。没有放宽超时阈值。
该中间轮保留在 replay_final/，最终采用 replay_atomic_health/。

部分旧集成测试仅等待固定时长就查询规划，未确认新帧已成功入图。
测试现已等待真实规划器健康报告确认近期消费，并在拒绝时保留服务端日志；没有绕过健康检查。
最终完整测试见 tests_atomic_final.log，中间失败日志保留用于审计。

## 查看状态

在运行该离线链路、已设置正确 ROS 环境的终端中：

```bash
ros2 topic echo /planner/live_obstacle_status --once --qos-durability transient_local
ros2 topic echo /offline_localization/localization_available --once --qos-durability transient_local
ros2 topic echo /offline_localization/ready --once --qos-durability transient_local
ros2 topic echo /offline_localization/status --once --qos-durability transient_local
```

健康 JSON 包含 ready、reason、consumed_stamp_ns、processed_age_s、successful_updates。
这些命令只读；本轮未在 NX 上执行。

## 文件

- [最终逐项规划报告](replay_atomic_health/planning_report.json)
- [汇总与中间轮结果](summary.json)
- [测试明细](test_counts.json)
- [最终测试日志](tests_atomic_final.log)
- [代码安装一致性与收尾](verification.json)

本机源码已编译安装。地图、定位数据库和原有 FAST-LIO 源码日志均校验保持；NX 尚未同步或编译。
