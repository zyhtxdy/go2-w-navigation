# 2026-11-02 离线导航时效预算与恢复原因修复

已在本机修改、编译、回归并同步本人 `zyh-nav2:/work/zyh_ws/src`。本次只同步源码；没有在 NX 编译或启停节点，没有发送真机目标、速度或运动请求，未操作其他容器。SSH 已断开。正在运行的 NX 程序仍使用先前版本。

## 采用的预算

| 项目 | 修改前 | 修改后 |
| --- | ---: | ---: |
| 全局障碍输入接收、成功消费、处理进度期限 | 1 s | 2 s |
| 全局障碍状态心跳期限 | 1 s | 1 s |
| 重定位后台结果接收预算 | 4 s | 6 s |
| 两次有效全局复核最大间隔 | 8 s | 12 s |
| 当前扫描输入预算／当前定位跟踪超时 | 3 s | 3 s |
| 局部 costmap 两路观测期限 | 0.8 s | 0.8 s |
| 适配器局部地形期限／里程计期限 | 1 s／0.5 s | 1 s／0.5 s |
| 桥接速度超时／输入恢复等待 | 0.3 s／3 s | 0.3 s／3 s |
| 实测动态障碍逐点保留时间 | 0.6 s | 0.6 s |

这些是根据本次 NX 约 3.94 s、后续约 5 s 的计算观察制定的软件测试初值，尚未进行采用新配置后的真机运动验证。12 s 是全局复核的期限，不能代替当前扫描匹配、当前里程计和当前 TF，也没有把旧结果时间戳改成现在。

## 贯穿整个输入链路

新增 `live_obstacle_freshness_timeout`，启动默认 2.0 s。C++ 接收门限、转换后复查、提交前复查及消费健康检查使用同一参数。Python 适配器将数据/进度期限与 1 s 心跳期限分开。

离线 launch 将同一值传给规划参数重写与适配器；旧自定义 YAML 中缺少新字段也会补齐。回放检查入口增加同名命令行选项并对齐两端。可在启动时覆盖：

```bash
live_obstacle_freshness_timeout:=2.0
```

参数由启动读取，不是此次正在运行进程的热更新。

## 恢复原因修复

规划器的 `global_obstacle_localization_unavailable` 是适配器定位状态的反馈。现在在共同的 set_ready 路径中查询实际定位原因：

- 已知可恢复的定位复核／跟踪超时保留原来的有限等待，不再被通用反馈直接取消。
- 定位重新确认完成但规划器还暂报旧 unavailable 时，仅已有的零速度等待可以继续；等待起点不刷新。
- 必须收到新鲜规划器状态、通过定位连续性与当前路径复核后才恢复输出。
- 原因不明的初次失效、歧义、非法帧、里程计不一致等仍不能获得这项等待特例。
- 3 s 总恢复期限不延长，硬故障仍立即取消。

## 验证

本机五包编译通过：astar_3d_planner、navigation_bringup、fast_lio_nav2、go2_offline_navigation、go2_relocalization。

| 回归包 | 通过 |
| --- | ---: |
| astar_3d_planner | 426 |
| go2_offline_navigation | 319 |
| navigation_bringup | 12 |
| go2_relocalization | 38 |
| 合计 | 795 |

失败和跳过均为 0。首次全回归找到两处旧测试夹具需要随新契约更新：公开路径测试仍假设 1.15 s 必须过期；手工创建的 LaunchContext 没有新声明的参数。前者已扩展为 1.15 s 仍有效、2.15 s 必须过期的双边界测试，后者补上明确的 2.0 s 上下文；对应两组重新编译／执行通过。保留初次及重测日志，最终 XML 在 final_test_results/。

另一次原录包完整执行链检查在 localhost 域 196 运行，运动 API 桥接关闭，11/11 项通过。覆盖下目标、生成路径、控制与平滑及门控输出贯通、主动取消、空闲状态断流、活动任务地形断流后局部 costmap 报旧并停止非零输出。它不是闭环真机到点或停车距离验证。FAST-LIO 原调试日志已恢复。

## NX 同步

更新并独立重读校验 20 个源码、配置与测试文件。只修改 NX 导航配置中的新全局期限字段，以及定位 YAML 的 6 s／12 s 两项；保留其他现场参数，包括局部膨胀半径 0.25 m。已有文件 UID/GID 和权限保持原值。

容器内备份：`/root/zyh_sync_backups/timing-budget-20261102-69ea0338`。包含 previous-source.tar.gz、manifest.json、sync_receipt.json、source_verification.json。

build/install/log/maps 的 9087 个条目元数据前后一致。没有修改 DDS、宿主机网络、其他容器或运行中的参数。源文件与实际运行设置的区别必须保留：本次没有执行下面的编译命令。

## 后续生效

先结束本人当前的导航 launch，再在本人容器内编译，随后重新启动原离线 launch：

```bash
cd /work/zyh_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export MAKEFLAGS='-j2 -l2'
colcon build --packages-select go2_relocalization astar_3d_planner navigation_bringup fast_lio_nav2 go2_offline_navigation --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source install/setup.bash
```

不需要为这次修复重建或替换地图。当前严格的匹配质量、歧义、车身碰撞、局部实时障碍和实际定位数据时效检查继续保留。
