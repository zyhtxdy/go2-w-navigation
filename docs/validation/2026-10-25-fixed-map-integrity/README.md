# 离线地图一致性与支撑快照验证

2026-10-22 开始实现，2026-10-25 完成本机编译与验证。只在本机操作，没有连接或同步 NX，没有真机测试。

## 已实现

1. 地图服务器保留发布 `/offline_map/identity` 和 `/offline_map/id`。身份包含地图与数据库签名、实际 XYZ 数据摘要、点数、路径和会话信息。
2. 离线入口计算预期地图 ID 与两张点云摘要，写入启动时的参数副本。旧自定义参数文件缺少新增字段时也补入；原文件保持不变。纯数字 SHA-256 保持字符串类型。
3. 离线规划器校验地图 ID、坐标系、消息布局、数据摘要及实际订阅话题的发布者。相同内容重发忽略时间戳，不重复加载；不同内容或重复来源锁定故障并拒绝混入，拒绝新规划与路径有效性检查。话题重映射可用。
4. 离线 `/planner/support_map` 使用 Reliable、Transient Local、深度 1；激活时补发已加载的快照。离线 RViz 配置增加默认关闭的 `PlannerSupportMap` 显示项。
5. 修复空障碍点云构造迭代器的越界风险，以及 cleanup/configure 后一次性地图输入可能因旧停止标志丢失的问题。

在线配置默认关闭身份检查；在线支撑快照保持原来的 Best Effort、Volatile 和一秒限频。搜索、支撑覆盖与碰撞阈值没有调整。

## 验证结果

| 项目 | 结果 |
| --- | --- |
| 本机编译 | astar_3d_planner、navigation_bringup、go2_offline_navigation、fast_lio_nav2 通过 |
| 规划器自动化测试 | 150 项通过 |
| 离线导航自动化测试 | 75 项通过 |
| 总计 | 225 项通过，零失败、零错误、零跳过 |
| 源码与本机安装文件 | 10 组字节一致 |
| 测试进程清理 | 无剩余测试进程 |

测试数量取 JUnit 中的实际测试用例数，不重复计入 CTest 包装项。详见 `verification.json` 和 `test_results/`。

新增覆盖包括：标准 SHA-256 向量、无效点云与空障碍、同图重发、错图锁定、重复发布者、跨话题来源冲突、并发回调、地图 ID 晚于点云到达、晚加入订阅、节点重新激活、cleanup/configure 后保留地图重载、ROS 话题重映射、旧自定义 YAML 与纯数字摘要。

原有完整离线入口测试也通过：在私有本机域中使用合成输入启动重定位、地图服务器、Nav2 并得到软件控制器输出；没有运动桥接或 `/api/sport/request` 发布者。这不代表真实机器狗已完成导航验收。

## 查看结果

正常离线入口自动启用新检查，不需要额外添加启动参数。重新加载本机安装环境后可查看：

```bash
source /opt/ros/humble/setup.bash
source /home/zyhtxdy/go2-w_ws/install/setup.bash
ros2 topic echo /offline_map/identity --once --qos-durability transient_local
ros2 topic echo /planner/map_status --once --qos-durability transient_local
```

`/planner/map_status` 含地图 ID、预期与收到的摘要、点数、接受/重复/拒绝计数和发布者信息。
其 `ready` 仅表示输入认证通过，规划器还检查地图实际加载和定位状态。
`/offline_map/ready` 仍是地图服务器心跳，不等于规划器认证通过。
身份冲突不自动清除；解决来源冲突后重新启动离线链路。发布者暂缺为可恢复的等待状态。
这项输入检查不直接修改速度门控，不等同于即时急停。

## 日志

- `build.log`：首次完整编译。
- `build_empty_cloud_fix.log`：空点云修复后的编译。
- `install_config.log`：RViz 与本机配置安装。
- `test.log`：两包完整回归。
- `build_legacy_config_fix.log`、`test_legacy_config_fix.log`：旧参数文件兼容修改后的编译及受影响测试复验。

原来观察到固定支撑点变化的现场原因仍未证实。本次增加了运行时核验与诊断，不能将软件检查通过等同于已定位此前现场根因。
