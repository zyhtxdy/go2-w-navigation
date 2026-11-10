# 本机规划拒绝诊断与近场历史修复（2026-10-14）

只在本机操作，没有连接 NX、没有真机运动测试（0 趟）。使用用户历史包 `bags/go2_inputs_20261010_094309`，调用 `ComputePathToPose`，没有启动局部控制器或运动桥。

## 已完成

1. 从规划器实际锁定的缓存记录失败原因、拒绝格、贡献点来源及输入时间，避免使用较早的 RViz 点云猜原因。起点投影和搜索分别记录；直线候选的拒绝点不等于整个绕行搜索失败的唯一原因。
2. 地形流增加 `obstacle_source` 分类字段，旧 XYZ／intensity 语义保留；只给地形输出增加 4 字节，原始 `/hesai/points` 未改。旧的无来源字段输入按 unknown 处理。
3. 将地形已有的 0.65 m 近场合成标记策略应用到稳定器和规划器历史缓存。仅清除纯合成来源；实测、混合、未知来源不因此清除。观察中心统一为底盘中心。
4. 回放暂停完成初始化后自动恢复，避免初始化时钟回跳。原始状态桥的倒退保护保留；错误日志增加具体时刻和已处理条数。退出收尾只补原包结束时间，不制造零速或其他状态。
5. 在 `fast_lio_nav2` 安装同一源导航参数的副本，离线默认入口不再依赖完整导航控制器的安装。完整 `navigation_bringup` 本机编译仍缺 `behaviortree_cpp_v3`；本次没有安装或修改系统依赖。

播放器在初始化时创建时钟，并在开始播放时跳回起始时间；单纯用墙钟延迟启动进程仍不能确保初始化阶段没有回跳。暂停启动及 `PlayNext` 等待队列准备的依据见 [rosbag2 Humble 官方实现](https://github.com/ros2/rosbag2/blob/humble/rosbag2_transport/src/rosbag2_transport/player.cpp)。正常播放模式未用于真机数据采集。

## 复现与限制

修复前的准确缓存捕获见 [replay_99.json](replay_99.json)：55 秒有 170 个安全支撑候选，但全部连接段被拒；贡献点是约 0.45 m 处的历史合成标记，不是“没有地面”。此前只看较早实测点快照，不能推出这个具体原因。

[replay_98.json](replay_98.json) 的状态回放曾中途因时钟倒退退出，只保留为失败记录，不当作完整融合验证。[initial_startup_dependency_failure.json](initial_startup_dependency_failure.json) 是早期导航配置包未安装的启动失败，不计入规划结果。

[replay_100.json](replay_100.json) 中近场清理后起点能投影，但仍搜索不连通。它尚未包含后续搜索诊断，接收状态条数也不完整，不替代最终验证。

最终默认参数回放见 [replay_101.json](replay_101.json) 与 [replay_103.json](replay_103.json)，汇总见 [summary.json](summary.json)。这些是历史包离线计算值，没有外部位置或障碍测量对照，不能当真机定位精度或到点成功率。部分较早 JSON 的 `selected_log` 包含测试结束时 SIGINT 停止节点的日志，与运行时失败分开看。

| 检查 | 第一次默认回放 | 第二次默认回放 |
| --- | --- | --- |
| 原始状态接收 | 34359/34359 条 | 34359/34359 条 |
| 时钟倒退 | 0 次 | 0 次 |
| 固定目标规划 | 2/3 成功 | 2/3 成功 |
| 成功路径 | 两条均为直线 | 两条均为直线 |
| 成功路径后续有效性 | 6/6 通过 | 6/6 通过 |
| 55 秒失败 | 直线被实测类点阻挡，搜索不连通 | 起点连接段被实测类点阻挡 |
| 后打开诊断订阅 | 收到持久标记 | 收到持久标记 |

规划 86 项、地形 26 项、回放时钟 3 项检查，共 115 项通过。已校验红格、贡献点与橙色膨胀格对应失败时的实际缓存；这些位置是内部计算值。临时观察脚本与对照参数已删除，验证进程已退出。源码清单与哈希见 [source_manifest.json](source_manifest.json)。

30 cm 邻接半径只做临时对照，见 [replay_102.json](replay_102.json)。20 cm 网格对角间距约 28.3 cm，当前 25 cm 半径确实排除这些相邻格；但该遍目标先被实测类近身点挡住，尚未进入搜索，不能证明扩大半径能解决此目标。默认仍是 0.25 m。单元检查验证了对角邻接，同时确认实际台阶与中间障碍仍会拒绝。

全局 0.48 m 膨胀半径在 0.10 m 网格中取整为 5 格；局部轮廓目前为 0.80 × 0.50 m 矩形，其外接圆半径约 0.472 m。全局圆形包络较侧向矩形保守，这本身不能证明配置错误。上述尺寸与 0.75 m 净空均为配置值，不是本次真机实测；是否可缩减需要实物尺寸和点来源核对。

## 本机使用

终端 1：启动正式回放入口，播放器会自动开始；不需要另起驱动或单独播放原始状态。

```bash
cd /home/zyhtxdy/go2-w_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch fast_lio_nav2 replay_planning.launch.py \
  bag:=/home/zyhtxdy/go2-w_ws/bags/go2_inputs_20261010_094309
```

终端 2：最好在回放开始前打开 RViz，以免错过非持久点云与动态 TF。

```bash
source /opt/ros/humble/setup.bash
source /home/zyhtxdy/go2-w_ws/install/setup.bash
export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset CYCLONEDDS_URI ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE
rviz2 -d /home/zyhtxdy/go2-w_ws/src/fast_lio_nav2/config/replay_planning.rviz \
  --ros-args -p use_sim_time:=true
```

使用 **2D Goal Pose** 向 `/goal_pose` 请求全局路径，路径为 `/plan`，不执行运动。`StartFailureDiagnostics` 已启用。蓝球为起点参考，绿色球为投影起点，黄色球为直线被拒位置，红格为碰撞格，紫球及橙格为贡献点及膨胀切片。

同样环境下读取最近诊断：

```bash
ros2 topic echo /planner/start_failure --once --qos-durability transient_local
```

新请求会清除旧标记；标记是失败时的快照，不会随之后点云自动刷新。回放从头重来需要停止并重启整个 launch，不能直接在旧建图状态上循环播放。
