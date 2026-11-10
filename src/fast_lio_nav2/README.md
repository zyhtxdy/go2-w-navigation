# 本机录包建图与全局规划

已知地图导航另有独立入口，见 [go2_offline_navigation](../go2_offline_navigation/README.md)。
`odom_fusion.launch.py` 新增 `publish_map_odom_tf`，默认 true，保持原在线行为。
仅在已知地图定位接管 map→odom 时设为 false；原录包回放命令不需要改。

## 启动静止与输入坐标系（2026-10-29）

整套建图仍应在机器人静止时启动：FAST-LIO 自己也要初始化重力和陀螺零偏。
融合端 `/imu/corrected` 现在等待连续静止窗口，不再直接平均启动后的前 400 条数据。
未通过窗口时原样发布角速度；运动、数据间断或时间回退会重置尚未完成的窗口。
窗口通过后才减去零偏，不会在后续正常转弯时继续估计并扣除真实转速。

`odom_fusion.launch.py` 的初始阈值如下，也可用同名 launch 参数覆盖。
它们是软件初值，不是本体 IMU 的实测标定结果。

| 参数 | 初值 | 含义 |
| --- | --- | --- |
| `calibration_samples` | 400 | 窗口至少包含的连续样本数 |
| `calibration_minimum_duration` | 1.0 s | 窗口至少持续的源时间 |
| `calibration_maximum_gyro_norm` | 0.05 rad/s | 每条样本的角速度模长上限 |
| `calibration_maximum_axis_variance` | 0.0002 (rad/s)² | 窗口每轴角速度方差上限 |
| `calibration_maximum_orientation_change` | 0.01 rad | 相对窗口首帧的姿态变化上限 |
| `calibration_maximum_sample_gap` | 0.1 s | 两条样本允许的最大时间间隔 |

Go2 桥接的启动入口默认打开已有的姿态输出，`go2_imu_publish_orientation:=false`
可关闭它。此四元数只为静止窗口提供相对姿态检查；FAST-LIO 仍只使用加速度和
角速度，EKF 仍只融合 IMU 的偏航角速度，不融合此姿态。
已有旧包的 `/go2/imu` 标记姿态不可用，不会凭空补出姿态。此时退回角速度阈值和
方差检查，并提示启动必须静止；仅凭陀螺无法区分任意慢的匀速转动与零偏。
即使姿态有效，低于姿态变化阈值的运动也不能保证被识别。

`lio_base_odom_node` 只接受配置的 `odom → body` 输入。
不同坐标系的数据会被拒绝，不再仅警告后重新贴上 `odom → base_link` 标签。
此检查能挡住错误来源的坐标解释，不能解决其他节点重复发布同名 TF 的冲突。

`replay_planning.launch.py` 回放雷达、本体 IMU 和运动状态桥接输出，启动
FAST-LIO、里程计融合、地形处理和实际 3D A* 全局规划服务。
同时启动 RViz 目标接口，将 `/goal_pose` 转成计算路径请求。
原始狗状态由只读录包桥重新生成机身线速度，避免使用旧的方向错误输出。
不启动实时驱动、实时状态订阅、局部控制器或运动指令桥接。
节点使用本机通信和独立 ROS 域 77；回放结束后保留计算出的支撑地图供规划检查。

## 融合时间与角速度处理（2026-10-13）

激光定位计算后才发布，但保留扫描结束的测量时间。EKF 已开启
`smooth_lagged_data: true`、`history_length: 1.0`，回到测量时刻更新后
重放后续 IMU／速度输入；`permit_corrected_publication: false` 保持输出
时间戳向前。此历史窗口是当前录包验证值，不是所有设备的延迟保证。

原 `imu0_twist_rejection_threshold: 3.0` 在正常角速度变化时也会拒绝更新，
造成角速度与朝向滞后；已移除，使用 robot_localization 默认的角速度更新。
输入节点仍检查非有限角速度，激光定位的 5σ 门限与桥接节点平移跳变检查
保留；没有修改协方差、运动桥接、限速或加入 PID。

受控迟到输入与真实录包的对照、结果和局限见
融合修复记录（历史附件已在本机归档）。

本机需安装规划和融合运行依赖；本次已安装缺少的 `nav2_planner` 和
`robot_localization`。其他电脑可先安装：

```bash
sudo apt install ros-humble-nav2-planner ros-humble-nav2-lifecycle-manager \
  ros-humble-nav2-common ros-humble-robot-localization ros-humble-rosbag2-storage-mcap
```

先编译当前源码，避免安装目录使用旧外参或旧地形、规划代码：

```bash
cd /home/zyhtxdy/go2-w_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to fast_lio_nav2 --executor sequential
source install/setup.bash
```

rosbag 播放器只发布 `/hesai/points`、`/go2/imu`。
只读 `go2_odom_bag_bridge` 从同一个包直接读取 `/lf/sportmodestate`，
按回放时钟重新生成 `velocity_frame=body` 的 `/go2/odom_raw`。
旧包中已经录下的 `/go2/odom_raw` 含额外坐标旋转错误，不再播放。
原始状态消息时间戳全零，离线输出使用原记录接收时间，仍是采样时间近似。
直接读取还避开了该旧原始消息在 FastDDS 直接序列化发布时的失败；原包不修改。

播放器以暂停状态启动，正式 `replay_starter.py` 节点等第一条队列消息准备好后自动恢复，避免初始化期间时钟先推进再回跳。原始状态桥仍严格拒绝回放时钟倒退。播放器正常退出后，启动入口允许该辅助节点补齐原包结束时刻，时间不会低于已观察时钟，不生成任何虚构状态。

离线默认参数是 `navigation_bringup/config/nav2_mppi_params.yaml` 同一源文件在本包安装时的副本，不需要为离线计算路径编译完整导航控制器。修改该源配置后，应重新编译本包；真机导航安装配置则重新编译 `navigation_bringup`。`params_file` 仍允许显式指定其他文件。

`/planner/start_failure` 保存最近失败的 JSON，配套 MarkerArray 已加入 RViz 配置。来源及近场历史修复见 [规划器说明](../nav2/astar_3d_planner/README.md)，本轮结果见 2026-10-14 本机记录（历史附件已在本机归档）。
因此当前入口要求包里有原始 `/lf/sportmodestate`，只有旧桥接输出的包不能
靠改变 launch 参数修正。`/lowstate` 继续保留用于单独检查 IMU 桥接。

终端 1：先打开 RViz。

```bash
source /opt/ros/humble/setup.bash
source /home/zyhtxdy/go2-w_ws/install/setup.bash
export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset CYCLONEDDS_URI ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE
rviz2 -d /home/zyhtxdy/go2-w_ws/src/fast_lio_nav2/config/replay_planning.rviz \
  --ros-args -p use_sim_time:=true
```

终端 2：再启动回放与规划。当前本机代码已经编译，无需重复编译。

```bash
cd /home/zyhtxdy/go2-w_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset CYCLONEDDS_URI ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE

ros2 launch fast_lio_nav2 replay_planning.launch.py \
  bag:=/home/zyhtxdy/go2-w_ws/bags/go2_inputs_20261010_094309 \
  params_file:=/home/zyhtxdy/go2-w_ws/src/nav2/navigation_bringup/config/nav2_mppi_params.yaml
```

此配置已设置 Fixed Frame 为 `map`，PointCloud2 使用 Best Effort。
显示 RegisteredMap（`/cloud_registered`）、SupportHistory（累计显示
`/terrain/support`）、CurrentGlobalObstacles（当前 `/terrain/obstacles_global`），
以及绿色 GlobalPlan（Path `/plan`，Reliable、Volatile）。
显示累计支撑只是 RViz 效果，规划器另在自己的内存中累计支撑。

先打开 RViz，再在另一个终端启动上述回放，等待包完整播放一次，保留 launch 运行。
点云与动态 TF 不补发历史消息，回放结束才打开 RViz 会错过地图与位姿显示。
点击工具栏 **2D Goal Pose**，在已观测的地面区域按下并拖动鼠标指定朝向，松开后请求规划。
Tool Properties 中该工具的 Topic 应为 `/goal_pose`。
本机规划终端出现 `Path ready on /plan` 后，绿色 GlobalPlan 显示本次全局路径。
`/path` 是 FAST-LIO 运动轨迹，不是全局规划路径。
先打开 Path 显示再下目标：`/plan` 不补发旧消息，后来加显示时需要重新点击目标。
录包结束后当前位置保持在最后一帧，每次点击都是从这个位置重新规划。

也可在同样设置环境的终端计算一个已验证的返回路径：

```bash
ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose \
  "{goal: {header: {frame_id: map}, pose: {position: {x: -2.19, y: -0.79, z: 0.0}, orientation: {w: 1.0}}}, planner_id: GridBased, use_start: false}"
```

这是计算路径请求，不是机器人到点任务。此启动文件没有 `/navigate_to_pose`
任务执行器；使用标准 **2D Goal Pose**，不要使用 **Nav2 Goal** 或 Navigation2 面板。
若规划失败，终端会提示 `No new path` 和具体规划错误；RViz 可能保留上一条成功路径，
它不代表新目标规划成功。
重新建图时停止 launch 并从头启动，避免在旧 FAST-LIO 状态上循环回放。

有支撑仍可能因障碍及膨胀范围被拒绝；起点盲区、坡度、台阶和净空检查沿用
真机规划参数。此录包验证只能说明这些输入上的离线规划结果，不能代替
机器狗响应速度指令、到点或窄通道倒退测试。

2026-10-10 本机验证：完整回放上述录包一次，通过实际规划服务请求 17 个目标，
13 次返回路径，4 次拒绝。拒绝项包括地图外目标、障碍目标，以及两个附近有
障碍回波、没有安全落点的历史轨迹目标。并非机器人曾经过的每个点都适合作为
当前目标；当前障碍及膨胀范围仍会参与判定。

成功路径均从请求起点开始，水平点间距不超过 0.05 m；需要投影的目标
终点偏移不超过配置的 0.20 m。以上是录包离线计算值，没有外部测量对照。
本次运行的 8 项禾赛预处理、18 项地形和 43 项规划／插件测试全部通过。
另一次完整回放验证了上述命令：`use_start: false` 从 TF 自动获取起点，
计算路径请求返回 `SUCCEEDED`。
FAST-LIO 的本机安装配置已更新为源码中的外参；本次没有修改 A* 搜索规则、
降低障碍膨胀或把未知区域当作可走区域。雷达外参精度和真机建图漂移仍需另行验证。
