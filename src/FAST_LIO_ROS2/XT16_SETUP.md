# Go2W / 禾赛 XT16 接入

当前包含 XT16 雷达驱动接入、FAST-LIO 点云解析、本体 IMU 和运动状态桥接源码。
已清理 Scout 底盘、轮式里程计、旧 CAN 配置和 Livox 驱动及专用解析。
融合配置使用 FAST-LIO 位姿、本体运动状态的平面速度和本体 IMU 角速度；机身 TF、时间同步与导航尺寸
仍需实测，不能把源码准备完成当作 Go2 真机融合已经可用。

## 在 Orin NX 的 Ubuntu 22.04 / ROS 2 Humble 容器内编译

使用 ARM64 容器，在 NX 内编译源码。Docker 使用 `--network host`，并在宿主机
配置能与 XT16 通信的网卡。不要搬用电脑的 `build/`、`install/` 或 SDK 二进制。

禾赛官方驱动已经放在 `src/driver/HesaiLidar_ROS_2.0`，包含 SDK 子模块。
若通过 git 重新获取驱动，需要 `--recurse-submodules`。

```bash
source /opt/ros/humble/setup.bash
sudo apt-get install libboost-all-dev libyaml-cpp-dev libssl-dev libpcl-dev \
  libeigen3-dev python3-dev python3-yaml ros-humble-pcl-ros \
  ros-humble-pcl-conversions ros-humble-ament-cmake-gtest
colcon build --packages-select hesai_ros_driver fast_lio --parallel-workers 2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DFIND_CUDA=OFF
source install/setup.bash
```

当前源码不再包含 Livox 驱动或 Livox-SDK2 编译依赖。
只需要禾赛驱动时，可用 `--packages-select hesai_ros_driver`。

## 两份配置各自的用途

- `config/hesai_xt16.yaml`：禾赛驱动的网络/点云配置。当前设备 IP 为
  `192.168.123.20`，UDP 接收端口为 `2368`。2026-10-08 NX 抓包显示数据从
  `192.168.123.20:10000` 发往 `192.168.123.18:2368`；驱动解码仍待验证。
  更换设备或网络后须重新核对。此配置不会修改雷达转速。
  校准文件是 SDK 示例，优先换成本机序列号对应文件。
- `config/xt16.yaml`：FAST-LIO 参数。`lidar_type=5` 读取禾赛的原生
  `x/y/z/intensity/ring/timestamp`；`scan_line=16`，`timestamp_unit=0`。
  默认按 10 Hz（600 rpm）接收，实际转速改变时同步调整 `scan_rate`。

XT16 的 `timestamp` 是绝对秒数。本实现将它减去点云 header 的帧起始时间，
再转为 FAST-LIO 使用的相对毫秒，避免将 epoch 秒数截断为 float。
输出按逐点时间排序，降采样保留最后一个点。字段缺失、非有限坐标、越界 ring、
错误时间基准和损坏数据均经过检查，不会用零时间代替缺失的真实时间。
当前 XT16 分支要求 `feature_extract_enable=false`，直接用于 FAST-LIO2。

## 先验证雷达

从工作空间根目录启动，只运行驱动，不运行 FAST-LIO 或机身 TF：

```bash
ros2 launch fast_lio mapping.launch.py start_mapping:=false \
  publish_map_odom_tf:=false
```

在另一个已 source 的终端检查：

```bash
ros2 topic hz /hesai/points
ros2 topic echo /hesai/points --field fields --once
ros2 topic echo /hesai/points --field header --once
```

自定义网络配置可以复制 `hesai_xt16.yaml` 并通过
`hesai_config_file:=/绝对路径/hesai_xt16.yaml` 指定。复制时一起复制
`calibration/`，或把校准文件路径改成绝对路径。

## 接入实际 IMU 与外参后建图

默认 IMU 输入已改为 `/go2/imu`，由新增的 `go2_imu_bridge` 从 `/lowstate`
的 `unitree_go/msg/LowState.imu_state` 提取狗本体 IMU，再发布为
`sensor_msgs/msg/Imu`，坐标系名称为 `go2_imu`。它不是雷达内置的 `/utlidar/imu`。
桥接只读状态、发布 IMU，不发布运动指令或物理 TF。
需要确认角速度（rad/s）、线加速度（m/s²）、频率、坐标方向和与 XT16 共用的时间基准。
`LowState` 没有 ROS `Header`；本次桥接使用回调接收时的 ROS 时间，
不解释 `tick`，**没有完成严格的采样时间同步**。默认不提供姿态估计，
`orientation_covariance[0]=-1`；陀螺仪和加速度协方差为零，表示未知。
2026-10-09 NX 静止检查发现设备时间停在 2020 年，当前配置改为
`use_timestamp_type=1`，使用 NX UDP 接收时间作为初期调试近似。
禾赛 SDK 同时生成帧头和逐点时间；没有只替换 header 后保留旧逐点时间。
本体桥也使用 NX 接收时间，二者仍有传输和处理时延，不等于严格采样同步。
后续移动建图和外参标定仍需根据设备配置核对 GPS/PTP 或经过验证的时间转换。

## 本体 IMU 源码与后续手动编译

`src/driver/unitree_go` 包含固定版本的官方完整消息定义，
`src/driver/go2_imu_bridge` 包含 IMU 转换节点，
`src/driver/go2_odom_bridge` 包含运动状态速度转换节点。
无需运动控制 SDK，也无需另行编译 Cyclone DDS。
本次在 NX 上只同步源码；下面的编译和运行命令由使用者之后手动执行。

```bash
cd /work/zyh_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to go2_imu_bridge go2_odom_bridge
colcon build --packages-select hesai_ros_driver fast_lio --parallel-workers 2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DFIND_CUDA=OFF
source install/setup.bash
```

桥接不依赖底盘控制驱动。消息定义与当前机器人固件的
兼容性尚未真机验证。配置或 C++ 默认值变更之后，需要由使用者编译、安装对应包。

桥接默认不会跟随 `mapping.launch.py` 自动启动。后续先单独检查本体 IMU：

```bash
export ROS_DOMAIN_ID=0
unset CYCLONEDDS_URI
ros2 launch fast_lio mapping.launch.py \
  start_mapping:=false lidar_driver:=none start_go2_imu_bridge:=true \
  publish_map_odom_tf:=false
```

此命令只启动 IMU 桥接，不启动雷达驱动、建图算法或 TF。默认订阅 `/lowstate`；
如需低频状态流，指定 `lowstate_topic:=/lf/lowstate`，实际频率仍需测量。
也可独立运行 `ros2 launch go2_imu_bridge lowstate_imu.launch.py`。

在另一个终端检查 `/go2/imu` 的消息类型、频率、时间戳和单位，且确认静止、
转动时响应符合本体 IMU。完成时间和外参核对后再进入下面的建图步骤。

## 使用本体 IMU 建图

如果更换 IMU，可修改 `common.imu_topic` 或使用启动参数 `imu_topic`。
将已标定的 `common.time_offset_lidar_to_imu` 填入
`xt16.yaml`。`mapping.extrinsic_T/R` 表示 **T_IMU_LIDAR**，即从雷达系到使用的
IMU 系的变换。当前平移参考初值为 `[0.1710, 0, 0.0908]` 米，旋转为单位矩阵，
依据用户提供的 Go2 SLAM 资料中“内部 IMU 中心”的描述和 XT16 完整矩阵。
不能再减一次 `base -> IMU` 平移。该资料为第三方整理，Go2W 支架与当前点云轴向
尚未真机核实；使用前须检查，取得本机原厂标定或实测结果后覆盖。
来源及方向见 [安装外参说明](config/calibration/README.md)。

```bash
ros2 launch fast_lio mapping.launch.py start_go2_imu_bridge:=true imu_topic:=/go2/imu
```

也可用 `config_path:=/自定义配置目录 config_file:=xt16.yaml` 加载自己的参数。
默认发布 `/Odometry`（`odom -> body`）以及 `/cloud_registered_body`，其中 `body`
代表所用 IMU 坐标系。默认不发布 `body -> base_link` 安装外参。
本工作空间的 `base_link` 按宇树官方 Go2W 模型的机身 `base` 定义，不是地面投影帧。
官方模型的 `base -> IMU` 平移为 `[-0.02557, 0, 0.04232]` 米，旋转为零；
因此启动参数中 `body -> base_link` 的名义默认值为 `[0.02557, 0, -0.04232]` 米。
若需要这条 TF，通过 `publish_body_base_tf:=true` 显式启用。
实际标定结果、不同 IMU 或不同基准帧用 `body_to_base_x/y/z/roll/pitch/yaw` 覆盖，
并避免与其他 TF 发布者重复。这些模型值不能当作当前机器人已实测标定。

若驱动已由其他程序启动或回放 rosbag：

```bash
ros2 launch fast_lio mapping.launch.py lidar_driver:=none \
  lidar_topic:=/hesai/points imu_topic:=/go2/imu
```

回放时再加 `use_sim_time:=true` 并使用 rosbag 的时钟。包里已有 `/go2/imu`
就不要启桥接；只有 `/lowstate` 时可显式启桥接，但仍只是回放接收时间近似。

融合启动文件的 `imu_topic` 默认 `/go2/imu`，同时传给 FAST-LIO
和陀螺零偏校正节点。FAST-LIO 使用原始 IMU；EKF 继续订阅校正后的
`/imu/corrected`。校正节点保留原始时间戳和坐标系，不负责坐标变换。
融合启动文件默认启动两个状态桥，并转发 `lowstate_topic` 和 `go2_imu_frame_id`。
融合启动文件已删除 Scout 轮式里程计输入和旧静态安装外参；替代输入为下面的
本体运动状态速度。融合启动默认发布 `base_link -> body` 和
`base_link -> go2_imu`，各为 `[-0.02557, 0, 0.04232]` 米、零旋转。
两条变换共享 `base_to_imu_x/y/z/roll/pitch/yaw` 参数，采用宇树 Go2W 模型名义值，
不是当前机器的实测标定；如果使用其他安装或基准帧，需要覆盖这些参数。
当 `go2_imu_frame_id:=body` 时只发布一条，避免同一子帧重复发布。
若机器人模型或其他程序已提供这两条 TF，设置 `publish_body_imu_tf:=false`。
融合启动关闭独立建图的 `body -> base_link` 发布，以避免与 EKF 的 `odom -> base_link`
争用同一个子帧；同时关闭 FAST-LIO 的 `odom -> body`，由 EKF 发布 `odom -> base_link`。
运动状态速度帧 `go2_body` 现按官方 Go2W 仿真实现采用 IMU 中心作为参考初值，
默认发布相同的 `base_link -> go2_body`，共享 `base_to_imu_*` 参数。
这是有来源的模型约定，当前真机固件是否一致仍未验证。
`sportstate_reference_point:=base` 只用于已确认速度代表 base 原点的情况；
使用外部标定 TF 或回放包已有该 TF 时设为 `sportstate_reference_point:=external`。
核验依据和步骤见 [GO2_STATE_FRAME.md](../fast_lio_nav2/GO2_STATE_FRAME.md)。
如果后续需要融合包，可以手动使用 `colcon build --packages-up-to fast_lio_nav2`。
当前导航配置中的轮廓和高度仍未作 Go2 实测标定，使用前须替换。

## 接入 `/sportmodestate` 本体速度

新增 `go2_odom_bridge` 读取 `unitree_go/msg/SportModeState`，输出
`/go2/odom_raw`（`nav_msgs/msg/Odometry`）。这是厂家提供的运动状态估计，
不是原始轮编码器数据；本次不积分速度生成位姿，也不发布 TF 或运动指令。

桥接默认按官方示例将 `velocity` 解释为 Go2 原生里程计系速度。
使用同条消息的 wxyz 四元数执行 `Rᵀ × velocity`，得到机身坐标下的速度。
**该计算假设四元数表示机身到上述里程计系的姿态，当前 Go2W 上尚未核实。**
四元数或速度无效时丢弃样本；不会把错误姿态替换成单位姿态继续发布。
只有实测确认固件直接提供机身速度后，才使用 `velocity_frame:=body`。
依据：[Unitree 官方读取示例](https://github.com/unitreerobotics/unitree_ros2/blob/master/example/src/src/read_motion_state.cpp)。

输出提供转换后的 `twist.linear.x/y/z`。位姿和角速度是带大协方差的占位，
不得拿 `/go2/odom_raw` 的位姿判断位置。EKF 的 `odom1` 只融合 x/y 线速度；
LIO 继续提供 x/y/yaw 位姿，`/imu/corrected` 继续提供 yaw 角速度。
厂家运动状态、LIO 和 IMU 可能共享本体 IMU，当前没有资料证明它们相互独立。

`child_frame_id` 默认 `go2_body`，独立桥接不发布 TF；统一融合 launch 发布上述模型参考 TF。
确认二者确实为同一坐标系和同一原点后，才可将 `child_frame_id:=base_link`
（融合启动中同时设 `go2_body_frame_id:=base_link sportstate_reference_point:=base`）。
不能仅为了消除 TF 报错改名字。
`header.frame_id=go2_state_odom` 仅标记厂家的原生参考系，桥接不会把它接到地图 TF 树；
EKF 只消费 twist，不消费此消息位姿。

默认时间戳来自 ROS 回调接收时间，是近似值。显式 `timestamp_source:=message`
才使用原始 `stamp.sec/nanosec`，使用前必须核实与 LIO/ROS 时间同钟；本次不猜时间偏移。
回放时使用 `use_sim_time:=true` 并发布 `/clock`，接收时间仍是回放处理时刻的近似值。
速度标准差初值 `linear_velocity_stddev=0.2` m/s 只用于初始调参，**不是实测噪声**。
输入 Best Effort，输出 Reliable，均为 Volatile、Keep Last 20。
输入中断时不重复发布旧速度，也不伪造零速度；EKF 超时预测不等于失联急停。

之后由使用者单独运行检查（本次同步不会执行）：

```bash
ros2 launch go2_odom_bridge sportmodestate_odom.launch.py
ros2 topic echo /go2/odom_raw --once --field twist
```

先检查输入类型与频率、静止和前后/左右运动的符号、转向后的速度轴、四元数参考系
及时间关系。桥接的详细参数和离线测试见
[go2_odom_bridge README](../driver/go2_odom_bridge/README.md)。
回放包已有 `/go2/odom_raw` 或该话题已有桥接发布者时，不重复启动。

核对时间与模型参考 TF 后，整套里程计融合只需启动一份 launch：

```bash
ros2 launch fast_lio_nav2 odom_fusion.launch.py
```

默认 `/sportmodestate`。融合启动可用 `sportmodestate_topic`、
`go2_body_frame_id`、`sportstate_velocity_frame`、`sportstate_timestamp_source`、
`sportstate_linear_velocity_stddev`、`sportstate_reference_point` 调整对应参数；两个桥的启动开关默认均为 true。
已有 `/go2/imu` 或 `/go2/odom_raw` 发布者（包括回放包）时，分别设置
`start_go2_imu_bridge:=false` 或 `start_go2_odom_bridge:=false`，避免重复发布。
这份 launch 已包含雷达驱动和 FAST-LIO，不要再单独启动 `mapping.launch.py`。

## 本地验证

```bash
colcon test --packages-select fast_lio --event-handlers console_direct+
colcon test-result --verbose
```

解析测试使用与禾赛 ROS 2 驱动一致的非对齐 26 字节点布局，检查大 epoch 时间
精度、排序、降采样保留帧结束时间、字段错误、时钟不一致和无效点处理。
编译和这些测试不能替代 NX 上的接收、同步、外参及建图验证。

## 2026-10-09 NX 静止测试记录

本人 `zyh-nav2` 容器已编译七个相关包，并完成一次 60 秒静止建图测试、一次 12 秒退出复测，
运动趟数均为 0。FAST-LIO 完成本体 IMU 初始化并持续输出位姿，保存了静止点云地图。
不代表移动建图、完整外参、EKF 或导航已经验证。

本次采用独立安装目录，原 `/work/zyh_ws/install` 未替换。加载本次编译结果：

```bash
source /work/zyh_ws/setup_static_test.bash
```

这个脚本只加载环境，不启动节点。完整观察值、原时钟问题和测试范围见
静止建图测试记录（历史附件已在本机归档）。
