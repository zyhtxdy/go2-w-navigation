# Go2 状态速度桥接

只读取宇树运动状态 `/lf/sportmodestate`，转换成 `/go2/odom_raw`。
输出类型为 `nav_msgs/msg/Odometry`，保留机身坐标中的完整三轴线速度，当前扩展卡尔曼滤波器（EKF）只选择融合 `vx`、`vy`。
节点不发布控制指令或 TF（坐标变换）。只在收到有效输入时发布，不在输入停更时定时发送伪零速度。

## 来源、录包依据和坐标配置

输入为 `unitree_go/msg/SportModeState`。
[宇树官方读取运动状态示例](https://github.com/unitreerobotics/unitree_ros2/blob/master/example/src/src/read_motion_state.cpp)
将 `velocity` 标注为 Odometry frame（里程计参考坐标系）的速度。
2026-10-13 对本工作空间的 `go2_inputs_20261010_094309` 录包作内部一致性对照时，
原始 `velocity` 直接使用更符合 LIO 的机身速度；再旋转一次会使部分前进／倒退方向反转。
因此本工作空间默认 `velocity_frame=body`，直接保留输入的机身 `vx`、`vy`、`vz`。
这不是所有宇树机器人或固件的统一坐标定义，也不是外部测量验证的速度精度。

此次对照使用 ROS 域 92 的一次离线回放中的 LIO-base 位姿，
以 ±0.2 秒窗口差分并按 LIO yaw 旋回机身方向。
389 个运动采样窗口中，已录旋转输出与 LIO 机身速度的 XY 差值范数中位数为 `0.578 m/s`；
原始速度直接使用的对应中位数为 `0.042 m/s`。
约 15 秒时，LIO 机身前向速度观察值为 `+0.544 m/s`，原始状态为 `+0.490 m/s`，
旧旋转输出却为 `−0.466 m/s`。
这些是录包内部一致性观察值，参考点、三维倾斜、时延和状态估计误差仍需核对。
详见 此次对照结果（历史附件已在本机归档）。

显式 `velocity_frame=odom` 仍保留，用于已核实输出里程计参考系速度的其他接口。
该模式使用归一化后的 `imu_state.quaternion`（顺序 `w,x,y,z`）计算 `R(q)^T v`，
把完整三轴输入速度旋回机身坐标；姿态包含倾斜时，输入 `vz` 也参与机身 `vx`、`vy` 的计算。
**启用此模式前，必须确认速度和四元数使用同一参考系，且四元数确实从机身旋到该参考系。**
两种模式均检查三轴速度及四元数的长度和有限性，拒绝零四元数；四元数不作为输出姿态测量。

默认 `child_frame_id=go2_body`，默认 `header.frame_id=go2_state_odom`。
这些名称本身没有证明它们与已有 `base_link`、FAST-LIO 的 `body` 或工程 `odom` 重合。
根据 [ROS Odometry 消息定义](https://github.com/ros2/common_interfaces/blob/humble/nav_msgs/msg/Odometry.msg)，
twist 必须位于 `child_frame_id` 坐标系。接入 EKF 前要核对机身轴向，并提供真实的机身到 EKF 基准帧变换；
不要只把 frame 名改成 `base_link` 来掩盖安装偏差。本节点不创建这些 TF。

官方 Go2W MuJoCo 模型的 `frame_vel` 传感器绑定在 `imu` site，官方仿真桥把其世界系速度
填入高层状态。因此统一融合 launch 默认采用 IMU 中心作为**模型参考初值**，发布
`base_link -> go2_body = [-0.02557, 0, 0.04232]` 米、零旋转。
这不是当前真机固件的参考点已确认；还需核对四元数与速度参考系、时间及转向数据。
通过 `sportstate_reference_point=imu/base/external` 可分别使用模型 IMU 点、经确认的
base 点或外部标定 TF。详见 [坐标核验记录](../../fast_lio_nav2/GO2_STATE_FRAME.md)。

## 有效字段和协方差

| 输出字段 | 内容 | 测量是否有效 |
|---|---|---|
| `twist.twist.linear.x` | 机身前向速度 | 待真机核对后的候选输入 |
| `twist.twist.linear.y` | 机身侧向速度 | 待真机核对后的候选输入 |
| `twist.twist.linear.z` | 机身竖向速度 | 待真机核对；当前 EKF 不选 |
| `pose.pose` | 零平移、单位四元数占位 | 无效；EKF 不融合 |
| `twist.twist.angular` | 零占位 | 无效；EKF 不融合 |

输入 `position` 和 `yaw_speed` 不参与输出。
三轴线速度方差均为参数 `linear_velocity_stddev` 的平方，默认标准差 `0.2 m/s`，即方差 `0.04 (m/s)^2`。
**这是保守的初始调参值，未经过当前 Go2W 实测，不代表速度精度。**
所有 pose 对角方差、角速度对角方差均为 `1e6`，只用于表示占位的不确定性；非对角项为零。
EKF 的观测选择矩阵仍须明确只选 `vx`、`vy`，不能仅依靠大协方差过滤占位字段。
保留 `vz` 和同等方差，便于下游按真实的机身到 `base_link` 倾斜变换转换三轴速度；不能预先丢弃竖向速度。

## 时间

默认 `timestamp_source=receive`：使用回调开始时的 ROS 时钟接收时间，不访问输入 `stamp`。
开启 `use_sim_time` 时按 ROS 仿真时钟接收时间处理；仿真时钟尚未推进时可能为零。
接收时间不是传感器采样时间，本节点不估计网络时延、时间偏移或漂移。

只有显式设置 `timestamp_source=message` 时，才读取输入 `stamp.sec`、`stamp.nanosec`。
秒必须属于 int32 范围，纳秒必须属于 `[0, 1e9)`，整体必须非零；缺失、非法或零时间戳会导致样本被拒绝，
不会悄悄改用接收时间。合法的负秒保留其值，不猜测时钟原点。
合法格式不能证明和其他输入同钟；融合前必须核对它与 XT16、机身 IMU、FAST-LIO 的时间基准和时延。

## 配置和 QoS

| 参数 | 默认值 | 用途 |
|---|---|---|
| `input_topic` | `/lf/sportmodestate` | 宇树状态输入 |
| `output_topic` | `/go2/odom_raw` | 工程速度观测输出 |
| `frame_id` | `go2_state_odom` | 占位 pose 的参考帧 |
| `child_frame_id` | `go2_body` | 输出速度所属机身帧 |
| `velocity_frame` | `body` | 当前录包采用机身速度；显式 `odom` 旋回机身 |
| `timestamp_source` | `receive` | `receive` 接收时间；`message` 验证后的消息时间 |
| `linear_velocity_stddev` | `0.2` | 三轴线速度的初始标准差 |
| `queue_depth` | `20` | 输入、输出 Keep Last 队列深度 |
| `use_sim_time` | `false` | 使用 ROS 仿真时钟 |

输入 QoS：Best Effort / Volatile / Keep Last 20。
输出 QoS：Reliable / Volatile / Keep Last 20，以兼容下游与 RViz。
输入无数据时输出不更新，程序不会用假零速度充当“已停住”的证明。

## 手动启动

需先安装或构建 `unitree_go` 消息包和本包，并加载对应工作空间。
2026-10-09 在本人 NX 容器中已编译并做读取测试。当前 `/sportmodestate` 没有发布者，
因此默认输入改为有发布者的 `/lf/sportmodestate`；话题前缀本身不保证发布频率。
其他固件可通过 `input_topic` 覆盖，不启用或修改机器狗服务。

```bash
ros2 launch go2_odom_bridge sportmodestate_odom.launch.py
```

在当前使用的 DDS 域和网卡环境中验证能持续读取 `/lf/sportmodestate`，再检查输出。
禁止用不属于当前固件的速度坐标假设直接宣称融合已可用。

回放上述录包时，要排除已录的 `/go2/odom_raw`，从原始 `/lf/sportmodestate` 重新桥接。
只修改启动参数、继续回放旧桥接输出不会修正它的速度方向。
该录包的 34359 条原始运动状态 `stamp` 全部为零，须继续使用明确标记的接收时间。

## 离线验证

测试仅构造 Python 数据对象和标准 ROS 消息，不创建 ROS 节点、不依赖机器狗或 `unitree_go`。

```bash
source /opt/ros/humble/setup.bash
PYTHONPATH=src/driver/go2_odom_bridge:$PYTHONPATH \
  /usr/bin/python3 -m unittest discover -s src/driver/go2_odom_bridge/test -v
```

## 原始状态录包回放

`bag_path` 非空时，节点直接读包中的原始 `input_topic` CDR 数据，按 `/clock`
时序转换后只发布标准 Odometry，不订阅实时狗状态。需 `use_sim_time=true`。
`timestamp_source=receive` 在此模式使用原记录时间作为接收时间近似；原始
消息全零时间戳不会被当采样时间。输出在数据结束后停止，时钟倒退需重启
完整建图回放。当前统一离线入口负责提供这些参数。

这用于重算旧包里的方向错误速度，也避开旧 Unitree 原始消息通过 FastDDS
直接序列化发布失败的问题；不修改原始包，也不发布任何控制指令。
