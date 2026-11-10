# Go2 本体 IMU 桥接

读取 `/lowstate` 的 `unitree_go/msg/LowState.imu_state`，发布 `/go2/imu`，
类型为 `sensor_msgs/msg/Imu`。它与雷达内置的 `/utlidar/imu` 是不同来源。
节点只订阅状态并发布 IMU，不发布运动指令或坐标变换（TF）。

## 数据处理和限制

- 陀螺仪、加速度三个分量直接映射；接口约定分别是 rad/s 和 m/s²。
  实际狗上的单位、轴向、频率目前没有真机核实。
- 不旋转轴，不减重力，不校正偏置，不解释 `tick` 的单位或依据它定时。
- `header.stamp` 是回调开始时的 ROS 接收时间，**不是已确认的采样时间**。
  尚未实现严格的 XT16/IMU 时间同步；无线延迟或 DDS 排队会进入时间误差。
- `frame_id` 默认 `go2_imu`，仅标记本体 IMU 坐标系。该名称不代表已经标定。
  XT16 到本体 IMU、IMU 到机身的实际外参仍需测量。
- 默认不发布姿态：四元数为单位四元数，`orientation_covariance[0] = -1`。
  开启姿态后，按 Unitree 的 wxyz 顺序归一化并映射到 ROS；使用前须核实
  实际姿态约定。零四元数、短数组或非有限四元数会标记姿态不可用，仍保留
  有效的陀螺仪和加速度。巨大有限四元数采用缩放归一化，避免模长溢出。
- 非有限或不完整的陀螺仪/加速度样本丢弃，告警最多每 5 秒输出一次。
  其他协方差保持全零，表示未知；没有填入未经测量的噪声数字。
- 输入输出使用 Best Effort、Volatile、Keep Last，默认深度 20。

消息字段参照 [Unitree 官方 LowState 定义](https://github.com/unitreerobotics/unitree_ros2/blob/master/cyclonedds_ws/src/unitree/unitree_go/msg/LowState.msg)
和 [IMUState 定义](https://github.com/unitreerobotics/unitree_ros2/blob/master/cyclonedds_ws/src/unitree/unitree_go/msg/IMUState.msg)。
必须使用与机器人兼容的完整 `unitree_go` 消息定义，不能只保留 IMU 字段。

## 编译和启动

本包需要已提供源码或已安装的 `unitree_go` 消息包，不需要运动控制 SDK。
先加载 ROS 环境和消息包所在工作空间，再在本工作空间执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to go2_imu_bridge
source install/setup.bash
```

以下为后续手动启动方式；文件同步到 NX 不代表已经编译或运行：

```bash
ros2 launch go2_imu_bridge lowstate_imu.launch.py
```

参数：`input_topic`、`output_topic`、`frame_id`、`queue_depth`、
`publish_orientation`、标准参数 `use_sim_time`。

需要姿态时显式设置 `publish_orientation:=true`。录包回放中已经存在
`/go2/imu` 时不要再启动桥接，避免出现两个发布者。

接入 FAST-LIO 需使用 `imu_topic:=/go2/imu`。仅替换话题不代表时间同步和
外参已标定，也不代表整套融合和运动执行链路已经真机可用。

## 离线转换测试

测试仅创建标准消息和 `SimpleNamespace` 假 LowState，不导入 `unitree_go`，
不创建 ROS 节点、不连接机器人：

```bash
source /opt/ros/humble/setup.bash
PYTHONPATH="$PWD/src/driver/go2_imu_bridge:$PYTHONPATH" \
  python3 -m unittest discover -s src/driver/go2_imu_bridge/test -v
```
