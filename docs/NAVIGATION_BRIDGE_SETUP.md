# Go2W Nav2 速度桥接与初始参数（2026-10-09）

本次范围：本机开发、同步到本人 `zyh-nav2` 容器、编译。
没有启动 Nav2、速度桥接，没有发目标、速度或 Sport 请求，没有真机测试。

## 接线

`real_nav2.launch.py` 启动 Nav2 和 `go2_cmd_vel_bridge`。
Nav2 controller 的输出先经过 velocity_smoother，再由桥接转为宇树运动请求。
输出话题为 `/go2/nav/cmd_vel`，消息类型 `geometry_msgs/msg/Twist`；
桥接输出 `/api/sport/request`，消息类型 `unitree_api/msg/Request`。
`Move=1008` 与 JSON `x/y/z` 字段来源是
[宇树官方 SportClient](https://github.com/unitreerobotics/unitree_ros2/blob/668d1ec5a05d1c38d3306bdca7d59f2ba3581a88/example/src/src/common/ros2_sport_client.cpp)。

节点级重映射只改变 `velocity_smoother:cmd_vel_smoothed` 和
`behavior_server:cmd_vel`，不把平滑器输入重映射成自己的输出。
默认不订阅公共 `/cmd_vel`，避免接收别人的方案。

`odom_fusion.launch.py` 仍单独提供 FAST-LIO、EKF、IMU/里程计数据桥接和地形处理。
本次没有新增旧地图重定位功能；固定 `map -> odom` 仍属于在线运行时的坐标连接。

## Go2W 初始参数

以下是开发者选择的初始限制与规划包络，不是实测结果或厂家整机尺寸。
本体原点沿用此前核对的
[宇树 Go2W 模型](https://github.com/unitreerobotics/unitree_ros/blob/master/robots/go2w_description/urdf/go2w_description.urdf)。
模型包含头、腿和轮子；不采用本体外壳的碰撞盒作为整狗 footprint。
加装 XT16 和运动姿态会改变实际包络，尚未核对当前机器尺寸。

| 参数 | 本次初值 | 说明 |
| --- | --- | --- |
| 局部 / 全局 footprint | X ±0.40 m、Y ±0.25 m | 0.80 × 0.50 m 初始规划包络，含余量 |
| 3D A* 障碍膨胀半径 | 0.48 m | 不小于上述矩形的外接半径约 0.472 m |
| 局部代价地图膨胀范围 | 0.50 m | 使用完整 footprint 检查碰撞 |
| 竖向净空检查高度 | 0.75 m | 含加装雷达的保守初值，非测量值 |
| 前进上限 | 0.20 m/s | MPPI、DWB、平滑器和桥接一致 |
| 横移上限 | 0 | 当前使用轮式前进 / 转向模式 |
| 转向上限 | 0.35 rad/s | 控制器、平滑器和桥接一致 |
| 起步对齐转速 | 0.25 rad/s | Rotation Shim 的初值 |
| 线加速度 / 减速度 | 0.30 / 0.50 m/s² | 平滑器和局部控制器初值 |
| 角加速度 / 减速度 | 0.60 / 0.60 rad/s² | 平滑器和局部控制器初值 |
| 最大规划坡度 / 台阶 | 10° / 0.08 m | 先限制在实验室平地轮式场景 |
| 速度平滑器超时 | 0.20 s | 上游速度停止更新时归零 |
| 桥接超时 | 0.30 s | 本进程仍运行时发送 StopMove |
| IMU 到地面 Z 偏移 | -0.50 m，暂保留 | 缺少当前站姿实测，不能当成校准值 |

上述坡度和台阶也同步给地形分类，避免前后模块使用不同通行标准。
MPPI 默认路径跟踪仍不倒车；桥接允许最多 0.15 m/s 后退，供后续明确的倒车任务使用。
这不代表已实现“原路倒退”。
两份配置 `nav2_mppi_params.yaml`（默认）与 `nav2_params.yaml`（DWB 备选）均更新。

## 加载环境

编译使用本人容器的 `/work/zyh_ws/build` 和 `/work/zyh_ws/install`。
以后从正常目录加载：

```bash
source /opt/ros/humble/setup.bash
source /work/zyh_ws/install/setup.bash
```

ROS 网络变量仍按 `NETWORK_SETUP.md` 配置；这两个 source 只加载环境。
`setup_static_test.bash` 仍指向旧的独立验证目录，不包含这次新增导航桥接。

之后由操作人员决定何时启动导航。`real_nav2.launch.py` 默认会启动能下发运动请求的桥接。
仅查看规划时使用 `start_cmd_vel_bridge:=false`。
实测 IMU 离地高度后，可用 launch 参数 `lio_height_offset:=负的高度米数` 替换初值。

## 限制

- 超时停车依赖桥接进程仍在运行，未验证断电、SIGKILL、网络失联时机器人行为。
- 软件停止与禁用接口见桥接包 README；不代表已通过硬件急停或遥控接管验收。
- Sport 请求结构已按官方消息定义实现，当前 Go2W 固件是否接受、运动模式和人工控制优先级未测试。
- 没有本次真机趟数、成功率或到点精度记录。
