# Go2W 速度指令桥接

`geometry_msgs/msg/Twist` → `unitree_api/msg/Request`。
将 `linear.x`、`linear.y`、`angular.z` 转换为 Go2 的 `x`、`y`、`z`。
默认开启正向线速度和角速度前馈补偿；角速度按用户要求正负对称，倒车线速度与横移透传，未增加桥接总限速。
宇树官方 [ROS 2 SportClient](https://github.com/unitreerobotics/unitree_ros2/blob/668d1ec5a05d1c38d3306bdca7d59f2ba3581a88/example/src/src/common/ros2_sport_client.cpp)
定义 `Move=1008`，参数 JSON 为 `x=vx, y=vy, z=角速度`；`StopMove=1003`。
沿机器人本体坐标：X 向前、Y 向左、正角速度绕 Z 左转，单位 m/s 和 rad/s。

## 启动接线

`navigation_bringup/launch/real_nav2.launch.py` 默认启动此桥接。

```text
Nav2 controller → /cmd_vel_nav → velocity_smoother
                                      ↓
                              /go2/nav/cmd_vel
                                      ↓
                             go2_cmd_vel_bridge
                                      ↓
                             /api/sport/request
```

Humble Nav2 的平滑器输出和 behavior_server 输出按节点名称重映射到专用话题。
桥接不订阅公共 `/cmd_vel`，避免接收其他方案的同名速度。
参数见 `config/go2_cmd_vel.yaml`；`real_nav2.launch.py` 可覆盖 `cmd_vel_topic` 和 `sport_request_topic`。
不接入高频关节控制，不自动站起、不切换运动模式、不关闭遥控器。
使用前须由操作人员使机器人处于能接受 Sport Move 的状态。
此 launch 仍只启动 Nav2 和速度桥接，建图、融合和地形处理由 `odom_fusion.launch.py` 提供。

## 正向速度补偿（2026-10-19）

原入口默认开启正向速度补偿，无需另起一个桥接：

```bash
ros2 launch navigation_bringup real_nav2.launch.py
```

关闭线速度补偿时增加 `forward_compensation:=false`；全部恢复 1:1 时再增加 `angular_compensation:=false`。
在 `real_nav2.launch.py` 中，开关以这个启动参数为准，其余系数读取桥接 YAML。
直接运行桥接时同样默认开启，可用 ROS 参数 `forward_compensation_enabled:=false` 关闭。
校准参数在节点启动后只读；修改它们需要重新启动本桥接，不能在运动过程中静默切换。
Nav2 和平滑器填写希望达到的速度，桥接将它转换成 Go2 Move 的原始参数。
例如目标 0.30 m/s 对应原始指令约 0.393；不要再在上游重复套用同一补偿。

用户提供的正向指令/EKF 速度观察如下，单位 m/s：

| 原始指令 | EKF 观察值 |
| --- | --- |
| 0.10 | 0.046 |
| 0.15 | 0.091 |
| 0.20 | 0.136 |
| 0.25 | 0.160 |
| 0.30 | 0.210 |
| 0.35 | 0.260 |
| 0.40 | 0.290 |
| 0.45 | 0.340 |
| 0.50 | 0.400 |
| 0.60 | 0.490 |
| 0.70 | 0.590 |

每档趟数、稳态统计窗口和外部测速对照没有记录。以下是这 11 个档位的经验拟合，
不能等同于已经通过独立实测标定，也不能证明补偿后速度一定准确。

`v_ekf ≈ 0.899731458 * u - 0.053265985`，其逆模型参数取六位小数：

| 启动参数 | 默认值 | 含义 |
| --- | --- | --- |
| `forward_compensation_enabled` | `true` | 开启正向补偿 |
| `forward_compensation_gain` | `1.111443` | 比例，至少为 1 |
| `forward_compensation_offset` | `0.059202` | 原始指令偏置，非负 |
| `forward_compensation_blend_speed` | `0.05` | 目标速度达到此值时使用完整偏置 |
| `forward_compensation_max_command` | `0.70` | 补偿增量适用的原始指令上边界 |

对于 `0 < v < 0.70`，令 `t=min(v/0.05, 1)`，计算：

```text
u_fit = 1.111443 * v + 0.059202 * t² * (3 - 2t)
u_sent = max(v, min(u_fit, 0.70))
```

这样零附近不会突然加上固定偏置；0.05 m/s 以上使用完整逆模型。
本节线速度模型保持零速、倒车和横移原值；角速度由下一节的独立模型处理。线速度补偿只增加正向指令。
补偿增量最多推到已测过的 0.70；输入本来就大于等于 0.70 时仍按原值传递，
不继续外推补偿。这一边界不会限制关闭补偿时的 1:1 桥接。
到达该边界后不能保证目标速度能达到；上游仍须自行配置导航运行速度。

| 目标速度 | 预测原始指令 |
| --- | --- |
| 0.10 | 0.170346 |
| 0.20 | 0.281491 |
| 0.30 | 0.392635 |
| 0.40 | 0.503779 |

上述预测不是新增真机测试结果。第一版不加入 PID，不读取或修改里程计。
零附近缺少实测数据，连续过渡并不保证微速行驶时已经消除死区。
补偿使原始指令的变化幅度也发生变化，仍需验证起步、减速和实际停车距离。

观察话题均为 `geometry_msgs/msg/TwistStamped`：

- `/go2_cmd_vel_bridge/cmd_vel_requested`：当前有效目标速度。
- `/go2_cmd_vel_bridge/cmd_vel_sent`：桥接生成的原始指令；不是实际车速。

两者每次发送使用相同时间戳，沿本体速度轴，`frame_id` 留空，桥接不增加 TF。
发出 StopMove 时两者发布零值，停车重复结束后一起静默；未接收运动指令时不发布。
调试话题不接回控制链。

```bash
ros2 topic echo /go2_cmd_vel_bridge/cmd_vel_requested --field twist.linear.x
ros2 topic echo /go2_cmd_vel_bridge/cmd_vel_sent --field twist.linear.x
```

## 转换与停车

- 2026-10-13 按用户要求取消桥接限速，删除前进、后退、横移与转向四个上限参数；代码不再声明这些参数。
- 默认 MPPI / 平滑器的测试上限为前进 0.30 m/s、转向 ±0.45 rad/s；横移为零由上游配置，不在桥接中强制清零。
- 直接向桥接输入话题发布 Twist 时，不经过 Nav2 / 平滑器的限幅；是否进行正向补偿取决于桥接开关。
- 20 Hz 发送最新有效指令，单调时钟检查 0.3 秒超时。
- Twist 六个分量均须有限；拒绝升降、横滚、俯仰速度。
- 只有发过非零 Move 的会话才发 StopMove；从未接收运动指令时保持静默。
- 零指令、无效指令、超时、禁用或软件停止触发停车；连续发送 3 次后保持静默。
- 发送队列深度为 1、Volatile，请求有效期不超过指令超时。
- Ctrl-C/SIGTERM 正常退出时尝试发送停车；信号强杀、断电和失联无法由本进程保证停车。
- 软件停车依赖桥接进程仍在运行；本次修改没有验证硬件急停。

## 人工禁用与软件停止

`/go2_cmd_vel_bridge/set_enabled`：`std_srvs/srv/SetBool`，false 禁用桥接。
禁用后停止当前会话并保持静默，不持续发送停车去争抢人工控制。
true 恢复时要求新的速度指令。
`/go2/nav/estop`：`std_msgs/msg/Bool`，true 锁住本桥接；false 清除后要求新指令。
操作人员可以用 `start_cmd_vel_bridge:=false` 完全不启动桥接，
或 `bridge_enabled:=false` 启动但不接收运动指令。
机器人固件是否接受此接口、遥控器优先级及实际停车行为还需之后实测。

## 本次验证范围

2026-10-13 的 1:1 转换修改只在本机运行无 ROS 通信的单元测试。
没有启动桥接或 Nav2，没有发布目标或运动请求；未同步或编译 NX。

2026-10-19 新增补偿的纯软件检查，以及真实 ROS 节点的接线检查。
本次仅修改并编译本机，尚未同步 NX，真机测试 0 趟。
ROS 检查使用 localhost、独立 Domain 124 和 `/test_forward_compensation/fake_sport_request`，
确认物理 `/api/sport/request` 无发布者；没有启动驱动、Nav2 控制器或进行真机运动。
验证覆盖默认透传、正向模型、低速连续性、适用范围边界、非有限输入、
重复发送不叠加补偿、原停车路径、只读参数、SDK 请求 JSON 和调试话题。


## 角速度补偿（2026-10-28）

根据本次实际 Request 下发和 `/odom` 观察记录：0.10～0.40 各一次，包含额外的 0.35 档；
每档去掉前 2 秒后统计平均值。两次 0.05 的请求接收长间隔不进入拟合。
这些是 EKF 观察值，没有外部角速度真值对照。原始瞬时反馈波动约 0.11 rad/s，
稳态均值拟合不能代表单帧精度或多次重复精度；补偿后尚未真机验证。
数据和完整分析见 [本次标定记录](../../../docs/validation/2026-10-28-angular-calibration/README.md)。

响应拟合：`w_ekf ≈ 1.133298444 * u - 0.029350016`。
逆模型：`|u| = 0.882380 * |w_target| + 0.025898`。
模型允许小幅增加低角速度，也允许减小偏快的高角速度。
按用户明确要求，负角速度使用同一绝对值模型再取反；本次负方向未实测。
不加入 PID，不修改 `/odom`，不根据反馈在线调整参数。

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| angular_compensation_enabled | true | 默认启用 |
| angular_compensation_gain | 0.882380 | 逆模型比例 |
| angular_compensation_offset | 0.025898 | 逆模型偏置，rad/s |
| angular_compensation_blend_speed | 0.05 | 零附近和上边缘的连续过渡宽度，rad/s |
| angular_compensation_max_speed | 0.40 | 完整模型的目标角速度绝对值上边界，非总限速 |

零速始终是零；绝对值 0～0.05 使用 smoothstep 引入偏置。
0.05～0.40 使用完整逆模型；0.40～0.45 只把边界修正连续退回零，不继续外推拟合斜率；
绝对值达到 0.45 后原样传递。两段过渡是工程处理，尚未重新实测验证。
默认配置下映射连续、单调、保留符号，不会把微小输入突然抬成固定转速。
原有零命令、超时、禁用、软件停止和退出停车逻辑仍有效。

在线、离线入口都可加 `angular_compensation:=false` 单独关闭。
直接运行桥接时用 ROS 参数 `angular_compensation_enabled:=false`。
系数启动后只读；YAML 使用 `/**` 以同时匹配在线和离线桥接节点名称。

观察原始期望和实际提交的请求（不是实际反馈）：

```bash
ros2 topic echo /go2_cmd_vel_bridge/cmd_vel_requested --field twist.angular.z
ros2 topic echo /go2_cmd_vel_bridge/cmd_vel_sent --field twist.angular.z
```

离线节点对应 `/offline_cmd_vel_bridge/cmd_vel_requested`、`/offline_cmd_vel_bridge/cmd_vel_sent`。
本次只在本机验证代码，不同步 NX、不启动实际运动桥接或下发运动请求。
