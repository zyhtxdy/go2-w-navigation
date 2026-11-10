# Go2W 运动状态坐标核验

本次只核对资料、模型及离线代码，不启动 NX 节点，不控制机器狗，不在 NX 编译。

## 资料核验结论（运动录包分析前）

| 项目 | 结论 | 适用范围 |
|---|---|---|
| 原始 `velocity` 的表达方向 | 官方真机示例标注为 Odometry frame | 官方 ROS 示例 |
| 速度参考点 | 官方 Go2W 仿真使用本体 IMU 中心 | 官方仿真实现；真机固件待核验 |
| IMU 相对模型 base 的位置 | `[-0.02557, 0, 0.04232]` 米 | Go2W 官方模型 |
| IMU 相对模型 base 的方向 | 零旋转 | Go2W 官方模型 |
| 真机速度与四元数是否共享世界参考 | 尚未验证 | 当前 Go2W |
| 真机测试次数 | 没有记录 | 本次核验 |

因此统一融合启动采用上述 IMU 点作为**名义参考初值**，接通
`base_link -> go2_body`。不将 `go2_body` 设成与 `base_link` 零平移，
也不宣称完成了真机标定。资料核验阶段尚无运动录包；下面补充 2026-10-13 的现有录包对照。

## 2026-10-13 运动录包对照

纯读取 `go2_inputs_20261010_094309` 的原始 `/lf/sportmodestate`，并以
独立 FAST-LIO 换算的 `/lio/odom_base` 作内部对照。389 个运动窗口使用
前后各 0.2 秒的位姿差分，再旋回机身方向；这不是外部速度标定。

| 输入候选 | 与 LIO 机身 XY 速度差的中位范数 |
| --- | ---: |
| 已录 `/go2/odom_raw`（原 world→body 再旋转） | 0.578 m/s |
| 原始 `state.velocity` 直接使用 | 0.042 m/s |

约第 15 秒，LIO 机身前向速度为 +0.544 m/s，原始状态为 +0.490 m/s，
已录桥接却为 −0.466 m/s；约第 60 秒倒退时，分别为 −0.596、−0.593、
+0.588 m/s。方向检查说明此录包的原始线速度更符合机身方向，再用高层
四元数旋转会反转部分前进／倒退速度。默认配置据此改成
`sportstate_velocity_frame=body`；显式 `odom` 保留用于其他已核实的接口。
不能把这份录包结论推广为所有宇树型号或固件的定义。

34359 条原始状态的消息时间戳全为零，继续使用接收时间近似，不选择
`message`。姿态变化与 LIO 一致不等于速度与该姿态共享世界参考系。
速度的精确参考点、完整外参和绝对精度仍未标定。

新的离线验证必须重新桥接原始状态，排除旧的 `/go2/odom_raw` 输出。
改变 launch 参数不会改变包里已经算错的速度数据。详细依据见
[融合修复记录](../../docs/validation/2026-10-13-fusion-diagnosis/README.md)。

## 可追溯依据

1. [官方 Go2W MuJoCo 模型](https://github.com/unitreerobotics/unitree_mujoco/blob/main/unitree_robots/go2w/go2w.xml)：
   `imu` site 的位置为 `-0.02557 0 0.04232`，没有额外旋转；
   `frame_pos` 和 `frame_vel` 均指定 `objtype="site" objname="imu"`。
   Git blob：`88b7dbe2f6ca775a60cb1ddb2e4c2f1ee9573c5a`。
2. [官方仿真 Python 桥接](https://github.com/unitreerobotics/unitree_mujoco/blob/main/simulate_python/unitree_sdk2py_bridge.py)：
   将模型提供的 frame 速度填入 `high_state.velocity`。
   Git blob：`a9ca97ae12ea15f5eaf94eaa83f1f366dd2b5b4c`。
3. [MuJoCo framelinvel 定义](https://mujoco.readthedocs.io/en/stable/XMLreference.html#sensor-framelinvel)：
   返回指定对象坐标原点的线速度，以全局方向表达。
4. [官方真机读取示例](https://github.com/unitreerobotics/unitree_ros2/blob/master/example/src/src/read_motion_state.cpp)：
   标注 position 和 velocity 为 Odometry frame，没有明确速度对应的物理参考点。
5. [Go2W URDF](https://github.com/unitreerobotics/unitree_ros/blob/master/robots/go2w_description/urdf/go2w_description.urdf)：
   `base -> imu` 固定安装值与上述 MuJoCo 模型一致。
   Git blob：`c3798d6839875ca756ddd7c007d9d4f480388bab`。

注意：官方仿真桥没有为高层状态填有效 `imu_state.quaternion`。
它能作为速度参考点的源码证据，不能直接作为当前依赖高层四元数的桥接节点的完整测试输入。

## 单个融合 launch 的设置

```bash
ros2 launch fast_lio_nav2 odom_fusion.launch.py
```

| 参数值 | 发布的 `base_link -> go2_body` | 使用条件 |
|---|---|---|
| `sportstate_reference_point:=imu`（默认） | 采用 `base_to_imu_*` 的六个安装参数 | 官方仿真模型参考初值，真机尚未验证 |
| `sportstate_reference_point:=base` | 零平移、零旋转 | 已确认真机速度参考点和轴向就是模型 base |
| `sportstate_reference_point:=external` | 本 launch 不发布这条 TF | 使用经标定的外部 TF，或回放已有该 TF |

`go2_body_frame_id` 仅选择输出名字，不改变速度参考点。
若显式使用 `go2_body_frame_id:=base_link`，必须同时选择 `sportstate_reference_point:=base`。
这两个参数配套，不用改名字来绕过模型偏移。
外部 TF 若显示轴向不同，还须核对四元数转换与该 TF 是否重复旋转。

## 真机核验步骤（尚未执行）

先核对时间，再核对方向，最后比较速度参考点。这里只给以后由使用者执行的步骤，
不占用当前正在使用的机器人。

1. 检查 XT16、原始本体 IMU、`/lf/sportmodestate`、FAST-LIO 的时钟来源和时延。
   2026-10-09 NX 静止检查发现雷达设备时间停在 2020 年，驱动配置改用 NX UDP 接收时间；
   本体桥同样使用 NX 接收时间。二者共享系统时钟只是初期近似，传输和处理时延仍须核对。
2. 单独用雷达和 IMU 跑 FAST-LIO，换算得到 `/lio/odom_base`。
   不用已融合本体速度的 `/odom` 作为独立对照，避免用输入验证自身。
3. 待狗空闲，由人遥控录制静止、不同朝向直行、左转和右转片段。
   原始数据必须保留，建议录制：

   ```bash
   ros2 bag record /hesai/points /lowstate /lf/sportmodestate \
     /go2/imu /Odometry /lio/odom_base /tf /tf_static
   ```

4. 比较 `R(q)^T velocity` 与独立 LIO 轨迹求出的机身速度，核对前向、侧向符号，
   以及转向后输入采用 world/odom 还是 body 方向。记录每个片段的时长和趟数。
5. 检查转向片段中的参考点差异。设 `r` 为 base 到速度参考点的向量：

   ```text
   v_base = R_base_go2body * v_go2body + r × omega_base
   ```

   这也是 robot_localization 将 twist 换算到 base_link 时使用的关系；
   其角速度来自滤波器当前状态，当前融合由 `/imu/corrected` 提供 yaw 角速度。
   官方模型候选 `r=[-0.02557,0,0.04232]` 与 `r=0` 在转向片段才有可区分的效果。
6. 静止或只有直行数据不能确定参考点；只有水平转向不能确定 Z 偏移。
   真机 Z 偏移和完整坐标定义仍需厂家说明或额外三维测量。

以上模型值没有外部测量对照；当前没有实测误差、成功率或结论。

## 2026-10-09 EKF 静止检查补充

本人容器补齐 `robot_localization` 运行依赖。实测 `/sportmodestate` 没有发布者，
`/lf/sportmodestate` 有发布者，融合与速度桥默认输入改为后者，仍可通过参数覆盖。
本次消息原始 stamp 全为零，因此保持 `timestamp_source=receive`。
三路输入、基准点转换、EKF 和私有 TF 已做 30 秒静止测试（1 次，0 趟运动），
融合输出约 30 Hz；移动速度的坐标和参考点仍待验证。
详见 [EKF 检查记录](../../docs/validation/2026-10-09-ekf/README.md)。
