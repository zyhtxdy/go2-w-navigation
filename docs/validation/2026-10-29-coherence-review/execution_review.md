# 导航执行链一致性复核（2026-10-29）

范围：本机启动配置、离线速度门控、任务取消、行为树路径复核、局部控制和速度桥接。最初完成只读审查和纯单元检查；随后统一编译后，在 localhost 独立域增加一次完整录包执行链与故障注入检查。不连接 NX，不发布机器狗 API 运动指令，不代表真机通过率。

## 已修复的三项不一致

| 优先级 | 原问题与触发条件 | 最小修复 | 验证及边界 |
| --- | --- | --- | --- |
| P1 | 在线模式局部地形停止更新、里程计和控制器仍运行时，两路 `expected_update_rate: 0.0` 会令代价地图观测永久标为 current；桥接仅检测速度指令超时，无法发现感知停更。 | canonical `src/nav2/navigation_bringup/config/nav2_mppi_params.yaml:347`、`:357` 两处改为 `0.8` 秒。 | 地形处理间隔配置为 `0.20` 秒；Humble 接受的是秒数，非 Hz。完整离线执行链中停止地形更新后，局部代价地图报告观测过期，controller 与后续两路没有继续输出非零。在线使用相同局部配置，但未另起在线全栈重复测试。 |
| P2 | 离线一次导航正常结束后，`forwarded_nonzero` 仅在取消流程结束清零。之后即使任务全终止、持续零指令，短暂输入故障仍把旧路径当作在执行任务，进入恢复和取消流程。 | `navigation_guard.py:80`、`:136`、`:147` 增加本轮运动关联任务与首次终止证据；只有相关任务都确认终止、证据覆盖最后非零指令、已转发零指令且安静超过既有超时，才归为 idle。 | 重复旧终止消息、空状态、无跟踪任务的运动、最后非零之后没有新终止证据，均不能清除运动历史；既有取消屏障仍保留。 |
| P2 | 在线入口暴露 `forward_compensation`，离线入口只暴露角速度开关；在离线命令中填写前进补偿 false 不会覆盖桥接 YAML 默认 true。 | `offline_nav.launch.py:106` 显式传 `forward_compensation_enabled`；`:124` 增加同名参数，默认 true。 | 四组前进/转向 true、false 组合均构造真实 launch actions 并检查桥接参数覆盖；未启动节点。 |

修改前备份：同目录 `execution_before/`，包含修改前 canonical 配置、guard、guard 测试、离线 launch 与测试注册 CMake 文件。

## Humble 行为核对

已核对本机安装头文件及上游 Humble 分支：`ObservationBuffer` 将 `expected_update_rate` 直接转为秒数时长；零值跳过更新超时检查。`ObstacleLayer` 会将 marking、clearing 观测的新鲜程度传播为 layer 的 current 状态。来源：[ObservationBuffer](https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_costmap_2d/src/observation_buffer.cpp)、[ObstacleLayer](https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_costmap_2d/plugins/obstacle_layer.cpp)。

`0.8` 秒是恢复已有感知停更检测的配置值；没有修改速度补偿、运动上限、footprint 或局部碰撞检查。软件停止输出已有一次故障注入证据，机器狗实际停止延迟和距离没有测量。

## 已核对且未发现断链的部分

- 在线速度链：controller → `cmd_vel_nav` → smoother → 私有输出 → bridge。只重映射 smoother 和 behavior server 的输出，不重映射内部输入。
- 离线速度链：内部 Nav2 bridge 关闭；私有 unchecked 速度 → localization adapter → 离线 bridge。没有新增重复 bridge。
- 离线恢复时，规划输入 ready 与最终运动 ready 分开发布，路径复核可在速度仍锁住时运行；未发现必须先解锁运动才能复核的循环依赖。
- 行为树在路径尚未生成时进入初次规划；传感器/定位不可用与真实几何无效有独立状态；未发现先前 `should_send_request_ = false` 永久禁止后续调用的情况，Humble 的父类每次 tick 会重置它。
- 前进和转向补偿存在拟合区间及平滑回到 1:1 的区间；最大拟合值不等于原始请求的硬上限，未据此新增限幅。
- 离线配置在 CMake 安装时复制 canonical Nav2 YAML；源目录没有第二份 `config/nav2_mppi_params.yaml` 是安装规则的结果，非文件丢失。

## 能力边界

目前局部控制仍是 RotationShim + 前进 MPPI：`angular_dist_threshold: 0.35`、`vx_min: 0.0`、`PreferForwardCritic` 启用。全局 guidance 路径存在，不代表在狭窄通道里必有可原地转身的局部轨迹。目标在身后且空间不允许原地转身的情况，需要和起步对准后的向前穿越区分；本轮没有关闭 Shim 的碰撞检查，也没有开启倒车策略。

## 本机检查

执行命令：

```bash
source /opt/ros/humble/setup.bash
PYTHONPATH="$PWD/src/go2_offline_navigation:$PYTHONPATH" python3 -m pytest -q \
  src/go2_offline_navigation/test/test_navigation_guard.py \
  src/go2_offline_navigation/test/test_execution_configuration.py
```

结果：`57 passed in 0.38s`。其中 guard 51 项，launch/配置 6 项。

`python3 -m py_compile` 检查修改的 guard、离线 launch 和新增配置测试通过。新增测试已注册到 `go2_offline_navigation/CMakeLists.txt`。统一编译由主任务执行，本审查分工没有单独编译。

## 完整执行链与故障注入（本机回放 1 次，真机 0 趟）

输入为 `bags/go2_inputs_20261010_094309`、`maps/lab_v1_database` 和 `docs/validation/2026-10-26-live-path-obstacles/navigation_candidate`。使用 localhost 域 196，真实离线导航栈开启，速度桥接关闭。

结果：[run_candidate/report.json](integration/run_candidate/report.json)，11 项检查全部通过：

- 录包约第 10 秒经真实 NavigateToPose 接受 `(4.5, -0.1)` 目标，生成路径，并看到 controller、smoother、离线 gate 三段非零指令。
- 主动取消后结果状态为 CANCELED，两路末端速度收到零。取消后短暂暂停本次回放自己的 terrain 子进程，日志确认为 `Navigation inputs unavailable while idle`；恢复输入后重新 ready。
- 第二任务已有非零指令后，再暂停同一个 terrain 子进程。局部地图报告观测过期；故障开始 1.3 秒后的检查窗口内，controller、smoother 和 gate 均无非零指令，并已观察到末端零指令。
- 全程无 `/api/sport/request` publisher、无速度桥接；五个自建子进程均以返回码 0 退出。FAST-LIO 原始调试日志已恢复，没有遗留本次 FAST-LIO 进程。

回放位姿独立于控制指令，以上只证明消息链、取消和停更保护，不证明闭环跟踪质量或真机窄通道成功率。
