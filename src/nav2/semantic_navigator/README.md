# Semantic Navigator

该功能包位于视觉语言导航与现有 Nav2 之间。它不直接输出速度，而是把
“去会议室”之类的指令解析为语义实体，选择距离机器人最近的候选停靠位，
然后调用 Nav2 的 `NavigateToPose`。若一个停靠位失败，会按距离依次尝试
同一实体的其他停靠位。

## 配置语义位置

编辑 `config/semantic_locations.yaml`，将测量后的 `map` 坐标写入 `poses`，
并设置 `enabled: true`。每个实体应配置多个位于障碍物外、具有地面支撑的
候选停靠位，`yaw` 使用弧度。

## 启动和测试

先启动里程计、建图及 Nav2，再执行：

```bash
ros2 launch semantic_navigator semantic_navigation.launch.py
ros2 run semantic_navigator semantic_goal 去会议室
```

也可以直接发布指令：

```bash
ros2 topic pub --once /semantic_navigation/command \
  std_msgs/msg/String "{data: '去会议室'}"
```

状态为 UTF-8 JSON：

```bash
ros2 topic echo /semantic_navigation/status
```

发送“取消导航”“停止导航”或 `cancel` 可以取消当前目标。

## 后续接入视觉/VLN

视觉语义建图模块应负责检测、三维定位、多帧实体融合，并将持久化实体及其
候选停靠位写入同一语义地图格式。VLN/大模型只负责目标消歧和生成语义指令，
最终目标仍由本节点交给 3D A* 与 MPPI，从而保留地面支撑和底盘安全约束。
