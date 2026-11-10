# Unitree Go2 消息定义

来源：宇树官方 [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2)。
固定提交：`668d1ec5a05d1c38d3306bdca7d59f2ba3581a88`。
原目录：`cyclonedds_ws/src/unitree/unitree_go`。

完整复制上游 CMake 中列出的 24 个 `.msg`，没有删减 `LowState` 字段。
文件来源和逐文件 SHA-256 在 `UPSTREAM.json`，许可证在 `LICENSE`。

本工作空间只调整了构建文件：将额外的 Connext DDS IDL 生成设为可选，
普通 ROS 2 Humble 使用标准 `rosidl_generate_interfaces` 生成消息。
包描述也更新了。消息内容保持上游原样。

此包只定义消息，不运行雷达、底盘驱动或控制节点，也不需要另行编译 Cyclone DDS。
实际机器人固件是否与这个上游版本兼容，仍需后续检查。
