# NX 容器与笔记本的 ROS 2 通信

## 2026-10-21：终端 RViz 自动加载 DDS 和独显设置

本机 `~/.bash_aliases` 中的 `rviz2` 启动函数现在为 RViz 子进程自动设置
`rmw_cyclonedds_cpp`，加载本工作空间的
`config/network/cyclonedds_laptop_wlo1_peer.xml`，并启用 NVIDIA PRIME 渲染。
默认 ROS 域为 0、允许局域网通信；显式设置的 `ROS_DOMAIN_ID` 与
`ROS_LOCALHOST_ONLY` 会保留。函数不会改变父终端或其他 ROS 节点的环境，
也不会修改路由或 NX 配置。它只作用于 Bash 中直接调用的 `rviz2`，
不拦截 `ros2 run`、`ros2 launch` 或绝对路径执行。

新 Bash 终端自动加载；已经打开的终端先运行：

```bash
source ~/.bash_aliases
rviz2 -d /home/zyhtxdy/go2-w_ws/src/fast_lio_nav2/config/offline_navigation.rviz
```

已经运行的 RViz 进程不会受到环境变量修改影响，需要保存显示配置并重新打开。
`/32` 专用路由仍需存在；本次只读确认路由正确，没有修改它。
原启动函数备份为 `~/.bash_aliases.before-rviz-dds-20261021`。

## 2026-10-19：切换 Cyclone DDS 后话题不可见的修复

已在笔记本复现：同一网络、Domain 0，Fast DDS 能发现机器狗导航话题；Cyclone DDS 默认配置只显示 `/parameter_events` 和 `/rosout`。到 NX 的 Wi-Fi 地址和 `192.168.123.18` 均能 ping 通，笔记本的 `/32` 专用路由正确，不能归因于路由丢失。当前 CLI daemon 已使用 Cyclone，不能归因于仍驻留 Fast DDS daemon。

之前只切换 RMW 并 `unset CYCLONEDDS_URI` 的命令缺少当前网络需要的发现配置，已补充笔记本专用配置 [cyclonedds_laptop_wlo1_peer.xml](../config/network/cyclonedds_laptop_wlo1_peer.xml)：

- 使用笔记本 `wlo1`，将 NX 的 `192.168.123.18` 设为发现 Peer。
- 设置 `ParticipantIndex=auto`，使 NX 的现有单播发现能找到本机的可预测端口。
- `AllowMulticast=spdp` 保留发现组播、使用户数据使用单播。仅启用 Peer/auto 而仍允许数据组播时，本次能发现话题并收到可视点云，但 `/odom` 一次 15 秒等候未收到；改为 `spdp` 后收到 `/odom`。这是本次观察，未做网络逐包归因。

**以下只在笔记本执行。关闭本人旧 RViz 后重新启动；不要将这份笔记本配置用于 NX。**

```bash
source /opt/ros/humble/setup.bash
source /home/zyhtxdy/go2-w_ws/install/setup.bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/zyhtxdy/go2-w_ws/config/network/cyclonedds_laptop_wlo1_peer.xml
unset ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
ros2 daemon stop
ros2 topic list --no-daemon --spin-time 8
rviz2
```

本次只在本机新增上述 XML 和更新本文档。没有修改 shell 启动文件、路由或 NX，也没有停止/重启导航节点、发送目标或运动指令。检查进程已退出。已有路由无需重复添加。若以后网卡名或 NX 地址变化，需要相应调整配置和路由。

核查结果：新配置下发现 `/odom`、`/plan`、`/global_costmap/costmap`、地形及点云等话题；实际收到 `/odom` 与 `/cloud_registered_viz` 的消息头。没有代为启动 RViz，也没有通过下发目标验证动作通信。

配置语义参见 [Cyclone DDS 0.10.5 配置定义](https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/0.10.5/etc/cyclonedds.xsd) 和 [发现机制](https://cyclonedds.io/docs/cyclonedds/latest/config/discovery-config.html)。

## 2026-10-19：RViz 下发目标时报 RTPS_READER_HISTORY

用户终端中每次 `Setting goal pose` 后出现：

```text
Change payload size of '32' bytes is larger than the history payload size of '19' bytes and cannot be resized.
```

本次本机只读检查确认 RViz 加载 `librmw_fastrtps_cpp.so`；最近一次 NX 运行环境记录为 `rmw_cyclonedds_cpp`。Fast DDS 的固定预分配接收缓存装不下该消息，会拒收它；不能通过增大点云订阅深度或改 Best Effort 解决。与目标同时出现，怀疑涉及导航动作底层服务的跨 DDS 通信，但尚未捕获具体出错端点，不能认定所有目标都未送达。

建议把本机 RViz 与 NX 的 DDS 实现统一，并使用上一节已验证的笔记本发现配置。**本次没有代为安装软件、关闭 RViz 或修改 NX。** 在自己的笔记本安装一次：

```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
```

关闭本人旧 RViz，再从已配置好网络的终端启动；本机路由仍按当前 NX 与笔记本的实际 IP 设置：

```bash
source /opt/ros/humble/setup.bash
source /home/zyhtxdy/go2-w_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/zyhtxdy/go2-w_ws/config/network/cyclonedds_laptop_wlo1_peer.xml
unset ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
rviz2
```

以上只改变此终端及它新启动的进程。先检查显示与报错是否恢复；目标是否被导航接收，应结合导航服务端日志判断。Wayland 提示和 `Stereo is NOT SUPPORTED` 不影响普通二维/三维单目显示。

依据：[Fast DDS 2.6.11 接收历史源码](https://github.com/eProsima/Fast-DDS/blob/v2.6.11/src/cpp/rtps/history/ReaderHistory.cpp#L57)、[ROS 2 跨 DDS 服务互通讨论](https://github.com/ros2/rmw_cyclonedds/issues/184)。

## 2026-10-09 历史排查记录

以下保留当时的环境和测试记录；其中独立验证安装路径不代表现在应使用的工作空间版本。

仅涉及本人 `zyh-nav2` 容器和自己的笔记本；没有修改 NX 宿主机路由、网卡、时间或其他容器。

## 现象与检查

- NX 的 eth0 是 `192.168.123.18/24`，wlan0 是 `10.56.61.177/21`。
- 笔记本 wlo1 是 `10.56.60.21/21`，能 ping NX 的 Wi-Fi 地址，但本机话题发现为空。
- NX CycloneDDS 的默认网卡选择日志显示 eth0；它对外公布的是内部有线地址。
- 同时启用 eth0、wlan0 时，笔记本能收到测试 String，但两个只读测试中 `/lowstate` 收到数量均为 0。
- 单独使用 eth0 的 8 秒对照收到 3992 条 `/lowstate`；增加笔记本 Peer 后的 6 秒对照收到 3001 条。
- 以上为各一次只读通信测试的观察值，0 趟运动；没有发送控制指令。

因此保留 eth0 作为 DDS 接口，通过 Peer 向笔记本做单播发现，并在笔记本为 NX 内部地址增加专用路由。
配置文件为 [cyclonedds_nx_eth0_peer.xml](../config/network/cyclonedds_nx_eth0_peer.xml)。
配置中的笔记本 Peer 地址需要随笔记本 IP 更新。

## 笔记本的临时路由

**以下命令只在自己的笔记本执行，不在 NX 或 NX 容器执行。**
我的会话没有笔记本 sudo 权限，需要用户在本人终端执行。

```bash
sudo ip route add 192.168.123.18/32 via 10.56.61.177 dev wlo1
ip route get 192.168.123.18
ping -c 3 192.168.123.18
```

目标必须经 `10.56.61.177`，不应再经实验室默认网关 `10.56.63.254`。
这是 NX 自己持有的地址，不需要修改 NX 转发配置。
如果提示 File exists，先检查已有路由，不直接覆盖其他用途的配置。
回退仅删除这条新增的专用路由：

```bash
sudo ip route del 192.168.123.18/32 via 10.56.61.177 dev wlo1
```

## NX 容器启动

当前 `/work/zyh_ws/install` 是旧版：两个桥接默认关闭、雷达时钟选项为 0。
检查时正在运行的实例使用了它，`/go2/imu` 和 `/go2/odom_raw` 发布者均为 0，`/odom` 没有收到数据。
需要在原终端退出本人这次旧 launch，再加载已编译验证的独立安装结果。
不要在加载此脚本后再 source 旧 install 覆盖它。

```bash
source /work/zyh_ws/setup_static_test.bash
ros2 pkg prefix fast_lio_nav2
ros2 launch fast_lio_nav2 odom_fusion.launch.py
```

包路径应为 `/work/zyh_ws/validation/static_mapping_20261009_1433/install/fast_lio_nav2`。
环境脚本现在加载本人容器内的 eth0 + 笔记本 Peer 配置，使用 ROS_DOMAIN_ID=0 和 CycloneDDS。
启动时保持静止，等待 IMU 初始化和零偏校正。
仅修改环境脚本不会改变已运行进程的 DDS 配置；需要重新启动本人节点。

## 笔记本查看

笔记本当前只有 Fast DDS 的 RMW，没有安装 CycloneDDS。
不要直接 export rmw_cyclonedds_cpp，否则提示找不到共享库。

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset CYCLONEDDS_URI ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE
ros2 daemon stop
ros2 topic list --no-daemon --spin-time 5
ros2 topic echo /odom nav_msgs/msg/Odometry --once --field header
```

RViz 必须从同样设置环境的终端重新启动；点云显示订阅选择 Best Effort。
首次只在网络测试参与者中验证了 Fast DDS 与 CycloneDDS 的 String 通信，不等于完整融合输出已在笔记本验证。
尚待用户添加笔记本路由、重启新版本人节点后检查 `/odom`、TF 和点云。
本次没有停止正在运行的本人旧 launch，也没有停止其他容器或机器狗服务。

DDS 默认单网卡选择和静态 Peer 机制参见
[CycloneDDS 网络接口说明](https://cyclonedds.io/docs/cyclonedds/latest/config/network_interfaces.html) 与
[发现配置说明](https://cyclonedds.io/docs/cyclonedds/latest/config/discovery-config.html)。
