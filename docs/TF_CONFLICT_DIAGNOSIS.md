# 局部代价地图坐标跳变排查（2026-10-12）

范围：SSH 到本人 `zyh-nav2`，读取当前进程、日志，并短时订阅 TF、里程计及地形数据。没有修改运行配置，没有启动、停止或重启业务节点，没有发布目标或运动请求。SSH 已断开。

## 证据

第二次成功只读采样约 8 秒，未进行运动测试：

- `/Odometry` 仅发现 `/laser_mapping` 发布；81 条样本位置约 `(-0.71, -0.20, 0.19)` 米。
- `/odom` 仅发现 `/odom_ekf` 发布；241 条样本位置约 `(-0.68, -0.21, 0)` 米。
- `/tf` 发现两个发布者：`/odom_ekf` 和 `/motion_service_bridge`。
- 同一条 `odom -> base_link` 采到 640 条变换，其中 240 条为近原点坐标，400 条为约 `(-1239.69, -790.55, 0)` 米。
- 近原点 TF 样例为 `(-0.67948, -0.21224, 0)` 米，与本方案 `/odom` 一致。
- `/terrain/clear_scan` 与 `/terrain/obstacles` 坐标系均为 `base_link`。
- 控制器日志显示局部地图中心在正常坐标和上千米坐标之间跳变。

以上为消息观察值，无外部位置测量。Humble 的 Python 接口本次未提供每条消息的发布者 GID，因此异常值来源由发现的两个 TF 发布者、变换内容及 EKF 消息对照推断；同名 TF 中两组互斥坐标交替出现已直接确认。

## 结论与后续处理

本次 Sensor origin out of map bounds 警告的直接原因是同名 `odom -> base_link` TF 冲突。该结论不验证运动建图精度，也不解决地面覆盖问题。

建议将本人链路的动态、静态 TF 分别重映射到 `/go2/nav/tf` 和 `/go2/nav/tf_static`。本人的融合、地形、Nav2 与 RViz 需要统一使用这两个话题；仅修改某一个节点不能完成隔离。

本次只完成排查，尚未实施 TF 隔离，不操作 `/motion_service_bridge`。

## 本人程序关闭后的复查

用户确认已关闭本人程序后，再次只读订阅约 8.00 秒：

- `/tf` 发布者仅剩 `/motion_service_bridge`，没有发现 `/odom_ekf`。
- 实际收到 362 条 `odom -> base_link`，位置约 `(-1239.65, -790.52, 0)` 米。
- 同时收到 362 条 `base_link -> imu_link`。

此时仅发现一个动态 TF 发布者，直接确认异常坐标变换仍由 `/motion_service_bridge` 持续发布，独立于已关闭的本人 EKF。节点的程序位置与作者仍未核实。没有停止或修改它；诊断订阅已退出，SSH 已断开。

## xujia 工作空间与运行来源复查

用户明确授权只读查找 xujia 工作空间后，核对结果：

- 工作空间 `/home/xj/machine_dog_docker/work` 存在。排除构建、安装、日志、录包与地图目录，搜索 1989 个文本文件；5 个文件提及 `motion_service_bridge` 或相关帧，没有找到该节点实现。
- `real_fullstack.sh` 第 4–8 行记录已有 `motion_service_bridge` 发布 `odom -> base_link` 及 `base_link -> imu_link`，并记录其链路使用不同帧名避开冲突。第 13 行将 `motion_app_v1` 运行列为前置条件。
- NX 上 `motion_app_v1` 容器运行中，镜像 `motion-service:v1-test`。`docker top` 显示 `/bin/bash /workspace/build/run_motion_service.sh` 与 `./motion_service` 两个进程。
- 进程的主机 PID 为 2831，容器内 PID 为 31；可执行文件 `/workspace/build/motion_service`。`/proc/2831/cgroup` 中的 Docker ID 与 `motion_app_v1` 的 ID 一致。
- 本次 ROS 图中 `/tf`、`/tf_static` 唯一发布节点均为 `/motion_service_bridge`。实际采到 `odom -> base_link`、`base_link -> imu_link`，以及静态 `imu_link -> lidar_link`，与截图完整链一致。
- 静态 `imu_link -> lidar_link` 平移观察值为 `(0.055, 0, 0.475)` 米；该服务值没有独立标定验证，不作为本人 FAST-LIO 外参依据。

综合正在运行的程序、容器归属、ROS 节点接口和 xujia 启动脚本的前置条件，来源定位为已有 `motion_app_v1` 运动服务；xujia 方案引用它，未在其工作空间发现节点实现。具体代码作者没有记录。

查询只读取目录、文件、Docker 元数据和进程信息，没有进入其他容器执行程序，没有修改任何远端文件或启动、停止任何业务节点。本人容器内的临时诊断订阅已退出，SSH 已断开。

## motion_app_v1 由用户关闭后的复查

用户关闭该容器后，只读检查结果：

- Docker 状态：`motion_app_v1 running=false status=exited`。
- 从本人 `zyh-nav2` 内、ROS_DOMAIN_ID=0、新建诊断订阅观察约 10.01 秒。
- `/tf` 和 `/tf_static` 均未发现发布者，未收到任何变换消息。
- 没有发现 `/motion_service_bridge` 节点。

该观察窗口内此前冲突的 TF 已不再发布。若旧 RViz 或 TF 图仍显示它，图像/缓存不能证明仍在发布；应使用新订阅重新核对。本人程序再次启动后的独占 TF 情况尚未在本次检查中验证。未启动、停止或修改任何业务节点，SSH 已断开。
