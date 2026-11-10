# 2026-11-02 NX 源码同步完成

仅操作本人 `zyh-nav2`（容器 ID a1b5c94d58a8）及 `/work/zyh_ws/src`、容器内备份目录。容器原来处于运行状态，保持运行；未启动、停止或重启业务节点，没有发布目标、速度或运动 API 请求，未编译。SSH 已断开。

## 同步范围

检查 11 个相关包的 222 个源码、构建、配置和测试文件；更新 58 个，其中新增 14 个。包括上次尚未部署的规划引导、障碍范围精化及本轮一致性修复，避免只同步最后 34 个文件而遗漏前置依赖。

检查的包为 fast_lio、fast_lio_nav2、Go2 IMU/里程计/速度桥接、terrain_costmap、terrain_navigation_interfaces、astar_3d_planner、navigation_bringup、go2_offline_navigation、go2_relocalization。内容一致的文件保持原样。

固定地图服务器、离线启动代码与 C++ 规划器一同更新，包含 16 字节障碍来源消息及 schema 2 身份校验。源包 test/data/ 中的回归夹具随源码同步；实际 maps/ 地图没有上传或修改。

## 参数与权限

保留 NX 现场局部 inflation_radius=0.25 m；同步已验证的 support_inference_radius=0.4 m、两路 expected_update_rate=0.8 秒及 7 个 guidance/范围精化字段。其余远端参数语义保持不变；具体 10 项变更见 parameter_changes.json。

已有文件 UID/GID 与权限均保持原值；新增源码及目录使用 1000:1000，避免此前无法编辑的问题。没有删除远端独有文件，没有改 DDS、网络、宿主机配置或其他容器。

## 校验与备份

58 个更新文件及范围内共 222 个文件，均经写入后及独立重连两次 SHA-256 校验。58 个更新文件的属主权限另行核对。

对 build/install/log/maps 共 8969 个条目的元数据快照前后一致。同步前后本人容器内均未发现业务进程；未启动节点做测试。

容器内备份：`/root/zyh_sync_backups/source-sync-20261102-949cf4b2`。

- previous-source.tar.gz：覆盖前原文件。
- manifest.json：变更计划、原权限及预期哈希。
- sync_receipt.json、source_verification.json：写入及独立核对记录。

本机同目录保留清单、参数合并结果和校验证据。

## 生效状态

本次只同步源码，安装目录保持原样；未完成 NX 编译或真机验证。后续在本人容器编译后，再 source 正常 install 使用。

```bash
cd /work/zyh_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export MAKEFLAGS='-j2 -l2'
colcon build --packages-select fast_lio go2_imu_bridge fast_lio_nav2 astar_3d_planner navigation_bringup go2_offline_navigation --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source install/setup.bash
```

以上编译命令仅供后续使用，本次没有执行。
