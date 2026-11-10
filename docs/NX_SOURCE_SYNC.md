# NX 源码同步记录

## 2026-11-10 全局近障碍接入（仅同步，未编译）

本人 `zyh-nav2:/work/zyh_ws` 增量同步 8 个源码、配置、测试和说明文件，
其中新增 1 个使用已有实验室地图的规划回归。保留源码属主 `1000:1000`。
近场真实回波同时进入局部障碍与全局实时实测层，保留物理 XYZ、原始观测时间
及既有 0.6 秒期限；未改变固定地图、全局碰撞阈值或已有局部绕行策略。
本机 89 项地形检查与新增真实地图规划软件回归通过，不代表真机避障成功率。

本轮仅同步源码，不编译、不更新正常 install、不启停节点，不发送目标或运动请求。
使用新版前需要在本人容器编译，再由操作者重新启动自己的程序。
未修改宿主机设置、地图或其他容器；同步时本人容器原本已运行，状态保持。

容器备份：`/root/zyh_sync_backups/global-near-obstacle-20261110_162500`。
本机清单：`/tmp/go2_nx_global_near_sync_20261110_162500`。

## 2026-11-10 近处障碍与清除射线修复（已编译安装）

仅更新本人 `zyh-nav2:/work/zyh_ws`。同步本轮 12 个源码、配置和测试文件，
原文件与本机修改前版本一致；保留源码属主 `1000:1000`。
增加独立于地面拟合的当前帧近场实测障碍识别；缺测清线使用 NaN，
两套 Nav2 配置的 `terrain_clear.inf_is_valid` 改为 false。

`terrain_costmap`、`navigation_bringup` 在容器中 Release 编译成功（约 60 秒）。
先安装到容器备份目录中的暂存位置，再原子更新正常 `install` 的 8 个必要产物，
保留正常环境 hooks；3 份安装配置与源码一致，AArch64 架构及动态库依赖核对通过。
继续使用 `/work/zyh_ws/install/setup.bash`，下次启动自己的程序后加载新版。

备份及构建记录：`/root/zyh_sync_backups/near-obstacle-20261110_152555`。
本机清单：`/tmp/go2_nx_near_sync_20261110_152555`。
本次未启动或停止 ROS 节点，未发送目标、速度或运动请求，未修改地图、宿主机设置
或其他容器。同步时容器已运行，未更改容器运行状态。

> 2026-11-02 时效修复：全局障碍期限统一 2 s、重定位后台结果 6 s／全局复核 12 s，并修复恢复原因反馈导致提前取消的问题。本机 795 项回归及 11 项录包执行链检查通过；本人容器已更新并核对 20 文件，未在 NX 编译或重启。见修改与同步记录（历史附件已在本机归档）。

> 2026-11-02 最新状态：本人 `zyh-nav2:/work/zyh_ws/src` 已更新 58 个文件（新增 14 个），范围内 222 个文件经独立哈希核对。保留现场局部膨胀 0.25 m，合入本轮已验证的规划与时效修复。未编译、未启停节点、未改地图或其他容器，SSH 已断开。见源码同步记录（历史附件已在本机归档）。

> 2026-10-28 首目标修复：修复行为树首次 path 未赋值导致立即 Goal failed 的回归。12 项本机检查通过，已在本人容器编译并原子更新正常 install 的 BT 插件，未重启业务节点或代发目标；需用户重新启动导航加载新库。见记录（历史附件已在本机归档）。

> 2026-10-28 角速度补偿后续同步：本人 `zyh-nav2` 更新 9 个文件，桥接、导航入口和离线入口三个包全部编译成功。默认正负对称补偿，导航现场参数和地图保持，未启动业务节点或发送运动请求，SSH 已断开。见记录（历史附件已在本机归档）。

> 2026-10-28 最新状态：本人 `zyh-nav2:/work/zyh_ws` 已更新 28 个文件，163 个范围内文件核对通过；新服务接口、规划器、行为树和地形/融合/离线入口共 6 包 Release 编译安装成功。保留现场参数，只补新版缺失字段，地图未动。未启动任何 ROS 业务节点；容器恢复为任务前停止状态，SSH 已断开。见同步与编译记录（历史附件已在本机归档）。

> 2026-10-26 后续地图上传：保护版已另存到 `zyh-nav2:/work/zyh_ws/maps/lab_v1_navigation_guarded_20261025`，6 个文件校验一致，确认与现有定位数据库配套；原图保留，未启停节点。使用时替换 `map_directory` 参数。源码尚未在 NX 编译。见地图上传记录（历史附件已在本机归档）。

> 2026-10-26 最新状态：本人 `zyh-nav2:/work/zyh_ws/src` 已同步最近修改的 55 个源码、配置与测试文件，包括固定地图校验、地形与规划修复、障碍观测时间和消费超时保护。核对 414 个范围内文件，413 个与本机一致；NX 独有的导航参数文件保留原样，局部膨胀半径仍为 0.25 m。已备份，未编译、未修改 install、未启停节点或发送运动指令，未上传地图，SSH 已断开。见同步记录（历史附件已在本机归档）。

> 2026-10-21 历史状态：已修复离线起点连接提前结束、目标外廓候选丢失及不安全捷径无安全回退的问题，并改进后台全局校验调度与诊断。本人 `zyh-nav2` 同步 18 个改动文件、61 个范围内文件校验一致；四包 Release 编译并更新正常 install。NX 在 1 秒跟踪期限下完整回放一遍，记录 255 次有效定位，无中途定位失效；没有启动运动接口。临时编译目录已清理，SSH 已断开。见本次修复与验证（历史附件已在本机归档）。下文保留历史批次记录。

本次通过 SSH，仅使用 `docker exec` 操作本人 `zyh-nav2` 容器的源码目录 `/work/zyh_ws/src`，以及容器内备份目录。
更新 25 个文件，410 个范围内源码文件经 SHA-256 校验与本机一致。
未编译，未启动、停止或重启 ROS 节点，未发送目标或运动指令，SSH 已断开。
未修改宿主机网络、DDS 配置或其他容器。

`/work` 是本人目录 `/home/unitree/zyh/work` 的已有挂载；通过容器更新的源码会保存在该挂载内。
没有写入宿主机的其他目录。

## 更新内容

- XT16 当前配置。
- FAST-LIO 融合入口及离线回放入口、配套文件。
- 全局规划器实际支撑高度与坡度判定修复。
- 地形模块的诊断统计、精确时间戳匹配修复及最长 100 ms 等待位姿。
- 配套 CMake、package.xml 与核心测试。

真机导航入口、Go2 IMU/里程计桥接、速度桥接与导航参数在同步前已与本机内容一致。
没有同步录包、build/install/log、FAST-LIO 调试日志、PCD、Git 历史或未接入主链路的重定位原型。
没有删除远端独有文件；仅有权限差异而内容相同的文件保持原样。

## 备份与生效

容器内备份：`/root/zyh_sync_backups/source-sync-20261012-094021`。
`previous-source.tar.gz` 保存被覆盖的原文件，`manifest.json` 记录更新清单及新文件。

新源码尚未编译安装。后续在本人容器中编译后，使用正常安装目录：

```bash
cd /work/zyh_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select fast_lio fast_lio_nav2 terrain_costmap astar_3d_planner navigation_bringup --parallel-workers 1 --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

不要加载旧的 `setup_static_test.bash` 验证安装目录。
源码一致性校验不代表本次已验证 NX 编译、真机建图或自主导航。

## 文件清单

- `src/FAST_LIO_ROS2/config/xt16.yaml`
- `src/fast_lio_nav2/CMakeLists.txt`
- `src/fast_lio_nav2/package.xml`
- `src/fast_lio_nav2/README.md`
- `src/fast_lio_nav2/launch/replay_planning.launch.py`
- `src/fast_lio_nav2/launch/odom_fusion.launch.py`
- `src/fast_lio_nav2/config/replay_planning.rviz`
- `src/fast_lio_nav2/scripts/rviz_goal_planner.py`
- `src/nav2/astar_3d_planner/CMakeLists.txt`
- `src/nav2/astar_3d_planner/README.md`
- `src/nav2/astar_3d_planner/include/astar_3d_planner/astar_3d_planner.hpp`
- `src/nav2/astar_3d_planner/include/astar_3d_planner/occupy_map.hpp`
- `src/nav2/astar_3d_planner/include/astar_3d_planner/astar_3d.hpp`
- `src/nav2/astar_3d_planner/src/astar_3d.cpp`
- `src/nav2/astar_3d_planner/src/occupy_map.cpp`
- `src/nav2/astar_3d_planner/src/astar_3d_planner.cpp`
- `src/nav2/astar_3d_planner/test/test_astar_3d.cpp`
- `src/nav2/terrain_costmap/CMakeLists.txt`
- `src/nav2/terrain_costmap/package.xml`
- `src/nav2/terrain_costmap/include/terrain_costmap/terrain_analyzer.hpp`
- `src/nav2/terrain_costmap/config/terrain_costmap.yaml`
- `src/nav2/terrain_costmap/src/terrain_costmap_node.cpp`
- `src/nav2/terrain_costmap/src/terrain_analyzer.cpp`
- `src/nav2/terrain_costmap/test/test_terrain_analyzer.cpp`
- `src/nav2/terrain_costmap/test/test_pose_waiting.py`

## 2026-10-13 增量同步

按用户要求更新本人 `zyh-nav2:/work/zyh_ws/src` 的 6 个文件。内容为前进上限 0.30 m/s、角速度上限 ±0.45 rad/s、对齐转速 0.35 rad/s；PathFollowCritic 的交接距离 1.2 m；速度桥接改成 vx/vy/wz 原样转换并删除四个限幅参数。

备份位于 `/root/zyh_sync_backups/source-sync-20261013-134900`。6 个文件均已做 SHA-256 一致性校验。没有编译、启动或重启任何业务节点，没有发送控制指令，没有更改宿主机配置或其他容器。SSH 已断开。

后续由用户在本人容器中安装本次涉及的两个包，再重启本人导航加载：

```bash
cd /work/zyh_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select navigation_bringup go2_cmd_vel_bridge --executor sequential --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

## 2026-10-13 地形与近场规划同步

本轮 15 个文件更新到本人 `zyh-nav2:/work/zyh_ws/src`，均通过 SHA-256 一致性校验。内容为地形负向异常恢复修复、固定世界网格、近场直线候选、共同高度基准补偿、时间戳配对缓存、累计支撑显示及回归检查。本次没有修改 EKF 配置。

容器内备份：`/root/zyh_sync_backups/ground-navigation-20261013-081904`。
仅更新源码和配置；未编译、未启停业务节点、未发送目标或速度，SSH 已断开。未修改宿主机配置或其他容器。完整文件清单及哈希见 source_sync.json（历史附件已在本机归档），编译和验证限制见 [地形与规划记录](PLANNING_QUALITY.md)。

## 2026-10-19 源码同步（未编译）

更新本人 `zyh-nav2` 容器 `/work/zyh_ws/src` 的 72 个新建或改动文件，其中新建 41 个。
同步范围的 461 个文件逐一通过 SHA256 一致性校验。包括 FAST-LIO 配置、Go2 里程计桥接、
融合和回放入口、地形处理、全局规划器、在线导航配置，以及新建的独立重定位／离线地图导航包。
离线入口仍独立，未合并在线入口；本轮没有另外调整速度参数。

容器内备份：`/root/zyh_sync_backups/source-sync-20261019-095334`。
`previous-source.tar.gz` 保存覆盖前的文件，`manifest.json` 记录原、新哈希及新建文件。
完整同步记录：source_sync.json（历史附件已在本机归档）。

没有删除远端独有文件；没有上传录包、工作区 build/install/log、Git 历史和 Python 缓存。
另跳过 `FAST_LIO_ROS2/doc` 中构建不需要的论文、演示 GIF 和结果图片，保留驱动 SDK 真正源码。
仅写入本人容器的源码目录及备份目录；`/work` 沿用已有的本人 `/home/unitree/zyh/work` 挂载。

本轮未编译，未修改安装目录，未启动、停止或重启 ROS 节点，未发目标或运动指令；
未修改宿主机配置或其他容器，SSH 已断开。源码同步校验不代表 NX 编译和真机测试已完成。

## 2026-10-19 在线规划改进增量同步（未编译）

按用户要求同步到本人 `zyh-nav2:/work/zyh_ws/src`：更新 14 个文件，其中新增 1 个支撑更新测试文件。内容为 FLOAT32 支撑边界修复、按连续高度关联与跨 Z 格重新索引、稳健地面拟合，以及坡度/台阶拒绝诊断和配套测试、说明。14 个文件均经过远端 SHA256 校验。

容器内备份：`/root/zyh_sync_backups/planning-support-20261019-104437`。
覆盖前的文件保存在 `previous-source.tar.gz`，清单为 `manifest.json`。
本机对应清单：source_sync.json（历史附件已在本机归档）。

没有同步笔记本专用网络配置，没有覆盖本轮改动范围外的文件，没有删除远端独有文件。未编译、未启停业务节点、未发送目标或运动命令，未修改 NX 宿主机配置或其他容器，SSH 已断开。当前运行进程仍使用原编译产物，需由操作者编译并重新启动自己的程序后加载新版。

## 2026-10-21 定位与矩形路径校验同步（未编译）

仅操作本人 `zyh-nav2` 容器的 `/work/zyh_ws/src` 和容器内备份目录。
同步前逐一比对本机、远端及历史校验值；本次待覆盖文件没有出现远端另行修改的冲突。
更新 22 个文件，其中 3 个为新建的矩形外廓检查源码、头文件和核心测试。
范围包括重定位的里程计先验、候选诊断、定位适配器提前确认，以及规划器的矩形平移／转向扫掠校验、配置和配套构建／测试文件。
同时补齐上一批未一致的地形分析头文件。

速度前馈补偿代码、默认开启配置与在线启动入口在远端已经与本机一致，未重复覆盖。
对重定位、离线导航、全局规划、导航入口、速度桥接、地形处理和融合入口 7 个包中的 106 个源码与配置文件完成 SHA256 一致性检查。
源码文件保留所属用户 `1000:1000`，避免容器写入后造成宿主机编辑权限问题。

备份位于 `/root/zyh_sync_backups/source-sync-20261021-101637`。
`previous-source.tar.gz` 保存覆盖前的 19 个已有文件；`manifest.json` 记录原、新哈希及 3 个新增文件。
本机完整记录：同步清单（历史附件已在本机归档）、校验结果（历史附件已在本机归档）。

使用新版源码中的地图加载函数只读检查容器现有 `maps/lab_v1_database` 与 `maps/lab_v1_navigation`：
配对签名和地图文件校验通过，坐标系为 `map`，891 个支撑点和 982 个障碍点。
这是文件完整性检查，不是地图质量或真机规划成功率测试。

未编译，未修改安装目录，未启动或停止节点，未发送导航目标、速度或运动请求。
没有修改 NX 宿主机网络、时间、服务及其他容器，没有删除远端文件，也没有上传笔记本专用网络配置。
SSH 已断开。新版需要由操作者在本人容器中编译、重新加载环境并重启自己的节点后才会生效。


## 2026-10-21 重定位后台调度改进（已编译）

只操作本人 `zyh-nav2` 容器，更新重定位后台搜索、最新扫描全候选核验、运行时新鲜度与取消代次保护、默认处理间隔、诊断及回归测试；离线回放验证工具增加独立参数文件入口。11 个文件更新，17 个范围内文件 SHA256 核对一致，属主保持 1000:1000。

`go2_relocalization` 和 `go2_offline_navigation` 在本人容器内 Release 编译通过，安装配置与源码一致。
编译前确认目标包没有运行节点；保存旧源码及安装产物于 `/root/zyh_sync_backups/relocalization-async-20261021-033833`。
未启动机器狗、传感器、建图、导航、控制或 ROS 测试节点；无目标和速度请求，未改动其他容器和宿主机配置。
仅运行读取保存地图与关键帧的无 ROS 离线基准。最终目标包无运行进程，SSH 已断开。

本机完成默认参数完整录包和严格新鲜度压力回放；NX 只做离线微基准，环境变化后的实时定位尚未验收。
结果和限制见 验证记录（历史附件已在本机归档）。


## 2026-10-21 NX 完整录包验证与回放工具同步

核心重定位源码已与 NX 一致，本轮核对 88 个源码/配置文件，只更新回放验证工具的 CPU/RSS 记录及显式关闭速度桥接配置，重编译 `go2_offline_navigation`。
备份 `/root/zyh_sync_backups/nx-full-replay-20261021-054130`。

使用独立 domain 128、仅 localhost、录包仿真时钟，完整回放既有 68.902 s 传感器包一遍。
启动软件建图、融合、地形处理、已知地图重定位和定位适配器，不启动实际驱动、Nav2 导航或物理速度桥接。
记录到 164 次不同时间戳有效定位，首次 ready 后至包结束无中途失效。详见结果（历史附件已在本机归档）。
测试运动接口发布者为 0；测试进程已退出；FAST-LIO 原有日志已恢复，本轮日志另存；结果已下载到本机。
没有修改其他容器或 NX 宿主机设置，SSH 已断开。此测试不代表真机导航验收。


## 2026-10-21 离线导航自动取消与支撑连接修复（已编译安装）

本人 zyh-nav2 中同步更新 10 个源码、配置、说明和回归测试文件；33 个范围内文件校验一致。
备份 `/root/zyh_sync_backups/offline-navigation-fix-20261021-064321`。
全局检索和全候选核验改为同一后台线程顺序执行，前台只做当前点云轻量跟踪；原有 3 s 新鲜度和保护保留。
离线支撑连接半径按地图导出网格对角间距适配，当前为 0.30 m，在线原配置 0.25 m 保持。
定位适配器新增失效原因与恢复日志，原速度门控及取消逻辑保持。

两包在暂存目录以 Release 编译；使用原子替换更新正常 install 的 7 个文件，保留环境 hooks。
暂存目录已清理。安装操作未停止/重启业务进程，更新时本人重定位业务进程已退出。
正常 source 和原离线启动入口继续使用，下次由操作者重新启动后加载新版。

完整 NX 录包测试 1 遍：跟踪超时收紧为 1 s，首次 ready 4.462 s，237 次不同时间戳有效定位，
轻量跟踪最大约 94 ms，后台计算最大约 2.58 s，期间无跟踪超时。
测试关闭真实驱动、Nav2 导航和运动桥接，运动发布者为 0；全部测试进程已退出，原 FAST-LIO 日志恢复。
没有改动宿主机或其他容器，没有发送运动指令；SSH 已断开。

详见验证记录（历史附件已在本机归档）。
