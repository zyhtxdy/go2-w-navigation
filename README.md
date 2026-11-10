# Go2W 激光建图与导航工作空间

面向宇树 Go2W 的 ROS 2 Humble 工作空间，使用禾赛 XT16、本体 IMU 和运动状态，支持在线建图导航及已知点云地图导航。方案从 Scout Mini 工作空间移植，并持续修改地形处理、全局规划和机器狗接口。

当前属于开发测试版本。窄通道通过、动态绕行及重规划衔接仍在验证；部署参数应以对应设备的安装、标定和测试结果为准。

## 模块

| 目录 | 作用 |
| --- | --- |
| `src/FAST_LIO_ROS2` | 修改版 FAST-LIO2，含 XT16 预处理和重力找平 |
| `src/driver/HesaiLidar_ROS_2.0` | 禾赛 ROS 驱动及随仓保存的 SDK |
| `src/driver/go2_imu_bridge` | 从 `/lowstate` 提取本体 IMU |
| `src/driver/go2_odom_bridge` | 将宇树运动状态转换为线速度观测 |
| `src/driver/go2_cmd_vel_bridge` | Twist 转宇树运动请求，含速度补偿与超时处理 |
| `src/driver/unitree_go`、`src/driver/unitree_api` | 宇树消息定义 |
| `src/fast_lio_nav2` | 位姿参考点转换、IMU 零偏处理、EKF、在线入口及回放 |
| `src/nav2/terrain_costmap` | 地面拟合、支撑面提取及实时障碍处理 |
| `src/nav2/astar_3d_planner` | 支撑高度层上的全局 A* 与矩形扫掠检查 |
| `src/nav2/terrain_navigation_interfaces` | 带原因分类的路径检查服务接口 |
| `src/nav2/navigation_bringup` | Nav2、行为树、Rotation Shim 和 MPPI 配置 |
| `src/go2_relocalization` | 已知点云地图检索与 GICP 重定位 |
| `src/go2_offline_navigation` | 固定地形地图、定位适配和离线导航入口 |
| `src/nav2/semantic_navigator` | 独立的语义地点任务模块，默认导航入口不启动 |

## 编译

先安装 ROS 2 Humble、colcon 和 rosdep，并完成本机的 rosdep 初始化。在工作空间根目录执行：

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source install/setup.bash
```

项目内的 FAST-LIO、ikd-Tree、禾赛驱动和 SDK 作为普通源码保存，不需要额外拉取子模块。来源及本次整理前的上游版本见 [THIRD_PARTY.md](THIRD_PARTY.md)。

## 运行入口

每个终端先加载 ROS 和本工作空间的 `install/setup.bash`。机器人端保持静止，等待惯性初始化；确认设备地址、安装外参及输入话题后再进行运动测试。

在线模式需要两个终端：

```bash
# 终端 1：传感器、FAST-LIO2、融合和地形处理。
ros2 launch fast_lio_nav2 odom_fusion.launch.py
```

```bash
# 终端 2：Nav2。此入口默认启动运动指令桥接。
ros2 launch navigation_bringup real_nav2.launch.py
```

离线模式使用独立入口，不与上述在线链路同时启动：

```bash
# 定位数据库与导航地图须配套；路径替换为本机实际目录。
# 默认不启动运动桥接，需要执行导航时再显式设置 start_cmd_vel_bridge:=true。
ros2 launch go2_offline_navigation offline_nav.launch.py \
  database:=/absolute/path/to/database \
  map_directory:=/absolute/path/to/navigation_map
```

地图生成、定位检查、录包回放与 RViz 配置见：

- [在线链路与本机回放](src/fast_lio_nav2/README.md)
- [离线地图准备及导航](src/go2_offline_navigation/README.md)
- [重定位模块](src/go2_relocalization/README.md)
- [XT16 设置](src/FAST_LIO_ROS2/XT16_SETUP.md)
- [网络配置示例](config/network)

网络 XML、雷达 IP 和部分文档保留开发设备示例，换电脑或设备时需核对。源码参数也不代表 NX 现场运行参数已经同步。

## 迁移到 Unitree B2-W

当前代码和参数适配的是 Go2W，尚未完成 B2-W 适配或真机验证。FAST-LIO2、EKF、地形处理、全局 A*、重定位及 Nav2 的整体链路可以继续使用；B2-W 的消息接口、安装关系、车体尺寸和运动响应需要重新核对。下表是迁移时的修改位置，不包含未经测量的 B2-W 参数。

### 接口、外参与运行参数

| 修改项 | 代码或配置位置 | B2-W 上需要做什么 |
| --- | --- | --- |
| 本体 IMU 输入 | [go2_imu_bridge](src/driver/go2_imu_bridge) | 确认实际状态话题、消息包及字段；核对加速度/角速度单位、轴向、四元数约定和时间戳。若类型不同，修改消息导入、订阅与转换代码，只改话题名不够。 |
| 本体线速度输入 | [go2_odom_bridge](src/driver/go2_odom_bridge) | 确认运动状态接口、速度属于机身系还是里程计系、速度参考点和时间源；据此设置 `velocity_frame`、参考点 TF 和协方差。不能将 Go2W 录包上的坐标解释直接当作 B2-W 的结论。 |
| 运动请求接口 | [go2_cmd_vel_bridge](src/driver/go2_cmd_vel_bridge) | 核对消息类型、服务/话题、请求编号、速度轴向、模式要求和停止语义；验证人工接管与超时停止行为。当前 Go2 接口兼容性需要实机确认。 |
| 雷达驱动 | [hesai_xt16.yaml](src/FAST_LIO_ROS2/config/hesai_xt16.yaml) | 使用实际雷达的 IP、UDP 端口、校准文件和时间源；确认点云字段、逐点时间及实际转速。 |
| 雷达到本体 IMU 的外参 | [xt16.yaml](src/FAST_LIO_ROS2/config/xt16.yaml) | 重新标定 `mapping.extrinsic_T`、`mapping.extrinsic_R`。方向约定为 `p_IMU = R × p_LIDAR + t`；当前 `[0.1710, 0, 0.0908]` 与绕 Z 轴 +90° 是 Go2W 配置，不能当作 B2-W 安装值。 |
| 本体 IMU/速度参考点到车体的 TF | [odom_fusion.launch.py](src/fast_lio_nav2/launch/odom_fusion.launch.py) | 重新设置 `base_to_imu_*`、`sportstate_reference_point` 及相关 frame；当前 `[-0.02557, 0, 0.04232]` 是 Go2W 模型参考值。保证 `odom→base_link` 由 EKF 独占，离线 `map→odom` 由定位适配器独占。 |
| 地面高度基准和车身净空 | [nav2_mppi_params.yaml](src/nav2/navigation_bringup/config/nav2_mppi_params.yaml)、[offline_nav.launch.py](src/go2_offline_navigation/launch/offline_nav.launch.py) | 按 B2-W 实际工作姿态核对 `lio_height_offset` 和 `GridBased.robot_height`。前者连接 LIO 的 IMU 高度与规划地面参考，后者用于净空检查；当前 `-0.50` 和 `0.75` 不能按机型直接沿用。 |
| 全局/局部车体外廓 | [nav2_mppi_params.yaml](src/nav2/navigation_bringup/config/nav2_mppi_params.yaml) | 同时更新 local/global costmap 的 `footprint`、padding，并核对全局矩形扫掠和局部碰撞检查。按实际轮腿、载荷和传感器外伸确定包络，不能沿用 Go2W 的 `0.80 × 0.50 m`。 |
| 地形判定和感知范围 | [terrain_costmap.yaml](src/nav2/terrain_costmap/config/terrain_costmap.yaml)、上述 Nav2 配置 | 重新核对输入高度范围、近距/自体过滤、坡度、台阶、障碍高度和起点连接范围；雷达安装变高不代表可以把脚下未知区域都判为可走。当前 XT16 `blind=0.38` 也需重新检查。 |
| 速度、转向与控制器 | [nav2_mppi_params.yaml](src/nav2/navigation_bringup/config/nav2_mppi_params.yaml) | 按 B2-W 响应调整 MPPI/平滑器速度及加减速度、采样范围、预测时域、Shim 阈值和转速，并核对膨胀层。约 1.1m 通道的通行能力取决于实际外廓和转向空间，不能仅通过缩小膨胀半径认定可通过。 |
| 速度前馈补偿 | [go2_cmd_vel.yaml](src/driver/go2_cmd_vel_bridge/config/go2_cmd_vel.yaml) | 当前正向与角速度补偿来自 Go2W。迁移初期关闭 `forward_compensation`、`angular_compensation`，重新记录指令与速度响应后再拟合 B2-W 系数；EKF 反馈不等于外部测速真值。 |
| EKF 和输入时效 | [ekf.yaml](src/fast_lio_nav2/config/ekf.yaml)、桥接和定位配置 | 保留明确的观测选择，重新核对测量延迟、噪声与协方差、静止初始化和处理负载；接收时间戳不能证明雷达/IMU 硬件同步。 |
| 网络与算力 | [config/network](config/network) | 替换实际网卡、设备地址、DDS Peer 和 ROS 域，核对容器网络；在 B2-W 实际计算设备上检查完整链路负载，不能照搬现有 NX 的耗时预算。 |

已有包名及 `/go2/...` 工程话题可以暂时保留，但这些名称不证明接口已经兼容 B2-W。若要改名，需要一起修改订阅者、发布者、launch、TF 和回放配置，避免仅修改标签而未转换数据。

### 启动入口需要一并适配

`odom_fusion.launch.py` 已提供部分 IMU 外参及运动状态参考点参数，但当前 `offline_nav.launch.py` 没有将这些参数全部声明并透传。迁移时应补齐透传或提供独立的 B2-W 启动入口，不能假设给顶层 launch 增加同名参数就会覆盖内部 Go2W 默认值。

在线入口 [real_nav2.launch.py](src/nav2/navigation_bringup/launch/real_nav2.launch.py) 和离线入口都提供速度补偿开关。完成 B2-W 接口验证前使用 `start_cmd_vel_bridge:=false`；后续启用运动桥接时，未重新标定的 Go2W 补偿保持关闭。

### 如果使用前 XT16、后 Mid360

当前主入口按单个 XT16 点云流与本体 IMU 配置，尚未接入这套双雷达方案。需要另行接入 Mid360 驱动，核对两路时钟、逐点时间与安装外参，并设计相应处理链路。不能直接拼接两路点云后作为单个 XT16 扫描输入现有 FAST-LIO；用于局部避障的多雷达数据接入也要明确坐标变换、时间和自体过滤。

### 到隧道后的建图与复用顺序

1. 先验证 B2-W 原始点云、本体状态、时间基准、安装 TF，再验证静止及遥控运动中的 FAST-LIO/EKF 输出。
2. 同一次建图会话运行 `record_database.launch.py`，采集隧道关键帧；关闭前调用保存服务，确认保存成功。FAST-LIO 重启后不能继续把新原点数据当作同一个会话。
3. 用 `build_terrain_map` 从该定位库导出配套导航地形图，检查支撑连续性及障碍分类。现有实验室 `lab_v1_*` 地图保留作回归数据，隧道使用新目录和新库。
4. 关闭运动桥接，检查重定位、`map→odom→base_link`、实时地形和全局路径，再安排运动测试。当前库不做回环优化，采集时的漂移会进入保存地图。
5. 分别验证前进、转向、临时障碍绕行、停车与恢复。原路倒退仍需独立规划/控制配置；允许负速度本身不能保证在窄隧道中可靠倒退巡检。

定位数据库、导航地形图的生成命令见[离线导航说明](src/go2_offline_navigation/README.md)。本节仅记录迁移工作项，没有修改当前 Go2W 的程序、参数或测试行为。

## 测试与本地数据

`src/*/test`、`src/nav2/*/test` 和驱动包内的 `test` 保存可复用的回归测试及小型数据夹具，随源码保留。构建时默认可通过 `BUILD_TESTING=OFF` 跳过 CMake 测试；需要检查时重新开启：

```bash
colcon build --executor sequential --cmake-args -DBUILD_TESTING=ON
colcon test --executor sequential
colcon test-result --verbose
```

`validate_replay`、`replay_planning.launch.py` 等仍用于本机无真机调试，保留在对应包中。历史验证文档、一次性排查脚本、旧源码快照、日志及结果已移到工作空间外归档。`docs/` 中的现场排查与整理记录仅保留本机，不随仓库发布。

`bags/`、`build/`、`install/`、`log/` 在本地保留，由 `.gitignore` 排除。`maps/` 默认忽略；当前已显式收录配套的 `lab_v1_database` 和 `lab_v1_navigation_guarded_20261026`，其余地图保留在本地。克隆代码后需要自行编译；录包及其他场地地图需另行准备。

## 仓库整理

提交源码和文档变更前可检查：

```bash
git status --short
git add --dry-run .
```

