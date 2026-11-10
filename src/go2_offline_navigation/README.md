# Go2 离线地图导航（本机初版）

独立入口：`ros2 launch go2_offline_navigation offline_nav.launch.py`。
原有在线建图导航命令和默认参数保持原行为，尚未合并模式开关。
已做本机和 NX 隔离录包验证；各次改动的历史验证记录已移到本机归档，不代表已完成隧道导航验收。

## 链路和坐标

- 两种模式复用 FAST-LIO2、EKF、本体传感器桥接和实时地形处理。
- 离线模式关闭 `odom_fusion.launch.py` 的恒等 `map→odom`；新定位适配器独占发布动态变换。
- 原重定位节点继续只发布普通结果，不直接发 TF；适配器检查结果和诊断的测量时间一致。
- EKF 继续独占 `odom→base_link`；body、雷达和本体 IMU 安装关系沿用现有配置。
- 全局 A* 从固定 `map` 支撑面及静态障碍规划，默认接收 `/terrain/obstacles_measured_global` 的实时实测障碍。
- 局部代价地图继续在 `odom` 下处理 `/terrain/obstacles` 与 `/terrain/clear_scan`。

适配器保持原重定位计算的完整 map→odom 变换，检查同时间 LIO 与 EKF 的平面差异，
默认拒绝大于 0.2 m / 0.15 rad 的差异。EKF 二维高度和 LIO 三维高度不强行相等；
规划高度仍由原始 LIO 变换到 map 后加 `lio_height_offset`。这些门槛是初始设置，未标定。
当前轮式二维导航与三维定位在真实坡道、找平误差下的相容性仍需真实回放验证。

## 准备地图

先用已有建图链路采集关键帧数据库（建议同一次会话完成），另开终端：

```bash
ros2 launch go2_relocalization record_database.launch.py output:=/绝对路径/tunnel_database
# 采集结束前保存；关闭节点不会自动保存。
ros2 service call /relocalization_recorder/save std_srvs/srv/Trigger '{}'
```

地图库包含 map.pcd、manifest.csv、keyframes/。记录器尚不校正采集轨迹漂移。
再从这张地图生成固定导航地形：

```bash
ros2 run go2_offline_navigation build_terrain_map \
  /绝对路径/tunnel_database /绝对路径/tunnel_navigation
```

工具在重叠分块内复用现有地形拟合算法，导出 support.csv、obstacles.csv、navigation_map.json。
导航地图沿用数据库 map 的原点、轴向和高度，没有人为补齐未观测地面。
地图 SHA256 与关键帧位姿清单 SHA256 会写入元数据；启动拒绝混用不同地图库。
静态危险点包含真实障碍及地形拟合判为不可通行的点；需要在 RViz 检查分类。
新导出同时保存 `hazard_sources.csv`，逐点标明 `measured`（实测高出拟合面的点）或
`synthetic`（坡度、拟合残差或台阶检查生成的危险标记），元数据记录两类数量。
`terrain_tiles.csv` 保存每块的拒绝原因统计。格子统计含重叠拟合边缘，不能直接相加
作为全图唯一格子数；`owned_*` 是不重叠的实际导出计数。这些旁文件不改变规划输入。
`terrain_cells.csv` 保存每个实际观测格的高度范围、拟合误差、坡度、地面支持回波数、
最终拒绝原因及合成标记状态；格子按所属块去重，旁文件摘要也参与导出文件校验。

稀疏地面可另导出候选地图进行对照：

```bash
ros2 run go2_offline_navigation build_terrain_map \
  /绝对路径/tunnel_database /绝对路径/tunnel_navigation_candidate \
  --recover-sparse-ground-layers
```

此选项只作用于这次离线导图。当原拟合未通过坡度或残差检查时，从邻格的真实回波中
寻找至少覆盖 60% 独立观测格的地面层，并重新检查支持数、坡度、残差和台阶。
每个导出支撑格仍须有该地面层的实际回波，不补未观测地面，不覆盖旧地图。
恢复平面也不能高于本格最低实际回波超过地面一致性容差，含明显低层的格子保留原
拒绝结果，避免把真实陡坡或台阶上方的平层当成地面。
实时地形处理默认保持原算法。候选地图的障碍分类仍需检查，不能只按支撑点数量选择地图。
工具不会在分块中心执行机器人近场虚拟障碍清除。当前只支持每个 XY 格的单个支撑面，
不适合上下层道路重叠。地图原有漂移、稀疏支撑和错误分类不会因保存而消失。
输出目录必须不存在。地图是只读版本；要更新地图，导出到新目录并重启离线入口。

## 启动

```bash
source /opt/ros/humble/setup.bash
source /home/zyhtxdy/go2-w_ws/install/setup.bash
ros2 launch go2_offline_navigation offline_nav.launch.py \
  database:=/绝对路径/tunnel_database \
  map_directory:=/绝对路径/tunnel_navigation
```

默认启动传感器、里程计、重定位、地图加载、定位适配器和 Nav2，**不启动运动桥接**。
明确指定 `start_cmd_vel_bridge:=true` 才会通过 `/api/sport/request` 输出运动请求。
重定位候选默认双线程；可用 `localization_candidate_threads:=1` 控制计算并行度。
NX 已有无控制器的隔离回放负载记录；与实时地形及控制器并行时仍需评估。
此参数不能与 `use_sim_time:=true` 一起使用。
已存在输入发布者时使用 `lidar_driver:=none`、`start_go2_imu_bridge:=false`、
`start_go2_odom_bridge:=false`，避免重复驱动和桥接。
`start_odometry:=false` 用于已有 FAST-LIO/EKF/地形链路；外部链路必须关闭恒等 map→odom，
例如 `odom_fusion.launch.py publish_map_odom_tf:=false`。不要同时运行在线导航入口。

离线全局规划的实时输入可用 `live_obstacle_topic` 显式指定。默认实测障碍用于更新人或
物体的占用，固定地图保留建图时的坡度、台阶及障碍约束。局部代价地图继续接收完整
实时地形危险点；对照旧输入时可设 `live_obstacle_topic:=/terrain/obstacles_global`。

近场竖直实测回波即使缺少地面拟合，也同时进入局部障碍和全局实测流。
全局使用输入帧的 FAST-LIO 位姿及安装外参转换真实 XYZ，保留逐点原始观测时间，
不使用局部障碍云为 marking 重写后的相对高度 Z。检测仅使用当前帧，已检测回波
沿用下述实测障碍期限，短时缺测不会立即消失，过期不会由两秒地形历史重新生成。
首次规划检查这些实时障碍；已有参考线仍按原局部绕行预算复核，不因本次补障
而改成每次遇到临时障碍都强制重规划。此分支不生成支撑面，不改动固定地图文件。

地面拟合继续使用约 2 秒历史。`measured_obstacle_decay_time` 单独限制全局实测障碍的
输入配准扫描的源观测年龄，默认 `0.6` 秒；在 `offline_nav.launch.py` 或 `odom_fusion.launch.py`
中可覆盖，必须大于 0 且不超过地面历史时长。对照原 2 秒窗口可设为 `2.0`。
它不会增加实测障碍的入场等待，不改变局部完整危险分类、固定地图、外廓或碰撞距离。
0.6 秒是待真机验证的初始值；实际撤销还受处理和发布周期影响，并非严格的墙钟截止。
实测全局障碍随每次地形处理刷新，不再等待 `terrain_publish_interval`；支撑面和混合危险流
仍按原地形发布周期输出。

全局实测点云增加 `observation_sec` / `observation_nanosec`，来自输入
`/cloud_registered_body` 的 header，即 FAST-LIO 当前扫描结束时间。
它与原始 `/hesai/points` 的 header 分开记录，不把两者视为相同时间定义或硬件同步证明。
点云 header 仍表示本次分析时刻。规划失败诊断中的 `source_observation_stamp_ns`
对应实际返回点的时间；旧点云没有这些字段时明确记为 0。混合全局危险流的实测多帧
确认只计不同配准扫描的源时间前进，重复分析历史点不会刷新确认或保留时间；局部避障仍使用
完整地形历史。常规地形的空间簇检测仍会利用地形历史；独立近障碍簇只使用当前帧，
两者都不是物体类别或运动状态识别。
地面结构发生变化后，需要检查实时局部地形并更新离线地图。

离线导航还会检查规划器是否成功消费实时障碍：`/planner/live_obstacle_status` 每约 100ms
发布一次，包含 `ready`、`reason`、`consumed_stamp_ns`、`processed_age_s`、`successful_updates`。
只有坐标转换及内部地图更新成功才续期；新鲜空点云是正常更新，重复旧戳、转换失败或旧 epoch
不会续期。源时间和实际成功处理的稳态时间共用 `live_obstacle_freshness_timeout`，默认 2 秒；
状态报告心跳为 1 秒，检测与传输调度延迟另计。
断流时保留最后一帧占用并拒绝新规划、拒绝路径有效性检查，不将断流当成空障碍。

`/offline_localization/localization_available` 只表示定位与里程计可用，供规划器输入处理和显示；
`/offline_localization/planning_ready` 还要求全局障碍消费健康、本地地形和地图通过，供规划器
规划及复核路径；最终 `/offline_localization/ready` 再要求运动保护和恢复路径复核通过。
短时数据过期先停车并保留任务，默认最多等待 10 s（`input_recovery_timeout`，稳态时钟）。
行为树的输入不可用等待为 12 s，避免在定位连续确认和路径复核完成前先结束任务。
等待预算从第一次异常开始，原因变化、坏帧及请求重试均不能续期；等待期间始终零速。
恢复需要全部原有时效检查通过、map→odom 无超限跳变，且适配器对当前 `/plan` 的独立
`/terrain/is_path_valid_detailed` 请求返回有效；期间收到的速度丢弃，不缓存重放。
路径更换后不接受旧请求结果。持续失效、地图身份冲突、时钟回退、歧义定位和超限位置修正仍取消任务。
规划器的定位可用性心跳统一为 1 s，并区分未收到、过期、时钟异常与显式 false；
只有 adapter 按实际故障原因决定等待还是取消，不把反射回来的可用性状态当成新的硬故障。
单个格式错误、错帧或乱序消息只丢弃，不能刷新最后有效消息的接收和源时间；
最后有效数据仍须通过原有里程计 0.5 s、本地地形 1 s、当前定位 3 s 等时效检查。
全局匹配结果 6 s、全局复验 12 s、本地代价地图观测 0.8 s 的预算保持不变。
这样首次启动和断流恢复不会形成互相等待。只启动定位、不启动任何规划器时，最终 ready
保持 false 属于预期，已验证的 map TF 仍可显示。在线入口不使用这项离线门控。

本机只检查地图和定位，可加 `start_odometry:=false start_navigation:=false`，
并由独立回放提供 /Odometry、/cloud_registered_body、/lio/odom_base、/odom、地形及 TF。
离线已知地图模式与已有 `replay_planning.launch.py` 的在线算法录包回放不同。

## 定位及运动状态

```bash
ros2 topic echo /offline_localization/ready
ros2 topic echo /offline_localization/status
ros2 topic echo /relocalization/status
```

只有重定位有效、适配器连续确认、地图心跳、EKF、实时地形与规划器全局障碍消费均新鲜时才 ready。
适配器订阅 `/relocalization/candidate_map_to_odom`，仅在诊断给出的
`qualified_candidate_stamp` 匹配时提前检查候选。两边仍各自检查连续性，
适配器同时检查同时间 LIO/EKF 一致性；上游没有确认 valid 前，不发布定位 TF
或宣布导航就绪。这样减少重复等待，不减少所需帧数，不放宽质量及新鲜度门槛。
定位有效性和导航就绪分别判断：只丢失实时地形时仍更新定位 TF，方便 RViz 观察；
导航速度会阻断。定位结果或心跳过期时，停止更新地图定位 TF 并重新确认。
失效立即阻断速度并发送零速。明确的缺失/过期原因才允许上述有界等待；未知原因不等待。
等待期间到达的活动任务状态不会触发另一条“立即取消”分支。超时或严重错误取消后，
恢复仍要等待取消服务确认、旧目标结束，并连续至少 `command_timeout`（默认 0.3 s）
没有非零速度输入。取消被拒绝、服务失联或旧速度持续发布时，保持暂停并给出原因。
取消截止时间固定在故障时刻，重试不会连同之后的新目标一起取消；已知旧目标在时钟
回退后通过目标 ID 取消。适配器启动时发现旧活动目标也会先处理取消。
跨时钟取消若缺少旧目标 ID 或完成证据，会保守暂停并提示重启 Nav2 与定位适配器；
不会用旧时间戳或“取消全部”猜测目标归属。首次就绪前也要求旧非零输入先停止。
确认旧任务结束后重新下发目标，不自动接着执行旧任务。
默认大于 0.2 m 或 0.15 rad 的 map→odom 修正需要至少 3 个新结果重新确认。
确认帧之间超过 3 s 会重新计数；暂停前的确认不会直接接着用于恢复。
适配器监听仿真时钟切换／回退，并清理 TF 缓存和旧 LIO 历史。错误诊断保留具体拒绝原因。
适配器内部按 ROS 测量时间和墙钟心跳同时检查新鲜度；暂停回放也可能暂停导航。
物理桥接沿用原有命令超时机制；此软件流程不保证断电或底层执行器故障时能停车。

规划器在离线模式不累计实时支撑、不做在线高度漂移补偿、不复用旧路径。
固定障碍不会被实时清除。实时全局障碍采用最新地形窗口替换；未保留离开窗口后的
临时障碍历史。map→odom 修正应用到当前 odom 地形窗口，不把不同定位修正下的
历史障碍永久混合。定位失效会清除实时叠加，固定地图保留。
恢复后的实时障碍使用新的输入时间基线；先前已在处理的旧帧不能覆盖新的动态窗口。
离线实时全局障碍在接收和提交时都检查消息时间，拒绝未来或超过 1 s 的帧，
避免回放倒退后迟到的旧消息再次推高时间基线。固定地图不受这一有效期限制。
静态支撑输入对 FLOAT32 序列化造成的网格边界微小误差稳定体素键，避免规则 0.2 m
地形样本在 0.1 m 规划网格里出现假的 0.3 m 断连；在线支撑输入及查询也共用该边界修正。

## RViz

笔记本加载与 NX 对应的 ROS 网络环境后，直接打开离线专用配置：

```bash
rviz2 -d /home/zyhtxdy/go2-w_ws/src/fast_lio_nav2/config/offline_navigation.rviz
```

配置默认显示完整已知地图、固定支撑、固定障碍、当前匹配扫描、里程计及全局路径。
前三种静态点云使用 Reliable、Transient Local，晚启动 RViz 也能收到已经发布的地图；
完整地图需要离线入口指定 `publish_known_map:=true`。固定点云本身位于 `map`，
因此即使定位还未生成 `map→odom` 也能先显示；里程计与实时点云仍需要相应变换。
`real_navigation.rviz` 是在线配置，没有这些离线地图显示项。
本机与 NX 的 DDS 配置见 [网络说明](../../docs/NETWORK_SETUP.md)，GPU 默认启动设置不会替代 ROS 网络环境。

Fixed Frame 使用 map。添加以下显示：

| 话题 | 类型 | QoS |
| --- | --- | --- |
| /offline_map/support | PointCloud2 | Reliable、Transient Local |
| /offline_map/obstacles | PointCloud2 | Reliable、Transient Local |
| /relocalization/aligned_cloud | PointCloud2 | Best Effort、Volatile |
| /plan | Path | 沿用现有设置 |
| /local_costmap/costmap | Map | 沿用现有设置 |

全局三维 A* 仍使用自己的支撑/障碍缓存，global_costmap 不是完整二维离线地图。
需要整张定位地图显示时增加 `publish_known_map:=true`，无线链路将多传输一张地图。
地图发布一次并保留给晚启动的订阅者，不每帧发送整张地图。

固定地图同时发布 `/offline_map/id`（`std_msgs/String`，64 位十六进制 SHA-256）
与 `/offline_map/identity`（`std_msgs/String`，JSON），两者均为 Reliable、Transient Local，
并与固定点云一样仅发布一次。JSON 的 `schema_version=1`，包含绝对地图/数据库路径、
`database_signature`、`map_id`、节点名、`session_id`、`publication_stamp`，以及
`clouds.support` / `clouds.obstacles` 中的 `topic`、`points` 和 `data_sha256`。
`data_sha256` 校验实际消息中按小端 float32 打包的 XYZ 字节；时间戳不参与摘要。
`map_id` 对 `schema_version`、`frame_id`、`database_signature`、`clouds` 组成的
紧凑排序 JSON（UTF-8）计算 SHA-256，路径、会话和发布时间不参与地图身份。
两个话题用于核对运行中加载的地图；不能仅凭 RViz 中重叠点云的外观判断固定地图是否改变。

## 编译和验证边界

```bash
colcon build --packages-select astar_3d_planner fast_lio_nav2 go2_relocalization navigation_bringup go2_offline_navigation
colcon test --packages-select astar_3d_planner go2_offline_navigation
```

完整运行另需已编译 go2_relocalization、navigation_bringup 和安装 Nav2 控制/行为组件。
本机已补齐缺少的 Nav2 bringup、MPPI、转向控制器及其依赖；安装时没有升级已有软件包。
真实隧道的重复结构、断电/搬动后找回、固定地图覆盖度、定位跳变、NX 负载及倒退巡检
仍需后续验收；本机合成测试不能替代这些试验。

已准备好地图库时，可自动用传感器录包复查定位：

```bash
ros2 run go2_offline_navigation validate_replay \
  /绝对路径/录包 /绝对路径/tunnel_database /绝对路径/tunnel_navigation \
  /绝对路径/新的结果目录 --domain 114
```

工具使用独立域和 localhost；启动录包、FAST-LIO/EKF、实时地形与定位，不启动导航控制器、
运动桥接或雷达驱动。包需包含 `/hesai/points`、`/go2/imu`、`/lf/sportmodestate`。
结束后自动关闭自启进程并保存日志与 report.json；拒绝已有输出目录或被占用的测试域。
`ever_navigation_ready` 表示定位适配器曾为 ready，不证明路径可规划或真实巡检成功。
用同一包建库再回放只验证链路和重复数据匹配，不能代替独立采集的定位验收。

## 固定目标的定位与规划联合回放

`validate_replay` 加 `--planning-cases` 后，在真实传感器回放上额外启动全局规划器，
不启动控制器、行为树或运动桥接。参数通过共享 `offline_planner_settings()` 生成，
与离线导航入口使用相同的固定地图、身份检查、支撑连接和碰撞设置。

```bash
ros2 run go2_offline_navigation validate_replay \
  /home/zyhtxdy/go2-w_ws/bags/go2_inputs_20261010_094309 \
  /home/zyhtxdy/go2-w_ws/maps/lab_v1_database \
  /home/zyhtxdy/go2-w_ws/maps/lab_v1_navigation \
  /home/zyhtxdy/go2-w_ws/validation/新的评测目录 \
  --domain 155 \
  --planning-cases /home/zyhtxdy/go2-w_ws/src/go2_offline_navigation/config/replay_planning_cases_lab_v1.yaml
```

输出目录必须不存在。示例目标文件仅适用于本机 `lab_v1` 地图，预期地图 ID 不符会拒绝启动。
其他地图需要单独准备目标文件；不要在看过结果后删掉失败目标再计算成功率。
`--planner-params` 可指定参数对照文件，工具仍会覆盖必要的离线地图设置，
完整生效参数保存在输出目录的 `planner_params.yaml`。

目标文件保存检查时刻、原始 map 坐标、朝向与来源。示例在录包第 10、30、50 秒检查
6 个历史原始目标；另一个地图外负对照单独统计。每组查询串行执行，记录实际请求时刻、
相对计划时刻的延迟与当时 TF 起点。原始 Z=0 保留当前未指定地面高度的语义。
某一检查时刻没有定位、地图认证或规划服务时，记录未就绪，不延后选择有利时刻。
超时后停止后续请求，避免尚未结束的旧任务与新查询交叉。

`planning_report.json` 保存完整目标、真实输入状态、路径坐标、规划 Action 状态、
墙钟耗时、路径长度、终点 XY/朝向偏差和失败诊断。`planning_report.csv` 便于逐项比较。
成功必须是 Action 成功且返回有效非空路径；目标被接收不算成功。
同时给出按全部计划查询和按实际请求计算的比例，负对照不混入普通目标比例。

位姿始终来自原录包轨迹，目标仅用于请求路径；这不是沿新路线行走的闭环仿真，
不检验控制跟踪、真机到点或定位绝对精度。路径规划返回的 `planning_time` 使用 ROS 时钟，
短计算可能小于回放时钟间隔而显示为 0，需同时查看墙钟耗时。
工具自动备份并恢复找到的 FAST-LIO 源码调试日志；自定义源码布局可用
`--fastlio-log-directory` 指定日志目录。已有本机 FAST-LIO 进程时拒绝同时回放。


## 2026-10-21：离线支撑连接与取消诊断

离线入口读取导航地图元数据中的 `terrain_parameters.grid_resolution`，
连接距离取原配置与支撑栅格对角间距向上取整到规划分辨率后的较大值。
当前 0.20 m 支撑栅格、0.10 m 规划分辨率对应 0.30 m；只改变离线启动时的参数副本，
原在线配置 0.25 m 保持。该适配不生成支撑、不填补缺口，连续地面、坡度、台阶、障碍和完整外廓检查保留。
缺少间距记录的旧地图沿用配置值，不推测地图间距。

定位适配器在 ready 失效时新增带原因的警告，说明导航速度阻断及任务取消；
恢复时打印 ready 提示。原保护逻辑和阈值保持，已取消任务需要操作者重新下发。

## 离线起点连接和矩形落点选择

离线入口在自己的参数副本中启用 `retain_start_projection_bridge`；在线配置默认关闭。
已有当前列支撑时，不再强制经过网格中心，避免几厘米折线引入无意义的起步转向。
需要盲区连接时，候选起点会先检查机器人初始朝向、连接线及转向矩形扫掠，受阻则
继续尝试其他候选；最终完整路线检查保留。
近墙转向导致原路线失败时，离线规划最多检查 8 个替代支撑起点；每次额外搜索最多
展开 50000 个节点，累计 0.25 秒后不再开始下一次重试。这是重试调度预算，不是单次
搜索的硬超时。连接先通过连续地面检查，再检查整条路线的地形、外廓和转向。
路径有效性服务仅为本节点最新输出的完整相同路径保留已验证连接，并重新检查碰撞；
其他路径沿用普通起点投影规则。
此项不增大支撑覆盖半径，也不修改固定地图。仅当机器人当前列没有直接观测支撑、
路径保留实际选定的支撑起点时，允许沿笔直连接到达该起点；连接长度仍受
`start_projection_xy_radius` 限制。邻侧偶然出现一个附近支撑点，不会提前结束这条连接。
已观测高差、坡度和碰撞仍检查；到达选定起点后，后续未知缺口仍会拒绝。
这沿用近场盲区需要连接到可见地面的假设，不能证明缺少观测的位置确实有地面。

离线规划在选择目标支撑面前检查目标朝向的完整矩形外廓。精确 XY 的中心虽可通行、
但矩形角部碰撞时，保留原 `goal_projection_xy_radius` 范围内其他安全候选，
最终仍检查整条路线及转向扫掠。没有扩大目标容差或忽略实测障碍。
如果简化后的路线不满足矩形检查，最多回退一次到原搜索路线，并重新检查地形和完整外廓。
全部候选都受阻时，诊断给出 `goal_orientation_footprint_collision`，不会强行输出路线。
目标投影失败时另给出 `goal_no_support_xy`、`goal_support_outside_height_window` 或
`goal_body_clearance_blocked`；`surface_candidates` 保存最多 256 个候选的支撑高度、
拒绝原因和碰撞体素，超出部分有明确计数。

重定位按剩余全局校验期限提前安排后台计算。过期结果只可作为下一帧全部候选的初值，
不发布旧位姿、不刷新有效期。3 秒输入/跟踪门限和 8 秒全局校验门限保持原值。
后台质量、原始扫描年龄和耗时会写入诊断；临近期满与失效原因也写入普通日志。

## 2026-10-22：固定地图一致性与支撑快照

离线入口根据所选地图计算预期地图 ID 和两张固定点云的消息摘要，并在自己的参数副本中
启用 `verify_static_map_identity`。旧的自定义参数文件缺少这些身份字段时，
离线入口也会在参数副本中补入。规划器核对 `/offline_map/id`、`map` 坐标系、
XYZ 消息格式和实际数据 SHA-256；支持点、静态障碍及 ID 必须各只有一个发布者，
且来自同一节点。话题被 ROS 重映射后，审计跟随实际订阅话题。

相同数据重复发布只计数，不重复累加；时间戳变化不视为地图变化。不同数据、地图 ID、
格式或重复来源会锁定故障，拒绝混入地图，并拒绝新规划和路径有效性检查。
先解决冲突再重启离线链路；发布者新旧实例短暂重叠也可能触发此故障。
仅发布者暂时未发现是等待状态，来源恢复后可继续检查。
地图与数据库运行期间不要原地更新；导出新目录后重启整套入口。

```bash
ros2 topic echo /offline_map/identity --once --qos-durability transient_local
ros2 topic echo /planner/map_status --once --qos-durability transient_local
```

第一项保留地图路径、数据库签名、会话及发布时间，供人工核查；C++ 认证的是地图 ID、
预期点云摘要、消息格式及发布者图，不逐项解析 JSON 路径或会话字段。
第二项包含认证原因、预期/收到摘要、点数、接受/重复/拒绝计数和发布者信息。
其中 `ready` 表示地图输入检查通过；地图实际应用完成、定位和实时地形仍有各自检查。
`/offline_map/ready` 仍只是地图服务器心跳，不能代替规划器的认证状态。
这项检查不直接修改速度门控，也不等同于即时急停。

离线 `/planner/support_map` 改为 Reliable、Transient Local、深度 1，
节点激活时补发已加载的支撑快照。因此支撑在激活前到达、晚开 RViz 或重新激活节点时，
都可以看到内部支撑快照。RViz 显示此话题时可选 Reliable、Transient Local、Decay Time=0。
离线 RViz 配置新增 `PlannerSupportMap` 显示项，默认关闭；勾选后可检查规划器实际加载的
支撑点。核验时可暂时关闭 `OfflineSupport` 和实时扫描，避免点云叠加。
在线配置默认关闭地图身份检查，支撑快照继续沿用 Best Effort、Volatile 和原有限频。

## 可选的回放碰撞证据

`validate_replay` 加 `--obstacle-evidence` 后，仅对带实际障碍点的结构化规划失败，
保留请求前后各 3 秒的 `/hesai/points`、`/cloud_registered_body`、实际全局规划障碍输入，
以及 `/Odometry`。默认关闭，不增加真机控制节点。`commands.json` 保存实际子进程命令；
`--measured-obstacle-decay-time 0.6` 可与 `2.0` 分别输出到新目录，比较实测障碍保留时间。

证据在输出目录的 `obstacle_evidence/evidence.json`。共享的 `.cdr.gz` 文件保留完整
PointCloud2 原始字段和数据，读取方式是 `gzip.decompress()` 后按
`sensor_msgs/msg/PointCloud2` 调用 `rclpy.serialization.deserialize_message()`。
每事件的 `.cdrseq.gz` 保存原始 Odometry；解压后依次读取 4 字节小端长度和该长度的 CDR。
云的 `map`、`base_link` 变换按云自己的时间查询并保存，缺少对应 TF 会明确标注，
不会用最新 TF 补成一条未经验证的轨迹。压缩和近邻分析在回放结束后进行。

每个碰撞点按 0.15 m 三维半径统计附近回波，原始雷达的独立帧按正的原始 header 时间去重。
地形点含 `observation_sec`、`observation_nanosec` 时，另外记录实际观测帧；
重复发布地形结果不会增加原始观测帧数。已配准 body 云的命中帧单列，不能代替原始云。
首次、末次命中和之后首次未命中只描述保存窗口内的几何证据；未命中不能证明地面空闲，
原始云的刚体变换也不能代替逐点运动去畸变。不能据此认定是人、自身回波或反光。

默认最多 16 个事件，云内存估算上限 256 MiB，保存的压缩 CDR 文件合计上限 128 MiB。
重叠窗口共享云记录，每事件最多 1000 条里程计。内存达到上限时先淘汰未绑定事件的滚动
缓存；只有已捕获事件自身也超出上限时，才淘汰其中最早的云，避免后续无关数据赶走碰撞证据。
内存淘汰、文件未保存、TF 缺失、
窗口没有播完均有记录。事件达到上限时仍完成原有规划统计，保留被省略的事件时间。
