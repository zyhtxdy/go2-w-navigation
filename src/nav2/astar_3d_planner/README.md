# Astar3D support-height checks

## 旧路径的前向重接（2026-10-27）

缓存复用和 `/terrain/is_path_valid` 共用前向重接：先沿用旧路径的稳定投影，
再在投影后的当前近直线段内尝试最多 4 个接入点，前视上限 0.6 m。
接入点相对该段首切线超过 0.05 rad 即停止，保留后续拐角和旧路径尾段。
这避免几厘米横向误差被解释成先朝最近投影点原地大转向、再转回来。

每个候选都包含机器人真实 XY 起点和完整保留尾段，以原有路径间距采样后
重新检查连续地面、坡度台阶和完整矩形扫掠。真实起点碰撞、连接途中碰撞
或旧路径后段的新障碍仍会拒绝。前向候选均不通过时，再对原投影连接进行
同样检查，不使用忽略近场碰撞的方法放行。关闭完整矩形检查时保留原连接。

路径进度只按机器人对原路径的投影更新，不能因前视接入点提前前移；终点
位置与本次要求的朝向保留。缓存复用也以本次起点、目标朝向做完整校验。
前向尝试按阶段检查约 50 ms 预算（单次完整检查及最终原连接另计）。

这里限制的是局部前向连接不跨段；原有整条折线的最近投影关联规则没有重写，
因此不声称解决所有重叠分支的定位关联。没有通过完整检查的本地连接时，
路线仍会失效或交给重新规划。该功能不下发速度，也不启用倒车。


## 矩形约束下的必要碰撞预筛（2026-10-27）

`footprint_check_enabled: true` 时，硬预筛从已加 padding 的矩形边界推导
以底盘原点为圆心的内切半径，不再把 `inflate_radius` 向上取整的整格圆
当作最终通行门槛。只有整个候选栅格内的底盘原点都必然碰撞时才提前拒绝；
其余候选仍要经过原始障碍体素的完整矩形平移和转向扫掠检查。
因此预筛放行不代表栅格中心或整个栅格内的每个位姿都能通过。

`inflate_radius` 和 `cost_inflate_cells` 保留避墙软代价范围；从旧硬门槛
释放的格也有软代价，正反向搜索和路径优化都从观测地面的第一个车身高度格
开始读取，避免遗漏低矮障碍的代价。底盘原点不在矩形内部时，必要圆预筛禁用，
完整外廓检查仍生效。关闭完整矩形检查则保留旧圆形膨胀行为。

例如 0.1 m 格子内的真实直线可以安全通过，但将它移到格中心会碰墙。
旧预筛可能提前拒绝真实直线；新预筛允许继续验证真实 XY，同时仍拒绝碰墙的
格中心路径。它不增加新的亚栅格搜索节点，不能消除所有离散化漏解。

失败诊断新增 `prefilter_mode` 和 `hard_prefilter_radius_m`；原
`inflation_cells` 表示配置圆的整格范围，不能再把它当作矩形模式的硬半径。
起终点、直达线、A*、路径优化与缓存复核共用同一预筛地图。


## 矩形外廓与转向检查（2026-10-19）

`GridBased.footprint_check_enabled: true` 启用全局路径外廓校验，在线、
离线模式共用。包络读取 Planner Server 的 `global_costmap.footprint`，
包含 Nav2 的 `footprint_padding`，取其在底盘坐标系下的矩形边界。
当前原始配置为 0.80 × 0.50 m，属于含余量的初始配置，不是实测轮廓。
修改外廓后需重启规划器。高度使用 `GridBased.robot_height`。

校验包括实际起点朝向到第一段方向的转向、每段移动、拐角转向，以及
终点向目标朝向的旋转。路径按向前沿各段行驶、拐角原地转向的假设检查；
它不会生成倒车轨迹。平移与旋转之间按体素分辨率取样并增加覆盖采样间隙
的几何余量；恰好 180° 的原地转向检查两个方向。直接路线、A* 最终输出、
缓存复用和 `/terrain/is_path_valid` 均使用该检查。

外廓直接检查活动的原始障碍体素，包括实测、合成、未知及固定地图来源，
不对矩形重复叠加 `inflate_radius`。障碍体素按完整方格相交判断，
不只检查点中心。当前必要预筛见上节；候选失败后还会按预算尝试其他
起终点和朝向搜索，不能保证找到所有连续空间中存在的路线。
局部控制器自己的外廓检查仍保留，实际运动轨迹应由其独立检查。

矩形碰撞不使用近场或终点障碍宽限。`/planner/start_failure` 中
`stage=footprint`、`reason=footprint_collision` 表示外廓碰撞，并记录
障碍来源与碰撞格；终端日志还记录路径索引和朝向。
`footprint_sampling_budget_exceeded` 表示超过 `footprint_max_samples`
（默认 20000）的计算预算，不表示不存在可行路线。
`footprint_invalid_pose` / `footprint_invalid_orientation` 表示输入无效。

这是平面朝向下的障碍检查，保留原有地面高度格与车身高度约定；
没有检查每个轮子下方的支撑，也没有推断未观测区域的地面，
更没有覆盖车身俯仰、侧倾后的完整三维外廓。原有中心支撑、坡度与台阶
检查继续生效。当前仅完成本机软件验证，没有连接 NX 或进行运动测试。

本次本机编译通过，115 项规划器核心测试通过（含新增 10 项外廓测试）；
另有 2 项实际 ROS Planner Server 集成测试通过，分别使用 0.48 m 和
0.24 m 搜索膨胀配置。集成检查覆盖静态地图、动态障碍绕行与清除、
矩形角落碰撞、缓存路线拒绝、剩余路线失效以及碰撞诊断。测试只启动
地图与规划节点，使用本机隔离域，没有启动运动控制桥接。真机测试 0 趟。

Support layers store the latest observed continuous height. A*
connections and sampled path segments use these heights for both slope and
step limits; a crossing of a 0.1 m voxel boundary is not treated as a 0.1 m
physical rise. The step check no longer adds half a voxel to its configured
limit. Intermediate support is checked so a flat pair of endpoints cannot
hide an unsafe rise between them.

The ROS plugin emits observed heights in newly planned routes. Cached-route
reuse and the terrain path-validity service share the same terrain validation;
dynamic-obstacle grace does not skip slope or step checks. Reused route heights
are refreshed from their support layers. The initial blind bridge keeps its
existing start-projection rule; only its initial prefix may be skipped, so a
later crossing on a looping route cannot bypass validation. Collision windows
also cover the updated observed floor height while retaining the former voxel
window conservatively.

Collision validation of refreshed routes resolves the current support-layer
key from its observed height before checking clearance. This prevents a merged
layer whose height crosses a Z-bin boundary from being mistaken for a gap
mismatch. Both cached-route reuse and the path-validity service reject missing
support or height mismatches explicitly; their existing dynamic-obstacle grace
rules are otherwise unchanged.

Support updates average heights within each input voxel. Existing layers are
associated by actual continuous height difference in metres, bounded by
`support_merge_height`, rather than by distance between integer Z-bin indices.
When an updated layer crosses a Z boundary its affected column is rekeyed, so
the layer remains queryable at its observed height. When several observations
match a layer, prefer the closest observation, without inventing an average
between distinct surfaces. The representation still cannot
distinguish closely spaced surfaces in the same XY column; for example, the
default 0.15 m merge setting does not guarantee that an actual 0.08 m step in
one column forms a separate layer. Separate XY columns and retained vertical
layers are checked using their observed heights. This change does not infer
ground in unobserved regions or remove collision checks.

Online and fixed-map support inputs, drift-overlap matching and support queries
share narrowly bounded FLOAT32 boundary normalization in XY. A nominal 0.2 m
raster must not become a 0.3 m graph gap through serialization; arbitrary point
coordinates are not rounded to a grid centre. Obstacle quantization, the 0.25 m
connection radius, slope limits and step limits are unchanged.

Slope/step failures include the two actual support keys, continuous heights
and XY separation in `direct_reject.support_edge` and path-validation logs.
These measurements describe the planning map, not independently measured terrain.

## 近场连接与历史支撑高度（2026-10-13）

规划时先检查机器人实际位置到候选目标的直线，避免雷达脚下缺回波时，
先选中旁侧支撑格再产生无必要的绕行。只有路线开头允许缺支撑，长度不
超过已有 `start_projection_xy_radius`（当前配置 2 m）；目标必须有观测
支撑，进入支撑后不能再跨缺口。首个支撑与预估地面高度相差不得超过
`max_step_height`；障碍、坡度和台阶仍按原阈值检查。该连接没有生成地面，
不能证明盲区内安全；局部感知和操作者仍需覆盖传感器看不到的区域。

`compensate_height_drift` 默认关闭，Go2W 当前 MPPI 配置中开启。对静态
场景内同一 XY 地面共同缓慢变化的 Z，统一历史支撑与障碍的高度基准，
避免旧、新观测形成假的高程接缝。至少需要 30 个独立重叠柱、70% 一致，
且 X/Y 覆盖均至少 0.6 m；每次共同修正不超过 0.02 m。它不拟合平地、
不填未知区域，也不修复 FAST-LIO 原始点云和里程计的漂移。真实空间坡度、
台阶及楼层相对高差保留；大片真实表面缓慢升降与估计漂移无法仅凭这些
观测区分，动态地面场景应关闭此功能。

开启补偿时，支撑和全局障碍必须来自同一个发布时间戳。两路各缓存最多
10 帧，支持回调错开或乱序，超时缺少对应帧会告警；不混用不同帧做补偿。
地形模块的稳定障碍可包含短时保留的旧观测，因此同一次发布不等于每个
障碍点都刚刚测得。补偿关闭时，保留两路独立更新行为。

`/planner/support_map` 每秒发布规划器累计的真实支撑点，使用当前高度
基准。它与 `/terrain/support`（地形模块当前约 2 秒窗口）不同。
RViz 中两者都应使用 Best Effort、Volatile，累计地图的 Decay Time 设 0。
可加载工作空间的 `fast_lio_nav2/config/real_navigation.rviz` 查看。

## 本机编译与验证（2026-10-10）

本机已通过系统软件包安装补齐 `ros-humble-nav2-core`、
`ros-humble-nav2-costmap-2d` 和其依赖 `ros-humble-nav2-voxel-grid`，
三个包的 Nav2 版本均为 1.1.20。工作空间中没有保留额外的依赖副本。

完整 ROS 共享库及测试程序已经通过正常 `colcon build` 编译，产物位于
`install/astar_3d_planner`。正常 `colcon test` 的 38 项 gtest 全部通过：
29 项地图与 A* 测试、8 项缓存路径截取测试、1 项插件发现与加载测试。
回归覆盖毫米级跨高度格变化、真实陡坡、路径中间隆起、严格台阶上限、
缺少支撑、障碍、合并层高度更新、缓存路径高度刷新和绕圈路径。

测试结果位于工作空间根目录的：

- `build/astar_3d_planner/test_results/astar_3d_planner/test_astar_3d.gtest.xml`
- `build/astar_3d_planner/test_results/astar_3d_planner/test_plugin_loading.gtest.xml`

复现命令：

```bash
cd /home/zyhtxdy/go2-w_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select astar_3d_planner \
  --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
colcon test --packages-select astar_3d_planner
colcon test-result --test-result-base build/astar_3d_planner --verbose
```

本次只完成本机编译和离线测试，没有连接或更新 NX，没有启动导航或运动
控制节点。本次自主真机测试 0 趟。测试不证明实际场地的支撑覆盖、传感器
外参或运动建图漂移已经解决。

## 已完成的合成场景验证

2026-10-10 补充完成实际地形节点 → ROS 话题 → 实际全局规划插件测试。
直接读取当前地形及 MPPI 导航配置，仅替换测试话题和服务名。
平地、6° 缓坡、箱体绕行，以及陡坡、台阶、缺口、障碍墙拒绝，共 7 个
场景全部通过。场景发现并修复了 20 cm 支撑网格被 10 cm 中间采样列
误算坡度的问题；按实际支撑间距计算坡度，原有支撑与碰撞检查保留。
规划器 gtest 因新增 5 项回归，现为 43 项；地形模块 18 项也全部通过。

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select terrain_costmap astar_3d_planner \
  --parallel-workers 1 \
  --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
colcon test --packages-select terrain_costmap astar_3d_planner
```

2026-10-10 按要求清理了上述 7 个场景的大型测试源码、构建入口及重复
日志和 XML 报告。当前命令只运行保留的核心测试，不再运行这些场景。
坡度、台阶、支撑缺口、碰撞及插件加载测试保留；规划算法没有因清理而修改。
历史输入、结果及起点投影高度限制见
[离线测试记录](../../../docs/validation/2026-10-10-offline-terrain-planning/README.md)。
## 起点与搜索拒绝诊断

规划失败时，`/planner/start_failure` 发布 JSON 文本，`/planner/start_failure_markers` 发布 RViz 标记。二者采用 Reliable、Transient Local、深度 1，保存最近一次状态，后来打开 RViz 也能看到。新规划请求及节点停用会清除旧标记；这些话题用于诊断，不触发运动或改变碰撞判断。

JSON 的 `stage` 区分起点投影与后续搜索。`reason` 包括：

| 值 | 含义 |
| --- | --- |
| `map_has_no_support` | 缓存内还没有支撑面 |
| `no_support_xy` | 起点搜索范围内没有支撑柱 |
| `support_outside_height_window` | 支撑层不在指定高度搜索窗口内 |
| `surface_clearance_blocked` | 范围内的支撑候选因车身净空碰撞被拒 |
| `start_bridge_blocked` | 有安全支撑候选，但起点连接到候选的采样段被障碍阻挡 |
| `ground_disconnected` | 当前选定起点与目标之间没有通过检查的支撑连接 |
| `search_node_limit` | 搜索节点预算耗尽，不能据此判定地面不连通 |

搜索失败还记录直线候选的第一个拒绝原因和位置，以及展开节点数、邻接距离／高度／采样段拒绝次数。`collision_context=direct_candidate_only` 表示标出的障碍阻挡了直线候选，不能将它当成整个绕行搜索失败的唯一原因。绿色球为已投影起点，黄色球为直线被拒位置。

失败重查和证据捕获均持有同一个地图读锁。`collision.key` 是实际被拒的占用格；`obstacle.key` 是对该格贡献占用的活动障碍体素之一，按照建图所用的离散膨胀偏移反查。多点同时贡献时仅展示其中一个，不表示它是唯一原因。

`support_stamp_ns`、`obstacle_stamp_ns` 是缓存最近接受的输入时间；两者不同时，不能称为同一帧快照。`source_cloud_stamp_ns` 是贡献点最后进入缓存的点云时间，稳定器可能保留此前的点，不是独立测得的雷达回波时间。高度基准补偿后，`point_current_datum` 是当前规划坐标下的位置，`point_observed_z` 保留该点入缓存时的 Z。

地形障碍流增加 `obstacle_source` 字段：1 为地面拟合后认定的实测障碍点，2 为地形判定生成的标记；旧的无该字段点云按 unknown 处理。一个缓存体素含多类来源时报告 mixed；实测或未知历史不会降级成可清理的纯合成来源。实测类不能直接证明物体真实存在，也不能排除机身、配准或地面判定错误。

RViz 蓝球为起点地面参考，红格为被拒格，紫球为贡献点，半透明橙格为该点的实际离散膨胀范围；只显示被挡住的高度切片。显示配置已加入 `real_navigation.rviz` 和 `replay_planning.rviz`。

## 近身合成标记历史

地形原来只抑制当前帧 0.65 m 内的合成标记，稳定器与规划器却仍会保持历史标记。本次把已有近场策略应用到这两层缓存：仅及时移除进入该范围的纯合成标记；实测、混合、未知来源仍按正常碰撞与过期规则处理。

规划参数 `synthetic_obstacle_clearance_radius` 默认 0（关闭），当前 Go2W 配置显式设为 0.65，与地形配置一致。缓存更新的观察中心使用 LIO 位姿结合静态变换求出的底盘中心，与地形分析一致。没有扩大盲区、改变障碍膨胀、坡度或台阶限制。

# 离线模式的支撑连接与外廓检查补充

`retain_start_projection_bridge` 默认关闭，离线入口单独启用。仅保留机器人到实际选定
支撑起点的有界直线连接，不增大支撑半径；已选起点之后仍严格检查支撑、坡度和高差。
机器人当前列已有直接观测支撑时不使用此例外。

`use_static_map=true` 时，目标选项先通过指定朝向的矩形位姿检查，避免精确终点的中心
可通行而角部碰撞时丢失附近其他可行落点。目标投影容差与最终完整扫掠检查保留。
简化路径矩形检查失败时可回退一次到原搜索路线，重新验证连续地形与完整外廓。
固定障碍和实时实测障碍均不因上述处理被删除。

# 全局障碍体素的细化检查

工作空间全局规划配置采用以下 XY 细化，插件本身默认关闭：

| 参数 | 工作空间配置 | 作用 |
| --- | --- | --- |
| `refine_static_obstacle_extents` | `true` | 配合引导模式，对整条路径复核固定点范围 |
| `static_obstacle_extent_margin` | `0.02` m | 固定点范围保留的 XY 余量 |
| `near_start_measured_refinement_radius` | `0.80` m | 当前起点周围允许复核纯实测实时体素的范围 |
| `near_start_measured_obstacle_margin` | `0.03` m | 实测点范围保留的 XY 余量 |

先做原完整体素与矩形车身检查，仅在相交时用**全部有效点**的包围盒加余量复核。
包围盒限制在原体素内，只削减空的 XY 量化区域，竖直体素规则和车身尺寸不变。
实时范围以本次请求真实起点为中心，搜索每条边时不重新设置中心。
未知来源、合成危险点、混合来源和区域外实时体素保持完整体素检查。

同一实时体素保留所有仍有效更新的范围，不使用单个诊断代表点替代完整范围。
固定图与实时回波共用一个体素时取两者范围的并集，实时观测不会覆盖固定障碍。
混入未知／合成来源后保持保守，直至该动态体素移除或被明确替换。
重置、过期、快照和输入流重启同步维护这些范围，原时效阈值不变。

旧的 `near_start_static_refinement_radius: 0.60`、
`near_start_static_obstacle_margin: 0.02` 保留兼容；仅关闭全局固定范围复核时，
才限制为该近起点范围。所有细化均关闭后恢复原完整体素规则。
严格转向模式单独启用全图范围复核曾出现一次冻结输入限时拒绝，
不能把它与当前引导模式混为同一套验证结果。

搜索快照、返回前新地图复核、缓存路径和有效性服务采用一致规则。
地面支撑、坡度、高差和矩形通道检查保留。

## 全局引导模式与严格机动模式

插件的 `check_in_place_turn_sweeps` 默认为 `true`，保留原有“转向、平移、
到点转向”的完整机动验证。工作空间当前配置为 `false`，采用全局引导模式：

- 检查机器人当前实际朝向下的起点位姿。
- 每段路径按该段方向检查完整矩形车身的连续平移通道。
- 折点两侧的矩形位姿均检查，但不要求在折点原地转过所有中间角度。
- 目标仍检查请求朝向下的实际矩形位姿，目标内部的真实占用不会放行。
- 缓存、最新地图复核与路径有效性服务使用相同模式。
- 引导模式的后备搜索合并同一地面层、同一位置的到达朝向，逐条检查矩形通道；
  严格模式保留朝向状态。不同高度层不合并，也不扩大搜索时限。

因此结果是供局部控制器跟踪的几何引导路径，不再声称已经证明整段转向机动
可执行。当前局部控制包含 Rotation Shim 和 MPPI；前者会尝试起步旋转并检查
碰撞。局部控制仍可能停下或拒绝执行，不能用全局规划成功率替代真机到点成功率。
参见 [Humble Rotation Shim 源码](https://api.nav2.org/nav2-humble/html/nav2__rotation__shim__controller_8cpp_source.html)。

固定图数据和哈希不变，没有创建起点免碰撞区。该调整改善点云量化模型的保守性，
不是传感器误差或整机间隙的实测标定，也不能保证 MPPI 一定能完成跟踪。

