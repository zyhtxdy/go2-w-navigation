# Go2 已知点云地图重定位（初版）

此包独立于当前建图导航链路。已有 `mapping.launch.py`、`odom_fusion.launch.py`、
`real_nav2.launch.py` 没有加入此包，现有参数和 TF 发布者没有调整。
本包没有速度发布、导航任务或运动接口；重定位 launch 强制 `publish_tf=false`，
节点也拒绝开启 TF。本包只输出配准结果；离线导航入口由独立适配器检查后接入规划器。

## 算法与边界

保留 FAST-LIO2 作为里程计前端。以本包自己实现的环／扇区最大高度描述子
检索关键帧（Scan Context 风格，非官方 Scan Context / Scan Context++ 移植），
生成位置与 yaw 候选，再使用已有 PCL GICP 精配准。
配准后独立计算对应点重叠率、均方根距离；比较不同位置或方向候选的分差。
综合分数为 `overlap − 0.25 × RMSE / evaluation_distance`，默认至少 0.70。
初次合成搬动测试中，错误场景仍有约 0.664 重叠率、0.156 m 残差；
因此额外加入这个保守门槛。这些是合成输入的软件观察值，真机 0 趟，
不是实测隧道误定位率，也不能证明所有相似场景都能区分。
另一组实际 ROS 合成搬动输入仍能以约 0.829 重叠率、0.070 m 残差通过局部匹配；
因此保留周期性全局复核，不能靠不断抬高单个残差门槛解决结构混淆。
使用匹配到的目标局部平面法向形成 3×3 信息矩阵，检查平移方向退化。
这不是完整六自由度可观测性证明，门槛只是初始建议，未做真实隧道标定。

初次启动无需给初值；匹配失效后回到全局搜索，手动重定位服务也可强制搜索。
默认处理间隔为 0.2 s；轻量跟踪使用独立前台线程。
全局检索和全候选核验在同一个后台线程顺序运行，避免核验阻塞跟踪，也避免两批候选同时争用 CPU。
完整全局配准本身已检查全部检索候选。首次定位仍要求扫描年龄在 3 s 内；
已定位后的后台结果使用独立的 `max_global_result_age`，默认 4 s，并受 8 s 全局复核期限约束。
后台结果保留原始测量时间；前台发布前仍须用年龄不超过 3 s 的当前扫描完整跟踪并通过质量及跳变检查，
且在前台完成后再次核对后台结果年龄。不能把旧全局位姿改成当前时间发布。
若全局检索耗时超过对应阶段的新鲜度预算，结果仅作为初值；按里程计增量推进全部原有候选，
用新扫描完整配准并重新计算质量、歧义和退化指标。旧分数不参与新扫描的验收。
后台测量结果通过代次和输入 epoch 检查后交给前台；已定位时，还需对当前扫描重新做轻量配准，
不能把旧全局位姿简单改成新时间戳。初次定位由三个不同扫描的全局核验结果完成一致确认。
后台旧结果不刷新全局核验有效期；质量拒绝仍会使定位失效。
候选初值最多保留 8 s；当前扫描仍须满足原有 3 s 新鲜度和跟踪超时。
最近一次全局核验超过 8 s 也判失效，不允许一直靠局部跟踪维持定位。
已定位时从上一后台完整检索/核验周期完成起等待 1 s，再开始新周期。
`confirmation_timeout` 控制初次多帧确认间隔，默认 3 s；`tracking_timeout` 控制已定位后的连续输出，默认也是 3 s。
两者独立，便于用更严格的跟踪超时测试后台计算是否仍会阻塞。

协方差、搜索树和法向按完全相同的目标点集合缓存；缓存上限为 8 份、合计 20 万目标点。
每个查询的源点协方差只计算一次；不改变候选数量、配准迭代或质量门槛。
独立观察入口默认单线程；离线导航入口默认 `localization_candidate_threads:=2`。
候选并行结果按原候选顺序收集；缓存及并行与原计算的候选、分数和位姿已做对照。
里程计明显平移／旋转／时间断续也触发重新搜索。
若后续增大全局检查间隔，中间帧只局部跟踪；搬动后局部错配可能直到下次全局检查才发现。
只有至少三个不同时间戳的匹配给出一致的 `map→odom` 结果才发布有效定位。
已经定位且上一结果仍新鲜时，全局搜索额外使用里程计预测作为一个配准初值；
原有全部检索候选仍参与评分和歧义检查，不把里程计预测当作位置真值。
诊断输出综合分数及门槛、分数余量、收敛状态、不同位置/朝向备选分差、
是否使用预测初值，便于区分低质量、歧义、退化与处理超时。
确认等待的空闲超时从上个结果完成时计算；帧间隔及定位有效期仍按原始测量时间判断。
源云与 `/Odometry` 按消息时间配对，检查输入坐标系、时间新鲜度和单位四元数。
过期、无匹配、歧义或平移退化时 `/relocalization/valid` 为 false。

长直且几何重复的隧道可能无法仅凭雷达确定沿隧道的绝对位置。
有限 top-K 检索也不能排除未检索候选中的重复地点；连续一致性仍可能一致地匹配错段。
不能保证任意断电／搬动后自动找回。实际验收需要不同地点启动、反向放置、
重复通道和搬动测试，并根据消防设施、岔口等独特结构评估是否需辅助标志或定位信息。
本模块也不会修复已保存地图中的建图漂移、未观测地面或现有支撑图保存缺口。
候选搜索主要处理 yaw 变化，roll／pitch 初值沿用地图关键帧；翻倒姿态未验收。

选择依据：[Scan Context 作者仓库](https://github.com/gisbi-kim/scancontext_tro)、
[PCL GICP](https://pointclouds.org/documentation/classpcl_1_1_generalized_iterative_closest_point.html)、
[几何退化研究](https://www.cs.cmu.edu/~kaess/pub/Zhang16icra.html)。
后续实测计算性能不足时，可评估 [small_gicp](https://github.com/koide3/small_gicp)。

## 本机编译

```bash
cd /home/zyhtxdy/go2-w_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select go2_relocalization --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

依赖已有 ROS2 Humble、PCL、Eigen，不下载另一套 FAST-LIO。
2026-10-21 的后台搜索改进先在本机开发与回放验证；NX 仅做本人容器内编译和保存关键帧的离线配准基准，不启动机器狗。
本次原始记录已移到工作空间外的本机验证归档。

## 地图库怎么准备

推荐采集实际扫描关键帧：启动现有建图链路后，另开终端启动记录器。
本包不启动建图、雷达驱动或运动节点。关键帧默认每移动 1 m 或转过 15°采一次，
点云体素 0.2 m、最大距离 20 m、至少 100 个点；以上为初始配置，不是真机结论。

```bash
ros2 launch go2_relocalization record_database.launch.py output:=/绝对路径/tunnel_database
# 遥控采集由操作者自行安排；结束前主动导出。
ros2 service call /relocalization_recorder/save std_srvs/srv/Trigger '{}'
```

目标目录必须不存在。只有 save 返回 success=true 才完成持久化；
采集中断电会丢失尚未导出的内存关键帧。地图原点和高度沿用采集时 `/Odometry` 的 odom；
导出后将这个固定会话坐标系作为已知地图 map。不要把多次重启的不同 odom 坐标直接混合。
当前 recorder 不校正采集轨迹漂移；有漂移的库需要先修正地图／关键帧位姿。

如果已存在 PCD 和同坐标系的本体 IMU 轨迹 CSV，可以生成虚拟视角库：

```bash
ros2 run go2_relocalization database_builder /绝对路径/map.pcd /绝对路径/poses.csv /绝对路径/new_database
```

CSV 表头 `x,y,z,qx,qy,qz,qw`，可增加 `id`；单位米，四元数必须归一化。
位姿是地图下的 **body IMU**，不是 base_link 或雷达，朝向也必须对应。
工具按每个轨迹位姿裁剪 20 m 点云、变到 body 并体素化。
虚拟裁剪含遮挡背后的点，不等于实际可见扫描，自动检索效果需要真实数据验证。
仅一张 PCD 而没有采集位姿时，不自动捏造关键帧。

库内：`manifest.csv` 为 `id,x,y,z,qx,qy,qz,qw`，`keyframes/<id>.pcd` 为局部 body 云，
`map.pcd` 为地图云。描述子在加载时重建；写库使用临时目录后原子发布，拒绝覆盖旧库。

## 独立观察启动

保持现有建图导航启动方式，在另一个终端运行：

```bash
ros2 launch go2_relocalization relocalization.launch.py database:=/绝对路径/tunnel_database
ros2 topic echo /relocalization/status
ros2 topic echo /relocalization/valid
# 可主动清掉旧确认，重新全局搜索；不会发运动指令。
ros2 service call /relocalization/relocalize std_srvs/srv/Trigger '{}'
```

输入：`/cloud_registered_body`（PointCloud2，body）、`/Odometry`（Odometry，odom→body）。
默认处理间隔 0.2 s；后台搜索与前台跟踪各只保留最新待处理帧，不积压扫描。
后台结果必须通过其真实测量时间的新鲜度和全局质量检查；过期搜索结果需要新扫描核验。
会消耗计算和订阅带宽；本轮核验解耦版本的 NX 编译及回放状态见验证记录。

| 输出 | 含义 |
| --- | --- |
| `/relocalization/pose` | 已确认的 body IMU 地图位姿，PoseStamped |
| `/relocalization/map_to_odom` | 计算的地图到当前里程计变换，TransformStamped，普通结果话题 |
| `/relocalization/candidate_map_to_odom` | 通过配准质量及歧义检查的待确认变换；不是有效定位、不是 TF，不能直接驱动导航 |
| `/relocalization/aligned_cloud` | 对齐后的当前点云，PointCloud2，只在有人订阅时发布，最多约 6000 点 |
| `/relocalization/known_map` | 已知地图云，PointCloud2，需显式开启，一次发布并使用 transient_local |
| `/relocalization/status` | DiagnosticArray，状态、拒绝原因、重叠率、距离、法向信息比 |
| `/relocalization/valid` | Bool，是否有未过期的确认结果 |

位姿有效时：`T_map_odom = T_map_body × inverse(T_odom_body)`。
诊断另含 `registration_ms`、`registration_candidates`、`discarded_generation_results`，
分别表示最近一次前台配准耗时、候选数，以及状态改变后丢弃的过期任务结果数。
`matching_stage` 区分 `tracking` 与 `fresh_global_verification`；
`global_registration_ms`、`global_candidates`、`global_search_busy` 单独记录后台搜索。
`global_verification_ms`、`global_verification_candidates`、`global_verification_busy` 记录后台核验；
`discarded_stale_verifications` 记录未被用于刷新定位的过期后台结果。
后台按近期完整一轮的耗时检查剩余全局校验期限；余量不足时提前结束等待，
不会增加扫描新鲜度或 8 秒全局校验期限。若一轮核验算完时扫描已过期，
只保留所有候选作为初值，立即在最新扫描上重新评分，避免重新检索和额外等待。
过期结果不能发布位姿，也不能刷新有效时间；复核仍包含全部候选及歧义检查。

`global_job_stage`、`global_job_input_stamp`、`global_job_elapsed_ms` 表示正在进行的任务；
`global_result_status`、`global_result_outcome`、`global_result_input_stamp`、
`global_result_input_age_s`、`global_result_job_ms` 和 `global_cycle_ms` 保留最近完成任务的上下文。
全局质量单独记录在 `global_result_rmse_m`、`global_result_overlap`、
`global_result_quality_score`、`global_result_score_gap`、
`global_result_normal_information_ratio`，不会被快速跟踪结果覆盖。
临近全局期限、丢弃过期后台结果和定位失效时，普通 ROS 日志也会保留这些时间与质量数据，
无需提前打开诊断话题才能排查取消原因。
`global_proposal_stamp` 是初值来源扫描时间，`last_verified_stamp` 是本次新扫描的测量时间；
二者不能互换。`last_global_verified_stamp` 记录最近一次新扫描全局核验时间。
`pose` 指 IMU 原点，展示机器人中心需再使用现有 body/base_link 安装关系。
诊断会保留上次数值；必须同时看 valid 和状态，旧 pose 不是持续有效的证明。
连续确认只是软件门控，valid 还不是经过验收的导航安全信号。

需要检查对齐效果时，启动命令增加 `publish_known_map:=true`；会额外传输整张已知地图，
默认关闭以减少无线负载。RViz Fixed Frame 设为 map，添加 known_map（Reliable、
Transient Local）和 aligned_cloud（Best Effort、Volatile）。
新库 map 与当前静态恒等 map→odom 不一定对齐，不能要求结果和当前在线图重合；
对比的是已知地图和新模块对齐后的点云。当前导航仍使用原来的坐标链。

回放需 `ros2 bag play <包路径> --clock`，新模块加 `use_sim_time:=true`；
已有包必须含 body 点云和 Odometry，不能把未启用仿真时钟的历史时间戳当实时数据。

## 后续接入离线导航

已新增独立本机初版入口 `go2_offline_navigation/offline_nav.launch.py`，
说明见 [离线导航 README](../go2_offline_navigation/README.md)。
本包继续保持结果观察模式；新适配器负责检查状态、独占发布动态 map→odom，
EKF 继续发布 odom→base_link。新入口加载同一数据库生成的固定支撑／静态障碍，
与实时危险点分开维护，并在定位失效或大幅修正时阻断速度、请求取消旧导航任务。
原在线入口默认不接入这些节点。该离线链路尚未通过真实隧道或 NX 验收。
适配器可在上游确认期间检查同时间戳的高质量候选，将两边的连续确认重叠执行。
上游未明确 valid、LIO/EKF 检查失败、发生跳变或数据过期时，仍不发布有效定位 TF，
也不开放速度。旧版重定位节点没有候选话题时，继续采用原来的串行确认。

## 离线测试

已保存关键帧的配准耗时对照（只比较内部轨迹参考，不代表外部测量精度）：

```bash
ros2 run go2_relocalization registration_benchmark /绝对路径/tunnel_database uncached 1
ros2 run go2_relocalization registration_benchmark /绝对路径/tunnel_database cached 2
```

```bash
colcon test --packages-select go2_relocalization --event-handlers console_direct+
colcon test-result --test-result-base build/go2_relocalization --verbose
```

合成测试覆盖未知初始朝向、保存重载、搬动后重新搜索、重复地点歧义拒绝、
直隧道平移退化、坏输入，以及时间配对和多帧确认。
ROS 集成测试使用独立 domain、仅 localhost 和测试话题，不连接机器人。
合成误差和成功次数不代表真实 XT16、NX 或隧道的定位精度。
