# 离线真机规划日志只读排查（2026-10-21）

只读取本人 `zyh-nav2` 容器的现有日志、当前启动参数和已保存地图文件。
没有启动或停止业务节点，没有下发目标、速度或修改参数、地图；SSH 会话均已关闭。
操作者正在自行测试。缺少完整真机测试卡和外部测量，本记录不统计真机趟数或成功率。

## 范围

- 当前 planner_server PID 3991、controller_server PID 3987、bt_navigator PID 4028。
- 运行配置确认：全局支撑 `/offline_map/support`，固定危险点 `/offline_map/obstacles`，实时危险点 `/terrain/obstacles_global`。
- 固定地图 891 支撑点、982 危险点。来源文件标注 290 measured、692 synthetic；这些是导出文件统计，不代表障碍个数或误检率。
- 地图按观测导出，不自动填充未观测地面。离线模式不使用实时支撑补充固定支撑。

## 已找到的原因

1. 地面支撑缺口与连通性：日志出现 `missing_ground_support`、`ground_disconnected`，路径在约 `(1.19,-0.80)` 附近缺支撑。对照导出 support.csv，该位置最近支撑点约 0.201 m；运行 support_radius=0.15 m、connection_radius=0.25 m。这是文件计算值，不是外部实测。失败搜索日志中的 height_rejected 均为 0，因此这些搜索失败没有表现为高度阈值拒绝。
2. 碰撞输入有两种：21 条 footprint_collision 诊断中，14 条来自固定 `/offline_map/obstacles`，7 条来自实时 `/terrain/obstacles_global`（synthetic_terrain_marker）。它们包含重复重试，不能当成 21 趟失败。
3. 至少一个固定碰撞点 `(1.8953,0.5531,0.1331)` 在 hazard_sources.csv 中标注为 measured，不能把固定图中的所有碰撞都当成虚拟标记或误检。另一些实时碰撞确为地形算法生成的危险标记，需要与实际地形对照。
4. 路径跟踪期间多次出现 `Control loop missed its desired rate of 20.0000Hz`。本次下载的控制器日志有 986 条此警告，它们是日志条数，不是真机趟数。仅凭日志不能确定耗时来自 MPPI、TF 等待还是 CPU 竞争。
5. 部分时刻存在 `Known-map localization is not ready` 和历史 TF 外推错误。后者含旧时间戳请求，不能直接归因于两机时差，需单独核对路径、目标及坐标变换的时间基准。

## 不能由截图认定的事

原始地图看起来空旷，不表示固定支撑连续，也不表示规划器的固定/实时危险点为空。
需在 RViz 单独显示 `/offline_map/support`、`/offline_map/obstacles`、`/terrain/obstacles_global`。
前两个话题需 Reliable + Transient Local，实时话题用 Best Effort + Volatile。
颜色用于区分话题，不能用已知地图按高度渲染的红色或绿色直接判定占用。

目前证据支持：导航地形图有局部支撑缺口，固定与实时危险点会参与车体外廓碰撞；
控制器另有频率不达标问题。不能据此认定 FAST-LIO 几何地图整体建坏，或实际场地存在障碍。
也不能把 saved map 的 synthetic 数量直接当成误检数量。

见 [日志汇总](log_summary.json)、[运行参数](running_nav2_params.yaml) 和本目录保存的原始节点日志。


## 追加：当前异常的分层复现结论

### 1. 定位调度会触发自动取消条件（已记录失效链）

原始状态记录 [cancel_trace_current.json](cancel_trace_current.json)，90.01 s 左右的只读观测；
汇总见 [cancel_trace_summary.json](cancel_trace_summary.json)。操作者已停止本轮运动测试，
采样没有代理下发目标；因此不能将此段的失效事件冒充某一历史 goal 的取消瞬间。

- 第一次 `stale_or_future_alignment` 前，最近有效定位的消息年龄达到 **3.032 s**；
  前台新扫描全候选核验随后完成，耗时 **2709.24 ms**。核验结果自身年龄约 2.77 s，
  仍通过新鲜度门槛，但等待期间上一跟踪结果先超过了 3 s。
- 第二次同类失效时，旧结果年龄 **3.057 s**，随后重定位节点明确输出 `tracking_timeout`，
  再进入 `awaiting_distinct_consistent_frames`，发生恢复确认。
- 这次是过期，不是未来时间戳；年龄均在 NX 内部计算，不能把它归因于笔记本显示颜色或直接归因于两机时差。
- 代码核实：后台全局检索已拆开，但 `verifyGlobalProposal` 仍在前台跟踪工作线程同步计算。
  该调用执行期间，最新扫描的轻量跟踪无法推进。真机当前负载下这项阻塞仍会碰到 3 s 门槛。
- `LocalizationAdapter.set_ready` 在 ready→false 时调用 `NavigationGuard.pause`；
  后续 poll 对导航、跟踪及恢复动作发取消请求，并阻断速度。恢复 ready 不会自动续跑已取消任务。
- 因此该软件调度问题足以解释无人点击时的自动 canceled。历史日志只有客户端取消记录，
  没有持久化失效原因和请求发起者，不能逐次确认每个历史 canceled 都是同一触发。
- 图中导航动作状态发布者各 1 个，`/odom` 发布者 1 个；未发现重复动作状态发布者。
  可发 cancel 的客户端包括本人定位适配器、RViz、waypoint_follower 和行为树相关节点，
  列表本身不能证明哪一个发出了某次请求。

修复方向：将耗时的全候选核验与轻量跟踪进一步解耦，保留质量、歧义、新鲜度及丢失后的取消保护。
本轮未修改正在运行的节点，也没有扩大超时掩盖该问题。
此前一遍 NX 回放测试没有启动 Nav2 控制器，不能代表当前包含导航控制的并发负载。

### 2. 固定支撑拓扑与障碍共同限制可达性（本机生产算法复现）

使用生产 `occupy_map.cpp`、`astar_3d.cpp`，按运行中的参数，加载下载的 support.csv 和固定危险点，
转换为 FLOAT32 模拟 PointCloud2 序列化。按两处失败日志的 projected_start 和首个 goal key 复现。
不创建 ROS 节点，不连接机器人。源码 [reproduce_support_search.cpp](reproduce_support_search.cpp)，
输入 [reproduction_cases.json](reproduction_cases.json)，结果 [search_ablation.csv](search_ablation.csv)。
本机临时可执行文件已删除。

- 案例 0：约 `(3.65,-0.35)` 到 `(0.25,-0.95)`，连接半径 0.25 m + 固定危险点时搜索失败。
- 同一支撑图，诊断时不加载障碍则能搜索成功；只加载 measured 固定危险点仍失败。
  说明不能将此失败单独归因于 synthetic 标记或支撑图完全不连通。
- 仅在本机诊断中把连接半径改为 0.30 m，保留全部固定危险点，核心搜索能够成功。
  当前支撑导出间距为 0.20 m，某些对角连接距离约 0.283 m，大于原 0.25 m。
  这个结果说明栅格拓扑与障碍绕行规则存在耦合，不代表可直接部署这个参数。
  复现没有覆盖实时危险点、起终点所有候选、完整车体外廓及控制器，不能把 search_ok 当真机可通行结论。
- 案例 1：约 `(0.45,-0.55)` 到 `(0.45,-1.55)`，固定图完整加载时核心搜索能够成功，
  与当时实时日志中的失败不同。缺少当时动态障碍快照，不能将该次失败归结为固定图本身；
  已有其他日志明确记录实时 synthetic_terrain_marker 触发外廓碰撞。
- 两个案例的直线路径检查还显示：案例 0 在固定支撑图上仍有 missing_ground_support，
  允许图搜索绕行不等于直线可以通过。

### 3. 控制周期与 TF 是独立的跟踪影响因素

控制器日志持续出现 20 Hz 未达标、map/odom 未来外推与旋转控制器变换失败。
部分取消附近存在这些现象，但不能只靠同时发生确定 CPU 竞争是唯一原因。
10.043 s 实际只读进程采样：重定位约 217.86% CPU，控制器约 30.67%，FAST-LIO 约 32.56%，
地形约 10.26%，EKF 约 22.80%；100% 为一个逻辑核。原始数据 [live_status_sample.json](live_status_sample.json)。
该段包含正在执行的 FollowPath 状态变化，不能当成固定工况或外部性能标定。

### 当前工作边界

完成的是只读诊断与本机离线对照，尚未改动真机业务代码、地图、参数，未启停业务节点，
没有向机器人发目标、速度或取消请求。两个失败的探针均已异常退出；成功探针均在采样结束后销毁并退出。
仅使用本人容器；SSH 会话已关闭。采样日志中的状态和耗时是内部观察值，不是真机验收成功率或外部定位精度。
