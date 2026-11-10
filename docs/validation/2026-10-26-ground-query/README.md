# 2026-10-26 支撑查询统一与规划拒绝诊断

本轮仅在笔记本修改、编译和运行软件测试，没有连接 NX、同步容器、启动运动桥接或下发机器人目标。没有改地图文件、碰撞外廓、速度、MPPI 采样数、Rotation Shim 或倒车配置。

## 与今天失败记录的关系

1. `2026-10-26-stop-repair` 已确认目标 `(0.18,-1.51)` 附近有支撑，却被旧的整数栅格圆形筛选漏掉。这一米制半径修复保持有效，并保留对应的单元及公开插件回归。
2. `2026-10-26-angular-response` 中后一次目标先后得到约 0.583 m 和 0.268 m 的直达路径，后来出现 `Failed to make progress`、`global_obstacle_source_stale`、TF 外推及定位超时。本轮支撑推断不能据此宣称解决执行停滞或取消；数据时效和停车保护没有放宽。
3. `2026-10-26-midroute-stop` 中有新鲜实测回波和静态障碍触发矩形检查。缺少历史完整姿态、路径及回波，不能删掉它们当作误报。本轮保留全部障碍与矩形扫掠检查，新增直线候选被拒绝的诊断，即使最终绕行成功也能看到原因。

## 本轮实现

测量地图与查询推断分开保存。原 `support_columns_`、`support_heights_` 和 `/planner/support_map` 始终只含实测支撑；固定地图输入的哈希、CSV 与地图身份检查不变。没有向 `/terrain/support` 或 `/offline_map/support` 写入任何推断点。

启用 `GridBased.support_inference_enabled` 时，候选节点生成、本列地面高度查询、连接检查、路径拉直、密集路径复核和缓存路径复核均读取相同的地面层查询结果。旧的“借用单侧一个邻点”扩张在此模式下停用。关闭开关恢复原有测量候选及邻近借用行为。

仅对没有实测地面列的位置尝试推断，要求：

- 半径内至少三个不同 XY 列的实测样本，几何不共线，查询点严格位于其凸包内部；单侧点、线状点及边界外推均拒绝。
- 按高度带分别拟合。一个高度带不能有同列多层歧义，不混合楼层，不在已有实测列里补另一层。
- 检查高度跨度、平面坡度及最大拟合残差。只使用实测点，推断点不参与后续推断，不能逐步向未知区域生长。
- 每个候选和整条路径仍检查原有障碍、台阶、坡度与车身扫掠；推断出地面不表示这个位置没有障碍。

在线及离线默认参数文件已启用，初始软件界限如下，不是传感器精度测量：

| 参数 | 值 |
| --- | --- |
| support_inference_enabled | true |
| support_inference_radius | 0.25 m |
| support_inference_max_height_span | 0.06 m |
| support_inference_max_residual | 0.01 m |
| support_inference_max_slope_deg | 10° |
| support_inference_max_columns | 32768 |

这是小空缺的局部地面假设，不是地面观测，也不能证明未观测处没有落差。大范围脚下盲区仍沿用既有受限起步连接规则。半径内证据不足时保持未知，不强行铺平。

推断按查询懒计算，同一支撑更新代内缓存；新的支撑更新或 reset 会清空缓存，真实测量优先。缓存达到上限后，新的未知列保持未知。障碍更新不复用通行结论，碰撞检查始终读当前障碍。

## RViz 和终端诊断

Fixed Frame 设为 `map`，新增两个 MarkerArray 显示：

| 显示类型 / 话题 | 内容 |
| --- | --- |
| PointCloud2 `/planner/support_map` | 规划器实际保存的实测地面；颜色可自行设置 |
| MarkerArray `/planner/ground_queries` | 青色：推断地面；橙色：查询后仍未知的位置 |
| MarkerArray `/planner/direct_path_markers` | 被拒绝的首个直线候选的位置和原因文字 |

`ground_queries` 只显示当前支撑更新代中查过的缺失列，最多 2048 点、每秒最多更新一次；它不是整张未知区域地图，也不是通行代价地图。Marker 的 namespace 区分没有足够样本、未被支撑包围、高度不一致、坡度或残差超限等原因。

```bash
ros2 topic echo /planner/direct_path_status
ros2 topic echo /planner/start_failure
```

直线状态的 `event=direct_candidate_rejected` 只表示一个直线候选被拒绝；`scope=first_rejected_goal_option` 说明保留的是首个被拒绝选项，不表示整个任务失败。状态包含拒绝位置、碰撞格、障碍来源及支撑/障碍时间戳。每次新规划会先清除上次的直线状态和标记。

## 验证结果

- 本机 Release 编译 `astar_3d_planner`、`navigation_bringup` 成功。另编译安装 `go2_offline_navigation`，确认在线、离线安装目录的默认参数均启用本轮开关。
- 规划器 205 项测试、导航插件加载 1 项测试全部通过，无失败、错误或禁用。新增的 12 项支撑推断检查及 3 项公开规划插件检查包含在上述总数中。
- 覆盖小空缺参与搜索、直线检查与密集复核的一致性；单侧支撑、线状样本、大空洞、非平面噪声、坡度、不同高度层、实测替代、缓存失效、并发读及实时障碍阻挡。
- 公开插件构造的稀疏 20 cm 地面格案例输出直线路径，并确实收到推断来源的 RViz 标记。构造的真实障碍案例输出绕行路径，并收到 `occupied_body_column`、`static_map_hazard` 的直线拒绝状态。大范围未知案例仍拒绝规划。
- 首轮噪声案例的 4 cm 单点变化恰好对应平面最大残差 1 cm，等于允许界限，原断言却要求拒绝。将测试输入改为 5 cm 变化以测试明确超限，没有改变业务残差界限；随后该项和完整检查均通过。

保存的今日导航地图：`../2026-10-26-live-path-obstacles/navigation_candidate`，1300 个实测支撑。`saved_map_queries.jsonl` 比较六个目标的静态地图核心搜索，包含上述两处日志目标。开关前后六例都能搜索到路线，实测快照始终为 1300 点；启用后多出有来源标记的推断候选。

该六例不是历史真机故障完整回放：没有当时完整姿态、动态障碍与控制命令，不包含完整矩形动作与机器人执行。查询/搜索耗时在本机关闭时约 0.58～1.01 ms，开启时约 2.20～6.11 ms，均为本轮单次软件计时，不能用来推断 NX 时延或真机成功率。

证据：`build.log`、`offline_build.log`、`focused.log`、`plugin_focused.log`、`tests_full.log`、`test_counts.json`、`saved_map_queries.jsonl`；源码变化见 `changes.diff`。没有做真机运动验证，也没有同步 NX。
