# 2026-10-27 路径校验分类与恢复

本轮仅在本机修改、编译和软件检查；不连接 NX，不启动运动接口或发送速度。

旧 `/terrain/is_path_valid`（Nav2 IsPathValid）保留，返回值仍由同一套校验生成。
新增 `/terrain/is_path_valid_detailed`（terrain_navigation_interfaces/srv/ValidateTerrainPath），
在同一个服务响应中返回 is_valid、status、reason、invalid_pose_indices，避免异步诊断和请求错配。
自定义 path_validity_service 时，详细接口是在原服务名后加 `_detailed`。

| 分类 | 处理 |
| --- | --- |
| VALID | 继续跟踪，清零失效计数 |
| COLLISION | 按原连续三次失效规则触发重规划 |
| TERRAIN_INVALID | 地面/高度/连接不可行，按原连续三次规则重规划 |
| PATH_MISMATCH / INVALID_REQUEST | 路径关联/坐标系不符，按原规则重新规划 |
| LOCALIZATION_UNAVAILABLE | 停止跟踪，等待数据，不重新搜索 |
| OBSTACLES_UNAVAILABLE | 障碍源未就绪/过期，停止跟踪并等待 |
| TF_UNAVAILABLE | 变换缺失，停止跟踪并等待 |
| MAP_UNAVAILABLE / INTERNAL_ERROR | 地图不可用/异常，停止跟踪并等待 |
| 服务超时/通信异常/未知或矛盾结果 | 保守视为不可校验，不当作可通行 |

碰撞分类保留起点投影和连续地面检查中的已有碰撞证据；详细 reason 保留原失败原因。
未改变地面、外廓、膨胀、传感器时间有效期或速度参数。

两个默认行为树均接入新接口。TerrainReplanAllowed 在输入不可用时阻止规划和上下文清图。
TerrainValidationGuard 包住整个跟踪 PipelineSequence：校验不可用导致 pipeline 失败并 halt FollowPath，
然后每 0.5 秒重试校验；单次等待上限 5 秒，超时交回原有有限次数恢复流程。
不能直接让校验条件返回 RUNNING，否则已启动的 PipelineSequence 仍可能继续 tick FollowPath。
数据不可用不累计几何失败次数；恢复后重新校验，不直接把旧响应当作有效。
RPC 用稳态时间兜底超时，超时清理待处理请求。

离线定位适配器原有停车、取消旧任务和重新下目标的规则保持；不会自动重发被取消的任务。
行为树 halt 是软件跟踪任务取消，不是真机制动距离或底层故障停车的保证。
新接口需要规划器、BT 插件和两份行为树一起更新。旧自定义行为树如果继续使用
IsTerrainPathValid 插件，需把 service_name 改到详细接口并接入相应输出/守卫。

测试覆盖真实公开规划服务的有效/碰撞/TF缺失/定位未就绪/障碍过期/地图未就绪/错误坐标系，
以及 BT 的三次几何计数、数据失效立即中断、RPC超时恢复、整个 pipeline 的控制器 halt、
无效输入不调用规划器、等待有上限。测试使用隔离 ROS 域和本机通信，无真机测试趟数。
最终编译与测试结果见 build.log、tests.log 和 test_counts.json；首次失败记录见 tests_first.log。

最终结果：Release 编译 3 个包通过；259 项规划器测试、8 项行为树/插件测试全部通过，无失败、错误或禁用。
两份实际行为树结构检查通过；变更与源码指纹见 changes.diff、source_hashes.json。
首次失败是测试初始化顺序和起点障碍分类缺失，已修正后重新完整检查。
本机安装已更新；相关进程需要重启加载新库，NX 尚未同步。
