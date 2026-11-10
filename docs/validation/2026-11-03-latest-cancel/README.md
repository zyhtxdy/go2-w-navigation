# 2026-11-03 最近一次导航取消排查

仅只读检查本人 NX 的 zyh-nav2 容器。未发布运动指令，未启动、重启或修改导航节点。

## 直接原因

北京时间 15:58:00 的整任务取消，由 offline_localization 门控主动发起。门控先进入可恢复的 alignment_expired 等待，约 0.476 秒后因 confirming_alignment_or_correction 转入取消分支。此处等待重新确认与真实变换修正共用原因，而该原因未被归入可恢复等待。

## 时间线（北京时间）

| 时间 | 事件 |
|---|---|
| 15:57:38.874 | 本次导航开始 |
| 15:57:49.848 | footprint_collision 连续判定触发重规划；该时刻没有取消整个目标 |
| 15:58:00.211 | alignment_expired；停车并保留任务等待恢复 |
| 15:58:00.288 | 控制器报 map→odom 未来外推错误 |
| 15:58:00.447 | 行为树报告 localization_not_ready，停止跟踪 |
| 15:58:00.687 | confirming_alignment_or_correction；门控请求取消活动任务 |
| 15:58:00.758 | navigate_to_pose 整个目标记录 Goal canceled |
| 15:58:01.964 | 门控再次 ready，但之前的目标已经被取消 |

从进入等待到取消约 0.55 秒；从硬取消分支到 ready 约 1.28 秒。均为这 1 次取消事件的日志时间差，并非外部测量。

## 代码核对

`localization_adapter.py` 的 `try_accept` 在上游允许发布但自身确认帧尚不足时，设置 `confirming_alignment_or_correction`。`set_ready` 仅把 `localization_awaiting_distinct_consistent_frames`（且已处于 hold）作为确认阶段可恢复原因；前者落入 `navigation_guard.pause()`，请求取消。

定位超时后重置确认，恢复的新帧再次需要确认，因此此分支可能把正常重新确认变成取消。日志足以确认本次取消分支，尚不足以确定最初 alignment_expired 的底层原因或证明当时没有真实定位修正。没有同时间段的定位匹配指标、候选变换序列和性能记录。

## 建议修复方向

- 区分正常重新确认与真实定位跳变。
- 正常重新确认在原有有界恢复期限内保持零速度与同一导航目标；原因变化不重置期限。
- 真实定位跳变、地图不一致和恢复超时仍保持明确失败处理。
- 恢复后重新验证剩余路径再放行运动。

本次仅排查，没有实施上述代码修改。

## 证据

- `recent_log_events.json`：最近几轮导航、控制器和重定位事件。
- `cancel_window.json`：本次取消前后各节点原文，包含 Python 门控日志。
