# 2026-10-28 首次目标 path 未初始化修复

用户在关闭另一个容器解决 TF 冲突后，导航仍立即失败。
现场 bt_navigator 日志确认目标已经到达，并反复报：

```text
Behavior tree threw exception: IsTerrainPathValid requires path. Exiting with failure.
```

这是上轮行为树调整的回归：IsTerrainPathValid 被放在 GlobalUpdatedGoal 前面，
首次目标尚未由 ComputePath 写入黑板 path，节点把正常未初始化状态当成了配置错误。
旧测试显式写入空 Path，漏掉了真正没有 path 值的 Nav2 首次状态。
现场证据保存在 ../2026-10-28-odom-tf-diagnosis/goal_logs.json。

## 修复

读取失败时，只接受“有合法黑板映射，但值尚不存在或为空”的首次状态，
走既有 initial_plan_required 分支：清 unavailable，不发校验 RPC，交给 Fallback 首次规划。
缺少端口映射、非法字面量或已有错误类型仍报错。
已有路径的真实服务校验、连续几何失效计数、不可用输入停车规则没有变化。
同一插件服务于单目标和多目标两份行为树，不需要放宽任何地图或碰撞参数。

## 本机验证

navigation_bringup 编译成功，12 项 BT/插件测试全部通过。
新增四项检查：未赋值黑板不发 RPC；首次规划写入路径后正常校验并可处理下个目标；
缺映射继续报错；错误类型继续报错。
首次规划集成检查使用真实 Fallback/Guard 和假的 Plan，不是补做真机运动。
原数据缺失、RPC超时、停跟踪与三次几何失效等测试继续通过。

## NX 安装

更新本人 zyh-nav2:/work/zyh_ws 的两个源码/测试文件，属主保持原值。
在容器内暂存目录 Release 编译 navigation_bringup，BUILD_TESTING=OFF，退出码 0。
将新的 BT 共享库以临时文件加 os.replace 原子替换到正常 install，
避免截断可能被运行进程映射的旧库；新库与暂存编译产物 SHA256 一致，动态依赖解析通过。
旧 inode 817563，新 inode 815357。未改动安装环境 hooks 或导航参数。

备份：`/root/zyh_sync_backups/first-goal-20261028-faa15cb5`。
原源码保存在 previous-source.tar.gz，原插件保存在 previous-bt-plugin.so。
本轮未启动、停止或重启业务节点，未代发 goal 或速度请求，未操作其他容器。
SSH 已断开。**旧进程不会因 source 环境而自动换库，需用户退出并重启自己的导航 launch。**
真机目标能否成功仍需用户重新下发确认，本轮没有代发实际导航任务。

证据：build.log、tests.log、test_counts.json、nx_build.log、remote_before.json、
source_hashes.json、changes.diff、sync_plan.json、install_verification.json、status.json。
