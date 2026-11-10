# 本机完整执行链诊断

`execution_smoke.py` 只使用原始录包、原场地导航地图。默认只打印启动清单；需要显式 `--execute` 才运行。

等待本机新代码编译完成后执行：

```bash
cd /home/zyhtxdy/go2-w_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 docs/validation/2026-10-29-coherence-review/integration/execution_smoke.py \
  --execute --domain 196 \
  --output docs/validation/2026-10-29-coherence-review/integration/run
```

使用私有 localhost ROS 域。离线入口 `start_navigation:=true`、`start_cmd_vel_bridge:=false`、`publish_known_map:=false`；不会创建宇树运动 API publisher。每轮结果目录必须不存在。

检查内容：真实 NavigateToPose 行为树接受目标、生成 `/plan`；controller 到 smoother 再到 offline gate 的私有速度话题；主动取消与零指令；取消后短时地形输入故障是否仍被视为空闲；活动任务中只暂停本脚本进程组内的 terrain 节点，记录观测过期与速度输出停止。暂停节点会在清理时先恢复，再终止自己创建的子进程组。

FAST-LIO 无条件写入的原始调试日志会先备份并在子进程全部停止后恢复。若已有本机 FAST-LIO，或域内已有其他节点，则拒绝运行。

输出保留每个子进程日志、实际命令、原始速度样本、路径与检查结果。bag 中的位姿并不受计算指令驱动，不能把自然到点、动作成功或取消状态当作闭环跟踪／真机通过结论。地形故障检验的是软件输出停止，不是机器狗物理停止距离。

2026-10-29 已完成一次本机完整检查，使用 `--navigation-map /home/zyhtxdy/go2-w_ws/docs/validation/2026-10-26-live-path-obstacles/navigation_candidate`，输出 [run_candidate/report.json](run_candidate/report.json)。11 项检查全部通过，五个自建进程均正常退出，FAST-LIO 原日志恢复成功，真机测试 0 趟。
