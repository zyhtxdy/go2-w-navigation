# MPPI 自身 CostCritic 的直接评分复现

## 范围

只检查 MPPI CostCritic，不使用全局规划器、Rotation Shim、速度桥接或真机。没有连接 NX。本机使用私有 ROS_DOMAIN_ID=198、ROS_LOCALHOST_ONLY=1；没有导航/控制服务器，没有运动发布者。

链接本机实际安装的 /opt/ros/humble/lib/libmppi_critics.so，调用 mppi::critics::CostCritic::score，版本 1.1.20，与此前核实的 NX MPPI 版本一致。没有重写该函数进行替代评分。

## 输入

沿用实际 NX 的 0.35 m 原始 costmap、当前矩形及 padding。将此前查到的 2610 个“中心代价为零、矩形代价为254”的姿态，各自构造成一条 50 个相同姿态的诊断候选；此构造专门检查碰撞评分，不代表真实运动学可达轨迹或实际执行轨迹。路径终点设在状态位置 3 m 外，排除 near_goal 分支的干扰。

## 结果

| 项目 | 直接调用结果 |
|---|---|
| 离线诊断候选数 | 2610 |
| 中心代价为零 | 2610 |
| Nav2 矩形检查返回致命碰撞254 | 2610 |
| 实际 CostCritic 输出障碍代价零 | 2610 |
| 实际 CostCritic 最小/最大代价 | 0 / 0 |
| all_trajectories_collide 标志 | false |

这是 MPPI 自身评分结果的直接复现，不只是对源码的推断。

另外，给同一评分器输入 56 个中心位于原始致命障碍格的阳性对照，输出均非零，范围3～300。说明评分器并未被关闭，也不是全部输入都会得零分。

## 可以确认的结论

在该版本、该原始地图与这些诊断候选下，中心零代价快捷分支跳过了真实存在的矩形碰撞，碰撞没有转化成障碍惩罚，所有候选碰撞标志也未触发。因此，即使 costmap 里存在障碍、consider_footprint=true、collision_cost 很大，评分器仍可能把这类碰撞候选当作没有障碍代价。这是 MPPI 评分分支的具体缺口。

这可以解释为何 MPPI 可能继续给出朝向障碍的控制、同时不报“所有轨迹碰撞”。没有事发时实际生成/选中的轨迹记录，仍不能把这些诊断候选认定为那次机器人实际执行的轨迹。

## 修复应对准的位置

CostCritic::score 的中心零代价提前跳过，以及 inCollision 基于中心代价判断是否做矩形检查的前置条件。修复必须保证矩形真实碰撞能进入惩罚，不能仅提高 collision_cost 权重。若保留快速判断，需要独立、保守地证明整个矩形范围无障碍，并测量计算开销。

本次仅完成独立复现，未修改生产 MPPI 库或参数。

## 证据

- probe/exact_critic_probe.cpp：调用安装库的诊断程序。
- exact_critic_results.json：结构化结果与隔离条件。
- exact_critic_actual.log：真实地图诊断候选的原始日志。
- exact_critic_positive_control.log：阳性对照原始日志。
- exact_critic_counterfactual.log：同障碍的欧氏扩展代价场诊断；不是实际 InflationLayer 重放，也不是修复或真机验证。
