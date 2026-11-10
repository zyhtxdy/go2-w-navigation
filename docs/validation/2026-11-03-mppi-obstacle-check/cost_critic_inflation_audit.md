# CostCritic 与局部膨胀范围的版本核对

## 已确认的部署版本

NX 的 controller_server 加载 /opt/ros/humble/lib/libmppi_controller.so 与 libmppi_critics.so；nav2_mppi_controller 包版本 1.1.20；两份加载库的 MD5 都与系统软件包清单一致。证据见 loaded_mppi_version.json。

## 源码行为

对应版本源码：
https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_mppi_controller/src/critics/cost_critic.cpp

score() 先读取轨迹点中心的 costmap 代价。若该代价小于 1，直接 continue；后续 inCollision() 的 footprint 检查不会执行。consider_footprint=true 不会绕过这条提前跳过分支。

当前 NX 参数及快照：inflation_radius=0.25 m；矩形 ±0.40、±0.25 m；padding 约 0.01 m；当前地图只有 lethal/inscribed 与 free 格，无中间软代价格。

带 padding 后配置边界约 ±0.41、±0.26 m，外接半径 hypot(0.41,0.26)=0.485489 m。膨胀区域远小于矩形外接范围，可能出现矩形已靠近/覆盖障碍而中心仍为零代价的轨迹点，继而跳过该点的碰撞检查。这个版本使用中心代价快捷判断的前提与当前配置不匹配；增大 collision_cost 权重不能修复零代价提前跳过。

## 修改建议及范围

优先单独验证 local_costmap 的 inflation_layer.inflation_radius=0.55 m，覆盖当前配置的外接半径并留出网格余量；保持实际 footprint 与 padding，不改全局硬碰撞半径。该值为建议配置，尚未修改、尚未真机对照。

0.55 m 是以障碍为中心生成局部软代价的距离，并非把机器人硬碰撞半径改成 0.55 m；外圈代价递减，不能据此推导 1.1 m 通道必然被判定为不可通行。

此前建议的 0.40～0.45 m 可以产生软梯度，但仍不足以覆盖约 0.486 m 外接半径，故此处修正为首先验证 0.55 m。

## 判断边界

这是与部署版本对应的明确配置风险，能解释有障碍数据却缺少转向/避障惩罚的现象。没有当时局部 costmap 与候选轨迹评分，仍不能断言它是每次未绕行的唯一原因。全局路径校验提前重规划、局部有限预测时域和真实通行空间仍需分别验证。

本次仅只读检查，没有修改源码、运行参数或安装配置，没有发运动指令，SSH 已关闭。
