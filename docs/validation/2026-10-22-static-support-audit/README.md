# 离线支撑地图本机检查

日期：2026-10-22。用户报告 `/offline_map/support` 的点数和形状变化，RViz Fixed Frame 为 `map`。本轮只在笔记本检查，未连接 NX，未访问机器狗 ROS 域。

## 检查结果

当前本机代码和隔离运行没有复现固定支撑地图变化。不能据此确定此前 NX 运行时的原因，也不能据此保证整条真机导航链路无异常。

| 检查项 | 结果 | 证据 |
| --- | --- | --- |
| 静态地图实际发布 | 早期订阅者只收到 1 帧，891 点，frame 为 map | map_server_report.json |
| 后接入的订阅者 | 收到同一保留帧，时间戳和数据 SHA256 完全相同 | map_server_report.json |
| 地图数据来源 | 消息数据 SHA256 与加载的 support.csv 对应 float32 XYZ 数组一致 | map_server_report.json |
| 同时变化的实时支撑话题 | 发布 32 帧 `/terrain/support`，未触发 `/offline_map/support` 更新 | map_server_report.json |
| 发布者 | 隔离测试域中只有 offline_map_server 发布固定支撑 | map_server_report.json |
| 离线规划参数 | use_static_map=true；support_topic=/offline_map/support | launch_report.json |
| 障碍输入 | 固定障碍为 /offline_map/obstacles；实时障碍为 /terrain/obstacles_global | launch_report.json |
| 本机源码与安装版本 | map_server.py、map_io.py、offline_nav.launch.py 字节一致 | launch_report.json |

隔离运行使用 ROS_DOMAIN_ID=148、ROS_LOCALHOST_ONLY=1，清除自定义 DDS 配置。只启动地图发布器和检查节点；launch 参数检查仅展开配置，没有启动导航节点。测试进程已退出，未修改导航代码、地图或参数。

## 代码链路

- `map_server.py` 启动时加载地图，通过 `published` 标志只发布一次固定点云；随后仅发布 ready 心跳。
- `offline_nav.launch.py` 把规划器支撑输入设置为 `/offline_map/support`，并关闭高度漂移补偿。没有把实时 `/terrain/support` 重映射到固定地图话题。
- 规划器内部可视化输出是 `/planner/support_map`，没有反写 `/offline_map/support`。实时障碍更新不修改支撑数据容器。
- 地图消息本身处于 `map` 坐标系。RViz 固定坐标系同为 `map` 时，定位对 `map→odom` 的修正不能解释这些原始 XYZ 数据改变。

## 仍未确定

- 此前运行时是否有其他同名发布者，或同时运行了多个地图发布器。当前源码只发现一个生产发布器，但离线规划器尚未检查发布者唯一性或输入地图身份。
- RViz 当时实际启用了哪些显示项。工作空间离线配置同时启用固定支撑和实时对齐点云；本机默认保存配置中只有已知地图与实时对齐点云，没有固定支撑显示项。这只能提示叠加显示的可能性，不能证明用户当时选错话题。
- 此前是否存在未保存的 GUI 配置或运行版本差异。本机当前源码与安装版本一致，不代表过去每个运行实例都一致。

本机现有 RViz 日志的丢帧提示针对 `odom` 坐标系点云，不能解释 `map` 坐标系固定支撑的点数持续变化。现场尚未取得连续消息内容和发布者记录，因此本轮不将上述候选原因定为结论。
