# 本机与 NX 源码同步（2026-11-10）

仅访问 `unitree@10.56.61.177` 的本人 `zyh-nav2` 容器，工作空间 `/work/zyh_ws`。容器同步前后均为停止状态，通过 Docker cp 完成读取和复制，未启动容器、编译、运行 ROS 节点或下发任何运动请求。未操作其他容器或修改宿主机配置。

## 比较与合并

同步前，比较范围内 507 个源码文件相同，4 个文件有差异。保留 NX 现场调参，将其同步到本机；没有以本机旧默认值覆盖 NX：

- `src/FAST_LIO_ROS2/config/xt16.yaml`：本机 `blind` 从 0.50 同步为 NX 的 0.38。
- `src/nav2/navigation_bringup/config/nav2_mppi_params.yaml`：本机采用 NX 的 17 项现场参数差异，包括 Shim 进入/退出约 25.73°、转向速度 0.50、MPPI 64×0.05s、vx/wz 上限 0.60、采样标准差 0.15/0.45、PathAlign 权重 6、6m 局部地图、膨胀半径 0.52/衰减 12 及速度平滑器参数。使用本机文件布局并修正相应注释，解析后的参数结构与 NX 原文件完全相同；合并文本同时写入两端。
- `src/go2_relocalization/config/relocalization.yaml`：两端参数值原本相同，仅将本机较新的注释同步到 NX。
- `src/FAST_LIO_ROS2/README.md`：将本机整理后的上游媒体链接同步到 NX。

未改导航算法或测试。未同步地图、录包、build/install/log、第三方大型文档媒体和 Git 元数据；NX 原有子模块声明保留，没有把本机仓库整理扩展为远端清理。

## 校验和生效范围

最终 511 个范围内文件 SHA256 与本机一致。NX 仅 3 个文件发生文本更新，其他范围内文件和全部文件的 UID/GID/权限保持不变；所有改动 YAML 解析通过。

本次未更新两端 `install/`，当前安装配置不一定与源码相同。后续由操作者编译并重新启动自己的程序后加载新配置。本次没有真机功能测试。

远端更新前备份：`/work/zyh_ws/.codex_sync_backups/source_20261110_103835.tar.gz`。

本机完整比较、旧文件及校验结果：`/home/zyhtxdy/go2-w_ws_archive/nx_sync_20261110_103835/`。SSH 已断开。
