# 2026-10-26 地形误判修复与候选地图同步

按用户要求，仅向本人 `zyh-nav2` 容器同步文件。

- 容器 ID：`a1b5c94d58a8808ee209ca50f73164ea6547f7672a2ec3e69e30d8aacc6f5151`。
- 更新源码：`/work/zyh_ws/src/nav2/terrain_costmap/src/terrain_analyzer.cpp`。
- 更新回归测试：`/work/zyh_ws/src/nav2/terrain_costmap/test/test_terrain_analyzer.cpp`。
- 两个更新文件属主保持 `1000:1000`，权限保持原值。
- 修改前备份：`/root/zyh_sync_backups/terrain-fix-sync-20261026-5f65c6ff`。
- 新地图目录：`/work/zyh_ws/maps/lab_v1_navigation_guarded_20261026`，共 6 个文件。
- 新地图含 1300 个支撑点、1089 个危险点；地图身份为 `bd220b8f11e28d90c219cc541f89b7098037e7f47f041f04128070af284a9868`。

独立读取校验确认：两个源码文件及六个地图文件与本机 SHA-256 一致；在容器中直接调用地图加载函数，文件校验及与 `lab_v1_database` 的配对均通过。原 `lab_v1_navigation`、`lab_v1_navigation_guarded_20261025`、定位数据库和 `nav2_mppi_params.yaml` 共 14 个文件的 SHA-256 未改变。

未编译、未修改 install、未启停业务节点、未改变现场参数，未下发目标、速度或机器人 API 请求。没有操作其他容器或修改宿主机设置；源码及地图均写入本人容器已有挂载目录。SSH 已主动断开。

后续操作者需在本人容器中编译 `terrain_costmap` 和 `go2_offline_navigation`，并重新启动有关程序。离线启动命令需要显式指定：

```bash
map_directory:=/work/zyh_ws/maps/lab_v1_navigation_guarded_20261026
```

定位数据库仍使用 `/work/zyh_ws/maps/lab_v1_database`。本次同步没有切换正在运行的地图，也不代表已通过真机通道测试。

- [同步清单](manifest.json)
- [写入结果与备份](sync_result.json)
- [独立校验](verification.json)
