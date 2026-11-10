# NX 保护版导航地图上传

2026-10-26。按用户希望在 NX 复测本机保护版地图的要求，仅在本人 zyh-nav2 容器中上传文件。

- 新目录：/work/zyh_ws/maps/lab_v1_navigation_guarded_20261025。
- 定位数据库继续使用：/work/zyh_ws/maps/lab_v1_database。
- 上传 6 个地图与诊断文件，共 420299 字节，独立复查 SHA-256 全部一致。
- 地图身份：e255fca6c2a5037976ee02a9f3e33df6de371bf3fb902c0599b5e11f980454b1。
- 支撑点 1177，危险点 1212；这些为导图算法输出计数。

上传前核对 NX 数据库 map.pcd、manifest.csv 与地图声明的签名相同。
上传后在容器内用新版源码的 load_map 验证文件校验值、数据库配对和地图身份，通过。
另行读取核对确认原 lab_v1_navigation 目录和定位数据库未改变。

没有覆盖原地图，没有编译、修改启动参数或启动任何节点，没有目标、速度及机器人 API 请求。
SSH 已断开。新地图未做 NX 真机导航验收。

沿用离线启动命令，将 map_directory 替换为新目录即可；数据库参数保持原路径。
此前刚同步的源码仍需要编译后使用。

- [上传结果](upload_result.json)
- [独立校验](verification.json)
- [上传前基准](manifest.json)
