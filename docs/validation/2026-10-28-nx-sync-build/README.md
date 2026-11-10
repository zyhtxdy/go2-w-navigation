# 2026-10-28 NX 同步与编译完成

只操作本人 `zyh-nav2`（容器 ID a1b5c94d58a8），工作空间 `/work/zyh_ws`。
首次旧地址不通；用户开机后连接成功。容器当时停止，先只读检查 entrypoint、root bashrc 和系统 bashrc，
确认只有环境加载，没有业务自启动，再临时开启容器进行同步编译。

## 更新与参数

更新 28 个源码、构建、行为树、测试和配置文件；检查 163 个范围内文件，
除有意保留的远端参数差异外均与本机清单一致，合并配置也按预期 SHA256 验证。
新增 terrain_navigation_interfaces，配套更新规划器和 BT 插件、两份默认行为树，
包括之前完成的候选恢复、支撑推断、矩形必要预筛、旧路径前向重接和校验失败分类。
未实现或上传用户明确暂缓的统一搜索限时改造。

保留 NX 全部原有导航字段及现场调参，局部 inflation_radius 仍为 0.25 m。
只补入 14 个新版缺失字段（固定地图校验默认项、带朝向搜索/路径优化、支撑推断），
具体内容见 parameter_additions.json；语义对比确认其余参数不变。
更新文件属主保持 1000:1000，新建源码目录同样为 1000:1000。
没有删除远端独有文件，没有上传或修改地图、DDS、宿主机设置或其他容器。

## 编译与安装核对

在正常工作空间以 Release、BUILD_TESTING=OFF、包间顺序及最多 2 个 make 作业编译：

```bash
cd /work/zyh_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export MAKEFLAGS='-j2 -l2'
colcon build --packages-select terrain_navigation_interfaces terrain_costmap astar_3d_planner navigation_bringup fast_lio_nav2 go2_offline_navigation --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
```

6 个包全部成功，退出码 0。go2_offline_navigation 的 PCL 配置报告可选 pcap/png IO 未启用，
未导致构建失败。没有执行 ROS 测试或启动业务节点。

安装后的新接口头文件存在，规划器和 BT 库的动态依赖全部解析。
两份安装行为树与源码一致；navigation_bringup、fast_lio_nav2、go2_offline_navigation
安装中的三份导航参数与远端合并后源码一致。地图文件哈希与同步前相同。

全程没有启动传感器、建图、定位、导航、控制器、速度桥接，没有下发目标或运动请求。
最终进程核对只剩 PID 1 的空闲 bash。编译后将容器恢复为原先停止状态，SSH 已断开。
后续由用户开启容器、source 正常 install，再自行启动测试；旧 setup_static_test.bash 不属于本轮安装入口。

## 备份与证据

容器内备份：`/root/zyh_sync_backups/source-sync-20261028-cc0c6fe5`。
previous-source.tar.gz 保存覆盖前源码，previous-install.tar.gz 保存六包原安装目录；
manifest.json、sync_receipt.json、build.log、build_exit.json 保留。临时构建脚本已删除。

本机证据包括 local_manifest.json、remote_preflight.json、sync_plan.json、sync_receipt.json、
source_verification.json、install_verification.json、build.log、library_dependencies.log 和 status.json。
源码同步和编译通过不代表本次做过真机验证。
