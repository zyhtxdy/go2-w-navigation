# 2026-10-28 角速度补偿同步与编译

按用户要求只操作本人 `zyh-nav2`，工作空间 `/work/zyh_ws`。
更新 9 个文件：桥接补偿/命令/节点源码、配置、说明和两个测试文件，以及在线和离线启动入口。
正负方向按用户要求取同一绝对值模型，默认开启；本次负向仍是对称假设，没有负向真机标定。
保留既有线速度系数、发布频率、超时等桥接设置。
未写入导航调参文件、地图、DDS、宿主机设置或其他容器。

## 编译与核对

在容器正常 build/install 下顺序编译：

```bash
cd /work/zyh_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export MAKEFLAGS='-j2 -l2'
colcon build --packages-select go2_cmd_vel_bridge navigation_bringup go2_offline_navigation --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
```

三个包全部编译成功，退出码 0。没有在 NX 执行测试或启动桥接。
9 个源码文件逐一校验；已安装的三个 Python 模块、桥接 YAML、两份启动文件共 6 项与源码一致。
源码属主保持 1000:1000。仅导入安装后的纯数学模型核对输出，没有构造 ROS 节点：

| 目标角速度 rad/s | 模型提交值 rad/s |
| --- | --- |
| -0.40 | -0.378850 |
| -0.20 | -0.202374 |
| 0 | 0 |
| 0.10 | 0.114136 |
| 0.30 | 0.290612 |
| 0.40 | 0.378850 |
| 0.45 | 0.450000 |

这些数值是模型输出，不是补偿后的新真机测量。
导航参数和地图哈希与同步前相同。没有发送运动请求或执行业务节点的启停/重启。
同步开始时观察到 odom_fusion 的 PID 80；最终核对时该 PID 已不存在，
本轮没有向该进程发送信号或重启指令，仅凭 PID 观察不能确定退出原因。

SSH 已断开。用户后续重新加载正常 `install/setup.bash` 并启动自己的导航/桥接即可使用新版。
角速度补偿默认 true，在线与离线均可通过 `angular_compensation:=false` 关闭。

## 备份与证据

备份位置：`/root/zyh_sync_backups/angular-compensation-20261028-35b3c09a`。
包含 previous-source.tar.gz、previous-install.tar.gz、manifest.json、sync_receipt.json、build.log、build_exit.json。
临时构建脚本已删除，未删除用户文件。
本机证据：remote_before.json、local_manifest.json、sync_plan.json、sync_receipt.json、
build.log、verification.json、status.json。
