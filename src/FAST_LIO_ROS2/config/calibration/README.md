# XT16 校准文件来源

这里的 `xt16_angles.csv` 和 `xt16_firetimes.csv` 复制自禾赛官方 SDK 的
`PandarXT-16_Angle Correction File.csv` 和 `PandarXT-16_Firetime Correction File.csv`。

- 上游：https://github.com/HesaiTechnology/HesaiLidar_SDK_2.0
- SDK 提交：`9d5dc4fc4ade5be5f6a6ca00e71dd4050b054168`
- ROS 驱动提交：`e7e112f0809f0eed5e3c81c55a1a0376474db234`
- SDK 许可：见 `src/driver/HesaiLidar_ROS_2.0/src/driver/HesaiLidar_SDK_2.0/LICENSE`。

这些是厂商的示例文件，并非已确认对应用户这台 XT16 的逐台标定文件。
取得序列号对应的校准文件后，在 `hesai_xt16.yaml` 中替换路径。
相对路径由 FAST-LIO 的启动文件按该 YAML 所在目录解析。

## Go2W / XT16 安装外参来源（2026-10-09）

下列安装关系与上面的雷达内部角度、发射时序校准文件是不同参数。
代码使用名义安装参考值；当前机器的轴向、支架尺寸和外参没有真机验证。

### 本体 IMU 与机身

宇树官方 Go2W URDF 的 `imu_joint` 定义 `base -> imu`：
平移 `[-0.02557, 0, 0.04232]` 米，RPY `[0, 0, 0]`。
官方 MuJoCo Go2W 模型的 IMU site 使用相同位置。

- [宇树官方 Go2W URDF](https://github.com/unitreerobotics/unitree_ros/blob/master/robots/go2w_description/urdf/go2w_description.urdf)，文件 Git blob：`c3798d6839875ca756ddd7c007d9d4f480388bab`。
- [宇树官方 Go2W MuJoCo 模型](https://github.com/unitreerobotics/unitree_mujoco/blob/main/unitree_robots/go2w/go2w.xml)，文件 Git blob：`88b7dbe2f6ca775a60cb1ddb2e4c2f1ee9573c5a`。

本工作空间把 `base_link` 定义为上述机身 `base` 的原点和轴向；不是地面投影帧。
FAST-LIO 的 `body` 表示所使用的本体 IMU 坐标系，因此独立建图启动中
`body -> base_link` 的名义默认平移为 `[0.02557, 0, -0.04232]` 米，旋转为零。
此 TF 默认不发布，需要 `publish_body_base_tf:=true` 显式启用。
更换 IMU、定义不同的 `base_link` 或已有 TF 发布者时，需要覆盖参数并核对 TF 树。

### XT16 与本体 IMU

依据用户提供的 [Go2 SLAM 与导航资料](https://github.com/mamdaliof/Unitree-GO2-Quadruped-Robot-for-autonomus-navigation-and-SLAM/blob/master/Go2_Documentation/Software_Interface_Services/03_slam_and_navigation.md)
第 2 节的描述，参考原点是机器人内部 IMU 中心；XT16 的完整矩阵给出的平移为
`[0.1710, 0, 0.0908]` 米，旋转为单位矩阵。
该链接为第三方整理资料，文件 Git blob：`14ff9ce5538d46609308faad3d17fc743f816449`；
不是宇树官方 GitHub 仓库，也没有证明与当前 Go2W 支架版本完全一致。

按其文字定义将矩阵作为 `T_IMU_LIDAR` 安装参考初值使用：

```text
p_IMU = R_IMU_LIDAR * p_LIDAR + t_IMU_LIDAR
```

最初据此设置 `extrinsic_T=[0.1710, 0, 0.0908]`、`extrinsic_R=I`。
此平移已相对 IMU，不能再减去 `base -> IMU` 平移。
数值按该资料的描述解释，不单凭 `T_Dog2lidar` 变量名推断变换方向。
`extrinsic_est_en=false` 保持不变，不宣称本次完成在线或离线外参标定。

#### 当前试验旋转（2026-10-09）

用户报告遥控倒车时 `/Odometry` 显示侧向位移，并明确要求先试 `+90°`。
当前 `xt16.yaml` 的 `R_IMU_LIDAR` 改为绕 +Z 轴的右手 +90°：

```text
[0 -1  0]
[1  0  0]
[0  0  1]
```

它把雷达 +X 映射到本体 IMU +Y，把雷达 +Y 映射到本体 IMU -X。
平移仍为 `[0.1710, 0, 0.0908]`，不旋转该已用 IMU 坐标表达的平移。
此值由用户指定用于试验，没有完成安装轴向验证或外参标定。
需要重新启动 FAST-LIO 初始化，并清除 RViz 已累计的旧点云后比较效果。

使用前仍需检查：

- `/lowstate.imu_state` 的实际坐标轴与模型 IMU 是否一致。
- 当前 `/hesai/points` 的 x/y/z 是否符合该资料的 XT16 坐标约定。驱动保持 `transform_flag=false`，避免重复补偿安装变换。
- XT16 实际安装中心与方向是否符合参考值；该资料写 Go2，Go2W 的适用性仍需当前机器验证。
- `go2_body` 是运动状态速度的参考帧；这份 XT16 资料没有确认它的原点，后续采用独立的官方 Go2W 仿真模型作为参考初值，见下面说明。
- 时间同步是独立问题；外参修改没有改变任何输入的时间戳来源。

其他研究平台或不同支架的外参只可作对照，不能混用其旋转矩阵或平移。

### 融合启动中的 TF 接入

`fast_lio_nav2/odom_fusion.launch.py` 默认发布 `base_link -> body` 和
`base_link -> go2_imu`，共用上述官方 Go2W 模型的 `T_base_imu`，即
平移 `[-0.02557, 0, 0.04232]` 米、零旋转。可通过
`base_to_imu_x/y/z/roll/pitch/yaw` 覆盖；外部已有这两条 TF 时，设置
`publish_body_imu_tf:=false`。两条 IMU 帧别名的安装变换保持一致。

融合中只由 EKF 发布 `odom -> base_link`，不启用独立建图的
`body -> base_link` 或 FAST-LIO 的 `odom -> body`，避免多个父帧。
运动状态参考帧 `go2_body` 现采用官方 Go2W 仿真的 IMU 参考点作为名义默认值，
故 `base_link -> go2_body` 也使用同一 `T_base_imu`，不是单位平移。
这项模型约定与当前真机固件是否一致尚未确认；依据及可切换配置见
[GO2_STATE_FRAME.md](../../../fast_lio_nav2/GO2_STATE_FRAME.md)。
