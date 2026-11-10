# 第三方源码来源

本工作空间包含修改后的第三方源码。2026-11-10 整理时，将嵌套 Git 元数据和子模块声明移到工作空间外归档；代码按普通目录纳入根仓库，便于一次克隆取得完整源码。

下列提交为整理前各独立仓库的 HEAD，不代表当前目录与上游完全相同。FAST-LIO 包含本项目修改；原始 `git status` 和 Git 历史保存在本机归档中。

| 目录 | 上游 | 整理前 HEAD |
| --- | --- | --- |
| `src/FAST_LIO_ROS2` | [Ericsii/FAST_LIO_ROS2](https://github.com/Ericsii/FAST_LIO_ROS2) | `2fffc570a25d0df172720bac034fbdb6a13d2162` |
| `src/FAST_LIO_ROS2/include/ikd-Tree` | [hku-mars/ikd-Tree](https://github.com/hku-mars/ikd-Tree) | `e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4` |
| `src/driver/HesaiLidar_ROS_2.0` | [HesaiTechnology/HesaiLidar_ROS_2.0](https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0) | `e7e112f0809f0eed5e3c81c55a1a0376474db234` |
| `src/driver/HesaiLidar_ROS_2.0/src/driver/HesaiLidar_SDK_2.0` | [HesaiTechnology/HesaiLidar_SDK_2.0](https://github.com/HesaiTechnology/HesaiLidar_SDK_2.0) | `9d5dc4fc4ade5be5f6a6ca00e71dd4050b054168` |
| `src/driver/unitree_go`、`src/driver/unitree_api` | [unitreerobotics/unitree_ros2](https://github.com/unitreerobotics/unitree_ros2) | 来源说明见各包 README；未保存独立 Git HEAD |

原有 `LICENSE`、代码版权声明及禾赛标定文件许可均保留。ikd-Tree 当前副本的来源说明见其 README。FAST-LIO 的 `LICENSE` 文件与 `package.xml` 许可证字段存在上游遗留差异，本次没有自行改写许可证或替项目指定新许可证。

FAST-LIO 的两个大型演示 GIF 和论文 PDF 已移出本工作空间；上游 README 中对应展示链接改为上游固定版本文件。其余文档图片和源码保持。
