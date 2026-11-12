# 智能小车 ROS 2 工程（2026-11-12 备份）

阿克曼转向小车的赛道（S 弯）规划与控制工程。包含车载实测源码、本地改进候选版、210 次实跑记录和配套文档。

原车环境：Ubuntu 22.04 LTS · ROS 2 Humble · ARM64 · Python 3.10。

## 目录结构

| 路径 | 内容 |
| --- | --- |
| `源码/车载实测版/` | 最后一次实跑对应的 S 弯控制程序与参数 |
| `源码/本地候选版/` | 实跑之后通过本地回放完成的修正版，建议在此继续开发 |
| `源码/ROS工作区/src/` | 车载 ROS 2 工作区源码，16 个目录、13 个含 `package.xml` 的包 |
| `源码/run_user_course.py` | 车上有界弯道测试的启动脚本（路径按原车 `/root/racecar/tools/s_bend` 固定） |
| `源码/export_rosbags.py` | 原始日志导出 bag 的脚本 |
| `源码/manifest.json` | 实测版代码与候选修改文件的 SHA-256 |
| `rosbags/` | 210 次实跑的 rosbag2（SQLite3 + CDR），每次运行一个目录 |
| `bag_index.csv` | 按运行检索的索引 |
| `manifest.json` | 原日志路径、SHA-256、时长及各话题的消息数 |
| `offline_replay_kit.zip` | 离线回放工具包 |
| `使用说明.md` | bag 的话题、时间轴与数据来源说明 |
| `Ubuntu开发交接.md` | 在 Ubuntu 上继续开发的完整步骤 |

## 快速查看

播放最新一次实跑（需要 ROS 2 Humble）：

```bash
ros2 bag info ./rosbags/20261112_212001_drive_op9zcbzp
ros2 bag play ./rosbags/20261112_212001_drive_op9zcbzp --clock
```

话题清单和数据来源见 [`使用说明.md`](使用说明.md)；离线回放、ROS 工作区构建、底盘驱动限制见 [`Ubuntu开发交接.md`](Ubuntu开发交接.md)。

## 注意事项

- **数据来源**：bag 中的话题由原始日志恢复，不是车辆原始的 IMU / 编码器消息流，因此不能直接替代真车节点的 `/IMU_data`、`/encoder` 输入。
- **底盘驱动**：`racecar_driver` 只提供 ARM64 厂商静态库 `lib/libracecar_driver.a`，没有实现源码；x86-64 环境构建时需 `--packages-skip racecar_driver`。
- **仓库体积**：约 311 MB，其中 `rosbags/` 约 242 MB。克隆较慢，只关心代码时可用
  `git clone --depth 1 --filter=blob:none <url>`，或 sparse-checkout 只取 `源码/`。
- **第三方代码**：`源码/ROS工作区/src/` 下多数包来自上游项目（lslidar、slam_gmapping、openslam_gmapping、robot_localization、navigation2、teb_local_planner、rf2o 等），各自保留原许可证；`src/` 根目录的 `.rar/.zip/.tar.gz` 是这些上游包的原始压缩档。
- **未验证项**：完整 ROS 工作区尚未在新的 Ubuntu 环境编译验证；候选版控制逻辑的实车表现仍需在原设备上确认。
- **历史参数快照**：`源码/车载实测版/dense_settings.json.bak*` 是调参过程中的参数备份，保留用于对照。

## 许可证

自研部分（`源码/车载实测版/`、`源码/本地候选版/`）版权归作者所有，未声明开源许可证。第三方 ROS 包遵循各自仓库的许可证。
