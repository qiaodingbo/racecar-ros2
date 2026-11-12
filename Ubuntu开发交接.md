# 小车项目 Ubuntu 开发交接

原车环境为 Ubuntu 22.04 LTS、ROS 2 Humble、ARM64、Python 3.10。Humble 的 Ubuntu 22.04 软件包同时提供 amd64 和 arm64 版本。规划算法、日志回放和模拟可以在普通 x86-64 Ubuntu 电脑上继续开发。 [ROS 2 Humble 安装说明](https://github.com/ros2/ros2_documentation/blob/humble/source/Installation/Ubuntu-Install-Debs.rst)

- `offline/vehicle/` 是最后一次实测对应的控制代码和配置。
- `offline/candidate/` 是实测后完成两处修正的候选版，适合在此基础上继续开发。重点文件是 `dense_s_bend.py`、`planners/dense/progressive.py` 和 `core/geometry.py`。
- `offline/logs/` 有六份代表性原始日志，`offline/scripts/` 有日志分析、路线回放和控制模拟。
- `rosbags/` 有 210 次运行的 ROS 2 bag。`bag_index.csv` 可检索每次运行，`BAGS.md` 解释话题和时间轴。
- `ros_ws/src/` 是从车载归档提取的 ROS 工作区，有 15 个 ROS 包。归档时跳过了源码目录中残留的旧编译产物。

## 路线回放与模拟

在解压目录执行以下命令。只需 Python 3 和 NumPy。脚本读取原始记录并模拟控制，不需要 ROS 环境或小车。

```bash
sudo apt install python3-numpy
log=offline/logs/20261112_212001_drive_op9zcbzp.jsonl
python3 offline/scripts/replay_user_course.py offline/vehicle "$log"
python3 offline/scripts/replay_user_course.py offline/candidate "$log"
python3 offline/scripts/verify_stop_and_gap.py offline/candidate "$log"
```

实测版在该日志中复现了三帧 `handoff_discontinuity`，候选版回放通过了这三帧。最后一条命令会输出单帧丢线、持续丢线和障碍停车的模拟结果，并在当前目录写入 `obstacle_stop_simulation.json`。回放使用记录时的雷达与位姿，停车模拟采用简化滑行模型。

## ROS 2 bag 播放

安装 ROS 2 Humble 后，在解压目录运行。使用 ROS 基础版时可按需安装 `ros-humble-rosbag2` 和 `ros-humble-rosbag2-storage-default-plugins`。

```bash
source /opt/ros/humble/setup.bash
ros2 bag info ./rosbags/20261112_212001_drive_op9zcbzp
ros2 bag play ./rosbags/20261112_212001_drive_op9zcbzp --clock
```

bag 保留雷达、已记录的融合位姿、控制和规划数据。原始 IMU 和编码器独立话题没有记录，运行原车控制节点时依赖的 `/IMU_data` 与 `/encoder` 因此无法直接用这些 bag 代替。开发规划时用上面的离线回放脚本。

## 构建 ROS 工作区

此步骤用于开发 ROS 节点。先按官方文档安装 Humble 和 `ros-dev-tools`。首次使用 rosdep 时运行 `sudo rosdep init`，随后从解压目录执行。

```bash
sudo apt update
sudo apt install python3-serial python3-numpy libboost-thread-dev libeigen3-dev libopencv-dev libpcap-dev
source /opt/ros/humble/setup.bash
rosdep update --rosdistro humble
cd ros_ws
rosdep install --from-paths src --ignore-src --rosdistro humble \
  --skip-keys 'pyserial cmake_modules' -y
colcon build --symlink-install --packages-skip racecar_driver --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

`pyserial` 是原清单中的名称，上面已通过 `python3-serial` 安装。`rf2o_laser_odometry` 的清单保留了 `cmake_modules`，其 CMake 实际用的是 `eigen3_cmake_module`。完整 ROS 工作区尚未在新的 Ubuntu 环境编译验证。 [rosdep 文档](https://github.com/ros-infrastructure/rosdep/blob/master/doc/overview.rst)

## 底盘驱动边界

`racecar_driver` 有 ROS 节点源码和接口头文件，链接的 `lib/libracecar_driver.a` 是 ARM64 厂商静态库，归档里没有库的实现源码。x86-64 无法链接这份库，所以构建命令跳过这个包。若要在 x86 运行真车底盘驱动，需要厂商提供对应架构的库或实现源码。ARM64 真车部署还依赖匹配的雷达、串口设备、驱动和配置。 [Linux ELF 架构编号](https://github.com/torvalds/linux/blob/master/include/uapi/linux/elf-em.h)

## 核对交付

210 个 bag 已逐个解码读回，最新一次的 129 帧雷达和 129 条位姿已核对。`offline/manifest.json` 保存实测版文件与日志的 SHA-256。实测版和候选版已从分享包重新解压并运行离线回放。底盘驱动和候选控制逻辑的实车表现仍需在原设备上验证。
