# Wheel_legged_Robot

双轮足式平衡机器人项目，包含 RK3576 上位机 ROS 2 工作空间和 STM32 控制板下位机固件。系统通过串口桥接上位机与下位机，可完成机器人状态发布、底盘速度控制、SLAM 建图和 Nav2 自主导航。

## 项目结构

```text
Wheel_legged_Robot/
├── fishbot_ws/              # RK3576 上位机 ROS 2 工作空间
│   └── src/
│       ├── fishbot_bringup/       # 机器人启动、串口桥、TF/URDF 启动
│       ├── fishbot_description/   # 基础机器人描述
│       ├── fishbot_navigation2/   # slam_toolbox 与 Nav2 配置、地图、launch
│       ├── lslidar_driver/        # 雷神雷达 ROS 2 驱动
│       ├── lslidar_msgs/          # 雷达消息定义
│       ├── ros_serial2wifi/       # 串口/WiFi 辅助通信节点
│       ├── wheel_legged_urdf_pkg/ # 双轮足机器人 URDF、网格和 RViz 配置
│       └── ydlidar_ros2/          # YDLidar ROS 2 驱动
├── wheel_legged_ros/        # STM32 控制板下位机工程
│   ├── Core/                # STM32CubeMX 生成的 HAL/FreeRTOS 基础代码
│   ├── Drivers/             # STM32 HAL、CMSIS 等驱动
│   ├── Middlewares/         # FreeRTOS 中间件
│   ├── User/                # 平衡控制、腿部/轮部控制、IMU、CAN、电机和 ROS 通信
│   └── MDK-ARM/             # Keil MDK 工程文件
├── solidworks/              # 机械结构、加工图纸和三维模型
│   ├── hhu外壳改版sw/       # 外壳改版零件及 STL/3MF 导出文件
│   ├── 机加工图纸及模型/    # DXF 图纸、STEP 模型和加工 BOM
│   └── 机械结构.zip         # 机械结构资料压缩包└── install.txt              # 依赖安装和设备规则记录
```

## 功能

- 双轮足式平衡机器人底盘控制
- STM32 下位机读取 IMU、轮端状态并执行平衡/运动控制
- 上下位机串口通信，默认设备名为 `/dev/serial_ttl`
- 上位机发布机器人 TF、URDF、里程计和雷达数据
- 支持 `slam_toolbox` 在线建图
- 支持 Navigation2 载入地图并进行自主导航
- 支持 RViz 可视化调试
- 提供机器人外壳、零件和机加工相关的 CAD 资料

## 硬件与软件环境

### 上位机

- RK3576
- Ubuntu 22.04
- ROS 2 Humble
- 雷达设备，默认可配置为 `/dev/lidar`
- 与 STM32 控制板连接的串口，默认可配置为 `/dev/serial_ttl`

### 下位机

- STM32H723 控制板
- FreeRTOS
- STM32 HAL
- Keil MDK-ARM / STM32CubeMX
- BMI088 IMU
- 达妙 DM4310 电机及 CAN 通信链路

## 机械设计资料

`solidworks/` 目录保存轮足机器人的机械结构资料，主要包括：

- `hhu外壳改版sw/`: 外壳改版零件的 SolidWorks `SLDPRT` 源文件，以及用于加工、打印或快速查看的 `STL`、`3MF` 文件
- `机加工图纸及模型/`: 机加工零件的 `DXF` 图纸、`STEP` 三维模型和 `BOM` 表格
- `机械结构.zip`: 机械结构资料的打包备份

SolidWorks 临时锁文件（文件名以 `~$` 开头）不会提交到仓库。修改或重新导出 CAD 文件后，请同步检查对应的 URDF 网格和装配尺寸，避免机械结构与仿真模型不一致。
## 上位机依赖安装

```bash
sudo apt update

sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-rviz2 \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-tf-transformations \
  ros-humble-pcl-conversions \
  ros-humble-diagnostic-updater \
  ros-humble-xacro \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-serial \
  libpcap-dev \
  libpcl-dev
```

如果 `rosdep` 尚未初始化：

```bash
sudo rosdep init
rosdep update
```

将当前用户加入串口权限组，执行后需要重新登录：

```bash
sudo usermod -a -G dialout $USER
```

## 设备规则

建议为 STM32 串口和雷达配置固定设备名，避免 `/dev/ttyACM*`、`/dev/ttyUSB*` 编号变化。

创建规则文件：

```bash
sudo nano /etc/udev/rules.d/99-wheel-legged-robot.rules
```

写入以下内容，并根据实际 `idVendor` / `idProduct` 调整：

```bash
# STM32 控制板串口 -> /dev/serial_ttl
SUBSYSTEM=="tty", ATTRS{idVendor}=="2e88", ATTRS{idProduct}=="4603", MODE="0666", SYMLINK+="serial_ttl"

# 雷达串口 -> /dev/lidar
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE="0666", SYMLINK+="lidar"
```

加载规则：

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

重新插拔设备后检查：

```bash
ls -l /dev/serial_ttl
ls -l /dev/lidar
```

## 编译上位机 ROS 2 工作空间

```bash
source /opt/ros/humble/setup.bash
cd ~/fishbot_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

如果项目目录不是 `~/fishbot_ws`，请替换为实际路径。

## 启动机器人

启动底层 bringup，包括 URDF/TF、串口桥和雷达驱动：

```bash
source /opt/ros/humble/setup.bash
source ~/fishbot_ws/install/setup.bash
ros2 launch fishbot_bringup bringup.launch.py
```

`fishbot_bringup` 默认串口参数：

- `serial_port`: `/dev/serial_ttl`
- `baudrate`: `115200`

## 建图

先启动机器人 bringup，再在另一个终端启动 SLAM：

```bash
source /opt/ros/humble/setup.bash
source ~/fishbot_ws/install/setup.bash
ros2 launch fishbot_navigation2 slam_toolbox.launch.py use_sim_time:=false
```

建图完成后可使用 `slam_toolbox` 或 Nav2 相关工具保存地图到 `fishbot_ws/src/fishbot_navigation2/maps/`。

## 导航

启动机器人 bringup 后，在另一个终端启动 Nav2：

```bash
source /opt/ros/humble/setup.bash
source ~/fishbot_ws/install/setup.bash
ros2 launch fishbot_navigation2 navigation2.launch.py \
  map:=$HOME/fishbot_ws/src/fishbot_navigation2/maps/room.yaml \
  use_sim_time:=false \
  params_file:=$HOME/fishbot_ws/src/fishbot_navigation2/config/nav2_params.yaml
```

启动后可在 RViz 中设置初始位姿，并通过 `Nav2 Goal` 下发导航目标点。

## 下位机固件

`wheel_legged_ros/` 为 STM32 控制板代码，主要包含：

- `User/Controller`: 平衡与底盘控制逻辑
- `User/APP`: 左右腿/轮任务、状态观测、IMU、ROS 通信任务
- `User/Devices`: BMI088、DM4310 等设备驱动
- `User/Algorithm`: PID、Kalman、EKF、Mahony、VMC 等算法
- `User/Bsp`: CAN、PWM、DWT 等板级支持

编译和烧录步骤：

1. 使用 Keil MDK-ARM 打开 `wheel_legged_ros/MDK-ARM/CtrlBoard-H7_IMU.uvprojx`
2. 检查 STM32H723 目标芯片、调试器和下载配置
3. 编译工程并通过 ST-Link/J-Link 下载到控制板
4. 确认控制板串口连接到 RK3576，并与上位机串口桥保持 115200 波特率

## 上下位机通信

上位机通过 `fishbot_bringup/src/serial_bridge_node.py` 与下位机串口通信。下位机 `User/APP/ros_common.c` 中实现速度指令解析和里程计/IMU 数据发送。

- 上位机向下位机下发线速度 `vx` 与角速度 `wz`
- 下位机回传速度、角速度、位姿、IMU 和姿态数据
- 数据帧使用 `0xAA 0x55` 作为帧头，并带校验字段

## 注意事项

- 默认 bringup 使用 `lslidar_driver` 中的 `lsn10_launch.py`，请根据实际雷达型号调整 launch 或参数文件
- 如果串口设备名不是 `/dev/serial_ttl`，请修改 `fishbot_bringup/launch/bringup.launch.py`
- 导航前需要确认 TF、里程计、雷达 `/scan` 数据正常
- 真实机器人调试前请先确认急停、电池、电机方向和限位保护
- CAD 资料包含二进制和较大的 DXF 文件，首次克隆或拉取更新可能需要较长时间
- 当前仓库保存的是零件、导出模型和加工资料；如需完整 SolidWorks 装配体，请同时保留对应的 `SLDASM` 和引用零件文件

## 许可证

本项目当前未声明开源许可证。如需公开发布和二次分发，建议补充合适的 LICENSE 文件。
