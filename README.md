# Xiaoxdummy

基于稚晖君 [Dummy Robot](https://github.com/peng-zhihui/Dummy-Robot) 的开源六轴机械臂项目，包含 STM32 固件以及完整的 ROS2 控制栈，支持 Gazebo 仿真和真实硬件控制。

## 项目结构

```
xiaoxdummy/
├── firmware/                         # STM32 电机控制板固件
│   ├── dummy-35motor-fw/             # 35 步进电机固件 (STM32F103)
│   ├── dummy-42motor-fw/             # 42 步进电机固件 (STM32F103)
│   └── dummy-ref-core-fw/            # 参考核心固件 (STM32F405)
└── ros2/
    └── xiaoxdummy_ws/                # ROS2 工作空间
        └── src/
            ├── xiaoxdummy_control/   # Launch、控制器配置、键盘控制
            ├── xiaoxdummy_hw/        # ros2_control 硬件接口插件
            ├── xiaoxdummy_firmware/  # 串口固件桥接节点
            └── xiaoxdummy_interface/ # ROS2 服务接口定义
```

## 环境要求

- **操作系统**：Ubuntu 22.04+
- **ROS2**：Humble 或更高版本
- **ROS2 依赖包**：
  - `ros2_control`、`ros2_controllers`
  - `controller_manager`
  - `joint_trajectory_controller`、`joint_state_broadcaster`
  - `ros_gz_sim`、`ros_gz_bridge`、`gz_ros2_control`（仿真需要）
  - `xacro`、`robot_state_publisher`、`rviz2`
- **固件编译**（仅编译固件时需要）：
  - `arm-none-eabi-gcc` 工具链
  - CMake >= 3.19

## 安装

### 1. 克隆项目

```bash
git clone <本项目地址>
cd xiaoxdummy
```

### 2. 安装 ROS2 依赖

```bash
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-controller-manager ros-humble-xacro \
  ros-humble-robot-state-publisher ros-humble-rviz2 \
  ros-humble-ros-gz-sim ros-humble-ros-gz-bridge ros-humble-gz-ros2-control
```

### 3. 编译 ROS2 工作空间

```bash
cd ros2/xiaoxdummy_ws
colcon build --symlink-install
source install/setup.bash
```

### 4. 串口权限（真实硬件）

查看当前串口设备：

```bash
ls /dev/ttyACM*
```

赋予串口读写权限：

```bash
sudo chmod 666 /dev/ttyACM0
```

## 使用方法

### Gazebo 仿真

无需连接硬件，直接启动仿真环境：

```bash
source ros2/xiaoxdummy_ws/install/setup.bash
ros2 launch xiaoxdummy_control xiaoxdummy_control_launch.py use_gazebo:=true
```

### 真实硬件控制

将机械臂控制板通过串口连接到电脑（默认 `/dev/ttyACM0`），然后启动：

```bash
source ros2/xiaoxdummy_ws/install/setup.bash
ros2 launch xiaoxdummy_control xiaoxdummy_control_launch.py \
  use_real_hardware:=true \
  serial_port:=/dev/ttyACM0 \
  baud_rate:=115200 \
  command_speed:=180.0
```

### 键盘控制

在启动 launch 时加上 `control_file` 参数，会自动打开一个终端窗口用于键盘控制：

```bash
ros2 launch xiaoxdummy_control xiaoxdummy_control_launch.py \
  use_real_hardware:=true \
  control_file:=xiaoxdummy_keyboard.py
```

也可以单独启动键盘控制节点：

```bash
ros2 run xiaoxdummy_control xiaoxdummy_keyboard.py
```

键盘按键映射：

| 按键 | 功能 |
|------|------|
| `q` / `a` | Joint1 正转 / 反转 |
| `w` / `s` | Joint2 正转 / 反转 |
| `e` / `d` | Joint3 正转 / 反转 |
| `r` / `f` | Joint4 正转 / 反转 |
| `t` / `g` | Joint5 正转 / 反转 |
| `y` / `h` | Joint6 正转 / 反转 |
| `v` | 使能机械臂 |
| `b` | 关闭使能 |
| `z` | 回到 Home 位（全零） |
| `c` | 回到启动时的收缩位 |
| `p` | 查询硬件状态 |
| `x` | 退出 |

## Launch 参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `use_gazebo` | `false` | 是否使用 Gazebo 仿真 |
| `use_real_hardware` | `false` | 是否连接真实硬件 |
| `serial_port` | `/dev/ttyACM0` | 串口设备路径 |
| `baud_rate` | `115200` | 串口波特率 |
| `command_speed` | `180.0` | 关节运动速度 (deg/s) |
| `control_file` | （空） | 控制脚本，可选 `xiaoxdummy_keyboard.py` |

## 固件编译

如需重新编译 STM32 固件，需安装 `arm-none-eabi-gcc` 工具链：

```bash
sudo apt install gcc-arm-none-eabi
```

### 42 步进电机 / 35 步进电机固件

```bash
cd firmware/dummy-42motor-fw   # 或 dummy-35motor-fw
mkdir build && cd build
cmake ..
make
```

编译产物：`Ctrl-Step-STM32-fw.elf`、`.hex`、`.bin`

### 核心控制板固件

```bash
cd firmware/dummy-ref-core-fw
mkdir build && cd build
cmake ..
make
```

编译产物：`Core-STM32F4-fw.elf`、`.hex`、`.bin`

## 参考链接

- [Dummy Robot（稚晖君）](https://github.com/peng-zhihui/Dummy-Robot)
