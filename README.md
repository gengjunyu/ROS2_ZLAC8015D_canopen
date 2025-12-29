# ROS2 ZLAC8015D CANopen 驱动程序

这是一个用于控制 ZLAC8015D 电机控制器的 ROS2 驱动程序包，支持通过 CANopen 协议进行电机控制和编码器数据读取，并包含完整的键盘控制功能。

## 项目概述

本项目包含以下功能包：
- `can_msgs`: CAN 消息定义
- `overlord100_msgs`: 轮子数据消息定义  
- `zlac8015d`: 主要的驱动节点实现

## 主要功能

✅ **电机控制** - 支持差分驱动机器的精确速度控制  
✅ **编码器反馈** - 实时读取轮子速度和角度数据  
✅ **键盘遥控** - 标准键盘远程控制功能  
✅ **CANopen通信** - 基于工业标准的CANopen协议  
✅ **安全保护** - 速度限制和安全保护机制

## 硬件要求

- ZLAC8015D 电机控制器
- CAN 接口适配器
- 支持 ROS2 的系统（推荐 Ubuntu 20.04/22.04）

## 安装说明

### 1. 环境准备
```bash
# 确保 ROS2 环境已安装
source /opt/ros/humble/setup.bash  # 根据您的 ROS2 版本调整
```

### 2. 编译项目
```bash
cd ~/your_workspace
colcon build --packages-select can_msgs overlord100_msgs zlac8015d
source install/setup.bash
```

## 功能说明

### 电机控制节点 (motors_driver_node)

该节点负责接收速度控制命令并转发给电机控制器。

**订阅话题：**
- `/wheels_control` (overlord100_msgs/msg/WheelsData) - 左右轮速度控制命令

**发布话题：**
- `/CAN/can0/transmit` (can_msgs/msg/Frame) - CAN 发送消息

**功能特点：**
- 接收 float64 类型的左右轮速度值 (单位: rev/min)
- 自动将速度限制在 [-255, 255] rev/min 范围内
- 通过 CANopen 协议发送同步速度控制命令

### 键盘控制转换节点 (cmd_vel_to_wheels_node)

该节点将标准的 ROS2 键盘控制消息转换为电机控制消息，实现远程遥控功能。

**订阅话题：**
- `/cmd_vel` (geometry_msgs/msg/Twist) - 标准线速度和角速度控制命令

**发布话题：**
- `/wheels_control` (overlord100_msgs/msg/WheelsData) - 左右轮速度控制命令

**功能特点：**
- 差分驱动运动学转换
- 可配置轮距和速度限制
- 兼容标准 ROS2 键盘控制包

### 编码器节点 (encoders_node)

该节点负责读取编码器数据并发布轮子速度和角度信息。

**订阅话题：**
- `/CAN/can0/receive` (can_msgs/msg/Frame) - CAN 接收消息

**发布话题：**
- `/encoders_velocities` (overlord100_msgs/msg/WheelsData) - 轮子速度数据
- `/encoders_angles` (overlord100_msgs/msg/WheelsData) - 轮子角度数据

**功能特点：**
- 实时编码器数据采集
- 高精度轮速计算
- 角度累积跟踪

## 使用方法

### 1. 启动 CAN 接口
确保 CAN 接口已正确配置：
```bash
sudo ip link set can0 up type can bitrate 500000
```

### 2. 启动节点

#### 方法一：完整功能启动
```bash
# 终端1：启动电机驱动节点
ros2 run zlac8015d motors_driver_node

# 终端2：启动编码器节点  
ros2 run zlac8015d encoders_node

# 终端3：启动键盘控制转换节点
ros2 run zlac8015d cmd_vel_to_wheels_node

# 终端4：启动键盘控制（需要先安装 teleop-twist-keyboard）
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

#### 方法二：仅电机控制
```bash
# 启动电机驱动节点
ros2 run zlac8015d motors_driver_node

# 启动编码器节点  
ros2 run zlac8015d encoders_node
```

### 3. 控制电机

#### 键盘控制（推荐）
```bash
# 安装键盘控制包
sudo apt install ros-humble-teleop-twist-keyboard

# 启动转换节点
ros2 run zlac8015d cmd_vel_to_wheels_node

# 启动键盘控制
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**键盘控制说明：**
- `w/W`: 前进
- `x/X`: 后退  
- `a/A`: 左转
- `d/D`: 右转
- `s/S`: 停止

#### 手动命令控制
```bash
# 发送速度命令 (左右轮速度，单位: rev/min)
ros2 topic pub /wheels_control overlord100_msgs/msg/WheelsData '{left: 100.0, right: 100.0}'
```

### 4. 监控数据
```bash
# 监控轮子速度
ros2 topic echo /encoders_velocities

# 监控轮子角度
ros2 topic echo /encoders_angles

# 监控键盘控制命令
ros2 topic echo /cmd_vel

# 监控转换后的轮速命令
ros2 topic echo /wheels_control
```

## 消息定义

### WheelsData (overlord100_msgs/msg)
```yaml
float64 left   # 左轮数据
float64 right  # 右轮数据
```

### Frame (can_msgs/msg)
```yaml
std_msgs/Header header
uint32 id          # CAN ID
bool is_rtr        # 是否为远程传输请求
bool is_extended   # 是否为扩展帧
bool is_error      # 是否为错误帧
uint8 dlc          # 数据长度码
uint8[8] data      # 数据负载
```

## 配置参数

### 键盘控制转换节点参数
- `wheel_base`: 轮距（米），默认 0.3m
- `max_linear_speed`: 最大线速度（米/秒），默认 2.0 m/s
- `max_angular_speed`: 最大角速度（弧度/秒），默认 2.0 rad/s

### 电机驱动节点参数
- `NODE_ID`: CANopen 节点 ID (默认: 1)
- `CAN_OUT_TOPIC`: CAN 发送话题 (默认: "/CAN/can0/transmit")
- `CAN_IN_TOPIC`: CAN 接收话题 (默认: "/CAN/can0/receive")

### 速度限制
- 电机速度限制：-255 到 255 rev/min
- 轮速自动限制：基于配置的最大线速度

## 完整使用流程

### 快速开始
```bash
# 1. 配置 CAN 接口
sudo ip link set can0 up type can bitrate 500000

# 2. 编译项目
cd ~/ros2_ws
colcon build --packages-select can_msgs overlord100_msgs zlac8015d
source install/setup.bash

# 3. 启动所有节点
ros2 run zlac8015d motors_driver_node &
ros2 run zlac8015d encoders_node &
ros2 run zlac8015d cmd_vel_to_wheels_node &
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 使用启动文件
```bash
# 使用启动文件（如果配置了 launch 文件）
ros2 launch zlac8015d cmd_vel_to_wheels.launch.py
```

## 故障排除

### 1. CAN 通信问题
```bash
# 检查 CAN 接口状态
ip -details link show can0

# 查看 CAN 消息
candump can0
```

### 2. 键盘控制无响应
```bash
# 检查 cmd_vel 话题
ros2 topic echo /cmd_vel

# 检查转换节点
ros2 topic echo /wheels_control

# 确认 teleop_twist_keyboard 已安装
ros2 pkg list | grep teleop
```

### 3. 节点启动问题
- 确保话题名称正确且 CAN 桥接节点正在运行
- 检查依赖包是否正确安装

### 4. 电机无响应
- 检查 CAN 连接
- 验证电机控制器电源
- 确认节点 ID 配置正确
- 检查 wheels_control 话题是否正常发布

## 开发说明

### 项目架构
```
src/
├── can_msgs/                 # CAN 消息定义
│   └── msg/Frame.msg        # CAN 帧消息格式
├── overlord100_msgs/         # 轮子数据消息定义  
│   └── msg/WheelsData.msg   # 左右轮数据格式
├── zlac8015d/                # 主要功能包
│   ├── src/
│   │   ├── motors_driver_node.cpp        # 电机驱动节点
│   │   ├── encoders_node.cpp             # 编码器读取节点
│   │   └── cmd_vel_to_wheels_node.cpp    # 键盘控制转换节点
│   ├── launch/
│   │   └── cmd_vel_to_wheels.launch.py   # 键盘控制启动文件
│   ├── CMakeLists.txt                    # 编译配置
│   └── package.xml                        # 包配置
```

### 添加新功能
1. 在对应包中修改源代码
2. 更新消息定义（如需要）
3. 更新 CMakeLists.txt 和 package.xml
4. 重新编译：
```bash
colcon build --packages-select <package_name>
```

### 调试
启用调试信息：
```bash
# 电机驱动节点调试
ros2 run zlac8015d motors_driver_node --ros-args --log-level DEBUG

# 键盘控制节点调试
ros2 run zlac8015d cmd_vel_to_wheels_node --ros-args --log-level DEBUG

# 编码器节点调试
ros2 run zlac8015d encoders_node --ros-args --log-level DEBUG
```

## 许可证

TODO: 添加许可证信息

## 话题映射表

| 话题名称 | 消息类型 | 方向 | 功能 |
|----------|----------|------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | 订阅 | 键盘控制输入（线速度/角速度） |
| `/wheels_control` | `overlord100_msgs/WheelsData` | 订阅/发布 | 左右轮速度控制 |
| `/encoders_velocities` | `overlord100_msgs/WheelsData` | 发布 | 编码器速度反馈 |
| `/encoders_angles` | `overlord100_msgs/WheelsData` | 发布 | 编码器角度反馈 |
| `/CAN/can0/transmit` | `can_msgs/Frame` | 发布 | CAN 发送消息 |
| `/CAN/can0/receive` | `can_msgs/Frame` | 订阅 | CAN 接收消息 |

## 依赖要求

### ROS2 包依赖
- `rclcpp`: ROS2 C++ 客户端库
- `geometry_msgs`: 几何消息类型
- `can_msgs`: CAN 消息类型（项目自定义）
- `overlord100_msgs`: 轮子数据消息类型（项目自定义）

### 系统依赖
- `ros-humble-teleop-twist-keyboard`: 键盘控制包
- CAN 接口驱动和工具

## 版本信息

- ROS2 版本：Humble（推荐）
- C++ 标准：C++17
- CAN 协议：CANopen
- 消息类型：自定义 overlord100_msgs

12345