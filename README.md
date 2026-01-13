# ZLAC8015D ROS2 控制节点

[![ROS2](https://img.shields.io/badge/ROS2-Humble%20%7C%20Iron%20%7C%20Rolling-blue)](https://www.ros.org/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![Python](https://img.shields.io/badge/Python-3.8%2B-blue)](https://www.python.org/)

ROS2控制节点，用于控制ZLAC8015D CANopen伺服轮毂电机驱动器。支持差速驱动、位置控制、速度控制和转矩控制等多种模式。

## ✨ 功能特性

- 🎮 **多种控制模式**：支持位置模式、速度模式、转矩模式三种控制模式
- 🚗 **差速驱动**：标准`cmd_vel`接口，兼容ROS导航栈
- 📊 **状态监控**：实时发布关节状态、驱动器状态和故障码
- 🔧 **CANopen通信**：基于`python-can`的真实CANopen接口实现
- 🧪 **模拟模式**：支持模拟接口，无需硬件即可测试
- ⚙️ **灵活配置**：支持电机方向反转、机械参数配置等
- 🛡️ **故障处理**：自动故障检测和清除功能

## 📋 系统要求

- **ROS2**: Humble / Iron / Rolling
- **Python**: 3.8+
- **CAN总线接口**: SocketCAN（Linux）或使用模拟模式
- **依赖库**: `python-can`

## 🚀 快速开始

### 1. 安装依赖

```bash
# 安装python-can库
pip3 install python-can

# 或者使用apt安装（Ubuntu/Debian）
sudo apt-get install python3-can
```

### 2. 创建工作空间并克隆代码

```bash
mkdir -p ~/canopen_ws/src
cd ~/canopen_ws/src
# 假设你已经克隆了代码库
# git clone <your-repo-url> zlac8015d_control
```

### 3. 编译工作空间

```bash
cd ~/canopen_ws
colcon build --packages-select zlac8015d_control
source install/setup.bash
```

### 4. 配置CAN接口（如果使用真实硬件）

```bash
# 设置CAN接口（波特率500kHz）
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0

# 检查CAN接口状态
ip link show can0

# 如果遇到权限问题，将用户添加到dialout组
sudo usermod -a -G dialout $USER
# 然后重新登录
```

## 📖 使用方法

### 启动节点

```bash
# 使用默认配置
ros2 launch zlac8015d_control zlac8015d_control.launch.py

# 指定节点ID和CAN接口
ros2 launch zlac8015d_control zlac8015d_control.launch.py \
    node_id:=1 \
    can_interface:=can0

# 使用模拟模式（测试用，无需硬件）
ros2 launch zlac8015d_control zlac8015d_control.launch.py use_mock:=true
```

### 初始化驱动器

**重要**：使用前必须先初始化驱动器！

```bash
# 1. 初始化驱动器
ros2 service call /initialize std_srvs/srv/Trigger

# 2. 使能驱动器
ros2 service call /enable std_srvs/srv/SetBool "{data: true}"
```

### 控制电机

#### 1. 速度控制（差速驱动）

```bash
# 前进（线速度 0.5 m/s）
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# 旋转（角速度 0.5 rad/s）
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"

# 停止（发送零速度）
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

#### 2. 位置控制

```bash
# 发布位置命令（弧度）
ros2 topic pub /joint_position_command sensor_msgs/msg/JointState \
    "{name: ['left_wheel_joint', 'right_wheel_joint'], position: [1.0, 1.0]}"
```

#### 3. 速度控制（直接关节速度）

```bash
# 发布速度命令（rad/s）
ros2 topic pub /joint_velocity_command sensor_msgs/msg/JointState \
    "{name: ['left_wheel_joint', 'right_wheel_joint'], velocity: [1.0, 1.0]}"
```

### 查看状态

```bash
# 查看关节状态（位置、速度）
ros2 topic echo /joint_states

# 查看驱动器状态码
ros2 topic echo /driver_status

# 查看故障码
ros2 topic echo /fault_code
```

### 停止和故障处理

```bash
# 停止电机
ros2 service call /stop std_srvs/srv/Trigger

# 清除故障
ros2 service call /clear_fault std_srvs/srv/Trigger

# 禁用驱动器
ros2 service call /enable std_srvs/srv/SetBool "{data: false}"
```

## 🔌 ROS2接口

### 订阅的话题

| 话题名称 | 消息类型 | 说明 |
|---------|---------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | 速度命令（差速驱动，线速度m/s，角速度rad/s） |
| `/joint_position_command` | `sensor_msgs/JointState` | 位置命令（弧度） |
| `/joint_velocity_command` | `sensor_msgs/JointState` | 速度命令（直接关节速度，rad/s） |

### 发布的话题

| 话题名称 | 消息类型 | 说明 |
|---------|---------|------|
| `/joint_states` | `sensor_msgs/JointState` | 关节状态（位置、速度） |
| `/driver_status` | `std_msgs/Int32` | 驱动器状态码 |
| `/fault_code` | `std_msgs/Int32` | 故障码 |

### 服务

| 服务名称 | 服务类型 | 说明 |
|---------|---------|------|
| `/initialize` | `std_srvs/Trigger` | 初始化驱动器 |
| `/stop` | `std_srvs/Trigger` | 停止电机 |
| `/clear_fault` | `std_srvs/Trigger` | 清除故障 |
| `/enable` | `std_srvs/SetBool` | 使能/禁用驱动器 |

## ⚙️ 参数配置

编辑 `config/default.yaml` 文件以配置节点参数：

```yaml
zlac8015d_control_node:
  ros__parameters:
    # CANopen节点配置
    node_id: 1                    # 驱动器节点ID (1-127)
    can_interface: "can0"         # CAN接口名称
    
    # 调试选项
    use_mock: false               # 是否使用模拟接口（测试用）
    
    # 发布频率
    publish_rate: 50.0            # 状态发布频率 (Hz)
    
    # 机械参数
    encoder_resolution: 1024      # 编码器分辨率 (counts/rev)
    wheel_radius: 0.1            # 轮子半径 (m)
    wheel_base: 0.5              # 轮距 (m)
    
    # 电机方向配置（如果轮子方向相反，设置其中一个为true）
    reverse_left_motor: false     # 是否反转左电机方向
    reverse_right_motor: false    # 是否反转右电机方向
```

### 参数说明

- **node_id**: CANopen节点ID，必须与驱动器配置的节点ID匹配
- **can_interface**: CAN接口名称（如`can0`、`can1`）
- **use_mock**: 是否使用模拟接口，`true`表示使用模拟模式（无需硬件）
- **publish_rate**: 状态发布频率，建议50-100Hz
- **encoder_resolution**: 编码器分辨率，根据实际硬件配置
- **wheel_radius**: 轮子半径（米），用于速度转换
- **wheel_base**: 轮距（米），用于差速驱动计算
- **reverse_left_motor/reverse_right_motor**: 如果左右轮方向相反，设置其中一个为`true`

## 📊 驱动器状态码

| 状态码 | 状态名称 | 说明 |
|-------|---------|------|
| 0 | NOT_READY_TO_SWITCH_ON | 未准备好切换 |
| 1 | SWITCH_ON_DISABLED | 切换禁用 |
| 2 | READY_TO_SWITCH_ON | 准备切换 |
| 3 | SWITCHED_ON | 已切换 |
| 4 | OPERATION_ENABLED | 运行使能（正常工作状态） |
| 5 | QUICK_STOP_ACTIVE | 急停激活 |
| 6 | FAULT_REACTION_ACTIVE | 故障反应激活 |
| 7 | FAULT | 故障 |
| 99 | UNKNOWN | 未知状态 |

## 🔍 故障码说明

参考ZLAC8015D协议文档中的故障码定义（对象字典0x603F）：

| 故障码 | 说明 |
|-------|------|
| 0x00000001 | 过压 |
| 0x00000002 | 欠压 |
| 0x00000004 | 过流 |
| 0x00000008 | 过载 |
| 0x00000020 | 编码器超差 |
| ... | 详见协议文档 |

## 🛠️ 故障排除

### 问题1: 初始化失败

**症状**: 调用`/initialize`服务返回失败

**解决方案**:
1. 检查CAN接口是否已配置并启动：
   ```bash
   ip link show can0
   ```
2. 检查CAN总线波特率是否为500kHz
3. 检查节点ID是否与驱动器匹配
4. 查看节点日志获取详细错误信息

### 问题2: 电机没有反应

**症状**: 初始化成功，但发送速度命令后电机不动

**解决方案**:
1. 确认已调用`/enable`服务使能驱动器
2. 检查CAN总线连接是否正常
3. 使用`candump can0`查看CAN消息是否正常发送
4. 检查驱动器状态码是否为4（OPERATION_ENABLED）

### 问题3: 两个轮子方向相反

**症状**: 前进时两个轮子转向相反

**解决方案**:
在`config/default.yaml`中设置电机方向反转：
```yaml
reverse_left_motor: true   # 或 reverse_right_motor: true
```

### 问题4: 权限错误

**症状**: `PermissionError`或无法访问CAN接口

**解决方案**:
```bash
sudo usermod -a -G dialout $USER
# 重新登录后生效
```

### 问题5: 找不到python-can库

**症状**: `ImportError: No module named 'can'`

**解决方案**:
```bash
pip3 install python-can
# 或
sudo apt-get install python3-can
```

## 🏗️ 架构说明

本包包含以下主要组件：

- **zlac8015d_control_node.py**: ROS2节点主程序
- **zlac8015d_driver.py**: 驱动器控制逻辑，实现CANopen状态机
- **canopen_interface.py**: CANopen通信接口
  - `SocketCANopenInterface`: 真实硬件接口（基于python-can）
  - `MockCANopenInterface`: 模拟接口（用于测试）

## 📝 开发说明

### 代码结构

```
zlac8015d_control/
├── config/
│   └── default.yaml          # 默认配置文件
├── launch/
│   └── zlac8015d_control.launch.py  # 启动文件
├── zlac8015d_control/
│   ├── __init__.py
│   ├── zlac8015d_control_node.py    # ROS2节点
│   ├── zlac8015d_driver.py           # 驱动器控制
│   └── canopen_interface.py          # CANopen接口
├── package.xml
├── setup.py
└── README.md
```

### 扩展CANopen接口

如果需要使用其他CAN库（如`canopen`库），可以继承`CANopenInterface`类：

```python
from zlac8015d_control.canopen_interface import CANopenInterface

class CustomCANopenInterface(CANopenInterface):
    def connect(self) -> bool:
        # 实现连接逻辑
        pass
    
    def sdo_write(self, index, subindex, data, data_type="auto") -> bool:
        # 实现SDO写入
        pass
    
    # 实现其他必需方法...
```

## ⚠️ 注意事项

1. **CAN总线配置**: 确保CAN总线波特率设置为500kHz，与驱动器匹配
2. **节点ID匹配**: 配置的节点ID必须与驱动器的实际节点ID一致
3. **初始化顺序**: 使用前必须先调用`/initialize`服务，然后调用`/enable`服务
4. **模式切换**: 切换控制模式时，驱动器会自动切换，无需手动设置
5. **模拟模式**: 生产环境中应禁用模拟模式（`use_mock=false`）
6. **参数配置**: 编码器分辨率和机械参数需要根据实际硬件配置
7. **安全**: 首次使用时建议在安全环境下测试，确保急停功能正常

## 📄 许可证

MIT License

## 🙏 致谢

- ZLAC8015D驱动器制造商
- CANopen协议标准 (CiA 301, CiA 402)
- python-can库开发者

## 📚 参考文档

- [ZLAC8015D V4.0 CANopen通信例程文档](https://www.zlac.com/)
- [CANopen协议标准](https://www.can-cia.org/can-knowledge/canopen/)
- [ROS2官方文档](https://docs.ros.org/)

## 🤝 贡献

欢迎提交Issue和Pull Request！

---

**注意**: 本软件仅供学习和研究使用。在生产环境中使用前，请充分测试并确保安全。