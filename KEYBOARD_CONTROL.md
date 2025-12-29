# 键盘控制使用说明

## 概述

本项目现在支持通过键盘控制机器人移动。使用 `cmd_vel_to_wheels_node` 将标准的 ROS2 键盘控制消息转换为电机控制消息。

## 安装依赖

```bash
# 安装键盘控制包
sudo apt install ros-humble-teleop-twist-keyboard
```

## 编译项目

```bash
cd ~/ros2_ws
colcon build --packages-select zlac8015d
source install/setup.bash
```

## 使用方法

### 方法一：直接启动

1. **启动转换节点：**
```bash
ros2 run zlac8015d cmd_vel_to_wheels_node
```

2. **在新终端中启动键盘控制：**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 方法二：使用启动文件

```bash
ros2 launch zlac8015d cmd_vel_to_wheels.launch.py
```

然后在新终端启动键盘控制：
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 键盘控制说明

使用 `teleop_twist_keyboard` 的默认控制按键：

| 按键 | 功能 | 说明 |
|------|------|------|
| w/W  | 前进 | 线速度增加 |
| x/X  | 后退 | 线速度减少 |
| a/A  | 左转 | 角速度增加 |
| d/D  | 右转 | 角速度减少 |
| s/S  | 停止 | 速度归零 |
| q/Q  | 增加线速度步长 | 控制精度 |
| z/Z  | 减少线速度步长 | 控制精度 |

## 参数配置

转换节点支持以下参数配置：

- `wheel_base`: 轮距（米），默认 0.3m
- `max_linear_speed`: 最大线速度（米/秒），默认 2.0 m/s
- `max_angular_speed`: 最大角速度（弧度/秒），默认 2.0 rad/s

可以通过启动文件或命令行参数修改：

```bash
ros2 run zlac8015d cmd_vel_to_wheels_node --ros-args -p wheel_base:=0.4 -p max_linear_speed:=1.5
```

## 运动学原理

使用差分驱动运动学模型：

```
v_left = v_x - (omega * wheel_base / 2)
v_right = v_x + (omega * wheel_base / 2)
```

其中：
- `v_left`: 左轮速度
- `v_right`: 右轮速度  
- `v_x`: 线速度（前进为正）
- `omega`: 角速度（逆时针为正）
- `wheel_base`: 轮距

## 故障排除

1. **机器人不动：**
   - 检查电机驱动节点是否运行
   - 检查 CAN 连接
   - 确认话题是否正确发布：`ros2 topic echo /wheels_control`

2. **键盘输入无响应：**
   - 确认 `teleop_twist_keyboard` 是否正常运行
   - 检查 `cmd_vel` 话题：`ros2 topic echo /cmd_vel`

3. **转换节点报错：**
   - 检查依赖是否正确安装
   - 确认编译是否成功

## 话题映射

| 输入话题 | 消息类型 | 功能 |
|----------|----------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | 线速度和角速度 |

| 输出话题 | 消息类型 | 功能 |
|----------|----------|------|
| `/wheels_control` | `overlord100_msgs/WheelsData` | 左右轮速度控制 |

## 示例使用流程

```bash
# 终端1：启动转换节点
ros2 run zlac8015d cmd_vel_to_wheels_node

# 终端2：启动键盘控制
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 终端3：监听轮速（可选）
ros2 topic echo /wheels_control
```

现在您就可以使用键盘控制机器人了！