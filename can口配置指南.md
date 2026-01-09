# 加载SocketCAN内核模块
sudo modprobe can
sudo modprobe can_raw
sudo modprobe vcan  # 虚拟CAN接口（无硬件时用）

# 方式1：物理CAN接口（如can0，波特率500000）
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0

# 方式2：虚拟CAN接口（无硬件测试用）
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0

# 验证接口是否正常
ip link show can0  # 或 vcan0


二、快速使用官方 ros2socketcan_bridge 包
ROS2 已提供成熟的socketcan_bridge包，无需从零开发，直接安装使用即可。
1. 安装依赖包
bash
运行
# 替换${ROS_DISTRO}为你的ROS2版本（如humble/iron）
sudo apt update
sudo apt install ros-${ROS_DISTRO}-socketcan-bridge ros-${ROS_DISTRO}-socketcan-interface ros-${ROS_DISTRO}-can-msgs
2. 启动桥接节点
启动双向桥接（ROS2 ↔ CAN），指定 CAN 接口和话题名：
bash
运行
# 物理接口can0（替换为vcan0则用虚拟接口）
ros2 run socketcan_bridge socketcan_bridge_node --ros-args \
  -p can_interface:=can0 \
  -p tx_topic:=/can_tx  # ROS2→CAN的发送话题
  -p rx_topic:=/can_rx  # CAN→ROS2的接收话题
3. 测试桥接功能
ROS2→CAN：发布 ROS2 CAN 帧消息到 CAN 总线
bash
运行
ros2 topic pub /can_tx can_msgs/msg/Frame "{id: 0x123, dlc: 2, data: [0x01, 0x02], is_extended: false}" --once

CAN→ROS2：订阅 CAN 总线数据（ROS2 端接收）
bash
运行
ros2 topic echo /can_rx
