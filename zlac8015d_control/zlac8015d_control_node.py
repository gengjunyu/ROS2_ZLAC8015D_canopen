"""
ZLAC8015D ROS2控制节点
提供位置、速度、转矩控制接口
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32, Bool, Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, SetBool
import time
from typing import Optional

from .zlac8015d_driver import ZLAC8015DDriver, DriverState, OperationMode
from .canopen_interface import CANopenInterface, MockCANopenInterface, SocketCANopenInterface


class ZLAC8015DControlNode(Node):
    """ZLAC8015D ROS2控制节点"""
    
    def __init__(self):
        super().__init__('zlac8015d_control_node')
        
        # 声明参数
        self.declare_parameter('node_id', 1)
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('use_mock', False)  # 是否使用模拟接口（用于测试）
        self.declare_parameter('publish_rate', 50.0)  # 发布频率 (Hz)
        self.declare_parameter('encoder_resolution', 1024)  # 编码器分辨率
        self.declare_parameter('wheel_radius', 0.1)  # 轮子半径 (m)
        self.declare_parameter('wheel_base', 0.5)  # 轮距 (m)
        self.declare_parameter('reverse_left_motor', False)  # 是否反转左电机方向
        self.declare_parameter('reverse_right_motor', False)  # 是否反转右电机方向
        
        # 获取参数
        node_id = self.get_parameter('node_id').value
        can_interface = self.get_parameter('can_interface').value
        use_mock = self.get_parameter('use_mock').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.encoder_resolution = self.get_parameter('encoder_resolution').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.reverse_left_motor = self.get_parameter('reverse_left_motor').value
        self.reverse_right_motor = self.get_parameter('reverse_right_motor').value
        
        # 创建CANopen接口
        if use_mock:
            self.get_logger().info('使用模拟CANopen接口')
            can_interface_obj = MockCANopenInterface(node_id, can_interface)
        else:
            # 使用真实的CANopen接口
            self.get_logger().info(f'使用真实CANopen接口: {can_interface}')
            can_interface_obj = SocketCANopenInterface(node_id, can_interface)
            # 连接CAN总线
            if not can_interface_obj.connect():
                self.get_logger().error(f'无法连接到CAN接口 {can_interface}')
                raise RuntimeError(f'CAN接口连接失败: {can_interface}')
        
        # 创建驱动器实例
        self.driver = ZLAC8015DDriver(can_interface_obj)
        
        # QoS配置
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        
        # 创建订阅者
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            qos_profile
        )
        
        self.position_cmd_sub = self.create_subscription(
            JointState,
            'joint_position_command',
            self.position_cmd_callback,
            10
        )
        
        self.velocity_cmd_sub = self.create_subscription(
            JointState,
            'joint_velocity_command',
            self.velocity_cmd_callback,
            10
        )
        
        # 创建发布者
        self.joint_state_pub = self.create_publisher(
            JointState,
            'joint_states',
            qos_profile
        )
        
        self.status_pub = self.create_publisher(
            Int32,
            'driver_status',
            qos_profile
        )
        
        self.fault_pub = self.create_publisher(
            Int32,
            'fault_code',
            qos_profile
        )
        
        # 创建服务
        self.init_srv = self.create_service(
            Trigger,
            'initialize',
            self.initialize_service
        )
        
        self.stop_srv = self.create_service(
            Trigger,
            'stop',
            self.stop_service
        )
        
        self.clear_fault_srv = self.create_service(
            Trigger,
            'clear_fault',
            self.clear_fault_service
        )
        
        self.enable_srv = self.create_service(
            SetBool,
            'enable',
            self.enable_service
        )
        
        # 状态变量
        self.is_initialized = False
        self.is_enabled = False
        self.current_mode = OperationMode.NO_MODE
        
        # 创建定时器
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info(f'ZLAC8015D控制节点已启动 (节点ID: {node_id}, CAN接口: {can_interface})')
    
    def timer_callback(self):
        """定时回调：发布状态信息"""
        if not self.is_initialized:
            return
        
        # 更新驱动器状态
        self.driver.update_state()
        
        # 发布关节状态
        self.publish_joint_state()
        
        # 发布驱动器状态
        status_msg = Int32()
        status_msg.data = int(self.driver.current_state)
        self.status_pub.publish(status_msg)
        
        # 发布故障码
        if self.driver.fault_code != 0:
            fault_msg = Int32()
            fault_msg.data = self.driver.fault_code
            self.fault_pub.publish(fault_msg)
    
    def publish_joint_state(self):
        """发布关节状态"""
        # 获取位置和速度
        left_pos, right_pos = self.driver.get_actual_position()
        left_vel, right_vel = self.driver.get_actual_velocity()
        
        if left_pos is None or right_pos is None:
            return
        
        # 转换为角度（假设编码器反馈的是counts）
        # 根据实际编码器配置调整
        left_angle = (left_pos / self.encoder_resolution) * 2.0 * 3.14159
        right_angle = (right_pos / self.encoder_resolution) * 2.0 * 3.14159
        
        # 转换为角速度 (rad/s)
        left_velocity = (left_vel / 10.0 / 60.0) * 2.0 * 3.14159 if left_vel is not None else 0.0
        right_velocity = (right_vel / 10.0 / 60.0) * 2.0 * 3.14159 if right_vel is not None else 0.0
        
        # 创建JointState消息
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ['left_wheel_joint', 'right_wheel_joint']
        joint_state.position = [left_angle, right_angle]
        joint_state.velocity = [left_velocity, right_velocity]
        joint_state.effort = [0.0, 0.0]  # 转矩信息可以添加
        
        self.joint_state_pub.publish(joint_state)
    
    def cmd_vel_callback(self, msg: Twist):
        """速度命令回调（差速驱动模式）"""
        if not self.is_initialized or not self.is_enabled:
            return
        
        # 差速驱动模型：将线速度和角速度转换为左右轮速度
        linear = msg.linear.x
        angular = msg.angular.z
        
        left_velocity = (linear - angular * self.wheel_base / 2.0) / self.wheel_radius
        right_velocity = (linear + angular * self.wheel_base / 2.0) / self.wheel_radius
        
        # 转换为r/min
        left_rpm = left_velocity * 60.0 / (2.0 * 3.14159)
        right_rpm = right_velocity * 60.0 / (2.0 * 3.14159)
        
        # 应用电机方向反转
        if self.reverse_left_motor:
            left_rpm = -left_rpm
        if self.reverse_right_motor:
            right_rpm = -right_rpm
        
        # 限制速度范围
        left_rpm = max(-1000, min(1000, int(left_rpm)))
        right_rpm = max(-1000, min(1000, int(right_rpm)))
        
        # 如果不在速度模式，切换到速度模式
        if self.current_mode != OperationMode.PROFILE_VELOCITY:
            if self.driver.enable_velocity_mode(sync_control=False):
                self.current_mode = OperationMode.PROFILE_VELOCITY
                self.get_logger().info('切换到速度模式')
        
        # 设置目标速度
        self.driver.set_target_velocity(left_rpm, right_rpm, sync=False)
    
    def position_cmd_callback(self, msg: JointState):
        """位置命令回调"""
        if not self.is_initialized or not self.is_enabled:
            return
        
        if len(msg.position) < 2:
            self.get_logger().warn('位置命令需要至少2个关节')
            return
        
        # 转换为counts（根据实际编码器配置调整）
        left_pos = int(msg.position[0] * self.encoder_resolution / (2.0 * 3.14159))
        right_pos = int(msg.position[1] * self.encoder_resolution / (2.0 * 3.14159))
        
        # 切换到位置模式
        if self.current_mode != OperationMode.PROFILE_POSITION:
            if self.driver.enable_position_mode():
                self.current_mode = OperationMode.PROFILE_POSITION
                self.get_logger().info('切换到位置模式')
        
        # 设置目标位置（使用绝对位置）
        absolute = True
        if len(msg.name) > 0 and 'relative' in str(msg.name).lower():
            absolute = False
        
        self.driver.set_target_position(left_pos, right_pos, absolute=absolute)
    
    def velocity_cmd_callback(self, msg: JointState):
        """速度命令回调（直接关节速度）"""
        if not self.is_initialized or not self.is_enabled:
            return
        
        if len(msg.velocity) < 2:
            self.get_logger().warn('速度命令需要至少2个关节')
            return
        
        # 转换为r/min
        left_rpm = int(msg.velocity[0] * 60.0 / (2.0 * 3.14159))
        right_rpm = int(msg.velocity[1] * 60.0 / (2.0 * 3.14159))
        
        # 应用电机方向反转
        if self.reverse_left_motor:
            left_rpm = -left_rpm
        if self.reverse_right_motor:
            right_rpm = -right_rpm
        
        # 限制速度范围
        left_rpm = max(-1000, min(1000, left_rpm))
        right_rpm = max(-1000, min(1000, right_rpm))
        
        # 切换到速度模式
        if self.current_mode != OperationMode.PROFILE_VELOCITY:
            if self.driver.enable_velocity_mode(sync_control=False):
                self.current_mode = OperationMode.PROFILE_VELOCITY
                self.get_logger().info('切换到速度模式')
        
        # 设置目标速度
        self.driver.set_target_velocity(left_rpm, right_rpm, sync=False)
    
    def initialize_service(self, request, response):
        """初始化服务"""
        self.get_logger().info('执行驱动器初始化...')
        
        if self.driver.initialize():
            self.is_initialized = True
            response.success = True
            response.message = '驱动器初始化成功'
            self.get_logger().info(response.message)
        else:
            response.success = False
            response.message = '驱动器初始化失败'
            self.get_logger().error(response.message)
        
        return response
    
    def stop_service(self, request, response):
        """停止服务"""
        self.get_logger().info('停止电机...')
        self.driver.stop()
        self.is_enabled = False
        response.success = True
        response.message = '电机已停止'
        return response
    
    def clear_fault_service(self, request, response):
        """清除故障服务"""
        self.get_logger().info('清除故障...')
        
        if self.driver.clear_fault():
            response.success = True
            response.message = '故障已清除'
            self.get_logger().info(response.message)
        else:
            response.success = False
            response.message = '清除故障失败'
            self.get_logger().error(response.message)
        
        return response
    
    def enable_service(self, request, response):
        """使能/禁用服务"""
        if request.data:
            if not self.is_initialized:
                response.success = False
                response.message = '请先初始化驱动器'
                return response
            
            self.is_enabled = True
            response.success = True
            response.message = '驱动器已使能'
            self.get_logger().info(response.message)
        else:
            self.driver.stop()
            self.is_enabled = False
            response.success = True
            response.message = '驱动器已禁用'
            self.get_logger().info(response.message)
        
        return response


def main(args=None):
    rclpy.init(args=args)
    
    node = ZLAC8015DControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.driver.stop()
        # 断开CAN连接
        if hasattr(node.driver, 'can') and hasattr(node.driver.can, 'disconnect'):
            node.driver.can.disconnect()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
