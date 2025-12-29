from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # cmd_vel 到 wheels_control 转换节点
        Node(
            package='zlac8015d',
            executable='cmd_vel_to_wheels_node',
            name='cmd_vel_to_wheels_node',
            output='screen',
            parameters=[
                {'wheel_base': 0.3},           # 轮距，单位：米
                {'max_linear_speed': 2.0},     # 最大线速度，单位：米/秒
                {'max_angular_speed': 2.0}    # 最大角速度，单位：弧度/秒
            ]
        ),
    ])