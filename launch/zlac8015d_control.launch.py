from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 获取包路径
    pkg_share = FindPackageShare(package='zlac8015d_control').find('zlac8015d_control')
    
    # 声明启动参数
    node_id_arg = DeclareLaunchArgument(
        'node_id',
        default_value='1',
        description='CANopen节点ID'
    )
    
    can_interface_arg = DeclareLaunchArgument(
        'can_interface',
        default_value='can0',
        description='CAN接口名称'
    )
    
    use_mock_arg = DeclareLaunchArgument(
        'use_mock',
        default_value='false',
        description='是否使用模拟接口'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='default.yaml',
        description='配置文件名称'
    )
    
    # 创建节点
    control_node = Node(
        package='zlac8015d_control',
        executable='zlac8015d_control_node',
        name='zlac8015d_control_node',
        parameters=[
            PathJoinSubstitution([pkg_share, 'config', LaunchConfiguration('config_file')]),
            {
                'node_id': LaunchConfiguration('node_id'),
                'can_interface': LaunchConfiguration('can_interface'),
                'use_mock': LaunchConfiguration('use_mock'),
            }
        ],
        output='screen'
    )
    
    return LaunchDescription([
        node_id_arg,
        can_interface_arg,
        use_mock_arg,
        config_file_arg,
        control_node,
    ])
