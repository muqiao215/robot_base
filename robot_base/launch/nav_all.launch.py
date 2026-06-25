import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_base')

    default_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    default_map = '/home/ubuntu/ros2_ws/map/my_map.yaml'

    # ================= 1. 声明参数文件 / 地图路径 =================
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Full path to the Nav2 parameters file to use'
    )

    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Full path to the map yaml file to use'
    )

    # ================= 2. 底盘驱动 =================
    drive_node = Node(
        package='robot_base',
        executable='drive_node',
        name='drive_node',
        output='screen'
    )

    # ================= 3. 静态TF：base_link → laser =================
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.05', '0.0', '1.35', '0.0', '-0.244346', '3.1415926', 'base_link', 'laser'],
        name='static_tf_laser'
    )

    # ================= 4. RViz 地图重发布 =================
    # map_server 的 /map 是 transient local；部分远端 RViz 对历史地图接收不稳定。
    # 这里用 volatile QoS 低频重发 /map_rviz，只服务 RViz，不影响 Nav2 的 transient-local 订阅。
    map_republisher = Node(
        package='robot_base',
        executable='map_republisher',
        name='map_republisher',
        output='screen',
        parameters=[{
            'input_map_topic': '/map',
            'output_map_topic': '/map_rviz',
            'publish_period': 2.0,
        }]
    )

    # ================= 4. Nav2 导航（一次性包含所有节点） =================
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            )
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': 'false',
            'params_file': LaunchConfiguration('params_file')  # ← 关键：把参数文件传进去！
        }.items()
    )

    return LaunchDescription([
        drive_node,
        static_tf,
        map_republisher,
        declare_params_file_cmd,
        declare_map_cmd,
        nav2_bringup,
    ])
