import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 底盘驱动节点
    drive_node = Node(
        package='robot_base',
        executable='drive_node',
        name='drive_node',
        output='screen'
    )
    
    # 2. 静态 TF：base_link → laser
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.05', '0.0', '1.35', '0.0', '-0.244346', '0.0', 'base_link', 'laser'],
        name='static_tf_laser'
    )
    

    # 3. SLAM Toolbox 异步建图
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[{
            'use_sim_time': False,
            'odom_frame': 'odom',
            'map_frame': 'map',
            'base_frame': 'base_link',
            'scan_topic': '/scan',
            'mode': 'mapping',
            'resolution': 0.1,
            'max_laser_range': 12.0,
            'wait_for_transform_duration': 5.0,
            'map_update_interval': 1.0,
            'use_lifecycle_managed_nodes': False,
            'minimum_time_interval': 0.5,
            'transform_timeout': 0.2,
            'tf_buffer_duration': 30.0,
            'stack_size_to_use': 40000000,
            'enable_interactive_mode': False,
        }],
        remappings=[
            ('scan', '/scan'),
            ('/map', '/map')
        ],
        output='screen'
    )
    
    return LaunchDescription([
        drive_node,
        static_tf,
        slam_toolbox,
    ])
