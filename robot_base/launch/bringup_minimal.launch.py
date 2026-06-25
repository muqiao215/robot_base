import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    drive_node = Node(
        package='robot_base',
        executable='drive_node',
        name='drive_node',
        output='screen'
    )
    
    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('sllidar_ros2'),
                'launch',
                'sllidar_c1_launch.py'
            )
        ]),
        launch_arguments={'serial_port': '/dev/ttyUSB1'}.items()
    )
    
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.05', '0.0', '1.35', '0.0', '-0.244346', '0.0', 'base_link', 'laser'],
        name='static_tf_laser'
    )
    
    return LaunchDescription([
        drive_node,
        sllidar_launch,
        static_tf,
    ])
