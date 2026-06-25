import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

# 底盘单独启动：drive_node + base_link->laser 静态 TF。
# 与 Nav2(nav_all.launch.py)分开，方便单独重启调试底盘，不影响导航栈/RViz 连接。
#
# 用法:
#   ros2 launch robot_base drive.launch.py
# 调试时可直接传参（无需重新编译/改代码），例如：
#   ros2 launch robot_base drive.launch.py kinematic_k:=0.31 feedback_sign:=1.0 min_command_pwm:=1150
# 也可运行时热调（仅对已声明为 ROS 参数的项有效，如 kinematic_k / feedback_sign / allow_cmd_odom_fallback）：
#   ros2 param set /drive_node kinematic_k 0.31


def generate_launch_description():
    # ---- 可调参数（带默认值，与 drive_node.py 内默认一致）----
    # 旋转系数 k = (左右轮距 + 前后轴距)/2 ≈ (0.445 + 0.17)/2 = 0.3075
    declare_k = DeclareLaunchArgument(
        'kinematic_k', default_value='0.3075',
        description='麦轮旋转系数 k=(L+W)/2；原地转一圈看 odom yaw 是否≈2π 来标定'
    )
    # 反馈极性：控制板回传方向与指令相反时为 -1，方向反了改 1
    declare_sign = DeclareLaunchArgument(
        'feedback_sign', default_value='-1.0',
        description='轮速反馈极性(-1 或 1)；前进时 odom x 应增加'
    )
    # 反馈丢失时是否用 cmd_vel 开环兜底
    declare_fallback = DeclareLaunchArgument(
        'allow_cmd_odom_fallback', default_value='true',
        description='反馈丢失时用 cmd_vel 开环积分兜底，避免 odom 冻结'
    )
    # 电机启动死区下限：低速指令 PWM 会被拉到此值，否则落地电机只颤不转。实测约 1100~1150。
    declare_min_pwm = DeclareLaunchArgument(
        'min_command_pwm', default_value='1150',
        description='电机启动死区下限 PWM；太低则低速指令电机转不动'
    )

    # ================= 底盘驱动 =================
    drive_node = Node(
        package='robot_base',
        executable='drive_node',
        name='drive_node',
        output='screen',
        parameters=[{
            'kinematic_k': LaunchConfiguration('kinematic_k'),
            'feedback_sign': LaunchConfiguration('feedback_sign'),
            'allow_cmd_odom_fallback': LaunchConfiguration('allow_cmd_odom_fallback'),
            'min_command_pwm': LaunchConfiguration('min_command_pwm'),
        }]
    )

    # ================= 静态TF：base_link → laser =================
    # 用命名参数避免 roll/pitch/yaw 顺序误解；数值需以实物安装方向为准，
    # 在 RViz 里叠加 Map+LaserScan 确认激光点云与地图墙体重合。
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0.05',
            '--y', '0.0',
            '--z', '1.35',
            '--roll', '0.0',
            '--pitch', '-0.244346',
            '--yaw', '3.1415926',
            '--frame-id', 'base_link',
            '--child-frame-id', 'laser',
        ],
        name='static_tf_laser'
    )

    return LaunchDescription([
        declare_k,
        declare_sign,
        declare_fallback,
        declare_min_pwm,
        drive_node,
        static_tf,
    ])
