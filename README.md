# robot_base

A ROS 2 (`ament_python`) package for a 4-wheel **mecanum** robot base. It bridges
`cmd_vel` to the motor controller over serial, integrates wheel-speed feedback
into odometry + `odom→base_link` TF, and ships launch files for SLAM
(slam_toolbox) and Nav2 navigation.

Developed on ROS 2 Jazzy / Python 3.12 / Ubuntu 24.04 (Raspberry Pi).

## Nodes

| Executable | Description |
| --- | --- |
| `drive_node` | Subscribes `cmd_vel`, runs mecanum inverse kinematics → PWM, reads `$MSPD` wheel-speed feedback from the controller, integrates forward kinematics and publishes `odom` + TF `odom→base_link`. Auto-reconnects the serial port. |
| `map_republisher` | Republishes `/map` (transient-local) to `/map_rviz` with VOLATILE QoS at a low rate, so remote RViz clients receive the map reliably without disturbing Nav2's transient-local subscribers. |
| `test_wheels` | Standalone (non-ROS) script to spin each wheel forward/reverse for wiring verification. |

## Hardware

- 4× mecanum wheels, X-pattern wiring.
- Wheel diameter `0.097 m`, wheel base (L–R) `0.445 m`, axle (F–R) `0.17 m`.
- Motor controller on serial `/dev/ttyUSB0` @ `115200` baud.
  - PWM command: `$pwm:m1,m2,m3,m4#` (clamped to ±1300).
  - Wheel-speed feedback: `$MSPD:m1,m2,m3,m4#` (mm/s), enabled via `$upload:0,0,1#`.
  - PWM mapping: `3600 PWM ≈ 820 mm/s`.
- LiDAR: Slamtec C1 via `sllidar_ros2` on `/dev/ttyUSB1`.
- Static TF `base_link→laser`: `x=0.05 z=1.35 yaw≈-0.244 rad`.

## Topics / TF

- Subscribes: `cmd_vel` (`geometry_msgs/Twist`).
- Publishes: `odom` (`nav_msgs/Odometry`), TF `odom→base_link`.
- SLAM: `/scan` → `/map`; `map_republisher` → `/map_rviz`.

## Build

```bash
# inside a colcon workspace src/
cd <ws>/src
git clone https://github.com/muqiao215/robot_base.git
cd <ws>
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select robot_base
source install/setup.bash
```

Dependencies: `rclpy geometry_msgs nav_msgs sensor_msgs serial tf2_ros`
(runtime); `slam_toolbox`, `nav2_bringup`, `sllidar_ros2`, `rviz2` for the
launch files.

## Launch

```bash
# Drive + SLAM (mapping)
ros2 launch robot_base bringup.launch.py

# Drive + LiDAR + static TF (minimal, no SLAM/Nav)
ros2 launch robot_base bringup_minimal.launch.py

# Drive + Nav2 + map republisher (navigation)
ros2 launch robot_base nav_all.launch.py

# RViz with the bundled config
ros2 launch robot_base rviz_nav.launch.py
```

> Note: `nav_all.launch.py` currently contains absolute paths
> (`/home/ubuntu/ros2_ws/src/robot_base/config/nav2_params.yaml` and
> `/home/ubuntu/ros2_ws/map/my_map.yaml`). Override the Nav2 params file with
> `params_file:=...`; adjust `map_yaml` in the launch file for your layout.

## Repository layout

```
robot_base/        ROS 2 package (source, launch, config, rviz)
map/               example occupancy grid (my_map.pgm / my_map.yaml)
docs/frames/       TF tree snapshots (tf2_tools view_frames)
```

## License

Apache License 2.0. See [LICENSE](LICENSE).
