import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import serial
import threading
import time
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class RobotDriveNode(Node):
    def __init__(self):
        super().__init__('robot_drive_node')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.wheel_speeds = [0.0, 0.0, 0.0, 0.0]  # m1, m2, m3, m4
        self.last_feedback_time = None
        self.last_cmd_time = None
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_wz = 0.0
        self.feedback_timeout = 0.5
        self.cmd_timeout = 0.5
        self.min_feedback_speed = 0.01
        self.min_cmd_speed = 0.01
        self._last_drive_debug_time = 0.0
        self._last_sent_pwm = (0, 0, 0, 0)

        # 电机启动死区(参数化，便于实车标定)：当 |PWM| 落在
        # [min_command_pwm_input, min_command_pwm) 区间时，补偿拉到 min_command_pwm，
        # 否则落地电机克服不了静摩擦只会颤动。实测本机死区下限约 1100~1150。
        self.declare_parameter('min_command_pwm', 1150)
        self.min_command_pwm = self.get_parameter('min_command_pwm').get_parameter_value().integer_value
        self.declare_parameter('min_command_pwm_input', 120)
        self.min_command_pwm_input = self.get_parameter('min_command_pwm_input').get_parameter_value().integer_value

        # 反馈丢失时与其让 odom 冻结(导致整栈卡死、被迫重启)，不如开环积分——会漂但栈不死。
        self.declare_parameter('allow_cmd_odom_fallback', True)
        self.allow_cmd_odom_fallback = self.get_parameter(
            'allow_cmd_odom_fallback'
        ).get_parameter_value().bool_value
        self._reported_feedback_problem = False

        # ================= 硬件参数 =================
        self.wheel_diameter = 0.097       # 轮子直径 97mm
        self.wheel_radius = self.wheel_diameter / 2.0

        # 轴距参数
        self.wheel_base = 0.445           # 左右轮距 (m)
        self.axle_length = 0.17           # 前后轴距 (m)

        # 旋转系数 k = lx + ly = 前后轴距/2 + 左右轮距/2 ≈ 0.3075。
        # IK 与 FK 共用此常数，保证自洽。做成参数，实车标定时无需重新编译。
        self.declare_parameter('kinematic_k', (self.wheel_base + self.axle_length) / 2.0)
        self.k_rot = self.get_parameter('kinematic_k').get_parameter_value().double_value

        # 反馈极性。控制板若回传方向与指令相反，则为 -1；方向反了就翻成 1。实车标定确认。
        self.declare_parameter('feedback_sign', -1.0)
        self.feedback_sign = self.get_parameter('feedback_sign').get_parameter_value().double_value

        # PWM 映射：3600 PWM → 820 mm/s
        self.pwm_per_mms = 3600.0 / 820.0   # 每 mm/s 对应多少 PWM

        # ================= 串口配置 =================
        self.ser = None
        self.serial_port = '/dev/ttyUSB0'
        self.baudrate = 115200
        self.connect_serial()

        # 开启速度回传
        self.enable_feedback()

        # 重连定时器常驻：运行中 USB 抖动/掉线也能自动重连并重新开回传。
        self.create_timer(2.0, self.check_and_reconnect)

        # ================= ROS2 通信 =================
        self.subscription = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        # 状态变量
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # 启动串口读取线程
        self.running = True
        self.serial_thread = threading.Thread(target=self.read_serial)
        self.serial_thread.start()

        # 定时器：发布里程计 (50Hz)
        self.create_timer(0.02, self.publish_odom)

        self.get_logger().info(
            'Robot Drive Node Started! (k_rot=%.4f, feedback_sign=%.1f, fallback=%s)' % (
                self.k_rot, self.feedback_sign, self.allow_cmd_odom_fallback
            )
        )

    def enable_feedback(self):
        """发送 $upload:0,0,1# 开启控制板速度回传。每次(重)连后都要发一次，否则反馈不恢复。"""
        if self.ser and self.ser.is_open:
            try:
                self.ser.write("$upload:0,0,1#\n".encode())
                self.get_logger().info('已发送指令开启速度回传')
            except Exception as e:
                self.get_logger().error(f'发送回传指令失败: {e}')

    def connect_serial(self):
        try:
            # 如果已有串口且开着，先关掉
            if self.ser and self.ser.is_open:
                self.ser.close()

            self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=0.1)
            self.get_logger().info(f'Serial connected: {self.serial_port}')
            return True
        except Exception as e:
            self.get_logger().error(f'Serial connection failed: {e}')
            self.ser = None
            return False

    def send_motor_cmd(self, m1, m2, m3, m4):
        def compensate_deadband(pwm):
            magnitude = abs(pwm)
            if self.min_command_pwm_input <= magnitude < self.min_command_pwm:
                return math.copysign(self.min_command_pwm, pwm)
            return pwm

        m1 = max(-1300, min(1300, round(compensate_deadband(m1))))
        m2 = max(-1300, min(1300, round(compensate_deadband(m2))))
        m3 = max(-1300, min(1300, round(compensate_deadband(m3))))
        m4 = max(-1300, min(1300, round(compensate_deadband(m4))))
        self._last_sent_pwm = (m1, m2, m3, m4)
        cmd = f'$pwm:{m1},{m2},{m3},{m4}#'
        try:
            # 如果串口没开，先尝试重连
            if self.ser is None or not self.ser.is_open:
                self.connect_serial()

            if self.ser and self.ser.is_open:
                self.ser.write(cmd.encode())
            else:
                self.get_logger().error('Serial is closed, cannot send.')

        except Exception as e:
            self.get_logger().error(f'Send error: {e}')
            # 发生严重错误（如IO错误），关闭串口准备重连
            if self.ser:
                try:
                    self.ser.close()
                except Exception:
                    pass
            self.ser = None  # 置空，让下一次发送/定时器触发重连

    def check_and_reconnect(self):
        """检查串口状态，如果断开则重连，并重新开启回传。"""
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn('Serial disconnected, trying to reconnect...')
            if self.connect_serial():
                self.get_logger().info('Reconnected successfully!')
                # 重连后必须重发 upload，否则回传不恢复 → odom 永久冻结。
                self.enable_feedback()

    def read_serial(self):
        buffer = ''
        while self.running:
            try:
                # 串口不可用时短暂休眠等待重连，而不是空转。
                if self.ser is None or not self.ser.is_open:
                    time.sleep(0.05)
                    continue

                n = self.ser.in_waiting
                if n:
                    data = self.ser.read(n).decode('utf-8', errors='ignore')
                    buffer += data
                    while '$' in buffer and '#' in buffer:
                        start = buffer.find('$')
                        end = buffer.find('#')
                        if end > start:
                            line = buffer[start:end + 1]
                            buffer = buffer[end + 1:]
                            if line.startswith('$MSPD:'):
                                parts = line[6:-1].split(',')
                                if len(parts) == 4:
                                    try:
                                        self.wheel_speeds = [float(p) for p in parts]
                                        self.last_feedback_time = time.monotonic()
                                    except ValueError:
                                        pass
                        else:
                            # '#' 在 '$' 之前（半包/脏数据）：丢掉 '$' 之前的垃圾并跳出，
                            # 等待后续数据，避免死循环空转。
                            buffer = buffer[start:]
                            break
            except (serial.SerialException, OSError) as e:
                # 串口出错就关闭置空，交给 check_and_reconnect 重连。
                self.get_logger().error(f'Serial read error: {e}; will reconnect')
                try:
                    if self.ser:
                        self.ser.close()
                except Exception:
                    pass
                self.ser = None
            except Exception as e:
                self.get_logger().error(f'Unexpected read_serial error: {e}')
            time.sleep(0.005)

    def cmd_vel_callback(self, msg):
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z
        self.cmd_vx = vx
        self.cmd_vy = vy
        self.cmd_wz = wz
        self.last_cmd_time = time.monotonic()

        # 这里只负责发送电机控制指令（里程计积分在 publish_odom 用反馈做）

        # 标准 X 型麦克纳姆轮逆运动学，转向系数用半距之和 k_rot。
        k = self.k_rot
        m1 = vx - vy - k * wz
        m2 = vx + vy - k * wz
        m3 = vx + vy + k * wz
        m4 = vx - vy + k * wz

        # 转换为 PWM
        pwm1 = m1 * 1000.0 * self.pwm_per_mms
        pwm2 = m2 * 1000.0 * self.pwm_per_mms
        pwm3 = m3 * 1000.0 * self.pwm_per_mms
        pwm4 = m4 * 1000.0 * self.pwm_per_mms

        self.send_motor_cmd(pwm1, pwm2, pwm3, pwm4)

    def publish_odom(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        monotonic_now = time.monotonic()
        has_recent_feedback = (
            self.last_feedback_time is not None and
            monotonic_now - self.last_feedback_time <= self.feedback_timeout
        )

        has_recent_cmd = (
            self.last_cmd_time is not None and
            monotonic_now - self.last_cmd_time <= self.cmd_timeout
        )
        cmd_is_nonzero = (
            abs(self.cmd_vx) > self.min_cmd_speed or
            abs(self.cmd_vy) > self.min_cmd_speed or
            abs(self.cmd_wz) > self.min_cmd_speed
        )

        feedback_is_zero = False
        if has_recent_feedback:
            feedback_is_zero = max(abs(speed) for speed in self.wheel_speeds) < self.min_feedback_speed * 1000.0

        # 选择速度来源：优先用反馈做正运动学；但若"板子回传 0 而我们正在发非零指令"
        # (已知偶发故障)且开了兜底，则改用 cmd_vel，避免 odom 错误冻结。
        feedback_bogus_zero = feedback_is_zero and has_recent_cmd and cmd_is_nonzero
        use_feedback = has_recent_feedback and not (feedback_bogus_zero and self.allow_cmd_odom_fallback)

        if use_feedback:
            # 控制板回传 mm/s → m/s，乘 feedback_sign 统一极性，再用与 IK 自洽的正运动学积分。
            s = self.feedback_sign
            m1 = s * self.wheel_speeds[0] / 1000.0
            m2 = s * self.wheel_speeds[1] / 1000.0
            m3 = s * self.wheel_speeds[2] / 1000.0
            m4 = s * self.wheel_speeds[3] / 1000.0

            vx = (m1 + m2 + m3 + m4) / 4.0
            vy = (-m1 + m2 + m3 - m4) / 4.0
            wz = (-m1 - m2 + m3 + m4) / (4.0 * self.k_rot)

            if self._reported_feedback_problem and not feedback_is_zero:
                self.get_logger().info('Wheel speed feedback restored; using MSPD odometry')
                self._reported_feedback_problem = False
        elif has_recent_cmd and self.allow_cmd_odom_fallback:
            # 开环兜底：用 cmd_vel 当速度。会漂，但保证 odom/TF/AMCL 不冻结、栈不卡死。
            vx = self.cmd_vx
            vy = self.cmd_vy
            wz = self.cmd_wz
            if not self._reported_feedback_problem:
                self.get_logger().warn(
                    'MSPD wheel-speed feedback missing/zero; falling back to cmd_vel odometry'
                )
                self._reported_feedback_problem = True
        else:
            vx = 0.0
            vy = 0.0
            wz = 0.0
            if has_recent_cmd and cmd_is_nonzero and not self._reported_feedback_problem:
                self.get_logger().warn(
                    'MSPD wheel-speed feedback is missing or zero while cmd_vel is nonzero; '
                    'odometry holding still (enable allow_cmd_odom_fallback to dead-reckon)'
                )
                self._reported_feedback_problem = True

        if has_recent_cmd and cmd_is_nonzero and monotonic_now - self._last_drive_debug_time >= 1.0:
            self._last_drive_debug_time = monotonic_now
            self.get_logger().info(
                'drive debug: cmd_vel=(%.3f, %.3f, %.3f) pwm=%s mspd=%s recent_feedback=%s feedback_zero=%s' % (
                    self.cmd_vx, self.cmd_vy, self.cmd_wz,
                    self._last_sent_pwm,
                    tuple(round(s, 1) for s in self.wheel_speeds),
                    has_recent_feedback, feedback_is_zero
                )
            )

        # 积分更新位姿
        self.x += (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt
        self.y += (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt
        self.theta += wz * dt

        # 发布 TF
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)
        self.tf_broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        # 填上 twist，便于下游(调试/EKF)使用。
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        self.odom_pub.publish(odom)

    def destroy_node(self):
        self.running = False
        if self.ser:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RobotDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
