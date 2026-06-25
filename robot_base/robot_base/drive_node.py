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
        self.min_command_pwm = 900
        self.min_command_pwm_input = 120
        self._last_drive_debug_time = 0.0
        self._last_sent_pwm = (0, 0, 0, 0)
        self.declare_parameter('allow_cmd_odom_fallback', False)
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
        
        # PWM 映射：3600 PWM → 820 mm/s
        self.pwm_per_mms = 3600.0 / 820.0   # 每 mm/s 对应多少 PWM
        
        # ================= 串口配置 =================
        self.ser = None
        self.serial_port = '/dev/ttyUSB0'
        self.baudrate = 115200
        self.connect_serial()
        
        # ★★★ 新增：发送指令，开启速度回传 ★★★
        if self.ser and self.ser.is_open:
            upload_cmd = "$upload:0,0,1#\n"
            try:
                self.ser.write(upload_cmd.encode())
                self.get_logger().info('已发送指令开启速度回传')
            except Exception as e:
                self.get_logger().error(f'发送指令失败: {e}')
        
        if not self.ser or not self.ser.is_open:
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

        self.get_logger().info('Robot Drive Node Started!')

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
                except:
                    pass
            self.ser = None # 置空，让下一次发送触发重连

    def check_and_reconnect(self):
        """检查串口状态，如果断开则重连"""
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn('Serial disconnected, trying to reconnect...')
            self.connect_serial()
            if self.ser and self.ser.is_open:
                self.get_logger().info('Reconnected successfully!')

    def read_serial(self):
        buffer = ''
        while self.running:
            try:
                if self.ser and self.ser.in_waiting:
                    data = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
                    buffer += data
                    while '$' in buffer and '#' in buffer:
                        start = buffer.find('$')
                        end = buffer.find('#')
                        if end > start:
                            line = buffer[start:end+1]
                            buffer = buffer[end+1:]
                            if line.startswith('$MSPD:'):
                                parts = line[6:-1].split(',')
                                if len(parts) == 4:
                                    try:
                                        # ★★★ 关键修改：赋值给 self.wheel_speeds ★★★
                                        self.wheel_speeds = [float(p) for p in parts]
                                        self.last_feedback_time = time.monotonic()
                                    except:
                                        pass
            except:
                pass
            time.sleep(0.005)

    def cmd_vel_callback(self, msg):
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z
        self.cmd_vx = vx
        self.cmd_vy = vy
        self.cmd_wz = wz
        self.last_cmd_time = time.monotonic()

        # ★★★ 删除这里的里程计积分（改用串口反馈积分）★★★
        # 这里只负责发送电机控制指令

        L = self.wheel_base
        W = self.axle_length

        # 标准X型麦克纳姆轮逆运动学
        m1 = vx - vy - (L + W) * wz
        m2 = vx + vy - (L + W) * wz
        m3 = vx + vy + (L + W) * wz
        m4 = vx - vy + (L + W) * wz

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

        if has_recent_feedback:
            if has_recent_cmd and cmd_is_nonzero and feedback_is_zero and not self._reported_feedback_problem:
                self.get_logger().warn(
                    'MSPD wheel-speed feedback is zero while cmd_vel is nonzero; '
                    'holding odometry on wheel feedback only'
                )
                self._reported_feedback_problem = True

            # 控制板回传的是 mm/s，换算成 m/s 后用麦克纳姆正运动学积分。
            m1 = self.wheel_speeds[0] / 1000.0
            m2 = self.wheel_speeds[1] / 1000.0
            m3 = self.wheel_speeds[2] / 1000.0
            m4 = self.wheel_speeds[3] / 1000.0

            vx = -(m1 + m2 + m3 + m4) / 4.0
            vy = -(-m1 + m2 + m3 - m4) / 4.0
            wz = -(m3 + m4 - m1 - m2) / (self.wheel_base * 2.5)

            if self._reported_feedback_problem and not feedback_is_zero:
                self.get_logger().info('Wheel speed feedback restored; using MSPD odometry')
                self._reported_feedback_problem = False
        elif has_recent_cmd and self.allow_cmd_odom_fallback:
            vx = self.cmd_vx
            vy = self.cmd_vy
            wz = self.cmd_wz
            if not self._reported_feedback_problem:
                self.get_logger().warn(
                    'MSPD wheel-speed feedback is missing; falling back to cmd_vel odometry'
                )
                self._reported_feedback_problem = True
        else:
            vx = 0.0
            vy = 0.0
            wz = 0.0
            if has_recent_cmd and cmd_is_nonzero and not self._reported_feedback_problem:
                self.get_logger().warn(
                    'MSPD wheel-speed feedback is missing or zero while cmd_vel is nonzero; '
                    'holding odometry on wheel feedback only'
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
