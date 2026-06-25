import copy

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid


class MapRepublisher(Node):
    def __init__(self):
        super().__init__('map_republisher')
        self.declare_parameter('input_map_topic', '/map')
        self.declare_parameter('output_map_topic', '/map_rviz')
        self.declare_parameter('publish_period', 2.0)

        self._input_map_topic = self.get_parameter('input_map_topic').value
        self._output_map_topic = self.get_parameter('output_map_topic').value
        publish_period = float(self.get_parameter('publish_period').value)
        if publish_period <= 0.0:
            publish_period = 2.0

        self._last_map = None
        self._received_original = False

        transient_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        volatile_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._publisher = self.create_publisher(OccupancyGrid, self._output_map_topic, volatile_qos)
        self._subscription = self.create_subscription(
            OccupancyGrid,
            self._input_map_topic,
            self._on_map,
            transient_qos,
        )
        self._timer = self.create_timer(publish_period, self._publish_latest)

        self.get_logger().info(
            f"Republishing {self._input_map_topic} to {self._output_map_topic} every {publish_period:.1f}s with VOLATILE QoS for RViz"
        )

    def _on_map(self, msg):
        self._last_map = copy.deepcopy(msg)
        if not self._received_original:
            self._received_original = True
            self.get_logger().info(
                f"Received map {msg.info.width}x{msg.info.height} frame={msg.header.frame_id}; publishing RViz copy"
            )
        self._publish_latest()

    def _publish_latest(self):
        if self._last_map is None:
            return
        self._publisher.publish(self._last_map)


def main(args=None):
    rclpy.init(args=args)
    node = MapRepublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
