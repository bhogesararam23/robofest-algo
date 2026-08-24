try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from robofest_interfaces.msg import MineMap, MineDetection
except ImportError:
    rclpy = None


class FakeVisionNode(Node):
    def __init__(self):
        super().__init__('fake_vision')
        self.pub = self.create_publisher(MineMap, '/perception/mine_detections', 10)
        self.heartbeat = self.create_publisher(String, '/vision/heartbeat', 10)
        self.create_timer(1.0, self.tick)
        self.create_timer(0.1, lambda: self.heartbeat.publish(String(data='fake_vision')))

    def tick(self):
        msg = MineMap()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        for i, (x, y) in enumerate([(8.0, 2.5), (19.5, -1.0), (35.0, 1.2), (46.0, -2.0)], 1):
            detection = MineDetection()
            detection.id = i
            detection.x = x
            detection.y = y
            detection.confidence = 0.85
            detection.type = 'marker' if i % 2 == 0 else 'surface'
            msg.mines.append(detection)
        self.pub.publish(msg)


def main(args=None):
    if rclpy is None:
        raise RuntimeError('ROS 2 is required to run fake_vision_node')
    rclpy.init(args=args)
    node = FakeVisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
