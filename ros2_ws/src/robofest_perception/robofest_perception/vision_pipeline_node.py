try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image, Range
    from robofest_interfaces.msg import MineMap, MineDetection
except ImportError:
    rclpy = None

class VisionPipelineNode(Node):
    def __init__(self):
        super().__init__('vision_pipeline')
        for name, value in [('lower_hsv',[20,80,80]),('upper_hsv',[40,255,255])]: self.declare_parameter(name, value)
        self.declare_parameter('min_area_px', 50.0); self.declare_parameter('min_circularity', 0.35); self.declare_parameter('focal_length_px', 420.0)
        self.range = None
        self.pub = self.create_publisher(MineMap, '/perception/mine_detections', 10)
        self.create_subscription(Range, '/sensor/tof_range', self.range_cb, 10)
        self.create_subscription(Image, '/camera/image_raw', self.image_cb, 10)
    def range_cb(self, msg): self.range = msg.range
    def image_cb(self, msg):
        if self.range is None or self.range <= 0: return
        try:
            import cv2
            import numpy as np
            from cv_bridge import CvBridge
            if not hasattr(self, '_bridge'): self._bridge = CvBridge()
            image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            low = np.array(self.get_parameter('lower_hsv').value, dtype=np.uint8); high = np.array(self.get_parameter('upper_hsv').value, dtype=np.uint8)
            mask = cv2.inRange(hsv, low, high); contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            height, width = image.shape[:2]; cx, cy = width / 2.0, height / 2.0; focal = float(self.get_parameter('focal_length_px').value)
            out = MineMap(); out.header = msg.header; out.header.frame_id = 'base_link'; mine_id = 1
            for contour in contours:
                area = cv2.contourArea(contour)
                perimeter = cv2.arcLength(contour, True)
                circularity = 4.0 * 3.141592653589793 * area / (perimeter * perimeter) if perimeter else 0.0
                if area < float(self.get_parameter('min_area_px').value) or circularity < float(self.get_parameter('min_circularity').value): continue
                moments = cv2.moments(contour)
                if moments['m00'] == 0: continue
                px = moments['m10'] / moments['m00']; py = moments['m01'] / moments['m00']
                det = MineDetection(); det.id = mine_id; det.x = float((px-cx) * self.range / focal); det.y = float((py-cy) * self.range / focal); det.confidence = min(1.0, float(area / 1000.0)); det.type = 'surface'; out.mines.append(det); mine_id = (mine_id % 255) + 1
            self.pub.publish(out)
        except (ImportError, RuntimeError) as exc:
            self.get_logger().error(f'Perception dependencies unavailable: {exc}')

def main(args=None):
    if rclpy is None: raise RuntimeError('ROS 2 is required to run vision_pipeline_node')
    rclpy.init(args=args); node=VisionPipelineNode(); rclpy.spin(node); node.destroy_node(); rclpy.shutdown()
