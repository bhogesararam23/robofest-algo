import math

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image, Range, Imu
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import String
    from robofest_interfaces.msg import MineMap, MineDetection, DetectionMap, Detection
except ImportError:
    rclpy = None
    Node = object

from .object_detector import DetectorConfig, ObjectDetector


def _quaternion_to_euler(q):
    """Convert quaternion [x, y, z, w] to (roll, pitch, yaw) in radians."""
    x, y, z, w = q
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def _project_pixel_to_ground(pixel_x, pixel_y, image_width, image_height,
                             altitude, roll, pitch, yaw, h_fov_deg, v_fov_deg,
                             drone_x, drone_y):
    """Project one image pixel onto the ground plane in the odom frame."""
    img_cx = float(image_width) / 2.0
    img_cy = float(image_height) / 2.0
    tan_h_half = math.tan(math.radians(float(h_fov_deg) / 2.0))
    tan_v_half = math.tan(math.radians(float(v_fov_deg) / 2.0))
    u = (float(pixel_x) - img_cx) / (float(image_width) * 0.5)
    v = (float(pixel_y) - img_cy) / (float(image_height) * 0.5)
    ray_body_x, ray_body_y, ray_body_z = u * tan_h_half, v * tan_v_half, 1.0
    cos_phi, sin_phi = math.cos(roll), math.sin(roll)
    cos_theta, sin_theta = math.cos(pitch), math.sin(pitch)
    cos_psi, sin_psi = math.cos(yaw), math.sin(yaw)
    ray_z = (-sin_theta * ray_body_x
             + sin_phi * cos_theta * ray_body_y
             + cos_phi * cos_theta * ray_body_z)
    if ray_z <= 0.05:
        return None
    ray_x = ((cos_psi * cos_theta) * ray_body_x
             + (cos_psi * sin_theta * sin_phi - sin_psi * cos_phi) * ray_body_y
             + (cos_psi * sin_theta * cos_phi + sin_psi * sin_phi) * ray_body_z)
    ray_y = ((sin_psi * cos_theta) * ray_body_x
             + (sin_psi * sin_theta * sin_phi + cos_psi * cos_phi) * ray_body_y
             + (sin_psi * sin_theta * cos_phi - cos_psi * sin_phi) * ray_body_z)
    scale = float(altitude) / ray_z
    return float(drone_x) + ray_x * scale, float(drone_y) + ray_y * scale


class VisionPipelineNode(Node):
    def __init__(self):
        super().__init__('vision_pipeline')
        self.declare_parameter('mine_hsv_ranges', [
            0, 100, 100, 10, 255, 255,
            170, 100, 100, 180, 255, 255,
            20, 80, 80, 40, 255, 255,
        ])
        self.declare_parameter('min_mine_area_px', 50.0)
        self.declare_parameter('max_mine_area_px', 2500.0)
        self.declare_parameter('min_object_area_px', 120.0)
        self.declare_parameter('min_mine_circularity', 0.35)
        self.declare_parameter('object_contrast_threshold', 18.0)
        self.declare_parameter('background_border_px', 8)
        self.declare_parameter('h_fov_deg', 62.0)
        self.declare_parameter('v_fov_deg', 48.0)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)

        self.range = None
        self.roll_rad = 0.0
        self.pitch_rad = 0.0
        self.drone_x = 0.0
        self.drone_y = 0.0
        self.drone_yaw_rad = 0.0

        self.pub = self.create_publisher(MineMap, '/perception/mine_detections', 10)
        self.detection_pub = self.create_publisher(DetectionMap, '/perception/detections', 10)
        self.label_pub = self.create_publisher(String, '/perception/detection_labels', 10)
        self.heartbeat = self.create_publisher(String, '/vision/heartbeat', 10)
        self.create_subscription(Range, '/sensor/tof_range', self.range_cb, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_cb, 10)
        self.create_subscription(PoseStamped, '/localization/pose', self.pose_cb, 10)
        self.create_subscription(Image, '/camera/image_raw', self.image_cb, 10)
        self.create_timer(0.1, self.heartbeat_cb)
        self.detector = None

    def _get_detector(self):
        if self.detector is None:
            values = list(self.get_parameter('mine_hsv_ranges').value)
            if len(values) % 6 != 0:
                raise ValueError('mine_hsv_ranges must contain groups of six values')
            config = DetectorConfig(
                mine_hsv_ranges=tuple(tuple(values[i:i + 6]) for i in range(0, len(values), 6)),
                min_mine_area_px=float(self.get_parameter('min_mine_area_px').value),
                max_mine_area_px=float(self.get_parameter('max_mine_area_px').value),
                min_object_area_px=float(self.get_parameter('min_object_area_px').value),
                min_mine_circularity=float(self.get_parameter('min_mine_circularity').value),
                object_contrast_threshold=float(self.get_parameter('object_contrast_threshold').value),
                background_border_px=int(self.get_parameter('background_border_px').value),
            )
            self.detector = ObjectDetector(config)
        return self.detector

    def heartbeat_cb(self):
        self.heartbeat.publish(String(data='vision_pipeline'))

    def range_cb(self, msg):
        self.range = msg.range

    def imu_cb(self, msg):
        q = msg.orientation
        self.roll_rad, self.pitch_rad, _ = _quaternion_to_euler([q.x, q.y, q.z, q.w])

    def pose_cb(self, msg):
        self.drone_x = msg.pose.position.x
        self.drone_y = msg.pose.position.y
        q = msg.pose.orientation
        _, _, self.drone_yaw_rad = _quaternion_to_euler([q.x, q.y, q.z, q.w])

    def image_cb(self, msg):
        try:
            from cv_bridge import CvBridge
            if not hasattr(self, '_bridge'):
                self._bridge = CvBridge()
            image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            detections = self._get_detector().detect(image)
            output = DetectionMap()
            output.header = msg.header
            output.header.frame_id = 'odom'
            mines = MineMap()
            mines.header = msg.header
            mines.header.frame_id = 'base_link'
            labels = []
            image_height, image_width = image.shape[:2]
            for detection_id, item in enumerate(detections, 1):
                result = Detection()
                result.id = detection_id
                result.type = item.label
                result.confidence = item.confidence
                result.pixel_x = item.pixel_x
                result.pixel_y = item.pixel_y
                result.width_px = item.width_px
                result.height_px = item.height_px
                ground = None
                if self.range is not None and math.isfinite(self.range) and self.range > 0.0:
                    altitude = self.range * math.cos(self.pitch_rad) * math.cos(self.roll_rad)
                    ground = _project_pixel_to_ground(
                        item.pixel_x, item.pixel_y, image_width, image_height, altitude,
                        self.roll_rad, self.pitch_rad, self.drone_yaw_rad,
                        self.get_parameter('h_fov_deg').value,
                        self.get_parameter('v_fov_deg').value,
                        self.drone_x, self.drone_y,
                    )
                result.position_valid = ground is not None
                if ground is not None:
                    result.x, result.y = ground
                output.detections.append(result)
                labels.append(f'{item.label}:{detection_id}')
                if item.label == 'mine' and ground is not None:
                    mine = MineDetection()
                    mine.id = detection_id
                    # MineMap's existing subscriber expects detections in the
                    # base_link frame and applies the drone pose itself.
                    mine.x = ground[0] - self.drone_x
                    mine.y = ground[1] - self.drone_y
                    mine.confidence = item.confidence
                    mine.type = 'mine'
                    mines.mines.append(mine)
            self.detection_pub.publish(output)
            self.label_pub.publish(String(data=','.join(labels) if labels else 'NONE'))
            if mines.mines:
                self.pub.publish(mines)
        except (ImportError, RuntimeError, ValueError) as exc:
            self.get_logger().error(f'Perception unavailable: {exc}')


def main(args=None):
    if rclpy is None:
        raise RuntimeError('ROS 2 is required to run vision_pipeline_node')
    rclpy.init(args=args)
    node = VisionPipelineNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
