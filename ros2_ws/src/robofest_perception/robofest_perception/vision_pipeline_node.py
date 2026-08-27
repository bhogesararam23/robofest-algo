import math

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image, Range, Imu
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import String
    from robofest_interfaces.msg import MineMap, MineDetection
except ImportError:
    rclpy = None


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


class VisionPipelineNode(Node):
    def __init__(self):
        super().__init__('vision_pipeline')
        for name, value in [('lower_hsv', [20, 80, 80]), ('upper_hsv', [40, 255, 255])]:
            self.declare_parameter(name, value)
        self.declare_parameter('min_area_px', 50.0)
        self.declare_parameter('min_circularity', 0.35)
        self.declare_parameter('focal_length_px', 420.0)
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
        self.heartbeat = self.create_publisher(String, '/vision/heartbeat', 10)
        self.create_subscription(Range, '/sensor/tof_range', self.range_cb, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_cb, 10)
        self.create_subscription(PoseStamped, '/localization/pose', self.pose_cb, 10)
        self.create_subscription(Image, '/camera/image_raw', self.image_cb, 10)
        self.create_timer(0.1, self.heartbeat_cb)

    def heartbeat_cb(self):
        self.heartbeat.publish(String(data='vision_pipeline'))

    def range_cb(self, msg):
        self.range = msg.range

    def imu_cb(self, msg):
        q = msg.orientation
        roll, pitch, _ = _quaternion_to_euler([q.x, q.y, q.z, q.w])
        self.roll_rad = roll
        self.pitch_rad = pitch

    def pose_cb(self, msg):
        self.drone_x = msg.pose.position.x
        self.drone_y = msg.pose.position.y
        q = msg.pose.orientation
        _, _, yaw = _quaternion_to_euler([q.x, q.y, q.z, q.w])
        self.drone_yaw_rad = yaw

    def image_cb(self, msg):
        if self.range is None or self.range <= 0:
            return
        try:
            import cv2
            import numpy as np
            from cv_bridge import CvBridge
            if not hasattr(self, '_bridge'):
                self._bridge = CvBridge()
            image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            low = np.array(self.get_parameter('lower_hsv').value, dtype=np.uint8)
            high = np.array(self.get_parameter('upper_hsv').value, dtype=np.uint8)
            mask = cv2.inRange(hsv, low, high)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            height, width = image.shape[:2]
            img_cx = width / 2.0
            img_cy = height / 2.0
            img_w = float(self.get_parameter('image_width').value)
            img_h = float(self.get_parameter('image_height').value)
            h_fov = float(self.get_parameter('h_fov_deg').value)
            v_fov = float(self.get_parameter('v_fov_deg').value)
            tan_h_half = math.tan(math.radians(h_fov / 2.0))
            tan_v_half = math.tan(math.radians(v_fov / 2.0))

            # Correct range to true vertical altitude using drone attitude.
            # When the drone is tilted, the slanted range reading is longer
            # than the true altitude above ground directly below the camera.
            cos_pitch = math.cos(self.pitch_rad)
            cos_roll = math.cos(self.roll_rad)
            altitude = self.range * cos_pitch * cos_roll

            out = MineMap()
            out.header = msg.header
            out.header.frame_id = 'base_link'
            mine_id = 1
            for contour in contours:
                area = cv2.contourArea(contour)
                perimeter = cv2.arcLength(contour, True)
                circularity = 4.0 * 3.141592653589793 * area / (perimeter * perimeter) if perimeter else 0.0
                if area < float(self.get_parameter('min_area_px').value) or circularity < float(self.get_parameter('min_circularity').value):
                    continue
                moments = cv2.moments(contour)
                if moments['m00'] == 0:
                    continue
                px = moments['m10'] / moments['m00']
                py = moments['m01'] / moments['m00']

                # Build ray in camera body frame (normalized coords -> ray direction)
                u = (px - img_cx) / (img_w * 0.5)
                v = (py - img_cy) / (img_h * 0.5)
                ray_body_x = u * tan_h_half
                ray_body_y = v * tan_v_half
                ray_body_z = 1.0

                # Apply rotation: R = Rz(yaw) * Ry(pitch) * Rx(roll)
                phi = self.roll_rad
                theta = self.pitch_rad
                psi = self.drone_yaw_rad
                cos_phi = math.cos(phi)
                sin_phi = math.sin(phi)
                cos_theta = math.cos(theta)
                sin_theta = math.sin(theta)
                cos_psi = math.cos(psi)
                sin_psi = math.sin(psi)

                r_z = (-sin_theta * ray_body_x
                       + sin_phi * cos_theta * ray_body_y
                       + cos_phi * cos_theta * ray_body_z)
                if r_z < 0.05:
                    r_z = 0.05

                r_x = ((cos_psi * cos_theta) * ray_body_x
                       + (cos_psi * sin_theta * sin_phi - sin_psi * cos_phi) * ray_body_y
                       + (cos_psi * sin_theta * cos_phi + sin_psi * sin_phi) * ray_body_z)
                r_y = ((sin_psi * cos_theta) * ray_body_x
                       + (sin_psi * sin_theta * sin_phi + cos_psi * cos_phi) * ray_body_y
                       + (sin_psi * sin_theta * cos_phi - cos_psi * sin_phi) * ray_body_z)

                scale = altitude / r_z
                world_x = self.drone_x + r_x * scale
                world_y = self.drone_y + r_y * scale

                det = MineDetection()
                det.id = mine_id
                det.x = float(world_x)
                det.y = float(world_y)
                det.confidence = min(1.0, float(area / 1000.0))
                det.type = 'surface'
                out.mines.append(det)
                mine_id = (mine_id % 255) + 1
            self.pub.publish(out)
        except (ImportError, RuntimeError) as exc:
            self.get_logger().error(f'Perception dependencies unavailable: {exc}')


def main(args=None):
    if rclpy is None:
        raise RuntimeError('ROS 2 is required to run vision_pipeline_node')
    rclpy.init(args=args)
    node = VisionPipelineNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
