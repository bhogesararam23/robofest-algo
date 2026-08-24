from time import monotonic
try:
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
except ImportError:
    rclpy = None
class SafetyGeofenceNode(Node):
    def __init__(self):
        super().__init__('safety_geofence'); self.x=0.; self.y=0.; self.last_odom=0.; self.heartbeats={}; self.pub=self.create_publisher(String,'/mission/current_command',10)
        self.create_subscription(Odometry,'/odom',self.odom_cb,10)
        for topic in ['/vision/heartbeat','/localization/heartbeat','/drivers/heartbeat']: self.create_subscription(String,topic,lambda m,t=topic:self.heartbeat_cb(t),10)
        self.create_timer(0.1,self.check)
    def odom_cb(self,msg): self.x=msg.pose.pose.position.x; self.y=msg.pose.pose.position.y; self.last_odom=monotonic()
    def heartbeat_cb(self,topic): self.heartbeats[topic]=monotonic()
    def check(self):
        unsafe=not (0.0 <= self.x <= 60.0 and 0.0 <= self.y <= 15.0)
        unsafe = unsafe or (monotonic()-self.last_odom > 0.5) or any(monotonic()-t > 0.5 for t in self.heartbeats.values())
        if unsafe: msg=String(); msg.data='EMERGENCY_STOP'; self.pub.publish(msg)
def main(args=None):
    if rclpy is None: raise RuntimeError('ROS 2 is required to run safety_geofence_node')
    rclpy.init(args=args); n=SafetyGeofenceNode(); rclpy.spin(n); n.destroy_node(); rclpy.shutdown()
