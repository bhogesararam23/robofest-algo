try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry, Path
except ImportError:
    rclpy = None
class FCBridgeNode(Node):
    def __init__(self):
        super().__init__('fc_bridge'); self.command='HOLD'; self.pose=(0.,0.,0.); self.path=[]; self.index=0
        self.vel_pub=self.create_publisher(Twist,'/mavros/setpoint_velocity/cmd_vel',10)
        self.heartbeat=self.create_publisher(String,'/drivers/heartbeat',10)
        self.create_subscription(String,'/mission/current_command',self.command_cb,10); self.create_subscription(Path,'/navigation/safe_path',self.path_cb,10); self.create_subscription(Odometry,'/odom',self.odom_cb,10)
        self.create_timer(0.02,self.control_tick)
    def command_cb(self,msg): self.command=msg.data; self.index=0
    def path_cb(self,msg): self.path=msg.poses; self.index=0
    def odom_cb(self,msg): self.pose=(msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z)
    def control_tick(self):
        out=Twist(); x,y,z=self.pose
        if self.command in ('SEARCH','GUIDE') and self.path:
            target=self.path[min(self.index,len(self.path)-1)].pose.position; dx=target.x-x; dy=target.y-y; dz=target.z-z; dist=(dx*dx+dy*dy+dz*dz)**0.5
            if dist < 0.3 and self.index < len(self.path)-1: self.index += 1
            scale=min(1.0, 0.7/max(dist,0.01)); out.linear.x=dx*scale; out.linear.y=dy*scale; out.linear.z=dz*scale
        elif self.command == 'TAKEOFF': out.linear.z=0.6 if z < 1.5 else 0.0
        elif self.command == 'EMERGENCY_STOP': out=Twist()
        self.vel_pub.publish(out); h=String(); h.data='fc_bridge'; self.heartbeat.publish(h)
def main(args=None):
    if rclpy is None: raise RuntimeError('ROS 2 is required to run fc_bridge_node')
    rclpy.init(args=args); n=FCBridgeNode(); rclpy.spin(n); n.destroy_node(); rclpy.shutdown()
