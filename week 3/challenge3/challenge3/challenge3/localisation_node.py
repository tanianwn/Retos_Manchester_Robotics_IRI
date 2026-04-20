import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
import transforms3d
import numpy as np

class LocalisationNode(Node):
    def __init__(self):
        super().__init__('localisation_node')
        
        # Physical Constants
        self._l = 0.18  # Wheelbase
        self._r = 0.05  # Wheel radius
        
        # State Variables
        self.X, self.Y, self.Th = 0.0, 0.0, 0.0
        self.wr, self.wl = 0.0, 0.0
        self.last_time = self.get_clock().now()

        # QoS for sensor compatibility
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)

        # Subscriptions & Publishers
        self.create_subscription(Float32, 'VelocityEncR', self.wr_cb, qos)
        self.create_subscription(Float32, 'VelocityEncL', self.wl_cb, qos)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        self.create_timer(0.02, self.update) # 50Hz
        self.get_logger().info("Localisation Node Started with Best Effort QoS.")

    def wr_cb(self, msg): self.wr = msg.data
    def wl_cb(self, msg): self.wl = msg.data

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0: return
        self.last_time = now

        v = self._r * (self.wr + self.wl) / 2.0
        w = self._r * (self.wr - self.wl) / self._l

        # Euler Integration
        self.Th += w * dt
        self.X += v * np.cos(self.Th) * dt
        self.Y += v * np.sin(self.Th) * dt

        # Publish Odometry
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.X
        odom.pose.pose.position.y = self.Y
        
        q = transforms3d.euler.euler2quat(0, 0, self.Th)
        odom.pose.pose.orientation.w = q[0]
        odom.pose.pose.orientation.x = q[1]
        odom.pose.pose.orientation.y = q[2]
        odom.pose.pose.orientation.z = q[3]
        
        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(LocalisationNode())
    rclpy.shutdown()
    
if __name__ == '__main__':
   main()
