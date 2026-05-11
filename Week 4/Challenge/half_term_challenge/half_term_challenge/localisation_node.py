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
        
        self._l = 0.18  
        self._r = 0.05  
        self.X, self.Y, self.Th = 0.0, 0.0, 0.0
        self.wr, self.wl = 0.0, 0.0
        self.last_time = self.get_clock().now()

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)

        self.create_subscription(Float32, 'VelocityEncR', self.wr_cb, qos)
        self.create_subscription(Float32, 'VelocityEncL', self.wl_cb, qos)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # Timer a 50Hz
        self.create_timer(0.02, self.update)
        self.get_logger().info("Localización iniciada (Modo eficiente).")

    def wr_cb(self, msg): self.wr = msg.data
    def wl_cb(self, msg): self.wl = msg.data

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0: return
        self.last_time = now

        v = self._r * (self.wr + self.wl) / 2.0
        w = self._r * (self.wr - self.wl) / self._l

        self.Th += w * dt
        self.X += v * np.cos(self.Th) * dt
        self.Y += v * np.sin(self.Th) * dt

        self.publish_odom(now, v, w)

    def publish_odom(self, now, v, w):
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.X
        odom.pose.pose.position.y = self.Y
        
        q = transforms3d.euler.euler2quat(0, 0, self.Th)
        odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, \
        odom.pose.pose.orientation.y, odom.pose.pose.orientation.z = q

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = LocalisationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
