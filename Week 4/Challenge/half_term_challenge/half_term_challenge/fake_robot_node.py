import rclpy
from rclpy.node import Node

import numpy as np

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, TransformStamped
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster


class SquareRobot(Node):

    def __init__(self):
        super().__init__('square_robot')

        # Estado
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Ganancias
        self.Kv = 0.8
        self.Kw = 2.0

        self.dt = 0.02

        # Trayectoria cuadrada
        self.goals = [
            (2.0, 0.0),
            (2.0, 2.0),
            (0.0, 2.0),
            (0.0, 0.0)
        ]

        self.goal_index = 0

        # Publicadores
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.path_pub = self.create_publisher(Path, '/path', 10)
        self.marker_pub = self.create_publisher(Marker, '/error_marker', 10)
        self.goal_pub = self.create_publisher(Marker, '/goal_marker', 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.path = Path()
        self.path.header.frame_id = 'odom'

        self.timer = self.create_timer(self.dt, self.update)

        self.get_logger().info("Seguimiento cuadrado iniciado")

   
    def wrapToPi(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

    
    def update(self):

        gx, gy = self.goals[self.goal_index]

        # Error posición
        ex = gx - self.x
        ey = gy - self.y

        ed = np.sqrt(ex**2 + ey**2)

        # Ángulo deseado
        theta_d = np.arctan2(ey, ex)

        # Error angular
        et = self.wrapToPi(theta_d - self.theta)

        # Cambio de waypoint
        if ed < 0.1:
            self.goal_index = (self.goal_index + 1) % len(self.goals)

        # Control proporcional
        V = self.Kv * ed
        W = self.Kw * et

        # Saturación
        V = np.clip(V, -0.5, 0.5)
        W = np.clip(W, -1.5, 1.5)

        # Cinemática
        self.theta += W * self.dt
        self.x += V * np.cos(self.theta) * self.dt
        self.y += V * np.sin(self.theta) * self.dt

        now = self.get_clock().now().to_msg()

        
        # ODOM
        
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y

        odom.pose.pose.orientation.z = np.sin(self.theta/2)
        odom.pose.pose.orientation.w = np.cos(self.theta/2)

        odom.twist.twist.linear.x = float(V)
        odom.twist.twist.angular.z = float(W)

        self.odom_pub.publish(odom)

        
        # TF
        
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = np.sin(self.theta/2)
        t.transform.rotation.w = np.cos(self.theta/2)

        self.tf_broadcaster.sendTransform(t)

       
        # PATH
        
        pose = PoseStamped()
        pose.header.stamp = now
        pose.header.frame_id = 'odom'
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y

        self.path.poses.append(pose)

        if len(self.path.poses) > 1000:
            self.path.poses.pop(0)

        self.path_pub.publish(self.path)

        
        # Error Arrow
        
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = now
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        marker.scale.x = 0.05
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        marker.color.r = 1.0
        marker.color.a = 1.0

        p1 = odom.pose.pose.position
        p2 = odom.pose.pose.position
        p2.x = gx
        p2.y = gy

        marker.points = [p1, p2]

        self.marker_pub.publish(marker)

        
        # Goal Marker
        
        g = Marker()
        g.header.frame_id = 'odom'
        g.header.stamp = now
        g.type = Marker.SPHERE
        g.action = Marker.ADD

        g.pose.position.x = gx
        g.pose.position.y = gy

        g.scale.x = 0.2
        g.scale.y = 0.2
        g.scale.z = 0.2

        g.color.g = 1.0
        g.color.a = 1.0

        self.goal_pub.publish(g)

        # Logs
        self.get_logger().info(
            f"Goal:{self.goal_index}  ed:{ed:.2f}  et:{et:.2f}"
        )


def main():
    rclpy.init()
    node = SquareRobot()
    rclpy.spin(node)
    rclpy.shutdown()