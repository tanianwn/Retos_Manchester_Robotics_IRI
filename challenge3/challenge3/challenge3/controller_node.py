import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from challenge3_interfaces.msg import GoalList
import numpy as np

class PID:
    """Standard PID Controller implementation."""
    def __init__(self, kp, ki, kd, max_out, min_out):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_out = max_out
        self.min_out = min_out
        
        self.error_sum = 0.0
        self.last_error = 0.0
        self.last_time = None

    def compute(self, error, dt):
        if dt <= 0.0:
            return 0.0
        
        # Integral with basic anti-windup (clamping)
        self.error_sum += error * dt
        
        # Derivative
        d_error = (error - self.last_error) / dt
        self.last_error = error
        
        output = (self.kp * error) + (self.ki * self.error_sum) + (self.kd * d_error)
        
        # Apply output constraints
        abs_output = np.clip(abs(output), self.min_out, self.max_out)
        return np.sign(output) * abs_output

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(GoalList, '/goals', self.goal_cb, 10)

        # PID Parameters - Linear
        # Tune these: kp (speed), ki (close steady-state gap), kd (reduce overshoot)
        self.linear_pid = PID(kp=0.8, ki=0.01, kd=0.05, max_out=0.2, min_out=0.009)
        
        # Angular Gain
        self.kp_a = 1.0  
        self.max_w, self.min_w = 2, 0.1
        
        self.current_goal = None
        self.pose = [0.0, 0.0, 0.0] 
        self.stop_forever = False
        self.last_time = self.get_clock().now()

    def wrap_to_pi(self, angle):
        """Wraps the given angle to the range [-pi, pi]."""
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def odom_cb(self, msg):
        # Update Pose
        self.pose[0] = msg.pose.pose.position.x
        self.pose[1] = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.pose[2] = np.arctan2(siny_cosp, cosy_cosp)
        
        self.control()

    def goal_cb(self, msg):
        new_goal = msg.current_goal.position
        if self.current_goal is None or (self.current_goal.x != new_goal.x or self.current_goal.y != new_goal.y):
            self.current_goal = new_goal
            # Reset PID integral when a new goal is received
            self.linear_pid.error_sum = 0.0
        self.stop_forever = msg.final_goal

    def control(self):
        if self.current_goal is None:
            return

        # Time delta for PID
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        dx = self.current_goal.x - self.pose[0]
        dy = self.current_goal.y - self.pose[1]
        
        rho = np.sqrt(dx**2 + dy**2) 
        
        # Use wrap_to_pi to calculate the shortest angular distance
        angle_to_goal = np.arctan2(dy, dx)
        alpha = self.wrap_to_pi(angle_to_goal - self.pose[2])

        # Final goal: 1.5cm | Intermediate: 4cm
        limit = 0.03 if self.stop_forever else 0.05

        if rho < limit:
            self.stop_robot()
            if self.stop_forever:
                self.current_goal = None
            return

        cmd = Twist()
        
        # Phase A: Rotate to face goal if the error is large
        if abs(alpha) > 0.1:
            v_cmd = 0.0
            w_cmd = np.sign(alpha) * max(abs(self.kp_a * alpha), self.min_w)
        else:
            # Phase B: Drive and Correct heading
            # Using the PID controller for linear velocity
            v_cmd = self.linear_pid.compute(rho, dt)
            
            # Simple P control for heading during movement
            if abs(alpha) > 0.02:
                w_cmd = np.sign(alpha) * max(abs(self.kp_a * alpha), self.min_w)
            else:
                w_cmd = 0.0

        cmd.linear.x = float(v_cmd) # PID class already handles clipping
        cmd.angular.z = float(np.clip(w_cmd, -self.max_w, self.max_w))
        self.vel_pub.publish(cmd)

    def stop_robot(self):
        self.vel_pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
