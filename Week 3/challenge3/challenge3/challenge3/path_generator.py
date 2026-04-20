import rclpy
from rclpy.node import Node
from challenge3_interfaces.msg import GoalList
from nav_msgs.msg import Odometry
import numpy as np

class PathGenerator(Node):
    def __init__(self):
        super().__init__('path_generator')
        self.goal_pub = self.create_publisher(GoalList, '/goals', 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        
        # TRAJECTORY
        self.targets = [[2.0, 0.0],[2.0, 2.0],[0.0, 2.0],[0.0,0.0]]
        self.current_idx = 0
        self.mission_finished = False

    def odom_cb(self, odom):
        curr_x = odom.pose.pose.position.x
        curr_y = odom.pose.pose.position.y
        target = self.targets[self.current_idx]
        
        dist = np.sqrt((target[0] - curr_x)**2 + (target[1] - curr_y)**2)
        
        # LOWERED THRESHOLD: 5cm for intermediate points
        if dist < 0.05 and not self.mission_finished: 
            if self.current_idx < len(self.targets) - 1:
                self.current_idx += 1
                self.get_logger().info(f"Intermediate Goal Reached. Moving to Target {self.current_idx}")
            else:
                self.mission_finished = True
                self.get_logger().info("All targets sent. Waiting for Final Goal precision stop.")

        # Publish custom GoalList message
        msg = GoalList()
        msg.current_goal.position.x = float(self.targets[self.current_idx][0])
        msg.current_goal.position.y = float(self.targets[self.current_idx][1])
        msg.is_reachable = True 
        msg.final_goal = self.mission_finished
        self.goal_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(PathGenerator())
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
