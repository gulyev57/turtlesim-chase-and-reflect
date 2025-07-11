import rclpy
from rclpy.node import Node, get_logger
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import random
import math
import json

class MinimalNode(Node):
    def __init__(self):
        super().__init__('robot2_node')
        self.get_logger().info('Node has started')


        with open('config.json') as f:
            data = json.load(f)

        self.robot_config = {}
        self.robot_config.update(data["robot_parameters"])


        self.pose = Pose()
        self.current_theta = 0.0  # in __init__
        self.turning = False      # are we turning to align?
        self.target_theta = None  # target direction
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.listener_callback, 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        twist = Twist()

        if self.pose.x == 0.0 and self.pose.y == 0.0 and self.pose.theta == 0.0:
            self.get_logger().warn("Waiting for initial pose...")
            return

        if self.turning and self.target_theta is not None:
            angle_diff = self.normalize_angle(self.target_theta - self.current_theta)

            if abs(angle_diff) < 0.1:
                self.turning = False
                self.target_theta = None
                twist.linear.x = 2.0
                twist.angular.z = 0.0
                self.get_logger().info("Finished turning. Moving straight.")
            else:
                twist.linear.x = 0.0
                twist.angular.z = 2.0 * angle_diff
                self.get_logger().info(f"Turning to align. Δθ = {angle_diff:.2f}")

        else:
            wall = self.is_at_wall()
            if wall:
                if random.choice([True, False]):
                    self.reflect(wall)
                else:
                    self.random_bounce(wall)
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            else:
                twist.linear.x = 2.0
                twist.angular.z = 0.0
        twist.linear.x = self.robot_config["linear_speed"]
        twist.angular.z = self.robot_config["angular_speed"]
        self.publisher_.publish(twist)

    def reflect(self, wall):
        if wall in ['left', 'right']:
            reflected_theta = (math.pi - self.current_theta) % (2 * math.pi)
        elif wall in ['top', 'bottom']:
            reflected_theta = (-self.current_theta) % (2 * math.pi)
        else:
            return

        self.target_theta = reflected_theta
        self.turning = True
        self.get_logger().info(f"Reflecting off {wall}! Target angle: {reflected_theta:.2f}")

    def random_bounce(self, wall):
        if wall == 'top':
            new_theta = random.uniform(-3 * math.pi / 4, -math.pi / 4)
        elif wall == 'bottom':
            new_theta = random.uniform(math.pi / 4, 3 * math.pi / 4)
        elif wall == 'right':
            new_theta = random.uniform(3 * math.pi / 4, 5 * math.pi / 4)
        elif wall == 'left':
            new_theta = random.uniform(-math.pi / 4, math.pi / 4)
        else:
            new_theta = random.uniform(0, 2 * math.pi)

        self.target_theta = new_theta % (2 * math.pi)
        self.turning = True
        self.get_logger().info(f"Random bounce from {wall} wall! Target angle: {self.target_theta:.2f}")

    def listener_callback(self, msg):
        self.pose = msg
        self.current_theta = msg.theta
        self.get_logger().info(f"x: {msg.x:.2f}, y: {msg.y:.2f}, theta: {msg.theta:.2f}")

    def is_at_wall(self):
        if self.pose.x <= 0.5:
            return 'left'
        elif self.pose.x >= 10.5:
            return 'right'
        elif self.pose.y <= 0.5:
            return 'bottom'
        elif self.pose.y >= 10.5:
            return 'top'
        else:
            return None

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle



def main(args=None):
    rclpy.init(args=args)
    node = MinimalNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
