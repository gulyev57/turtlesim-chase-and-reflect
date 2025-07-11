import math
import random
from re import sub
import rclpy
from rclpy import service
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn
import json

class MinimalNode(Node):
    def __init__(self):
        super().__init__('robot1_node')

        with open('config.json') as f:
            data = json.load(f)
        
        self.robot_config = {}
        self.robot_config.update(data["robot_parameters"])


        self.get_logger().info('Node has started')
        self.leader_pose = Pose()
        self.pose = Pose()
        self.publisher_ = None
        self.subscription = None
        self.timer = None

        self.leader_subscription = self.create_subscription(Pose, '/turtle1/pose', self.leader_callback, 10)
        self.client = self.create_client(Spawn, '/spawn')
        while not self.client.wait_for_service(timeout_sec= 0.5):
            self.get_logger().info("Waiting for the service")
        self.spawn_follower_turtle()

    def spawn_follower_turtle(self):
        request = Spawn.Request()
        request.x = random.uniform(0.0,10.0)
        request.y = random.uniform(0.0,10.0)
        request.theta = 0.0
        request.name = 'turtle2'

        future = self.client.call_async(request)
        future.add_done_callback(self.on_spawn_complete)

    def on_spawn_complete(self, future):
        if future.result() is not None:
            self.get_logger().info("Turtle2 spawned succesfully")
            self.publisher_ = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
            self.subscription = self.create_subscription(Pose, '/turtle2/pose', self.listener_callback, 10)
            self.timer = self.create_timer(0.1, self.timer_callback)
        else:
            self.get_logger().error("Turtle2 was not spawned")
    def timer_callback(self):
        if ((self.leader_pose.x == 0 and self.leader_pose.y == 0 and self.leader_pose.theta == 0) or
                (self.pose.x == 0 and self.pose.y == 0 and self.pose.theta == 0)):
            self.get_logger().warn("Waiting for initial pose...")
            return
        dx = self.leader_pose.x - self.pose.x
        dy = self.leader_pose.y - self.pose.y
        distance = math.sqrt( dx**2 + dy**2) - robot_config["follow_distance"]
        target_angle = math.atan2(dy, dx)
        heading_error = self.normalize_angle(target_angle - self.pose.theta)
        
        # --- Stanley Controller ---
        cross_track_error = (dy * math.cos(self.pose.theta) - dx * math.sin(self.pose.theta))
        k = 1.0  # gain
        velocity = 1.0
        steering_correction = math.atan2(k * cross_track_error, velocity)
        angular_z = heading_error + steering_correction
        linear_x = min(distance, 2.0)

        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.publisher_.publish(twist)

    def leader_callback(self, msg):
        self.leader_pose = msg
        self.get_logger().info(f"Leader Turtle coordinates :: x: {msg.x:.2f}, y: {msg.y:.2f}, theta: {msg.theta:.2f}")
    def listener_callback(self, msg):
        self.pose = msg
        self.get_logger().info(f"Follower Turtle coordinates :: x: {msg.x:.2f}, y: {msg.y:.2f}, theta: {msg.theta:.2f}")
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
