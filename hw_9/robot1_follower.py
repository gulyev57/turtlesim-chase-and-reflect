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
        self.get_logger().info('Node has started')
        
        with open('config.json') as f:
            data = json.load(f)
        
        self.robot_config = {}
        self.robot_config.update(data["robot_parameters"])

        self.leader_pose = Pose()
        self.pose = Pose()
        self.publisher_ = None
        self.subscription = None
        self.timer = None

        self.leader_subscription = self.create_subscription(Pose, '/turtle1/pose', self.leader_callback, 10)
        self.angle_pid = PIDController(Kp=6.0, Ki = 0.0, Kd = 0.5)
        self.dist_pid = PIDController(Kp=2.0, Ki = 0.0, Kd = 0.2)
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
        distance = math.sqrt( dx**2 + dy**2) - self.robot_config["follow_distance"]
        target_angle = math.atan2(dy, dx)
        heading_error = self.normalize_angle(target_angle - self.pose.theta)

        angle_correction = self.angle_pid.compute(heading_error, 0.1)
        angle_correction = max(min(angle_correction, 2.0), -2.0)
        speed = self.dist_pid.compute(distance, 0.1)

        twist = Twist()
        twist.linear.x = min(speed, 2.0)
        twist.angular.z = angle_correction
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

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0.0
        self.integral = 0.0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative


def main(args=None):
    rclpy.init(args=args)
    node = MinimalNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
