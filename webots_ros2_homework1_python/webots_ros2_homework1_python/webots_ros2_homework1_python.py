import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from apriltag_msgs.msg import AprilTagDetectionArray
import math
import time

class TurtleBotController(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.cmd = Twist()

        # Subscribe to odometry to track robot's position
        self.create_subscription(Odometry, '/odom', self.update_odometry, 10)
        self.create_subscription(AprilTagDetectionArray, '/detections', self.apriltag_callback, 10)
        self.detected_tags = set()

    def update_odometry(self, msg):
        self.odometry_ready = True

    def apriltag_callback(self, msg):
        for detection in msg.detections:
            tag_id = detection.id[0]
            if tag_id not in self.detected_tags:
                self.detected_tags.add(tag_id)
                self.get_logger().info(f"Tag detected: ID={tag_id}")

    # Move straight for a specific distance
    def move_straight(self, speed, target_distance):
        self.cmd.linear.x = speed
        self.cmd.angular.z = 0.0
        duration = target_distance / speed
        start_time = time.time()

        while time.time() - start_time < duration:
            self.publisher_.publish(self.cmd)
            time.sleep(0.1)

        # Stop after reaching target distance
        self.cmd.linear.x = 0.0
        self.publisher_.publish(self.cmd)

    # Rotate the robot by a specific angle
    def rotate(self, angular_speed, target_angle):
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = angular_speed

        duration = abs(target_angle / angular_speed)
        start_time = time.time()

        while time.time() - start_time < duration:
            self.publisher_.publish(self.cmd)
            time.sleep(0.1)


        # Stop rotation
        self.cmd.angular.z = 0.0
        self.publisher_.publish(self.cmd)

    def search_environment(self):
        step_distance = 1.0
        for i in range(4):
            self.move_straight(0.1, step_distance)
            self.rotate(0.3, math.radians(90))
            step_distance += 0.5

    def shutdown(self):
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publisher_.publish(self.cmd)
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    controller_node = TurtleBotController()

    # Wait for odometry to be ready before performing movements
    while not controller_node.odometry_ready:
        rclpy.spin_once(controller_node, timeout_sec=1)

    try:
        controller_node.search_environment()
    except KeyboardInterrupt:
        pass
    finally:
        controller_node.shutdown()

if __name__ == '__main__':
    main()
