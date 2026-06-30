import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class LidarAvoider(Node):
    def __init__(self):
        super().__init__('lidar_avoider')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.safe_distance = 0.35
        self.get_logger().info('=== LiDAR Obstacle Avoider ACTIVE ===')

    def scan_callback(self, msg):
        ranges = [r for r in msg.ranges if not math.isinf(r) and not math.isnan(r)]
        if not ranges:
            return
        total = len(msg.ranges)
        front_indices = list(range(0, total//6)) + list(range(5*total//6, total))
        left_indices  = list(range(total//6, total//3))
        right_indices = list(range(2*total//3, 5*total//6))

        def min_range(indices):
            vals = [msg.ranges[i] for i in indices
                    if not math.isinf(msg.ranges[i]) and not math.isnan(msg.ranges[i])]
            return min(vals) if vals else 999.0

        front = min_range(front_indices)
        left  = min_range(left_indices)
        right = min_range(right_indices)
        cmd = Twist()

        if front < self.safe_distance:
            self.get_logger().warn(
                f'OBSTACLE DETECTED FRONT: {front:.2f}m — TURNING AWAY!')
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5 if left > right else -0.5
        elif left < self.safe_distance:
            self.get_logger().info(f'Obstacle LEFT: {left:.2f}m — adjusting right')
            cmd.linear.x = 0.1
            cmd.angular.z = -0.3
        elif right < self.safe_distance:
            self.get_logger().info(f'Obstacle RIGHT: {right:.2f}m — adjusting left')
            cmd.linear.x = 0.1
            cmd.angular.z = 0.3
        else:
            self.get_logger().info(
                f'Path clear — F:{front:.2f}m L:{left:.2f}m R:{right:.2f}m')
            cmd.linear.x = 0.15
            cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = LidarAvoider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
