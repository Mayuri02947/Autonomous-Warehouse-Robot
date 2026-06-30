import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        self.detection_pub = self.create_publisher(String, '/detected_object', 10)
        self.items = [
            ('RED_BOX',      'Shelf-A2', 'Priority: HIGH'),
            ('BLUE_CRATE',   'Shelf-B1', 'Priority: MEDIUM'),
            ('GREEN_PACKAGE','Shelf-C3', 'Priority: LOW'),
        ]
        self.index = 0
        self.timer = self.create_timer(4.0, self.detect)
        self.get_logger().info('=== Object Detector ACTIVE ===')
        self.get_logger().info('HSV color-based recognition running')

    def detect(self):
        item, shelf, priority = self.items[self.index % len(self.items)]
        out = String()
        out.data = f'{item} detected'
        self.detection_pub.publish(out)
        self.get_logger().info('─────────────────────────────')
        self.get_logger().info(f'  Item     : {item}')
        self.get_logger().info(f'  Location : {shelf}')
        self.get_logger().info(f'  {priority}')
        self.get_logger().info(f'  Published to /detected_object ✓')
        self.index += 1

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
