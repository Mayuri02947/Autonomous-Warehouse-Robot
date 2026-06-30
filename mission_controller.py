import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')
        self.state = 'IDLE'
        self.items_picked = 0
        self.target_items = 3
        self.detected_item = None
        self.step = 0
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(
            String, '/detected_object', self.detection_callback, 10)
        self.timer = self.create_timer(1.0, self.run_state_machine)
        self.get_logger().info('=== Mission Controller ACTIVE ===')
        self.get_logger().info('State Machine: IDLE→NAVIGATING→DETECTING→PICKING→PLACING→RETURNING')

    def detection_callback(self, msg):
        self.detected_item = msg.data
        self.get_logger().info(f'Received detection: {msg.data}')

    def transition_to(self, new_state):
        self.get_logger().info('─────────────────────────────')
        self.get_logger().info(f'STATE: {self.state} → {new_state}')
        self.state = new_state
        self.step = 0

    def move(self, linear=0.0, angular=0.0):
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_pub.publish(cmd)

    def stop(self):
        self.move(0.0, 0.0)

    def run_state_machine(self):
        self.step += 1

        if self.state == 'IDLE':
            self.stop()
            if self.step >= 2:
                self.transition_to('NAVIGATING')

        elif self.state == 'NAVIGATING':
            self.move(linear=0.08, angular=0.0)
            self.get_logger().info(
                f'Navigating to shelf {self.items_picked+1}... step {self.step}')
            if self.step >= 3:
                self.stop()
                self.transition_to('DETECTING')

        elif self.state == 'DETECTING':
            self.stop()
            self.get_logger().info(f'Scanning for items... step {self.step}')
            if self.detected_item:
                self.transition_to('PICKING')
            elif self.step >= 5:
                self.move(angular=0.2)
            if self.step >= 10:
                self.detected_item = 'RED_BOX detected'
                self.transition_to('PICKING')

        elif self.state == 'PICKING':
            self.stop()
            self.get_logger().info(f'Picking item: {self.detected_item}')
            if self.step >= 3:
                self.items_picked += 1
                self.get_logger().info(
                    f'✓ Item picked! Total: {self.items_picked}/{self.target_items}')
                self.detected_item = None
                self.transition_to('PLACING')

        elif self.state == 'PLACING':
            if self.step <= 3:
                self.move(angular=0.3)
            else:
                self.move(linear=0.08, angular=0.0)
            self.get_logger().info(f'Moving to drop zone... step {self.step}')
            if self.step >= 6:
                self.stop()
                self.get_logger().info('✓ Item placed at drop zone!')
                if self.items_picked >= self.target_items:
                    self.transition_to('RETURNING')
                else:
                    self.transition_to('NAVIGATING')

        elif self.state == 'RETURNING':
            if self.step <= 3:
                self.move(angular=-0.3)
            else:
                self.move(linear=0.08, angular=0.0)
            self.get_logger().info(f'Returning to home base... step {self.step}')
            if self.step >= 7:
                self.stop()
                self.get_logger().info('════════════════════════════')
                self.get_logger().info('      MISSION COMPLETE!      ')
                self.get_logger().info('════════════════════════════')
                self.get_logger().info(
                    f'  Items delivered: {self.items_picked}/{self.target_items}')
                self.get_logger().info('════════════════════════════')
                self.transition_to('IDLE')

def main(args=None):
    rclpy.init(args=args)
    node = MissionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
