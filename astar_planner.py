import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
import heapq
import math

class AStarPlanner(Node):
    def __init__(self):
        super().__init__('astar_planner')
        self.waypoints = [
            (1.0, 0.0),
            (1.0, 1.0),
            (0.0, 1.0),
            (0.0, 0.0),
        ]
        self.current_wp = 0
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.timer = self.create_timer(3.0, self.navigate)
        self.get_logger().info('=== A* Path Planner ACTIVE ===')
        self.get_logger().info(f'Total waypoints: {len(self.waypoints)}')

    def heuristic(self, a, b):
        return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

    def astar(self, start, goal, grid_size=20):
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.reverse()
                return path
            for dx, dy in [(0,1),(1,0),(0,-1),(-1,0),(1,1),(-1,1),(1,-1),(-1,-1)]:
                neighbor = (current[0]+dx, current[1]+dy)
                if not (0 <= neighbor[0] < grid_size and 0 <= neighbor[1] < grid_size):
                    continue
                tentative_g = g_score[current] + self.heuristic(current, neighbor)
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        return []

    def navigate(self):
        if self.current_wp >= len(self.waypoints):
            self.get_logger().info('✓ All waypoints completed!')
            return
        target = self.waypoints[self.current_wp]
        start = (0, 0)
        goal  = (int(target[0]*10), int(target[1]*10))
        path  = self.astar(start, goal)
        self.get_logger().info('─────────────────────────────')
        self.get_logger().info(
            f'A* Computing path to waypoint {self.current_wp+1}/{len(self.waypoints)}')
        self.get_logger().info(f'  Start : (0.0, 0.0)')
        self.get_logger().info(f'  Goal  : ({target[0]}, {target[1]})')
        self.get_logger().info(f'  Path steps computed: {len(path)}')
        self.get_logger().info(
            f'  Heuristic distance: {self.heuristic(start,goal):.2f}')
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = target[0]
        goal_msg.pose.position.y = target[1]
        self.goal_pub.publish(goal_msg)
        self.get_logger().info(f'  Goal published to /goal_pose ✓')
        self.current_wp += 1

def main(args=None):
    rclpy.init(args=args)
    node = AStarPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
