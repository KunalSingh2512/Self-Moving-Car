import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import Point

class GlobalPlannerNode(Node):
    def __init__(self):
        super().__init__('global_planner')
        
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.target_sub = self.create_subscription(Point, '/target_waypoint', self.target_callback, 10)
        self.path_pub = self.create_publisher(Path, '/global_path', 10)

    def map_callback(self, msg):
        # Store the latest map for planning
        self.current_map = msg

    def target_callback(self, msg):
        # ==========================================
        # ⚠️ ABHAY: WRITE PATHFINDING ALGORITHM HERE ⚠️
        # Task: Find the shortest/best path between Point A (car location) 
        # and Point B (msg.x, msg.y) using the graph-based map.
        # ==========================================
        pass

def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()