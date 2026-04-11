import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from my_car_interfaces.msg import ObstacleInfo

class LocalPlannerNode(Node):
    def __init__(self):
        super().__init__('local_planner')
        
        self.path_sub = self.create_subscription(Path, '/global_path', self.path_callback, 10)
        self.yolo_sub = self.create_subscription(ObstacleInfo, '/yolo_obstacles', self.yolo_callback, 10)
        
        # Publishing Velocity to the wheels
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def yolo_callback(self, msg):
        # Now Saloni can access specific variables!
        detected_object = msg.object_label
        distance_to_object = msg.distance
        is_it_passable = msg.is_passable
        
        # ==========================================
        # ⚠️ SALONI: WRITE DECISION LOGIC HERE ⚠️
        # ==========================================

    def path_callback(self, msg):
        # Follow the path when there are no obstacles
        pass

def main(args=None):
    rclpy.init(args=args)
    node = LocalPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()