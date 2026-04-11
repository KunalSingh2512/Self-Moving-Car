import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from std_msgs.msg import String # We will replace this with Kunal's Custom YOLO Message later!

class LocalPlannerNode(Node):
    def __init__(self):
        super().__init__('local_planner')
        
        self.path_sub = self.create_subscription(Path, '/global_path', self.path_callback, 10)
        self.yolo_sub = self.create_subscription(String, '/yolo_obstacles', self.yolo_callback, 10)
        
        # Publishing Velocity to the wheels
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def yolo_callback(self, msg):
        # ==========================================
        # ⚠️ SALONI: WRITE DECISION LOGIC HERE ⚠️
        # Task: Implement trajectory logic based on YOLO labels.
        # Case 1 (Stone): Drive over it.
        # Case 2 (Big Object): Generate bypass trajectory.
        # Case 3 (Pedestrian/Child): Execute Fail-safe Stop.
        # ==========================================
        
        # Example of stopping the car:
        # stop_msg = Twist()
        # stop_msg.linear.x = 0.0
        # stop_msg.angular.z = 0.0
        # self.cmd_pub.publish(stop_msg)
        pass

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