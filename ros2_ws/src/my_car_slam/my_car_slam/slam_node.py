import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid

class SlamNode(Node):
    def __init__(self):
        super().__init__('slam_node')
        
        # Listening to LiDAR and Odometry
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Publishing the 2D Map
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)

    def scan_callback(self, msg):
        # ==========================================
        # ⚠️ AARUSH & SHUKSHAM: WRITE SLAM LOGIC HERE ⚠️
        # Task: Process the LiDAR scan data into a 2D map array.
        # ==========================================
        pass

    def odom_callback(self, msg):
        # ==========================================
        # ⚠️ AARUSH & SHUKSHAM: WRITE ODOMETRY LOGIC HERE ⚠️
        # Task: Track the car's movement to update the map accurately.
        # ==========================================
        pass

def main(args=None):
    rclpy.init(args=args)
    node = SlamNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()