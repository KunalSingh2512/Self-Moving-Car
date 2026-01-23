#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_node')
        # Publisher: Sends X,Y coordinates
        self.publisher_ = self.create_publisher(Point, '/target_waypoint', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.get_logger().info('Mission Node Started: Waiting to send points...')

    def timer_callback(self):
        msg = Point()
        
        # ==========================================
        # TODO: SHUKSHAM'S AREA (EDIT BELOW)
        # ==========================================
        # Instructions:
        # 1. Write logic to calculate the next point in the circle.
        # 2. Or create a list of points and cycle through them.
        
        # Example Static Point (Replace this logic):
        msg.x = 1.0 
        msg.y = 0.0
        msg.z = 0.0
        
        # ==========================================
        # END OF SHUKSHAM'S AREA
        # ==========================================
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published Waypoint: x={msg.x}, y={msg.y}')

def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()