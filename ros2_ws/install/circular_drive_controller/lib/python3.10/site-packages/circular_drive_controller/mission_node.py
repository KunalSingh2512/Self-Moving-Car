#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import NavSatFix

class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_node')
        
        # The GNSS sensor is mounted 12cm behind the robot center.
        self.gnss_offset_x = -0.12 

        self.subscription = self.create_subscription(
            NavSatFix,
            '/gnss/fix',
            self.gnss_callback,
            10
        )
        
        # Variables to store latest robot position
        self.current_latitude = 0.0
        self.current_longitude = 0.0
        self.gnss_received = False

        # Sends X,Y target coordinates to Abhay
        self.publisher_ = self.create_publisher(Point, '/target_waypoint', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)
        
        self.get_logger().info('Mission Node Started: Broadcasting Static Points (Waiting for GNSS Logic)...')
    
    def gnss_callback(self, msg):
        self.current_latitude = msg.latitude
        self.current_longitude = msg.longitude
        self.gnss_received = True

    def timer_callback(self):
        msg = Point()
        
        # ==========================================
        # STATIC DEFAULT 
        # ==========================================
        # We set a default point so Abhay's code ALWAYS has something to drive to.
        # Even if GNSS fails, the car will try to go 2 meters forward.
        msg.x = 2.0 
        msg.y = 0.0
        msg.z = 0.0

        # ==========================================
        # 2. SHUKSHAM'S AREA
        # ==========================================
        # Instructions for Shuksham:
        # The code above sets a static target (2.0 meters).
        # Use the block below to OVERWRITE msg.x and msg.y based on GNSS data.
        
        if self.gnss_received:
            self.get_logger().info(f'GNSS Fix: Lat={self.current_latitude}, Lon={self.current_longitude}')
            
            # WRITE LOGIC HERE
            # Example: 
            # if self.current_latitude > 28.5:
            #     msg.x = 5.0  (Change target to 5 meters)
            #     msg.y = 1.0  (Change target to 1 meter left)
            pass 

        else:
            self.get_logger().warning('No GNSS Fix yet. Sending Static Default (2.0m)...')
        
        # ==========================================
        # END OF SHUKSHAM'S AREA
        # ==========================================
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published Target: x={msg.x}, y={msg.y}')

def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()