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

        # Sends X,Y target coordinates to the Abhay
        self.publisher_ = self.create_publisher(Point, '/target_waypoint', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)
        
        self.get_logger().info('Mission Node Started: Listening for GNSS & Waiting to send points...')
    
    # Update position whenever hardware sends data
    def gnss_callback(self, msg):
        self.current_latitude = msg.latitude
        self.current_longitude = msg.longitude
        self.gnss_received = True

    def timer_callback(self):
        msg = Point()
        
        # ==========================================
        # TODO: SHUKSHAM'S AREA (EDIT BELOW)
        # ==========================================
        # Instructions:
        # 1. You now have access to self.current_latitude and self.current_longitude
        # 2. Logic: "If I am at Latitute X, go to Point Y"
        
        if self.gnss_received:
            # Printing the live data for now
            self.get_logger().info(f'Current GPS: Lat={self.current_latitude}, Lon={self.current_longitude}')
            
            # NOTE: Remember the GNSS is physically at -0.12m (rear of chassis).

            # Example Logic (Replace this):
            msg.x = 1.0 
            msg.y = 0.0
            msg.z = 0.0
        else:
            self.get_logger().warning('Waiting for GNSS fix...')
        
        # ==========================================
        # END OF SHUKSHAM'S AREA
        # ==========================================
        
        self.publisher_.publish(msg)
        # Only log publish if we actually have data to avoid clutter
        if self.gnss_received:
            self.get_logger().info(f'Published Target: x={msg.x}, y={msg.y}')

def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()