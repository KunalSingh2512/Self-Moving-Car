#!/usr/bin/env python3
"""
Mission Node: Large Tangent Circle with Precision Arrival
- Radius increased to 8.0m.
- Tolerance reduced to 0.5m to force the car to 'touch' the loop-closing point.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import NavSatFix
from visualization_msgs.msg import Marker, MarkerArray
import math

class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_node')

        # ===== CONFIGURATION =====
        self.radius = 8.0           # Increased Radius
        self.waypoint_count = 72    # More points for a smoother large circle
        self.arrival_dist = 0.6     # High Precision: Force robot to get close (touch the point)
        
        # ===== STATE =====
        self.origin_lat = None
        self.origin_lon = None
        self.current_lat = None
        self.current_lon = None
        
        self.waypoints_enu = []     # List of (East, North) tuples
        self.wp_index = 0           # Current target index
        self.path_generated = False

        # ===== I/O =====
        self.create_subscription(NavSatFix, '/gnss/fix', self.gnss_callback, 10)
        self.target_pub = self.create_publisher(Point, '/target_waypoint', 10)
        self.viz_pub = self.create_publisher(MarkerArray, '/waypoint_markers', 10)

        # Control Loop (10Hz)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("🔵 Mission Node Waiting for GPS Fix...")

    def gnss_callback(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

        # Set Origin on First Fix
        if self.origin_lat is None:
            self.origin_lat = msg.latitude
            self.origin_lon = msg.longitude
            self.get_logger().info(f"✅ Origin Set: {self.origin_lat}, {self.origin_lon}")
            self.generate_tangent_path()

    def generate_tangent_path(self):
        """
        Generates a circle starting at (0,0) facing East, turning Left.
        Center is at (0, Radius) in ENU.
        """
        self.waypoints_enu = []
        center_x = 0.0          # East offset
        center_y = self.radius  # North offset (Left of robot)

        for i in range(self.waypoint_count):
            # Start at -90 degrees (Bottom of circle) to match East heading
            angle = -math.pi/2 + (i * (2 * math.pi / self.waypoint_count))
            
            p_east = center_x + self.radius * math.cos(angle)
            p_north = center_y + self.radius * math.sin(angle)
            
            self.waypoints_enu.append((p_east, p_north))
            
        self.path_generated = True
        self.get_logger().info(f"🚀 Path Generated: {len(self.waypoints_enu)} points (Radius: {self.radius}m).")

    def control_loop(self):
        if not self.path_generated or self.current_lat is None:
            return

        # 1. Calculate Current Position in Meters
        R_EARTH = 6371000.0
        d_lat = math.radians(self.current_lat - self.origin_lat)
        d_lon = math.radians(self.current_lon - self.origin_lon)
        lat0 = math.radians(self.origin_lat)

        curr_north = d_lat * R_EARTH
        curr_east = d_lon * R_EARTH * math.cos(lat0)

        # 2. Check Distance to Current Target
        tgt_east, tgt_north = self.waypoints_enu[self.wp_index]
        dist = math.sqrt((tgt_east - curr_east)**2 + (tgt_north - curr_north)**2)

        # 3. Switch Logic (Stricter now!)
        # Only switch if we are strictly within 0.6m of the dot.
        # This ensures we don't "cut corners" and actually touch the return point.
        if dist < self.arrival_dist:
            old_index = self.wp_index
            self.wp_index = (self.wp_index + 1) % len(self.waypoints_enu)
            self.get_logger().info(f"📍 Touched WP#{old_index}. Moving to #{self.wp_index}")

        # 4. Publish Target for Control Node
        target_pt = self.waypoints_enu[self.wp_index]
        
        msg = Point()
        msg.x = float(target_pt[0]) # East
        msg.y = float(target_pt[1]) # North
        self.target_pub.publish(msg)

        # 5. Visualize
        self.publish_markers()

    def publish_markers(self):
        arr = MarkerArray()
        for i, (e, n) in enumerate(self.waypoints_enu):
            m = Marker()
            m.header.frame_id = "map"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.scale.x = 0.5; m.scale.y = 0.5; m.scale.z = 0.5
            
            # Current Target is RED (Chase this!), others GREEN
            if i == self.wp_index:
                m.color.r = 1.0; m.color.a = 1.0
                m.scale.x = 0.8; m.scale.y = 0.8
            else:
                m.color.g = 1.0; m.color.a = 1.0
                
            m.pose.position.x = e
            m.pose.position.y = n
            arr.markers.append(m)
        self.viz_pub.publish(arr)

def main():
    rclpy.init()
    node = MissionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()