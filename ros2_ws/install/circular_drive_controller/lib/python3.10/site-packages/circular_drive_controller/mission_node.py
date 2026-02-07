#!/usr/bin/env python3

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
        # Simulation/Path parameters
        self.radius = 8.0           # Radius
        self.waypoint_count = 72    # More points for smoother circular motion
        self.arrival_dist = 0.6     # High Precision: Force robot to get close

        # State variables
        self.origin_latitude = None
        self.origin_longitude = None
        self.current_latitude = None
        self.current_longitude = None
        
        self.waypoints_enu = []     # List of (East, North) tuples
        self.wp_index = 0           # Current target index
        self.path_generated = False

        # GNSS subscriber
        self.create_subscription(
            NavSatFix, 
            '/gnss/fix', 
            self.gnss_callback, 
            10
        )

        # Target waypoint publisher
        self.publisher_ = self.create_publisher(Point, '/target_waypoint', 10)
        
        # Visualization publisher (Added to match functionality)
        self.viz_publisher_ = self.create_publisher(MarkerArray, '/waypoint_markers', 10)

        # Timer for control loop (10Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info("🔵 Mission Node Started: Waiting for GPS Fix...")

    def gnss_callback(self, msg):
        self.current_latitude = msg.latitude
        self.current_longitude = msg.longitude

        # Set Origin on First Fix
        if self.origin_latitude is None:
            self.origin_latitude = msg.latitude
            self.origin_longitude = msg.longitude
            self.get_logger().info(f"✅ Origin Set: {self.origin_latitude}, {self.origin_longitude}")
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

    def timer_callback(self):
        if not self.path_generated or self.current_latitude is None:
            return

        # 1. Calculate Current Position in Meters (Manual ENU conversion)
        R_EARTH = 6371000.0
        d_lat = math.radians(self.current_latitude - self.origin_latitude)
        d_lon = math.radians(self.current_longitude - self.origin_longitude)
        lat0 = math.radians(self.origin_latitude)

        curr_north = d_lat * R_EARTH
        curr_east = d_lon * R_EARTH * math.cos(lat0)

        # 2. Check Distance to Current Target
        tgt_east, tgt_north = self.waypoints_enu[self.wp_index]
        dist = math.sqrt((tgt_east - curr_east)**2 + (tgt_north - curr_north)**2)

        # 3. Switch Logic (Stricter now!)
        # Only switch if we are strictly within 0.6m of the dot.
        if dist < self.arrival_dist:
            old_index = self.wp_index
            self.wp_index = (self.wp_index + 1) % len(self.waypoints_enu)
            self.get_logger().info(f"📍 Touched WP#{old_index}. Moving to #{self.wp_index}")

        # 4. Prepare Message
        target_pt = self.waypoints_enu[self.wp_index]
        
        msg = Point()
        msg.x = float(target_pt[0]) # East
        msg.y = float(target_pt[1]) # North
        msg.z = 0.0

        # Publish
        self.publisher_.publish(msg)

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
        self.viz_publisher_.publish(arr)

def main():
    rclpy.init()
    node = MissionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()