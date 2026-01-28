#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import NavSatFix
from geopy.distance import geodesic
from pyproj import Proj
import math

# GNSS Waypoints
WAYPOINTS = [
    (28.6139, 77.2090),   # WP1
    (28.6145, 77.2102),   # WP2
    (28.6152, 77.2115),   # WP3
]

class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_node')

        # GNSS offset (12cm behind robot center)
        self.gnss_offset_x = -0.12  # Meters

        # Simulation parameters 
        self.declare_parameter('simulate_gnss', True)  # Enable simulation if no real GNSS
        self.declare_parameter('simulation_speed', 0.00001)  # Degrees per second (approx 1m/s)
        self.simulate_gnss = self.get_parameter('simulate_gnss').value
        self.simulation_speed = self.get_parameter('simulation_speed').value

        # State variables
        self.current_latitude = 28.6139  # Start near WP1
        self.current_longitude = 77.2090
        self.current_x = 0.0
        self.current_y = 0.0
        self.gnss_received = False
        self.last_gnss_time = self.get_clock().now()
        self.wp_index = 0

        # GNSS subscriber
        self.create_subscription(
            NavSatFix,
            '/gnss/fix',
            self.gnss_callback,
            10
        )



        # Target waypoint publisher
        self.publisher_ = self.create_publisher(Point, '/target_waypoint', 10)

        # Next waypoint publisher
        self.next_wp_pub = self.create_publisher(NavSatFix, '/next_waypoint', 10)

        # Timer for control and simulation
        self.timer = self.create_timer(0.5, self.timer_callback)

        # GPS → UTM conversion
        self.proj = Proj(proj="utm", zone=43, ellps="WGS84")

        self.get_logger().info('Mission Node Started: GNSS Waypoints + Simulation + Circular Motion')

    def gnss_callback(self, msg):
        self.current_latitude = msg.latitude
        self.current_longitude = msg.longitude
        self.gnss_received = True
        self.last_gnss_time = self.get_clock().now()

        # Convert to UTM with offset
        raw_x, raw_y = self.proj(self.current_longitude, self.current_latitude)
        self.current_x = raw_x + self.gnss_offset_x
        self.current_y = raw_y

    def simulate_gnss_update(self):
        # Simulate movement toward next waypoint
        wp_lat, wp_lon = WAYPOINTS[self.wp_index]
        delta_lat = wp_lat - self.current_latitude
        delta_lon = wp_lon - self.current_longitude
        dist = math.sqrt(delta_lat**2 + delta_lon**2)

        if dist > 0:
            # Move a small step toward waypoint
            step_lat = (delta_lat / dist) * self.simulation_speed * 0.5  # Scale for timer
            step_lon = (delta_lon / dist) * self.simulation_speed * 0.5
            self.current_latitude += step_lat
            self.current_longitude += step_lon

        # Update UTM
        raw_x, raw_y = self.proj(self.current_longitude, self.current_latitude)
        self.current_x = raw_x + self.gnss_offset_x
        self.current_y = raw_y

    def timer_callback(self):
        # Check if real GNSS is stale (simulate if needed)
        now = self.get_clock().now()
        if self.simulate_gnss and (now - self.last_gnss_time).nanoseconds > 5e9:  # 5 seconds
            self.simulate_gnss_update()
            self.gnss_received = True  # Treat as received for logic

        msg = Point()
        

        if not self.gnss_received:
            # Fallback: Stop
            msg.x = 0.0
            msg.y = 0.0
            msg.z = 0.0
    
            self.get_logger().warn("No GNSS: Publishing stop.")
        else:
            # Get current waypoint
            wp_lat, wp_lon = WAYPOINTS[self.wp_index]
            wp_x, wp_y = self.proj(wp_lon, wp_lat)

            # Relative target
            msg.x = wp_x - self.current_x
            msg.y = wp_y - self.current_y
            msg.z = 0.0

            # Distance and bearing
            dist = geodesic((self.current_latitude, self.current_longitude), (wp_lat, wp_lon)).meters
            bearing = math.atan2(msg.y, msg.x)

            # Log waypoints
            current_wp = WAYPOINTS[self.wp_index]
            next_wp = WAYPOINTS[(self.wp_index + 1) % len(WAYPOINTS)]
            self.get_logger().info(
                f"📍 Current WP{self.wp_index+1}: {current_wp} | ➡️ Next: {next_wp} | Dist: {dist:.2f}m"
            )

            # Publish next waypoint GPS
            next_msg = NavSatFix()
            next_msg.latitude = next_wp[0]
            next_msg.longitude = next_wp[1]
            next_msg.altitude = 0.0
            self.next_wp_pub.publish(next_msg)

            # Waypoint reached: Switch (loop infinitely)
            if dist < 2.0:
                self.get_logger().info(f"Waypoint {self.wp_index+1} REACHED ✅ | Looping to next.")
                self.wp_index = (self.wp_index + 1) % len(WAYPOINTS)

            

        # Publish
        self.publisher_.publish(msg)
        

def main():
    rclpy.init()
    node = MissionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()