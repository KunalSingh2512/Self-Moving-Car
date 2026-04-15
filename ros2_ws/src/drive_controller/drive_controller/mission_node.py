#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import NavSatFix
import math

class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_node')

        self.arrival_dist = 2.0 

        self.origin_latitude = None
        self.origin_longitude = None
        self.current_latitude = None
        self.current_longitude = None

        self.gps_waypoints = [
            (28.91928, 77.131156),
            (28.919290, 77.131115),
            (28.919285, 77.131104),
            (28.919285, 77.131085),
            (28.919285, 77.130897),
            (28.919293, 77.130750),
            (28.919293, 77.130650),
            (28.919298, 77.130550),
            (28.919298, 77.130450),
            (28.919308, 77.130350),
            (28.919308, 77.130250),
            (28.919310, 77.130150),
            (28.919315, 77.130050),
            (28.919315, 77.129995),
            (28.919315, 77.129895),
            (28.919395, 77.129795),
            (28.919315, 77.129795), # First loop ends here, reversing path
            (28.919315, 77.129895), 
            (28.919315, 77.129995),
            (28.919315, 77.130050),
            (28.919310, 77.130150),
            (28.919308, 77.130250),
            (28.919308, 77.130350),
            (28.919298, 77.130450),
            (28.919298, 77.130550),
            (28.919298, 77.130650),
            (28.919293, 77.130750),
            (28.919293, 77.130750),
            (28.919285, 77.130897),
            (28.919285, 77.131085),
            (28.919285, 77.131105),
            (28.919290, 77.131114),
            (28.919298, 77.130156)
        ]

        self.waypoints_enu = []
        self.wp_index = 0
        self.path_generated = False
        self.mission_complete = False

        self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.publisher_ = self.create_publisher(Point, '/target_waypoint', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("Mission Node Initialized. Waiting for initial GPS lock...")

    def gps_callback(self, msg):
        self.current_latitude = msg.latitude
        self.current_longitude = msg.longitude

        if self.origin_latitude is None:
            self.origin_latitude = msg.latitude
            self.origin_longitude = msg.longitude
            self.get_logger().info(f"GPS Lock acquired! Origin set to: {self.origin_latitude}, {self.origin_longitude}")
            self.generate_gps_waypoint_path()

    def latlon_to_enu(self, lat, lon):
        R_EARTH = 6371000.0
        d_lat = math.radians(lat - self.origin_latitude)
        d_lon = math.radians(lon - self.origin_longitude)
        lat0 = math.radians(self.origin_latitude)

        north = d_lat * R_EARTH
        east = d_lon * R_EARTH * math.cos(lat0)
        return east, north

    def generate_gps_waypoint_path(self):
        self.waypoints_enu = []
        for lat, lon in self.gps_waypoints:
            east, north = self.latlon_to_enu(lat, lon)
            self.waypoints_enu.append((east, north))
        self.path_generated = True
        self.get_logger().info(f"Generated ENU path with {len(self.waypoints_enu)} waypoints.")

    def timer_callback(self):
        if not self.path_generated or self.current_latitude is None:
            return

        curr_east, curr_north = self.latlon_to_enu(self.current_latitude, self.current_longitude)
        tgt_east, tgt_north = self.waypoints_enu[self.wp_index]

        dist = math.sqrt((tgt_east - curr_east)**2 + (tgt_north - curr_north)**2)

        if dist < self.arrival_dist:
            if self.wp_index < len(self.waypoints_enu) - 1:
                self.wp_index += 1
                self.get_logger().info(f"Waypoint {self.wp_index} reached! Advancing to waypoint {self.wp_index + 1}.")
            elif not self.mission_complete:
                self.mission_complete = True
                self.get_logger().info("MISSION COMPLETE: CAR HAS RETURNED TO ROYAL CAFE!")

        msg = Point()
        msg.x = self.waypoints_enu[self.wp_index][0]
        msg.y = self.waypoints_enu[self.wp_index][1]
        msg.z = 0.0
        self.publisher_.publish(msg)

def main():
    rclpy.init()
    node = MissionNode()
    rclpy.spin(node)
    rclpy.shutdown()