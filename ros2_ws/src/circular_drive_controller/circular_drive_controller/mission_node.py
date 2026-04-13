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
            (28.919283, 77.131428),
            (28.919294, 77.131135),
            (28.919291, 77.131084),
            (28.919291, 77.131033),
            (28.919283, 77.130985),
            (28.919294, 77.130927),
            (28.919296, 77.130927),
            (28.919297, 77.130868),
            (28.919282, 77.130821),
            (28.919297, 77.130781),
            (28.919294, 77.130738),
            (28.919298, 77.130684),
            (28.919301, 77.130627),
            (28.919301, 77.130577),
            (28.919302, 77.130515),
            (28.919311, 77.130455),
            (28.919312, 77.130388),
            (28.919310, 77.130329),
            (28.919316, 77.130260),
            (28.919318, 77.130197),
            (28.919313, 77.130134),
            (28.919310, 77.130054),
            (28.919315, 77.130001),
            (28.919314, 77.129939),
            (28.919314, 77.129881),
            (28.919318, 77.129810),
            (28.919338, 77.129740),
            (28.919353, 77.129668),
            (28.919378, 77.129605),
            (28.919405, 77.129549),
            (28.919427, 77.129487),
            (28.919446, 77.129430),
            (28.919474, 77.129358), # FIXED: Comma changed to decimal, added trailing comma
            (28.919493, 77.129311),
            (28.919519, 77.129265),
            (28.919535, 77.129218),
            (28.919553, 77.129168),
            (28.919570, 77.129133),
            (28.919589, 77.129080),
            (28.919606, 77.129032),
            (28.919634, 77.128975),
            (28.919653, 77.128924),
            (28.919679, 77.128867),
            (28.919704, 77.128811),
            (28.919721, 77.128762),
            (28.919747, 77.128695),
            (28.919764, 77.128665),
            (28.919785, 77.128624),
            (28.919802, 77.128582),
            (28.919828, 77.128522),
            (28.919836, 77.128456),
            (28.919830, 77.128407),
            (28.919735, 77.128398),
            (28.919739, 77.128400),
            (28.919705, 77.128398),
            (28.919679, 77.128398),
            (28.919652, 77.128398),
            (28.919611, 77.128392),
            (28.919575, 77.128393),
            (28.919533, 77.128390),
            (28.919488, 77.128386),
            (28.919460, 77.128385),
            (28.919397, 77.128389),
            (28.919334, 77.128383),
            (28.919298, 77.128380),
            (28.919260, 77.128379),
            (28.919191, 77.128376),
            (28.919103, 77.128371),
            (28.919046, 77.128366),
            (28.918952, 77.128354),
            (28.918870, 77.128353),
            (28.918791, 77.128353),
            (28.918724, 77.128352),
            (28.918642, 77.128346),
            (28.918578, 77.128357),
            (28.918542, 77.128356),
            (28.918547, 77.128404),
            (28.918538, 77.128469),
            (28.918530, 77.128527),
            (28.918531, 77.128555),
            (28.918518, 77.128640),
            (28.918516, 77.128709),
            (28.918522, 77.128762)
        ]

        self.waypoints_enu = []
        self.wp_index = 0
        self.path_generated = False
        self.mission_complete = False

        # FIXED: Changed topic from /gnss/fix to /gps/fix
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
                self.get_logger().info("WE HAVE ARRIVED AT THE ADMIN BLOCK! Mission Accomplished.")

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