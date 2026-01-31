#!/usr/bin/env python3
"""
ROS2 Control Node - IMU + GNSS Fusion (Complementary Filter)
Uses IMU for smooth high-speed updates, and GNSS for absolute heading correction.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Point, Twist, PoseStamped
import math

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        
        # ===== CONFIGURATION =====
        self.move_threshold = 0.1  # GPS must detect 10cm move to correct heading
        self.kp_lin = 0.8          # Speed Gain
        self.kp_ang = 1.4          # Turn Gain
        self.filter_alpha = 0.2    # Fusion Weight: 20% GPS (Correction), 80% IMU (Smoothness)
        
        # ===== STATE =====
        self.origin_lat = None
        self.origin_lon = None
        
        self.curr_x = 0.0
        self.curr_y = 0.0
        self.yaw = 0.0             # Robot Heading
        
        self.imu_w = 0.0           # Current Gyro Z (Yaw Rate)
        self.heading_initialized = False

        # ===== I/O =====
        self.create_subscription(NavSatFix, '/gnss/fix', self.gnss_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.create_subscription(Point, '/target_waypoint', self.target_callback, 10)
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/fusion/pose', 10)
        
        # Control Loop runs at 20Hz (0.05s)
        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.control_loop)

        # Buffer for target
        self.target_x = 0.0
        self.target_y = 0.0

    def imu_callback(self, msg):
        # Store latest gyro reading for the control loop
        self.imu_w = msg.angular_velocity.z

    def gnss_callback(self, msg):
        # 1. Set Origin
        if self.origin_lat is None:
            self.origin_lat = msg.latitude
            self.origin_lon = msg.longitude
            self.get_logger().info(f"✅ Origin Set: {self.origin_lat}, {self.origin_lon}")
            return

        # 2. Convert GPS to Meters (ENU)
        R_EARTH = 6371000.0
        d_lat = math.radians(msg.latitude - self.origin_lat)
        d_lon = math.radians(msg.longitude - self.origin_lon)
        lat0 = math.radians(self.origin_lat)
        
        new_x = d_lon * R_EARTH * math.cos(lat0)
        new_y = d_lat * R_EARTH 
        
        # 3. Calculate GPS Heading (Ground Truth)
        dx = new_x - self.curr_x
        dy = new_y - self.curr_y
        dist = math.sqrt(dx**2 + dy**2)
        
        # Only correct using GPS if we moved enough to get a clean vector
        if dist > self.move_threshold:
            gps_heading = math.atan2(dy, dx)
            
            if not self.heading_initialized:
                # First time: Trust GPS 100%
                self.yaw = gps_heading
                self.heading_initialized = True
            else:
                # Fusion: Blend current Yaw (IMU-based) with GPS Yaw
                # Shortest angle interpolation to avoid 360->0 jumps
                diff = gps_heading - self.yaw
                while diff > math.pi: diff -= 2*math.pi
                while diff < -math.pi: diff += 2*math.pi
                
                # Apply Correction (Complementary Filter)
                self.yaw += self.filter_alpha * diff

        # Update Position
        self.curr_x = new_x
        self.curr_y = new_y

    def target_callback(self, msg):
        self.target_x = msg.x
        self.target_y = msg.y

    def control_loop(self):
        if self.origin_lat is None: return

        # 1. PREDICT: Integrate IMU Gyro for smoothness
        # This runs 20 times a second, keeping the yaw smooth between GPS updates
        if self.heading_initialized:
            self.yaw += self.imu_w * self.dt
            # Normalize
            while self.yaw > math.pi: self.yaw -= 2*math.pi
            while self.yaw < -math.pi: self.yaw += 2*math.pi

        # 2. CALCULATE ERROR
        dx = self.target_x - self.curr_x
        dy = self.target_y - self.curr_y
        dist = math.sqrt(dx**2 + dy**2)
        target_angle = math.atan2(dy, dx)
        
        err_ang = target_angle - self.yaw
        while err_ang > math.pi: err_ang -= 2*math.pi
        while err_ang < -math.pi: err_ang += 2*math.pi
        
        # 3. CONTROL
        cmd = Twist()
        
        if not self.heading_initialized:
            # Blind start to get GPS vector
            cmd.linear.x = 0.5
        else:
            # Normal driving
            if abs(err_ang) > 1.0:
                cmd.linear.x = 0.3
            else:
                cmd.linear.x = min(1.0, self.kp_lin * dist)
                
            cmd.angular.z = self.kp_ang * err_ang
            cmd.angular.z = max(-1.5, min(1.5, cmd.angular.z))

        self.cmd_pub.publish(cmd)
        self.publish_viz()

    def publish_viz(self):
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = self.curr_x
        msg.pose.position.y = self.curr_y
        msg.pose.orientation.z = math.sin(self.yaw/2)
        msg.pose.orientation.w = math.cos(self.yaw/2)
        self.pose_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()