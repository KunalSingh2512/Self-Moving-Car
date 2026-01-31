#!/usr/bin/env python3
"""
ROS2 Control Node - Robust Sensor Fusion
Uses:
1. ODOM: For Velocity (Speed)
2. IMU: For Angular Velocity (Smooth Turns)
3. GNSS: For Absolute Position AND True Heading Correction
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Point, Twist, PoseStamped
import math

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        
        # ===== CONFIGURATION =====
        self.move_threshold = 0.1   # Must move 10cm to update heading from GPS
        self.filter_alpha = 0.2     # Fusion: 20% GPS (Correction), 80% IMU (Smoothness)
        self.kp_lin = 0.8
        self.kp_ang = 1.4

        # ===== STATE =====
        self.origin_lat = None
        self.origin_lon = None
        
        self.curr_x = 0.0
        self.curr_y = 0.0
        self.yaw = 0.0              # The Fused Heading
        
        # Sensor Data Buffers
        self.enc_v = 0.0            # From Odom
        self.imu_w = 0.0            # From IMU
        
        self.heading_initialized = False

        # ===== SUBSCRIBERS =====
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.create_subscription(NavSatFix, '/gnss/fix', self.gnss_callback, 10)
        self.create_subscription(Point, '/target_waypoint', self.target_callback, 10)
        
        # ===== PUBLISHERS =====
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/fusion/pose', 10)
        
        # Timer (20Hz)
        self.dt = 0.05
        self.target_x = 0.0
        self.target_y = 0.0
        self.create_timer(self.dt, self.control_loop)

    def odom_callback(self, msg):
        # USE 1: Odom gives us Speed
        self.enc_v = msg.twist.twist.linear.x

    def imu_callback(self, msg):
        # USE 2: IMU gives us Turn Rate
        self.imu_w = msg.angular_velocity.z

    def gnss_callback(self, msg):
        # USE 3: GNSS gives us Position & Heading Correction
        if self.origin_lat is None:
            self.origin_lat = msg.latitude
            self.origin_lon = msg.longitude
            self.get_logger().info(f"Origin Set: {self.origin_lat}, {self.origin_lon}")
            return

        # 1. Convert GPS to Meters (ENU)
        R_EARTH = 6371000.0
        d_lat = math.radians(msg.latitude - self.origin_lat)
        d_lon = math.radians(msg.longitude - self.origin_lon)
        lat0 = math.radians(self.origin_lat)
        
        new_x = d_lon * R_EARTH * math.cos(lat0)
        new_y = d_lat * R_EARTH 
        
        # 2. Calculate True Heading from Motion Vector
        dx = new_x - self.curr_x
        dy = new_y - self.curr_y
        dist = math.sqrt(dx**2 + dy**2)
        
        # Only correct heading if we moved significantly (filtering noise)
        if dist > self.move_threshold:
            gps_heading = math.atan2(dy, dx) # The Truth
            
            if not self.heading_initialized:
                self.yaw = gps_heading
                self.heading_initialized = True
            else:
                # COMPLEMENTARY FILTER:
                # Drag the current Yaw towards the GPS Yaw
                diff = gps_heading - self.yaw
                # Normalize angle diff
                while diff > math.pi: diff -= 2*math.pi
                while diff < -math.pi: diff += 2*math.pi
                
                # Apply Correction
                self.yaw += self.filter_alpha * diff

        # Always update Position from GPS (It's absolute)
        self.curr_x = new_x
        self.curr_y = new_y

    def target_callback(self, msg):
        self.target_x = msg.x
        self.target_y = msg.y

    def control_loop(self):
        if self.origin_lat is None: return

        # === PREDICTION STEP (Dead Reckoning) ===
        # Use Odom (Speed) and IMU (Turn) to predict motion between GPS clicks
        if self.heading_initialized:
            # 1. Update Heading using IMU
            self.yaw += self.imu_w * self.dt
            
            # Normalize Yaw
            while self.yaw > math.pi: self.yaw -= 2*math.pi
            while self.yaw < -math.pi: self.yaw += 2*math.pi
            
            # 2. Update Position using Odom Speed (Optional but smoother)
            # (Note: GPS will overwrite this, but this helps between updates)
            self.curr_x += self.enc_v * math.cos(self.yaw) * self.dt
            self.curr_y += self.enc_v * math.sin(self.yaw) * self.dt

        # === CONTROL STEP ===
        dx = self.target_x - self.curr_x
        dy = self.target_y - self.curr_y
        dist = math.sqrt(dx**2 + dy**2)
        target_angle = math.atan2(dy, dx)
        
        err_ang = target_angle - self.yaw
        while err_ang > math.pi: err_ang -= 2*math.pi
        while err_ang < -math.pi: err_ang += 2*math.pi
        
        cmd = Twist()
        
        if not self.heading_initialized:
            # Blind start to generate vector
            cmd.linear.x = 0.5
        else:
            if abs(err_ang) > 1.0:
                cmd.linear.x = 0.3
            else:
                cmd.linear.x = min(0.8, self.kp_lin * dist)
                
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