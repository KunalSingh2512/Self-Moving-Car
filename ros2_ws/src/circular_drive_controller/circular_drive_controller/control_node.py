#!/usr/bin/env python3
"""
ROS2 Control Node - Integrated 5-State EKF & Ultrasonic Avoidance
UPGRADE: Fixed EKF History Bug + 'Pivot and Punch' Navigation
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix, LaserScan
from geometry_msgs.msg import Point, Twist, PoseStamped
import math
import numpy as np

# =====================================================
# EXTENDED KALMAN FILTER CLASS 
# =====================================================

class EKF:
    def __init__(self, x0, P0, Q, R, dt):
        self.x = np.array(x0, dtype=float)
        self.P = np.array(P0, dtype=float)
        self.Q = np.array(Q, dtype=float)
        self.R = np.array(R, dtype=float)
        self.dt = float(dt)
        self.R_earth = 6371000.0  
        self.origin_lat = None
        self.origin_lon = None
        self.prev_pn_gnss = None
        self.prev_pe_gnss = None

    def predict(self, imu_data, encoder_data):
        pn, pe, psi, v, bgz = self.x
        omega_z_meas = imu_data['omega_z']
        v_enc = encoder_data['v_linear']
        
        w = omega_z_meas - bgz
        
        pn_new = pn + v * math.cos(psi) * self.dt
        pe_new = pe + v * math.sin(psi) * self.dt
        psi_new = psi + w * self.dt
        v_new = v_enc  
        bgz_new = bgz  
        
        psi_new = (psi_new + np.pi) % (2 * np.pi) - np.pi
        
        self.x = np.array([pn_new, pe_new, psi_new, v_new, bgz_new])
        
        F = np.eye(5)
        F[0, 2] = -v * math.sin(psi) * self.dt  
        F[0, 3] = math.cos(psi) * self.dt       
        F[1, 2] = v * math.cos(psi) * self.dt   
        F[1, 3] = math.sin(psi) * self.dt       
        F[2, 4] = -self.dt                      
        
        self.P = F @ self.P @ F.T + self.Q

    def update(self, gnss_data):
        if self.origin_lat is None:
            self.origin_lat = gnss_data['latitude']
            self.origin_lon = gnss_data['longitude']
            return
        
        pn_gnss, pe_gnss = self._gnss_to_ned(gnss_data['latitude'], gnss_data['longitude'])
        
        if self.prev_pn_gnss is None:
            self.prev_pn_gnss = pn_gnss
            self.prev_pe_gnss = pe_gnss
            return

        d_pn = pn_gnss - self.prev_pn_gnss
        d_pe = pe_gnss - self.prev_pe_gnss
        dist_moved = math.sqrt(d_pn**2 + d_pe**2)

        if dist_moved > 0.15:
            meas_psi = math.atan2(d_pe, d_pn) 
            z = np.array([pn_gnss, pe_gnss, meas_psi])
            
            H = np.zeros((3, 5))
            H[0, 0] = 1.0 
            H[1, 1] = 1.0 
            H[2, 2] = 1.0 
            
            R_step = self.R
            
            # JARVIS FIX: Only reset history AFTER we actually move 0.15m!
            self.prev_pn_gnss = pn_gnss
            self.prev_pe_gnss = pe_gnss
        else:
            z = np.array([pn_gnss, pe_gnss])
            
            H = np.zeros((2, 5))
            H[0, 0] = 1.0
            H[1, 1] = 1.0
            
            R_step = self.R[:2, :2]

        h_x = H @ self.x
        y = z - h_x
        
        if len(y) == 3:
             y[2] = (y[2] + np.pi) % (2 * np.pi) - np.pi

        S = H @ self.P @ H.T + R_step
        
        try:
            S_inv = np.linalg.inv(S)
            K = self.P @ H.T @ S_inv
        except np.linalg.LinAlgError:
            return
        
        self.x = self.x + K @ y
        self.P = (np.eye(5) - K @ H) @ self.P

    def _gnss_to_ned(self, lat, lon):
        delta_lat = math.radians(lat - self.origin_lat)
        delta_lon = math.radians(lon - self.origin_lon)
        lat_rad = math.radians(self.origin_lat)
        
        p_n = delta_lat * self.R_earth
        p_e = delta_lon * self.R_earth * math.cos(lat_rad)
        
        return p_e, p_n 


# =====================================================
# ROS2 CONTROL NODE WITH EKF & ULTRASONIC
# =====================================================

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.target_sub = self.create_subscription(Point, '/target_waypoint', self.target_callback, 10)
        self.gnss_sub = self.create_subscription(NavSatFix, '/gps/fix', self.gnss_callback, 10)
        self.ultrasonic_sub = self.create_subscription(LaserScan, '/ultrasonic/scan', self.ultrasonic_callback, 10)
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.estimated_pose_pub = self.create_publisher(PoseStamped, '/ekf/pose', 10)
        
        x0 = np.zeros(5)
        P0 = np.eye(5)
        Q = np.diag([0.01, 0.01, 0.01, 0.1, 0.001])
        R = np.diag([0.5, 0.5, 0.2])
        self.ekf = EKF(x0, P0, Q, R, dt=0.05)
        self.get_logger().info("✓ Jarvis System Online. EKF Math Fixed and Pivot Logic Engaged!")
        
        self.imu_angular_z = 0.0
        self.encoder_v_linear = 0.0
        self.target_x = 0.0
        self.target_y = 0.0
        
        self.front_clearance = 4.0 
        self.emergency_brake_distance = 1.2 
        self.brake_latch_counter = 0
        
        self.kp_linear = 0.8
        self.kp_angular = 1.2
        
        self.create_timer(0.05, self.control_loop)

    def odom_callback(self, msg):
        self.encoder_v_linear = msg.twist.twist.linear.x

    def imu_callback(self, msg):
        self.imu_angular_z = msg.angular_velocity.z

    def gnss_callback(self, msg):
        gnss_data = {'latitude': msg.latitude, 'longitude': msg.longitude}
        self.ekf.update(gnss_data)

    def target_callback(self, msg):
        self.target_x = msg.x
        self.target_y = msg.y

    def ultrasonic_callback(self, msg):
        valid_ranges = [r for r in msg.ranges if not math.isinf(r) and not math.isnan(r)]
        if valid_ranges:
            self.front_clearance = min(valid_ranges)
        else:
            self.front_clearance = 4.0

    def control_loop(self):
        imu_data = {'omega_z': self.imu_angular_z}
        encoder_data = {'v_linear': self.encoder_v_linear}
        
        try:
            self.ekf.predict(imu_data, encoder_data)
        except Exception as e:
            self.get_logger().error(f"EKF prediction error: {e}")
            return
        
        estimated_pn = self.ekf.x[0]      
        estimated_pe = self.ekf.x[1]      
        estimated_psi = self.ekf.x[2]     
        
        self.publish_estimated_pose(estimated_pn, estimated_pe, estimated_psi)
        
        dx = self.target_x - estimated_pn
        dy = self.target_y - estimated_pe
        distance_error = math.sqrt(dx**2 + dy**2)
        
        target_heading = math.atan2(dy, dx)
        heading_error = target_heading - estimated_psi
        
        while heading_error > math.pi:
            heading_error -= 2 * math.pi
        while heading_error < -math.pi:
            heading_error += 2 * math.pi
        
        cmd = Twist()
        
        if self.ekf.origin_lat is None:
            return

        # JARVIS FIX: Pivot and Punch Logic
        if distance_error > 0.2:
            if abs(heading_error) > 0.4: # If heading is off by > ~23 degrees, STOP AND PIVOT
                 cmd.linear.x = 0.0
                 cmd.angular.z = math.copysign(1.0, heading_error) # Spin left or right at 1.0 rad/s
            else: # If aligned, punch the gas
                 cmd.linear.x = min(0.6, self.kp_linear * distance_error)
                 cmd.angular.z = self.kp_angular * heading_error
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            
        # Ultrasonic Emergency Brake
        if self.front_clearance < self.emergency_brake_distance:
            self.get_logger().warn(f"OBSTACLE DETECTED AT {self.front_clearance:.2f}m! Engaging Emergency Brake.")
            self.brake_latch_counter = 10  
            
        if self.brake_latch_counter > 0:
            cmd.linear.x = 0.0 
            cmd.angular.z = 0.0
            self.brake_latch_counter -= 1
        
        # TELEMETRY HUD (Prints every 1 second) =====
        self.telemetry_counter = getattr(self, 'telemetry_counter', 0) + 1
        if self.telemetry_counter % 20 == 0: 
            self.get_logger().info(
                f"\n=== TELEMETRY HUD ===\n"
                f"TARGET   : X:{self.target_x:.2f}, Y:{self.target_y:.2f}\n"
                f"CURRENT  : X:{estimated_pn:.2f}, Y:{estimated_pe:.2f}, Yaw:{math.degrees(estimated_psi):.1f}°\n"
                f"ERRORS   : Dist: {distance_error:.2f}m | Heading: {math.degrees(heading_error):.1f}°\n"
                f"COMMANDS : Gas: {cmd.linear.x:.2f} m/s | Steering: {cmd.angular.z:.2f} rad/s\n"
                f"============================"
            )

        self.cmd_pub.publish(cmd)

    def publish_estimated_pose(self, pn, pe, psi):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = pn
        pose_msg.pose.position.y = pe
        pose_msg.pose.position.z = 0.0
        qz = math.sin(psi / 2)
        qw = math.cos(psi / 2)
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw
        self.estimated_pose_pub.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()