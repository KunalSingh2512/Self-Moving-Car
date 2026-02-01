#!/usr/bin/env python3
"""
ROS2 Control Node - Integrated 5-State EKF
Formatted to match Abhay's Original Structure
Logic: Motion-Derived Heading to fix shaking/U-turns
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point, Twist, PoseStamped
import math
import numpy as np
from sensor_msgs.msg import NavSatFix

# =====================================================
# EXTENDED KALMAN FILTER CLASS
# =====================================================

class EKF:
    """Extended Kalman Filter for Robot Navigation with Sensor Fusion"""
    
    def __init__(self, x0, P0, Q, R, dt):
        """
        Initialize EKF
        
        Parameters:
            x0: Initial state vector (5,) = [pn, pe, psi, v, bgz]
            P0: Initial covariance (5, 5)
            Q: Process noise (5, 5)
            R: Measurement noise (3, 3)
            dt: Time step in seconds
        """
        self.x = np.array(x0, dtype=float)
        self.P = np.array(P0, dtype=float)
        self.Q = np.array(Q, dtype=float)
        self.R = np.array(R, dtype=float)
        self.dt = float(dt)
        
        # Constants
        self.R_earth = 6371000.0  # Earth radius in meters
        
        # GNSS reference point
        self.origin_lat = None
        self.origin_lon = None
        
        # History for Motion-Derived Heading
        self.prev_pn_gnss = None
        self.prev_pe_gnss = None

    def predict(self, imu_data, encoder_data):
        """PREDICTION STEP: Update state using IMU + Encoder (Unicycle Model)"""
        # Unpack 5-state vector
        pn, pe, psi, v, bgz = self.x
        
        # Inputs
        omega_z_meas = imu_data['omega_z']
        v_enc = encoder_data['v_linear']
        
        # Correct turn rate with estimated bias
        w = omega_z_meas - bgz
        
        # 1. State Transition
        pn_new = pn + v * math.cos(psi) * self.dt
        pe_new = pe + v * math.sin(psi) * self.dt
        psi_new = psi + w * self.dt
        v_new = v_enc  # Trust encoder for velocity in this model
        bgz_new = bgz  # Bias random walk
        
        # Normalize Yaw
        psi_new = (psi_new + np.pi) % (2 * np.pi) - np.pi
        
        # Update State
        self.x = np.array([pn_new, pe_new, psi_new, v_new, bgz_new])
        
        # 2. Jacobian F (Analytical)
        # F = ∂f/∂x
        F = np.eye(5)
        F[0, 2] = -v * math.sin(psi) * self.dt  # d(pn)/d(psi)
        F[0, 3] = math.cos(psi) * self.dt       # d(pn)/d(v)
        F[1, 2] = v * math.cos(psi) * self.dt   # d(pe)/d(psi)
        F[1, 3] = math.sin(psi) * self.dt       # d(pe)/d(v)
        F[2, 4] = -self.dt                      # d(psi)/d(bgz)
        
        # Propagate Covariance
        self.P = F @ self.P @ F.T + self.Q

    def update(self, gnss_data):
        """UPDATE STEP: Correct state using GNSS + Motion Heading"""
        if self.origin_lat is None:
            self.origin_lat = gnss_data['latitude']
            self.origin_lon = gnss_data['longitude']
            return
        
        # Convert GNSS to NED (Meters)
        pn_gnss, pe_gnss = self._gnss_to_ned(gnss_data['latitude'], gnss_data['longitude'])
        
        # Initialize history if empty
        if self.prev_pn_gnss is None:
            self.prev_pn_gnss = pn_gnss
            self.prev_pe_gnss = pe_gnss
            return

        # Calculate Motion Vector (Ground Truth Heading)
        d_pn = pn_gnss - self.prev_pn_gnss
        d_pe = pe_gnss - self.prev_pe_gnss
        dist_moved = math.sqrt(d_pn**2 + d_pe**2)
        
        # Update history
        self.prev_pn_gnss = pn_gnss
        self.prev_pe_gnss = pe_gnss

        # Decide Observation Type based on movement
        # Logic: Only observe Yaw if we moved > 0.15m (Fixes shaking)
        if dist_moved > 0.15:
            # Moving: Observe Position + Heading
            meas_psi = math.atan2(d_pe, d_pn) # ENU logic
            z = np.array([pn_gnss, pe_gnss, meas_psi])
            
            H = np.zeros((3, 5))
            H[0, 0] = 1.0 # Observe pn
            H[1, 1] = 1.0 # Observe pe
            H[2, 2] = 1.0 # Observe psi
            
            # Use full measurement noise
            R_step = self.R
        else:
            # Stationary: Only observe Position
            z = np.array([pn_gnss, pe_gnss])
            
            H = np.zeros((2, 5))
            H[0, 0] = 1.0
            H[1, 1] = 1.0
            
            # Use subset of R
            R_step = self.R[:2, :2]

        # Measurement Model
        h_x = H @ self.x
        
        # Innovation
        y = z - h_x
        
        # Normalize Yaw Residual if we are measuring it
        if len(y) == 3:
             y[2] = (y[2] + np.pi) % (2 * np.pi) - np.pi

        # Kalman Gain
        S = H @ self.P @ H.T + R_step
        
        try:
            S_inv = np.linalg.inv(S)
            K = self.P @ H.T @ S_inv
        except np.linalg.LinAlgError:
            return
        
        # Update State & Covariance
        self.x = self.x + K @ y
        self.P = (np.eye(5) - K @ H) @ self.P

    def _gnss_to_ned(self, lat, lon):
        """Convert GPS to NED/ENU coordinates"""
        delta_lat = math.radians(lat - self.origin_lat)
        delta_lon = math.radians(lon - self.origin_lon)
        lat_rad = math.radians(self.origin_lat)
        
        # Note: In standard ENU, Y is North, X is East. 
        # p_n (North) corresponds to Latitude change
        # p_e (East) corresponds to Longitude change
        p_n = delta_lat * self.R_earth
        p_e = delta_lon * self.R_earth * math.cos(lat_rad)
        
        # Swapped to match standard X=East, Y=North if needed, 
        # but sticking to strict Lat/Lon derivation here:
        # Lat -> Y (North), Lon -> X (East)
        return p_e, p_n # Return as (East, North) to match X,Y logic


# =====================================================
# ROS2 CONTROL NODE WITH EKF
# =====================================================

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        
        # ===== SUBSCRIBERS =====
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu', self.imu_callback, 10)
        self.target_sub = self.create_subscription(
            Point, '/target_waypoint', self.target_callback, 10)
        self.gnss_sub = self.create_subscription(
            NavSatFix, '/gnss/fix', self.gnss_callback, 10)
        
        # ===== PUBLISHERS =====
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.estimated_pose_pub = self.create_publisher(PoseStamped, '/ekf/pose', 10)
        
        # ===== INITIALIZE EKF (5-State Version) =====
        # State: [pn, pe, psi, v, bgz]
        x0 = np.zeros(5)
        P0 = np.eye(5)
        
        # Process Noise Q [pn, pe, psi, v, bgz]
        Q = np.diag([0.01, 0.01, 0.01, 0.1, 0.001])
        
        # Measurement Noise R [GPS_x, GPS_y, GPS_Yaw]
        R = np.diag([0.5, 0.5, 0.2])
        
        self.ekf = EKF(x0, P0, Q, R, dt=0.05)
        self.get_logger().info("✓ EKF initialized successfully!")
        
        # ===== STATE VARIABLES =====
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        self.imu_angular_z = 0.0
        
        self.encoder_v_linear = 0.0
        
        self.target_x = 0.0
        self.target_y = 0.0
        
        # Control parameters (Stable Gains)
        self.kp_linear = 0.8
        self.kp_angular = 1.0
        
        # Timer for loop
        self.create_timer(0.05, self.control_loop)

    def odom_callback(self, msg):
        """Odometry callback - extract encoder data"""
        self.encoder_v_linear = msg.twist.twist.linear.x

    def imu_callback(self, msg):
        """IMU callback - extract gyro"""
        self.imu_angular_z = msg.angular_velocity.z

    def gnss_callback(self, msg):
        """GNSS callback - update EKF"""
        gnss_data = {
            'latitude': msg.latitude,
            'longitude': msg.longitude
        }
        self.ekf.update(gnss_data)

    def target_callback(self, msg):
        """Target callback - update target"""
        self.target_x = msg.x
        self.target_y = msg.y

    def control_loop(self):
        """Main control loop with EKF"""
        
        # ===== STEP 1: EKF PREDICTION =====
        imu_data = {'omega_z': self.imu_angular_z}
        encoder_data = {'v_linear': self.encoder_v_linear}
        
        try:
            self.ekf.predict(imu_data, encoder_data)
        except Exception as e:
            self.get_logger().error(f"EKF prediction error: {e}")
            return
        
        # ===== STEP 2: GET ESTIMATED STATE =====
        estimated_pn = self.ekf.x[0]      # East/X
        estimated_pe = self.ekf.x[1]      # North/Y
        estimated_psi = self.ekf.x[2]     # Yaw
        
        # Publish estimated pose
        self.publish_estimated_pose(estimated_pn, estimated_pe, estimated_psi)
        
        # ===== STEP 3: CALCULATE ERRORS =====
        dx = self.target_x - estimated_pn
        dy = self.target_y - estimated_pe
        distance_error = math.sqrt(dx**2 + dy**2)
        
        target_heading = math.atan2(dy, dx)
        heading_error = target_heading - estimated_psi
        
        # Normalize heading error
        while heading_error > math.pi:
            heading_error -= 2 * math.pi
        while heading_error < -math.pi:
            heading_error += 2 * math.pi
        
        # ===== STEP 4: CONTROL LOGIC =====
        cmd = Twist()
        
        # Wait for Origin
        if self.ekf.origin_lat is None:
            return

        # Simple Proportional Control (Stable)
        if distance_error > 0.1:
            if abs(heading_error) > 1.0:
                 cmd.linear.x = 0.2
            else:
                 cmd.linear.x = min(0.8, self.kp_linear * distance_error)
            
            cmd.angular.z = max(-1.2, min(1.2, self.kp_angular * heading_error))
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        
        # ===== STEP 5: PUBLISH COMMANDS =====
        self.cmd_pub.publish(cmd)

    def publish_estimated_pose(self, pn, pe, psi):
        """Publish EKF estimated pose"""
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