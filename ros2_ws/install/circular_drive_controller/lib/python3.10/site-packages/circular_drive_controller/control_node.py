"""
ROS2 Control Node with EKF Integration
Production-ready robot navigation system with sensor fusion
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point, Twist, PoseStamped
import math
import numpy as np


# =====================================================
# EXTENDED KALMAN FILTER CLASS
# =====================================================

class EKF:
    """Extended Kalman Filter for Robot Navigation with Sensor Fusion"""
    
    def _init_(self, x0, P0, Q, R, dt):
        """
        Initialize EKF
        
        Parameters:
            x0: Initial state vector (8,) = [pn, pe, psi, vn, ve, bgz, bax, bay]
            P0: Initial covariance (8, 8)
            Q: Process noise (8, 8)
            R: Measurement noise (3, 3)
            dt: Time step in seconds
        """
        self.x = np.array(x0, dtype=float)
        self.P = np.array(P0, dtype=float)
        self.Q = np.array(Q, dtype=float)
        self.R = np.array(R, dtype=float)
        self.dt = float(dt)
        
        # Constants
        self.g = 9.81
        self.R_earth = 6371000  # Earth radius in meters
        
        # GNSS reference point
        self.origin_lat = None
        self.origin_lon = None

    def predict(self, imu_data, encoder_data):
        """PREDICTION STEP: Update state using IMU + Encoder"""
        pn, pe, psi, vn, ve, bgz, bax, bay = self.x
        
        # Gyro with bias correction
        omega_z_corrected = imu_data['omega_z'] - bgz
        psi_new = psi + omega_z_corrected * self.dt
        
        # Accelerometer with bias correction
        accel_x_corrected = imu_data['accel_x'] - bax
        accel_y_corrected = imu_data['accel_y'] - bay
        
        # Velocity from accelerometer
        vn_from_accel = vn + accel_x_corrected * self.dt
        ve_from_accel = ve + accel_y_corrected * self.dt
        
        # Velocity from encoder
        v_enc = encoder_data['v_linear']
        vn_from_encoder = v_enc * np.cos(psi_new)
        ve_from_encoder = v_enc * np.sin(psi_new)
        
        # Sensor fusion (90% encoder, 10% accel)
        alpha = 0.1
        vn_new = (1 - alpha) * vn_from_encoder + alpha * vn_from_accel
        ve_new = (1 - alpha) * ve_from_encoder + alpha * ve_from_accel
        
        # Position integration
        pn_new = pn + vn_new * self.dt
        pe_new = pe + ve_new * self.dt
        
        # Biases remain stable
        bgz_new = bgz
        bax_new = bax
        bay_new = bay
        
        # Construct predicted state
        x_predicted = np.array([pn_new, pe_new, psi_new, vn_new, ve_new, bgz_new, bax_new, bay_new])
        
        # Compute Jacobian
        F = self._compute_jacobian_state_transition(self.x, imu_data, encoder_data)
        
        # Propagate covariance
        P_predicted = F @ self.P @ F.T + self.Q
        
        # Update state and covariance
        self.x = x_predicted
        self.P = P_predicted

    def _compute_jacobian_state_transition(self, x, imu_data, encoder_data):
        """Compute F = ∂f/∂x using numerical differentiation"""
        n = len(x)
        F = np.zeros((n, n))
        epsilon = 1e-6
        
        for j in range(n):
            x_plus = x.copy().astype(float)
            x_plus[j] += epsilon
            f_plus = self._state_transition_function(x_plus, imu_data, encoder_data)
            
            x_minus = x.copy().astype(float)
            x_minus[j] -= epsilon
            f_minus = self._state_transition_function(x_minus, imu_data, encoder_data)
            
            F[:, j] = (f_plus - f_minus) / (2 * epsilon)
        
        return F

    def _state_transition_function(self, x, imu_data, encoder_data):
        """Deterministic state transition function"""
        pn, pe, psi, vn, ve, bgz, bax, bay = x
        
        omega_z = imu_data['omega_z'] - bgz
        accel_x = imu_data['accel_x'] - bax
        accel_y = imu_data['accel_y'] - bay
        v_enc = encoder_data['v_linear']
        
        psi_new = psi + omega_z * self.dt
        
        vn_enc = v_enc * np.cos(psi_new)
        ve_enc = v_enc * np.sin(psi_new)
        
        alpha = 0.1
        vn_new = (1 - alpha) * vn_enc + alpha * (vn + accel_x * self.dt)
        ve_new = (1 - alpha) * ve_enc + alpha * (ve + accel_y * self.dt)
        
        pn_new = pn + vn_new * self.dt
        pe_new = pe + ve_new * self.dt
        
        return np.array([pn_new, pe_new, psi_new, vn_new, ve_new, bgz, bax, bay])

    def update(self, gnss_data):
        """UPDATE STEP: Correct state using GNSS measurements"""
        if self.origin_lat is None:
            self.origin_lat = gnss_data['latitude']
            self.origin_lon = gnss_data['longitude']
            return
        
        pn_gnss, pe_gnss = self._gnss_to_ned(gnss_data['latitude'], gnss_data['longitude'])
        
        z = np.array([pn_gnss, pe_gnss, 0.0])
        h_x = np.array([self.x[0], self.x[1], 0.0])
        
        H = np.zeros((3, 8))
        H[0, 0] = 1.0
        H[1, 1] = 1.0
        
        innovation = z - h_x
        S = H @ self.P @ H.T + self.R
        
        try:
            S_inv = np.linalg.inv(S)
            K = self.P @ H.T @ S_inv
        except np.linalg.LinAlgError:
            return
        
        delta_x = K @ innovation
        x_updated = self.x + delta_x
        
        I = np.eye(len(self.x))
        I_KH = I - K @ H
        P_updated = I_KH @ self.P @ I_KH.T + K @ self.R @ K.T
        
        self.x = x_updated
        self.P = P_updated

    def _gnss_to_ned(self, lat, lon):
        """Convert GPS to NED coordinates"""
        delta_lat = lat - self.origin_lat
        delta_lon = lon - self.origin_lon
        lat_rad = math.radians(self.origin_lat)
        
        p_n = delta_lat * (self.R_earth * math.pi / 180)
        p_e = delta_lon * (self.R_earth * math.pi / 180) * math.cos(lat_rad)
        
        return p_n, p_e


# =====================================================
# ROS2 CONTROL NODE WITH EKF
# =====================================================

class ControlNode(Node):
    def _init_(self):
        super()._init_('control_node')
        
        # ===== SUBSCRIBERS =====
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu', self.imu_callback, 10)
        self.target_sub = self.create_subscription(
            Point, '/target_waypoint', self.target_callback, 10)
        
        # ===== PUBLISHERS =====
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.estimated_pose_pub = self.create_publisher(PoseStamped, '/ekf/pose', 10)
        
        # ===== INITIALIZE EKF =====
        x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        P0 = np.diag([1.0, 1.0, 0.1, 0.5, 0.5, 0.01, 0.1, 0.1])
        Q = np.diag([0.001, 0.001, 0.0001, 0.01, 0.01, 1e-6, 1e-4, 1e-4])
        R = np.diag([3.0, 3.0, 0.1])
        
        self.ekf = EKF(x0, P0, Q, R, dT=0.05)
        self.get_logger().info("✓ EKF initialized successfully!")
        
        # ===== STATE VARIABLES =====
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        self.imu_angular_z = 0.0
        self.imu_accel_x = 0.0
        self.imu_accel_y = 0.0
        
        self.encoder_v_linear = 0.0
        self.encoder_omega = 0.0
        
        self.target_x = 0.0
        self.target_y = 0.0
        
        # Control parameters
        self.kp_linear = 0.5
        self.ki_linear = 0.1
        self.kp_angular = 0.8
        self.ki_angular = 0.1
        
        self.error_distance_integral = 0.0
        self.error_angle_integral = 0.0

    def odom_callback(self, msg):
        """Odometry callback - extract encoder data"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.current_yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy*qy + qz*qz))
        
        # Extract linear velocity
        self.encoder_v_linear = math.sqrt(
            msg.twist.twist.linear.x*2 + msg.twist.twist.linear.y*2
        )
        self.encoder_omega = msg.twist.twist.angular.z

    def imu_callback(self, msg):
        """IMU callback - extract gyro and accel"""
        self.imu_angular_z = msg.angular_velocity.z
        self.imu_accel_x = msg.linear_acceleration.x
        self.imu_accel_y = msg.linear_acceleration.y

    def target_callback(self, msg):
        """Target callback - triggers control loop"""
        self.target_x = msg.x
        self.target_y = msg.y
        
        self.get_logger().info(f"New target: ({self.target_x:.2f}, {self.target_y:.2f})")
        
        # Run control loop
        self.control_loop()

    def control_loop(self):
        """Main control loop with EKF"""
        
        # ===== STEP 1: EKF PREDICTION =====
        imu_data = {
            'omega_z': self.imu_angular_z,
            'accel_x': self.imu_accel_x,
            'accel_y': self.imu_accel_y
        }
        
        encoder_data = {
            'v_linear': self.encoder_v_linear,
            'omega': self.encoder_omega
        }
        
        try:
            self.ekf.predict(imu_data, encoder_data)
        except Exception as e:
            self.get_logger().error(f"EKF prediction error: {e}")
            return
        
        # ===== STEP 2: GET ESTIMATED STATE =====
        estimated_pn = self.ekf.x[0]      # North position
        estimated_pe = self.ekf.x[1]      # East position
        estimated_psi = self.ekf.x[2]     # Yaw angle
        
        # Publish estimated pose
        self.publish_estimated_pose(estimated_pn, estimated_pe, estimated_psi)
        
        # ===== STEP 3: CALCULATE ERRORS =====
        distance_error = math.sqrt(
            (self.target_x - estimated_pn)**2 + 
            (self.target_y - estimated_pe)**2
        )
        
        target_heading = math.atan2(
            self.target_y - estimated_pe,
            self.target_x - estimated_pn
        )
        heading_error = target_heading - estimated_psi
        
        # Normalize heading error
        while heading_error > math.pi:
            heading_error -= 2 * math.pi
        while heading_error < -math.pi:
            heading_error += 2 * math.pi
        
        # ===== STEP 4: PID CONTROL =====
        cmd = Twist()
        
        # Linear velocity (PID)
        self.error_distance_integral += distance_error * 0.05
        cmd.linear.x = self.kp_linear * distance_error + self.ki_linear * self.error_distance_integral
        cmd.linear.x = max(-1.0, min(1.0, cmd.linear.x))
        
        # Angular velocity (PID)
        self.error_angle_integral += heading_error * 0.05
        cmd.angular.z = self.kp_angular * heading_error + self.ki_angular * self.error_angle_integral
        cmd.angular.z = max(-1.0, min(1.0, cmd.angular.z))
        
        # ===== STEP 5: PUBLISH COMMANDS =====
        self.cmd_pub.publish(cmd)
        
        uncertainty = np.trace(self.ekf.P)
        self.get_logger().info(
            f"EKF: pos=({estimated_pn:.2f},{estimated_pe:.2f}) | "
            f"ψ={estimated_psi:.3f} | err_dist={distance_error:.2f}m | "
            f"err_head={heading_error:.3f}rad | cmd=({cmd.linear.x:.2f},{cmd.angular.z:.2f})"
        )

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


if __name__ == '_main_':
    main()