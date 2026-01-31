"""
ROS2 Control Node with EKF + GNSS Integration
Fix: Added GNSS subscriber to correct EKF drift.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import Point, Twist, PoseStamped
import math
import numpy as np

# [EKF CLASS REMAINS UNCHANGED - PASTE ABHAY'S EKF CLASS HERE]
# For brevity, I am assuming the EKF class is defined above or imported.
# Paste the EKF class from your previous message here.
class EKF:
    def __init__(self, x0, P0, Q, R, dt):
        self.x = np.array(x0, dtype=float)
        self.P = np.array(P0, dtype=float)
        self.Q = np.array(Q, dtype=float)
        self.R = np.array(R, dtype=float)
        self.dt = float(dt)
        self.R_earth = 6371000
        self.origin_lat = None
        self.origin_lon = None

    def predict(self, imu_data, encoder_data):
        # ... (Same as Abhay's code) ...
        # Simplified for brevity in this response, use full code
        pass 

    def update(self, gnss_data):
        """Correct state using GNSS"""
        if self.origin_lat is None:
            self.origin_lat = gnss_data['latitude']
            self.origin_lon = gnss_data['longitude']
            return
        
        # Calculate North/East position from GPS
        delta_lat = math.radians(gnss_data['latitude'] - self.origin_lat)
        delta_lon = math.radians(gnss_data['longitude'] - self.origin_lon)
        lat0 = math.radians(self.origin_lat)
        
        z_n = delta_lat * self.R_earth
        z_e = delta_lon * self.R_earth * math.cos(lat0)

        # Standard Kalman Update (z - Hx)
        # Observation is [North, East]
        z = np.array([z_n, z_e])
        
        # H matrix maps state [pn, pe, ...] to observation [pn, pe]
        H = np.zeros((2, 8))
        H[0, 0] = 1 # pn
        H[1, 1] = 1 # pe
        
        y = z - H @ self.x # Innovation
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)
        
        self.x = self.x + K @ y
        self.P = (np.eye(8) - K @ H) @ self.P


class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        
        # ===== SUBSCRIBERS =====
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.create_subscription(Point, '/target_waypoint', self.target_callback, 10)
        
        # NEW: Listen to GNSS to fix drift
        self.create_subscription(NavSatFix, '/gnss/fix', self.gnss_callback, 10)
        
        # ===== PUBLISHERS =====
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # ===== EKF SETUP =====
        x0 = np.zeros(8)
        P0 = np.eye(8)
        Q = np.eye(8) * 0.01
        R = np.eye(2) * 2.0 # High GNSS noise covariance
        self.ekf = EKF(x0, P0, Q, R, dt=0.05)
        
        # State
        self.target_e = 0.0 # East (x)
        self.target_n = 0.0 # North (y)
        self.enc_v = 0.0
        self.imu_w = 0.0
        self.imu_ax = 0.0
        
        # Control Gains
        self.kp_lin = 0.6
        self.kp_ang = 1.2

    def odom_callback(self, msg):
        self.enc_v = msg.twist.twist.linear.x

    def imu_callback(self, msg):
        self.imu_w = msg.angular_velocity.z
        self.imu_ax = msg.linear_acceleration.x

    def gnss_callback(self, msg):
        # Feed GPS data to EKF to correct position
        data = {'latitude': msg.latitude, 'longitude': msg.longitude}
        self.ekf.update(data)

    def target_callback(self, msg):
        self.target_e = msg.x
        self.target_n = msg.y
        self.control_loop()

    def control_loop(self):
        # 1. EKF Predict
        # (Assuming you insert Abhay's full predict logic here)
        # For simplicity in this snippets, assume ekf.x is updated
        # self.ekf.predict(...) 
        
        # 2. Get Estimated State
        est_n = self.ekf.x[0]
        est_e = self.ekf.x[1]
        est_psi = self.ekf.x[2] # Yaw
        
        # 3. Calculate Error to Target
        dn = self.target_n - est_n
        de = self.target_e - est_e
        dist = math.sqrt(dn**2 + de**2)
        
        target_ang = math.atan2(dn, de) # Angle to target (North/East)
        
        # Note: Standard atan2 is (y, x). Here Y=North, X=East.
        # Heading Error
        err_ang = target_ang - est_psi
        while err_ang > math.pi: err_ang -= 2*math.pi
        while err_ang < -math.pi: err_ang += 2*math.pi
        
        # 4. Control
        cmd = Twist()
        cmd.linear.x = min(0.8, self.kp_lin * dist)
        cmd.angular.z = self.kp_ang * err_ang
        
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()