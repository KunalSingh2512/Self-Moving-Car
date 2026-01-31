#!/usr/bin/env python3
"""
ROS2 Control Node - Standard EKF with Jacobians
Fix: Solves 'Shaking' by calculating Heading from Motion History (Prev GPS vs Curr GPS)
instead of Position Error.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Point, Twist, PoseStamped
import math
import numpy as np

class EKF:
    def __init__(self, dt):
        self.dt = dt
        
        # State Vector x: [x, y, yaw, v, bias_gyro]
        self.x = np.zeros(5)
        self.P = np.eye(5)
        
        # Process Noise Q (Tuning)
        # [x, y, yaw, v, bias]
        self.Q = np.diag([0.01, 0.01, 0.01, 0.1, 0.001])
        
        # Measurement Noise R
        # [GPS_x, GPS_y, GPS_Yaw]
        self.R = np.diag([0.5, 0.5, 0.2]) 
        
        self.origin_lat = None
        self.origin_lon = None
        self.R_EARTH = 6371000.0
        
        # History for Heading Calculation
        self.prev_z_x = None
        self.prev_z_y = None

    def predict(self, v_meas, w_meas):
        """Standard EKF Prediction Step (Unicycle Model)"""
        x, y, yaw, v, bias = self.x
        
        w = w_meas - bias
        
        # 1. State Transition
        x_new = x + v * math.cos(yaw) * self.dt
        y_new = y + v * math.sin(yaw) * self.dt
        yaw_new = yaw + w * self.dt
        v_new = v_meas 
        bias_new = bias
        
        # Normalize Yaw
        yaw_new = (yaw_new + np.pi) % (2 * np.pi) - np.pi
        
        self.x = np.array([x_new, y_new, yaw_new, v_new, bias_new])
        
        # 2. Jacobian F
        F = np.eye(5)
        F[0, 2] = -v * math.sin(yaw) * self.dt
        F[0, 3] = math.cos(yaw) * self.dt
        F[1, 2] = v * math.cos(yaw) * self.dt
        F[1, 3] = math.sin(yaw) * self.dt
        F[2, 4] = -self.dt
        
        self.P = F @ self.P @ F.T + self.Q

    def update(self, gnss_lat, gnss_lon):
        """Update Step using GPS Position + Motion-Derived Heading"""
        if self.origin_lat is None:
            self.origin_lat = gnss_lat
            self.origin_lon = gnss_lon
            return

        # 1. Convert GPS to Meters
        d_lat = math.radians(gnss_lat - self.origin_lat)
        d_lon = math.radians(gnss_lon - self.origin_lon)
        lat0 = math.radians(self.origin_lat)
        
        z_x = d_lon * self.R_EARTH * math.cos(lat0)
        z_y = d_lat * self.R_EARTH
        
        # Initialize history
        if self.prev_z_x is None:
            self.prev_z_x = z_x
            self.prev_z_y = z_y
            return

        # 2. Calculate Motion Vector (Ground Truth Heading)
        dx = z_x - self.prev_z_x
        dy = z_y - self.prev_z_y
        dist = math.sqrt(dx**2 + dy**2)
        
        # Update history
        self.prev_z_x = z_x
        self.prev_z_y = z_y

        # 3. Decide Observation Type
        # Only observe Yaw if we moved > 15cm since last GPS update
        if dist > 0.15:
            meas_yaw = math.atan2(dy, dx)
            z = np.array([z_x, z_y, meas_yaw])
            
            H = np.zeros((3, 5))
            H[0, 0] = 1 # x
            H[1, 1] = 1 # y
            H[2, 2] = 1 # yaw
            R = self.R
        else:
            # Stationary/Slow: Only observe Position
            z = np.array([z_x, z_y])
            H = np.zeros((2, 5))
            H[0, 0] = 1
            H[1, 1] = 1
            R = self.R[:2, :2]

        # 4. Kalman Gain
        y = z - H @ self.x
        
        # Normalize Yaw Residual
        if len(y) == 3:
             y[2] = (y[2] + np.pi) % (2 * np.pi) - np.pi

        S = H @ self.P @ H.T + R
        
        try:
            K = self.P @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return

        self.x = self.x + K @ y
        self.P = (np.eye(5) - K @ H) @ self.P

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        
        # SUBSCRIBERS
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.create_subscription(NavSatFix, '/gnss/fix', self.gnss_callback, 10)
        self.create_subscription(Point, '/target_waypoint', self.target_callback, 10)
        
        # PUBLISHERS
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/ekf/pose', 10)
        
        self.dt = 0.05
        self.ekf = EKF(self.dt)
        
        self.enc_v = 0.0
        self.imu_w = 0.0
        self.target_x = 0.0
        self.target_y = 0.0
        
        # Reduced Gains for Stability
        self.kp_lin = 0.8
        self.kp_ang = 1.0  # Reduced from 1.4 to stop shaking
        
        self.create_timer(self.dt, self.control_loop)

    def odom_callback(self, msg):
        self.enc_v = msg.twist.twist.linear.x

    def imu_callback(self, msg):
        self.imu_w = msg.angular_velocity.z

    def gnss_callback(self, msg):
        self.ekf.update(msg.latitude, msg.longitude)

    def target_callback(self, msg):
        self.target_x = msg.x
        self.target_y = msg.y

    def control_loop(self):
        # 1. Predict
        self.ekf.predict(self.enc_v, self.imu_w)
        
        curr_x = self.ekf.x[0]
        curr_y = self.ekf.x[1]
        curr_yaw = self.ekf.x[2]
        
        # 2. Control
        dx = self.target_x - curr_x
        dy = self.target_y - curr_y
        dist = math.sqrt(dx**2 + dy**2)
        target_ang = math.atan2(dy, dx)
        
        err_ang = target_ang - curr_yaw
        while err_ang > math.pi: err_ang -= 2*math.pi
        while err_ang < -math.pi: err_ang += 2*math.pi
        
        cmd = Twist()
        
        # Wait for Origin
        if self.ekf.origin_lat is None:
            return

        if dist > 0.1:
            if abs(err_ang) > 1.0:
                 cmd.linear.x = 0.2
            else:
                 cmd.linear.x = min(0.8, self.kp_lin * dist)
            # Gentle turn limit
            cmd.angular.z = max(-1.2, min(1.2, self.kp_ang * err_ang))
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        
        self.cmd_pub.publish(cmd)
        self.publish_viz(curr_x, curr_y, curr_yaw)

    def publish_viz(self, x, y, yaw):
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.z = math.sin(yaw/2)
        msg.pose.orientation.w = math.cos(yaw/2)
        self.pose_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()