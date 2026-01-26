#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
import math

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        
        # CONSTANTS
        self.MAX_LINEAR_VEL = 1.0  # m/s (Do not exceed 2.0)
        self.MAX_ANGULAR_VEL = 0.5 # rad/s (Slow turns for high friction tires)
        self.ACCEL_RATE = 0.05     # Ramp up speed by this amount per loop (Soft Start)
        
        # SUBSCRIBERS
        
        self.odom_sub = self.create_subscription(
            Odometry, 
            '/odometry/filtered', 
            self.odom_callback, 
            10
        )
        
        self.target_sub = self.create_subscription(
            Point, 
            '/target_waypoint', 
            self.target_callback, 
            10
        )
        
        # PUBLISHER
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # VARIABLES
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.target_x = 0.0
        self.target_y = 0.0
        self.current_speed = 0.0 

    def odom_callback(self, msg):
        # 1. Get Position from EKF
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # 2. Get Orientation (Yaw) from EKF
        # We extract yaw from the quaternion here directly
        q_x = msg.pose.pose.orientation.x
        q_y = msg.pose.pose.orientation.y
        q_z = msg.pose.pose.orientation.z
        q_w = msg.pose.pose.orientation.w

        siny_cosp = 2 * (q_w * q_z + q_x * q_y)
        cosy_cosp = 1 - 2 * (q_y * q_y + q_z * q_z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def target_callback(self, msg):
        self.target_x = msg.x
        self.target_y = msg.y
        self.control_loop()

    def control_loop(self):
        cmd = Twist()
        
        # ==========================================
        # TODO: ABHAY'S AREA (EDIT BELOW)
        # ==========================================
        # Instructions:
        # 1. Calculate Distance to Target
        # 2. Calculate Heading Error (Target Angle - self.current_yaw)
        # 3. Use Ramp-Up Logic for Linear Velocity (Drive)
        
        # Example Ramp-Up Skeleton:
        # if desired_speed > self.current_speed:
        #     self.current_speed += self.ACCEL_RATE
        # else:
        #     self.current_speed = desired_speed
            
        # cmd.linear.x = self.current_speed
        # cmd.angular.z = ... 
        
        # Temp Logic (Replace this):
        cmd.linear.x = 0.0 
        cmd.angular.z = 0.0
        
        # ==========================================
        # END OF ABHAY'S AREA
        # ==========================================
        
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()