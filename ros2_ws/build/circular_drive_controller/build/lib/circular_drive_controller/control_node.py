#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point, Twist
import math

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        
        # Subscribers (Inputs from Robot and Mission Planner)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10) 
        self.target_sub = self.create_subscription(Point, '/target_waypoint', self.target_callback, 10)
        
        # Publisher (Output to Robot Wheels)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Internal Variables
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.target_x = 0.0
        self.target_y = 0.0

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def imu_callback(self, msg):
        # Placeholder for IMU orientation logic if needed
        pass

    def target_callback(self, msg):
        self.target_x = msg.x
        self.target_y = msg.y
        # Trigger the control loop whenever a new target arrives
        self.control_loop()

    def control_loop(self):
        cmd = Twist()
        
        # ==========================================
        # TODO: ABHAY'S AREA (EDIT BELOW)
        # ==========================================
        # Instructions:
        # 1. Calculate Distance Error = sqrt((Tx-Cx)^2 + (Ty-Cy)^2)
        # 2. Calculate Heading Error = atan2(Ty-Cy, Tx-Cx) - current_yaw
        # 3. Use P-Controller to set linear.x and angular.z
        
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