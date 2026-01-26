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
        
        self.MAX_LINEAR_VEL = 1.0  # m/s (Do not exceed 2.0)
        self.MAX_ANGULAR_VEL = 0.5 # rad/s (Slow turns for high friction tires)
        self.ACCEL_RATE = 0.05     # Ramp up speed by this amount per loop (Soft Start)
        
        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.target_sub = self.create_subscription(Point, '/target_waypoint', self.target_callback, 10)
        
        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Internal Variables
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.target_x = 0.0
        self.target_y = 0.0
        
        # For Ramp-Up Logic
        self.current_speed = 0.0 

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def imu_callback(self, msg):
        # ADDED: Convert Quaternion (x,y,z,w) to Euler Yaw (Heading)
        # Abhay needs this 'yaw' to know which way the car is facing.
        q_x = msg.orientation.x
        q_y = msg.orientation.y
        q_z = msg.orientation.z
        q_w = msg.orientation.w

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
        # Calculate Error Distance & Heading.
        
        # RAMP UP LOGIC (Soft Start)
        # Instead of jumping to max speed, we increase slowly.
        # Example Logic:
                
        # if desired_speed > self.current_speed:
        #     self.current_speed += self.ACCEL_RATE
        # else:
        #     self.current_speed = desired_speed
            
        # if self.current_speed > self.MAX_LINEAR_VEL:
        #     self.current_speed = self.MAX_LINEAR_VEL
            
        # cmd.linear.x = self.current_speed
        # cmd.angular.z = ... (Limit this to self.MAX_ANGULAR_VEL)
        
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