#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

class MockRobot(Node):
    def __init__(self):
        super().__init__('mock_robot')
        
        # Publishers (Pretending to be Original Gazebo Simulation)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu', 10)
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Mock Robot Started: Publishing fake sensor data...')

    def timer_callback(self):
        # 1. Publish Fake Odometry (Static Position)
        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = 0.0
        odom.pose.pose.position.y = 0.0
        self.odom_pub.publish(odom)

        # 2. Publish Fake IMU (Static Orientation)
        imu = Imu()
        imu.header.frame_id = "base_link"
        imu.orientation.w = 1.0 # Neutral orientation
        self.imu_pub.publish(imu)

def main(args=None):
    rclpy.init(args=args)
    node = MockRobot()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()