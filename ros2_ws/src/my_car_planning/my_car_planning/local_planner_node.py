import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from my_car_interfaces.msg import ObstacleInfo


class LocalPlannerNode(Node):
    def __init__(self):
        super().__init__('local_planner')

        # ==========================================
        # GLOBAL PATH
        # ==========================================
        self.path_sub = self.create_subscription(
            Path,
            '/global_path',
            self.path_callback,
            10
        )

        # ==========================================
        # LANE OFFSET
        # Unit: meters
        # +ve -> right
        # -ve -> left
        #  0  -> centered
        # ==========================================
        self.lane_sub = self.create_subscription(
            Float32,
            '/lane_offset',
            self.lane_callback,
            10
        )

        # ==========================================
        # VEHICLE ODOMETRY
        # ==========================================
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # ==========================================
        # 3D LiDAR
        #
        # This is the PRIMARY obstacle-safety input
        # for the system when YOLO is unavailable.
        # ==========================================
        self.lidar_sub = self.create_subscription(
            PointCloud2,
            '/scan',
            self.lidar_callback,
            10
        )

        # ==========================================
        # YOLO OBSTACLES
        #
        # OPTIONAL:
        # The vehicle does NOT depend on YOLO.
        # This subscription only provides semantic
        # information when the YOLO node is available.
        # ==========================================
        self.yolo_sub = self.create_subscription(
            ObstacleInfo,
            '/yolo_obstacles',
            self.yolo_callback,
            10
        )

        # ==========================================
        # VELOCITY COMMAND
        # ==========================================
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # ==========================================
        # CURRENT STATE
        # ==========================================
        self.current_path = None

        self.lane_offset = 0.0

        self.current_pose = None
        self.current_linear_velocity = 0.0
        self.current_angular_velocity = 0.0

        # ==========================================
        # YOLO STATE
        #
        # None means YOLO is not currently providing
        # semantic obstacle information.
        # ==========================================
        self.yolo_available = False
        self.detected_object = None
        self.distance_to_object = None
        self.object_height = None
        self.is_object_passable = True

        # ==========================================
        # LiDAR OBSTACLE STATE
        # ==========================================
        self.lidar_obstacle_detected = False
        self.lidar_nearest_distance = float('inf')

        # Front safety limits
        self.lidar_min_range = 0.30
        self.lidar_max_range = 12.0

        # Only consider points in the forward region.
        # x > 0 means in front of base_link.
        self.front_angle_limit = math.radians(45.0)

        # Simple safety distance for the prototype.
        self.safety_distance = 1.0

        self.get_logger().info(
            'Local Planner started. '
            '3D LiDAR is available as the primary '
            'obstacle-safety source. YOLO is optional.'
        )

    # ==========================================
    # LANE CALLBACK
    # ==========================================
    def lane_callback(self, msg):
        self.lane_offset = msg.data

    # ==========================================
    # ODOMETRY CALLBACK
    # ==========================================
    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

        self.current_linear_velocity = (
            msg.twist.twist.linear.x
        )

        self.current_angular_velocity = (
            msg.twist.twist.angular.z
        )

    # ==========================================
    # 3D LiDAR CALLBACK
    # ==========================================
    def lidar_callback(self, msg):
        """
        Read the PointCloud2 data and find the nearest
        valid obstacle point in the forward 45-degree
        sector.

        This is intentionally a simple safety layer.
        It is NOT the final obstacle avoidance algorithm.
        """

        nearest_distance = float('inf')

        try:
            points = point_cloud2.read_points(
                msg,
                field_names=('x', 'y', 'z'),
                skip_nans=True
            )

            for point in points:
                x, y, z = point

                # Ignore points too close or too far.
                distance = math.sqrt(
                    x * x + y * y + z * z
                )

                if distance < self.lidar_min_range:
                    continue

                if distance > self.lidar_max_range:
                    continue

                # Ignore points behind the vehicle.
                if x <= 0.0:
                    continue

                # Horizontal angle relative to vehicle forward axis.
                angle = math.atan2(y, x)

                if abs(angle) > self.front_angle_limit:
                    continue

                # Ignore points too far below the vehicle.
                # This prevents ground points from being treated
                # as obstacles in the simple prototype safety layer.
                if z < -0.5:
                    continue

                nearest_distance = min(
                    nearest_distance,
                    distance
                )

            self.lidar_nearest_distance = nearest_distance

            if nearest_distance < self.safety_distance:
                self.lidar_obstacle_detected = True
            else:
                self.lidar_obstacle_detected = False

        except Exception as exc:
            self.get_logger().warn(
                f'LiDAR processing failed: {exc}'
            )

    # ==========================================
    # YOLO CALLBACK
    # ==========================================
    def yolo_callback(self, msg):
        self.yolo_available = True

        self.detected_object = msg.object_label
        self.distance_to_object = msg.distance
        self.object_height = msg.height
        self.is_object_passable = msg.is_passable

        # ==========================================
        # YOLO DATA RECEIVED
        #
        # YOLO is an optional perception source.
        # The local planner can use this information
        # when YOLO is available.
        # ==========================================

    # ==========================================
    # GLOBAL PATH CALLBACK
    # ==========================================
    def path_callback(self, msg):
        self.current_path = msg

        # ==========================================
        # ⚠️ SALONI: WRITE LOCAL PLANNING / CONTROL HERE
        #
        # Available inputs:
        #
        # self.current_path
        # self.lane_offset
        # self.current_pose
        # self.current_linear_velocity
        # self.current_angular_velocity
        #
        # LiDAR:
        # self.lidar_obstacle_detected
        # self.lidar_nearest_distance
        #
        # Optional YOLO:
        # self.yolo_available
        # self.detected_object
        # self.distance_to_object
        # self.object_height
        # self.is_object_passable
        #
        # Output:
        # geometry_msgs/Twist → /cmd_vel
        #
        # ==========================================
        pass


def main(args=None):
    rclpy.init(args=args)

    node = LocalPlannerNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()