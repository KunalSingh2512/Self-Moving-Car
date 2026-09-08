import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

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
        # YOLO OBSTACLE INFORMATION
        # ==========================================
        self.yolo_sub = self.create_subscription(
            ObstacleInfo,
            '/yolo_obstacles',
            self.yolo_callback,
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
        # VELOCITY COMMAND
        # ==========================================
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # ==========================================
        # STORED STATE
        # ==========================================
        self.current_path = None
        self.lane_offset = 0.0

        self.current_pose = None
        self.current_linear_velocity = 0.0
        self.current_angular_velocity = 0.0

        self.detected_object = None
        self.distance_to_object = None
        self.object_height = None
        self.is_object_passable = True

    # ==========================================
    # LANE CALLBACK
    # ==========================================
    def lane_callback(self, msg):
        self.lane_offset = msg.data

    # ==========================================
    # YOLO CALLBACK
    # ==========================================
    def yolo_callback(self, msg):
        self.detected_object = msg.object_label
        self.distance_to_object = msg.distance
        self.object_height = msg.height
        self.is_object_passable = msg.is_passable

        # ==========================================
        # ⚠️ SALONI: WRITE OBSTACLE DECISION LOGIC HERE
        #
        # Available information:
        #   self.detected_object
        #   self.distance_to_object
        #   self.object_height
        #   self.is_object_passable
        #   self.lane_offset
        # ==========================================

    # ==========================================
    # PATH CALLBACK
    # ==========================================
    def path_callback(self, msg):
        self.current_path = msg

        # ==========================================
        # ⚠️ SALONI: WRITE LOCAL PLANNING / CONTROL HERE
        #
        # Inputs available:
        #   self.current_path
        #   self.lane_offset
        #   self.current_pose
        #   self.current_linear_velocity
        #   self.current_angular_velocity
        #   self.detected_object
        #   self.distance_to_object
        #   self.object_height
        #   self.is_object_passable
        #
        # Output:
        #   geometry_msgs/Twist → /cmd_vel
        # ==========================================

    # ==========================================
    # ODOMETRY CALLBACK
    # ==========================================
    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

        self.current_linear_velocity = msg.twist.twist.linear.x
        self.current_angular_velocity = msg.twist.twist.angular.z


def main(args=None):
    rclpy.init(args=args)

    node = LocalPlannerNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()