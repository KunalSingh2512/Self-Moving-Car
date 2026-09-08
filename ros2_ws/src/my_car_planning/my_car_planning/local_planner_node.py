import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from my_car_interfaces.msg import ObstacleInfo


class LocalPlannerNode(Node):
    def __init__(self):
        super().__init__('local_planner')

        # Global path
        self.path_sub = self.create_subscription(
            Path,
            '/global_path',
            self.path_callback,
            10
        )

        # YOLO obstacle information
        self.yolo_sub = self.create_subscription(
            ObstacleInfo,
            '/yolo_obstacles',
            self.yolo_callback,
            10
        )

        # Lane lateral error
        # Unit: meters
        # +ve -> right
        # -ve -> left
        # 0    -> centered
        self.lane_sub = self.create_subscription(
            Float32,
            '/lane_offset',
            self.lane_callback,
            10
        )

        # Publishing velocity commands
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Store latest lane information
        self.lane_offset = 0.0

    def lane_callback(self, msg):
        # Receive the latest lateral lane error.
        # Saloni you can use this value in the actual controller.
        self.lane_offset = msg.data

    def yolo_callback(self, msg):
        # Now Saloni you can access specific variables!
        detected_object = msg.object_label
        distance_to_object = msg.distance
        is_it_passable = msg.is_passable

        # ==========================================
        # ⚠️ SALONI: WRITE DECISION LOGIC HERE ⚠️
        #
        # Inputs available:
        # - detected_object
        # - distance_to_object
        # - msg.height
        # - is_it_passable
        # - self.lane_offset
        #
        # ==========================================

    def path_callback(self, msg):
        # Follow the path when there are no obstacles.
        #
        # self.lane_offset contains the latest lane error.
        #
        # ==========================================
        # ⚠️ SALONI: WRITE LOCAL PLANNING / CONTROL HERE ⚠️
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