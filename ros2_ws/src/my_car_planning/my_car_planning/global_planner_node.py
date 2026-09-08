import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import Point


class GlobalPlannerNode(Node):
    def __init__(self):
        super().__init__('global_planner')

        # Map from SLAM
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        # Target waypoint from GNSS waypoint module
        self.target_sub = self.create_subscription(
            Point,
            '/target_waypoint',
            self.target_callback,
            10
        )

        # Current vehicle pose from odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Planned global path
        self.path_pub = self.create_publisher(
            Path,
            '/global_path',
            10
        )

        self.current_map = None
        self.current_pose = None
        self.target_waypoint = None

    def map_callback(self, msg):
        self.current_map = msg

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def target_callback(self, msg):
        self.target_waypoint = msg

        # ==========================================
        # ⚠️ GLOBAL PLANNER CODE GOES HERE ⚠️
        #
        # Inputs:
        #   self.current_map
        #   self.current_pose
        #   self.target_waypoint
        #
        # Task:
        #   Generate the best collision-free path
        #   from the current vehicle position to
        #   the target waypoint.
        #
        # Output:
        #   nav_msgs/Path → /global_path
        # ==========================================


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()