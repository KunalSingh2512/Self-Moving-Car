import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point

class WaypointNode(Node):
    def __init__(self):
        super().__init__('waypoint_node')
        
        # Listening to GPS
        self.gps_sub = self.create_subscription(NavSatFix, '/gnss/fix', self.gps_callback, 10)
        # Publishing X, Y coordinates
        self.target_pub = self.create_publisher(Point, '/target_waypoint', 10)

    def gps_callback(self, msg):
        # ==========================================
        # ⚠️ AYUSH: WRITE WAYPOINT LOGIC HERE ⚠️
        # Task: Convert latitude/longitude (msg.latitude, msg.longitude)
        # into local X, Y map coordinates for the planner.
        # ==========================================
        target_x = 0.0
        target_y = 0.0
        
        out_msg = Point()
        out_msg.x = target_x
        out_msg.y = target_y
        self.target_pub.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()