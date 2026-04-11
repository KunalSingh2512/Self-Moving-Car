import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2

class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__('lane_detector')
        self.bridge = CvBridge()
        
        # Listening to the camera
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        # Publishing the offset for the planner
        self.publisher_ = self.create_publisher(Float32, '/lane_offset', 10)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # ==========================================
        # ⚠️ HRITVIK: WRITE OPENCV CODE HERE ⚠️
        # Task: Identify road boundaries in simulation.
        # Output: Calculate 'lane_center_offset' (float).
        # ==========================================
        lane_center_offset = 0.0 # Replace this with your math
        
        # Publish the result
        msg_out = Float32()
        msg_out.data = float(lane_center_offset)
        self.publisher_.publish(msg_out)

        # Optional: Show the frame for debugging
        # cv2.imshow("Lane Detection", cv_image)
        # cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()