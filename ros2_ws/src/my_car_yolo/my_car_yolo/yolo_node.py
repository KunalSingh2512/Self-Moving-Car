import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from my_car_interfaces.msg import ObstacleInfo
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        self.bridge = CvBridge()
        
        # Load the pre-trained YOLOv8 model (downloads automatically the first time)
        self.get_logger().info("Loading YOLOv8 model...")
        self.model = YOLO('yolov8n.pt') # 'n' is for nano, the fastest version
        self.get_logger().info("YOLOv8 Loaded Successfully!")

        # Subscribe to the camera Rosbag feed
        self.subscription = self.create_subscription(
            Image, 
            '/camera/image_raw', 
            self.image_callback, 
            10)
            
        # Publish the AI output using YOUR custom interface
        self.publisher_ = self.create_publisher(ObstacleInfo, '/yolo_obstacles', 10)

    def image_callback(self, msg):
        # 1. Convert ROS Image to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # ==========================================
        # ⚠️ KUNAL: WRITE YOLO LOGIC HERE LATER ⚠️
        # Task: Run inference on cv_image. 
        # Label objects as "Passable" vs "Critical".
        # ==========================================
        
        # 2. Publish dummy data to keep the ROS 2 graph happy 
        #    until you write the real YOLO code above.
        msg_out = ObstacleInfo()
        msg_out.object_label = "Waiting_For_YOLO" 
        msg_out.distance = 0.0          
        msg_out.height = 0.0            
        msg_out.is_passable = True      
        
        self.publisher_.publish(msg_out)

def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()