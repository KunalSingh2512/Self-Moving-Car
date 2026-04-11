import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String  # We will upgrade this to your Custom Message later!
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
            
        # Publish the AI output for Saloni's Local Planner
        self.publisher_ = self.create_publisher(String, '/yolo_obstacles', 10)

    def image_callback(self, msg):
        # 1. Convert ROS Image to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # ==========================================
        # ⚠️ KUNAL: WRITE YOLO LOGIC HERE ⚠️
        # Task: Run inference on cv_image. 
        # Label objects as "Passable" vs "Critical".
        # ==========================================
        
        # Example of running the model:
        # results = self.model(cv_image)
        # for r in results:
        #     boxes = r.boxes
        #     ... your filtering logic here ...
        
        output_label = "Placeholder_Label" 
        
        # 2. Publish the result to the ROS graph
        msg_out = String()
        msg_out.data = output_label
        self.publisher_.publish(msg_out)

        # 3. Optional: Display the YOLO vision on your laptop screen
        # annotated_frame = results[0].plot()
        # cv2.imshow("YOLOv8 Vision", annotated_frame)
        # cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()