import rclpy
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image,CompressedImage
from cv_bridge import CvBridge
from ultralytics import YOLO
import torch
import numpy as np

class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.publisher = self.create_publisher(Image, '/detected_objects', 10)
        self.compressed_image_publisher = self.create_publisher(CompressedImage, '/detected_objects/compressed', 10)
        self.cv_bridge = CvBridge()
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.get_logger().info(f"Using device: {self.device}")
        self.model = YOLO('/yolo_detector_ws/yolov8n.pt')  # Load the YOLOv8 model

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Perform YOLOv8 detection
        results = self.model(cv_image)

        # Draw bounding boxes and labels on the image
        for result in results:
            boxes = result.boxes.xyxy.cpu().numpy().astype(int)
            classes = result.boxes.cls.cpu().numpy().astype(int)
            confidences = result.boxes.conf.cpu().numpy()

            for box, cls, conf in zip(boxes, classes, confidences):
                x1, y1, x2, y2 = box
                label = f'{result.names[cls]} {conf:.2f}'
                cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(cv_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

        # Convert the processed image back to ROS Image message
        output_msg = self.cv_bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        output_msg.header = msg.header  # Preserve the original header

        # Publish the processed image
        self.publisher.publish(output_msg)
        output_msg_compressed = self.cv_bridge.cv2_to_compressed_imgmsg(cv_image,'jpeg')
        self.compressed_image_publisher.publish(output_msg_compressed)

def main(args=None):
    rclpy.init(args=args)
    yolo_detector_node = YoloDetectorNode()
    rclpy.spin(yolo_detector_node)
    yolo_detector_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()