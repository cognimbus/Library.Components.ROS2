import rclpy
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from ultralytics import YOLO
import json
import torch
import numpy as np
from std_msgs.msg import String

class YoloDetectorNode(Node):
    def __init__(self):        
        super().__init__('yolo_detector_node')
        
        self.get_logger().info(f"Initializing YOLO")
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.publisher = self.create_publisher(Image, '/detected_objects', 10)
        self.compressed_image_publisher = self.create_publisher(CompressedImage, '/detected_objects/compressed', 10)
        self.class_publisher = self.create_publisher(String, '/detected_classes', 10)
        self.cv_bridge = CvBridge()
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.get_logger().info(f"YOLO Using device: {self.device}")
        self.model = YOLO('/yolo_detector_ws/yolov8n.pt')  # Load the YOLOv8 model
        self.color_map:dict = None
        self.load_color_map()
        self.get_logger().info(f"YOLO initialization complete")
        

    def load_color_map(self) -> None:
        with open('/yolo_detector_ws/data.json', 'r') as file:
            self.color_map = json.load(file)
                

    def get_color(self, class_id):
        if class_id not in self.color_map:
            self.color_map[class_id] = tuple(np.random.randint(0, 255, 3).tolist())
        return self.color_map[class_id]

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Perform YOLOv8 detection
        results = self.model(cv_image)

        detected_classes = set()

        # Create a copy of the image for drawing
        overlay = cv_image.copy()

        # Draw bounding boxes and labels on the image
        for result in results:
            boxes = result.boxes.xyxy.cpu().numpy().astype(int)
            classes = result.boxes.cls.cpu().numpy().astype(int)
            confidences = result.boxes.conf.cpu().numpy()

            for box, cls, conf in zip(boxes, classes, confidences):
                x1, y1, x2, y2 = box
                color = self.get_color(cls)
                label = f'{result.names[cls]} {conf:.2f}'
                
                # Draw filled rectangle on overlay
                cv2.rectangle(overlay, (x1, y1), (x2, y2), color, -1)
                
                # Add class to detected classes
                detected_classes.add(result.names[cls])

            # Blend the overlay with the original image
            alpha = 0.4  # Transparency factor
            cv_image = cv2.addWeighted(overlay, alpha, cv_image, 1 - alpha, 0)

            # Draw text labels
            for box, cls, conf in zip(boxes, classes, confidences):
                x1, y1, x2, y2 = box
                color = self.get_color(cls)
                label = f'{result.names[cls]} {conf:.2f}'
                cv2.putText(cv_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)

        # Convert the processed image back to ROS Image message
        output_msg = self.cv_bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        output_msg.header = msg.header  # Preserve the original header

        # Publish the processed image
        self.publisher.publish(output_msg)
        output_msg_compressed = self.cv_bridge.cv2_to_compressed_imgmsg(cv_image, 'jpeg')
        self.compressed_image_publisher.publish(output_msg_compressed)

        # Publish detected classes
        classes_msg = String()
        classes_msg.data = ', '.join(detected_classes)
        self.class_publisher.publish(classes_msg)

def main(args=None):
    rclpy.init(args=args)
    yolo_detector_node = YoloDetectorNode()
    rclpy.spin(yolo_detector_node)
    yolo_detector_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
