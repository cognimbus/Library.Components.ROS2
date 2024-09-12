# image
import cv2

# ROS
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge

# YOLO
from ultralytics import YOLO
import json
import torch
import numpy as np
import torchvision.transforms.functional as F


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
        
        self.declare_parameter("threshold", 0.0)
        self.threshold = self.get_parameter("threshold").get_parameter_value().double_value
        
        self.cv_bridge = CvBridge()
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.get_logger().info(f"YOLO Using device: {self.device}")
        self.model = YOLO('/upcreate3_object_detection_ws/yolov8n.pt')  # Load the YOLOv8 model
        self.color_map:dict = None
        self.load_color_map()
        self.get_logger().info(f"YOLO initialization complete")
        
    def load_color_map(self) -> None:
        with open('/upcreate3_object_detection_ws/data.json', 'r') as file:
            self.color_map = json.load(file)
                
    def get_color(self, class_id):
        class_id = str(class_id)
        if class_id not in self.color_map:
            self.color_map[class_id] = tuple(np.random.randint(0, 255, 3).tolist())
        return self.color_map[class_id]

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convert the image to a tensor and move it to the appropriate device
        image_tensor = torch.from_numpy(cv_image).permute(2, 0, 1).float().div(255.0).unsqueeze(0)
        image_tensor = image_tensor.to(self.device)

        # Perform YOLOv8 detection
        with torch.no_grad():
            results = self.model(image_tensor)

        detected_classes = set()

        # Create a copy of the image tensor for drawing
        overlay = image_tensor.clone()

        for result in results:
            boxes = result.boxes.xyxy
            classes = result.boxes.cls
            confidences = result.boxes.conf

            for box, cls, conf in zip(boxes, classes, confidences):
                if conf < self.threshold:
                    continue

                x1, y1, x2, y2 = box.int()
                color = torch.tensor(self.get_color(int(cls))).to(self.device) / 255.0
                label = f'{result.names[int(cls)]} {conf:.2f}'

                # Draw filled rectangle on overlay
                overlay[0, :, y1:y2, x1:x2] = color.view(3, 1, 1)

                # Add class to detected classes
                detected_classes.add(result.names[int(cls)])

        # Blend the overlay with the original image
        alpha = 0.4
        image_tensor = image_tensor * (1 - alpha) + overlay * alpha

        # Convert tensor back to numpy for text rendering (text rendering on GPU is complex)
        cv_image = (image_tensor[0].permute(1, 2, 0).cpu().numpy() * 255).astype(np.uint8)

        # Draw text labels
        for result in results:
            boxes = result.boxes.xyxy.cpu().numpy().astype(int)
            classes = result.boxes.cls.cpu().numpy().astype(int)
            confidences = result.boxes.conf.cpu().numpy()

            for box, cls, conf in zip(boxes, classes, confidences):
                if conf < self.threshold:
                    continue

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
