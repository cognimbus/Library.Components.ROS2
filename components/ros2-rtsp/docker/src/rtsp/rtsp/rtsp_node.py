import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge


class RTSP(Node):

    def __init__(self):
        super().__init__("RTSP_NODE")  # Call the base class constructor

        self.declare_parameter("url", "rtsp://127.0.0.1:8554/video")
        self.declare_parameter("image_topic_name", "rtsp_stream_image")

        self.rtsp_stream_url = self.get_parameter("url").get_parameter_value().string_value
        self.image_topic_name = self.get_parameter("image_topic_name").get_parameter_value().string_value
        # Create a VideoCapture object
        self.cap = cv2.VideoCapture(self.rtsp_stream_url)
        flag = False
        while not flag:
            # Check if the VideoCapture object was successfully opened
            if not self.cap.isOpened():
                self.get_logger().error("Failed to open rtsp stream. " + self.rtsp_stream_url)
                self.cap = cv2.VideoCapture(self.rtsp_stream_url)
            else:
                flag = True

        # Initialize the ROS publisher for the image topic
        self.image_pub = self.create_publisher(Image, f'{self.image_topic_name}', 10)
        self.compressed_image_pub = self.create_publisher(CompressedImage, f'{self.image_topic_name}/compressed', 10)

        # Initialize the cv_bridge object
        self.bridge = CvBridge()

        # Read frames from the rtsp stream and publish them to the ROS image topic
        while self.cap.isOpened():
            ret, frame = self.cap.read()

            if not ret:
                self.get_logger().error("Failed to read frame from rtsp stream.")
                continue

            # Convert the OpenCV image to a ROS image message
            ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            ros_compressed_image = self.bridge.cv2_to_compressed_imgmsg(frame, "jpeg")

            # Publish the ROS image message
            self.image_pub.publish(ros_image)
            self.compressed_image_pub.publish(ros_compressed_image)

        # Release the VideoCapture object
        self.cap.release()


def main(args=None):
    rclpy.init()
    rtsp_node = RTSP()
    rclpy.spin(rtsp_node)


if __name__ == '__main__':
    main()
