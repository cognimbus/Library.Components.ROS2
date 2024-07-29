import cv2
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge


class RTSP(Node):

    def __init__(self):
        super().__init__("RTSP_NODE")  # Call the base class constructor

        self.declare_parameter("url", "rtsp://127.0.0.1:8554/video")
        self.declare_parameter("image_topic_name", "rtsp_stream_image")
        self.declare_parameter("nobuffer", True)
        self.rtsp_stream_url = self.get_parameter("url").get_parameter_value().string_value
        self.image_topic_name = self.get_parameter("image_topic_name").get_parameter_value().string_value
        self.nobuffer = self.get_parameter("nobuffer").get_parameter_value().bool_value


        if self.nobuffer:
            os.environ['OPENCV_FFMPEG_CAPTURE_OPTIONS'] = 'prefer_tcp;fflags;nobuffer'
            self.get_logger().info("Construct the FFmpeg command with nobuffer flag")
        # Construct the FFmpeg command with nobuffer flag

        # Create a VideoCapture object
        self.cap = cv2.VideoCapture(self.rtsp_stream_url,cv2.CAP_FFMPEG)

        # Initialize the ROS publisher for the image topic
        self.image_pub = self.create_publisher(Image, f'{self.image_topic_name}', 10)
        self.compressed_image_pub = self.create_publisher(CompressedImage, f'{self.image_topic_name}/compressed', 10)

        # Initialize the cv_bridge object
        self.bridge = CvBridge()

        while rclpy.ok():
            try:
                ret, frame = self.cap.read()
                if not ret:
                    self.get_logger().warn("Failed to read frame. Reconnecting...")
                    self.cap.release()
                    self.cap = cv2.VideoCapture(self.rtsp_stream_url,cv2.CAP_FFMPEG)
                    continue

                # Convert and publish as before
                ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                ros_compressed_image = self.bridge.cv2_to_compressed_imgmsg(frame, "jpeg")
                self.image_pub.publish(ros_image)
                self.compressed_image_pub.publish(ros_compressed_image)

            except Exception as e:
                self.get_logger().error(f"Error processing frame: {str(e)}")

            # Add a small delay to prevent CPU overuse
            rclpy.spin_once(self, timeout_sec=0.01)

        # Release the VideoCapture object
        self.cap.release()


def main(args=None):
    rclpy.init()
    rtsp_node = RTSP()
    rclpy.spin(rtsp_node)


if __name__ == '__main__':
    main()
