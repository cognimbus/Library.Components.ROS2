#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class RtspPublisherNode : public rclcpp::Node
{
public:
    RtspPublisherNode() : Node("ffmpeg_rtsp_node")
    {
        RCLCPP_INFO(this->get_logger(), "Initializing RtspStreamer");
        
        this->declare_parameter("width", 1920);
        this->declare_parameter("height", 1080);
        this->declare_parameter("rtsp_url", "rtsp://127.0.0.1:8554/video");
        this->declare_parameter("buffer_size", 1);
        // Get parameter values
        desired_width_ = this->get_parameter("width").as_int();
        desired_height_ = this->get_parameter("height").as_int();
        buffer_size_ = this->get_parameter("buffer_size").as_int();
        rtsp_url = this->get_parameter("rtsp_url").as_string();

        // RCLCPP_INFO(this->get_logger(), rtsp_url.c_str());
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);
        compressed_publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("camera/image/compressed", 10);
        
        cap_.open(rtsp_url, cv::CAP_FFMPEG);
        
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open RTSP stream.");
            return;
        }
        
        // Set buffer size to 1 to minimize latency
        cap_.set(cv::CAP_PROP_BUFFERSIZE, buffer_size_);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(33), std::bind(&RtspPublisherNode::timer_callback, this));
        
    }

private:
    void timer_callback()
    {
        cv::Mat frame;
        if (cap_.read(frame)) {
            if (frame.empty()) {
                RCLCPP_WARN(this->get_logger(), "Received empty frame");
                return;
            }

            // Resize the image if necessary
            if (frame.cols != desired_width_ || frame.rows != desired_height_) {
                cv::resize(frame, frame, cv::Size(desired_width_, desired_height_), 0, 0, cv::INTER_LINEAR);
            }

            // Convert OpenCV Mat to ROS Image message
            sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
            publisher_->publish(*msg);

            sensor_msgs::msg::CompressedImage compressed_msg;
            compressed_msg.header.stamp = this->now();
            compressed_msg.format = "jpeg";
            cv::imencode(".jpg", frame, compressed_msg.data);
            compressed_publisher_->publish(compressed_msg);
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to read frame from stream");
            
            // Attempt to reconnect
            cap_.release();
            cap_.open(this->rtsp_url, cv::CAP_FFMPEG);
            if (!cap_.isOpened()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to reconnect to RTSP stream.");
            } else {
                RCLCPP_INFO(this->get_logger(), "Successfully reconnected to RTSP stream.");
                cap_.set(cv::CAP_PROP_BUFFERSIZE, buffer_size_);
            }
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string rtsp_url;
    cv::VideoCapture cap_;
    int desired_width_;
    int desired_height_;
    int buffer_size_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RtspPublisherNode>());
    rclcpp::shutdown();
    return 0;
}