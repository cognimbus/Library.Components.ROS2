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

        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);
        compressed_publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("camera/image/compressed", 10);
        
        this->open_rtsp_stream();

        timer_ = this->create_wall_timer(std::chrono::milliseconds(33), std::bind(&RtspPublisherNode::timer_callback, this));
        
    }

private:
    void timer_callback()
    {
        if (!cap_.isOpened()) {
            this->open_rtsp_stream();
            return;
        }
        
        cv::Mat frame;
        while (cap_.read(frame)) {
            if (frame.empty()) {
                RCLCPP_WARN(this->get_logger(), "Received empty frame");
                return;
            }

            // Resize the image if necessary
            if (frame.cols != desired_width_ || frame.rows != desired_height_) {
                cv::resize(frame, frame, cv::Size(desired_width_, desired_height_), 0, 0, cv::INTER_LINEAR);
            }

            // Convert OpenCV Mat to ROS Image message
            auto tempMsg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame);
            sensor_msgs::msg::Image::SharedPtr msg = tempMsg.toImageMsg();
            publisher_->publish(*msg);

            sensor_msgs::msg::CompressedImage::SharedPtr compressed_msg = tempMsg.toCompressedImageMsg();

            compressed_publisher_->publish(*compressed_msg);
        } 
    }
    
    void open_rtsp_stream(){
        
        RCLCPP_INFO(this->get_logger(), "Trying to connected to RTSP stream.");
        // Attempt to reconnect
        cap_.open(this->rtsp_url, cv::CAP_FFMPEG);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to RTSP stream.");
        } else {
            RCLCPP_INFO(this->get_logger(), "Successfully connected to RTSP stream.");
            cap_.set(cv::CAP_PROP_BUFFERSIZE, buffer_size_);
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