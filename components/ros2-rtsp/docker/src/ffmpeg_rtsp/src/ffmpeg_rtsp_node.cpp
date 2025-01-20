#include <rclcpp/rclcpp.hpp>
#include "rclcpp/rate.hpp"
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
        this->declare_parameter("max_reconnect_attempts", 30);
        this->declare_parameter("reconnect_delay_ms", 5000);
        // Get parameter values
        desired_width_ = this->get_parameter("width").as_int();
        desired_height_ = this->get_parameter("height").as_int();
        buffer_size_ = this->get_parameter("buffer_size").as_int();
        rtsp_url = this->get_parameter("rtsp_url").as_string();

        max_reconnect_attempts_ = this->get_parameter("max_reconnect_attempts").as_int();
        reconnect_delay_ms_ = this->get_parameter("reconnect_delay_ms").as_int();

        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);
        compressed_publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("camera/image/compressed", 10);
        this->open_rtsp_stream();
        timer_ = this->create_wall_timer(std::chrono::milliseconds(33), std::bind(&RtspPublisherNode::timer_callback, this));
    }

private:
    void timer_callback()
    {
        try{
        if (!cap_.isOpened()) {
            handle_disconnection();
            return;
        }
        
        cv::Mat frame;
        if (cap_.read(frame)) {
            if (frame.empty()) {
                RCLCPP_WARN(this->get_logger(), "Received empty frame");
                handle_disconnection();
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
            RCLCPP_INFO(this->get_logger(), "After Publish");
        }else{
            RCLCPP_WARN(this->get_logger(), "cant read frame");
            handle_disconnection();
            return;
        }
        }
        catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "CV Bridge exception: %s", e.what());
            return;
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Unexpected exception: %s", e.what());
            return;
        }
    }
    
    void handle_disconnection() {
        if (reconnect_attempts_ >= max_reconnect_attempts_) {
            RCLCPP_ERROR(this->get_logger(), "Max reconnection attempts reached. Stopping reconnection attempts.");
            timer_->cancel();
            return;
        }

        reconnect_attempts_++;
        RCLCPP_WARN(this->get_logger(), 
            "Connection lost. Attempt %d of %d. Waiting %d ms before reconnecting...",
            reconnect_attempts_, max_reconnect_attempts_, reconnect_delay_ms_);

        if (this->open_rtsp_stream()) {
            RCLCPP_INFO(this->get_logger(), "Successfully reconnected to RTSP stream");
            reconnect_attempts_ = 0;
        }
        // Create a WallRate for the delay since we want steady timing
        rclcpp::WallRate delay(1.0 / (reconnect_delay_ms_ / 1000.0));  // Convert ms to Hz
        delay.sleep();
    }
    bool open_rtsp_stream(){
        
        RCLCPP_INFO(this->get_logger(), "Trying to connected to RTSP stream.");
        // Attempt to reconnect
        cap_.open(this->rtsp_url, cv::CAP_FFMPEG);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to RTSP stream.");
            return false;
        } else {
            RCLCPP_INFO(this->get_logger(), "Successfully connected to RTSP stream.");
            cap_.set(cv::CAP_PROP_BUFFERSIZE, buffer_size_);
            return true;
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
    int reconnect_attempts_;
    int max_reconnect_attempts_;
    int reconnect_delay_ms_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RtspPublisherNode>());
    rclcpp::shutdown();
    return 0;
}