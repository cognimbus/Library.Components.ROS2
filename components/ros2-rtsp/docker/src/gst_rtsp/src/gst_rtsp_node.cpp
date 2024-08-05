#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

using namespace std::chrono_literals;

class RtspStreamer : public rclcpp::Node
{
public:
    RtspStreamer() : Node("rtsp_streamer")
    {
        RCLCPP_INFO(this->get_logger(), "Initializing RtspStreamer");
        
        this->declare_parameter("width", 1920);
        this->declare_parameter("height", 1080);
        this->declare_parameter("rtsp_url", "rtsp://127.0.0.1:8554/video");
        this->declare_parameter("max_buffers", 1);
        // Get parameter values
        int width = this->get_parameter("width").as_int();
        int height = this->get_parameter("height").as_int();
        int max_buffers = this->get_parameter("max_buffers").as_int();
        std::string rtsp_url = this->get_parameter("rtsp_url").as_string();

        raw_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);
        compressed_publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("camera/image/compressed", 10);
        
        RCLCPP_INFO(this->get_logger(), "Initializing GStreamer");
        gst_init(nullptr, nullptr);
        
        RCLCPP_INFO(this->get_logger(), "Creating GStreamer pipeline");
        GError *error = nullptr;
        std::string pipeline_str = 
            "rtspsrc location=" + rtsp_url + " latency=1 ! "
            "rtph264depay ! h264parse ! avdec_h264 ! "
            "videoconvert ! video/x-raw,format=BGR ! videoscale ! "
            "video/x-raw,width=" + std::to_string(width) + ",height=" + std::to_string(height) + " ! "
            "appsink name=mysink";
        
        pipeline_ = gst_parse_launch(pipeline_str.c_str(), &error);

        if (error != nullptr) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create pipeline: %s", error->message);
            g_clear_error(&error);
            throw std::runtime_error("Failed to create GStreamer pipeline");
        }
        
        RCLCPP_INFO(this->get_logger(), "Getting appsink from pipeline");
        auto appsink = GST_APP_SINK(gst_bin_get_by_name(GST_BIN(pipeline_), "mysink"));
        if (!appsink) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get appsink from pipeline");
            throw std::runtime_error("Failed to get appsink from pipeline");
        }
        
        gst_app_sink_set_emit_signals(appsink, TRUE);
        gst_app_sink_set_max_buffers(appsink, max_buffers);
        gst_app_sink_set_drop(appsink, TRUE);
        
        RCLCPP_INFO(this->get_logger(), "Connecting new-sample signal");
        g_signal_connect(appsink, "new-sample", G_CALLBACK(on_new_sample), this);
        
        RCLCPP_INFO(this->get_logger(), "Setting pipeline to PLAYING state");
        GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
        if (ret == GST_STATE_CHANGE_FAILURE) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set pipeline to PLAYING state");
            throw std::runtime_error("Failed to set pipeline to PLAYING state");
        }
        
        timer_ = this->create_wall_timer(std::chrono::milliseconds(33), 
                                        std::bind(&RtspStreamer::timer_callback, this));
        RCLCPP_INFO(this->get_logger(), "RtspStreamer initialization complete");
    }

    ~RtspStreamer()
    {
        gst_element_set_state(pipeline_, GST_STATE_NULL);
        gst_object_unref(pipeline_);
    }

private:
    static GstFlowReturn on_new_sample(GstAppSink *sink, gpointer user_data)
    {
        auto *self = static_cast<RtspStreamer*>(user_data);
        GstSample *sample = gst_app_sink_pull_sample(sink);
        if (sample) {
            GstBuffer *buffer = gst_sample_get_buffer(sample);
            GstMapInfo map;
            if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {
                GstCaps *caps = gst_sample_get_caps(sample);
                GstStructure *structure = gst_caps_get_structure(caps, 0);
                int width, height;
                gst_structure_get_int(structure, "width", &width);
                gst_structure_get_int(structure, "height", &height);
                
                RCLCPP_INFO(self->get_logger(), "Frame received: width=%d, height=%d, size=%zu", width, height, map.size);
                
                if (width > 0 && height > 0 && (int)map.size >= width * height * 3) {
                    cv::Mat frame(height, width, CV_8UC3, (char*)map.data);
                    self->publish_image(frame);
                } else {
                    RCLCPP_WARN(self->get_logger(), "Invalid frame dimensions or size: width=%d, height=%d, size=%zu", width, height, map.size);
                }
                
                gst_buffer_unmap(buffer, &map);
            } else {
                RCLCPP_WARN(self->get_logger(), "Failed to map buffer");
            }
            gst_sample_unref(sample);
        } else {
            RCLCPP_WARN(self->get_logger(), "Null sample received");
        }
        return GST_FLOW_OK;
    }

    void publish_image(const cv::Mat &frame)
    {
        // Publish raw image
        auto raw_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        raw_publisher_->publish(*raw_msg);

        // Publish compressed image
        sensor_msgs::msg::CompressedImage compressed_msg;
        compressed_msg.header.stamp = this->now();
        compressed_msg.format = "jpeg";
        cv::imencode(".jpg", frame, compressed_msg.data);
        compressed_publisher_->publish(compressed_msg);
    }

    void timer_callback()
    {
        // This callback is not used for processing, but keeps the node alive
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    GstElement *pipeline_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RtspStreamer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}