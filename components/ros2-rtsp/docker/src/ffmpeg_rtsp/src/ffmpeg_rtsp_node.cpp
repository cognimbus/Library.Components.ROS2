#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
}

class RtspPublisherNode : public rclcpp::Node
{
public:
    RtspPublisherNode() : Node("ffmpeg_rtsp_node")
    {
        RCLCPP_INFO(this->get_logger(), "Initializing RtspStreamer");
        
        this->declare_parameter("width", 1920);
        this->declare_parameter("height", 1080);
        this->declare_parameter("rtsp_url", "rtsp://cogniteam:cogniCOGNI@192.168.31.237:554/stream1");

        // Get parameter values
        desired_width_ = this->get_parameter("width").as_int();
        desired_height_ = this->get_parameter("height").as_int();
        std::string rtsp_url = this->get_parameter("rtsp_url").as_string();

        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);
        compressed_publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("camera/image/compressed", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(33), std::bind(&RtspPublisherNode::timer_callback, this));
        
        // Initialize FFmpeg
        avformat_network_init();
        format_ctx_ = avformat_alloc_context();

        // Set up options to disable buffering
        AVDictionary* options = NULL;
        av_dict_set(&options, "fflags", "nobuffer", 0);
        av_dict_set(&options, "flags", "low_delay", 0);
        av_dict_set(&options, "max_delay", "500000", 0);  // 0.5 seconds
        av_dict_set(&options, "reorder_queue_size", "0", 0);

        // Open input with the specified options
        if (avformat_open_input(&format_ctx_, rtsp_url.c_str(), NULL, &options) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Could not open RTSP stream.");
            return;
        }

        // Free the options dictionary
        av_dict_free(&options);

        // Rest of the constructor remains the same
        avformat_find_stream_info(format_ctx_, NULL);

        // Find the video stream
        for (unsigned int i = 0; i < format_ctx_->nb_streams; i++) {
            if (format_ctx_->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
                video_stream_index_ = i;
                break;
            }
        }

        // Initialize codec
        const AVCodec* codec = avcodec_find_decoder(format_ctx_->streams[video_stream_index_]->codecpar->codec_id);
        codec_ctx_ = avcodec_alloc_context3(codec);
        avcodec_parameters_to_context(codec_ctx_, format_ctx_->streams[video_stream_index_]->codecpar);
        avcodec_open2(codec_ctx_, codec, NULL);

        // Initialize frame and packet
        frame_ = av_frame_alloc();
        packet_ = av_packet_alloc();
        RCLCPP_INFO(this->get_logger(), "FFmpeg initialization complete");
    }
    
    ~RtspPublisherNode()
    {
        avformat_close_input(&format_ctx_);
        avcodec_free_context(&codec_ctx_);
        av_frame_free(&frame_);
        av_packet_free(&packet_);
    }

private:
    void timer_callback()
    {
        if (av_read_frame(format_ctx_, packet_) >= 0) {
            if (packet_->stream_index == video_stream_index_) {
                int ret = avcodec_send_packet(codec_ctx_, packet_);
                if (ret < 0) {
                    RCLCPP_WARN(this->get_logger(), "Error sending packet for decoding");
                    av_packet_unref(packet_);
                    return;
                }
                
                while (ret >= 0) {
                    ret = avcodec_receive_frame(codec_ctx_, frame_);
                    if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
                        break;
                    } else if (ret < 0) {
                        RCLCPP_WARN(this->get_logger(), "Error during decoding");
                        break;
                    }
                    
                    // Convert frame to OpenCV Mat
                    cv::Mat img(frame_->height, frame_->width, CV_8UC3);
                    SwsContext* sws_ctx = sws_getContext(
                        frame_->width, frame_->height, static_cast<AVPixelFormat>(frame_->format),
                        frame_->width, frame_->height, AV_PIX_FMT_BGR24,
                        SWS_BILINEAR, NULL, NULL, NULL
                    );
                    
                    if (!sws_ctx) {
                        RCLCPP_ERROR(this->get_logger(), "Could not initialize SwsContext");
                        return;
                    }
                    
                    uint8_t* dest[4] = {img.data, NULL, NULL, NULL};
                    int dest_linesize[4] = {img.step, 0, 0, 0};
                    sws_scale(sws_ctx, frame_->data, frame_->linesize, 0, frame_->height, dest, dest_linesize);
                    sws_freeContext(sws_ctx);

                    cv::Mat resized_img;
                    cv::resize(img, resized_img, cv::Size(desired_width_, desired_height_), 0, 0, cv::INTER_LINEAR);
                    // RCLCPP_INFO(self->get_logger(), "Frame received: width=%d, height=%d", desired_width_, desired_height_);
                    // Convert OpenCV Mat to ROS Image message
                    sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", resized_img).toImageMsg();
                    publisher_->publish(*msg);
                    sensor_msgs::msg::CompressedImage compressed_msg;
                    compressed_msg.header.stamp = this->now();
                    compressed_msg.format = "jpeg";
                    cv::imencode(".jpg", resized_img, compressed_msg.data);
                    compressed_publisher_->publish(compressed_msg);
                }
            }
            av_packet_unref(packet_);
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    int desired_width_;
    int desired_height_;
    AVFormatContext* format_ctx_;
    AVCodecContext* codec_ctx_;
    AVFrame* frame_;
    AVPacket* packet_;
    int video_stream_index_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RtspPublisherNode>());
    rclcpp::shutdown();
    return 0;
}