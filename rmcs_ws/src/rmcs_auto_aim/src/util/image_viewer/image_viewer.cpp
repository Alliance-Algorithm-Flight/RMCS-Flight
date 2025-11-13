#include <cstring>
#include <iostream>
#include <memory>
#include <opencv2/highgui.hpp>
#include <ostream>
#include <string>

#include <libavutil/pixfmt.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "util/image_viewer/image_viewer.hpp"
#include "util/profile/profile.hpp"

using namespace rmcs_auto_aim::util;

class CVBridgeViewer final : public ImageViewer::ImageViewer_ {
public:
    /**
     * @brief Constructs a CVBridgeViewer and creates a ROS2 publisher for Image messages.
     *
     * Initializes the viewer to publish OpenCV images as sensor_msgs::msg::Image on the specified topic
     * and allocates an internal publisher with a queue size of 10.
     *
     * @param node Reference to the rclcpp::Node used to create the ROS2 publisher.
     * @param name Topic name to publish sensor_msgs::msg::Image messages on.
     */
    explicit CVBridgeViewer(rclcpp::Node& node, const std::string& name)
        : node_(node)
        , name_(name) {

        publisher_ = node_.create_publisher<sensor_msgs::msg::Image>(name_, 10);
    }

    /**
     * @brief Draws the given drawable onto the internal image using the specified color.
     *
     * @param drawable Object that will render itself onto the viewer's internal image.
     * @param color Color used for drawing (cv::Scalar in OpenCV order: B, G, R[, A]).
     */
    void draw(const IAutoAimDrawable& drawable, const cv::Scalar& color) final {
        drawable.draw(image_, color);
    };

    /**
 * @brief Store the provided OpenCV image in the viewer's internal buffer.
 *
 * @param image OpenCV Mat to store; the internal buffer will share the Mat's underlying data (no deep copy is performed).
 */
void load_image(const cv::Mat& image) final { image_ = image; };

    /**
     * @brief Publishes the currently stored OpenCV image to the ROS2 image topic.
     *
     * If no image is loaded, the call is a no-op. When an image is present, the function
     * constructs a sensor_msgs::msg::Image with the node's current timestamp, sets the
     * encoding to BGR8, fills height/width/step and raw data from the stored cv::Mat,
     * and publishes the message on the viewer's publisher.
     */
    void show_image() final {
        if (image_.empty()) {
            return;
        }

        sensor_msgs::msg::Image msg;
        msg.header.stamp    = node_.get_clock()->now();
        msg.header.frame_id = "";
        msg.height          = static_cast<uint32_t>(image_.rows);
        msg.width           = static_cast<uint32_t>(image_.cols);
        msg.encoding        = sensor_msgs::image_encodings::BGR8;
        msg.is_bigendian    = false;
        msg.step            = static_cast<uint32_t>(image_.step);
        const auto data_size =
            static_cast<size_t>(image_.step) * static_cast<size_t>(image_.rows);
        msg.data.resize(data_size);
        if (data_size > 0) {
            std::memcpy(msg.data.data(), image_.data, data_size);
        }
        publisher_->publish(msg);
    };
    /**
 * @brief Releases the stored OpenCV image buffer.
 *
 * Ensures the internal cv::Mat is freed when the viewer is destroyed.
 */
~CVBridgeViewer() { image_.release(); }

private:
    rclcpp::Node& node_;
    const std::string& name_;
    cv::Mat image_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

class ImShowViewer final : public ImageViewer::ImageViewer_ {
public:
    /**
         * @brief Constructs an ImShowViewer for displaying images in an OpenCV window.
         *
         * @param name Window name used by OpenCV functions (e.g., imshow) to identify the display.
         */
        explicit ImShowViewer(const std::string& name)
        : name_(name) {}

    /**
     * @brief Renders the given drawable onto the viewer's internal image using the specified color.
     *
     * @param drawable Object that performs drawing onto a cv::Mat.
     * @param color BGR color used for drawing.
     */
    void draw(const IAutoAimDrawable& drawable, const cv::Scalar& color) final {
        drawable.draw(image, color);
    };

    /**
 * @brief Stores a deep copy of the provided OpenCV image for later display.
 *
 * @param image Source image to load; the image is cloned (deep-copied) before storing.
 */
void load_image(const cv::Mat& image) final { this->image = image.clone(); };

    /**
     * @brief Displays the stored image in the viewer window and processes GUI events.
     *
     * Shows the internal image in the OpenCV window identified by the viewer's name
     * and pumps the GUI event loop briefly to update the window.
     */
    void show_image() final {
        cv::imshow(name_, image);
        cv::waitKey(1);
    };

    /**
 * @brief Releases the stored OpenCV image resource used by the viewer.
 *
 * Ensures the internal cv::Mat is freed when the viewer is destroyed.
 */
~ImShowViewer() { image.release(); }

private:
    const std::string& name_;
    cv::Mat image;
};

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
}
class RtspViewer final : public ImageViewer::ImageViewer_ {
public:
    /**
     * @brief Constructs an RtspViewer and initializes resources required to stream video to an RTSP endpoint.
     *
     * Initializes the FFmpeg network layer, builds the RTSP output URL from the provided stream name, creates
     * the output/encoder context for the configured resolution and framerate, allocates the encoding AVFrame,
     * creates a software-scaling context for converting BGR source frames to the encoder pixel format, and
     * starts the internal timer thread that schedules frame availability.
     *
     * @param name Stream name appended to the RTSP base URL (resulting URL: "rtsp://localhost:8554/live/<name>").
     */
    explicit RtspViewer(const std::string& name)
        : name_("rtsp://localhost:8554/live/" + name) {
        frameCount_ = 0;

        avformat_network_init();
        auto& [width, height] = Profile::get_width_height();
        createOutputContext(name_.c_str(), width, height, fps);

        frame_         = av_frame_alloc();
        frame_->format = codecCtx_->pix_fmt;
        frame_->width  = codecCtx_->width;
        frame_->height = codecCtx_->height;
        if (av_frame_get_buffer(frame_, 32) < 0) {
            std::cerr << "Could not allocate frame buffer" << std::endl;
        }

        swsCtx_ = sws_getContext(
            width, height, AV_PIX_FMT_BGR24, width, height, codecCtx_->pix_fmt, SWS_BILINEAR,
            nullptr, nullptr, nullptr);

        timerThread_ = std::thread(
            [this]() { executeFunctionAtFrequency(std::chrono::milliseconds(1000 / fps)); });
    }

    /**
     * @brief Draws the given drawable onto the internal image using the specified color.
     *
     * @param drawable Object that will render itself onto the viewer's internal image.
     * @param color Color used for drawing (cv::Scalar in OpenCV order: B, G, R[, A]).
     */
    void draw(const IAutoAimDrawable& drawable, const cv::Scalar& color) final {
        drawable.draw(image_, color);
    };

    /**
 * @brief Stores a deep copy of the provided OpenCV image for later display.
 *
 * @param image Source image to clone and keep internally.
 */
void load_image(const cv::Mat& image) final { this->image_ = image.clone(); };

    /**
     * @brief Encodes and streams the most recently loaded image to the configured RTSP output if a new frame is available.
     *
     * If a new frame is available, this function converts the stored OpenCV image into the encoder's frame,
     * assigns a presentation timestamp, sends the frame to the encoder, and writes all produced packets to the RTSP output stream.
     * If no new frame is available, the function returns immediately. On any scaling, encoding, or write failure the function
     * aborts that update and returns; partial packets already produced are freed.
     *
     * @note This function updates internal streaming state (consumes the new-frame flag and increments the internal frame counter)
     * and performs I/O to the RTSP output context.
     */
    void show_image() final {
        if (!newFrameAvailable_)
            return;
        AVPacket pkt{};
        newFrameAvailable_.store(false);
        const uint8_t* srcSlice[1] = {image_.data};
        int srcStride[1]           = {static_cast<int>(image_.step[0])};
        int ret                    = 0;
        ret                        = sws_scale(
            swsCtx_, srcSlice, srcStride, 0, frame_->height, frame_->data, frame_->linesize);

        if (ret < 0) {
            std::cerr << "Error: Could not scale image" << ret << std::endl;
            return;
        }

        frame_->pts = frameCount_++;
        if (showImageThread_.joinable())
            showImageThread_.join();
        if ((ret = avcodec_send_frame(codecCtx_, frame_)) < 0) {
            std::cerr << "Error: Could not send frame" << std::endl;
            return;
        }
        while (avcodec_receive_packet(codecCtx_, &pkt) == 0) {
            pkt.stream_index = oc_->streams[0]->index;
            av_packet_rescale_ts(&pkt, codecCtx_->time_base, oc_->streams[0]->time_base);
            ret = av_interleaved_write_frame(oc_, &pkt);
            if (ret < 0) {
                std::cerr << "Error: Could not write frame" << std::endl;
            }
            av_packet_unref(&pkt);
        }
    }

    /**
 * @brief Disabled default constructor to prevent creating an RtspViewer without configuration.
 *
 * Instances must be constructed with the explicit constructor that accepts a stream name.
 */
RtspViewer() = delete;
    /**
     * @brief Cleans up and shuts down the RTSP viewer, releasing all allocated resources and stopping background activity.
     *
     * Performs finalization necessary to stop streaming and free resources: releases the stored OpenCV image, writes the format trailer,
     * closes the encoder and I/O, frees the FFmpeg format/context/frame/scale resources, joins the timer thread, and resets internal state flags.
     */
    ~RtspViewer() {
        image_.release();

        av_write_trailer(oc_);
        avcodec_close(codecCtx_);
        avio_close(oc_->pb);
        avformat_free_context(oc_);
        av_frame_free(&frame_);
        sws_freeContext(swsCtx_);
        timerThread_.join();
        newFrameAvailable_ = false;
        running            = false;
    }

private:
    std::atomic<bool> newFrameAvailable_{false};
    std::atomic<bool> frame_using_{false};
    std::atomic<bool> running{true};
    std::thread timerThread_;
    std::thread showImageThread_;

    constexpr static int fps = 60;

    /**
     * @brief Periodically signals that a new frame is available while the viewer is running.
     *
     * Repeatedly sets the internal `newFrameAvailable_` flag to `true` at the provided interval
     * until the `running` flag becomes false.
     *
     * @param interval Time between successive signals that a new frame is available.
     */
    void executeFunctionAtFrequency(std::chrono::milliseconds interval) {
        while (running) {
            newFrameAvailable_.store(true);
            std::this_thread::sleep_for(interval);
        }
    }
    /**
     * @brief Creates and configures an FFmpeg output context and encoder for streaming to an RTSP URL.
     *
     * Sets up the output format context and encoder according to the provided resolution and frame rate,
     * opens the output URL, writes the stream header, and stores the resulting contexts into the
     * instance members (e.g., oc_ and codecCtx_) on success. On failure the function logs an error and
     * leaves instance members unchanged.
     *
     * @param url RTSP output URL (e.g., "rtsp://...").
     * @param width Video frame width in pixels.
     * @param height Video frame height in pixels.
     * @param fps Target frames per second for the output stream.
     */
    void createOutputContext(const char* url, int width, int height, int fps) {
        AVFormatContext* oc = nullptr;
        avformat_alloc_output_context2(&oc, nullptr, "rtsp", url);
        if (!oc) {
            RCLCPP_ERROR(rclcpp::get_logger("RTSP Viewer"), "Could not create output context");
            return;
        }

        AVStream* stream = avformat_new_stream(oc, nullptr);
        if (!stream) {
            RCLCPP_ERROR(rclcpp::get_logger("RTSP Viewer"), "Could not create stream");
            return;
        }

        codecCtx_ = avcodec_alloc_context3(avcodec_find_encoder(AV_CODEC_ID_H264));
        if (!codecCtx_) {
            RCLCPP_ERROR(rclcpp::get_logger("RTSP Viewer"), "Could not allocate codec context");
            return;
        }

        codecCtx_->codec_id  = AV_CODEC_ID_H264;
        codecCtx_->width     = width;
        codecCtx_->height    = height;
        codecCtx_->time_base = {1, fps};
        codecCtx_->framerate = {fps, 1};
        codecCtx_->pix_fmt   = AV_PIX_FMT_YUV420P;
        codecCtx_->bit_rate  = 400000;

        if (oc->oformat->flags & AVFMT_GLOBALHEADER) {
            codecCtx_->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
        }

        if (avcodec_open2(codecCtx_, avcodec_find_encoder(codecCtx_->codec_id), nullptr) < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("RTSP Viewer"), "Could not open codec");
            return;
        }

        auto ret = avcodec_parameters_from_context(stream->codecpar, codecCtx_);

        if (ret < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("RTSP Viewer"), "Could not copy codec parameters");
            return;
        }

        if (!(oc->oformat->flags & AVFMT_NOFILE)) {
            if (avio_open(&oc->pb, url, AVIO_FLAG_WRITE) < 0) {
                RCLCPP_ERROR(rclcpp::get_logger("RTSP Viewer"), "Could not open output URL");
                return;
            }
        }

        ret = avformat_write_header(oc, nullptr);

        if (ret < 0) {
            RCLCPP_INFO(
                rclcpp::get_logger("RTSP Viewer"),
                "Error return code from avformat_write_header: %x", ret);
            return;
        }

        if (!oc) {
            RCLCPP_INFO(
                rclcpp::get_logger("RTSP Viewer"), "Error: Could not create output context");
        } else
            oc_ = oc;
    }
    const std::string name_;
    cv::Mat image_;

    struct SwsContext* swsCtx_;
    AVFormatContext* oc_;
    AVCodecContext* codecCtx_;
    AVFrame* frame_;

    int frameCount_;
};

class NullViewer final : public ImageViewer::ImageViewer_ {
public:
    void draw(const IAutoAimDrawable&, const cv::Scalar&) final {};

    void load_image(const cv::Mat&) final {};

    void show_image() final {};
};

/**
 * @brief Constructs and assigns a concrete ImageViewer_ implementation based on the provided type.
 *
 * Creates a new viewer instance and stores it in the static ImageViewer::viewer_ member.
 *
 * @param type Selector for the viewer implementation:
 *             - 1: ImShowViewer (displays images in an OpenCV window)
 *             - 2: CVBridgeViewer (publishes images to a ROS2 topic)
 *             - 3: RtspViewer (streams images over RTSP)
 *             - otherwise: NullViewer (no-op)
 * @param node ROS2 node used when constructing viewers that require a node (e.g., CVBridgeViewer).
 * @param name Name passed to the constructed viewer (window name, topic name, or stream identifier).
 */
void ImageViewer::createProduct(int type, rclcpp::Node& node, const std::string& name) {
    switch (type) {
    case 1: viewer_ = std::make_unique<ImShowViewer>(name); break;
    case 2: viewer_ = std::make_unique<CVBridgeViewer>(node, name); break;
    case 3: viewer_ = std::make_unique<RtspViewer>(name); break;
    default: viewer_ = std::make_unique<NullViewer>(); break;
    }
}

std::unique_ptr<rmcs_auto_aim::util::ImageViewer::ImageViewer_>
    rmcs_auto_aim::util::ImageViewer::viewer_;