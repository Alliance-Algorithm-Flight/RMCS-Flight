#pragma once

#include <memory>
#include <string>

#include <opencv2/core/mat.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
namespace rmcs_auto_aim::util {

class IAutoAimDrawable {
public:
    virtual void draw(cv::InputOutputArray image, const cv::Scalar& color) const = 0;

    /**
 * @brief Virtual destructor to allow safe polymorphic deletion of derived drawables.
 */
virtual ~IAutoAimDrawable() = default;
};

class ImageViewer {

public:
    class ImageViewer_ {
    public:
        virtual void draw(const IAutoAimDrawable&, const cv::Scalar&) = 0;

        virtual void load_image(const cv::Mat& image) = 0;

        virtual void show_image() = 0;

        /**
 * @brief Virtual destructor to allow safe polymorphic deletion of ImageViewer_ instances.
 *
 * Ensures derived implementations can be destroyed correctly when referenced via the base class pointer.
 */
virtual ~ImageViewer_() = default;
    };
    /**
     * @brief Load an image into the currently configured ImageViewer backend.
     *
     * If no backend has been installed, this function does nothing.
     *
     * @param image CV image to present to the viewer; accepted formats are those supported by the active backend.
     */
    static inline void load_image(const cv::Mat& image) {
        if (viewer_ == nullptr) [[unlikely]]
            return;

        viewer_->load_image(image);
    }

    /**
     * @brief Render a drawable onto the current viewer image using the specified color.
     *
     * If no image viewer backend is configured, the call is a no-op.
     *
     * @param drawable Drawable element to render.
     * @param color Color used to draw the element; interpreted as B,G,R[,A] channels (cv::Scalar channel order).
     */
    static inline void draw(const IAutoAimDrawable& drawable, const cv::Scalar& color) {
        if (viewer_ == nullptr) [[unlikely]]
            return;

        viewer_->draw(drawable, color);
    }
    /**
     * @brief Displays the currently loaded image using the configured image viewer backend.
     *
     * If no backend is configured, the call has no effect.
     */
    static inline void show_image() {
        if (viewer_ == nullptr) [[unlikely]]
            return;

        viewer_->show_image();
    }

    static void createProduct(int type, rclcpp::Node& node, const std::string& name);

private:
    static std::unique_ptr<ImageViewer_> viewer_;
};
} // namespace rmcs_auto_aim::util