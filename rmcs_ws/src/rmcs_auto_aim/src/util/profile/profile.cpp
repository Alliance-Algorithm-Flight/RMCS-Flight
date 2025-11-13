#include "profile.hpp"
#include <opencv2/core/types.hpp>

namespace rmcs_auto_aim::util {

struct Profile::Impl {
    /**
         * @brief Constructs an implementation instance using camera intrinsic and distortion parameters.
         *
         * Initializes the stored intrinsic and distortion parameter matrices from the provided values.
         *
         * @param fx Horizontal focal length in pixels.
         * @param fy Vertical focal length in pixels.
         * @param cx Principal point x-coordinate in pixels.
         * @param cy Principal point y-coordinate in pixels.
         * @param k1 First radial distortion coefficient.
         * @param k2 Second radial distortion coefficient.
         * @param k3 Third radial distortion coefficient.
         */
        Impl(
        const double& fx, const double& fy, const double& cx, const double& cy, const double& k1,
        const double& k2, const double& k3)
        : intrinsic_parameters((cv::Mat)(cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1))
        , distortion_parameters((cv::Mat)(cv::Mat_<double>(1, 5) << k1, k2, 0, 0, k3)) {}

    const cv::Mat intrinsic_parameters;
    const cv::Mat distortion_parameters;
    std::tuple<int, int> width_height;
};

/**
 * @brief Constructs a Profile initialized with camera intrinsic and distortion parameters.
 *
 * @param fx Focal length in pixels along the x axis.
 * @param fy Focal length in pixels along the y axis.
 * @param cx Principal point x coordinate (optical center) in pixels.
 * @param cy Principal point y coordinate (optical center) in pixels.
 * @param k1 Radial distortion coefficient k1.
 * @param k2 Radial distortion coefficient k2.
 * @param k3 Radial distortion coefficient k3.
 */
Profile::Profile(
    const double& fx, const double& fy, const double& cx, const double& cy, const double& k1,
    const double& k2, const double& k3) {
    impl_ = std::make_unique<Impl>(fx, fy, cx, cy, k1, k2, k3);
}

/**
 * @brief Sets the image dimensions for this Profile.
 *
 * Stores the image width and height (in pixels) used by this Profile.
 *
 * @param width Image width in pixels.
 * @param height Image height in pixels.
 */
void Profile::set_width_height(const int& width, const int& height) {
    impl_->width_height = std::make_tuple(width, height);
}

/**
 * @brief Accesses the camera intrinsic matrix.
 *
 * Returns the 3x3 camera intrinsic matrix containing focal lengths and principal point:
 * [fx, 0, cx; 0, fy, cy; 0, 0, 1].
 *
 * @return const cv::Mat& Reference to the 3x3 intrinsic parameter matrix.
 */
const cv::Mat& Profile::get_intrinsic_parameters() { return impl_->intrinsic_parameters; }
/**
 * @brief Retrieves the camera distortion parameters.
 *
 * The matrix is a 1x5 row containing distortion coefficients in the order [k1, k2, 0, 0, k3].
 *
 * @return const cv::Mat& Distortion parameter matrix (1 row × 5 columns).
 */
const cv::Mat& Profile::get_distortion_parameters() { return impl_->distortion_parameters; }
/**
 * @brief Access the stored image dimensions.
 *
 * @return const std::tuple<int,int>& A const reference to a tuple whose first element is the width and second element is the height.
 */
const std::tuple<int, int>& Profile::get_width_height() { return impl_->width_height; }
} // namespace rmcs_auto_aim::util

std::unique_ptr<rmcs_auto_aim::util::Profile::Impl> rmcs_auto_aim::util::Profile::impl_;