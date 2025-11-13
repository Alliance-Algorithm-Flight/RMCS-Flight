#pragma once
#include "util/image_viewer/image_viewer.hpp"
#include "util/math.hpp"
#include "util/profile/profile.hpp"
#include <Eigen/Eigen>
#include <Eigen/src/Core/Matrix.h>
#include <cmath>
#include <fast_tf/impl/cast.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <rmcs_description/tf_description.hpp>
#include <utility>
#include <vector>

namespace rmcs_auto_aim {
class Line : public util::IAutoAimDrawable {
public:
    /**
         * @brief Construct a 2D line from two image points representing its top and bottom endpoints.
         *
         * @param top Image coordinates of the line's top endpoint.
         * @param button Image coordinates of the line's bottom endpoint.
         */
        Line(cv::Point2f top, cv::Point2f button)
        : top_(std::move(top))
        , button_(std::move(button)) {}

    /**
     * @brief Draws the 2D line and its top endpoint onto the provided image.
     *
     * Draws a line between the stored top_ and button_ points and a filled circle at top_ using the specified color.
     *
     * @param image Image to draw on; modified in-place to contain the rendered line and endpoint marker.
     * @param color Color used for both the line and the circle (B,G,R[,A]).
     */
    void draw(cv::InputOutputArray image, const cv::Scalar& color) const final {
        cv::line(image, top_, button_, color);
        cv::circle(image, top_, 3, color, -1);
    };
    /**
     * @brief Calculates the angular difference between this line and another line.
     *
     * @param other The line to compare against.
     * @return double Absolute angular difference in radians between the two line directions; value is in [0, π].
     */
    double angle_distance(const Line& other) const {
        double ratio_err = util::math::clamp_pm_pi(util::math::ratio(top_ - button_))
                         - util::math::clamp_pm_pi(util::math::ratio(other.top_ - other.button_));
        return abs(ratio_err);
    }
    /**
     * @brief Computes the absolute difference in lengths between this line and another line.
     *
     * @param other Line to compare against.
     * @return double Absolute difference between the lengths of the two lines.
     */
    double length_distance(const Line& other) const {
        double ratio_err = cv::norm(top_ - button_) - cv::norm(other.top_ - other.button_);
        return abs(ratio_err);
    }
    /**
     * @brief Computes the combined perpendicular distance from the endpoints of another line to this infinite line.
     *
     * @param other The other Line whose endpoints are measured against this line.
     * @return double Sum of perpendicular distances from other.top_ and other.button_ to the infinite line defined by this object's top_ and button_, in the same units as the point coordinates (e.g., pixels).
     */
    double line_distance(const Line& other) const {
        double a = (top_.y - button_.y);
        double b = -(top_.x - button_.x);
        double c = b * -button_.y + a * -button_.x;
        double k = sqrt(a * a + b * b);

        double len1 = abs(a * other.top_.x + b * other.top_.y + c) / k;
        double len2 = abs(a * other.button_.x + b * other.button_.y + c) / k;
        return len1 + len2;
    }
    /**
 * @brief Deleted default constructor to prevent creating a Line without endpoints.
 *
 * A Line must be constructed with explicit `top` and `button` points; default construction is disabled.
 */
Line() = delete;

    const cv::Point2f top_;
    const cv::Point2f button_;
};
class Line3d {
public:
    /**
         * @brief Constructs a 3D line defined by orientation, position, and length.
         *
         * @param rotation Orientation of the line as an Eigen::Quaterniond.
         * @param position Position of the line's reference point (same coordinate frame used for projection).
         * @param length Length of the 3D line expressed in the same units as `position`.
         */
        explicit Line3d(Eigen::Quaterniond rotation, Eigen::Vector3d position, double length)
        : rotation_(std::move(rotation))
        , position(std::move(position))
        , length_(length) {}

    /**
     * @brief Projects this 3D line into 2D image coordinates using the given transform.
     *
     * Uses the provided transform to convert the line endpoints into the camera frame and
     * projects those endpoints with the active camera intrinsics and distortion parameters.
     *
     * @param tf Transform providing the camera frame context for projection.
     * @return Line A 2D Line whose endpoints are the projected image points of this 3D line.
     */
    Line to_line_2d(const rmcs_description::Tf& tf) {
        Eigen::Vector3d up_eigen   = Eigen::Vector3d::UnitZ() * length_ / 2.0;
        auto intrinsic_parameters  = util::Profile::get_intrinsic_parameters();
        auto distortion_parameters = util::Profile::get_distortion_parameters();

        auto rotationInCamera = fast_tf::cast<rmcs_description::CameraLink>(
            rmcs_description::OdomImu::DirectionVector(rotation_ * up_eigen), tf);

        auto pos1 = (*rotationInCamera + position) * 1000;
        auto pos2 = (-*rotationInCamera + position) * 1000;
        std::vector<cv::Point3f> object_points{
            {static_cast<float>(-pos1.y()), static_cast<float>(-pos1.z()),
             static_cast<float>(pos1.x())},
            {static_cast<float>(-pos2.y()), static_cast<float>(-pos2.z()),
             static_cast<float>(pos2.x())}
        };
        std::vector<cv::Point2f> imagePoints{};
        cv::Mat t = cv::Mat::zeros(3, 1, CV_32F), r = cv::Mat::zeros(3, 1, CV_32F);
        cv::projectPoints(
            object_points, t, r, intrinsic_parameters, distortion_parameters, imagePoints);
        return {imagePoints[0], imagePoints[1]};
    };

private:
    Eigen::Quaterniond rotation_;
    Eigen::Vector3d position;
    double length_;
};

class Line3dBottom {
public:
    /**
         * @brief Constructs a 3D bottom-aligned line descriptor.
         *
         * Initializes a Line3dBottom with the given orientation, position, and segment length used for projecting the line to 2D.
         *
         * @param rotation Orientation of the 3D line in world or sensor frame as a quaternion.
         * @param position Position of the 3D line origin in the same frame (meters).
         * @param length Length of the 3D line segment (meters).
         */
        explicit Line3dBottom(Eigen::Quaterniond rotation, Eigen::Vector3d position, double length)
        : rotation_(std::move(rotation))
        , position(std::move(position))
        , length_(length) {}

    /**
     * @brief Projects this 3D line into the image plane defined by the provided transform.
     *
     * Uses this object's rotation, position, and length to compute two 3D endpoints in the camera
     * frame, projects them with the current camera intrinsic and distortion parameters, and
     * returns a 2D Line built from the resulting image points.
     *
     * @param tf Transform representing the target camera frame used to convert endpoints into camera coordinates.
     * @return Line 2D line whose endpoints are the projected image points of the 3D endpoints.
     */
    Line to_line_2d(const rmcs_description::Tf& tf) {
        Eigen::Vector3d up_eigen   = Eigen::Vector3d::UnitY() * length_ / 2.0;
        auto intrinsic_parameters  = util::Profile::get_intrinsic_parameters();
        auto distortion_parameters = util::Profile::get_distortion_parameters();

        auto rotationInCamera = fast_tf::cast<rmcs_description::CameraLink>(
            rmcs_description::OdomImu::DirectionVector(rotation_ * up_eigen), tf);

        auto pos1 = (*rotationInCamera + position) * 1000;
        auto pos2 = (-*rotationInCamera + position) * 1000;
        std::vector<cv::Point3f> object_points{
            {static_cast<float>(-pos1.y()), static_cast<float>(-pos1.z()),
             static_cast<float>(pos1.x())},
            {static_cast<float>(-pos2.y()), static_cast<float>(-pos2.z()),
             static_cast<float>(pos2.x())}
        };
        std::vector<cv::Point2f> imagePoints{};
        cv::Mat t = cv::Mat::zeros(3, 1, CV_32F), r = cv::Mat::zeros(3, 1, CV_32F);
        cv::projectPoints(
            object_points, t, r, intrinsic_parameters, distortion_parameters, imagePoints);
        return {imagePoints[0], imagePoints[1]};
    };

private:
    Eigen::Quaterniond rotation_;
    Eigen::Vector3d position;
    double length_;
};

class LightBar3d {
public:
    /**
         * @brief Constructs a LightBar3d from an orientation, a position, and a set of 3D points.
         *
         * @param rotation Orientation of the light bar as a quaternion.
         * @param position Translation (position) of the light bar as a 3D vector.
         * @param points Ordered 3D points that define the light bar geometry.
         */
        explicit LightBar3d(
        Eigen::Quaterniond rotation, Eigen::Vector3d position,
        const std::vector<Eigen::Vector3d>& points)
        : rotation_(std::move(rotation))
        , position(std::move(position))
        , points_(points) {}
    /**
     * @brief Projects the stored 3D light-bar points into the image and returns their 2D line representation.
     *
     * Uses the provided transform to compute camera-frame 3D object points, projects those points
     * with the current camera intrinsics and distortion, and constructs a 2D Line whose endpoints
     * are the midpoints of the first pair and the second pair of projected points.
     *
     * @param tf Coordinate-frame transform used to compute object points in the camera frame.
     * @return Line 2D line with `top_` set to the midpoint of imagePoints[0] and imagePoints[3],
     *              and `button_` set to the midpoint of imagePoints[1] and imagePoints[2].
     */
    Line to_line_2d(const rmcs_description::Tf& tf) {
        auto object_points = get_objective_point(tf);

        auto intrinsic_parameters  = util::Profile::get_intrinsic_parameters();
        auto distortion_parameters = util::Profile::get_distortion_parameters();
        std::vector<cv::Point2f> imagePoints{};
        cv::Mat t = cv::Mat::zeros(3, 1, CV_32F), r = cv::Mat::zeros(3, 1, CV_32F);
        cv::projectPoints(
            object_points, t, r, intrinsic_parameters, distortion_parameters, imagePoints);
        return {(imagePoints[0] + imagePoints[3]) / 2, (imagePoints[1] + imagePoints[2]) / 2};
    }

private:
    /**
     * @brief Computes 3D object points in the camera coordinate system for projection.
     *
     * For each of the four predefined object points this method:
     * - rotates the point by the stored `rotation_` and casts it into the camera frame using `tf`,
     * - translates by the stored `position`,
     * - scales the result by 1000,
     * - converts to the OpenCV coordinate ordering used by the projector by returning (-y, -z, x).
     *
     * @param tf Transformation/frame helper used to cast direction vectors into the camera link.
     * @return std::vector<cv::Point3f>  A vector of four 3D points in camera coordinates, ready for use with cv::projectPoints.
     */
    inline std::vector<cv::Point3f> get_objective_point(rmcs_description::Tf const& tf) const {
        auto positionInCamera = position;

        std::vector<cv::Point3f> points{};
        auto& objectPoints = points_;

        for (int i = 0; i < 4; i++) {
            auto rotationInCamera = fast_tf::cast<rmcs_description::CameraLink>(
                rmcs_description::OdomImu::DirectionVector(rotation_ * objectPoints[i]), tf);
            auto pos = (*rotationInCamera + positionInCamera) * 1000;
            points.emplace_back(-pos.y(), -pos.z(), pos.x());
        }
        return points;
    };
    Eigen::Quaterniond rotation_;
    Eigen::Vector3d position;
    const std::vector<Eigen::Vector3d>& points_;
};
} // namespace rmcs_auto_aim