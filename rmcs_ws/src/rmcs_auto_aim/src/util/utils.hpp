/**
 * @file fps_counter.hpp
 * @author Lorenzo Feng (lorenzo.feng@njust.edu.cn)
 * @brief
 * @version 0.1
 * @date 2024-10-22
 *
 * (C)Copyright: NJUST.Alliance - All rights reserved
 *
 */

#pragma once
#include <chrono>
#include <numbers>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/types.hpp>

#include "core/pnpsolver/armor/armor3d.hpp"

namespace rmcs_auto_aim::util {
class FPSCounter {
public:
    /**
     * @brief Increments the internal frame counter and updates the stored FPS value once a one-second interval completes.
     *
     * Each call advances the frame count; when at least one second has elapsed since counting began, the method stores the counted frames as the last FPS, resets the counter, and indicates that an update occurred.
     *
     * @return `true` if the stored last-FPS value was updated on this call, `false` otherwise.
     */
    bool Count() {
        if (_count == 0) {
            _count       = 1;
            _timingStart = std::chrono::steady_clock::now();
        } else {
            ++_count;
            if (std::chrono::steady_clock::now() - _timingStart >= std::chrono::seconds(1)) {
                _lastFPS = _count;
                _count   = 0;
                return true;
            }
        }
        return false;
    }

    /**
 * @brief Retrieve the most recently measured frames per second.
 *
 * @return int The last computed frames-per-second value.
 */
int GetFPS() const { return _lastFPS; }

private:
    int _count = 0, _lastFPS;
    std::chrono::steady_clock::time_point _timingStart;
};

static inline constexpr double Pi = std::numbers::pi;

/**
 * @brief Computes the yaw angle of an armor plate from its rotation.
 *
 * @param armor ArmorPlate3d instance whose rotation is used to compute the forward normal.
 * @return double Yaw angle in radians measured from the x-axis (atan2(normal.y, normal.x)), in the range (-π, π].
 */
static inline double GetArmorYaw(const ArmorPlate3d& armor) {
    Eigen::Vector3d normal = (*armor.rotation) * Eigen::Vector3d{1, 0, 0};
    return atan2(normal.y(), normal.x());
}

/**
 * @brief Compute the signed smallest difference between two angles.
 *
 * Computes (a - b) wrapped into the interval [-Pi, Pi] so the result represents
 * the shortest signed rotation from b to a.
 *
 * @param a Angle in radians.
 * @param b Angle in radians.
 * @return double Signed angle difference in radians, in the range [-Pi, Pi].
 */
static inline double GetMinimumAngleDiff(double a, double b) {
    double diff = std::fmod(a - b, 2 * Pi);
    if (diff < -Pi) {
        diff += 2 * Pi;
    } else if (diff > Pi) {
        diff -= 2 * Pi;
    }
    return diff;
}
/**
 * @brief Compute a unit 3D direction vector for a pixel by undistorting the point and applying camera intrinsics.
 *
 * Undistorts imagePoint using cameraMatrix and distCoeffs, maps it to a normalized camera-space ray, and writes
 * the resulting unit direction into direction_vec using the mapping direction_vec = [z, -x, -y] where (x,y,z)
 * is the normalized camera-space vector. This allows callers to obtain a consistent 3D ray for further yaw/pitch computation.
 *
 * @param imagePoint  Pixel coordinates in the image.
 * @param cameraMatrix  3x3 camera intrinsic matrix (type CV_64F).
 * @param distCoeffs  Camera distortion coefficients (can be empty).
 * @param[out] direction_vec  Output unit direction vector corresponding to the ray through imagePoint;
 *                            set to the normalized vector [z, -x, -y] derived from camera-space coordinates.
 * @param assumedDepth  Temporary depth scale applied before normalization (default 1.0).
 * @return true if the direction vector was computed (currently always returns true).
 */
static inline bool compute_yaw_pitch_from_point(
    const cv::Point2f& imagePoint, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
    Eigen::Vector3d& direction_vec, double assumedDepth = 1.0) {
    // 1. 去畸变
    std::vector<cv::Point2f> distortedPoints = {imagePoint};
    std::vector<cv::Point2f> undistortedPoints;
    cv::undistortPoints(
        distortedPoints, undistortedPoints, cameraMatrix, distCoeffs, cv::noArray(), cameraMatrix);
    cv::Point2f undistortedPoint = undistortedPoints[0];

    // 2. 归一化相机坐标
    double fx = cameraMatrix.at<double>(0, 0);
    double fy = cameraMatrix.at<double>(1, 1);
    double cx = cameraMatrix.at<double>(0, 2);
    double cy = cameraMatrix.at<double>(1, 2);

    double x_normalized = (undistortedPoint.x - cx) / fx;
    double y_normalized = (undistortedPoint.y - cy) / fy;

    // 3. 构建方向向量
    cv::Point3d directionVec(
        x_normalized * assumedDepth, y_normalized * assumedDepth, assumedDepth);
    // 4. 计算方向向量的模长并归一化
    double norm = cv::norm(directionVec);
    directionVec /= norm;

    direction_vec = Eigen::Vector3d{directionVec.z, -directionVec.x, -directionVec.y};

    direction_vec = direction_vec.normalized();

    return true;
}
} // namespace rmcs_auto_aim::util