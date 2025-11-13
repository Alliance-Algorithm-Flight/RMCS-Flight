#pragma once

#include <Eigen/Eigen>

namespace rmcs_auto_aim::util::math {

/**
 * @brief Extracts the yaw (rotation around the Z axis) from a quaternion.
 *
 * @param quaternion Quaternion representing an orientation.
 * @return double Yaw angle in radians in the range [-pi, pi].
 */
static inline double get_yaw_from_quaternion(const Eigen::Quaterniond& quaternion) {

    double yaw = atan2(
        2.0 * (quaternion.w() * quaternion.z() + quaternion.x() * quaternion.y()),
        1.0 - 2.0 * (quaternion.y() * quaternion.y() + quaternion.z() * quaternion.z()));

    return yaw;
}

/**
 * @brief Computes the absolute yaw difference between two orientations.
 *
 * Computes the smallest angular difference around the yaw axis between q1 and q2, normalized to the range [0, pi].
 *
 * @param q1 First orientation quaternion.
 * @param q2 Second orientation quaternion.
 * @return double The yaw angle error in radians, between 0 and pi inclusive.
 */
static inline double
    get_angle_err_rad_from_quaternion(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2) {
    double yaw1  = get_yaw_from_quaternion(q1);
    double yaw2  = get_yaw_from_quaternion(q2);
    auto yaw_err = abs(yaw1 - yaw2);

    while (yaw_err > 2 * std::numbers::pi)
        yaw_err -= 2 * std::numbers::pi;
    if (yaw_err > std::numbers::pi)
        yaw_err = 2 * std::numbers::pi - yaw_err;
    return yaw_err;
}
/**
 * @brief Compute the absolute difference between the magnitudes of two 3D vectors.
 *
 * Computes the Euclidean norm of each provided vector and returns the absolute
 * difference between those norms.
 *
 * @param v1 First 3D vector.
 * @param v2 Second 3D vector.
 * @return double Absolute difference |‖v1‖ - ‖v2‖|.
 */
static inline double
    get_distance_err_rad_from_vector3d(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2) {
    double d1 = v1.norm();
    double d2 = v2.norm();
    auto derr = abs(d1 - d2);

    return derr;
}
/**
 * @brief Computes the angle of a 2D point measured from the positive x-axis.
 *
 * @tparam Point Type of the point; must provide accessible `x` and `y` members.
 * @param point 2D point whose angle is computed.
 * @return double Angle in radians in the range (-pi, pi], measured counterclockwise from the positive x-axis.
 */
static constexpr double ratio(const auto& point) { return atan2(point.y, point.x); }
/**
 * @brief Adjusts an angle to lie within the open interval (-π, π).
 *
 * The input angle is modified by adding or subtracting π as needed until
 * the result is greater than -π and less than π.
 *
 * @param angle Angle in radians.
 * @return double Angle equivalent to the input that is within (-π, π).
 */
static constexpr double clamp_pm_pi(auto&& angle) {
    while (angle >= std::numbers::pi)
        angle -= std::numbers::pi;
    while (angle <= -std::numbers::pi)
        angle += std::numbers::pi;

    return angle;
}
/**
 * @brief Normalize an angle into the principal range (-2π, 2π).
 *
 * @return double The equivalent angle constrained to be greater than -2π and less than 2π.
 */
static constexpr double clamp_pm_tau(auto&& angle) {
    while (angle >= 2 * std::numbers::pi)
        angle -= 2 * std::numbers::pi;
    while (angle <= -2 * std::numbers::pi)
        angle += 2 * std::numbers::pi;

    return angle;
}
} // namespace rmcs_auto_aim::util::math