#include <rmcs_description/tf_description.hpp>

#include "trajectory_solvor.hpp"

using namespace rmcs_auto_aim;

class TrajectorySolver::StaticImpl {
public:
    /**
 * @brief Default-constructs a StaticImpl instance.
 */
StaticImpl() = default;

    /**
     * @brief Computes a firing direction and flight time to reach a target position given projectile speed.
     *
     * Computes a unit direction vector for the initial velocity required to aim at target_pos with the provided speed and sets fly_time to the estimated time of flight. If the internal ballistic discriminant is negative (no real elevation solution), the function falls back to a zero pitch (horizontal) solution and returns the corresponding direction and flight time.
     *
     * @param target_pos Target position with x, y, z components used as the aim point.
     * @param speed Initial projectile speed magnitude.
     * @param[out] fly_time Estimated time of flight for the returned direction.
     * @return rmcs_description::OdomImu::DirectionVector Unit direction vector for the computed shot.
     */
    [[nodiscard]] static rmcs_description::OdomImu::DirectionVector GetShotVector(
        const rmcs_description::OdomImu::Position& target_pos, const double& speed,
        double& fly_time) {

        const double& x = target_pos->x();
        const double& y = target_pos->y();
        const double& z = target_pos->z();

        double yaw   = atan2(y, x);
        double pitch = 0;

        double a = speed * speed; // v0 ^ 2
        double b = a * a;         // v0 ^ 4
        double c = x * x + y * y; // xt ^ 2
        double d = c * c;         // xt ^ 4
        double e = G * G;         // g ^ 2

        double xt = sqrt(c);      // target horizontal distance

        double f = b * d * (b - e * c - 2 * G * a * z);
        if (f >= 0) {
            pitch = -atan((b * c - sqrt(f)) / (G * a * c * xt));
        }

        auto result = rmcs_description::OdomImu::DirectionVector{
            cos(pitch) * cos(yaw), cos(pitch) * sin(yaw), -sin(pitch)};

        fly_time = xt / (cos(pitch) * speed);

        return result;
    }

private:
    constexpr const static double G = 9.80665;
};

/**
 * @brief Computes the unit shoot direction and flight time to reach a target position given an initial speed.
 *
 * @param target_pos Target position in the local frame.
 * @param speed Initial projectile speed.
 * @param[out] fly_time Computed flight time to the target in seconds.
 * @return rmcs_description::OdomImu::DirectionVector Unit direction vector to fire so the projectile reaches the target.
 */
[[nodiscard]] rmcs_description::OdomImu::DirectionVector TrajectorySolver::GetShotVector(
    const rmcs_description::OdomImu::Position& target_pos, const double& speed, double& fly_time) {
    return StaticImpl::GetShotVector(target_pos, speed, fly_time);
}