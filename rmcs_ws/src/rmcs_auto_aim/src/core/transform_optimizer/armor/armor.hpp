#pragma once

#include <cassert>
#include <cmath>
#include <numbers>
#include <vector>

#include <Eigen/Eigen>

#include <rmcs_description/tf_description.hpp>

#include "core/identifier/armor/armor.hpp"
#include "core/pnpsolver/armor/armor3d.hpp"
#include "core/transform_optimizer/armor/quadrilateral/quadrilateral.hpp"
#include "util/math.hpp"
#include "util/optimizer/fibonacci.hpp"

namespace rmcs_auto_aim::transform_optimizer {

constexpr inline static double epsilone = 0.001;

/**
 * @brief Apply a yaw-like rotation around the Z axis to an existing 3D armor rotation.
 *
 * @param inOutArmor3d Current armor rotation to which the Z-axis rotation will be applied.
 * @param angle Angle, in radians, to rotate around the Z axis.
 * @return rmcs_description::OdomImu::Rotation The resulting rotation after applying the given Z-axis rotation to the input rotation.
 */
inline rmcs_description::OdomImu::Rotation
    set_armor3d_angle(const Eigen::AngleAxis<double>& inOutArmor3d, const double& angle) {
    return rmcs_description::OdomImu::Rotation(
        Eigen::AngleAxis(angle, Eigen::Vector3d::UnitZ()) * inOutArmor3d);
}
/**
 * @brief Optimize 3D armor rotations to better match their 2D projected quadrilaterals.
 *
 * Adjusts each entry in @p inOutArmor3d so its projected quadrilateral under @p tf
 * more closely matches the corresponding 2D quadrilateral in @p inArmor2d.
 *
 * @param inArmor2d  Vector of 2D armor representations used as projection targets.
 * @param inOutArmor3d  Vector of 3D armor representations to be modified in place; each element's rotation
 *                     is updated to minimize projection error and then adjusted by an additional 180° rotation
 *                     about the armor's local X axis.
 * @param tf  Transformation used to project 3D quadrilaterals into the 2D frame.
 *
 * @note If @p inArmor2d and @p inOutArmor3d have different sizes, the function returns immediately without modifying anything.
 */
static inline void armor_transform_optimize(
    const std::vector<ArmorPlate>& inArmor2d, std::vector<ArmorPlate3d>& inOutArmor3d,
    const rmcs_description::Tf& tf) {
    if (inArmor2d.size() != inOutArmor3d.size())
        return;

    for (int i = 0, len = (int)inArmor2d.size(); i < len; i++) {
        auto squad2d = Quadrilateral(inArmor2d[i]);
        auto armor3d = inOutArmor3d[i];

        Eigen::AngleAxis rotation = Eigen::AngleAxis(
            165. / 180.0 * std::numbers::pi,
            *rmcs_description::OdomImu::DirectionVector(Eigen::Vector3d::UnitY()));

        double yaw = util::math::get_yaw_from_quaternion(*armor3d.rotation);
        auto angle = util::optimizer::fibonacci(
            yaw - std::numbers::pi / 5, yaw + std::numbers::pi / 5, epsilone,
            [&squad2d, &armor3d, &rotation, &tf](double angle) -> double {
                armor3d.rotation = set_armor3d_angle(rotation, angle);
                auto squad3d =
                    Quadrilateral3d(armor3d).ToQuadrilateral(tf, squad2d.is_large_armor());
                return squad3d - squad2d;
            });

        inOutArmor3d[i].rotation = set_armor3d_angle(rotation, angle);
        *inOutArmor3d[i].rotation =
            Eigen::AngleAxisd(
                std::numbers::pi, *inOutArmor3d[i].rotation * Eigen::Vector3d::UnitX())
            * *inOutArmor3d[i].rotation;
    }
}

} // namespace rmcs_auto_aim::transform_optimizer