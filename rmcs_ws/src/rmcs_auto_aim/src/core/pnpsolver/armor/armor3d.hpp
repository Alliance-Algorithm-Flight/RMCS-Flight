/**
 * @file armor_plate_3d.hpp
 * @author Lorenzo Feng (lorenzo.feng@njust.edu.cn)
 * @brief
 * @version 0.1
 * @date 2024-06-02
 *
 * (C)Copyright: NJUST.Alliance - All rights reserved
 *
 */
#pragma once

#include <geometry_msgs/msg/pose.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <rmcs_description/tf_description.hpp>
#include <rmcs_msgs/robot_id.hpp>

namespace rmcs_auto_aim {
struct ArmorPlate3d {
    rmcs_msgs::ArmorID id;
    rmcs_description::OdomImu::Position position;
    rmcs_description::OdomImu::Rotation rotation;

    /**
         * @brief Construct an ArmorPlate3d with the specified identifier, position, and rotation.
         *
         * @param id Armor identifier.
         * @param position Position of the armor plate in the odom/imu reference used by rmcs_description.
         * @param rotation Rotation (orientation) of the armor plate in the odom/imu reference used by rmcs_description.
         */
        explicit ArmorPlate3d(
        rmcs_msgs::ArmorID id, rmcs_description::OdomImu::Position position,
        rmcs_description::OdomImu::Rotation rotation)
        : id(id)
        , position(std::move(position))
        , rotation(std::move(rotation)) {}
};

struct ArmorPlate3dWithoutFrame {
    rmcs_msgs::ArmorID id;
    geometry_msgs::msg::Pose pose;

    /**
 * @brief Constructs an ArmorPlate3dWithoutFrame with default-initialized members.
 *
 * The `id` and `pose` members are value-initialized to their respective defaults.
 */
ArmorPlate3dWithoutFrame() = default;
    /**
     * @brief Construct an ArmorPlate3dWithoutFrame with an ID and pose specified by Eigen types.
     *
     * Populates the internal `pose` from the given `position` and `rotation`.
     *
     * @param id Armor identifier.
     * @param position Position vector whose x/y/z are copied into `pose.position`.
     * @param rotation Orientation quaternion whose x/y/z/w are copied into `pose.orientation`.
     */
    ArmorPlate3dWithoutFrame(
        rmcs_msgs::ArmorID id, Eigen::Vector3d position, Eigen::Quaterniond rotation)
        : id(id) {
        pose.position.x    = position.x();
        pose.position.y    = position.y();
        pose.position.z    = position.z();
        pose.orientation.x = rotation.x();
        pose.orientation.y = rotation.y();
        pose.orientation.z = rotation.z();
        pose.orientation.w = rotation.w();
    }
};
} // namespace rmcs_auto_aim