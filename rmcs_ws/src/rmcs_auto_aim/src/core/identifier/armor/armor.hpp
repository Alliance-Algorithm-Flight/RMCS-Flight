/**
 * @file Armor.hpp
 * @author Lorenzo Feng (lorenzo.feng@njust.edu.cn), Qzh
 * @brief
 * @version 0.1
 * @date 2024-06-02
 *
 * (C)Copyright: NJUST.Alliance - All rights reserved
 *
 */
#pragma once

#include <cassert>
#include <cstdint>
#include <stdexcept>
#include <utility>

#include <opencv2/core/types.hpp>

#include <rmcs_msgs/msg/armor_plate.hpp>
#include <rmcs_msgs/robot_id.hpp>

namespace rmcs_auto_aim {

struct LightBar {
    cv::Point2f top, bottom;
    float angle;

    /**
         * @brief Constructs a LightBar from its end points and orientation.
         *
         * @param _top  Position of the top endpoint of the light bar.
         * @param _bottom Position of the bottom endpoint of the light bar.
         * @param angle Orientation angle of the light bar.
         */
        LightBar(cv::Point2f _top, cv::Point2f _bottom, float angle)
        : top(std::move(_top))
        , bottom(std::move(_bottom))
        , angle(angle) {}
};

struct ArmorPlate {
    /**
     * @brief Constructs an ArmorPlate from two light bars.
     *
     * Initializes the armor's corner points in order: left.top, left.bottom, right.bottom, right.top,
     * and sets the armor identifier and size flag.
     *
     * @param left Left-side LightBar.
     * @param right Right-side LightBar.
     * @param armorId Identifier for the armor; defaults to Unknown.
     * @param isLargeArmor True if the armor is large, false otherwise.
     */
    ArmorPlate(
        const LightBar& left, const LightBar& right,
        rmcs_msgs::ArmorID armorId = rmcs_msgs::ArmorID::Unknown, bool isLargeArmor = false)
        : id(armorId)
        , is_large_armor(isLargeArmor) {
        points.push_back(left.top);
        points.push_back(left.bottom);
        points.push_back(right.bottom);
        points.push_back(right.top);
    }

    /**
         * @brief Constructs an ArmorPlate from an explicit list of corner points.
         *
         * Initializes an ArmorPlate by taking ownership of the provided corner points,
         * setting the armor identifier, and marking whether it is a large armor.
         *
         * @param points Four corner points in order: left_top, left_bottom, right_bottom, right_top.
         *               The vector is moved into the ArmorPlate (ownership transferred).
         * @param armorId Identifier to assign to the armor plate.
         * @param isLargeArmor True if the armor is a large variant, false otherwise.
         */
        explicit ArmorPlate(
        std::vector<cv::Point2f>&& points, rmcs_msgs::ArmorID armorId = rmcs_msgs::ArmorID::Unknown,
        bool isLargeArmor = false)
        : points(points)
        , id(armorId)
        , is_large_armor(isLargeArmor) {}

    /**
     * @brief Converts this ArmorPlate to an rmcs_msgs::msg::ArmorPlate message.
     *
     * Populates the message's `id` and the four corner fields (`left_top`, `left_bottom`,
     * `right_bottom`, `right_top`) from this object's `points` in order.
     *
     * @pre `points.size() == 4`
     * @return rmcs_msgs::msg::ArmorPlate Message with `id` set from `id` and corners assigned
     *         from `points[0]` through `points[3]` respectively.
     */
    explicit operator rmcs_msgs::msg::ArmorPlate() const {
        assert(points.size() == 4);
        rmcs_msgs::msg::ArmorPlate armor;
        armor.id             = static_cast<uint16_t>(id);
        armor.left_top.x     = points[0].x;
        armor.left_top.y     = points[0].y;
        armor.left_bottom.x  = points[1].x;
        armor.left_bottom.y  = points[1].y;
        armor.right_bottom.x = points[2].x;
        armor.right_bottom.y = points[2].y;
        armor.right_top.x    = points[3].x;
        armor.right_top.y    = points[3].y;
        return armor;
    }

    /**
     * @brief Compute the centroid of the armor plate's corner points.
     *
     * Calculates the average of the four corner points and returns their center
     * in the same coordinate space as the points.
     *
     * @return cv::Point2f Centroid (average) of the four corner points.
     * @throws std::runtime_error If the armor plate does not contain exactly four points.
     */
    [[nodiscard]] cv::Point2f center() const {
        if (points.size() != 4) {
            throw std::runtime_error("Invalid ArmorPlate object");
        }
        return (points[0] + points[1] + points[2] + points[3]) / 4;
    }

    std::vector<cv::Point2f> points;
    rmcs_msgs::ArmorID id;
    bool is_large_armor;
};

namespace whitelist_code {
constexpr uint8_t Hero        = 0x1;
constexpr uint8_t Engineer    = 0x2;
constexpr uint8_t InfantryIII = 0x4;
constexpr uint8_t InfantryIV  = 0x8;
constexpr uint8_t InfantryV   = 0x10;
constexpr uint8_t Sentry      = 0x20;
constexpr uint8_t Outpost     = 0x40;
constexpr uint8_t Base        = 0x80;
} // namespace whitelist_code

} // namespace rmcs_auto_aim