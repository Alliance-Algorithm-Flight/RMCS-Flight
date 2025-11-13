/**
 * @file number_identifier.hpp
 * @author Lorenzo Feng (lorenzo.feng@njust.edu.cn)
 * @brief
 * @version 0.1
 * @date 2024-06-07
 *
 * (C)Copyright: NJUST.Alliance - All rights reserved
 *
 */
#pragma once

#include <string>

#include <opencv2/opencv.hpp>

#include "core/identifier/armor/armor.hpp"

namespace rmcs_auto_aim {
class NumberIdentifier {
private:
    cv::dnn::Net _net;

public:
    explicit NumberIdentifier(const std::string& model_path);
    /**
 * @brief Deleted copy constructor to prevent copying of NumberIdentifier instances.
 *
 * Copying is disabled to avoid duplicating the owned DNN network resource.
 */
NumberIdentifier(const NumberIdentifier&) = delete;
    /**
 * @brief Deleted move constructor; moving NumberIdentifier instances is disabled.
 *
 * Prevents transfer of ownership of the internal neural-network resource.
 */
NumberIdentifier(NumberIdentifier&&)      = delete;

    bool Identify(const cv::Mat& imgGray, ArmorPlate& armor, const uint8_t& whitelist);
};
} // namespace rmcs_auto_aim