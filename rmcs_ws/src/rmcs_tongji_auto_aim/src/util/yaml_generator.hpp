/**
 * @file src/util/yaml_generator.hpp
 * @brief YAML configuration file generator for tongji fire controller
 */

#pragma once

#include <fstream>
#include <sstream>
#include <string>
#include <stdexcept>

namespace rmcs_tongji_auto_aim::util {

/**
 * @brief 生成 tongji FireController 所需的 YAML 配置文件
 */
class YamlGenerator {
public:
    /**
     * @brief Generate a temporary YAML configuration file for the tongji FireController.
     *
     * Writes categorized parameter groups (fire_controller, aiming_solver, aime_point_chooser, fire_decision)
     * to a temporary file and returns its path.
     *
     * @param control_delay_s Control delay in seconds.
     * @param bullet_speed Bullet speed in meters per second.
     * @param yaw_offset_rad Yaw offset in radians.
     * @param pitch_offset_rad Pitch offset in radians.
     * @param comming_angle_rad Incoming angle threshold in radians.
     * @param leaving_angle_rad Leaving angle threshold in radians.
     * @param auto_fire Whether automatic firing is enabled.
     * @param first_tolerance_rad Near-distance firing tolerance in radians.
     * @param second_tolerance_rad Far-distance firing tolerance in radians.
     * @param judge_distance Distance threshold in meters used for decision switching.
     * @return std::string Path to the created temporary YAML file.
     * @throws std::runtime_error If the temporary YAML file cannot be created or opened for writing.
     */
    static std::string generateFireControlYaml(
        double control_delay_s,
        double bullet_speed,
        double yaw_offset_rad,
        double pitch_offset_rad,
        double comming_angle_rad,
        double leaving_angle_rad,
        bool auto_fire,
        double first_tolerance_rad,
        double second_tolerance_rad,
        double judge_distance) {

        // 创建临时文件路径
        std::string temp_path = "/tmp/rmcs_tongji_fire_control.yaml";

        // 打开文件写入
        std::ofstream ofs(temp_path, std::ios::trunc);
        if (!ofs.is_open()) {
            throw std::runtime_error("Failed to create temp YAML file: " + temp_path);
        }

        // 写入 YAML 内容 (参数已经转换为弧度)
        ofs << "#####-----fire_controller parameters-----#####\n";
        ofs << "control_delay_s: " << control_delay_s << "\n";
        ofs << "\n";

        ofs << "#####-----aiming_solver parameters-----#####\n";
        ofs << "yaw_offset: " << yaw_offset_rad << "\n";
        ofs << "pitch_offset: " << pitch_offset_rad << "\n";
        ofs << "bullet_speed: " << bullet_speed << "\n";
        ofs << "\n";

        ofs << "#####-----aime_point_chooser parameters-----#####\n";
        ofs << "comming_angle: " << comming_angle_rad << "\n";
        ofs << "leaving_angle: " << leaving_angle_rad << "\n";
        ofs << "\n";

        ofs << "#####-----fire_decision parameters-----#####\n";
        ofs << "auto_fire: " << (auto_fire ? "true" : "false") << "\n";
        ofs << "first_tolerance: " << first_tolerance_rad << "\n";
        ofs << "second_tolerance: " << second_tolerance_rad << "\n";
        ofs << "judge_distance: " << judge_distance << "\n";

        ofs.close();

        return temp_path;
    }
};

}  // namespace rmcs_tongji_auto_aim::util