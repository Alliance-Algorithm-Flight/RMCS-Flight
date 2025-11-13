/**
 * @file src/tongji_auto_aim_initializer.cpp
 * @brief Initializer component for tongji auto-aim system
 */

#include <cstddef>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/robot_color.hpp>
#include <rmcs_msgs/robot_id.hpp>

namespace rmcs_tongji_auto_aim {

/**
 * @brief 自瞄系统初始化器
 *
 * 负责根据裁判系统信息确定目标颜色和白名单
 * 与 rmcs_auto_aim::AutoAimInitializer 功能完全一致
 */
class TongjiAutoAimInitializer
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    /**
     * @brief Constructs the TongjiAutoAimInitializer component and configures its ROS2 node and interfaces.
     *
     * @details Creates the node with automatic parameter declaration from overrides enabled, registers
     * the input interfaces "/predefined/update_count" and "/referee/id" (the referee input is required),
     * registers the output interfaces "/auto_aim/target_color" and "/auto_aim/whitelist", and logs initialization.
     */
    TongjiAutoAimInitializer()
        : rclcpp::Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {

        // 注册输入接口
        register_input("/predefined/update_count", update_count_);
        register_input("/referee/id", robot_msg_, false);

        // 注册输出接口
        register_output("/auto_aim/target_color", target_color_);
        register_output("/auto_aim/whitelist", whitelist_);

        RCLCPP_INFO(this->get_logger(), "TongjiAutoAimInitializer initialized.");
    }

    /**
     * @brief Initialize whitelist on first run and determine the auto-aim target color.
     *
     * On the first update (when update_count_ equals 0) sets the whitelist output to 0.
     * Then sets target_color_ based on referee information: if referee data is available,
     * selects the opposite color of the robot's own color (RED -> BLUE, otherwise RED);
     * if referee data is not available, defaults target_color_ to RED.
     */
    void update() override {
        // 首次更新时初始化白名单
        if (*update_count_ == 0) {
            // 白名单:如果目标不在此列表,将被忽略
            // 0 表示所有目标都在黑名单(需要根据实际需求配置)
            *whitelist_ = 0;
        }

        // 根据裁判系统信息确定目标颜色
        if (robot_msg_.ready()) {
            auto my_color = robot_msg_->color();
            if (my_color == rmcs_msgs::RobotColor::RED) {
                *target_color_ = rmcs_msgs::RobotColor::BLUE;
            } else {
                *target_color_ = rmcs_msgs::RobotColor::RED;
            }
        } else {
            // 如果裁判系统未就绪,使用默认配置
            *target_color_ = rmcs_msgs::RobotColor::RED;
        }
    }

private:
    // 输入接口
    InputInterface<size_t> update_count_;
    InputInterface<rmcs_msgs::RobotId> robot_msg_;

    // 输出接口
    OutputInterface<rmcs_msgs::RobotColor> target_color_;
    OutputInterface<uint8_t> whitelist_;
};

}  // namespace rmcs_tongji_auto_aim

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_tongji_auto_aim::TongjiAutoAimInitializer, rmcs_executor::Component)