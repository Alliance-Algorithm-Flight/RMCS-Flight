#include <cstddef>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/robot_color.hpp>
#include <rmcs_msgs/robot_id.hpp>

#include "core/identifier/armor/armor.hpp"

namespace rmcs_auto_aim {

class AutoAimInitializer
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    /**
     * @brief Constructs the AutoAimInitializer node and registers its IO interfaces.
     *
     * Initializes the ROS2 node with automatic declaration of parameters from overrides,
     * registers the input interfaces "/predefined/update_count" and "/referee/id" (the latter not required),
     * and registers the output interfaces "/auto_aim/target_color" and "/auto_aim/whitelist".
     * Emits an informational log message when initialization completes.
     */
    AutoAimInitializer()
        : rclcpp::Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        register_input("/predefined/update_count", update_count_);
        register_input("/referee/id", robot_msg_, false);

        register_output("/auto_aim/target_color", target_color_);
        register_output("/auto_aim/whitelist", whitelist_);

        RCLCPP_INFO(this->get_logger(), "AutoAimInitializer initialized.");
    }

    /**
     * @brief Initialize or update auto-aim outputs based on inputs.
     *
     * When the incoming update count equals 0, sets the whitelist output to 0.
     * If a robot identity message is available, sets the target color to the
     * opposite of this node's color (RED -> BLUE, otherwise RED). If the robot
     * identity message is not available, sets the target color to RED.
     *
     * The whitelist value is a bitmask using Tongji semantics:
     * - bit = 0: corresponding target is attackable
     * - bit = 1: corresponding target is masked (Base bit must be explicitly set to 1 to allow attack)
     */
    void update() override {
        if (*update_count_ == 0) {
            // clang-format off
            *whitelist_ = // Whitelist bitmask (Tongji semantics):
                0;        // - bit = 0: 对应目标可攻击
                          // - bit = 1: 屏蔽对应目标（Base 位需显式置 1 才允许攻击）
            // clang-format on
        }
        if (robot_msg_.ready()) {
            auto my_color = robot_msg_->color();
            if (my_color == rmcs_msgs::RobotColor::RED) {
                *target_color_ = rmcs_msgs::RobotColor::BLUE;
            } else {
                *target_color_ = rmcs_msgs::RobotColor::RED;
            }
        } else {
            // RCLCPP_INFO(this->get_logger(), "Using information from the configuration file.");
            *target_color_ = rmcs_msgs::RobotColor::RED;
        }
    }

private:
    InputInterface<size_t> update_count_;
    InputInterface<rmcs_msgs::RobotId> robot_msg_;

    OutputInterface<rmcs_msgs::RobotColor> target_color_;
    OutputInterface<uint8_t> whitelist_;
};
} // namespace rmcs_auto_aim

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_auto_aim::AutoAimInitializer, rmcs_executor::Component)