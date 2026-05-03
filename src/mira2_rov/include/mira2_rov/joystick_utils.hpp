#include <rclcpp/rclcpp.hpp>
#include <custom_msgs/msg/commands.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <cstdlib>

class JoystickController : public rclcpp::Node {
public:
    JoystickController() : Node("joystick_controller") {
        const char *qgc_env = std::getenv("MIRA_TELEOP_QGC");
        qgc_mode_ = (qgc_env != nullptr && std::string(qgc_env) == "1");

        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&JoystickController::joyCallback, this, std::placeholders::_1));
        base_pwm_pub_ = this->create_publisher<custom_msgs::msg::Commands>("/master/commands", 10);

        msg_to_pub_.arm = 0;
        msg_to_pub_.mode = "STABILIZE";
        msg_to_pub_.forward = 1500;
        msg_to_pub_.lateral = 1500;
        msg_to_pub_.thrust = 1500;
        msg_to_pub_.pitch = 1500;
        msg_to_pub_.yaw = 1500;
        msg_to_pub_.roll = 1500;
        msg_to_pub_.servo1 = 1500;
        msg_to_pub_.servo2 = 1500;

        sensitivity_all_axes_ = 0.6;
        sensitivity_yaw_ = 0.5;
        sensitivity_pitch_ = 0.5;

        prev_msg_ = 0;
        arm_disarm_ = 0;

        printConfig();
    }

private:
    bool qgc_mode_;

    void printConfig() {
        RCLCPP_INFO(this->get_logger(), "--------------------------------------------");
        if (qgc_mode_) {
            RCLCPP_INFO(this->get_logger(), "Teleop mode : QGC  (MIRA_TELEOP_QGC=1)");
        } else {
            RCLCPP_INFO(this->get_logger(), "Teleop mode : Default  (MIRA_TELEOP_QGC unset or 0)");
        }
        RCLCPP_INFO(this->get_logger(), "  Left  stick X  axes[0] -> Lateral");
        RCLCPP_INFO(this->get_logger(), "  Left  stick Y  axes[1] -> Forward / Back");
        RCLCPP_INFO(this->get_logger(), "  Right stick X  axes[3] -> Yaw");
        if (qgc_mode_) {
            RCLCPP_INFO(this->get_logger(), "  Right stick Y  axes[4] -> Thrust  (up = ascend)");
            RCLCPP_INFO(this->get_logger(), "  Right trigger  axes[2] -> Pitch up");
            RCLCPP_INFO(this->get_logger(), "  Left  trigger  axes[5] -> Pitch down");
        } else {
            RCLCPP_INFO(this->get_logger(), "  Right stick Y  axes[4] -> Pitch");
            RCLCPP_INFO(this->get_logger(), "  Left  trigger  axes[5] -> Thrust up");
            RCLCPP_INFO(this->get_logger(), "  Right trigger  axes[2] -> Thrust down");
        }
        RCLCPP_INFO(this->get_logger(), "  buttons[0] -> Arm / Disarm toggle");
        RCLCPP_INFO(this->get_logger(), "  buttons[1] -> Mode MANUAL");
        RCLCPP_INFO(this->get_logger(), "  buttons[2] -> Mode ALT_HOLD (depth hold)");
        RCLCPP_INFO(this->get_logger(), "  buttons[3] -> Mode STABILIZE");
        RCLCPP_INFO(this->get_logger(), "  buttons[4] -> Roll left");
        RCLCPP_INFO(this->get_logger(), "  buttons[5] -> Roll right");
        RCLCPP_INFO(this->get_logger(), "--------------------------------------------");
        if (qgc_mode_) {
            RCLCPP_INFO(this->get_logger(), "Tip: unset MIRA_TELEOP_QGC (or set to 0) to use default controls");
        } else {
            RCLCPP_INFO(this->get_logger(), "Tip: set MIRA_TELEOP_QGC=1 before launching to use QGC-style controls");
        }
    }

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        prev_msg_ = arm_disarm_;
        arm_disarm_ = msg->buttons[0];
        int mode_manual = msg->buttons[1];
        int mode_depth_hold = msg->buttons[2];
        int mode_stabilise = msg->buttons[3];

        int roll_left_button = msg->buttons[4];
        int roll_right_button = msg->buttons[5];

        if (qgc_mode_) {
            // Right stick Y controls thrust; triggers control pitch
            msg_to_pub_.thrust = 1500 + ((msg->axes[4]) * 400) * sensitivity_all_axes_;
            // Right trigger (axes[2]) = pitch up, left trigger (axes[5]) = pitch down
            msg_to_pub_.pitch = 1500 + ((((msg->axes[2]) + 1) * -200) + (((msg->axes[5]) + 1) * 200)) * sensitivity_pitch_;
        } else {
            // Right stick Y controls pitch; triggers control thrust
            msg_to_pub_.pitch = 1500 + ((msg->axes[4]) * 400) * sensitivity_pitch_;
            // Left trigger (axes[5]) = thrust up, right trigger (axes[2]) = thrust down
            msg_to_pub_.thrust = 1500 + ((((msg->axes[5]) + 1) * -200) + (((msg->axes[2]) + 1) * 200)) * sensitivity_all_axes_;
        }

        msg_to_pub_.forward = 1500 + ((msg->axes[1]) * 400) * sensitivity_all_axes_;
        msg_to_pub_.lateral = 1500 + ((msg->axes[0]) * -400) * sensitivity_all_axes_;
        msg_to_pub_.yaw = 1500 + (((msg->axes[3]) * -400)) * sensitivity_yaw_;

        if (arm_disarm_ == 1 && prev_msg_ == 0) {
            if (msg_to_pub_.arm == 0) {
                msg_to_pub_.arm = 1;
                RCLCPP_WARN(this->get_logger(), "VEHICLE ARMED");
            } else {
                msg_to_pub_.arm = 0;
                RCLCPP_WARN(this->get_logger(), "VEHICLE DISARMED");
            }
        }

        if (roll_left_button == 1) {
            msg_to_pub_.roll = 1400;
        } else if (roll_right_button == 1) {
            msg_to_pub_.roll = 1600;
        } else {
            msg_to_pub_.roll = 1500;
        }

        if (mode_stabilise == 1 && msg_to_pub_.mode != "STABILIZE") {
            msg_to_pub_.mode = "STABILIZE";
            RCLCPP_INFO(this->get_logger(), "Mode changed to STABILIZE");
        }
        if (mode_depth_hold == 1 && msg_to_pub_.mode != "ALT_HOLD") {
            msg_to_pub_.mode = "ALT_HOLD";
            RCLCPP_INFO(this->get_logger(), "Mode changed to ALT_HOLD");
        }
        if (mode_manual == 1 && msg_to_pub_.mode != "MANUAL") {
            msg_to_pub_.mode = "MANUAL";
            RCLCPP_INFO(this->get_logger(), "Mode changed to MANUAL");
        }

        base_pwm_pub_->publish(msg_to_pub_);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<custom_msgs::msg::Commands>::SharedPtr base_pwm_pub_;
    custom_msgs::msg::Commands msg_to_pub_;
    float sensitivity_all_axes_, sensitivity_yaw_, sensitivity_pitch_;
    int prev_msg_, arm_disarm_;
};
