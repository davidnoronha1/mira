#include <rclcpp/rclcpp.hpp>
#include <custom_msgs/msg/commands.hpp>
#include <sensor_msgs/msg/joy.hpp>

class JoystickController : public rclcpp::Node {
public:
    JoystickController() : Node("joystick_controller") {
        // Initialize publishers and subscribers
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&JoystickController::joyCallback, this, std::placeholders::_1));
        base_pwm_pub_ = this->create_publisher<custom_msgs::msg::Commands>("/master/commands", 10);

        // Initialize message fields
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

        prev_msg_ = 0;
        arm_disarm_ = 0;
    }

private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        int pitch_button = msg->buttons[5];
        int mode_stabilise = msg->buttons[1];
        int mode_acro = msg->buttons[7];
        int mode_manual = msg->buttons[3];
        int yaw_hold_button = msg->buttons[6];
        int pitch_up_button = msg->buttons[4];
        prev_msg_ = arm_disarm_;
        arm_disarm_ = msg->buttons[0];

        msg_to_pub_.pitch = 1500;
        msg_to_pub_.roll = 1500 + (((msg->buttons[4]) * -400) + ((msg->buttons[5]) * 400)) * sensitivity_all_axes_;
        msg_to_pub_.thrust = 1500 + ((((msg->axes[5]) + 1) * -200) + (((msg->axes[2]) + 1) * 200)) * sensitivity_all_axes_;
        msg_to_pub_.forward = 1500 + ((msg->axes[4]) * 400) * 1; // sensitivity_all_axes_;
        msg_to_pub_.lateral = 1500 + ((msg->axes[3]) * -400) * sensitivity_all_axes_;
        msg_to_pub_.yaw = 1500 + (((msg->axes[0]) * -400)) * sensitivity_yaw_;

        if (arm_disarm_ == 1 && prev_msg_ == 0) {
            if (msg_to_pub_.arm == 0) {
                msg_to_pub_.arm = 1;
                RCLCPP_WARN(this->get_logger(), "VEHICLE ARMED");
            } else {
                msg_to_pub_.arm = 0;
                RCLCPP_WARN(this->get_logger(), "VEHICLE DISARMED");
            }
        }

        if (pitch_button == 1) {
            msg_to_pub_.pitch = 1400;
        } else if (pitch_up_button == 1) {
            msg_to_pub_.pitch = 1600;
        } else {
            msg_to_pub_.pitch = 1500;
        }

        if (mode_stabilise == 1 && msg_to_pub_.mode != "STABILIZE") {
            msg_to_pub_.mode = "STABILIZE";
            RCLCPP_INFO(this->get_logger(), "Mode changed to STABILIZE");
        }
        if (mode_acro == 1 && msg_to_pub_.mode != "ACRO") {
            msg_to_pub_.mode = "ACRO";
            RCLCPP_INFO(this->get_logger(), "Mode changed to ACRO");
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
    float sensitivity_all_axes_, sensitivity_yaw_;
    int prev_msg_, arm_disarm_;
};

