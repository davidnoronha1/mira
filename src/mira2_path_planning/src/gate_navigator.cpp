#include <rclcpp/rclcpp.hpp>
#include <custom_msgs/msg/commands.hpp>
#include <custom_msgs/msg/telemetry.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <control_utils/control_utils.hpp>
#include <std_msgs/msg/float32.hpp>
#include <cmath>

class GateNavigator : public rclcpp::Node {
public:
    GateNavigator() : Node("gate_navigator"),
                      depth_pid_obj(), yaw_pid_obj(), lateral_pid_obj(), forward_pid_obj(),
                      image_center_x(320) {  // Assuming 640x480 resolution
        // Initialize PID parameters
        depth_pid_obj.kp = 5.5;    depth_pid_obj.ki = 0.03;    depth_pid_obj.kd = 31.5;  depth_pid_obj.base_offset = 1580;   depth_setpoint = 1069;
        yaw_pid_obj.kp = 3.18;      yaw_pid_obj.ki = 0.01;      yaw_pid_obj.kd = 7.2;      yaw_pid_obj.base_offset = 1500;     yaw_setpoint = 280;
        lateral_pid_obj.kp = -0.5;  lateral_pid_obj.ki = -0.05; lateral_pid_obj.kd = -2.0;  lateral_pid_obj.base_offset = 1500; 
        forward_pid_obj.kp = -0.8;  forward_pid_obj.ki = -0.1;  forward_pid_obj.kd = -4.0;  forward_pid_obj.base_offset = 1500;

        // Setup communications
        pwm_publisher = this->create_publisher<custom_msgs::msg::Commands>("/master/commands", 1);
        telemetry_sub = this->create_subscription<custom_msgs::msg::Telemetry>(
            "/master/telemetry", 1, std::bind(&GateNavigator::telemetryCallback, this, std::placeholders::_1));
        detection_sub = this->create_subscription<vision_msgs::msg::Detection2DArray>(
            "detectnet/detections", 1, std::bind(&GateNavigator::detectionCallback, this, std::placeholders::_1));
        pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "gate_pose", 1, std::bind(&GateNavigator::poseCallback, this, std::placeholders::_1));
        heading_sub = this->create_subscription<std_msgs::msg::Float32>(
            "/master/heading", 1, std::bind(&GateNavigator::headingCallback, this, std::placeholders::_1));

        // Timer for main loop
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&GateNavigator::mainLoop, this));
    }

private:
    // PID Controllers
    PID_Controller depth_pid_obj;
    PID_Controller yaw_pid_obj;
    PID_Controller lateral_pid_obj;
    PID_Controller forward_pid_obj;

    // Current state
    int image_center_x;
    int current_yaw;

    // Publishers/Subscribers
    rclcpp::Publisher<custom_msgs::msg::Commands>::SharedPtr pwm_publisher;
    rclcpp::Subscription<custom_msgs::msg::Telemetry>::SharedPtr telemetry_sub;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr heading_sub;

    custom_msgs::msg::Commands cmd_msg_;

    bool software_arm_flag = false;
    custom_msgs::msg::Commands cmd_pwm;

    // PID errors
    double forward_error, forward_setpoint;
    double lateral_error, lateral_setpoint;
    double depth_error, depth_setpoint;
    int yaw_error, yaw_setpoint;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time init_time;

    void headingCallback(const std_msgs::msg::Float32::SharedPtr msg) {
        current_yaw = msg->data;
        yaw_error = fmod((yaw_setpoint - current_yaw + 180), 360) - 180;
    }

    void telemetryCallback(const custom_msgs::msg::Telemetry::SharedPtr msg) {
        double current_depth = msg->external_pressure;
        depth_error = depth_setpoint - current_depth;
    }

    void detectionCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg) {
        if (!msg->detections.empty()) {
            double gate_center_x = msg->detections[0].bbox.center.position.x;
            lateral_error = image_center_x - gate_center_x;
        }
    }

    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        double distance = msg->pose.position.x;  // Assuming x is forward distance
    }

    void mainLoop() {
        if (!software_arm_flag) {
            software_arm_flag = true;
            init_time = this->now();
            yaw_pid_obj.emptyError();
        }

        cmd_pwm.mode = "STABILIZE";
        rclcpp::Time time_now = this->now();
        cmd_pwm.arm = true;

        if ((time_now - init_time).seconds() > 5.0) {
            if (software_arm_flag) {
                float pid_depth = depth_pid_obj.pid_control(
                    depth_error, (time_now - init_time).seconds(), false);

                float pid_yaw = yaw_pid_obj.pid_control(
                    yaw_error, (time_now - init_time).seconds(), false);

                float pid_lateral = lateral_pid_obj.pid_control(
                    lateral_error, (time_now - init_time).seconds(), false);

                if ((time_now - init_time).seconds() < 10.0) {
                    cmd_pwm.forward = 1500;
                    cmd_pwm.lateral = 1500;
                    cmd_pwm.thrust = pid_depth;
                    cmd_pwm.yaw = pid_yaw;
                    RCLCPP_INFO(this->get_logger(), "Sinking");
                } else if ((time_now - init_time).seconds() < 20.0) {
                    cmd_pwm.forward = 1800;
                    cmd_pwm.lateral = 1500;
                    cmd_pwm.thrust = pid_depth;
                    cmd_pwm.yaw = pid_yaw;
                    RCLCPP_INFO(this->get_logger(), "Moving forward");
                }
            } else {
                depth_pid_obj.emptyError();
                yaw_pid_obj.emptyError();
                cmd_pwm.arm = false;
                cmd_pwm.mode = "STABILIZE";
                cmd_pwm.forward = 1500;
                cmd_pwm.lateral = 1500;
                cmd_pwm.thrust = 1500;
                cmd_pwm.yaw = 1500;
            }
            pwm_publisher->publish(cmd_pwm);
        } else {
            yaw_setpoint = current_yaw;
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GateNavigator>());
    rclcpp::shutdown();
    return 0;
}
