#include <rclcpp/rclcpp.hpp>
#include <custom_msgs/msg/commands.hpp>
#include <custom_msgs/msg/telemetry.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <control_utils/control_utils.hpp>
#include <std_msgs/msg/float32.hpp>
#include <cmath>
#include <memory>

class GateNavigator {
public:
    GateNavigator(const rclcpp::Node::SharedPtr& node) 
        : node_(node),
          depth_pid_obj_("depth", node),
          yaw_pid_obj_("yaw", node),
          lateral_pid_obj_("lateral", node),
          forward_pid_obj_("forward", node),
          image_center_x_(320) {  // Assuming 640x480 resolution
        
        // Initialize PID parameters
        initializePIDControllers();

        // Setup communications
        pwm_publisher_ = node_->create_publisher<custom_msgs::msg::Commands>("/master/commands", 1);
        telemetry_sub_ = node_->create_subscription<custom_msgs::msg::Telemetry>(
            "/master/telemetry", 1, std::bind(&GateNavigator::telemetryCallback, this, std::placeholders::_1));
        detection_sub_ = node_->create_subscription<vision_msgs::msg::Detection2DArray>(
            "detectnet/detections", 1, std::bind(&GateNavigator::detectionCallback, this, std::placeholders::_1));
        pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            "gate_pose", 1, std::bind(&GateNavigator::poseCallback, this, std::placeholders::_1));
        heading_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
            "/master/heading", 1, std::bind(&GateNavigator::headingCallback, this, std::placeholders::_1));

        // Timer for main loop
        timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&GateNavigator::mainLoop, this));
        
        RCLCPP_INFO(node_->get_logger(), "Gate Navigator initialized");
    }

private:
    void initializePIDControllers() {
        // Depth PID
        depth_pid_obj_.kp = 5.5;
        depth_pid_obj_.ki = 0.03;
        depth_pid_obj_.kd = 31.5;
        depth_pid_obj_.base_offset = 1580;
        depth_setpoint_ = 1069.0;
        
        // Yaw PID
        yaw_pid_obj_.kp = 3.18;
        yaw_pid_obj_.ki = 0.01;
        yaw_pid_obj_.kd = 7.2;
        yaw_pid_obj_.base_offset = 1500;
        yaw_setpoint_ = 280;
        
        // Lateral PID
        lateral_pid_obj_.kp = -0.5;
        lateral_pid_obj_.ki = -0.05;
        lateral_pid_obj_.kd = -2.0;
        lateral_pid_obj_.base_offset = 1500;
        lateral_setpoint_ = 0.0;
        
        // Forward PID
        forward_pid_obj_.kp = -0.8;
        forward_pid_obj_.ki = -0.1;
        forward_pid_obj_.kd = -4.0;
        forward_pid_obj_.base_offset = 1500;
        forward_setpoint_ = 0.0;
    }

    // ROS2 Node
    rclcpp::Node::SharedPtr node_;

    // PID Controllers
    PID_Controller depth_pid_obj_;
    PID_Controller yaw_pid_obj_;
    PID_Controller lateral_pid_obj_;
    PID_Controller forward_pid_obj_;

    // Current state
    int image_center_x_;
    int current_yaw_ = 0;

    // Publishers/Subscribers
    rclcpp::Publisher<custom_msgs::msg::Commands>::SharedPtr pwm_publisher_;
    rclcpp::Subscription<custom_msgs::msg::Telemetry>::SharedPtr telemetry_sub_;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr heading_sub_;

    bool software_arm_flag_ = false;
    custom_msgs::msg::Commands cmd_pwm_;

    // PID errors and setpoints
    double forward_error_ = 0.0, forward_setpoint_{};
    double lateral_error_ = 0.0, lateral_setpoint_{};
    double depth_error_ = 0.0, depth_setpoint_{};
    int yaw_error_ = 0, yaw_setpoint_{};

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time init_time_;

    void headingCallback(const std_msgs::msg::Float32::SharedPtr msg) {
        current_yaw_ = static_cast<int>(msg->data);
        yaw_error_ = static_cast<int>(fmod((yaw_setpoint_ - current_yaw_ + 180), 360) - 180);
    }

    void telemetryCallback(const custom_msgs::msg::Telemetry::SharedPtr msg) {
        double current_depth = msg->external_pressure;
        depth_error_ = depth_setpoint_ - current_depth;
    }

    void detectionCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg) {
        if (!msg->detections.empty()) {
            double gate_center_x = msg->detections[0].bbox.center.position.x;
            lateral_error_ = image_center_x_ - gate_center_x;
        }
    }

    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        double distance = msg->pose.position.x;  // Assuming x is forward distance
        // Can be used for forward error calculation if needed
    }

    void mainLoop() {
        if (!software_arm_flag_) {
            software_arm_flag_ = true;
            init_time_ = node_->now();
            yaw_pid_obj_.emptyError();
        }

        cmd_pwm_.mode = "STABILIZE";
        rclcpp::Time time_now = node_->now();
        cmd_pwm_.arm = true;

        if ((time_now - init_time_).seconds() > 5.0) {
            if (software_arm_flag_) {
                float pid_depth = depth_pid_obj_.pid_control(
                    depth_error_, (time_now - init_time_).seconds(), false);

                float pid_yaw = yaw_pid_obj_.pid_control(
                    yaw_error_, (time_now - init_time_).seconds(), false);

                float pid_lateral = lateral_pid_obj_.pid_control(
                    lateral_error_, (time_now - init_time_).seconds(), false);

                if ((time_now - init_time_).seconds() < 10.0) {
                    cmd_pwm_.forward = 1500;
                    cmd_pwm_.lateral = 1500;
                    cmd_pwm_.thrust = static_cast<int>(pid_depth);
                    cmd_pwm_.yaw = static_cast<int>(pid_yaw);
                    RCLCPP_INFO(node_->get_logger(), "Sinking");
                } else if ((time_now - init_time_).seconds() < 20.0) {
                    cmd_pwm_.forward = 1800;
                    cmd_pwm_.lateral = 1500;
                    cmd_pwm_.thrust = static_cast<int>(pid_depth);
                    cmd_pwm_.yaw = static_cast<int>(pid_yaw);
                    RCLCPP_INFO(node_->get_logger(), "Moving forward");
                }
            } else {
                depth_pid_obj_.emptyError();
                yaw_pid_obj_.emptyError();
                cmd_pwm_.arm = false;
                cmd_pwm_.mode = "STABILIZE";
                cmd_pwm_.forward = 1500;
                cmd_pwm_.lateral = 1500;
                cmd_pwm_.thrust = 1500;
                cmd_pwm_.yaw = 1500;
            }
            pwm_publisher_->publish(cmd_pwm_);
        } else {
            yaw_setpoint_ = current_yaw_;
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("gate_navigator");
    auto navigator = std::make_shared<GateNavigator>(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
