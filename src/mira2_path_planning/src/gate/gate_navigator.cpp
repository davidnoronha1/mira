#include <utils/control.hpp>
#include <rclcpp/rclcpp.hpp>
#include <custom_msgs/msg/commands_normalized.hpp>
#include <custom_msgs/msg/telemetry.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <utils/control.hpp>
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
          image_center_x_(0.5) {  // Assuming 640x480 resolution
        
        // Initialize PID parameters
        initializePIDControllers();

        // Setup communications
        effort_publisher_ = node_->create_publisher<custom_msgs::msg::CommandsNormalized>("mira/command", 1);
        telemetry_sub_ = node_->create_subscription<custom_msgs::msg::Telemetry>(
            "mira/telemetry", 1, std::bind(&GateNavigator::telemetryCallback, this, std::placeholders::_1));
        detection_sub_ = node_->create_subscription<vision_msgs::msg::Detection2DArray>(
            "mira/perception/front/objects", 1, std::bind(&GateNavigator::detectionCallback, this, std::placeholders::_1));
        pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            "gate_pose", 1, std::bind(&GateNavigator::poseCallback, this, std::placeholders::_1));

        // Timer for main loop
        timer_ = node_->create_wall_timer(
            std::chrono::duration<double>(1.0 / utils::checked_parameter(*node_, "control_rate", 10.0, utils::positive)), std::bind(&GateNavigator::mainLoop, this));
        
        RCLCPP_INFO(node_->get_logger(), "Gate Navigator initialized");
    }

private:
    void initializePIDControllers() {
        depth_setpoint_ = utils::parameter(*node_, "depth_setpoint", 1069.0);
        yaw_setpoint_ = utils::parameter(*node_, "yaw_setpoint", 280);
        lateral_setpoint_ = 0.0;
        forward_setpoint_ = 0.0;
        utils::parameter(*node_, "startup_duration", 5.0);
        utils::parameter(*node_, "sink_duration", 5.0);
        utils::parameter(*node_, "forward_duration", 10.0);
        utils::parameter(*node_, "forward_effort", 0.75);

    }

    // ROS2 Node
    rclcpp::Node::SharedPtr node_;

    // PID Controllers
    PID_Controller depth_pid_obj_;
    PID_Controller yaw_pid_obj_;
    PID_Controller lateral_pid_obj_;
    PID_Controller forward_pid_obj_;

    // Current state
    double image_center_x_;
    int current_yaw_ = 0;

    // Publishers/Subscribers
    rclcpp::Publisher<custom_msgs::msg::CommandsNormalized>::SharedPtr effort_publisher_;
    rclcpp::Subscription<custom_msgs::msg::Telemetry>::SharedPtr telemetry_sub_;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

    bool software_arm_flag_ = false;
    custom_msgs::msg::CommandsNormalized cmd_effort_;

    // PID errors and setpoints
    double forward_error_ = 0.0, forward_setpoint_{};
    double lateral_error_ = 0.0, lateral_setpoint_{};
    double depth_error_ = 0.0, depth_setpoint_{};
    int yaw_error_ = 0, yaw_setpoint_{};

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time init_time_;

    void telemetryCallback(const custom_msgs::msg::Telemetry::SharedPtr msg) {
        double current_depth = msg->external_pressure.fluid_pressure;
        depth_error_ = depth_setpoint_ - current_depth;
        if (std::isfinite(msg->heading_ned_degrees.data)) {
            current_yaw_ = msg->heading_ned_degrees.data;
            yaw_error_ = static_cast<int>(fmod((yaw_setpoint_ - current_yaw_ + 180), 360) - 180);
        }
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

        cmd_effort_.mode = "STABILIZE";
        rclcpp::Time time_now = node_->now();
        cmd_effort_.arm = true;

        if ((time_now - init_time_).seconds() > node_->get_parameter("startup_duration").as_double()) {
            if (software_arm_flag_) {
                float pid_depth = depth_pid_obj_.pid_control(
                    depth_error_, (time_now - init_time_).seconds(), false);

                float pid_yaw = yaw_pid_obj_.pid_control(
                    yaw_error_, (time_now - init_time_).seconds(), false);

                float pid_lateral = lateral_pid_obj_.pid_control(
                    lateral_error_, (time_now - init_time_).seconds(), false);

                if ((time_now - init_time_).seconds() < node_->get_parameter("startup_duration").as_double() + node_->get_parameter("sink_duration").as_double()) {
                    cmd_effort_.forward = 0.0;
                    cmd_effort_.lateral = 0.0;
                    cmd_effort_.thrust = static_cast<float>(pid_depth);
                    cmd_effort_.yaw = static_cast<float>(pid_yaw);
                    RCLCPP_INFO(node_->get_logger(), "Sinking");
                } else if ((time_now - init_time_).seconds() < node_->get_parameter("startup_duration").as_double() + node_->get_parameter("sink_duration").as_double() + node_->get_parameter("forward_duration").as_double()) {
                    cmd_effort_.forward = node_->get_parameter("forward_effort").as_double();
                    cmd_effort_.lateral = 0.0;
                    cmd_effort_.thrust = static_cast<float>(pid_depth);
                    cmd_effort_.yaw = static_cast<float>(pid_yaw);
                    RCLCPP_INFO(node_->get_logger(), "Moving forward");
                } else {
                    cmd_effort_ = custom_msgs::msg::CommandsNormalized();
                    cmd_effort_.mode = "STABILIZE";
                }
            } else {
                depth_pid_obj_.emptyError();
                yaw_pid_obj_.emptyError();
                cmd_effort_.arm = false;
                cmd_effort_.mode = "STABILIZE";
                cmd_effort_.forward = 0.0;
                cmd_effort_.lateral = 0.0;
                cmd_effort_.thrust = 0.0;
                cmd_effort_.yaw = 0.0;
            }
            utils::publish_mission_command(effort_publisher_, node_, cmd_effort_);
        } else {
            yaw_setpoint_ = current_yaw_;
            cmd_effort_.arm = false;
            utils::publish_mission_command(effort_publisher_, node_, cmd_effort_);
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
