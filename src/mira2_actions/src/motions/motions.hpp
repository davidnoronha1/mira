#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <control_utils/control_utils.hpp>
#include <custom_msgs/msg/commands.hpp>
#include <custom_msgs/msg/telemetry.hpp>
#include <vision_msgs/msg/bounding_box2_d_array.hpp>
#include <string>
#include <vector>
#include <cmath>
#include "../common.hpp"

/**
 * @brief ApproachBB: Approach a detected bounding box
 * 
 * Moves forward while using PID control to center the bounding box.
 * Returns SUCCESS when the bounding box fills a sufficient portion of the frame.
 */
class ApproachBB : public BT::StatefulActionNode
{
public:
    ApproachBB(const std::string& name, 
               const BT::NodeConfiguration& config,
               ROSState* ros_state);

    static BT::PortsList providedPorts();
    
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    void bbox_callback(const vision_msgs::msg::BoundingBox2DArray::SharedPtr msg);
    void publish_neutral();

    ROSState* ros_state_;
    rclcpp::Subscription<vision_msgs::msg::BoundingBox2DArray>::SharedPtr bb_sub_;

    // PID Controllers
    PID_Controller yaw_pid;
    PID_Controller lateral_pid;

    // Parameters
    std::string target_object_;
    double forward_pwm_;
    double frame_width_;
    double frame_height_;
    double success_bb_area_;
    double x_tolerance_;
    double bb_lost_timeout_;
    double timeout_;
    std::string flight_mode_;

    // Runtime state
    rclcpp::Time start_time_;
    rclcpp::Time last_detection_time_;
    
    // Latest bounding box info
    bool bb_found_;
    double bb_x_center_norm_;
    double bb_area_norm_;
};

/**
 * @brief RotateToTargetHeading: Rotate the vehicle to a specified heading
 * 
 * Uses PID control to rotate to a target heading.
 * Returns SUCCESS when the heading is within tolerance.
 */
class RotateToTargetHeading : public BT::StatefulActionNode
{
public:
    RotateToTargetHeading(const std::string& name,
                         const BT::NodeConfiguration& config,
                         ROSState* ros_state);

    static BT::PortsList providedPorts();
    
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    PID_Controller yaw_pid;
    ROSState* ros_state_;

    double target_heading_;
    double yaw_error_;

    rclcpp::Time start_time_;
    double tolerance_;
    double timeout_;
};

/**
 * @brief YawSweep: Perform an oscillating yaw sweep around a center heading
 * 
 * Uses the shared ROSState instance for telemetry and publishing commands.
 * Sweeps back and forth for a specified number of cycles.
 */
class YawSweep : public BT::StatefulActionNode
{
public:
    YawSweep(const std::string& name, 
             const BT::NodeConfiguration& config, 
             ROSState* ros_state);

    static BT::PortsList providedPorts();
    
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    void cleanup_and_publish_neutral();
    void publish_neutral();

    PID_Controller yaw_pid;
    ROSState* ros_state_;

    // Sweep parameters
    double center_heading_;
    double amplitude_;
    int cycles_;
    double tolerance_;
    double dwell_time_;
    double timeout_;

    // Runtime state
    std::vector<double> targets_;
    size_t current_target_idx_;
    int completed_cycles_;
    rclcpp::Time start_time_;
    rclcpp::Time dwell_start_time_;
    bool in_dwell_;
};
