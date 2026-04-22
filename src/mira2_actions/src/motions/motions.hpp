#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <control_utils/control_utils.hpp>
#include <custom_msgs/msg/commands.hpp>
#include <custom_msgs/msg/telemetry.hpp>
#include <vision_msgs/msg/bounding_box2_d_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <string>
#include <vector>
#include <cmath>
#include "../common.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

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
    void bbox_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg);
    void publish_neutral();

    ROSState* ros_state_;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr bb_sub_;

    // PID Controllers
    PID_Controller lateral_pid;
    PID_Controller depth_pid;
    PID_Controller yaw_pid;

    // Parameters
    std::string target_object_;
    double forward_pwm_;
    double frame_width_;
    double frame_height_;
    double success_bb_area_;
    double bb_lost_timeout_;
    double timeout_;
    std::string flight_mode_;

    // Runtime state
    rclcpp::Time start_time_;
    rclcpp::Time last_detection_time_;
    
    // Latest bounding box info (freshness determined via last_detection_time_)
    double bb_x_center_norm_;
    double bb_y_center_norm_;
    double bb_area_norm_;

    // Depth control (visual y-servo, no pressure sensor)
    double depth_setpoint_base_;   // target y-row for dock center, e.g. 0.65
    double depth_visual_gain_;     // area-based setpoint shift per unit bb_area

    // Success conditions
    double success_y_threshold_;   // exit when dock y_center exceeds this (dock nearly below)

    // Yaw lock
    double locked_heading_;

    // Raw terminal status line throttle
    rclcpp::Time last_print_time_;
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

/**
 * @brief AlignXY: Align to target point using lateral and forward PD control
 * 
 * Subscribes to a Point topic and uses PD control to center the target.
 * Returns SUCCESS when target is centered within tolerance.
 */
class AlignXY : public BT::StatefulActionNode
{
public:
    AlignXY(const std::string& name,
            const BT::NodeConfiguration& config,
            ROSState* ros_state);

    static BT::PortsList providedPorts();
    
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    void point_callback(const geometry_msgs::msg::Point::SharedPtr msg);
    void publish_neutral();

    ROSState* ros_state_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr point_sub_;

    // PD Controllers
    PID_Controller lateral_pd;
    PID_Controller forward_pd;
    PID_Controller yaw_pd;

    // Parameters
    std::string point_topic_;
    double tolerance_x_;
    double tolerance_y_;
    double target_lost_timeout_;
    double timeout_;
    std::string flight_mode_;

    // Target tracking
    double target_nx_;
    double target_ny_;
    double prev_nx_;
    double prev_ny_;
    bool target_visible_;
    rclcpp::Time last_detection_time_;
    rclcpp::Time start_time_;
    rclcpp::Time last_update_time_;
    
    double locked_heading_;
    bool heading_locked_;
};

/**
 * @brief ApproachWithDepth: Approach target while controlling depth
 * 
 * Maintains XY alignment while diving based on depth proxy value.
 * Returns SUCCESS when depth target is reached.
 */
class ApproachWithDepth : public BT::StatefulActionNode
{
public:
    ApproachWithDepth(const std::string& name,
                     const BT::NodeConfiguration& config,
                     ROSState* ros_state);

    static BT::PortsList providedPorts();
    
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    void point_callback(const geometry_msgs::msg::Point::SharedPtr msg);
    void publish_neutral();

    ROSState* ros_state_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr point_sub_;

    // PD Controllers
    PID_Controller lateral_pd;
    PID_Controller forward_pd;
    PID_Controller yaw_pd;

    // Parameters
    std::string point_topic_;
    double success_depth_;
    double blind_lock_depth_;
    double thrust_pwm_shallow_;
    double thrust_pwm_deep_;
    double depth_threshold_;
    double target_lost_timeout_;
    double timeout_;
    std::string flight_mode_;

    // Target tracking
    double target_nx_;
    double target_ny_;
    double target_depth_;
    double prev_nx_;
    double prev_ny_;
    bool target_visible_;
    rclcpp::Time last_detection_time_;
    rclcpp::Time start_time_;
    rclcpp::Time last_update_time_;
    
    double locked_heading_;
    bool heading_locked_;
};

/**
 * @brief HoldPosition: Hold current position for a duration
 * 
 * Publishes neutral commands to maintain position.
 * Returns SUCCESS after duration has elapsed.
 */
class HoldPosition : public BT::StatefulActionNode
{
public:
    HoldPosition(const std::string& name,
                const BT::NodeConfiguration& config,
                ROSState* ros_state);

    static BT::PortsList providedPorts();
    
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    void publish_neutral();

    ROSState* ros_state_;
    
    double duration_;
    std::string flight_mode_;
    
    rclcpp::Time start_time_;
};

/**
 * @brief LateralEvasion: Perform oscillating lateral movement
 * 
 * Strafes left and right to evade obstacles.
 * Runs indefinitely or for specified cycles/timeout.
 */
class LateralEvasion : public BT::StatefulActionNode
{
public:
    LateralEvasion(const std::string& name,
                  const BT::NodeConfiguration& config,
                  ROSState* ros_state);

    static BT::PortsList providedPorts();
    
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    void publish_neutral();

    ROSState* ros_state_;
    
    double amplitude_;
    double period_;
    int cycles_;
    double timeout_;
    std::string flight_mode_;
    bool oscillate_;
    
    rclcpp::Time start_time_;
    rclcpp::Time cycle_start_time_;
    int current_direction_;
    int completed_cycles_;
};
