#pragma once

#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/bounding_box2_d_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <deque>
#include "../common.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

/**
 * @brief DetectObjectBoundingbox: Checks for consistent detection of a specified object
 * 
 * Listens to /vision/detections/boundingbox and looks for objects with a matching frame_id.
 * Returns SUCCESS only after the object has been consistently detected for a specified duration.
 */
class DetectObjectBoundingbox : public BT::StatefulActionNode
{
public:
    DetectObjectBoundingbox(const std::string& name, 
                           const BT::NodeConfiguration& config, 
                           ROSState* ros_state);

    static BT::PortsList providedPorts();
    
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    void bbox_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg);

    ROSState* ros_state_;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr bbox_sub_;
    
    std::string target_object_;
    double detection_duration_;
    double timeout_;
    
    // Detection tracking
    std::deque<rclcpp::Time> detection_times_;
    rclcpp::Time start_time_;
    rclcpp::Time last_detection_time_;
    bool object_detected_;
};

/**
 * @brief DetectTargetPoint: Checks for consistent target point detection with color filtering
 * 
 * Listens to a Point topic (e.g., bucket_target_2d) and optionally filters by color.
 * Returns SUCCESS only after the target has been consistently detected for a specified duration.
 */
class DetectTargetPoint : public BT::StatefulActionNode
{
public:
    DetectTargetPoint(const std::string& name, 
                     const BT::NodeConfiguration& config, 
                     ROSState* ros_state);

    static BT::PortsList providedPorts();
    
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    void point_callback(const geometry_msgs::msg::Point::SharedPtr msg);
    void color_callback(const std_msgs::msg::String::SharedPtr msg);

    ROSState* ros_state_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr point_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr color_sub_;
    
    std::string point_topic_;
    std::string color_topic_;
    std::string required_color_;
    double detection_duration_;
    double timeout_;
    
    // Detection tracking
    std::deque<rclcpp::Time> detection_times_;
    rclcpp::Time start_time_;
    rclcpp::Time last_detection_time_;
    bool target_detected_;
    bool color_matches_;
    std::string current_color_;
};
