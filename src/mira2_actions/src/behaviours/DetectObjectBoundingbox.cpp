#include "behaviours.hpp"

void DetectObjectBoundingbox::bbox_callback(const vision_msgs::msg::BoundingBox2DArray::SharedPtr msg)
{
    // Check if any bounding box has the target frame_id
    bool found_in_this_frame = false;
    for (const auto& bbox : msg->boxes) {
        // Check if the header frame_id matches our target object
        if (msg->header.frame_id == target_object_) {
            found_in_this_frame = true;
            break;
        }
    }
    
    if (found_in_this_frame) {
        rclcpp::Time now = ros_state_->node->now();
        detection_times_.push_back(now);
        last_detection_time_ = now;
        
        // Remove detections older than detection_duration_
        while (!detection_times_.empty()) {
            double age = (now - detection_times_.front()).seconds();
            if (age > detection_duration_) {
                detection_times_.pop_front();
            } else {
                break;
            }
        }
        
        // Check if we have consistent detections spanning the required duration
        if (!detection_times_.empty()) {
            double span = (detection_times_.back() - detection_times_.front()).seconds();
            if (span >= detection_duration_ && detection_times_.size() > 1) {
                object_detected_ = true;
                RCLCPP_DEBUG(ros_state_->node->get_logger(), 
                            "DetectObjectBoundingbox: consistent detection of '%s' for %.2f s",
                            target_object_.c_str(), span);
            }
        }
    }
}

DetectObjectBoundingbox::DetectObjectBoundingbox(const std::string& name, const BT::NodeConfiguration& config, ROSState* ros_state)
    : BT::StatefulActionNode(name, config), 
      ros_state_(ros_state),
      detection_duration_(2.0),
      timeout_(30.0),
      object_detected_(false)
{
    // Create subscription to bounding box topic
    bbox_sub_ = ros_state_->node->create_subscription<vision_msgs::msg::BoundingBox2DArray>(
        "/vision/bounding_box", 10,
        std::bind(&DetectObjectBoundingbox::bbox_callback, this, std::placeholders::_1));
}

BT::PortsList DetectObjectBoundingbox::providedPorts()
{
    return {
        BT::InputPort<std::string>("object", "Target object name/frame_id to detect"),
        BT::InputPort<double>("detection_duration", 2.0, "Required consistent detection duration (seconds)"),
        BT::InputPort<double>("timeout", 30.0, "Maximum time to wait for detection (seconds)")
    };
}

BT::NodeStatus DetectObjectBoundingbox::onStart()
{
    // Read input parameters
    auto object = getInput<std::string>("object");
    if (!object) {
        throw BT::RuntimeError("Missing required input [object]: ", object.error());
    }
    target_object_ = object.value();
    
    detection_duration_ = getInput<double>("detection_duration").value();
    timeout_ = getInput<double>("timeout").value();
    
    // Reset state
    detection_times_.clear();
    object_detected_ = false;
    start_time_ = ros_state_->node->now();
    last_detection_time_ = start_time_;
    
    RCLCPP_INFO(ros_state_->node->get_logger(), 
               "DetectObjectBoundingbox: looking for '%s' (need %.1f s consistent detection)",
               target_object_.c_str(), detection_duration_);
    
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DetectObjectBoundingbox::onRunning()
{
    // Check for timeout
    double elapsed = (ros_state_->node->now() - start_time_).seconds();
    if (elapsed > timeout_) {
        RCLCPP_WARN(ros_state_->node->get_logger(), 
                   "DetectObjectBoundingbox: timeout after %.1f s without detecting '%s'",
                   elapsed, target_object_.c_str());
        return BT::NodeStatus::FAILURE;
    }
        
    // Check if we have a consistent detection
    if (object_detected_) {
        RCLCPP_INFO(ros_state_->node->get_logger(), 
                   "DetectObjectBoundingbox: successfully detected '%s' consistently for %.1f s",
                   target_object_.c_str(), detection_duration_);
        return BT::NodeStatus::SUCCESS;
    }
        
    // Still waiting for consistent detection
    double time_since_last = (ros_state_->node->now() - last_detection_time_).seconds();
    RCLCPP_DEBUG(ros_state_->node->get_logger(), 
                "DetectObjectBoundingbox: waiting for '%s' (detections in window: %zu, time since last: %.2f s)",
                target_object_.c_str(), detection_times_.size(), time_since_last);
    
    return BT::NodeStatus::RUNNING;
}

void DetectObjectBoundingbox::onHalted()
{
    RCLCPP_INFO(ros_state_->node->get_logger(), "DetectObjectBoundingbox: halted");
    detection_times_.clear();
    object_detected_ = false;
}
