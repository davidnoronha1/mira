#include "behaviours.hpp"

void DetectTargetPoint::point_callback(const geometry_msgs::msg::Point::SharedPtr msg)
{
    (void)msg;  // Unused - we only care that a point was published
    rclcpp::Time now = ros_state_->node->now();
    last_detection_time_ = now;
    
    // If we require a specific color, check if it matches
    if (!required_color_.empty() && !color_matches_) {
        return;
    }
    
    // Valid detection received
    detection_times_.push_back(now);
    
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
            target_detected_ = true;
            RCLCPP_DEBUG(ros_state_->node->get_logger(), 
                        "DetectTargetPoint: consistent detection for %.2f s (color: %s)",
                        span, current_color_.c_str());
        }
    }
}

void DetectTargetPoint::color_callback(const std_msgs::msg::String::SharedPtr msg)
{
    current_color_ = msg->data;
    
    if (required_color_.empty()) {
        color_matches_ = true;  // No color requirement
    } else {
        color_matches_ = (current_color_ == required_color_);
        if (color_matches_) {
            RCLCPP_DEBUG(ros_state_->node->get_logger(),
                        "DetectTargetPoint: color matches '%s'", required_color_.c_str());
        }
    }
}

DetectTargetPoint::DetectTargetPoint(const std::string& name, 
                                     const BT::NodeConfiguration& config, 
                                     ROSState* ros_state)
    : BT::StatefulActionNode(name, config), 
      ros_state_(ros_state),
      detection_duration_(1.0),
      timeout_(30.0),
      target_detected_(false),
      color_matches_(false),
      current_color_()
{
}

BT::PortsList DetectTargetPoint::providedPorts()
{
    return {
        BT::InputPort<std::string>("point_topic", "Topic publishing target point (geometry_msgs/Point)"),
        BT::InputPort<std::string>("color_topic", "", "Optional topic publishing color (std_msgs/String)"),
        BT::InputPort<std::string>("required_color", "", "Optional required color (e.g., 'blue', 'orange')"),
        BT::InputPort<double>("detection_duration", 1.0, "Required consistent detection duration (seconds)"),
        BT::InputPort<double>("timeout", 30.0, "Maximum time to wait for detection (seconds)")
    };
}

BT::NodeStatus DetectTargetPoint::onStart()
{
    // Read input parameters
    auto point_topic = getInput<std::string>("point_topic");
    if (!point_topic) {
        throw BT::RuntimeError("Missing required input [point_topic]: ", point_topic.error());
    }
    point_topic_ = point_topic.value();
    
    color_topic_ = getInput<std::string>("color_topic").value();
    required_color_ = getInput<std::string>("required_color").value();
    detection_duration_ = getInput<double>("detection_duration").value();
    timeout_ = getInput<double>("timeout").value();
    
    // Create subscriptions
    point_sub_ = ros_state_->node->create_subscription<geometry_msgs::msg::Point>(
        point_topic_, 10,
        std::bind(&DetectTargetPoint::point_callback, this, std::placeholders::_1));
    
    if (!color_topic_.empty()) {
        color_sub_ = ros_state_->node->create_subscription<std_msgs::msg::String>(
            color_topic_, 10,
            std::bind(&DetectTargetPoint::color_callback, this, std::placeholders::_1));
    } else {
        color_matches_ = true;  // No color filtering
    }
    
    // Reset state
    detection_times_.clear();
    target_detected_ = false;
    start_time_ = ros_state_->node->now();
    last_detection_time_ = start_time_;
    current_color_ = "";
    
    RCLCPP_INFO(ros_state_->node->get_logger(), 
               "DetectTargetPoint: looking for target on '%s' (color: %s, duration: %.1f s)",
               point_topic_.c_str(), 
               required_color_.empty() ? "any" : required_color_.c_str(),
               detection_duration_);
    
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DetectTargetPoint::onRunning()
{
    // Check for timeout
    double elapsed = (ros_state_->node->now() - start_time_).seconds();
    if (elapsed > timeout_) {
        RCLCPP_WARN(ros_state_->node->get_logger(), 
                   "DetectTargetPoint: timeout after %.1f s",
                   elapsed);
        return BT::NodeStatus::FAILURE;
    }
    
    // Check if target has been consistently detected
    if (target_detected_) {
        RCLCPP_INFO(ros_state_->node->get_logger(),
                   "DetectTargetPoint: target detected successfully (color: %s)",
                   current_color_.c_str());
        return BT::NodeStatus::SUCCESS;
    }
    
    return BT::NodeStatus::RUNNING;
}

void DetectTargetPoint::onHalted()
{
    RCLCPP_INFO(ros_state_->node->get_logger(), "DetectTargetPoint: halted");
}
