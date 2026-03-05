#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include "../common.hpp"
#include <vision_msgs/msg/bounding_box2_d_array.hpp>
#include <string>
#include <deque>

// DetectObjectBoundingbox: Checks for consistent detection of a specified object
// Listens to /vision/detections/boundingbox and looks for objects with a matching frame_id
// Returns SUCCESS only after the object has been consistently detected for a specified duration
class DetectObjectBoundingbox : public BT::SyncActionNode
{
    ROSState* ros_state_;
    rclcpp::Subscription<vision_msgs::msg::BoundingBox2DArray>::SharedPtr bbox_sub_;
    
    std::string target_object_;
    double detection_duration_ = 2.0;  // seconds of consistent detection required
    double timeout_ = 30.0;  // maximum time to wait for detection
    
    // Detection tracking
    std::deque<rclcpp::Time> detection_times_;
    rclcpp::Time start_time_;
    rclcpp::Time last_detection_time_;
    bool initialized_ = false;
    bool object_detected_ = false;
    
    void bbox_callback(const vision_msgs::msg::BoundingBox2DArray::SharedPtr msg)
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

public:
    DetectObjectBoundingbox(const std::string& name, const BT::NodeConfiguration& config, ROSState* ros_state)
        : BT::SyncActionNode(name, config), ros_state_(ros_state)
    {
        // Create subscription to bounding box topic
        bbox_sub_ = ros_state_->node->create_subscription<vision_msgs::msg::BoundingBox2DArray>(
            "/vision/bounding_box", 10,
            std::bind(&DetectObjectBoundingbox::bbox_callback, this, std::placeholders::_1));
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("object", "Target object name/frame_id to detect"),
            BT::InputPort<double>("detection_duration", 2.0, "Required consistent detection duration (seconds)"),
            BT::InputPort<double>("timeout", 30.0, "Maximum time to wait for detection (seconds)")
        };
    }

    BT::NodeStatus tick() override
    {
        if (!initialized_) {
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
            initialized_ = true;
            
            RCLCPP_INFO(ros_state_->node->get_logger(), 
                       "DetectObjectBoundingbox: looking for '%s' (need %.1f s consistent detection)",
                       target_object_.c_str(), detection_duration_);
        }
        
        // Check for timeout
        double elapsed = (ros_state_->node->now() - start_time_).seconds();
        if (elapsed > timeout_) {
            RCLCPP_WARN(ros_state_->node->get_logger(), 
                       "DetectObjectBoundingbox: timeout after %.1f s without detecting '%s'",
                       elapsed, target_object_.c_str());
            initialized_ = false;
            return BT::NodeStatus::FAILURE;
        }
        
        // Check if we have a consistent detection
        if (object_detected_) {
            RCLCPP_INFO(ros_state_->node->get_logger(), 
                       "DetectObjectBoundingbox: successfully detected '%s' consistently for %.1f s",
                       target_object_.c_str(), detection_duration_);
            initialized_ = false;
            return BT::NodeStatus::SUCCESS;
        }
        
        // Still waiting for consistent detection
        double time_since_last = (ros_state_->node->now() - last_detection_time_).seconds();
        RCLCPP_DEBUG(ros_state_->node->get_logger(), 
                    "DetectObjectBoundingbox: waiting for '%s' (detections in window: %zu, time since last: %.2f s)",
                    target_object_.c_str(), detection_times_.size(), time_since_last);
        
        return BT::NodeStatus::RUNNING;
    }
    
    // void halt() override
    // {
    //     initialized_ = false;
    //     detection_times_.clear();
    //     object_detected_ = false;
    //     RCLCPP_INFO(ros_state_->node->get_logger(), "DetectObjectBoundingbox: halted");
    // }
};
