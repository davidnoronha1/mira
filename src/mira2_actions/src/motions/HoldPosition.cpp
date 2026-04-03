#include "motions.hpp"

HoldPosition::HoldPosition(const std::string& name,
                          const BT::NodeConfiguration& config,
                          ROSState* ros_state)
    : BT::StatefulActionNode(name, config),
      ros_state_(ros_state),
      duration_(3.0),
      flight_mode_("ALT_HOLD")
{
}

BT::PortsList HoldPosition::providedPorts()
{
    return {
        BT::InputPort<double>("duration", 3.0, "Duration to hold position (seconds)"),
        BT::InputPort<std::string>("flight_mode", "ALT_HOLD", "Flight controller mode"),
    };
}

BT::NodeStatus HoldPosition::onStart()
{
    duration_ = getInput<double>("duration").value();
    flight_mode_ = getInput<std::string>("flight_mode").value();
    
    start_time_ = ros_state_->node->now();
    
    RCLCPP_INFO(ros_state_->node->get_logger(),
               "HoldPosition: holding for %.1f seconds", duration_);
    
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus HoldPosition::onRunning()
{
    double elapsed = (ros_state_->node->now() - start_time_).seconds();
    
    // Publish neutral commands to maintain position
    publish_neutral();
    
    if (elapsed >= duration_) {
        RCLCPP_INFO(ros_state_->node->get_logger(),
                   "HoldPosition: completed (%.1f s)", elapsed);
        return BT::NodeStatus::SUCCESS;
    }
    
    RCLCPP_DEBUG(ros_state_->node->get_logger(),
                "HoldPosition: holding... (%.1f / %.1f s)", elapsed, duration_);
    
    return BT::NodeStatus::RUNNING;
}

void HoldPosition::onHalted()
{
    RCLCPP_INFO(ros_state_->node->get_logger(), "HoldPosition: halted");
    publish_neutral();
}

void HoldPosition::publish_neutral()
{
    custom_msgs::msg::Commands cmd;
    cmd.mode = flight_mode_;
    cmd.arm = true;
    cmd.forward = 1500;
    cmd.lateral = 1500;
    cmd.thrust = 1500;
    cmd.yaw = 1500;
    cmd.pitch = 1500;
    cmd.roll = 1500;
    ros_state_->cmd_publisher->publish(cmd);
}
