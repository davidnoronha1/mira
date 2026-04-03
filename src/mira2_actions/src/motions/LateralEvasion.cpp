#include "motions.hpp"

LateralEvasion::LateralEvasion(const std::string& name,
                              const BT::NodeConfiguration& config,
                              ROSState* ros_state)
    : BT::StatefulActionNode(name, config),
      ros_state_(ros_state),
      amplitude_(50.0),
      period_(3.0),
      cycles_(-1),
      timeout_(0.0),
      flight_mode_("ALT_HOLD"),
      oscillate_(true),
      current_direction_(1),
      completed_cycles_(0)
{
}

BT::PortsList LateralEvasion::providedPorts()
{
    return {
        BT::InputPort<double>("amplitude", 50.0, "Lateral PWM offset from neutral"),
        BT::InputPort<double>("period", 3.0, "Time per direction (seconds, ignored if oscillate=false)"),
        BT::InputPort<int>("cycles", -1, "Number of complete cycles (-1 for infinite)"),
        BT::InputPort<double>("timeout", 0.0, "Maximum time (0 for no timeout)"),
        BT::InputPort<bool>("oscillate", true, "If true, oscillates left/right; if false, goes one direction"),
        BT::InputPort<int>("direction", 1, "Initial direction: 1=right, -1=left"),
        BT::InputPort<std::string>("flight_mode", "ALT_HOLD", "Flight controller mode"),
    };
}

BT::NodeStatus LateralEvasion::onStart()
{
    amplitude_ = getInput<double>("amplitude").value();
    period_ = getInput<double>("period").value();
    cycles_ = getInput<int>("cycles").value();
    timeout_ = getInput<double>("timeout").value();
    oscillate_ = getInput<bool>("oscillate").value();
    current_direction_ = getInput<int>("direction").value();
    flight_mode_ = getInput<std::string>("flight_mode").value();
    
    start_time_ = ros_state_->node->now();
    cycle_start_time_ = start_time_;
    completed_cycles_ = 0;
    
    RCLCPP_INFO(ros_state_->node->get_logger(),
               "LateralEvasion: starting %s (amplitude: %.0f, direction: %s)",
               oscillate_ ? "oscillating sweep" : "one-way sweep",
               amplitude_,
               current_direction_ > 0 ? "RIGHT" : "LEFT");
    
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus LateralEvasion::onRunning()
{
    rclcpp::Time now = ros_state_->node->now();
    double elapsed_total = (now - start_time_).seconds();
    double elapsed_cycle = (now - cycle_start_time_).seconds();
    
    // Check timeout
    if (timeout_ > 0.0 && elapsed_total > timeout_) {
        RCLCPP_INFO(ros_state_->node->get_logger(),
                   "LateralEvasion: timeout reached");
        publish_neutral();
        return BT::NodeStatus::SUCCESS;
    }
    
    // Check if we should switch direction (only if oscillating)
    if (oscillate_ && elapsed_cycle > period_) {
        current_direction_ *= -1;
        cycle_start_time_ = now;
        
        // Count completed cycles (one cycle = left + right)
        if (current_direction_ == 1) {
            completed_cycles_++;
            RCLCPP_DEBUG(ros_state_->node->get_logger(),
                        "LateralEvasion: completed cycle %d", completed_cycles_);
            
            // Check if we've completed requested cycles
            if (cycles_ > 0 && completed_cycles_ >= cycles_) {
                RCLCPP_INFO(ros_state_->node->get_logger(),
                           "LateralEvasion: completed %d cycles", completed_cycles_);
                publish_neutral();
                return BT::NodeStatus::SUCCESS;
            }
        }
    }
    
    // Publish lateral movement command
    custom_msgs::msg::Commands cmd;
    cmd.mode = flight_mode_;
    cmd.arm = true;
    cmd.forward = 1500;
    cmd.lateral = 1500 + static_cast<int>(current_direction_ * amplitude_);
    cmd.thrust = 1500;
    cmd.yaw = 1500;
    cmd.pitch = 1500;
    cmd.roll = 1500;
    ros_state_->cmd_publisher->publish(cmd);
    
    RCLCPP_DEBUG(ros_state_->node->get_logger(),
                "LateralEvasion: sweeping (dir=%d, lat=%d)",
                current_direction_, cmd.lateral);
    
    return BT::NodeStatus::RUNNING;
}

void LateralEvasion::onHalted()
{
    RCLCPP_INFO(ros_state_->node->get_logger(), "LateralEvasion: halted");
    publish_neutral();
}

void LateralEvasion::publish_neutral()
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
