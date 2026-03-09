#include "motions.hpp"

YawSweep::YawSweep(const std::string& name, const BT::NodeConfiguration& config, ROSState* ros_state)
    : BT::StatefulActionNode(name, config), 
      yaw_pid("yaw", ros_state->node), 
      ros_state_(ros_state),
      center_heading_(0.0),
      amplitude_(30.0),
      cycles_(1),
      tolerance_(5.0),
      dwell_time_(1.0),
      timeout_(60.0),
      current_target_idx_(0),
      completed_cycles_(0),
      in_dwell_(false)
{}

BT::PortsList YawSweep::providedPorts()
    {
        return {
            BT::InputPort<double>("center_heading", "Center heading for sweep (degrees)"),
            BT::InputPort<double>("amplitude", 30.0, "Amplitude from center to extreme (deg)"),
            BT::InputPort<int>("cycles", 1, "Number of back-and-forth cycles to perform"),
            BT::InputPort<double>("dwell_time", 1.0, "Seconds to hold at each extreme"),
            BT::InputPort<double>("tolerance", 5.0, "Heading tolerance in degrees"),
            BT::InputPort<double>("timeout", 60.0, "Maximum time to attempt sweep (seconds)"),
            BT::InputPort<double>("yaw_pid_kp", 3.18, "Yaw PID proportional gain"),
            BT::InputPort<double>("yaw_pid_ki", 0.01, "Yaw PID integral gain"),
            BT::InputPort<double>("yaw_pid_kd", 7.2, "Yaw PID derivative gain"),
            BT::InputPort<double>("yaw_pid_base_offset", 1500.0, "Yaw PID base offset")
        };
    }

BT::NodeStatus YawSweep::onStart()
{
    // read inputs
    center_heading_ = getInput<double>("center_heading").value();
    amplitude_ = getInput<double>("amplitude").value();
    cycles_ = getInput<int>("cycles").value();
    dwell_time_ = getInput<double>("dwell_time").value();
    tolerance_ = getInput<double>("tolerance").value();
    timeout_ = getInput<double>("timeout").value();

    double kp = getInput<double>("yaw_pid_kp").value();
    double ki = getInput<double>("yaw_pid_ki").value();
    double kd = getInput<double>("yaw_pid_kd").value();
    double base_offset = getInput<double>("yaw_pid_base_offset").value();

    yaw_pid.kp = kp;
    yaw_pid.ki = ki;
    yaw_pid.kd = kd;
    yaw_pid.base_offset = base_offset;
    yaw_pid.emptyError();

    // build target sequence: center-amplitude -> center+amplitude -> repeat
    targets_.clear();
    for (int c = 0; c < cycles_; ++c) {
        targets_.push_back(std::fmod(center_heading_ - amplitude_ + 360.0, 360.0));
        targets_.push_back(std::fmod(center_heading_ + amplitude_ + 360.0, 360.0));
    }
    current_target_idx_ = 0;
    completed_cycles_ = 0;
    in_dwell_ = false;
    start_time_ = ros_state_->node->now();

    RCLCPP_INFO(ros_state_->node->get_logger(), "YawSweep: starting sweep center=%.1f amp=%.1f cycles=%d",
                center_heading_, amplitude_, cycles_);
    
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus YawSweep::onRunning()
{
    double elapsed = (ros_state_->node->now() - start_time_).seconds();
    if (elapsed > timeout_) {
        RCLCPP_WARN(ros_state_->node->get_logger(), "YawSweep: timeout after %.1f s", elapsed);
        cleanup_and_publish_neutral();
        return BT::NodeStatus::FAILURE;
    }

    if (current_target_idx_ >= targets_.size()) {
        // Completed all targets
        cleanup_and_publish_neutral();
        RCLCPP_INFO(ros_state_->node->get_logger(), "YawSweep: completed %d cycles", cycles_);
        return BT::NodeStatus::SUCCESS;
    }

    double current_heading = ros_state_->telemetry.heading;
    double target = targets_[current_target_idx_];
    // shortest angular difference
    double yaw_error = std::fmod((target - current_heading + 540.0), 360.0) - 180.0; // (+540 to keep positive before fmod)

    if (std::abs(yaw_error) <= tolerance_) {
        if (!in_dwell_) {
            // start dwell
            dwell_start_time_ = ros_state_->node->now();
            in_dwell_ = true;
            RCLCPP_DEBUG(ros_state_->node->get_logger(), "YawSweep: reached target %.1f, dwelling", target);
        } else {
            double dwell_elapsed = (ros_state_->node->now() - dwell_start_time_).seconds();
            if (dwell_elapsed >= dwell_time_) {
                // move to next target
                current_target_idx_++;
                in_dwell_ = false;
            }
        }
        // while dwelling, publish neutral to hold
        publish_neutral();
        return BT::NodeStatus::RUNNING;
    }

    // not at target, compute PID yaw output and publish
    float pid_yaw = yaw_pid.pid_control(static_cast<float>(yaw_error), elapsed, false);

    custom_msgs::msg::Commands cmd;
    cmd.mode = "STABILIZE";
    cmd.arm = true;
    cmd.forward = 1500;
    cmd.lateral = 1500;
    cmd.thrust = 1500;
    cmd.yaw = static_cast<int>(pid_yaw);
    ros_state_->cmd_publisher->publish(cmd);

    RCLCPP_DEBUG(ros_state_->node->get_logger(), "YawSweep: current=%.1f target=%.1f error=%.2f yaw_pwm=%d",
                 current_heading, target, yaw_error, cmd.yaw);

    return BT::NodeStatus::RUNNING;
}

void YawSweep::onHalted()
{
    RCLCPP_INFO(ros_state_->node->get_logger(), "YawSweep: halted");
    cleanup_and_publish_neutral();
}

void YawSweep::cleanup_and_publish_neutral()
{
    custom_msgs::msg::Commands cmd;
    cmd.mode = "STABILIZE";
    cmd.arm = true;
    cmd.forward = 1500;
    cmd.lateral = 1500;
    cmd.thrust = 1500;
    cmd.yaw = 1500;
    ros_state_->cmd_publisher->publish(cmd);
}

void YawSweep::publish_neutral()
{
    custom_msgs::msg::Commands cmd;
    cmd.mode = "STABILIZE";
    cmd.arm = true;
    cmd.forward = 1500;
    cmd.lateral = 1500;
    cmd.thrust = 1500;
    cmd.yaw = 1500;
    ros_state_->cmd_publisher->publish(cmd);
}

