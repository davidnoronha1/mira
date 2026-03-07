#include "motions.hpp"

void AlignXY::point_callback(const geometry_msgs::msg::Point::SharedPtr msg)
{
    target_nx_ = msg->x;
    target_ny_ = msg->y;
    target_visible_ = true;
    last_detection_time_ = ros_state_->node->now();
}

AlignXY::AlignXY(const std::string& name,
                 const BT::NodeConfiguration& config,
                 ROSState* ros_state)
    : BT::StatefulActionNode(name, config),
      ros_state_(ros_state),
      lateral_pd("lateral", ros_state->node),
      forward_pd("forward", ros_state->node),
      yaw_pd("yaw", ros_state->node),
      tolerance_x_(0.15),
      tolerance_y_(0.15),
      target_lost_timeout_(1.0),
      timeout_(20.0),
      flight_mode_("ALT_HOLD"),
      target_nx_(0.0),
      target_ny_(0.0),
      prev_nx_(0.0),
      prev_ny_(0.0),
      target_visible_(false),
      locked_heading_(0.0),
      heading_locked_(false)
{
}

BT::PortsList AlignXY::providedPorts()
{
    return {
        BT::InputPort<std::string>("point_topic", "Topic publishing target point (geometry_msgs/Point)"),
        BT::InputPort<double>("tolerance_x", 0.15, "X centering tolerance (normalized)"),
        BT::InputPort<double>("tolerance_y", 0.15, "Y centering tolerance (normalized)"),
        BT::InputPort<double>("target_lost_timeout", 1.0, "Seconds before FAILURE if target lost"),
        BT::InputPort<double>("timeout", 20.0, "Maximum time for alignment (seconds)"),
        BT::InputPort<std::string>("flight_mode", "ALT_HOLD", "Flight controller mode"),
        
        // Lateral PD
        BT::InputPort<double>("lateral_pd_kp", 300.0, "Lateral PD proportional gain"),
        BT::InputPort<double>("lateral_pd_kd", 100.0, "Lateral PD derivative gain"),
        BT::InputPort<double>("lateral_pd_base_offset", 1500.0, "Lateral PD base offset (neutral PWM)"),
        
        // Forward PD
        BT::InputPort<double>("forward_pd_kp", 300.0, "Forward PD proportional gain"),
        BT::InputPort<double>("forward_pd_kd", 100.0, "Forward PD derivative gain"),
        BT::InputPort<double>("forward_pd_base_offset", 1500.0, "Forward PD base offset (neutral PWM)"),
        
        // Yaw PD
        BT::InputPort<double>("yaw_pd_kp", 5.0, "Yaw PD proportional gain"),
        BT::InputPort<double>("yaw_pd_base_offset", 1500.0, "Yaw PD base offset (neutral PWM)"),
    };
}

BT::NodeStatus AlignXY::onStart()
{
    // Read input parameters
    auto point_topic = getInput<std::string>("point_topic");
    if (!point_topic) {
        throw BT::RuntimeError("Missing required input [point_topic]: ", point_topic.error());
    }
    point_topic_ = point_topic.value();
    
    tolerance_x_ = getInput<double>("tolerance_x").value();
    tolerance_y_ = getInput<double>("tolerance_y").value();
    target_lost_timeout_ = getInput<double>("target_lost_timeout").value();
    timeout_ = getInput<double>("timeout").value();
    flight_mode_ = getInput<std::string>("flight_mode").value();
    
    // Configure PD controllers
    lateral_pd.kp = getInput<double>("lateral_pd_kp").value();
    lateral_pd.kd = getInput<double>("lateral_pd_kd").value();
    lateral_pd.ki = 0.0;
    lateral_pd.base_offset = getInput<double>("lateral_pd_base_offset").value();
    lateral_pd.emptyError();
    
    forward_pd.kp = getInput<double>("forward_pd_kp").value();
    forward_pd.kd = getInput<double>("forward_pd_kd").value();
    forward_pd.ki = 0.0;
    forward_pd.base_offset = getInput<double>("forward_pd_base_offset").value();
    forward_pd.emptyError();
    
    yaw_pd.kp = getInput<double>("yaw_pd_kp").value();
    yaw_pd.ki = 0.0;
    yaw_pd.kd = 0.0;
    yaw_pd.base_offset = getInput<double>("yaw_pd_base_offset").value();
    yaw_pd.emptyError();
    
    // Create subscription
    point_sub_ = ros_state_->node->create_subscription<geometry_msgs::msg::Point>(
        point_topic_, 10,
        std::bind(&AlignXY::point_callback, this, std::placeholders::_1));
    
    // Reset state
    target_visible_ = false;
    start_time_ = ros_state_->node->now();
    last_detection_time_ = start_time_;
    last_update_time_ = start_time_;
    prev_nx_ = 0.0;
    prev_ny_ = 0.0;
    
    // Lock heading at current heading
    locked_heading_ = ros_state_->telemetry.heading;
    heading_locked_ = true;
    
    RCLCPP_INFO(ros_state_->node->get_logger(),
               "AlignXY: starting alignment (tolerance: ±%.2f, ±%.2f)",
               tolerance_x_, tolerance_y_);
    
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus AlignXY::onRunning()
{
    rclcpp::Time now = ros_state_->node->now();
    double elapsed = (now - start_time_).seconds();
    last_update_time_ = now;
    
    // Check for hard timeout
    if (elapsed > timeout_) {
        RCLCPP_WARN(ros_state_->node->get_logger(),
                   "AlignXY: timeout after %.1f s", elapsed);
        publish_neutral();
        return BT::NodeStatus::FAILURE;
    }
    
    // Check if target lost
    double time_since_detection = (now - last_detection_time_).seconds();
    if (elapsed > 0.5 && time_since_detection > target_lost_timeout_) {
        RCLCPP_WARN(ros_state_->node->get_logger(),
                   "AlignXY: target lost for %.1f s → FAILURE", time_since_detection);
        publish_neutral();
        return BT::NodeStatus::FAILURE;
    }
    
    // If target not visible, hold position
    if (!target_visible_) {
        RCLCPP_DEBUG(ros_state_->node->get_logger(),
                    "AlignXY: waiting for target...");
        publish_neutral();
        return BT::NodeStatus::RUNNING;
    }
    
    // Check success condition
    if (std::abs(target_nx_) < tolerance_x_ && std::abs(target_ny_) < tolerance_y_) {
        RCLCPP_INFO(ros_state_->node->get_logger(),
                   "AlignXY: target centered (nx=%.3f, ny=%.3f)", target_nx_, target_ny_);
        publish_neutral();
        return BT::NodeStatus::SUCCESS;
    }
    
    // Calculate PD control outputs
    float lateral_pwm = lateral_pd.pid_control(target_nx_, elapsed, false);
    float forward_pwm = forward_pd.pid_control(-target_ny_, elapsed, false);  // Negative because positive ny means object is down
    
    // Calculate heading lock
    double heading_error = locked_heading_ - ros_state_->telemetry.heading;
    if (heading_error > 180.0) heading_error -= 360.0;
    if (heading_error < -180.0) heading_error += 360.0;
    float yaw_pwm = yaw_pd.pid_control(heading_error, elapsed, false);
    
    // Publish command
    custom_msgs::msg::Commands cmd;
    cmd.mode = flight_mode_;
    cmd.arm = true;
    cmd.forward = static_cast<int>(forward_pwm);
    cmd.lateral = static_cast<int>(lateral_pwm);
    cmd.thrust = 1500;
    cmd.yaw = static_cast<int>(yaw_pwm);
    ros_state_->cmd_publisher->publish(cmd);
    
    RCLCPP_DEBUG(ros_state_->node->get_logger(),
                "AlignXY: nx=%.3f ny=%.3f fwd=%d lat=%d yaw=%d",
                target_nx_, target_ny_, cmd.forward, cmd.lateral, cmd.yaw);
    
    // Reset target visible flag for next callback
    target_visible_ = false;
    
    return BT::NodeStatus::RUNNING;
}

void AlignXY::onHalted()
{
    RCLCPP_INFO(ros_state_->node->get_logger(), "AlignXY: halted");
    publish_neutral();
}

void AlignXY::publish_neutral()
{
    custom_msgs::msg::Commands cmd;
    cmd.mode = flight_mode_;
    cmd.arm = true;
    cmd.forward = 1500;
    cmd.lateral = 1500;
    cmd.thrust = 1500;
    cmd.yaw = 1500;
    ros_state_->cmd_publisher->publish(cmd);
}
