#include "motions.hpp"

void ApproachWithDepth::point_callback(const geometry_msgs::msg::Point::SharedPtr msg)
{
    target_nx_ = msg->x;
    target_ny_ = msg->y;
    target_depth_ = msg->z;  // Depth proxy (e.g., bounding box area)
    target_visible_ = true;
    last_detection_time_ = ros_state_->node->now();
}

ApproachWithDepth::ApproachWithDepth(const std::string& name,
                                    const BT::NodeConfiguration& config,
                                    ROSState* ros_state)
    : BT::StatefulActionNode(name, config),
      ros_state_(ros_state),
      lateral_pd(name + "_lat", ros_state->node),
      forward_pd(name + "_fwd", ros_state->node),
      yaw_pd(name + "_yaw", ros_state->node),
      success_depth_(0.75),
      blind_lock_depth_(0.4),
      thrust_pwm_shallow_(1430.0),
      thrust_pwm_deep_(1460.0),
      depth_threshold_(0.4),
      target_lost_timeout_(1.0),
      timeout_(30.0),
      flight_mode_("ALT_HOLD"),
      target_nx_(0.0),
      target_ny_(0.0),
      target_depth_(0.0),
      prev_nx_(0.0),
      prev_ny_(0.0),
      target_visible_(false),
      locked_heading_(0.0),
      heading_locked_(false)
{
}

BT::PortsList ApproachWithDepth::providedPorts()
{
    return {
        BT::InputPort<std::string>("point_topic", "Topic publishing target point (geometry_msgs/Point)"),
        BT::InputPort<double>("success_depth", 0.75, "Depth proxy value to succeed"),
        BT::InputPort<double>("blind_lock_depth", 0.4, "Depth at which we can lose visual"),
        BT::InputPort<double>("thrust_pwm_shallow", 1430.0, "Thrust PWM for shallow depth"),
        BT::InputPort<double>("thrust_pwm_deep", 1460.0, "Thrust PWM for deep depth"),
        BT::InputPort<double>("depth_threshold", 0.4, "Depth threshold to switch thrust"),
        BT::InputPort<double>("target_lost_timeout", 1.0, "Seconds before aborting if target lost (when shallow)"),
        BT::InputPort<double>("timeout", 30.0, "Maximum time for approach (seconds)"),
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

BT::NodeStatus ApproachWithDepth::onStart()
{
    // Read input parameters
    auto point_topic = getInput<std::string>("point_topic");
    if (!point_topic) {
        throw BT::RuntimeError("Missing required input [point_topic]: ", point_topic.error());
    }
    point_topic_ = point_topic.value();
    
    success_depth_ = getInput<double>("success_depth").value();
    blind_lock_depth_ = getInput<double>("blind_lock_depth").value();
    thrust_pwm_shallow_ = getInput<double>("thrust_pwm_shallow").value();
    thrust_pwm_deep_ = getInput<double>("thrust_pwm_deep").value();
    depth_threshold_ = getInput<double>("depth_threshold").value();
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
        std::bind(&ApproachWithDepth::point_callback, this, std::placeholders::_1));
    
    // Reset state
    target_visible_ = false;
    start_time_ = ros_state_->node->now();
    last_detection_time_ = start_time_;
    last_update_time_ = start_time_;
    prev_nx_ = 0.0;
    prev_ny_ = 0.0;
    target_depth_ = 0.0;
    
    // Lock heading at current heading
    locked_heading_ = ros_state_->telemetry.heading;
    heading_locked_ = true;
    
    RCLCPP_INFO(ros_state_->node->get_logger(),
               "ApproachWithDepth: starting dive (target depth: %.2f)",
               success_depth_);
    
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ApproachWithDepth::onRunning()
{
    rclcpp::Time now = ros_state_->node->now();
    double elapsed = (now - start_time_).seconds();
    last_update_time_ = now;
    
    // Check for hard timeout
    if (elapsed > timeout_) {
        RCLCPP_WARN(ros_state_->node->get_logger(),
                   "ApproachWithDepth: timeout after %.1f s", elapsed);
        publish_neutral();
        return BT::NodeStatus::FAILURE;
    }
    
    // Success conditions
    if (target_visible_ && target_depth_ > success_depth_) {
        RCLCPP_INFO(ros_state_->node->get_logger(),
                   "ApproachWithDepth: target depth reached (%.2f > %.2f)",
                   target_depth_, success_depth_);
        publish_neutral();
        return BT::NodeStatus::SUCCESS;
    }
    
    // Blind lock - if we're close enough and lost visual
    double time_since_detection = (now - last_detection_time_).seconds();
    if (!target_visible_ && target_depth_ > blind_lock_depth_) {
        RCLCPP_INFO(ros_state_->node->get_logger(),
                   "ApproachWithDepth: close range blind lock (depth: %.2f)",
                   target_depth_);
        publish_neutral();
        return BT::NodeStatus::SUCCESS;
    }
    
    // Target lost when shallow - abort
    if (!target_visible_ && target_depth_ <= blind_lock_depth_ && 
        elapsed > 0.5 && time_since_detection > target_lost_timeout_) {
        RCLCPP_WARN(ros_state_->node->get_logger(),
                   "ApproachWithDepth: target lost high up, aborting");
        publish_neutral();
        return BT::NodeStatus::FAILURE;
    }
    
    // If target not visible but we're in acceptable blind range, hold position
    if (!target_visible_) {
        RCLCPP_DEBUG(ros_state_->node->get_logger(),
                    "ApproachWithDepth: target not visible, holding...");
        publish_neutral();
        return BT::NodeStatus::RUNNING;
    }
    
    // Calculate PD control outputs for XY alignment
    float lateral_pwm = lateral_pd.pid_control(target_nx_, elapsed, false);
    float forward_pwm = forward_pd.pid_control(-target_ny_, elapsed, false);
    
    // Calculate heading lock
    double heading_error = locked_heading_ - ros_state_->telemetry.heading;
    if (heading_error > 180.0) heading_error -= 360.0;
    if (heading_error < -180.0) heading_error += 360.0;
    float yaw_pwm = yaw_pd.pid_control(heading_error, elapsed, false);
    
    // Determine thrust based on depth
    int thrust_pwm = (target_depth_ > depth_threshold_) ? 
                     static_cast<int>(thrust_pwm_deep_) : 
                     static_cast<int>(thrust_pwm_shallow_);
    
    // Publish command
    custom_msgs::msg::Commands cmd;
    cmd.mode = flight_mode_;
    cmd.arm = true;
    cmd.forward = static_cast<int>(forward_pwm);
    cmd.lateral = static_cast<int>(lateral_pwm);
    cmd.thrust = thrust_pwm;
    cmd.yaw = static_cast<int>(yaw_pwm);
    ros_state_->cmd_publisher->publish(cmd);
    
    RCLCPP_DEBUG(ros_state_->node->get_logger(),
                "ApproachWithDepth: nx=%.3f ny=%.3f depth=%.3f thr=%d",
                target_nx_, target_ny_, target_depth_, cmd.thrust);
    
    // Reset target visible flag for next callback
    target_visible_ = false;
    
    return BT::NodeStatus::RUNNING;
}

void ApproachWithDepth::onHalted()
{
    RCLCPP_INFO(ros_state_->node->get_logger(), "ApproachWithDepth: halted");
    publish_neutral();
}

void ApproachWithDepth::publish_neutral()
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
