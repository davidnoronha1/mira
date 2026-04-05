#include "motions.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

void ApproachBB::bbox_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
{
    bb_found_ = false;
    bb_area_norm_ = 0.0;
    bb_x_center_norm_ = 0.5;
    for (const auto& detection : msg->detections)
    {
        if(detection.id != target_object_)
            continue;

        auto bbox = detection.bbox;

        double x_center_norm = bbox.center.position.x / frame_width_ ;

        double bb_area_pixels = bbox.size_x * bbox.size_y;
        double frame_area = frame_width_ * frame_height_;
        double bb_area = bb_area_pixels / frame_area;

        if(!bb_found_ || bb_area > bb_area_norm_)
        {
            bb_found_ = true;
            bb_x_center_norm_ = x_center_norm;
            bb_area_norm_ = bb_area;
        }
    }
    if(bb_found_)
        last_detection_time_ = ros_state_->node->now();
}

ApproachBB::ApproachBB(const std::string& name, 
           const BT::NodeConfiguration& config,
           ROSState* ros_state)
    : BT::StatefulActionNode(name, config),
    ros_state_(ros_state),
    lateral_pid("lateral",ros_state->node),
    depth_pid("depth",ros_state->node),
    forward_pwm_(1550.0),
    frame_width_(1.0),
    frame_height_(1.0),
    success_bb_area_(0.70),
    bb_lost_timeout_(1.0),
    timeout_(20.0),
    flight_mode_("ALT_HOLD"),
    bb_found_(false),
    bb_x_center_norm_(0.5),
    bb_area_norm_(0.0)
{
    bb_sub_ = ros_state_->node->create_subscription<vision_msgs::msg::Detection2DArray>(
        "/vision/detections", 10, std::bind(&ApproachBB::bbox_callback, this, std::placeholders::_1));
}

BT::PortsList ApproachBB::providedPorts()
{
    return {
        BT::InputPort<std::string>("object","Target Object label to approach "),
        BT::InputPort<double>("forward_pwm",1550.0,"PWM value for forward movement, default 1550"),
        // Frame resolution — set these if your vision node gives raw pixels.
            // Leave at 1.0 if it already gives normalized [0,1] coordinates.
            BT::InputPort<double>("frame_width",  1.0, "Camera frame width in pixels (or 1.0 if normalized)"),
            BT::InputPort<double>("frame_height", 1.0, "Camera frame height in pixels (or 1.0 if normalized)"),

            // Success condition
            BT::InputPort<double>("success_area_ratio", 0.70,
                "Succeed when BB area >= this fraction of frame area (0.0-1.0). "
                "0.70 means object fills 70% of screen → you are close enough."),

            // Centering tolerance
            BT::InputPort<double>("x_center_tolerance", 0.05,
                "Normalized horizontal tolerance to consider BB centered (0.05 = 5% of frame)"),

            // Loss detection
            BT::InputPort<double>("object_lost_timeout", 3.0,
                "Seconds after which FAILURE is declared if BB disappears"),

            // Hard timeout
            BT::InputPort<double>("timeout", 60.0,
                "Maximum time for entire approach before FAILURE (seconds)"),

            // Flight controller mode
            BT::InputPort<std::string>("flight_mode", "ALT_HOLD",
                "FC mode: ALT_HOLD (auto depth hold) or STABILIZE (manual thrust)"),

            // Lateral PID — strafes the vehicle to center BB horizontally
            BT::InputPort<double>("lateral_pid_kp",          1.0,    "Lateral PID proportional gain"),
            BT::InputPort<double>("lateral_pid_ki",          0.002,  "Lateral PID integral gain"),
            BT::InputPort<double>("lateral_pid_kd",          2.0,    "Lateral PID derivative gain"),
            BT::InputPort<double>("lateral_pid_base_offset", 1500.0, "Lateral PID base offset (neutral PWM)"),       
        };
}

BT::NodeStatus ApproachBB::onStart()
{
    auto object = getInput<std::string>("object");
    if(!object){
        throw BT::RuntimeError("Mission required input [object]: ", object.error());
    }

    target_object_ = object.value();
    forward_pwm_ = getInput<double>("forward_pwm").value();
    frame_width_ = getInput<double>("frame_width").value();
    frame_height_ = getInput<double>("frame_height").value();
    success_bb_area_ = getInput<double>("success_area_ratio").value();
    x_tolerance_ = getInput<double>("x_center_tolerance").value();
    bb_lost_timeout_ = getInput<double>("object_lost_timeout").value();
    timeout_ = getInput<double>("timeout").value();
    flight_mode_ = getInput<std::string>("flight_mode").value();

    // Configure PIDs
    lateral_pid.kp = getInput<double>("lateral_pid_kp").value();
    lateral_pid.ki = getInput<double>("lateral_pid_ki").value();
    lateral_pid.kd = getInput<double>("lateral_pid_kd").value();
    lateral_pid.base_offset = getInput<double>("lateral_pid_base_offset").value();
    lateral_pid.emptyError();

    start_time_ = ros_state_->node->now();
    last_detection_time_ = start_time_;
    bb_found_ = false;
    bb_area_norm_ = 0.0;

    RCLCPP_INFO(ros_state_->node->get_logger(),
        "ApproachBB: starting to approach to '%s' "
        "(success at bb_area=%.0f%%, flight_mode=%s)",
        target_object_.c_str(), success_bb_area_ * 100.0, flight_mode_.c_str());
    
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ApproachBB::onRunning()
{
    rclcpp::Time now = ros_state_->node->now();
    double elapsed = (now - start_time_).seconds();

    // ── 1. Hard timeout check ─────────────────────────────────────────────
        if (elapsed > timeout_) {
            RCLCPP_WARN(ros_state_->node->get_logger(),
                "ApproachBoundingBox: hard timeout after %.1f s", elapsed);
            publish_neutral();
            return BT::NodeStatus::FAILURE;
        }

    // ── 2. Object lost timeout ────────────────────────────────────────────
        // If the BB hasn't been seen for object_lost_timeout_ seconds → FAILURE.
        // This handles: object moved out of frame, vision node crashed, etc.
        double time_since_detection = (now - last_detection_time_).seconds();
        if (elapsed > 1.0 && time_since_detection > bb_lost_timeout_) {
            // The elapsed > 1.0 guard avoids false triggers right at startup
            // before the first detection has had a chance to arrive.
            RCLCPP_WARN(ros_state_->node->get_logger(),
                "ApproachBoundingBox: object '%s' lost for %.1f s → FAILURE",
                target_object_.c_str(), time_since_detection);
            publish_neutral();
            return BT::NodeStatus::FAILURE;
        }
    

     // ── 3. BB not visible in latest frame → hold and wait ────────────────
        if (!bb_found_) {
            RCLCPP_DEBUG(ros_state_->node->get_logger(),
                "ApproachBoundingBox: no detection this frame, waiting (%.1f s since last seen)",
                time_since_detection);
            publish_neutral();
            return BT::NodeStatus::RUNNING;
        }
    // ── 4. Success condition ──────────────────────────────────────────────
        // BB area as fraction of frame area >= threshold means we are close enough.
        // e.g. success_bb_area_ = 0.70 → BB fills 70% of screen → SUCCESS.
        if (bb_area_norm_ >= success_bb_area_) {
            RCLCPP_INFO(ros_state_->node->get_logger(),
                "ApproachBoundingBox: reached '%s' (bb_area=%.1f%% >= %.1f%%)",
                target_object_.c_str(),
                bb_area_norm_ * 100.0,
                success_bb_area_ * 100.0);
            publish_neutral();
            return BT::NodeStatus::SUCCESS;
        }
    
    // ── 5. Compute horizontal centering error ─────────────────────────────
        // x_error is positive when BB center is to the RIGHT of screen center.
        // x_error is negative when BB center is to the LEFT of screen center.
        // Range: [-0.5, +0.5] in normalized coordinates.
        //
        //   screen: [0.0 ──── 0.5 (center) ──── 1.0]
        //   error = bb_x_center - 0.5
        //   error > 0 → BB is right of center → yaw right, strafe right
        //   error < 0 → BB is left of center  → yaw left,  strafe left
        double x_error = bb_x_center_norm_ - 0.5;

        // ── 6. PID output ─────────────────────────────────────────────────────
        // Lateral PID strafes the vehicle to center the BB horizontally.
        float lateral_pwm = lateral_pid.pid_control(x_error, elapsed, false);

        // ── 7. Determine forward PWM ──────────────────────────────────────────
        // Optional: reduce forward surge when BB is not centered yet to avoid
        // overshooting past the object while still correcting alignment.
        // If |x_error| > x_center_tolerance_, use a gentler forward value.
        double effective_forward = forward_pwm_;
        if (std::abs(x_error) > x_tolerance_) {
            // Reduce forward by up to 50% proportional to how off-center we are.
            // At x_error=0.5 (fully off-center) → forward reduced to 50%.
            // At x_error=0.05 (just outside tolerance) → nearly full forward.
            double centering_factor = 1.0 - 0.5 * (std::abs(x_error) / 0.5);
            centering_factor = std::max(0.5, std::min(1.0, centering_factor));
            effective_forward = 1500.0 + (forward_pwm_ - 1500.0) * centering_factor;
        }

        // ── 8. Publish command ────────────────────────────────────────────────
        custom_msgs::msg::Commands cmd;
        cmd.mode    = flight_mode_;   // "ALT_HOLD" → FC holds depth automatically
        cmd.arm     = true;
        cmd.forward = static_cast<int>(effective_forward);
        cmd.lateral = static_cast<int>(lateral_pwm);
        cmd.thrust  = 1500;           // neutral — ALT_HOLD ignores this anyway
        cmd.yaw     = 1500;           // neutral — yaw not controlled here
        ros_state_->cmd_publisher->publish(cmd);

        RCLCPP_DEBUG(ros_state_->node->get_logger(),
            "ApproachBoundingBox: bb_area=%.1f%% x_err=%.3f "
            "fwd=%d lat=%d yaw=%d",
            bb_area_norm_ * 100.0, x_error,
            cmd.forward, cmd.lateral, cmd.yaw);

        return BT::NodeStatus::RUNNING;
    }

void ApproachBB::onHalted()
{
    RCLCPP_INFO(ros_state_->node->get_logger(), "ApproachBB: halted");
    publish_neutral();
}

void ApproachBB::publish_neutral()
{
    custom_msgs::msg::Commands cmd;
    cmd.mode    = flight_mode_;
    cmd.arm     = true;
    cmd.forward = 1500;
    cmd.lateral = 1500;
    cmd.thrust  = 1500;
    cmd.yaw     = 1500;
    ros_state_->cmd_publisher->publish(cmd);
}

