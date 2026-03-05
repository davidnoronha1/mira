#pragma once 

#include "../common.hpp"
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/bt_factory.h>
#include <cmath>
#include <control_utils/control_utils.hpp>
#include <custom_msgs/msg/commands.hpp>
#include <custom_msgs/msg/telemetry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/bounding_box2_d_array.hpp>


class ApproachBB : public BT::SyncActionNode
{
    ROSState* ros_state_;

    rclcpp::Subscription<vision_msgs::msg::BoundingBox2DArray>::SharedPtr bb_sub_;



    // make PID Controller
    PID_Controller yaw_pid;
    PID_Controller lateral_pid;


    // params read on first tick
    std::string target_object_;
    double forward_pwm_ = 1550.0;
    double frame_width_ = 1.0;
    double frame_height_ = 1.0;
    double success_bb_area_ = 0.70;
    double x_tolerance_ = 0.05;
    double bb_lost_timeout_ = 1.0;
    double timeout_ = 20.0;

    std::string flight_mode_ = "ALT_HOLD";

    //runtime state 

    bool initialized_ = false;
    rclcpp::Time start_time_;
    rclcpp::Time last_detection_time_; // tracks when we last saw the object 

    // latest bb info
    bool bb_found_           = false;
    double bb_x_center_norm_ = 0.5; // 0.5 -> screen center, 0.0 -> left edge, 1.0 -> right edge
    double bb_area_norm_         = 0.0; // fraction of frame area 


    void bbox_callback(const vision_msgs::msg::BoundingBox2DArray::SharedPtr msg)
    {
        bb_found_ = false;
        bb_area_norm_ = 0.0;      // already there ✓
        bb_x_center_norm_ = 0.5;  // ← add this too, stale center could cause initial jerk
        for (const auto& bbox : msg->boxes)
        {
            if(bbox.label != target_object_)
                continue;

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


public: 

ApproachBB(const std::string& name, 
           const BT::NodeConfiguration& config,
           ROSState* ros_state)
    : BT::SyncActionNode(name, config),
    yaw_pid("yaw",ros_state->node),
    lateral_pid("lateral",ros_state->node),
    ros_state_(ros_state)
    {
    bbox_sub_ = ros_state_->node->create_subscription<vision_msgs::msg::BoundingBox2DArray>(
        "/vision/detected_objects", 10, std::bind(&ApproachBB::bbox_callback, this, std::placeholders::_1));
    }


static BT::PortsList providedPorts()
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

            // Yaw PID — rotates the vehicle to center BB horizontally
            BT::InputPort<double>("yaw_pid_kp",          1.5,    "Yaw PID proportional gain"),
            BT::InputPort<double>("yaw_pid_ki",          0.005,  "Yaw PID integral gain"),
            BT::InputPort<double>("yaw_pid_kd",          3.0,    "Yaw PID derivative gain"),
            BT::InputPort<double>("yaw_pid_base_offset", 1500.0, "Yaw PID base offset (neutral PWM)"),

            // Lateral PID — strafes the vehicle to center BB horizontally
            BT::InputPort<double>("lateral_pid_kp",          1.0,    "Lateral PID proportional gain"),
            BT::InputPort<double>("lateral_pid_ki",          0.002,  "Lateral PID integral gain"),
            BT::InputPort<double>("lateral_pid_kd",          2.0,    "Lateral PID derivative gain"),
            BT::InputPort<double>("lateral_pid_base_offset", 1500.0, "Lateral PID base offset (neutral PWM)"),       
        };
}
    

    // ─────────────────────────────────────────────────────────────────────────
    // tick — called every BT cycle (10 Hz in your main loop)
    //
    // FLOW:
    //   First tick only:  read all ports, configure PIDs, record start time
    //   Every tick:
    //     1. Check hard timeout → FAILURE
    //     2. Check object lost timeout → FAILURE
    //     3. If BB found this frame:
    //        a. Check success condition (BB area >= threshold) → SUCCESS
    //        b. Compute x-center error → run yaw + lateral PIDs
    //        c. Publish forward + yaw + lateral command
    //     4. If BB not found this frame:
    //        a. Publish neutral to stop and wait
    //        b. RUNNING (still within object_lost_timeout_)
    // ─────────────────────────────────────────────────────────────────────────


BT::NodeStatus tick() override
{
    if(!initialized_){
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
        yaw_pid.kp = getInput<double>("yaw_pid_kp").value();
        yaw_pid.ki = getInput<double>("yaw_pid_ki").value();
        yaw_pid.kd = getInput<double>("yaw_pid_kd").value();
        yaw_pid.base_offset = getInput<double>("yaw_pid_base_offset").value();
        yaw_pid.emptyError();

        lateral_pid.kp = getInput<double>("lateral_pid_kp").value();
        lateral_pid.ki = getInput<double>("lateral_pid_ki").value();
        lateral_pid.kd = getInput<double>("lateral_pid_kd").value();
        lateral_pid.base_offset = getInput<double>("lateral_pid_base_offset").value();
        lateral_pid.emptyError();

        start_time_ = ros_state_->node->now();
        last_detection_time_ = start_time_;
        initialized_ = true;
        bb_found_ = false;
        bb_area_norm_ = 0.0;

        RCLCPP_INFO(ros_state_->node->get_logger(),
            "ApproachBB: starting to approach to '%s' "
            "(success at bb_area=%.0f%%, flight_mode=%s)",
            target_object_.c_str(), success_bb_area_ * 100.0, flight_mode_.c_str());
    }

    rclcpp::Time now = ros_state_->node->now();
    double elapsed = (now - start_time_).seconds();

    // ── 1. Hard timeout check ─────────────────────────────────────────────
        if (elapsed > timeout_) {
            RCLCPP_WARN(ros_state_->node->get_logger(),
                "ApproachBoundingBox: hard timeout after %.1f s", elapsed);
            publish_neutral();
            initialized_ = false;
            return BT::NodeStatus::FAILURE;
        }

    // ── 2. Object lost timeout ────────────────────────────────────────────
        // If the BB hasn't been seen for object_lost_timeout_ seconds → FAILURE.
        // This handles: object moved out of frame, vision node crashed, etc.
        double time_since_detection = (now - last_detection_time_).seconds();
        if (elapsed > 1.0 && time_since_detection > object_lost_timeout_) {
            // The elapsed > 1.0 guard avoids false triggers right at startup
            // before the first detection has had a chance to arrive.
            RCLCPP_WARN(ros_state_->node->get_logger(),
                "ApproachBoundingBox: object '%s' lost for %.1f s → FAILURE",
                target_object_.c_str(), time_since_detection);
            publish_neutral();
            initialized_ = false;
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
            initialized_ = false;
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

        // ── 6. PID outputs ────────────────────────────────────────────────────
        // Both PIDs get the SAME x_error but control different actuators.
        // Yaw:     rotates the whole vehicle — good for large errors
        // Lateral: strafes sideways — good for fine centering while surging
        float yaw_pwm     = yaw_pid.pid_control(x_error, elapsed, false);
        float lateral_pwm = lateral_pid.pid_control(x_error, elapsed, false);

        // ── 7. Determine forward PWM ──────────────────────────────────────────
        // Optional: reduce forward surge when BB is not centered yet to avoid
        // overshooting past the object while still correcting alignment.
        // If |x_error| > x_center_tolerance_, use a gentler forward value.
        double effective_forward = forward_pwm_;
        if (std::abs(x_error) > x_center_tolerance_) {
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
        cmd.yaw     = static_cast<int>(yaw_pwm);
        ros_state_->cmd_publisher->publish(cmd);

        RCLCPP_DEBUG(ros_state_->node->get_logger(),
            "ApproachBoundingBox: bb_area=%.1f%% x_err=%.3f "
            "fwd=%d lat=%d yaw=%d",
            bb_area_norm_ * 100.0, x_error,
            cmd.forward, cmd.lateral, cmd.yaw);

        return BT::NodeStatus::RUNNING;
    }

private:
    void publish_neutral()
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

    
};
