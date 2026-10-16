#pragma once

#include "objects.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <cmath>
#include <algorithm>
#include <optional>

// All missions share Pixhawk's GUIDED position/velocity/attitude controllers.
// This leaf only resolves a target and observes completion; it never sends effort.
class NavigateToPose : public BT::StatefulActionNode {
 public:
  NavigateToPose(const std::string& name, const BT::NodeConfiguration& config,
                 rclcpp::Node::SharedPtr node, std::shared_ptr<tf2_ros::Buffer> tf)
      : BT::StatefulActionNode(name, config), node_(node), tf_(std::move(tf)) {
    goals_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 1);
    hold_ = node_->create_client<std_srvs::srv::Trigger>("mira/guidance/hold");
    feedback_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        "vision/ekf_pose", 1, [this](geometry_msgs::msg::PoseStamped::SharedPtr p) { if (p->header.frame_id == "odom") pose_ = *p; });
  }

  static BT::PortsList providedPorts() {
    return {BT::InputPort<vision_msgs::msg::Detection2D>("object"),
            BT::InputPort<double>("stand_off", 0.0, "Stop before the object along the approach ray (metres)"),
            BT::InputPort<double>("through_distance", 0.0, "Continue beyond the object along the horizontal approach direction (metres)"),
            BT::InputPort<double>("approach_distance", 1.5, "Align this far before the gate when through_distance is positive (metres)"),
            BT::InputPort<double>("tolerance", 0.3, "Arrival radius (metres)"),
            BT::InputPort<double>("timeout", 60.0, "Navigation timeout (seconds)")};
  }

  BT::NodeStatus onStart() override {
    start_ = node_->now();
    settled_.reset();
    target_.reset();
    exit_goal_.reset();
    sent_ = false;
    tolerance_ = getInput<double>("tolerance").value();
    timeout_ = getInput<double>("timeout").value();
    const auto object = getInput<vision_msgs::msg::Detection2D>("object");
    if (!object || !std::isfinite(tolerance_) || tolerance_ <= 0 ||
        !std::isfinite(timeout_) || timeout_ <= 0) return fail("invalid object or navigation parameters");
    target_ = objectPose(object.value());
    if (!target_ || !fresh(*target_)) return fail("selected object is invalid or stale");
    return onRunning();
  }

  BT::NodeStatus onRunning() override {
    if ((node_->now() - start_).seconds() > timeout_) return fail("navigation timeout");
    if (!pose_ || !fresh(*pose_)) {
      if (sent_) return fail("EKF feedback is invalid or stale");
      return BT::NodeStatus::RUNNING;
    }
    if (!sent_) {
      if (!target_ || goals_->get_subscription_count() == 0) return BT::NodeStatus::RUNNING;
      if (!fresh(*target_)) return fail("selected object expired while waiting for goal subscriber or TF");
      try {
        goal_ = tf_->transform(*target_, "odom");
      } catch (const tf2::TransformException& error) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                            "Waiting for target transform: %s", error.what());
        return BT::NodeStatus::RUNNING;
      }
      auto& p = goal_.pose.position;
      const auto& current = pose_->pose.position;
      const double distance = std::hypot(std::hypot(p.x-current.x, p.y-current.y), p.z-current.z);
      const double stand_off = getInput<double>("stand_off").value();
      const double through = getInput<double>("through_distance").value();
      const double approach = getInput<double>("approach_distance").value();
      // A box gives a position, not an object orientation. Face the selected object.
      const auto& current_q = pose_->pose.orientation;
      target_yaw_ = std::hypot(p.x-current.x, p.y-current.y) > 1e-6
          ? std::atan2(p.y-current.y, p.x-current.x)
          : std::atan2(2*(current_q.w*current_q.z + current_q.x*current_q.y),
                       1-2*(current_q.y*current_q.y + current_q.z*current_q.z));
      if (!std::isfinite(distance) || !std::isfinite(stand_off) || stand_off < 0 ||
          !std::isfinite(through) || through < 0 || (through > 0 && stand_off > 0) ||
          (through > 0 && (!std::isfinite(approach) || approach <= 0)) ||
          !std::isfinite(target_yaw_)) return fail("invalid goal, stand_off or through_distance");
      if (distance > 0) {
        const double scale = std::min(stand_off / distance, 1.0);
        p.x -= scale * (p.x-current.x);
        p.y -= scale * (p.y-current.y);
        p.z -= scale * (p.z-current.z);
      }
      if (through > 0) {
        if (std::hypot(p.x-current.x, p.y-current.y) < 1e-6)
          return fail("gate traversal needs a horizontal approach direction");
        p.x += through * std::cos(target_yaw_);
        p.y += through * std::sin(target_yaw_);
      }
      goal_.pose.orientation.x = goal_.pose.orientation.y = 0;
      goal_.pose.orientation.z = std::sin(target_yaw_/2);
      goal_.pose.orientation.w = std::cos(target_yaw_/2);
      if (through > 0) {
        // A final point alone lets OA route around a gate post. First align on
        // the approach side, then cross with the same latched frame and heading.
        exit_goal_ = goal_;
        p.x -= (through+approach)*std::cos(target_yaw_);
        p.y -= (through+approach)*std::sin(target_yaw_);
      }
      goal_.header.stamp = node_->now();
      goals_->publish(goal_);
      sent_ = true;
      RCLCPP_INFO(node_->get_logger(), "Goal sent: %.3f, %.3f, %.3f", p.x, p.y, p.z);
    }
    const auto& a = pose_->pose.position;
    const auto& b = goal_.pose.position;
    const auto& q = pose_->pose.orientation;
    const double yaw = std::atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y+q.z*q.z));
    const double error = std::remainder(yaw-target_yaw_, 2*M_PI);
    if (std::hypot(std::hypot(a.x-b.x,a.y-b.y),a.z-b.z) <= tolerance_ &&
        std::abs(error) <= 0.15) {
      if (!settled_) settled_ = node_->now();
      if ((node_->now()-*settled_).seconds() >= 1.0) {
        if (exit_goal_) {
          goal_ = *exit_goal_;
          exit_goal_.reset();
          goal_.header.stamp = node_->now();
          goals_->publish(goal_);
          settled_.reset();
          const auto& p = goal_.pose.position;
          RCLCPP_INFO(node_->get_logger(), "Gate approach aligned; crossing goal: %.3f, %.3f, %.3f", p.x, p.y, p.z);
          return BT::NodeStatus::RUNNING;
        }
        RCLCPP_INFO(node_->get_logger(), "Arrival position and heading held for one second");
        return BT::NodeStatus::SUCCESS;
      }
    } else settled_.reset();
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override { stop(); }

 private:
  bool fresh(const geometry_msgs::msg::PoseStamped& p) const {
    const double age = (node_->now() - rclcpp::Time(p.header.stamp)).seconds();
    const auto& v = p.pose.position;
    const auto& q = p.pose.orientation;
    const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    return !p.header.frame_id.empty() && std::isfinite(norm) && std::abs(norm-1.0) < 0.01 && std::isfinite(v.x) && std::isfinite(v.y) &&
           std::isfinite(v.z) && age >= 0 && age <= 0.5;
  }
  void stop() {
    if (sent_) hold_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    sent_ = false;
  }
  BT::NodeStatus fail(const char* reason) {
    RCLCPP_WARN(node_->get_logger(), "Navigation failed: %s", reason);
    stop();
    return BT::NodeStatus::FAILURE;
  }
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goals_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr hold_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr feedback_;
  std::optional<geometry_msgs::msg::PoseStamped> pose_, target_, exit_goal_;
  geometry_msgs::msg::PoseStamped goal_;
  rclcpp::Time start_;
  std::optional<rclcpp::Time> settled_;
  bool sent_ = false;
  double tolerance_, timeout_, target_yaw_ = 0.0;
};
