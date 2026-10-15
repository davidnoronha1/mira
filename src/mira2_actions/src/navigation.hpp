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
                 rclcpp::Node::SharedPtr node)
      : BT::StatefulActionNode(name, config), node_(node), tf_(node->get_clock()),
        listener_(tf_) {
    goals_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 1);
    hold_ = node_->create_client<std_srvs::srv::Trigger>("mira/guidance/hold");
    feedback_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        "vision/ekf_pose", 1, [this](geometry_msgs::msg::PoseStamped::SharedPtr p) { if (p->header.frame_id == "odom") pose_ = *p; });
  }

  static BT::PortsList providedPorts() {
    return {BT::InputPort<vision_msgs::msg::Detection2D>("object"),
            BT::InputPort<double>("stand_off", 0.0, "Stop before the object along the approach ray (metres)"),
            BT::InputPort<double>("tolerance", 0.3, "Arrival radius (metres)"),
            BT::InputPort<double>("timeout", 60.0, "Navigation timeout (seconds)")};
  }

  BT::NodeStatus onStart() override {
    start_ = node_->now();
    settled_.reset();
    target_.reset();
    sent_ = false;
    tolerance_ = getInput<double>("tolerance").value();
    timeout_ = getInput<double>("timeout").value();
    const auto object = getInput<vision_msgs::msg::Detection2D>("object");
    if (!object || !std::isfinite(tolerance_) || tolerance_ <= 0 ||
        !std::isfinite(timeout_) || timeout_ <= 0) return BT::NodeStatus::FAILURE;
    target_ = objectPose(object.value());
    if (!target_ || !fresh(*target_)) return BT::NodeStatus::FAILURE;
    return onRunning();
  }

  BT::NodeStatus onRunning() override {
    if ((node_->now() - start_).seconds() > timeout_) return fail();
    if (!pose_ || !fresh(*pose_)) {
      if (sent_) return fail();
      return BT::NodeStatus::RUNNING;
    }
    if (!sent_) {
      if (!target_ || goals_->get_subscription_count() == 0) return BT::NodeStatus::RUNNING;
      if (!fresh(*target_)) return fail();
      try {
        goal_ = tf_.transform(*target_, "odom");
      } catch (const tf2::TransformException&) {
        return BT::NodeStatus::RUNNING;
      }
      auto& p = goal_.pose.position;
      const auto& current = pose_->pose.position;
      const double distance = std::hypot(std::hypot(p.x-current.x, p.y-current.y), p.z-current.z);
      const double stand_off = getInput<double>("stand_off").value();
      // A box gives a position, not an object orientation. Face the selected object.
      const auto& current_q = pose_->pose.orientation;
      target_yaw_ = std::hypot(p.x-current.x, p.y-current.y) > 1e-6
          ? std::atan2(p.y-current.y, p.x-current.x)
          : std::atan2(2*(current_q.w*current_q.z + current_q.x*current_q.y),
                       1-2*(current_q.y*current_q.y + current_q.z*current_q.z));
      if (!std::isfinite(distance) || !std::isfinite(stand_off) || stand_off < 0 ||
          !std::isfinite(target_yaw_)) return fail();
      if (distance > 0) {
        const double scale = std::min(stand_off / distance, 1.0);
        p.x -= scale * (p.x-current.x);
        p.y -= scale * (p.y-current.y);
        p.z -= scale * (p.z-current.z);
      }
      goal_.pose.orientation.x = goal_.pose.orientation.y = 0;
      goal_.pose.orientation.z = std::sin(target_yaw_/2);
      goal_.pose.orientation.w = std::cos(target_yaw_/2);
      goal_.header.stamp = node_->now();
      goals_->publish(goal_);
      sent_ = true;
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
  BT::NodeStatus fail() { stop(); return BT::NodeStatus::FAILURE; }
  rclcpp::Node::SharedPtr node_;
  tf2_ros::Buffer tf_;
  tf2_ros::TransformListener listener_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goals_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr hold_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr feedback_;
  std::optional<geometry_msgs::msg::PoseStamped> pose_, target_;
  geometry_msgs::msg::PoseStamped goal_;
  rclcpp::Time start_;
  std::optional<rclcpp::Time> settled_;
  bool sent_ = false;
  double tolerance_, timeout_, target_yaw_ = 0.0;
};
