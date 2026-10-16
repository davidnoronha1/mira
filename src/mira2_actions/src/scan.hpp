#pragma once
#include "objects.hpp"
#include <std_srvs/srv/trigger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <algorithm>
#include <utils/topics.hpp>

// Survey a configurable circle about the actual starting EKF pose. radius=0
// rotates in place. Viewpoints are passed through without stopping: the next
// heading is sent once the current one is within half a step, so the vehicle
// turns continuously at its yaw-rate limit. The descent (clearing the starting
// frame) and the final return still settle. Sensor observations, never
// simulator coordinates, determine an optional object-facing final heading.
class ScanArea : public BT::StatefulActionNode {
 public:
  ScanArea(const std::string& name, const BT::NodeConfiguration& config,
           rclcpp::Node::SharedPtr node, std::shared_ptr<tf2_ros::Buffer> tf)
      : BT::StatefulActionNode(name, config), node_(node), tf_(std::move(tf)) {
    goals_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(utils::topics::GOAL, 1);
    hold_ = node_->create_client<std_srvs::srv::Trigger>(utils::topics::HOLD);
    feedback_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        utils::topics::POSE, 1, [this](geometry_msgs::msg::PoseStamped::SharedPtr p) {
          if (p->header.frame_id == "odom") pose_ = *p;
        });
  }
  static BT::PortsList providedPorts() {
    return {BT::InputPort<double>("radius", 0.0, "Survey circle radius in metres; zero rotates in place"),
            BT::InputPort<double>("sweep_degrees", 360.0, "Angular coverage, 0 to 360 degrees"),
            BT::InputPort<double>("step_degrees", 30.0, "Angle between survey viewpoints (heading lead)"),
            BT::InputPort<double>("depth", 1.0, "Survey depth below the surface in metres"),
            BT::InputPort<double>("minimum_depth", .6, "Required clearance below the surface starting frame"),
            BT::InputPort<double>("dwell", .3, "Seconds of stable sensing per viewpoint"),
            BT::InputPort<double>("timeout", 150.0, "Maximum seconds"),
            BT::InputPort<std::string>("face_class", "", "Optional detected class to face after returning to spawn"),
            BT::InputPort<std::string>("objects_topic", utils::topics::FRONT_OBJECTS, "Depth detections")};
  }
  BT::NodeStatus onStart() override {
    start_ = node_->now();
    settled_.reset();
    center_.reset();
    observation_.reset();
    facing_.reset();
    sent_ = returning_ = false;
    descending_ = true;
    step_ = 0;
    radius_ = getInput<double>("radius").value();
    sweep_ = getInput<double>("sweep_degrees").value();
    increment_ = getInput<double>("step_degrees").value();
    depth_ = getInput<double>("depth").value();
    dwell_ = getInput<double>("dwell").value();
    timeout_ = getInput<double>("timeout").value();
    if (!std::isfinite(radius_) || radius_ < 0 || !std::isfinite(sweep_) || sweep_ < 0 || sweep_ > 360 ||
        !std::isfinite(increment_) || increment_ <= 0 || increment_ > 90 ||
        !std::isfinite(depth_) || depth_ <= 0 || !std::isfinite(dwell_) || dwell_ < 0 ||
        !std::isfinite(timeout_) || timeout_ <= 0) return fail("invalid scan area parameters");
    const double minimum_depth = getInput<double>("minimum_depth").value();
    if (!std::isfinite(minimum_depth) || minimum_depth <= 0 || depth_ < minimum_depth)
      return fail("survey depth must clear the starting frame before lateral motion");
    count_ = static_cast<int>(std::ceil(sweep_/increment_));
    if (!getInput<std::string>("face_class").value().empty()) {
      subscription_ = node_->create_subscription<vision_msgs::msg::Detection2DArray>(
          getInput<std::string>("objects_topic").value(), rclcpp::SensorDataQoS(),
          [this](vision_msgs::msg::Detection2DArray::SharedPtr message) {
            for (const auto& d : message->detections) {
              for (const auto& h : d.results) {
                if (h.hypothesis.class_id != getInput<std::string>("face_class").value()) continue;
                auto candidate = d;
                candidate.results = {h};
                auto pose = objectPose(candidate);
                if (pose && fresh(*pose)) observation_ = *pose;
              }
            }
          });
    }
    return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus onRunning() override {
    if ((node_->now()-start_).seconds() > timeout_) return fail("scan timeout");
    if (!pose_ || !fresh(*pose_)) {
      // Pause through brief MAVLink delivery stalls; fail on sustained loss.
      if (sent_ && (!pose_ || !fresh(*pose_, kStaleLimit))) {
        if (pose_) RCLCPP_WARN(node_->get_logger(), "EKF pose age %.3f s",
                               (node_->now()-rclcpp::Time(pose_->header.stamp)).seconds());
        return fail("scan EKF feedback is stale");
      }
      return BT::NodeStatus::RUNNING;
    }
    if (!center_) {
      center_ = *pose_;
      const auto& q = center_->pose.orientation;
      initial_yaw_ = std::atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z));
      RCLCPP_INFO(node_->get_logger(), "Scanning spawn: radius %.2f m, sweep %.0f deg, depth %.2f m",
                  radius_, sweep_, depth_);
    }
    if (observation_ && fresh(*observation_)) {
      try {
        const auto p = tf_->transform(*observation_, "odom");
        facing_ = std::atan2(p.pose.position.y-center_->pose.position.y,
                             p.pose.position.x-center_->pose.position.x);
      } catch (const tf2::TransformException&) {}
    }
    if (!sent_ && goals_->get_subscription_count() > 0) publish();
    if (!sent_) return BT::NodeStatus::RUNNING;
    const auto& p = pose_->pose.position;
    const auto& g = goal_.pose.position;
    const auto& q = pose_->pose.orientation;
    const double yaw = std::atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z));
    const double error = std::abs(std::remainder(yaw-target_yaw_, 2*M_PI));
    const double distance = std::hypot(std::hypot(p.x-g.x,p.y-g.y),p.z-g.z);
    if (!descending_ && !returning_ && distance <= .3 && error <= increment_*M_PI/360) {
      // Pass through the viewpoint: keep turning instead of braking to dwell.
      if (++step_ > count_) returning_ = true;
      publish();
      return BT::NodeStatus::RUNNING;
    }
    if (distance <= .25 && error <= .15) {
      if (!settled_) settled_ = node_->now();
      if ((node_->now()-*settled_).seconds() >= dwell_) {
        if (descending_) {
          descending_ = false;
          if (count_ == 0) return BT::NodeStatus::SUCCESS;
          settled_.reset();
          publish();
          return BT::NodeStatus::RUNNING;
        }
        if (returning_) {
          subscription_.reset();
          RCLCPP_INFO(node_->get_logger(), "Spawn scan complete; returned to the survey origin");
          return BT::NodeStatus::SUCCESS;
        }
        ++step_;
        if (step_ > count_) returning_ = true;
        settled_.reset();
        publish();
      }
    } else settled_.reset();
    return BT::NodeStatus::RUNNING;
  }
  void onHalted() override { stop(); }
 private:
  bool fresh(const geometry_msgs::msg::PoseStamped& p, double maximum_age = .5) const {
    const double age = (node_->now()-rclcpp::Time(p.header.stamp)).seconds();
    const auto& v = p.pose.position;
    const auto& q = p.pose.orientation;
    return age >= 0 && age <= maximum_age && std::isfinite(v.x+v.y+v.z+q.x+q.y+q.z+q.w) &&
           std::abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.0) < .01;
  }
  void publish() {
    goal_ = *center_;
    goal_.header.stamp = node_->now();
    goal_.pose.position.z = -depth_;
    target_yaw_ = initial_yaw_ + std::min(sweep_, step_*increment_)*M_PI/180;
    if (returning_) target_yaw_ = facing_.value_or(initial_yaw_);
    else if (!descending_) {
      goal_.pose.position.x += radius_*std::cos(target_yaw_);
      goal_.pose.position.y += radius_*std::sin(target_yaw_);
    }
    goal_.pose.orientation.x = goal_.pose.orientation.y = 0;
    goal_.pose.orientation.z = std::sin(target_yaw_/2);
    goal_.pose.orientation.w = std::cos(target_yaw_/2);
    goals_->publish(goal_);
    sent_ = true;
  }
  void stop() {
    subscription_.reset();
    if (sent_) hold_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    sent_ = false;
  }
  BT::NodeStatus fail(const char* reason) {
    RCLCPP_WARN(node_->get_logger(), "Scan failed: %s", reason);
    stop();
    return BT::NodeStatus::FAILURE;
  }
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goals_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr hold_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr feedback_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr subscription_;
  std::optional<geometry_msgs::msg::PoseStamped> pose_, center_, observation_;
  std::optional<double> facing_;
  std::optional<rclcpp::Time> settled_;
  geometry_msgs::msg::PoseStamped goal_;
  rclcpp::Time start_;
  double radius_, sweep_, increment_, depth_, dwell_, timeout_, initial_yaw_, target_yaw_;
  int step_, count_;
  bool sent_ = false, returning_ = false, descending_ = true;
};
