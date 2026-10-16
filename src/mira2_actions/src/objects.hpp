#pragma once

#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <cmath>
#include <optional>
#include <utils/topics.hpp>

// Seconds of continuous EKF silence before a motion node gives up and holds.
// Shorter MAVLink delivery stalls only pause decisions. pixhawk_controller's own 1 s
// watchdog disarms on a real feedback loss.
inline constexpr double kStaleLimit = 1.5;

// mira2_vision enriches each YOLO hypothesis with the registered-depth position
// in the detection header's optical frame. An unlocalized box has zero range.
inline std::optional<geometry_msgs::msg::PoseStamped>
objectPose(const vision_msgs::msg::Detection2D& object) {
  if (object.header.frame_id.empty() || object.results.size() != 1) return std::nullopt;
  const auto& pose = object.results.front().pose.pose;
  const auto& p = pose.position;
  const auto& q = pose.orientation;
  const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) || p.z <= 0 ||
      !std::isfinite(norm) || std::abs(norm-1.0) > 0.01) return std::nullopt;
  geometry_msgs::msg::PoseStamped result;
  result.header = object.header;
  result.pose = pose;
  return result;
}

class SelectObject : public BT::StatefulActionNode {
 public:
  SelectObject(const std::string& name, const BT::NodeConfiguration& config,
               rclcpp::Node::SharedPtr node)
      : BT::StatefulActionNode(name, config), node_(node) {}

  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("class_id"),
            BT::InputPort<std::string>("objects_topic", utils::topics::FRONT_OBJECTS, "Depth-enriched YOLO detections"),
            BT::OutputPort<vision_msgs::msg::Detection2D>("object")};
  }

  BT::NodeStatus onStart() override {
    const auto label = getInput<std::string>("class_id");
    if (!label || label.value().empty()) return BT::NodeStatus::FAILURE;
    label_ = label.value();
    selected_.reset();
    start_ = node_->now();
    subscription_ = node_->create_subscription<vision_msgs::msg::Detection2DArray>(
        getInput<std::string>("objects_topic").value(), rclcpp::SensorDataQoS(),
        [this](vision_msgs::msg::Detection2DArray::SharedPtr message) {
          selected_.reset();
          double best_score = -1;
          for (const auto& detection : message->detections) {
            const double age = (node_->now() - rclcpp::Time(detection.header.stamp)).seconds();
            if (age < 0 || age > 0.5) continue;
            for (const auto& hypothesis : detection.results) {
              const double score = hypothesis.hypothesis.score;
              if (hypothesis.hypothesis.class_id != label_ || !std::isfinite(score) ||
                  score < 0 || score > 1 || score <= best_score) continue;
              auto candidate = detection;
              candidate.results = {hypothesis};
              if (!objectPose(candidate)) continue;
              selected_ = candidate;
              best_score = score;
            }
          }
        });
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override {
    if (selected_) {
      const double age = (node_->now() - rclcpp::Time(selected_->header.stamp)).seconds();
      if (age >= 0 && age <= 0.5) {
        setOutput("object", *selected_);
        RCLCPP_INFO(node_->get_logger(), "Selected localized object '%s'", label_.c_str());
        subscription_.reset();
        return BT::NodeStatus::SUCCESS;
      }
    }
    // A visible target is assumed. This is a short acquisition window, not a search maneuver.
    if ((node_->now() - start_).seconds() >= 1.0) {
      RCLCPP_WARN(node_->get_logger(), "No fresh localized object for '%s'", label_.c_str());
      subscription_.reset();
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override { subscription_.reset(); selected_.reset(); }

 private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr subscription_;
  std::optional<vision_msgs::msg::Detection2D> selected_;
  std::string label_;
  rclcpp::Time start_;
};
