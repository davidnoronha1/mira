#pragma once

#include "objects.hpp"
#include <std_srvs/srv/trigger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <algorithm>
#include <utils/topics.hpp>

// Gate-specific visual approach. Fresh depth detections are averaged into the
// mapped gate while moving; geometry is latched before close-range occlusion.
// The vehicle heads for a staging point on the gate axis, so an obstacle detour
// (e.g. around a flare in front of the gate) leaves room to realign, then crosses
// as soon as it is aligned. The crossing itself is a straight line on Pixhawk's
// position controller (goal_direct): obstacle avoidance would otherwise
// route around a post of the narrow, visually verified aperture. Pixhawk owns OA
// for the approach and all motion control. No simulator coordinates enter this
// routine.
class TraverseGate : public BT::StatefulActionNode {
 public:
  TraverseGate(const std::string& name, const BT::NodeConfiguration& config,
               rclcpp::Node::SharedPtr node, std::shared_ptr<tf2_ros::Buffer> tf)
      : BT::StatefulActionNode(name, config), node_(node), tf_(std::move(tf)) {
    goals_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(utils::topics::GOAL, 1);
    direct_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(utils::topics::GOAL_DIRECT, 1);
    hold_ = node_->create_client<std_srvs::srv::Trigger>(utils::topics::HOLD);
    feedback_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        utils::topics::POSE, 1, [this](geometry_msgs::msg::PoseStamped::SharedPtr p) {
          if (p->header.frame_id == "odom") pose_ = *p;
        });
  }
  static BT::PortsList providedPorts() {
    return {BT::InputPort<vision_msgs::msg::Detection2D>("object"),
            BT::InputPort<std::string>("class_id", "gate", "Target class"),
            BT::InputPort<std::string>("objects_topic", utils::topics::FRONT_OBJECTS, "Depth detections"),
            BT::InputPort<double>("through_distance", 1.5, "Distance beyond the gate"),
            BT::InputPort<double>("stage_distance", 2.0, "Alignment point before the gate"),
            BT::InputPort<double>("timeout", 90.0, "Maximum seconds")};
  }
  BT::NodeStatus onStart() override {
    start_ = node_->now();
    last_publish_ = start_;
    settled_.reset();
    last_accept_.reset();
    geometry_latched_ = false;
    gate_.reset();
    sent_ = crossing_ = false;
    timeout_ = getInput<double>("timeout").value();
    through_ = getInput<double>("through_distance").value();
    stage_ = getInput<double>("stage_distance").value();
    samples_ = 0;
    if (!std::isfinite(timeout_) || timeout_ <= 0 || !std::isfinite(through_) || through_ < 1.0 ||
        !std::isfinite(stage_) || stage_ < 1.0 || stage_ > 3.5)
      return fail("invalid gate parameters");
    auto object = getInput<vision_msgs::msg::Detection2D>("object");
    if (!object || !(observation_ = objectPose(object.value())) || !fresh(*observation_, .5))
      return fail("gate observation is invalid or stale");
    subscription_ = node_->create_subscription<vision_msgs::msg::Detection2DArray>(
        getInput<std::string>("objects_topic").value(), rclcpp::SensorDataQoS(),
        [this](vision_msgs::msg::Detection2DArray::SharedPtr message) {
          if (crossing_) return;
          double best = -1;
          for (const auto& detection : message->detections) {
            for (const auto& hypothesis : detection.results) {
              if (hypothesis.hypothesis.class_id != getInput<std::string>("class_id").value() ||
                  !std::isfinite(hypothesis.hypothesis.score) || hypothesis.hypothesis.score < 0 ||
                  hypothesis.hypothesis.score > 1 || hypothesis.hypothesis.score <= best) continue;
              auto candidate = detection;
              candidate.results = {hypothesis};
              auto p = objectPose(candidate);
              if (p && fresh(*p, .5)) {
                observation_ = *p;
                best = hypothesis.hypothesis.score;
              }
            }
          }
        });
    return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus onRunning() override {
    if ((node_->now()-start_).seconds() > timeout_) return fail("gate timeout");
    if (!pose_ || !fresh(*pose_, .5)) {
      // MAVLink delivery stalls of ~0.25 s occur under load; pause decisions
      // while Pixhawk keeps flying the last target, and fail on sustained loss.
      if (sent_ && (!pose_ || !fresh(*pose_, kStaleLimit))) {
        if (pose_) RCLCPP_WARN(node_->get_logger(), "EKF pose age %.3f s",
                               (node_->now()-rclcpp::Time(pose_->header.stamp)).seconds());
        return fail("EKF feedback is stale");
      }
      return BT::NodeStatus::RUNNING;
    }
    if (!crossing_ && !geometry_latched_ && observation_ && fresh(*observation_, .5)) {
      try {
        const auto observed = tf_->transform(*observation_, "odom", tf2::durationFromSec(.05));
        if (!gate_) {
          gate_ = observed;
          samples_ = 1;
          const auto& p = pose_->pose.position;
          const auto& g = gate_->pose.position;
          const double d = std::hypot(g.x-p.x, g.y-p.y);
          if (d < .5) return fail("gate approach direction is undefined");
          nx_ = (g.x-p.x)/d;
          ny_ = (g.y-p.y)/d;
          accepted(d);
        } else if (rclcpp::Time(observed.header.stamp) > rclcpp::Time(gate_->header.stamp)) {
          // A stationary gate cannot jump when a flare/floor depth contaminates
          // the perimeter: reject such jumps. Gate against the current estimate,
          // not the first sighting, so slow odometry drift (dead reckoning) is
          // tracked instead of rejecting every later observation.
          const auto& b = observed.pose.position;
          auto& m = gate_->pose.position;
          if (std::hypot(std::hypot(b.x-m.x,b.y-m.y),b.z-m.z) <= .3) {
            // Average while keeping a short memory (~5 observations), since
            // single depth estimates scatter ~0.1 m but the odom frame drifts.
            ++samples_;
            const double weight = std::max(1.0/samples_, .2);
            m.x += (b.x-m.x)*weight;
            m.y += (b.y-m.y)*weight;
            m.z += (b.z-m.z)*weight;
            gate_->header.stamp = observed.header.stamp;
            const auto& p = pose_->pose.position;
            accepted(std::hypot(b.x-p.x, b.y-p.y));
          }
        }
      } catch (const tf2::TransformException& e) {
        // Continue from the previous valid observation while TF catches up.
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                             "Gate observation not transformable to odom: %s", e.what());
      }
    }
    if (!gate_ || goals_->get_subscription_count() == 0) {
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                           "Gate waiting: mapped=%d observation=%d fresh=%d controllers=%zu",
                           gate_.has_value(), observation_.has_value(),
                           observation_ && fresh(*observation_, .5), goals_->get_subscription_count());
      return BT::NodeStatus::RUNNING;
    }
    const auto& p = pose_->pose.position;
    const auto& g = gate_->pose.position;
    const double ahead = (g.x-p.x)*nx_ + (g.y-p.y)*ny_;
    const double lateral = std::abs((g.x-p.x)*ny_ - (g.y-p.y)*nx_);
    const double vertical = std::abs(g.z-p.z);
    const auto& q = pose_->pose.orientation;
    const double yaw = std::atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z));
    const double heading_error = std::abs(std::remainder(yaw-std::atan2(ny_, nx_), 2*M_PI));
    // Latch the gate before the posts become cropped or occluded at close
    // range, but only from a fresh sighting within 4 m: odometry then only has
    // to carry the gate a few metres. Without such a sighting there is no
    // crossing, rather than a crossing of a stale or drifted gate estimate.
    if (!geometry_latched_ && last_accept_ && (node_->now()-*last_accept_).seconds() <= 1.0 &&
        last_range_ <= 4.0) {
      geometry_latched_ = true;
      subscription_.reset();
      RCLCPP_INFO(node_->get_logger(), "Gate latched from a sighting %.1f m away", last_range_);
    }
    // Cross from anywhere in the corridor between just past the staging point
    // and 0.8 m before the plane: an aligned vehicle never stops to stage.
    if (!crossing_ && geometry_latched_ && ahead <= stage_+.7 && ahead > .8 && lateral <= .20 &&
        vertical <= .25 && heading_error <= .30 && direct_->get_subscription_count() > 0) {
      crossing_ = true;
      subscription_.reset();
      publish(through_, *direct_);
      RCLCPP_INFO(node_->get_logger(), "Gate corridor aligned (lateral %.2f m, %.1f m ahead from %d "
                  "observations); crossing", lateral, ahead, samples_);
    } else if (!crossing_) {
      if (ahead <= 0) return fail("reached the gate plane without an aligned crossing");
      // Head for the staging point; if a detour leaves the vehicle misaligned it
      // settles there (backing off if it ended up closer) before crossing.
      // Updates are bounded to avoid repeatedly restarting OA.
      const bool changed = !sent_ || std::hypot(g.x-last_gate_x_, g.y-last_gate_y_) > .15 ||
                           std::abs(g.z-last_gate_z_) > .10;
      if (changed && (!sent_ || (node_->now()-last_publish_).seconds() >= 1.0)) publish(-stage_, *goals_);
    }
    if (crossing_ && ahead <= -1.0 && lateral <= .35 && vertical <= .30 && heading_error <= .20) {
      if (!settled_) settled_ = node_->now();
      if ((node_->now()-*settled_).seconds() >= 1.0) {
        RCLCPP_INFO(node_->get_logger(), "Gate crossing complete");
        return BT::NodeStatus::SUCCESS;
      }
    } else settled_.reset();
    return BT::NodeStatus::RUNNING;
  }
  void onHalted() override { stop(); }
 private:
  void accepted(double range) {
    last_accept_ = node_->now();
    last_range_ = range;
  }
  bool fresh(const geometry_msgs::msg::PoseStamped& p, double maximum_age) const {
    const auto& v = p.pose.position;
    const auto& q = p.pose.orientation;
    const double age = (node_->now()-rclcpp::Time(p.header.stamp)).seconds();
    return !p.header.frame_id.empty() && age >= 0 && age <= maximum_age &&
           std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z) &&
           std::isfinite(q.x+q.y+q.z+q.w) &&
           std::abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.0) < .01;
  }
  void publish(double offset, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>& topic) {
    auto goal = *gate_;
    goal.header.frame_id = "odom";
    goal.header.stamp = node_->now();
    goal.pose.position.x += offset*nx_;
    goal.pose.position.y += offset*ny_;
    const double yaw = std::atan2(ny_, nx_);
    goal.pose.orientation.x = goal.pose.orientation.y = 0;
    goal.pose.orientation.z = std::sin(yaw/2);
    goal.pose.orientation.w = std::cos(yaw/2);
    topic.publish(goal);
    last_publish_ = node_->now();
    last_gate_x_ = gate_->pose.position.x;
    last_gate_y_ = gate_->pose.position.y;
    last_gate_z_ = gate_->pose.position.z;
    sent_ = true;
  }
  void stop() {
    subscription_.reset();
    if (sent_) hold_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    sent_ = false;
  }
  BT::NodeStatus fail(const char* reason) {
    RCLCPP_WARN(node_->get_logger(), "Gate traversal failed: %s", reason);
    stop();
    return BT::NodeStatus::FAILURE;
  }
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goals_, direct_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr hold_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr feedback_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr subscription_;
  std::optional<geometry_msgs::msg::PoseStamped> pose_, observation_, gate_;
  std::optional<rclcpp::Time> settled_, last_accept_;
  double last_range_ = 0;
  rclcpp::Time start_, last_publish_;
  double timeout_, through_, stage_, nx_, ny_, last_gate_x_, last_gate_y_, last_gate_z_;
  int samples_ = 0;
  bool sent_ = false, crossing_ = false, geometry_latched_ = false;
};
