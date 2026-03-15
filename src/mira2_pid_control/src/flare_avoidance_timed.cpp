/*
 * gate_flare_timed.cpp
 *
 * Timed gate + flare avoidance routine for AUV.
 * Depth is controlled by sinking for a fixed duration (thrust < 1500),
 * then locking ALT_HOLD using the disarm → MANUAL → rearm → disarm →
 * ALT_HOLD handshake (exact same pattern as VideoSubmissionController).
 * No PID controllers are used.
 *
 * Full stage sequence (all durations configurable in params yaml):
 *
 *   0  initial_wait           disarmed, ALT_HOLD, all neutral
 *   1  sinking                armed, ALT_HOLD, thrust < 1500 → descend
 *   2  althold_disarm         disarm  (mode switch requires disarmed)
 *   3  althold_to_manual      switch mode → MANUAL  (needed as intermediate)
 *   4  althold_manual_arm     arm in MANUAL
 *   5  althold_manual_disarm  disarm again
 *   6  althold_set            switch mode → ALT_HOLD  (depth now locked)
 *   7  forward_1              armed, ALT_HOLD, approach gate
 *   8  strafe_left            armed, ALT_HOLD, lateral < 1500, dodge flare
 *   9  forward_2              armed, ALT_HOLD, pass through gate
 *   10 strafe_right           armed, ALT_HOLD, lateral > 1500, re-centre
 *   11 disarm                 disarmed, done
 *
 * Arm/disarm : publish 'p' / 'q' to /keys
 * Restart    : ros2 service call /gate_flare_timed/restart std_srvs/srv/Trigger
 */

#include <custom_msgs/msg/commands.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/char.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <memory>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// Stage descriptor — identical layout to VideoSubmissionController
// ---------------------------------------------------------------------------
struct Stage {
  std::string name;
  float       duration;
  bool        arm;
  std::string mode;
  int         forward;
  int         lateral;
  int         thrust;
  int         yaw;
  int         roll;
  int         pitch;
};

// ---------------------------------------------------------------------------
// Controller
// ---------------------------------------------------------------------------
class GateFlareTimed {
public:
  explicit GateFlareTimed(rclcpp::Node::SharedPtr node)
      : node_(node)
  {
    declareStageParameters();
    loadStagesFromParameters();
    computeCumulativeTimes();

    pwm_pub_ = node_->create_publisher<custom_msgs::msg::Commands>("/master/commands", 10);

    keys_sub_ = node_->create_subscription<std_msgs::msg::Char>(
        "keys", 10,
        std::bind(&GateFlareTimed::keysCallback, this, std::placeholders::_1));

    restart_srv_ = node_->create_service<std_srvs::srv::Trigger>(
        "restart",
        std::bind(&GateFlareTimed::restartCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    param_cb_ = node_->add_on_set_parameters_callback(
        std::bind(&GateFlareTimed::parametersCallback, this, std::placeholders::_1));

    start_time_ = node_->now();

    // 10 Hz control loop — same as VideoSubmissionController
    timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&GateFlareTimed::controlLoop, this));

    RCLCPP_INFO(node_->get_logger(),
                "[GateFlareTimed] Ready — %zu stages loaded. "
                "Publish 'p' to /keys to start.", stages_.size());
    printStageInfo();
  }

private:
  // -------------------------------------------------------------------------
  // Parameter helpers
  // -------------------------------------------------------------------------
  void declareStageParameters()
  {
    node_->declare_parameter("num_stages", 12);
    const int n = node_->get_parameter("num_stages").as_int();

    for (int i = 0; i < n; ++i) {
      const std::string p = "stage_" + std::to_string(i) + "_";
      node_->declare_parameter(p + "name",     "stage_" + std::to_string(i));
      node_->declare_parameter(p + "duration", 0.0);
      node_->declare_parameter(p + "arm",      false);
      node_->declare_parameter(p + "mode",     std::string("ALT_HOLD"));
      node_->declare_parameter(p + "forward",  1500);
      node_->declare_parameter(p + "lateral",  1500);
      node_->declare_parameter(p + "thrust",   1500);
      node_->declare_parameter(p + "yaw",      1500);
      node_->declare_parameter(p + "roll",     1500);
      node_->declare_parameter(p + "pitch",    1500);
    }
  }

  void loadStagesFromParameters()
  {
    const int n = node_->get_parameter("num_stages").as_int();
    stages_.clear();
    stages_.reserve(n);

    for (int i = 0; i < n; ++i) {
      const std::string p = "stage_" + std::to_string(i) + "_";
      Stage s;
      s.name     = node_->get_parameter(p + "name").as_string();
      s.duration = static_cast<float>(node_->get_parameter(p + "duration").as_double());
      s.arm      = node_->get_parameter(p + "arm").as_bool();
      s.mode     = node_->get_parameter(p + "mode").as_string();
      s.forward  = node_->get_parameter(p + "forward").as_int();
      s.lateral  = node_->get_parameter(p + "lateral").as_int();
      s.thrust   = node_->get_parameter(p + "thrust").as_int();
      s.yaw      = node_->get_parameter(p + "yaw").as_int();
      s.roll     = node_->get_parameter(p + "roll").as_int();
      s.pitch    = node_->get_parameter(p + "pitch").as_int();
      stages_.push_back(s);
    }
  }

  void computeCumulativeTimes()
  {
    cumulative_times_.clear();
    float sum = 0.0f;
    for (const auto &s : stages_) {
      sum += s.duration;
      cumulative_times_.push_back(sum);
    }
  }

  void printStageInfo()
  {
    float total = 0.0f;
    for (size_t i = 0; i < stages_.size(); ++i) {
      const auto &s = stages_[i];
      RCLCPP_INFO(node_->get_logger(),
                  "  [%2zu] %-26s  %.2fs  arm=%-5s  mode=%-8s  "
                  "fwd=%d  lat=%d  thr=%d",
                  i, s.name.c_str(), s.duration,
                  s.arm ? "true" : "false", s.mode.c_str(),
                  s.forward, s.lateral, s.thrust);
      total += s.duration;
    }
    RCLCPP_INFO(node_->get_logger(), "  Total time: %.2fs", total);
  }

  // -------------------------------------------------------------------------
  // Callbacks
  // -------------------------------------------------------------------------
  void keysCallback(const std_msgs::msg::Char::SharedPtr msg)
  {
    if (msg->data == 'p') {
      armed_      = true;
      start_time_ = node_->now();
      RCLCPP_INFO(node_->get_logger(), "[GateFlareTimed] ARMED — sequence started");
    } else if (msg->data == 'q') {
      armed_ = false;
      RCLCPP_INFO(node_->get_logger(), "[GateFlareTimed] DISARMED");
    }
  }

  rcl_interfaces::msg::SetParametersResult parametersCallback(
      const std::vector<rclcpp::Parameter> &params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    bool reload = false;
    for (const auto &p : params) {
      if (p.get_name().find("stage_") == 0 || p.get_name() == "num_stages")
        reload = true;
    }
    if (reload) {
      loadStagesFromParameters();
      computeCumulativeTimes();
      printStageInfo();
      RCLCPP_INFO(node_->get_logger(), "[GateFlareTimed] Stages reloaded");
    }
    return result;
  }

  void restartCallback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    armed_      = false;
    start_time_ = node_->now();
    loadStagesFromParameters();
    computeCumulativeTimes();
    res->success = true;
    res->message = "Sequence reset";
    RCLCPP_INFO(node_->get_logger(), "[GateFlareTimed] Sequence reset via service");
  }

  // -------------------------------------------------------------------------
  // Control loop @ 10 Hz
  // -------------------------------------------------------------------------
  void controlLoop()
  {
    custom_msgs::msg::Commands cmd;
    cmd.servo1 = cmd.servo2 = 1500;

    if (!armed_) {
      cmd.arm     = false;
      cmd.mode    = "ALT_HOLD";
      cmd.forward = cmd.lateral = cmd.thrust =
      cmd.yaw     = cmd.roll    = cmd.pitch  = 1500;
      pwm_pub_->publish(cmd);
      return;
    }

    const float elapsed =
        static_cast<float>((node_->now() - start_time_).seconds());

    // Find current stage (identical logic to VideoSubmissionController)
    int stage_idx = -1;
    for (size_t i = 0; i < cumulative_times_.size(); ++i) {
      if (elapsed < cumulative_times_[i]) {
        stage_idx = static_cast<int>(i);
        break;
      }
    }

    if (stage_idx >= 0 && stage_idx < static_cast<int>(stages_.size())) {
      const Stage &s = stages_[stage_idx];

      cmd.arm     = s.arm;
      cmd.mode    = s.mode;
      cmd.forward = s.forward;
      cmd.lateral = s.lateral;
      cmd.thrust  = s.thrust;
      cmd.yaw     = s.yaw;
      cmd.roll    = s.roll;
      cmd.pitch   = s.pitch;

      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "[%2d | %-26s]  t=%.1fs  arm=%-5s  "
                           "mode=%-8s  fwd=%d  lat=%d  thr=%d",
                           stage_idx, s.name.c_str(), elapsed,
                           s.arm ? "true" : "false", s.mode.c_str(),
                           cmd.forward, cmd.lateral, cmd.thrust);
    } else {
      // Sequence complete — safe idle
      cmd.arm     = false;
      cmd.mode    = "ALT_HOLD";
      cmd.forward = cmd.lateral = cmd.thrust =
      cmd.yaw     = cmd.roll    = cmd.pitch  = 1500;

      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                           "[GateFlareTimed] Complete (%.1fs)", elapsed);
    }

    pwm_pub_->publish(cmd);
  }

  // -------------------------------------------------------------------------
  // Members
  // -------------------------------------------------------------------------
  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<custom_msgs::msg::Commands>::SharedPtr         pwm_pub_;
  rclcpp::Subscription<std_msgs::msg::Char>::SharedPtr             keys_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr               restart_srv_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
  rclcpp::TimerBase::SharedPtr                                     timer_;

  bool         armed_     = false;
  rclcpp::Time start_time_;

  std::vector<Stage> stages_;
  std::vector<float> cumulative_times_;
};

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node       = rclcpp::Node::make_shared("gate_flare_timed");
  auto controller = std::make_shared<GateFlareTimed>(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}