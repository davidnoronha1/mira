/*
 * gate_flare_imu.cpp
 *
 * IMU-integrated gate + flare avoidance routine for AUV.
 *
 * ── Depth strategy ───────────────────────────────────────────────────────────
 *   1. Sink in MANUAL mode (barometer does not fight the descent).
 *   2. Disarm → switch to ALT_HOLD (disarmed) → rearm.
 *      The Pixhawk latches its barometer at the moment of re-arm in ALT_HOLD,
 *      holding that depth for the rest of the run.
 *   Only 3 mode-switch stages are needed — no unnecessary cycling.
 *
 * ── Manoeuvre ────────────────────────────────────────────────────────────────
 *   The flare is between the start line and the gate frame, in the centre.
 *
 *      [start]──forward_1──[flare]
 *                    ↓ strafe_left   (dodge left, past flare)
 *                    ↓ strafe_right  (come back right, now aligned with gate)
 *                    ↓ forward_2     (straight through the gate)
 *
 * ── Phase table ──────────────────────────────────────────────────────────────
 *   0  initial_wait   – disarmed MANUAL, stationary (IMU bias calibration)
 *   1  sinking        – armed MANUAL,  thrust < 1500 → descend
 *   2  sink_disarm    – disarm  (mode switch requires disarmed state)
 *   3  althold_set    – switch mode → ALT_HOLD while disarmed
 *   4  althold_arm    – rearm in ALT_HOLD → barometer latches at current depth
 *   5  forward_1      – IMU forward until forward_dist_1 m
 *   6  strafe_left    – IMU lateral until lateral_dist m    (dodge flare)
 *   7  strafe_right   – IMU lateral until lateral_dist m    (realign with gate)
 *   8  forward_2      – IMU forward until forward_dist_2 m  (through gate)
 *   9  disarm         – done
 *
 * Subscribes to /master/imu_ned (sensor_msgs/msg/Imu, published by
 * pymav_master.py). Double-integrates linear_acceleration X (forward) and
 * Y (lateral). Bias is sampled during Phase 0 (stationary).
 *
 * Arm/disarm : publish 'p' / 'q' to /keys
 * Restart    : ros2 service call /gate_flare_imu/restart std_srvs/srv/Trigger
 */

#include <custom_msgs/msg/commands.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/char.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// Phase types
// ---------------------------------------------------------------------------
enum class PhaseType {
  TIMED,        // advances after target seconds
  IMU_FORWARD,  // advances when |forward distance| >= target metres
  IMU_LATERAL,  // advances when |lateral distance| >= target metres
};

struct Phase {
  std::string name;
  PhaseType   type;
  double      target;   // seconds (TIMED) or metres (IMU)
  double      timeout;  // safety fallback in seconds (0 = no timeout for TIMED)
  bool        arm;
  std::string mode;
  int         forward_pwm;
  int         lateral_pwm;
  int         thrust_pwm;
};

// ---------------------------------------------------------------------------
// Controller
// ---------------------------------------------------------------------------
class GateFlareImu {
public:
  explicit GateFlareImu(rclcpp::Node::SharedPtr node)
      : node_(node)
  {
    declareParameters();
    loadPhasesFromParameters();

    pwm_pub_ = node_->create_publisher<custom_msgs::msg::Commands>("/master/commands", 10);

    keys_sub_ = node_->create_subscription<std_msgs::msg::Char>(
        "keys", 10,
        std::bind(&GateFlareImu::keysCallback, this, std::placeholders::_1));

    imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
        "/master/imu_ned", 10,
        std::bind(&GateFlareImu::imuCallback, this, std::placeholders::_1));

    restart_srv_ = node_->create_service<std_srvs::srv::Trigger>(
        "restart",
        std::bind(&GateFlareImu::restartCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    param_cb_ = node_->add_on_set_parameters_callback(
        std::bind(&GateFlareImu::parametersCallback, this, std::placeholders::_1));

    resetState();

    // 50 Hz for tighter IMU integration
    timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&GateFlareImu::controlLoop, this));

    RCLCPP_INFO(node_->get_logger(),
                "[GateFlareImu] Ready — %zu phases loaded. "
                "Publish 'p' to /keys to start.", phases_.size());
    printPhaseInfo();
  }

private:
  // -------------------------------------------------------------------------
  // Parameters
  // -------------------------------------------------------------------------
  void declareParameters()
  {
    // IMU tuning
    node_->declare_parameter("imu_accel_deadband",   0.05);   // m/s²
    node_->declare_parameter("velocity_decay",       0.98);   // per-tick bleed
    node_->declare_parameter("bias_sample_duration", 5.0);    // seconds = initial_wait

    // Depth — open-loop in MANUAL
    node_->declare_parameter("sink_duration",        2.3);    // seconds
    node_->declare_parameter("sink_thrust",          1350);   // PWM <1500 = sink

    // Mode-switch timing (increase only if Pixhawk is slow to respond)
    node_->declare_parameter("sink_disarm_duration", 0.1);    // pause before mode switch
    node_->declare_parameter("althold_set_duration", 0.1);    // disarmed ALT_HOLD settle
    node_->declare_parameter("althold_arm_duration", 0.05);   // arm settling time

    // Forward motion
    node_->declare_parameter("forward_dist_1",       1.0);    // metres — approach
    node_->declare_parameter("forward_dist_2",       1.0);    // metres — through gate
    node_->declare_parameter("forward_1_timeout",   15.0);    // seconds
    node_->declare_parameter("forward_2_timeout",   15.0);    // seconds
    node_->declare_parameter("pwm_forward",          1600);   // >1500

    // Lateral motion
    node_->declare_parameter("lateral_dist",         0.4);    // metres (same for left and right)
    node_->declare_parameter("strafe_left_timeout",  8.0);    // seconds
    node_->declare_parameter("strafe_right_timeout", 8.0);    // seconds
    node_->declare_parameter("pwm_lateral_l",        1380);   // <1500 = left
    node_->declare_parameter("pwm_lateral_r",        1620);   // >1500 = right

    node_->declare_parameter("pwm_neutral",          1500);

    // Initial wait
    node_->declare_parameter("initial_wait_duration", 5.0);
  }

  void loadPhasesFromParameters()
  {
    // IMU tuning
    accel_deadband_  = node_->get_parameter("imu_accel_deadband").as_double();
    velocity_decay_  = node_->get_parameter("velocity_decay").as_double();
    bias_sample_dur_ = node_->get_parameter("bias_sample_duration").as_double();

    // Read all params
    const double init_wait  = node_->get_parameter("initial_wait_duration").as_double();
    const double sink_dur   = node_->get_parameter("sink_duration").as_double();
    const int    sink_thr   = node_->get_parameter("sink_thrust").as_int();
    const double dis_dur    = node_->get_parameter("sink_disarm_duration").as_double();
    const double ah_set_dur = node_->get_parameter("althold_set_duration").as_double();
    const double ah_arm_dur = node_->get_parameter("althold_arm_duration").as_double();
    const double fd1        = node_->get_parameter("forward_dist_1").as_double();
    const double fd2        = node_->get_parameter("forward_dist_2").as_double();
    const double t_f1       = node_->get_parameter("forward_1_timeout").as_double();
    const double t_f2       = node_->get_parameter("forward_2_timeout").as_double();
    const int    pwm_fwd    = node_->get_parameter("pwm_forward").as_int();
    const double ld         = node_->get_parameter("lateral_dist").as_double();
    const double t_sl       = node_->get_parameter("strafe_left_timeout").as_double();
    const double t_sr       = node_->get_parameter("strafe_right_timeout").as_double();
    const int    pwm_ll     = node_->get_parameter("pwm_lateral_l").as_int();
    const int    pwm_lr     = node_->get_parameter("pwm_lateral_r").as_int();
    const int    n          = node_->get_parameter("pwm_neutral").as_int();

    // Phase table
    //               name               type                 target     timeout  arm    mode         fwd     lat     thr
    phases_ = {
      // ── Phase 0 : stationary hold — also IMU bias calibration window ──
      { "initial_wait",  PhaseType::TIMED,       init_wait,  0,      false, "MANUAL",   n,       n,      n       },

      // ── Phase 1 : sink in MANUAL (barometer does not resist) ──────────
      { "sinking",       PhaseType::TIMED,       sink_dur,   0,      true,  "MANUAL",   n,       n,      sink_thr},

      // ── Phases 2-4 : depth lock  (disarm → ALT_HOLD → rearm) ─────────
      // Pixhawk latches barometer reading the moment it arms in ALT_HOLD.
      { "sink_disarm",   PhaseType::TIMED,       dis_dur,    0,      false, "MANUAL",   n,       n,      n       },
      { "althold_set",   PhaseType::TIMED,       ah_set_dur, 0,      false, "ALT_HOLD", n,       n,      n       },
      { "althold_arm",   PhaseType::TIMED,       ah_arm_dur, 0,      true,  "ALT_HOLD", n,       n,      n       },

      // ── Phases 5-8 : gate + flare manoeuvre ───────────────────────────
      { "forward_1",     PhaseType::IMU_FORWARD, fd1,        t_f1,   true,  "ALT_HOLD", pwm_fwd, n,      n       },
      { "strafe_left",   PhaseType::IMU_LATERAL, ld,         t_sl,   true,  "ALT_HOLD", n,       pwm_ll, n       },
      { "strafe_right",  PhaseType::IMU_LATERAL, ld,         t_sr,   true,  "ALT_HOLD", n,       pwm_lr, n       },
      { "forward_2",     PhaseType::IMU_FORWARD, fd2,        t_f2,   true,  "ALT_HOLD", pwm_fwd, n,      n       },

      // ── Phase 9 : done ────────────────────────────────────────────────
      { "disarm",        PhaseType::TIMED,       0.1,        0,      false, "ALT_HOLD", n,       n,      n       },
    };
  }

  void printPhaseInfo()
  {
    RCLCPP_INFO(node_->get_logger(), "Phase table:");
    for (size_t i = 0; i < phases_.size(); ++i) {
      const auto &p = phases_[i];
      const char *tname = (p.type == PhaseType::TIMED)       ? "TIMED  " :
                          (p.type == PhaseType::IMU_FORWARD)  ? "IMU_FWD" : "IMU_LAT";
      RCLCPP_INFO(node_->get_logger(),
                  "  [%zu] %-14s  %s  target=%.2f  to=%.1f  "
                  "arm=%-5s  mode=%-8s  fwd=%d  lat=%d  thr=%d",
                  i, p.name.c_str(), tname, p.target, p.timeout,
                  p.arm ? "true" : "false", p.mode.c_str(),
                  p.forward_pwm, p.lateral_pwm, p.thrust_pwm);
    }
  }

  // -------------------------------------------------------------------------
  // State
  // -------------------------------------------------------------------------
  void resetState()
  {
    armed_          = false;
    current_phase_  = 0;
    fwd_vel_        = lat_vel_       = 0.0;
    fwd_dist_       = lat_dist_      = 0.0;
    bias_fwd_       = bias_lat_      = 0.0;
    bias_sum_fwd_   = bias_sum_lat_  = 0.0;
    bias_count_     = 0;
    bias_ready_     = false;
    last_imu_time_  = rclcpp::Time(0, 0, RCL_ROS_TIME);
    phase_start_    = node_->now();
  }

  void resetPhaseDistances()
  {
    fwd_vel_ = lat_vel_ = fwd_dist_ = lat_dist_ = 0.0;
  }

  // -------------------------------------------------------------------------
  // Callbacks
  // -------------------------------------------------------------------------
  void keysCallback(const std_msgs::msg::Char::SharedPtr msg)
  {
    if (msg->data == 'p') {
      armed_         = true;
      current_phase_ = 0;
      phase_start_   = node_->now();
      resetPhaseDistances();
      bias_sum_fwd_ = bias_sum_lat_ = 0.0;
      bias_count_   = 0;
      bias_ready_   = false;
      RCLCPP_INFO(node_->get_logger(), "[GateFlareImu] ARMED — sequence started");
    } else if (msg->data == 'q') {
      armed_ = false;
      RCLCPP_INFO(node_->get_logger(), "[GateFlareImu] DISARMED");
    }
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    const rclcpp::Time now = node_->now();

    if (last_imu_time_.nanoseconds() == 0) {
      last_imu_time_ = now;
      return;
    }
    const double dt = (now - last_imu_time_).seconds();
    last_imu_time_  = now;
    if (dt <= 0.0 || dt > 0.5) return;

    double ax = msg->linear_acceleration.x;  // forward in body frame
    double ay = msg->linear_acceleration.y;  // lateral in body frame

    // ── Phase 0: accumulate bias while stationary ─────────────────────────
    if (armed_ && current_phase_ == 0) {
      bias_sum_fwd_ += ax;
      bias_sum_lat_ += ay;
      ++bias_count_;
      bias_fwd_ = bias_sum_fwd_ / bias_count_;
      bias_lat_ = bias_sum_lat_ / bias_count_;
      return;  // don't integrate during calibration
    }

    if (!bias_ready_) return;

    // ── Subtract bias + deadband ──────────────────────────────────────────
    ax -= bias_fwd_;
    ay -= bias_lat_;
    if (std::fabs(ax) < accel_deadband_) ax = 0.0;
    if (std::fabs(ay) < accel_deadband_) ay = 0.0;

    // ── Only integrate during IMU-gated motion phases ─────────────────────
    if (current_phase_ < static_cast<int>(phases_.size())) {
      const PhaseType t = phases_[current_phase_].type;
      if (t == PhaseType::IMU_FORWARD || t == PhaseType::IMU_LATERAL) {
        fwd_vel_  = fwd_vel_ * velocity_decay_ + ax * dt;
        lat_vel_  = lat_vel_ * velocity_decay_ + ay * dt;
        fwd_dist_ += std::fabs(fwd_vel_ * dt);
        lat_dist_ += std::fabs(lat_vel_ * dt);
      }
    }
  }

  rcl_interfaces::msg::SetParametersResult parametersCallback(
      const std::vector<rclcpp::Parameter> &)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    loadPhasesFromParameters();
    printPhaseInfo();
    RCLCPP_INFO(node_->get_logger(), "[GateFlareImu] Phases reloaded");
    return result;
  }

  void restartCallback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    resetState();
    loadPhasesFromParameters();
    res->success = true;
    res->message = "Sequence reset";
    RCLCPP_INFO(node_->get_logger(), "[GateFlareImu] Sequence reset via service");
  }

  // -------------------------------------------------------------------------
  // Control loop @ 50 Hz
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

    if (current_phase_ >= static_cast<int>(phases_.size())) {
      cmd.arm     = false;
      cmd.mode    = "ALT_HOLD";
      cmd.forward = cmd.lateral = cmd.thrust =
      cmd.yaw     = cmd.roll    = cmd.pitch  = 1500;
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                           "[GateFlareImu] Sequence complete");
      pwm_pub_->publish(cmd);
      return;
    }

    const rclcpp::Time now     = node_->now();
    const double       elapsed = (now - phase_start_).seconds();
    const Phase       &ph      = phases_[current_phase_];

    cmd.arm     = ph.arm;
    cmd.mode    = ph.mode;
    cmd.forward = ph.forward_pwm;
    cmd.lateral = ph.lateral_pwm;
    cmd.thrust  = ph.thrust_pwm;
    cmd.yaw     = 1500;
    cmd.roll    = 1500;
    cmd.pitch   = 1500;

    bool advance = false;

    switch (ph.type) {
      case PhaseType::TIMED:
        if (elapsed >= ph.target) {
          if (current_phase_ == 0) {
            // Phase 0 done — bias is calibrated
            bias_ready_ = true;
            RCLCPP_INFO(node_->get_logger(),
                        "[GateFlareImu] Bias calibrated — fwd=%.4f  lat=%.4f m/s²",
                        bias_fwd_, bias_lat_);
          }
          advance = true;
        }
        break;

      case PhaseType::IMU_FORWARD:
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
                             "[%d | %s]  fwd=%.3fm / %.3fm  t=%.1fs",
                             current_phase_, ph.name.c_str(),
                             fwd_dist_, ph.target, elapsed);
        if (fwd_dist_ >= ph.target) {
          RCLCPP_INFO(node_->get_logger(),
                      "[GateFlareImu] '%s' complete — %.3fm in %.1fs",
                      ph.name.c_str(), fwd_dist_, elapsed);
          advance = true;
        } else if (elapsed >= ph.timeout) {
          RCLCPP_WARN(node_->get_logger(),
                      "[GateFlareImu] '%s' TIMEOUT — %.3fm / %.3fm reached",
                      ph.name.c_str(), fwd_dist_, ph.target);
          advance = true;
        }
        break;

      case PhaseType::IMU_LATERAL:
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
                             "[%d | %s]  lat=%.3fm / %.3fm  t=%.1fs",
                             current_phase_, ph.name.c_str(),
                             lat_dist_, ph.target, elapsed);
        if (lat_dist_ >= ph.target) {
          RCLCPP_INFO(node_->get_logger(),
                      "[GateFlareImu] '%s' complete — %.3fm in %.1fs",
                      ph.name.c_str(), lat_dist_, elapsed);
          advance = true;
        } else if (elapsed >= ph.timeout) {
          RCLCPP_WARN(node_->get_logger(),
                      "[GateFlareImu] '%s' TIMEOUT — %.3fm / %.3fm reached",
                      ph.name.c_str(), lat_dist_, ph.target);
          advance = true;
        }
        break;
    }

    if (advance) {
      ++current_phase_;
      phase_start_ = node_->now();
      resetPhaseDistances();
      if (current_phase_ < static_cast<int>(phases_.size())) {
        RCLCPP_INFO(node_->get_logger(), "[GateFlareImu] → Phase %d: %s",
                    current_phase_, phases_[current_phase_].name.c_str());
      }
    }

    pwm_pub_->publish(cmd);
  }

  // -------------------------------------------------------------------------
  // Members
  // -------------------------------------------------------------------------
  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<custom_msgs::msg::Commands>::SharedPtr          pwm_pub_;
  rclcpp::Subscription<std_msgs::msg::Char>::SharedPtr              keys_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr            imu_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr                restart_srv_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
  rclcpp::TimerBase::SharedPtr                                      timer_;

  bool         armed_         = false;
  int          current_phase_ = 0;
  rclcpp::Time phase_start_;

  double fwd_vel_  = 0.0, lat_vel_  = 0.0;
  double fwd_dist_ = 0.0, lat_dist_ = 0.0;

  double bias_fwd_ = 0.0, bias_lat_     = 0.0;
  double bias_sum_fwd_ = 0.0, bias_sum_lat_ = 0.0;
  int    bias_count_   = 0;
  bool   bias_ready_   = false;

  double accel_deadband_  = 0.05;
  double velocity_decay_  = 0.98;
  double bias_sample_dur_ = 5.0;

  rclcpp::Time       last_imu_time_;
  std::vector<Phase> phases_;
};

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node       = rclcpp::Node::make_shared("gate_flare_imu");
  auto controller = std::make_shared<GateFlareImu>(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}