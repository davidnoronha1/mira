#include "custom_msgs/msg/commands.hpp"
#include <custom_msgs/msg/commands.hpp>
#include <custom_msgs/msg/telemetry.hpp>
#include <control_utils/control_utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/char.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <vector> // <<< added for durations

// Control parameters and PWM Commands
bool software_arm_flag = false;
custom_msgs::msg::Commands cmd_pwm;
PID_Controller depth;
double depth_error;
rclcpp::Time start_routine;

/* Keys callback
  Function for tuning the PID parameters
*/
void keys_callback(const std_msgs::msg::Char::SharedPtr msg) {
  char key = msg->data;
  if (key == 'q') {
    software_arm_flag = false;
    std::cout << "unarmed\n";
    start_routine = rclcpp::Clock().now();
    depth.emptyError();
  } else if (key == 'p') {
    software_arm_flag = true;
    std::cout << "armed\n";
    start_routine = rclcpp::Clock().now();
  }
}

void telemetryCallback(const custom_msgs::msg::Telemetry::SharedPtr msg) {
  double depth_external = msg->external_pressure;
  depth_error = 1075 - depth_external;
}

int main(int argc, char **argv) {
  // ROS 2 Node Declaration
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("depth_tuner_controller");

  // ROS 2 Publisher
  auto pwm_publisher = node->create_publisher<custom_msgs::msg::Commands>(
      "/master/commands", 10);

  // ROS 2 Subscriber
  auto keys_subscriber =
      node->create_subscription<std_msgs::msg::Char>("keys", 10, keys_callback);

  auto telemetry_sub = node->create_subscription<custom_msgs::msg::Telemetry>(
      "/master/telemetry", 10, telemetryCallback);

  // Depth
  depth.kp = -2;     // -2;
  depth.ki = -0.2;   // -0.2;
  depth.kd = -10.69; // -15.69;

  RCLCPP_INFO(node->get_logger(), "lilbitchlaky");

  // Arm Disarm Parameter
  bool arm = false;
  rclcpp::Time init_time = rclcpp::Clock().now();
  cmd_pwm.arm = false;

  // ----- EDIT ONLY THESE DURATIONS (seconds) TO CHANGE TIMINGS -----
  // First element is the initial delay that was previously in your code.
  // The rest correspond, in order, to the original else-if blocks.
  std::vector<float> stage_durations = {
      3.0f,   // initial wait (was delay)
      2.00f,  // sinking
      6.00f,  // stabilize surge (was 4.00 from 2.80->6.80)
      2.80f,  // stabilize yaw (was 2.20)
      2.60f,  // stabilize surge 2
      1.00f,  // delay
      0.05f,  // disarm (brief)
      0.05f,  // change to MANUAL
      1.80f,  // manual surge with roll
      0.05f,  // disarm
      1.00f,  // change to ALT_HOLD (very brief)
      3.50f,  // forward
      3.40f   // wait
      // any trailing time falls into 'else' that keeps neutral as before
  };
  // -----------------------------------------------------------------

  // Pre-compute cumulative sums so we can compare elapsed time against thresholds
  std::vector<float> cumulative;
  cumulative.reserve(stage_durations.size());
  float sum = 0.0f;
  for (float d : stage_durations) {
    sum += d;
    cumulative.push_back(sum);
  }

  rclcpp::Rate rate(10); // 10 Hz loop rate
  while (rclcpp::ok()) {
    if (software_arm_flag == true) {
      cmd_pwm.mode = "ALT_HOLD";
      rclcpp::Time time_now = rclcpp::Clock().now();
      cmd_pwm.arm = false;
      if (software_arm_flag == true) {
        float pid_depth = depth.pid_control(
            depth_error, (time_now - init_time).seconds(), false);
        std::cout << ((time_now - start_routine).seconds()) << "\n";

        // compute elapsed since start of routine
        float elapsed = (time_now - start_routine).seconds();

        // find which stage we're in by comparing elapsed with cumulative thresholds
        int stage = -1;
        for (size_t i = 0; i < cumulative.size(); ++i) {
          if (elapsed < cumulative[i]) {
            stage = static_cast<int>(i);
            break;
          }
        }

        // If stage == -1, we've passed all defined stages and fall into final else
        if (stage == 0) {
          // initial waiting stage (was delay)
          cmd_pwm.arm = false;
          cmd_pwm.forward = 1500;
          cmd_pwm.mode = "ALT_HOLD";
          cmd_pwm.lateral = 1500;
          cmd_pwm.thrust = 1500;
          cmd_pwm.yaw = 1500;
          depth.emptyError();
        } else if (stage == 1) {
          // sinking (previously thrust 1400 in your later variant)
          cmd_pwm.arm = true;
          cmd_pwm.forward = 1500;
          cmd_pwm.lateral = 1500;
          cmd_pwm.thrust = 1350;
          cmd_pwm.yaw = 1500;
          std::cout << "sinking ";

        } else if (stage == 2) {
          // stabilize surge
          cmd_pwm.arm = true;
          cmd_pwm.forward = 1705;
          cmd_pwm.lateral = 1500;
          cmd_pwm.thrust = 1475;
          cmd_pwm.yaw = 1475;
          std::cout << "stabilize surge ";
        } else if (stage == 3) {
          // stabilize yaw
          cmd_pwm.arm = true;
          cmd_pwm.forward = 1500;
          cmd_pwm.lateral = 1500;
          cmd_pwm.thrust = 1500;
          cmd_pwm.yaw = 1675;
          std::cout << "stabilize yaw ";
        } else if (stage == 4) {
          // stabilize surge 2
          cmd_pwm.arm = true;
          cmd_pwm.forward = 1650;
          cmd_pwm.lateral = 1500;
          cmd_pwm.thrust = 1550;
          cmd_pwm.yaw = 1475;
          std::cout << "stabilize surge ";
        }else if (stage == 5) {
          // delay
          cmd_pwm.arm = true;
          cmd_pwm.forward = 1500;
          cmd_pwm.lateral = 1500;
          cmd_pwm.thrust = 1550;
          cmd_pwm.yaw = 1500;
          std::cout << "stablize the pitch with alt hold ";
        } else if (stage == 6) {
          // brief disarm (keeps ALT_HOLD mode, cmd shows disarm)
          cmd_pwm.arm = false;
          cmd_pwm.mode = "ALT_HOLD";
          cmd_pwm.forward = 1500;
          cmd_pwm.lateral = 1500;
          cmd_pwm.thrust = 1500;
          cmd_pwm.yaw = 1500;
          std::cout << "disarm";
        } else if (stage == 7) {
          // change to MANUAL (disarmed)
          cmd_pwm.arm = false;
          cmd_pwm.mode = "MANUAL";
          cmd_pwm.forward = 1500;
          cmd_pwm.lateral = 1500;
          cmd_pwm.thrust = 1500;
          cmd_pwm.yaw = 1500;
          std::cout << "change to manual";
        } else if (stage == 8) {
          // manual surge with roll (armed)
          cmd_pwm.arm = true;
          cmd_pwm.mode = "MANUAL";
          cmd_pwm.forward = 1650;
          cmd_pwm.lateral = 1500;
          cmd_pwm.thrust = 1470;
          cmd_pwm.yaw = 1500;
          cmd_pwm.roll = 1700;
          std::cout << " surge with roll  and manual";
        } else if (stage == 9) {
          // disarm
          cmd_pwm.arm = false;
          cmd_pwm.mode = "MANUAL";
          cmd_pwm.forward = 1500;
          cmd_pwm.lateral = 1500;
          cmd_pwm.thrust = 1500;
          cmd_pwm.yaw = 1500;
          std::cout << "disarm";
        } else if (stage == 10) {
          // change to ALT_HOLD (brief)
          cmd_pwm.arm = false;
          cmd_pwm.mode = "ALT_HOLD";
          cmd_pwm.forward = 1500;
          cmd_pwm.lateral = 1500;
          cmd_pwm.thrust = 1420;
          cmd_pwm.yaw = 1500;
          std::cout << "change to stabilize";
        } else if (stage == 11) {
          // ALT_HOLD forward (armed)
          cmd_pwm.arm = true;
          cmd_pwm.mode = "ALT_HOLD";
          cmd_pwm.forward = 1520;
          cmd_pwm.lateral = 1500;
          cmd_pwm.thrust = 1600;
          cmd_pwm.yaw = 1500;
          cmd_pwm.roll = 1500;
          std::cout << "forward";
        } else if (stage == 12) {
          // wait
          cmd_pwm.arm = false;
          cmd_pwm.forward = 1500;
          cmd_pwm.lateral = 1500;
          cmd_pwm.thrust = 1500;
          cmd_pwm.yaw = 1500;
          std::cout << "wait";
        } else {
          // fallback: final else from your original code
          cmd_pwm.arm = false;
          cmd_pwm.forward = 1500;
          cmd_pwm.lateral = 1500;
          cmd_pwm.thrust = 1500;
          cmd_pwm.yaw = 1500;
          std::cout << "pusi";
        }
      }
    } else {
      depth.emptyError();
      cmd_pwm.arm = false;
      cmd_pwm.mode = "ALT_HOLD";
      cmd_pwm.forward = 1500;
      cmd_pwm.lateral = 1500;
      cmd_pwm.thrust = 1500;
      cmd_pwm.yaw = 1500;
    }
    pwm_publisher->publish(cmd_pwm);
    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
