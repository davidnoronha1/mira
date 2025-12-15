#include "custom_msgs/msg/commands.hpp"
#include <custom_msgs/msg/commands.hpp>
#include <custom_msgs/msg/telemetry.hpp>
#include <control_utils/control_utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/char.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <vector>
#include <string>
#include <tinyxml2.h>
#include <cstdlib>

// Control parameters and PWM Commands
bool software_arm_flag = false;
custom_msgs::msg::Commands cmd_pwm;
PID_Controller depth;
double depth_error;
rclcpp::Time start_routine;

struct Stage {
  float duration;
  std::string name;
  bool arm;
  std::string mode;
  int forward;
  int lateral;
  int thrust;
  int yaw;
  int roll;
};

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

std::vector<Stage> loadStagesFromXML(const std::string& filepath) {
  std::vector<Stage> stages;
  
  tinyxml2::XMLDocument doc;
  if (doc.LoadFile(filepath.c_str()) != tinyxml2::XML_SUCCESS) {
    std::cerr << "Error loading XML file: " << filepath << std::endl;
    return stages;
  }
  
  tinyxml2::XMLElement* root = doc.FirstChildElement("stages");
  if (!root) {
    std::cerr << "No 'stages' root element found" << std::endl;
    return stages;
  }
  
  for (tinyxml2::XMLElement* stageElem = root->FirstChildElement("stage");
       stageElem != nullptr;
       stageElem = stageElem->NextSiblingElement("stage")) {
    
    Stage stage;
    
    // Read attributes
    const char* name = stageElem->Attribute("name");
    if (name) {
      stage.name = name;
    }
    
    if (stageElem->QueryFloatAttribute("duration", &stage.duration) != tinyxml2::XML_SUCCESS) {
      continue;
    }
    
    // Read arm (default to false)
    stage.arm = false;
    stageElem->QueryBoolAttribute("arm", &stage.arm);
    
    // Read mode (default to ALT_HOLD)
    const char* mode = stageElem->Attribute("mode");
    stage.mode = mode ? mode : "ALT_HOLD";
    
    // Read PWM values (default to 1500)
    stage.forward = 1500;
    stage.lateral = 1500;
    stage.thrust = 1500;
    stage.yaw = 1500;
    stage.roll = 1500;
    
    stageElem->QueryIntAttribute("forward", &stage.forward);
    stageElem->QueryIntAttribute("lateral", &stage.lateral);
    stageElem->QueryIntAttribute("thrust", &stage.thrust);
    stageElem->QueryIntAttribute("yaw", &stage.yaw);
    stageElem->QueryIntAttribute("roll", &stage.roll);
    
    stages.push_back(stage);
  }
  
  return stages;
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
  depth.kp = -2;
  depth.ki = -0.2;
  depth.kd = -10.69;

  RCLCPP_INFO(node->get_logger(), "lilbitchlaky");

  // Load stages from XML
  const char* home = std::getenv("HOME");
  std::string xml_path = std::string(home) + "/video.xml";
  std::vector<Stage> stages = loadStagesFromXML(xml_path);
  
  if (stages.empty()) {
    RCLCPP_ERROR(node->get_logger(), "No stages loaded from XML! Using defaults.");
    // Fallback to hardcoded stages
    stages = {
      {5.0f, "initial_wait", false, "ALT_HOLD", 1500, 1500, 1500, 1500, 1500},
      {2.35f, "sinking", true, "ALT_HOLD", 1500, 1500, 1350, 1500, 1500},
      {6.00f, "stabilize_surge", true, "ALT_HOLD", 1700, 1500, 1450, 1475, 1500},
      {3.00f, "stabilize_yaw", true, "ALT_HOLD", 1500, 1500, 1550, 1675, 1500},
      {2.60f, "stabilize_surge_2", true, "ALT_HOLD", 1650, 1500, 1550, 1500, 1500},
      {1.00f, "delay", true, "ALT_HOLD", 1500, 1500, 1550, 1500, 1500},
      {0.05f, "disarm", false, "ALT_HOLD", 1500, 1500, 1500, 1500, 1500},
      {0.05f, "change_to_manual", false, "MANUAL", 1500, 1500, 1500, 1500, 1500},
      {1.80f, "manual_surge_with_roll", true, "MANUAL", 1650, 1500, 1450, 1500, 1700},
      {0.05f, "disarm_2", false, "MANUAL", 1500, 1500, 1500, 1500, 1500},
      {0.05f, "change_to_alt_hold", false, "ALT_HOLD", 1500, 1500, 1500, 1500, 1500},
      {3.50f, "forward", true, "ALT_HOLD", 1550, 1500, 1400, 1500, 1500},
      {3.40f, "wait", false, "ALT_HOLD", 1500, 1500, 1500, 1500, 1500}
    };
  } else {
    RCLCPP_INFO(node->get_logger(), "Loaded %zu stages from XML", stages.size());
  }

  // Pre-compute cumulative sums
  std::vector<float> cumulative;
  cumulative.reserve(stages.size());
  float sum = 0.0f;
  for (const Stage& stage : stages) {
    sum += stage.duration;
    cumulative.push_back(sum);
  }

  // Arm Disarm Parameter
  bool arm = false;
  rclcpp::Time init_time = rclcpp::Clock().now();
  cmd_pwm.arm = false;

  rclcpp::Rate rate(10); // 10 Hz loop rate
  while (rclcpp::ok()) {
    if (software_arm_flag == true) {
      rclcpp::Time time_now = rclcpp::Clock().now();
      
      float pid_depth = depth.pid_control(
          depth_error, (time_now - init_time).seconds(), false);
      
      float elapsed = (time_now - start_routine).seconds();
      std::cout << elapsed << " - ";

      // Find which stage we're in
      int stage_idx = -1;
      for (size_t i = 0; i < cumulative.size(); ++i) {
        if (elapsed < cumulative[i]) {
          stage_idx = static_cast<int>(i);
          break;
        }
      }

      if (stage_idx >= 0 && stage_idx < static_cast<int>(stages.size())) {
        const Stage& current_stage = stages[stage_idx];
        
        std::cout << current_stage.name;
        
        // Apply all settings from the stage
        cmd_pwm.arm = current_stage.arm;
        cmd_pwm.mode = current_stage.mode;
        cmd_pwm.forward = current_stage.forward;
        cmd_pwm.lateral = current_stage.lateral;
        cmd_pwm.thrust = current_stage.thrust;
        cmd_pwm.yaw = current_stage.yaw;
        cmd_pwm.roll = current_stage.roll;
        
        // Clear depth error for initial stage
        if (stage_idx == 0) {
          depth.emptyError();
        }
      } else {
        // Fallback - after all stages complete
        std::cout << "complete";
        cmd_pwm.arm = false;
        cmd_pwm.mode = "ALT_HOLD";
        cmd_pwm.forward = 1500;
        cmd_pwm.lateral = 1500;
        cmd_pwm.thrust = 1500;
        cmd_pwm.yaw = 1500;
        cmd_pwm.roll = 1500;
      }
      std::cout << std::endl;
    } else {
      depth.emptyError();
      cmd_pwm.arm = false;
      cmd_pwm.mode = "ALT_HOLD";
      cmd_pwm.forward = 1500;
      cmd_pwm.lateral = 1500;
      cmd_pwm.thrust = 1500;
      cmd_pwm.yaw = 1500;
      cmd_pwm.roll = 1500;
    }
    
    pwm_publisher->publish(cmd_pwm);
    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
