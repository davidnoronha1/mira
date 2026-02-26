#include <custom_msgs/msg/commands.hpp>
#include <custom_msgs/msg/telemetry.hpp>
#include <control_utils/control_utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/char.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <memory>
#include <vector>
#include <string>

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
  int pitch;
};

class VideoSubmissionController {
public:
  VideoSubmissionController(rclcpp::Node::SharedPtr node)
      : node_(node),
        depth_controller_("depth", node) {
    
    // Declare PID parameters
    node_->declare_parameter("depth_target", 1075.0);
    node_->declare_parameter("depth_pid_kp", -2.0);
    node_->declare_parameter("depth_pid_ki", -0.2);
    node_->declare_parameter("depth_pid_kd", -10.69);
    node_->declare_parameter("depth_pid_base_offset", 1500);
    
    // Declare stage parameters
    declareStageParameters();
    
    // Initialize PID controller
    initializePIDController();
    
    // Load stages from parameters
    loadStagesFromParameters();
    
    // Setup communications
    pwm_publisher_ = node_->create_publisher<custom_msgs::msg::Commands>(
        "/master/commands", 10);
    
    keys_subscriber_ = node_->create_subscription<std_msgs::msg::Char>(
        "keys", 10, std::bind(&VideoSubmissionController::keysCallback, this, std::placeholders::_1));
    
    telemetry_subscriber_ = node_->create_subscription<custom_msgs::msg::Telemetry>(
        "/master/telemetry", 10, std::bind(&VideoSubmissionController::telemetryCallback, this, std::placeholders::_1));
    
    // Setup restart service
    restart_service_ = node_->create_service<std_srvs::srv::Trigger>(
        "restart_sequence",
        std::bind(&VideoSubmissionController::restartCallback, this, 
                  std::placeholders::_1, std::placeholders::_2));
    
    // Setup parameter callback
    param_callback_handle_ = node_->add_on_set_parameters_callback(
        std::bind(&VideoSubmissionController::parametersCallback, this, std::placeholders::_1));
    
    // Initialize timing
    init_time_ = node_->now();
    start_routine_ = node_->now();
    
    // Pre-compute cumulative times
    computeCumulativeTimes();
    
    // Create timer for control loop (10 Hz)
    timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&VideoSubmissionController::controlLoop, this));
    
    RCLCPP_INFO(node_->get_logger(), "Video Submission Controller initialized with %zu stages", stages_.size());
    printStageInfo();
  }

private:
  void declareStageParameters() {
    // Declare parameters without defaults - values will come from YAML file
    node_->declare_parameter("num_stages", 13);
    
    int num_stages = node_->get_parameter("num_stages").as_int();
    
    for (int i = 0; i < num_stages; ++i) {
      std::string prefix = "stage_" + std::to_string(i) + "_";
      node_->declare_parameter(prefix + "name", "");
      node_->declare_parameter(prefix + "duration", 0.0);
      node_->declare_parameter(prefix + "arm", false);
      node_->declare_parameter(prefix + "mode", "ALT_HOLD");
      node_->declare_parameter(prefix + "forward", 1500);
      node_->declare_parameter(prefix + "lateral", 1500);
      node_->declare_parameter(prefix + "thrust", 1500);
      node_->declare_parameter(prefix + "yaw", 1500);
      node_->declare_parameter(prefix + "roll", 1500);
      node_->declare_parameter(prefix + "pitch", 1500);
    }
  }
  
  void initializePIDController() {
    depth_controller_.kp = node_->get_parameter("depth_pid_kp").as_double();
    depth_controller_.ki = node_->get_parameter("depth_pid_ki").as_double();
    depth_controller_.kd = node_->get_parameter("depth_pid_kd").as_double();
    depth_controller_.base_offset = node_->get_parameter("depth_pid_base_offset").as_int();
    
    depth_target_ = node_->get_parameter("depth_target").as_double();
    
    RCLCPP_INFO(node_->get_logger(), "Depth PID - Kp: %.2f, Ki: %.3f, Kd: %.2f, Offset: %.2f, Target: %.2f",
                depth_controller_.kp, depth_controller_.ki, depth_controller_.kd, 
                depth_controller_.base_offset, depth_target_);
  }
  
  void loadStagesFromParameters() {
    int num_stages = node_->get_parameter("num_stages").as_int();
    stages_.clear();
    stages_.reserve(num_stages);
    
    for (int i = 0; i < num_stages; ++i) {
      std::string prefix = "stage_" + std::to_string(i) + "_";
      
      Stage stage;
      stage.name = node_->get_parameter(prefix + "name").as_string();
      stage.duration = node_->get_parameter(prefix + "duration").as_double();
      stage.arm = node_->get_parameter(prefix + "arm").as_bool();
      stage.mode = node_->get_parameter(prefix + "mode").as_string();
      stage.forward = node_->get_parameter(prefix + "forward").as_int();
      stage.lateral = node_->get_parameter(prefix + "lateral").as_int();
      stage.thrust = node_->get_parameter(prefix + "thrust").as_int();
      stage.yaw = node_->get_parameter(prefix + "yaw").as_int();
      stage.roll = node_->get_parameter(prefix + "roll").as_int();
      stage.pitch = node_->get_parameter(prefix + "pitch").as_int();
      
      stages_.push_back(stage);
    }
  }
  
  void computeCumulativeTimes() {
    cumulative_times_.clear();
    cumulative_times_.reserve(stages_.size());
    
    float sum = 0.0f;
    for (const Stage &stage : stages_) {
      sum += stage.duration;
      cumulative_times_.push_back(sum);
    }
  }
  
  void printStageInfo() {
    RCLCPP_INFO(node_->get_logger(), "Stage configuration:");
    float total_time = 0.0f;
    for (size_t i = 0; i < stages_.size(); ++i) {
      const Stage &s = stages_[i];
      RCLCPP_INFO(node_->get_logger(), 
                  "  [%zu] %s: %.2fs, arm=%d, mode=%s, fwd=%d, lat=%d, thr=%d, yaw=%d",
                  i, s.name.c_str(), s.duration, s.arm, s.mode.c_str(),
                  s.forward, s.lateral, s.thrust, s.yaw);
      total_time += s.duration;
    }
    RCLCPP_INFO(node_->get_logger(), "Total sequence time: %.2fs", total_time);
  }
  
  void keysCallback(const std_msgs::msg::Char::SharedPtr msg) {
    char key = msg->data;
    
    if (key == 'q') {
      software_arm_flag_ = false;
      RCLCPP_INFO(node_->get_logger(), "UNARMED");
      start_routine_ = node_->now();
      depth_controller_.emptyError();
    } else if (key == 'p') {
      software_arm_flag_ = true;
      RCLCPP_INFO(node_->get_logger(), "ARMED - Starting sequence");
      start_routine_ = node_->now();
    }
  }
  
  void telemetryCallback(const custom_msgs::msg::Telemetry::SharedPtr msg) {
    double current_depth = msg->external_pressure;
    depth_error_ = depth_target_ - current_depth;
  }
  
  void controlLoop() {
    if (software_arm_flag_) {
      rclcpp::Time time_now = node_->now();
      float elapsed = (time_now - start_routine_).seconds();
      
      // Calculate PID output
      float pid_depth = depth_controller_.pid_control(
          depth_error_, (time_now - init_time_).seconds(), false);
      
      // Find current stage
      int stage_idx = -1;
      for (size_t i = 0; i < cumulative_times_.size(); ++i) {
        if (elapsed < cumulative_times_[i]) {
          stage_idx = static_cast<int>(i);
          break;
        }
      }
      
      if (stage_idx >= 0 && stage_idx < static_cast<int>(stages_.size())) {
        const Stage &current_stage = stages_[stage_idx];
        
        // Apply stage settings
        cmd_pwm_.arm = current_stage.arm;
        cmd_pwm_.mode = current_stage.mode;
        cmd_pwm_.forward = current_stage.forward;
        cmd_pwm_.lateral = current_stage.lateral;
        cmd_pwm_.thrust = current_stage.thrust;
        cmd_pwm_.yaw = current_stage.yaw;
        cmd_pwm_.roll = current_stage.roll;
        cmd_pwm_.pitch = current_stage.pitch;
        
        // Clear depth error for initial stage
        if (stage_idx == 0) {
          depth_controller_.emptyError();
        }
        
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "Stage: %s (%.1fs elapsed)", current_stage.name.c_str(), elapsed);
      } else {
        // Sequence complete
        cmd_pwm_.arm = false;
        cmd_pwm_.mode = "ALT_HOLD";
        cmd_pwm_.forward = 1500;
        cmd_pwm_.lateral = 1500;
        cmd_pwm_.thrust = 1500;
        cmd_pwm_.yaw = 1500;
        cmd_pwm_.roll = 1500;
        cmd_pwm_.pitch = 1500;
        
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "Sequence complete (%.1fs)", elapsed);
      }
    } else {
      depth_controller_.emptyError();
      
      cmd_pwm_.arm = false;
      cmd_pwm_.mode = "ALT_HOLD";
      cmd_pwm_.forward = 1500;
      cmd_pwm_.lateral = 1500;
      cmd_pwm_.thrust = 1500;
      cmd_pwm_.yaw = 1500;
      cmd_pwm_.roll = 1500;
      cmd_pwm_.pitch = 1500;
    }
    
    pwm_publisher_->publish(cmd_pwm_);
  }
  
  rcl_interfaces::msg::SetParametersResult parametersCallback(
      const std::vector<rclcpp::Parameter> &parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    
    bool reload_stages = false;
    bool reload_pid = false;
    
    for (const auto &param : parameters) {
      const std::string &name = param.get_name();
      
      // Check if PID parameters changed
      if (name.find("depth_pid") == 0 || name == "depth_target") {
        reload_pid = true;
      }
      
      // Check if stage parameters changed
      if (name.find("stage_") == 0 || name == "num_stages") {
        reload_stages = true;
      }
    }
    
    if (reload_pid) {
      initializePIDController();
      RCLCPP_INFO(node_->get_logger(), "PID parameters reloaded at runtime");
    }
    
    if (reload_stages) {
      loadStagesFromParameters();
      computeCumulativeTimes();
      printStageInfo();
      RCLCPP_INFO(node_->get_logger(), "Stage parameters reloaded at runtime");
    }
    
    return result;
  }
  
  void restartCallback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    (void)request; // Unused
    
    // Reset timing
    start_routine_ = node_->now();
    init_time_ = node_->now();
    
    // Clear PID errors
    depth_controller_.emptyError();
    depth_error_ = 0.0;
    
    // Reload all parameters
    initializePIDController();
    loadStagesFromParameters();
    computeCumulativeTimes();
    
    response->success = true;
    response->message = "Sequence restarted successfully";
    
    RCLCPP_INFO(node_->get_logger(), "Sequence restarted via service call");
  }

  // ROS2 Node
  rclcpp::Node::SharedPtr node_;
  
  // PID Controller
  PID_Controller depth_controller_;
  
  // Publishers and Subscribers
  rclcpp::Publisher<custom_msgs::msg::Commands>::SharedPtr pwm_publisher_;
  rclcpp::Subscription<std_msgs::msg::Char>::SharedPtr keys_subscriber_;
  rclcpp::Subscription<custom_msgs::msg::Telemetry>::SharedPtr telemetry_subscriber_;
  
  // Service
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr restart_service_;
  
  // Parameter callback
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  
  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  
  // Control state
  bool software_arm_flag_ = false;
  custom_msgs::msg::Commands cmd_pwm_;
  
  // PID state
  double depth_error_ = 0.0;
  double depth_target_;
  
  // Stage management
  std::vector<Stage> stages_;
  std::vector<float> cumulative_times_;
  
  // Timing
  rclcpp::Time init_time_;
  rclcpp::Time start_routine_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("video_submission_controller");
  auto controller = std::make_shared<VideoSubmissionController>(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
