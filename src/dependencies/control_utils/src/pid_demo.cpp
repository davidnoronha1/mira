#include <control_utils/control_utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <memory>
#include <cmath>
#include <stacktrace>

class PIDDemo {
public:
  PIDDemo(rclcpp::Node::SharedPtr node) 
      : node_(node),
        depth_controller_("depth", node),
        yaw_controller_("yaw", node) {
    
    // Initialize PID parameters
    initializePIDControllers();
    
    // Create simulated sensor publisher
    simulated_depth_pub_ = node_->create_publisher<std_msgs::msg::Float32>(
        "simulated_depth", 10);
    simulated_yaw_pub_ = node_->create_publisher<std_msgs::msg::Float32>(
        "simulated_yaw", 10);
    
    // Create control output publisher
    depth_output_pub_ = node_->create_publisher<std_msgs::msg::Float32>(
        "depth_control_output", 10);
    yaw_output_pub_ = node_->create_publisher<std_msgs::msg::Float32>(
        "yaw_control_output", 10);
    
    // Timer for demo loop (10 Hz)
    timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&PIDDemo::demoLoop, this));
    
    // Initialize simulation state
    current_depth_ = 1000.0;  // Starting depth
    current_yaw_ = 0.0;       // Starting yaw
    depth_setpoint_ = 1100.0; // Target depth
    yaw_setpoint_ = 90.0;     // Target yaw
    
    start_time_ = node_->now();
    last_disturbance_time_ = 0.0;
    
    RCLCPP_INFO(node_->get_logger(), "PID Controller Demo initialized");
    RCLCPP_INFO(node_->get_logger(), "Depth setpoint: %.2f", depth_setpoint_);
    RCLCPP_INFO(node_->get_logger(), "Yaw setpoint: %.2f", yaw_setpoint_);
    printPIDParameters();
  }

private:
  void initializePIDControllers() {
    // Depth PID initialization
    depth_controller_.kp = 5.5;
    depth_controller_.ki = 0.03;
    depth_controller_.kd = 20.0;
    depth_controller_.base_offset = 1500;
    
    // Yaw PID initialization
    yaw_controller_.kp = 3.0;
    yaw_controller_.ki = 0.01;
    yaw_controller_.kd = 7.0;
    yaw_controller_.base_offset = 1500;
  }
  
  void printPIDParameters() {
    RCLCPP_INFO(node_->get_logger(), "Depth PID - Kp: %.2f, Ki: %.3f, Kd: %.2f, Offset: %.0f",
                depth_controller_.kp, depth_controller_.ki, 
                depth_controller_.kd, depth_controller_.base_offset);
    RCLCPP_INFO(node_->get_logger(), "Yaw PID - Kp: %.2f, Ki: %.3f, Kd: %.2f, Offset: %.0f",
                yaw_controller_.kp, yaw_controller_.ki,
                yaw_controller_.kd, yaw_controller_.base_offset);
  }
  
  void simulateDepthPhysics(float control_output, float dt) {
    // Simple physics simulation: control output affects depth change
    // Normalize control output (1500 is neutral, 1100-1900 range)
    float thrust_force = (control_output - 1500.0f) / 400.0f; // -1 to +1
    
    // Simple first-order dynamics with damping
    float depth_velocity = thrust_force * 50.0f; // Max velocity of 50 units/s
    current_depth_ += depth_velocity * dt;
    
    // Add some drag/damping
    current_depth_ += (depth_setpoint_ - current_depth_) * 0.01f * dt;
    
    // Add disturbance force
    current_depth_ += depth_disturbance_ * dt * 20.0f;
    
    // Add small random noise
    current_depth_ += ((rand() % 100) / 100.0f - 0.5f) * 0.5f;
  }
  
  void simulateYawPhysics(float control_output, float dt) {
    // Simple yaw simulation
    float yaw_torque = (control_output - 1500.0f) / 400.0f; // -1 to +1
    
    // Angular velocity
    float yaw_velocity = yaw_torque * 30.0f; // Max 30 degrees/s
    current_yaw_ += yaw_velocity * dt;
    
    // Normalize to 0-360
    current_yaw_ = fmod(current_yaw_ + 360.0f, 360.0f);
    
    // Add damping
    float yaw_error = yaw_setpoint_ - current_yaw_;
    if (yaw_error > 180.0f) yaw_error -= 360.0f;
    if (yaw_error < -180.0f) yaw_error += 360.0f;
    current_yaw_ += yaw_error * 0.02f * dt;
    
    // Add disturbance torque
    current_yaw_ += yaw_disturbance_ * dt * 15.0f;
    
    // Add noise
    current_yaw_ += ((rand() % 100) / 100.0f - 0.5f) * 0.3f;
  }
  
  void applyDisturbances(double elapsed) {
    // Apply new disturbances every 2 seconds
    if (elapsed - last_disturbance_time_ >= 2.0) {
      // Random depth disturbance (simulating current pushing up/down)
      depth_disturbance_ = ((rand() % 100) / 100.0f - 0.5f) * 4.0f; // -2 to +2
      
      // Random yaw disturbance (simulating rotational current)
      yaw_disturbance_ = ((rand() % 100) / 100.0f - 0.5f) * 3.0f; // -1.5 to +1.5
      
      last_disturbance_time_ = elapsed;
      
      RCLCPP_INFO(node_->get_logger(), 
                  "New disturbances applied - Depth: %.2f, Yaw: %.2f",
                  depth_disturbance_, yaw_disturbance_);
    }
  }
  
  void demoLoop() {
    rclcpp::Time now = node_->now();
    double elapsed = (now - start_time_).seconds();
    float dt = 0.1f; // 100ms
    
    // Apply periodic disturbances
    applyDisturbances(elapsed);
    
    // Calculate errors
    float depth_error = depth_setpoint_ - current_depth_;
    float yaw_error = yaw_setpoint_ - current_yaw_;
    
    // Normalize yaw error to -180 to 180
    while (yaw_error > 180.0f) yaw_error -= 360.0f;
    while (yaw_error < -180.0f) yaw_error += 360.0f;
    
    // Run PID controllers
    float depth_output = depth_controller_.pid_control(depth_error, elapsed, false);
    float yaw_output = yaw_controller_.pid_control(yaw_error, elapsed, false);
    
    // Simulate physics
    simulateDepthPhysics(depth_output, dt);
    simulateYawPhysics(yaw_output, dt);
    
    // Publish simulated sensor data
    auto depth_msg = std_msgs::msg::Float32();
    depth_msg.data = current_depth_;
    simulated_depth_pub_->publish(depth_msg);
    
    auto yaw_msg = std_msgs::msg::Float32();
    yaw_msg.data = current_yaw_;
    simulated_yaw_pub_->publish(yaw_msg);
    
    // Publish control outputs
    auto depth_out_msg = std_msgs::msg::Float32();
    depth_out_msg.data = depth_output;
    depth_output_pub_->publish(depth_out_msg);
    
    auto yaw_out_msg = std_msgs::msg::Float32();
    yaw_out_msg.data = yaw_output;
    yaw_output_pub_->publish(yaw_out_msg);
    
    // Log progress every second
    if (static_cast<int>(elapsed) % 1 == 0 && elapsed - last_log_time_ >= 0.9) {
      RCLCPP_INFO(node_->get_logger(), 
                  "Time: %.1fs | Depth: %.2f (target: %.2f, error: %.2f, output: %.0f) | "
                  "Yaw: %.2f (target: %.2f, error: %.2f, output: %.0f)",
                  elapsed, current_depth_, depth_setpoint_, depth_error, depth_output,
                  current_yaw_, yaw_setpoint_, yaw_error, yaw_output);
      last_log_time_ = elapsed;
    }
    
    // Change setpoints after 10 seconds to demonstrate tracking
    if (elapsed > 10.0 && !setpoint_changed_) {
      depth_setpoint_ = 1050.0;
      yaw_setpoint_ = 180.0;
      setpoint_changed_ = true;
      RCLCPP_INFO(node_->get_logger(), "Changing setpoints - Depth: %.2f, Yaw: %.2f",
                  depth_setpoint_, yaw_setpoint_);
    }
  }
  
  // ROS2 Node
  rclcpp::Node::SharedPtr node_;
  
  // PID Controllers
  PID_Controller depth_controller_;
  PID_Controller yaw_controller_;
  
  // Publishers
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr simulated_depth_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr simulated_yaw_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr depth_output_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yaw_output_pub_;
  
  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  
  // Simulation state
  float current_depth_;
  float current_yaw_;
  float depth_setpoint_;
  float yaw_setpoint_;
  float depth_disturbance_ = 0.0f;
  float yaw_disturbance_ = 0.0f;
  
  // Timing
  rclcpp::Time start_time_;
  double last_log_time_ = 0.0;
  double last_disturbance_time_ = 0.0;
  bool setpoint_changed_ = false;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("pid_controller_demo");
  auto demo = std::make_shared<PIDDemo>(node);
  
  RCLCPP_INFO(node->get_logger(), "Starting PID Controller Demo...");
  RCLCPP_INFO(node->get_logger(), "Use 'ros2 topic echo /depth_pid_output' to see depth PID output");
  RCLCPP_INFO(node->get_logger(), "Use 'ros2 topic echo /yaw_pid_output' to see yaw PID output");
  RCLCPP_INFO(node->get_logger(), "Use 'ros2 param set /pid_controller_demo depth_pid_kp <value>' to tune parameters");
  
  try {
	rclcpp::spin(node);
  } catch(const std::exception &e) {
	RCLCPP_ERROR(node->get_logger(), "Exception in PID Demo: %s", e.what());
	std::stacktrace st = std::stacktrace::current();
	std::cerr << "Stack trace:\n" << st << "\n";
}
  rclcpp::shutdown();
  return 0;
}
