#include "navigation.hpp"
#include "gate.hpp"
#include "scan.hpp"
#include <ament_index_cpp/ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <utils/parameters.hpp>
#include <chrono>
#include <csignal>
#include <thread>

namespace {
volatile std::sig_atomic_t interrupted = 0;
void interrupt(int) { interrupted = 1; }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, interrupt);
  std::signal(SIGTERM, interrupt);
  auto node = std::make_shared<rclcpp::Node>("mira2_actions");
  // A newly created simulated clock is zero until /clock is discovered. Starting
  // acquisition there makes its one-second deadline expire on the first sample.
  const auto clock_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (rclcpp::ok() && !interrupted && node->get_clock()->ros_time_is_active() &&
         node->now().nanoseconds() == 0) {
    if (std::chrono::steady_clock::now() >= clock_deadline) {
      RCLCPP_ERROR(node->get_logger(), "No simulator clock received within 10 seconds");
      rclcpp::shutdown();
      return 1;
    }
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  const auto tree_path = utils::parameter(*node, "tree_xml",
      ament_index_cpp::get_package_share_directory("mira2_actions") + "/config/demo.xml");
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  // TF gets its own thread: a 10 Hz tick taking one /tf message per spin falls
  // seconds behind a 30 Hz EKF transform, leaving camera observations unusable.
  tf2_ros::TransformListener listener(*tf, node, true);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<SelectObject>("SelectObject", node);
  factory.registerNodeType<NavigateToPose>("NavigateToPose", node, tf);
  factory.registerNodeType<TraverseGate>("TraverseGate", node, tf);
  factory.registerNodeType<ScanArea>("ScanArea", node, tf);
  auto blackboard = BT::Blackboard::create();
  blackboard->set("scan_radius", utils::parameter(*node, "scan_radius", 0.0));
  blackboard->set("scan_sweep", utils::parameter(*node, "scan_sweep", 360.0));
  blackboard->set("scan_step", utils::parameter(*node, "scan_step", 30.0));
  blackboard->set("scan_depth", utils::parameter(*node, "scan_depth", 1.0));
  blackboard->set("target_class", utils::parameter(*node, "target_class", std::string("dock")));
  auto tree = factory.createTreeFromFile(tree_path, blackboard);
  // Discover dynamic TF before selecting a short-lived camera observation.
  // Otherwise the observation can predate this process's entire TF history.
  const auto tf_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (rclcpp::ok() && !interrupted && !tf->canTransform("odom", "base_link", tf2::TimePointZero)) {
    if (std::chrono::steady_clock::now() >= tf_deadline) {
      RCLCPP_ERROR(node->get_logger(), "No odom <- base_link transform within 10 seconds");
      rclcpp::shutdown();
      return 1;
    }
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  BT::Groot2Publisher publisher(tree, utils::parameter(*node, "groot_port", 1337));
  rclcpp::Rate rate(utils::checked_parameter(*node, "control_rate", 10.0, utils::positive));
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  while (rclcpp::ok() && !interrupted && status == BT::NodeStatus::RUNNING) {
    executor.spin_all(std::chrono::milliseconds(0));  // Drain every queued message.
    status = tree.tickOnce();
    rate.sleep();
  }
  tree.haltTree();
  if (interrupted) {
    RCLCPP_INFO(node->get_logger(), "Behavior tree cancelled; hold requested for active navigation");
    status = BT::NodeStatus::FAILURE;
  }
  RCLCPP_INFO(node->get_logger(), "Behavior tree finished: %s", BT::toStr(status).c_str());
  // Keep ROS alive through halt and give the asynchronous hold service time to
  // complete, including when SIGINT/SIGTERM interrupted a running navigation.
  const auto drain_deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < drain_deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  rclcpp::shutdown();
  return status == BT::NodeStatus::SUCCESS ? 0 : 1;
}
