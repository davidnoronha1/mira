#include "navigation.hpp"
#include <ament_index_cpp/ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <utils/parameters.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("mira2_actions");
  const auto tree_path = utils::parameter(*node, "tree_xml",
      ament_index_cpp::get_package_share_directory("mira2_actions") + "/config/demo.xml");
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<SelectObject>("SelectObject", node);
  factory.registerNodeType<NavigateToPose>("NavigateToPose", node);
  auto blackboard = BT::Blackboard::create();
  blackboard->set("target_class", utils::parameter(*node, "target_class", std::string("dock")));
  auto tree = factory.createTreeFromFile(tree_path, blackboard);
  BT::Groot2Publisher publisher(tree, utils::parameter(*node, "groot_port", 1337));
  rclcpp::Rate rate(utils::checked_parameter(*node, "control_rate", 10.0, utils::positive));
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
    rclcpp::spin_some(node);
    status = tree.tickOnce();
    rate.sleep();
  }
  tree.haltTree();
  // Service-based hold requests need executor time before process exit.
  if (rclcpp::ok()) rclcpp::spin_some(node);
  rclcpp::shutdown();
  return status == BT::NodeStatus::SUCCESS ? 0 : 1;
}
