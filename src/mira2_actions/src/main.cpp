#include <ament_index_cpp/ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <memory>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("mira2_action_server");
  node->declare_parameter("tree_xml", ament_index_cpp::get_package_share_directory("mira2_actions") + "/config/tree.xml");

  auto tree_xml_path = node->get_parameter("tree_xml").as_string();

  BT::BehaviorTreeFactory factory;

  auto tree = factory.createTreeFromFile(tree_xml_path);
  BT::Groot2Publisher publisher(tree, 1337);

  rclcpp::Rate rate(10); // 10Hz
  while (rclcpp::ok()) {
    tree.tickOnce();
    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}