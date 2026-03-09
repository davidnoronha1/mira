#include "behaviours/behaviours.hpp"
#include "motions/motions.hpp"
#include <ament_index_cpp/ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/xml_parsing.h" 
#include <ostream> // <-- this is the missing one

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("mira2_action_server");
  node->declare_parameter(
      "tree_xml",
      ament_index_cpp::get_package_share_directory("mira2_actions") +
          "/config/default.xml");

  auto tree_xml_path = node->get_parameter("tree_xml").as_string();

  BT::BehaviorTreeFactory factory;

  ROSState state(node);

  factory.registerNodeType<DetectObjectBoundingbox>("DetectObjectBoundingBox", &state);
  factory.registerNodeType<DetectTargetPoint>("DetectTargetPoint", &state);
  factory.registerNodeType<ApproachBB>("ApproachBoundingBox", &state);
  factory.registerNodeType<AlignXY>("AlignXY", &state);
  factory.registerNodeType<ApproachWithDepth>("ApproachWithDepth", &state);
  factory.registerNodeType<HoldPosition>("HoldPosition", &state);
  factory.registerNodeType<LateralEvasion>("LateralEvasion", &state);
  factory.registerNodeType<RotateToTargetHeading>("RotateToTargetHeading", &state);
  factory.registerNodeType<YawSweep>("YawSweep", &state);
  factory.registerNodeType<RotateToTargetHeading>("YawToTargetHeading", &state);

  // Generate the XML model string
  const std::string xml_models = BT::writeTreeNodesModelXML(factory);

  // Save it to a file so Groot2 can load it
  std::ofstream xml_file(ament_index_cpp::get_package_share_directory("mira2_actions") + "/mira2_bt_models.xml");
  if (xml_file.is_open()) {
    xml_file << xml_models;
    xml_file.close();
    RCLCPP_INFO(node->get_logger(), "Behavior Tree models XML saved to mira2_bt_models.xml");
  } else {
    RCLCPP_ERROR(node->get_logger(), "Failed to save Behavior Tree models XML");
  }

  auto tree = factory.createTreeFromFile(tree_xml_path);
  BT::Groot2Publisher publisher(tree, 1337);
  const char *machine_ip = std::getenv("MACHINE_IP");
  RCLCPP_INFO(node->get_logger(), "To connect to behaviour tree, connect to %s:1337", machine_ip ? machine_ip : "{MACHINE_IP}");

  rclcpp::Rate rate(10); // 10Hz
  while (rclcpp::ok()) {
    tree.tickOnce();
    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
