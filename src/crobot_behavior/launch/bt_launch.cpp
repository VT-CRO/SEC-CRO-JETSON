#include "behaviortree_cpp_v3/bt_factory.h"
#include <rclcpp/rclcpp.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <string>

//nodes
#include "crobot_behavior/action_nodes/go_to_pose.hpp"
#include "crobot_behavior/NavigationServer.hpp"


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("bt_launch");
  auto navigationNode = std::make_shared<NavigationServer>(rclcpp::NodeOptions());
  

  BT::BehaviorTreeFactory factory;
  
  //register commands
  factory.registerBuilder<GoToPose>(
    "GoToPose",
    [node](const std::string& name, const BT::NodeConfiguration& config)
    {
      return std::make_unique<GoToPose>(name, config, node);
    }
  );


  std::string pkg_path =
  ament_index_cpp::get_package_share_directory("crobot_behavior");

  std::string xml_path = pkg_path + "/trees/default.xml";

  auto tree = factory.createTreeFromFile(xml_path);

  //ticks the tree every 200 ms
  rclcpp::Rate rate(20);
  while (rclcpp::ok())
  {
    tree.tickRoot();
    //activate both the launch node and the navigation server
    rclcpp::spin_some(node);
    rclcpp::spin_some(navigationNode);
    rate.sleep();
  }

  rclcpp::shutdown();
}