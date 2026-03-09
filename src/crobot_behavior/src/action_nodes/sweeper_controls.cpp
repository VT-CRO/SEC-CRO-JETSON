#include <chrono>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "crobot_behavior/action_nodes/sweeper_controls.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <string>

SweeperControl::SweeperControl(const std::string &name,
 const BT::NodeConfiguration& config,
 rclcpp::Node::SharedPtr node_ptr)
: BT::StatefulActionNode(name, config), node_ptr_(node_ptr) 
{
     publisher_ = node_ptr->create_publisher<std_msgs::msg::Float64MultiArray>("/sweeper_position_controller/commands", 10);


}

BT::PortsList SweeperControl::providedPorts()   
{
    return {
        BT::InputPort<std::string>("sweeper_command")
    };
}

  
BT::NodeStatus SweeperControl::onStart() {
    if (done_) {
        return BT::NodeStatus::SUCCESS;
    }

    auto sweeperCommand = getInput("sweeper_command", sweeperString);
    
   
    if (!sweeperCommand)
    {
        throw BT::RuntimeError("Missing required input [goal]");
    }

    double position = std::stod(std::string(sweeperString));

    msg.layout.dim.resize(1);

    msg.data = {position};

    RCLCPP_INFO(node_ptr_->get_logger(), "[%s] Initalization Successful", (this->name()).c_str());
    return BT::NodeStatus::RUNNING;

}

BT::NodeStatus SweeperControl::onRunning() {
    if (done_) {
        RCLCPP_INFO(node_ptr_->get_logger(), "[%s] Completed", (this->name()).c_str());
        return BT::NodeStatus::SUCCESS;
    }

    publisher_->publish(msg);
    RCLCPP_INFO(node_ptr_->get_logger(), "[%s] Publish Successful", (this->name()).c_str());
    return BT::NodeStatus::SUCCESS;
}

void SweeperControl::onHalted()  {
}
