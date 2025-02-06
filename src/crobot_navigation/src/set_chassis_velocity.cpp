#include "crobot_navigation/behaviors/set_chassis_velocity.hpp"

SetChassisVelocity(const std::string &name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node_ptr):
        BT::SyncActionNode(name, {}),
        node_ptr_(node_ptr)
{

}

static PortsList SetChassisVelocity::providedPorts() {
    return { InputPort<Twist>("target") };
}

BT::NodeStatus SetChassisVelocity::tick() 
{
    
    


    return BT::NodeStatus::SUCCESS;
}