#include "crobot_navigation/behaviors/set_chassis_velocity.hpp"
#include "unistd.h"

SetChassisVelocity::SetChassisVelocity(const std::string &name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node_ptr):
        BT::SyncActionNode(name, {config}),
        node_ptr_(node_ptr)
{
    publisher_ = node_ptr_-> create_publisher<Twist>("/crobot_drive_controller/cmd_vel_stamped", 10);
}

BT::NodeStatus SetChassisVelocity::tick() 
{
    
    auto res = getInput<Twist>("target");
    auto res_sleep = getInput<int>("duration_s");

    if ( !res || !res_sleep ) {
        throw BT::RuntimeError("error reading port [target]: ", res.error());
    } 
    publisher_->publish(res.value());

    sleep(res_sleep.value());

    return BT::NodeStatus::SUCCESS;
}