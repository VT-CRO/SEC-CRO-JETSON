#include <cstdlib>
#include "crobot_behavior/action_nodes/get_that_bag.hpp"

GetThatBag::GetThatBag(
    const std::string &name,
    const BT::NodeConfiguration &config
) : BT::SyncActionNode(name, config)
{

}

BT::PortsList GetThatBag::providedPorts()
{
    return {
        BT::InputPort<std::string>("path")
    };
}

BT::NodeStatus GetThatBag::tick() {
    std::string path;
    getInput("path", path);
    std::string command = "ros2 bag play " + path;
    system(command.c_str());
    return BT::NodeStatus::SUCCESS;
}