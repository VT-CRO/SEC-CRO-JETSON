#include <chrono>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "crobot_behavior/action_nodes/go_to_pose.hpp"
#include <string>

GoToPose::GoToPose(const std::string &name,
 const BT::NodeConfiguration& config,
 rclcpp::Node::SharedPtr node_ptr)
: BT::StatefulActionNode(name, config), node_ptr_(node_ptr) 
{
    //if going through Nav2, send it to the exposed topic "navigate_through_poses"
    action_client_ptr_ = rclcpp_action::create_client<NavPoints>(
        node_ptr_,
        "navigate_through_poses"
    );


}

//might need this later to update position on the field also
BT::PortsList GoToPose::providedPorts()   
{
    return {
        //stores the input from the user in goalPoses
        BT::InputPort<std::string>("goalPoses")
    };
}
  
BT::NodeStatus GoToPose::onStart() {
    if (done_) {
        return BT::NodeStatus::SUCCESS;
    }

    //splices the string 
    auto navGoal = getInput("goalPoses", pose_strings);
   
    if (!navGoal)
    {
        throw BT::RuntimeError("Missing required input [goal]");
    }

    auto pose_split_string = BT::splitString(pose_strings, ';');
    //resize the vector for the points
    goal.poses.resize(pose_split_string.size());

    int index = 0;
    //loops through all the inputted parameters and adds it to the goal poses array
    for (const auto& pose : pose_split_string) {
        RCLCPP_INFO(node_ptr_->get_logger(), (std::string(pose)).c_str());
        auto values = BT::splitString(pose,',');

        if (values.size() != 3)
        {
        throw BT::RuntimeError(
            "Invalid format. Expected x,y,yaw");
        }

        double x = std::stod(std::string(values[0]));
        double y = std::stod(std::string(values[1]));
        double yaw = std::stod(std::string(values[2]));
        RCLCPP_INFO(node_ptr_->get_logger(), "X: [%s], Y:[%s]",
        std::to_string(x).c_str(), std::to_string(y).c_str());

        
        goal.poses[index].pose.position.x = x;
        goal.poses[index].pose.position.y = y;
        goal.poses[index].pose.position.z = 0;
        goal.poses[index].header.frame_id = "map";
        goal.poses[index].header.stamp = node_ptr_->now();
        tf2::Quaternion q;
        //radians to yaw
        q.setRPY(0.0, 0.0, yaw);
        q.normalize();
        
        goal.poses[index].pose.orientation = tf2::toMsg(q);


        index += 1;
    }

    if (!this->action_client_ptr_->wait_for_action_server())
    {
        RCLCPP_ERROR(node_ptr_->get_logger(), "Action server not available after waiting");
    }
    //sends the goal options
    auto send_goal_options = rclcpp_action::Client<NavPoints>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(&GoToPose::nav_to_pose_callback, this, std::placeholders::_1);

    //if we want custom navigation server, send it to the navigation server
    action_client_ptr_->async_send_goal(goal, send_goal_options);

    RCLCPP_INFO(node_ptr_->get_logger(), "[%s] Initalization Successful", (this->name()).c_str());
    return BT::NodeStatus::RUNNING;

}

BT::NodeStatus GoToPose::onRunning() {
    if (done_) {
        RCLCPP_INFO(node_ptr_->get_logger(), "[%s] Completed", (this->name()).c_str());
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
}

void GoToPose::onHalted()  {
}

void GoToPose::nav_to_pose_callback(const GoalHandleNav::WrappedResult &result)
{
    if (result.result)
    {
        done_ = true;
    }
}