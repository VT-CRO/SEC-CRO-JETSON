#ifndef GO_TO_POSITION_HPP_
#define GO_TO_POSITION_HPP_

#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "behaviortree_cpp/behavior_tree.h"

using NavPose = nav2_msgs::action::NavigateToPose;
using NavGoal = nav2_msgs::action::NavigateToPose_Goal;
using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavPose>;

namespace BT{
    template <> inline NavGoal convertFromString(StringView str)
    {
        auto parts = splitString(str, ';');
        if (parts.size() != 3)
        {
            throw RuntimeError("invalid input");
        } else {
            auto navGoal = NavPose::Goal();
            navGoal.pose.header.frame_id = "map";
            navGoal.pose.pose.position.x = convertFromString<float>(parts[0]);
            navGoal.pose.pose.position.y = convertFromString<float>(parts[1]);

            tf2::Quaternion q;
            q.setRPY(0, 0, convertFromString<float>(parts[0]));
            navGoal.pose.pose.orientation = tf2::toMsg(q);

            return navGoal;
        }
    }
}

class GoToPose : public BT::StatefulActionNode
{
    public:
        GoToPose(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node_ptr);

        static BT::PortsList providedPorts();

        BT::NodeStatus onStart() override;

        BT::NodeStatus onRunning() override;

        void onHalted() override;

    private:
        rclcpp::Node::SharedPtr node_ptr_;
        rclcpp_action::Client<NavPose>::SharedPtr action_client_ptr_;

        bool  done_flag_;

        NavGoal _goal;

        void nav_to_pose_callback(const GoalHandleNav::WrappedResult &result);
};

#endif