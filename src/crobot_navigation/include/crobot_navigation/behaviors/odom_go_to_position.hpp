#ifndef ODOM_GO_TO_POSITION_CPP
#define ODOM_GO_TO_POSITION_CPP

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "crobot_navigation/BezierPath.hpp"
#include "crobot_msgs/action/navigation_points.hpp"
#include <geometry_msgs/msg/pose2_d.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "behaviortree_cpp/behavior_tree.h"
#include <vector>
#include <cmath>

using NavPoints = crobot_msgs::action::NavigationPoints;
using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavPoints>;
using Pose2D = geometry_msgs::msg::Pose2D;

namespace BT{
    template <> inline Pose2D convertFromString(StringView str)
    {
        auto parts = splitString(str, ',');
        if (parts.size() != 3) {
            throw RuntimeError("invalid input");
        }
        double radians = convertFromString<float>(parts[2]) * M_PI / 180.0;
        Pose2D pose;
        pose.x = convertFromString<float>(parts[0]);
        pose.y = convertFromString<float>(parts[1]);
        pose.theta = radians;

        return pose;
    }

    template <> inline NavPoints::Goal convertFromString(StringView str)
    {
        auto points = splitString(str, ';');
        if (points.size() <= 0)
        {
            throw RuntimeError("invalid input");
        } else {
            auto navGoal = NavPoints::Goal();
            for (std::size_t i = 0; i < points.size(); i++) {
                
                Pose2D pose = convertFromString<Pose2D>(points[i]);
                navGoal.points.push_back(pose);
            }

            return navGoal;
        }
    }
}

class GoToPoseWithOdometry : public BT::StatefulActionNode
{
    public:
        GoToPoseWithOdometry(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node_ptr);

        static BT::PortsList providedPorts();

        BT::NodeStatus onStart() override;

        BT::NodeStatus onRunning() override;

        void onHalted() override;

    private:
        rclcpp::Node::SharedPtr node_ptr_;
        rclcpp_action::Client<NavPoints>::SharedPtr action_client_ptr_;

        bool  done_flag_;

        NavPoints::Goal _action_msg;

        void nav_to_pose_callback(const GoalHandleNav::WrappedResult &result);
};




#endif