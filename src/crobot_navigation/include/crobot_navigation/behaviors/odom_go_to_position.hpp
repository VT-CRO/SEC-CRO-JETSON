#ifndef ODOM_GO_TO_POSITION_CPP
#define ODOM_GO_TO_POSITION_CPP

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "crobot_controller/odometry.hpp"
#include "crobot_navigation/BezierPath.hpp"
#include <geometry_msgs/msg/pose2_d.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "behaviortree_cpp/behavior_tree.h"
#include <geometry_msgs/msg/pose2_d.hpp>

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
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;

        Pose2D current_pose_;
        Pose2D goal_pose_;

        // PID controller values 
        // should we have them here or keep them in crobot_navigation_server?
        double Kp_X = 0.1;
        double Kp_Y = 0.1;
        double Kp_H = 0;

        bool  done_flag_;

        void odom_gtp_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
};


#endif