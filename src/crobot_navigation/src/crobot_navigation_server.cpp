#include "crobot_navigation/crobot_navigation_server.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "crobot_navigation/behaviors/set_chassis_velocity.hpp"

namespace crobot_navigation
{
    CrobotNavigationActionServer::CrobotNavigationActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : Node("crobot_navigation_server", options)
    {
        using namespace std::placeholders;
        
        RCLCPP_INFO(this->get_logger(), "Starting navigation server!");

        this->action_server_ = rclcpp_action::create_server<NavigationGoalPoints>(
            this,
            "crobot_navigation",
            std::bind(&CrobotNavigationActionServer::handle_goal, this, _1, _2),
            std::bind(&CrobotNavigationActionServer::handle_cancel, this, _1),
            std::bind(&CrobotNavigationActionServer::handle_accepted, this, _1)
        );
        

        // Publisher for command velocity
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Subscriber for odometry
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&CrobotNavigationActionServer::odom_cb, this, _1));
    }

    void CrobotNavigationActionServer::odom_cb(const nav_msgs::msg::Odometry msg)
    {
        currentPos.x = msg.pose.pose.position.x * 100;
        currentPos.y = msg.pose.pose.position.y * 100;

        tf2::Quaternion q(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w);

        tf2::Matrix3x3 m(q);
        double r, p, y;
        m.getRPY(r, p, y);

        currentPos.theta = y;
    }

    rclcpp_action::GoalResponse CrobotNavigationActionServer::handle_goal(const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const NavigationGoalPoints::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal points:");
        for (auto p : goal->points) {
            RCLCPP_INFO(this->get_logger(), "\t(%f, %f)", p.x, p.y);
        }
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse CrobotNavigationActionServer::handle_cancel(
        const std::shared_ptr<GoalHandleNav> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void CrobotNavigationActionServer::handle_accepted(const std::shared_ptr<GoalHandleNav> goal_handle)
    {
        using namespace std::placeholders;
        std::thread{std::bind(&CrobotNavigationActionServer::execute, this, _1), goal_handle}.detach();
    }
    

    void CrobotNavigationActionServer::execute(const std::shared_ptr<GoalHandleNav> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Setting up path...");
        rclcpp::Rate loop_rate(1);
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<NavigationGoalPoints::Result>();

        BezierPath BP;
        std::vector <Pose2D> points = goal->points;
        double currentT = 0.0;
        std::vector<double> binomialCoef = BP.binomialCoefficients(points.size() - 1);

        BP.setupPath(points, binomialCoef); //Running Res's setupPath function
        
        // // Main loop for Implementation
        while (rclcpp::ok()) {
            currentT = BP.closestT(points, currentPos, currentT, binomialCoef);

            RCLCPP_INFO(this->get_logger(), "t: %f, (%f, %f)", currentPos.x, currentPos.y);

            Pose2D desired_pos = BP.pathBezier(points, currentT, binomialCoef);

            // Implementing PID controller

            // PID Controller for X
            double Kp_X = 0.01; // Proportional Gain Constant (To be Fine Tuned)

            double Error_X = desired_pos.x - currentPos.y;
            double Control_X = Kp_X * Error_X;

            // Publish Command Velocity for X
            // auto velocity_msg = geometry_msgs::msg::Twist();
            // velocity_msg.linear.x = Control_X;
            // this->velocity_publisher_->publish(velocity_msg);
            geometry_msgs::msg::Twist velocity_msg;
            velocity_msg.linear.x = Control_X;  // Set the desired velocities
            

            //PID Controller for Y
            double Kp_Y = 0.01; // Proportional Gain Constant (To be Fine Tuned)

            double Error_Y = desired_pos.y - currentPos.y;
            double Control_Y = Kp_Y * Error_Y;

            // Publish Command Velocity for Y
            // auto velocity_msg = geometry_msgs::msg::Twist();
            // velocity_msg.linear.y = Control_Y;
            // this->velocity_publisher_->publish(velocity_msg);
            // geometry_msgs::msg::Twist velocity_msg;
            velocity_msg.linear.y = Control_Y;  // Set the desired velocities

            //PID Controller for H
            double Kp_H = 1; // Proportional Gain Constant (To be Fine Tuned)

            double Error_H = desired_pos.theta - currentPos.theta;
            double Control_H = Kp_H * Error_H;

            // Publish Command Velocity for H
            // auto velocity_msg = geometry_msgs::msg::Twist();
            // velocity_msg.angular.z = Control_H;
            // this->velocity_publisher_->publish(velocity_msg);
            // geometry_msgs::msg::Twist velocity_msg;
            velocity_msg.angular.z = Control_H;  // Set the desired velocities

            publisher_->publish(velocity_msg);

            // Stop running if t=1 and we're within threshold for a certain amount of time
            // if (Error_X < 1 && Error_Y < 1 && Error_H < 1) {
            auto endPos = points.back();
            if (abs(endPos.x - currentPos.x) < 0.01 && abs(endPos.y - currentPos.y) < 0.01 && abs(endPos.theta - currentPos.theta) < 0.0175)
            {
                //stop running
                velocity_msg.linear.x = 0.0;
                velocity_msg.linear.y = 0.0;
                velocity_msg.angular.z = 0.0;

                publisher_->publish(velocity_msg);

                break;
            }
        }

        if (rclcpp::ok()) {
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }
    }
}


RCLCPP_COMPONENTS_REGISTER_NODE(crobot_navigation::CrobotNavigationActionServer)