#include "crobot_navigation/crobot_navigation_server.hpp"
#include "crobot_navigation/BP.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "crobot_navigation/behaviors/set_chassis_velocity.hpp"

namespace crobot_navigation
{
    CrobotNavigationActionServer::CrobotNavigationActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : Node("crobot_navigation_server", options)
    {
        using namespace std::placeholders;
        
        RCLCPP_INFO(this->get_logger(), "Starting navigation server!");

        // TODO: we set the action type as nav2_msgs/NavigateToPose but I have already went ahead and created
        //       an action that is a little bit more tailored to our needs in the action/NavigationPoints.action
        //       file. We need to switch the action type to that one, which may require messing with the CMakeLists.txt
        //       for this package for it to register as a valid action type. Please see the ROS2 actions documentation
        //       for information on how to set this up.
        this->action_server_ = rclcpp_action::create_server<NavPose>(
            this,
            "crobot_navigation",
            std::bind(&CrobotNavigationActionServer::handle_goal, this, _1, _2),
            std::bind(&CrobotNavigationActionServer::handle_cancel, this, _1),
            std::bind(&CrobotNavigationActionServer::handle_accepted, this, _1)
        );
        

        //trying to implement the publisher for the velocity
        //***********************************************************************//

        // rclcpp::Node::SharedPtr node_ptr = shared_from_this(); // Pass current node (or create a new one)
        // SetChassisVelocity set_velocity_node("set_chassis_velocity", BT::NodeConfig{}, node_ptr);
    }

    rclcpp_action::GoalResponse CrobotNavigationActionServer::handle_goal(const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const NavPose::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request!");
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
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        rclcpp::Rate loop_rate(1);
        const auto goal = goal_handle->get_goal();
        // auto feedback = std::make_shared<NavPose::Feedback>();
        auto result = std::make_shared<NavPose::Result>();

        // TODO: Run Res's path setup function

        // TODO: The "main" loop of this action
        //
        // Pseudocode:
        //
        // while (running) {
        //      t = closestT
        //      desired_pos = pathbezier(t)
        //      pid current_pos with desired_pos    // (only p controller is probably necessary--pid is a pretty well-established controller so dd advise looking up someone's implementation or looking at ours in the SEC-CRO-LIB repo)
        //      publish command velocity
        //      stop running if t=1 and we're within threshold for a certain amount of time
        // }


        BezierPath BP;
        std::vector <PoseStamped> points;
        PoseStamped currentPos;
        double currentT = 0.0;
        std::vector<double> binomialCoef;

        BP.setupPath(points, binomialCoef); //Running Res's setupPath function

        // Main loop for Implementation
        while (rclcpp::ok()) {
            double t = BP.closestT(points, currentPos, currentT, binomialCoef);
            PoseStamped desired_pos = BP.pathBezier(points, t, binomialCoef);

            // Implementing PID controller

            // PID Controller for X
            double Kp_X = 0.0; // Proportional Gain Constant (To be Fine Tuned)

            double Error_X = desired_pos.getX() - currentPos.getX();
            double Control_X = Kp_X * Error_X;

            // Publish Command Velocity for X
            // auto velocity_msg = geometry_msgs::msg::Twist();
            // velocity_msg.linear.x = Control_X;
            // this->velocity_publisher_->publish(velocity_msg);
            geometry_msgs::msg::Twist velocity_msg;
            velocity_msg.linear.x = Control_X;  // Set the desired velocities
            

            //PID Controller for Y
            double Kp_Y = 0.0; // Proportional Gain Constant (To be Fine Tuned)

            double Error_Y = desired_pos.getY() - currentPos.getY();
            double Control_Y = Kp_Y * Error_Y;

            // Publish Command Velocity for Y
            // auto velocity_msg = geometry_msgs::msg::Twist();
            // velocity_msg.linear.y = Control_Y;
            // this->velocity_publisher_->publish(velocity_msg);
            // geometry_msgs::msg::Twist velocity_msg;
            velocity_msg.linear.y = Control_Y;  // Set the desired velocities

            //PID Controller for H
            double Kp_H = 0.0; // Proportional Gain Constant (To be Fine Tuned)

            double Error_H = desired_pos.getH() - currentPos.getH();
            double Control_H = Kp_H * Error_H;

            // Publish Command Velocity for H
            // auto velocity_msg = geometry_msgs::msg::Twist();
            // velocity_msg.angular.z = Control_H;
            // this->velocity_publisher_->publish(velocity_msg);
            // geometry_msgs::msg::Twist velocity_msg;
            velocity_msg.angular.z = Control_H;  // Set the desired velocities


            // Stop running if t=1 and we're within threshold for a certain amount of time
            if (t == 1.0) {
                //stop running
                velocity_msg.linear.x = 0.0;
                velocity_msg.linear.y = 0.0;
                velocity_msg.angular.z = 0.0;
            }


        if (rclcpp::ok()) {
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }
    }
}


RCLCPP_COMPONENTS_REGISTER_NODE(crobot_navigation::CrobotNavigationActionServer)