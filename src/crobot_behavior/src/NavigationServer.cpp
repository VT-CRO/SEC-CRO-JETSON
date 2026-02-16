#include <chrono>
#include "crobot_behavior/NavigationServer.hpp"

NavigationServer::NavigationServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : Node("crobot_navigation", options) {
            this->action_server_ = rclcpp_action::create_server<NavPoints>(
            this,
            "crobot_navigation",
            std::bind(&NavigationServer::handle_goal, this, _1, _2),
            std::bind(&NavigationServer::handle_cancel, this, _1),
            std::bind(&NavigationServer::handle_accepted, this, _1)
        );
        }

rclcpp_action::GoalResponse NavigationServer::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const NavGoal> goal) 
    {
        //hold the (x,y) positions of the goal
        auto poses = goal->poses;
        auto lastPose = poses.back();

        goal_x_ = lastPose.pose.position.x;
        goal_y_ = lastPose.pose.position.y;



        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

rclcpp_action::CancelResponse NavigationServer::handle_cancel(
        const std::shared_ptr<NavigationServer::GoalHandleNav> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

void NavigationServer::handle_accepted(const std::shared_ptr<NavigationServer::GoalHandleNav> goal_handle)
  {
    std::thread{std::bind(&NavigationServer::execute, this, _1), goal_handle}.detach();
  }

void NavigationServer::execute(const std::shared_ptr<NavigationServer::GoalHandleNav> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing navigation goal");
    /*
    For now, this execution will automatically complete since there is currently no pose
    estimation on the robot :(
    */
    // auto feedback = std::make_shared<NavPose::Feedback>();

    // while (rclcpp::ok()) {
    //     //current_x_ and current_y_ WILL NOT WORK UNTIL POSE ESTIMATION IS COMPLETED
    //     //When pose estimation is completed, update current_x_ and current_y_ with said poses
    //     double dx = goal_x_ - current_x_;
    //     double dy = goal_y_ - current_y_;

    //     feedback->distance_remaining = std::sqrt(dx * dx + dy * dy);

    //     goal_handle->publish_feedback(feedback);

    //     if (feedback->distance_remaining < 0.05) {
    //     break;
    //     }

    //     rclcpp::sleep_for(std::chrono::milliseconds(200));
    // }

    goal_handle->succeed(std::make_shared<NavPoints::Result>());
  }