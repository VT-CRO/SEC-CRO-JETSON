#include <functional>
#include <memory>
#include <thread>

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
// #include <crobot_navigation/action/NavigationPoints.action>

namespace crobot_navigation
{
    class CrobotNavigationActionServer : public rclcpp::Node
    {
        public:
            using NavPose = nav2_msgs::action::NavigateToPose;
            using GoalHandleNav = rclcpp_action::ServerGoalHandle<NavPose>;

            explicit CrobotNavigationActionServer(const rclcpp::NodeOptions & options);

        private:
            rclcpp_action::Server<NavPose>::SharedPtr action_server_;

            rclcpp_action::GoalResponse handle_goal(
                const rclcpp_action::GoalUUID & uuid,
                std::shared_ptr<const NavPose::Goal> goal);

            rclcpp_action::CancelResponse handle_cancel(
                const std::shared_ptr<GoalHandleNav> goal_handle);

            void handle_accepted(const std::shared_ptr<GoalHandleNav> goal_handle);

            void execute(const std::shared_ptr<GoalHandleNav> goal_handle);
    };
}
