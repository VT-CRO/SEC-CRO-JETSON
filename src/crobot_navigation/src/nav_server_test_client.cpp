#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>
#include <vector>

#include "crobot_msgs/action/navigation_points.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace crobot_navigation
{
class NavActionClient : public rclcpp::Node
{
public:
  using NavigationGoalPoints = crobot_msgs::action::NavigationPoints;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigationGoalPoints>;

  explicit NavActionClient(const rclcpp::NodeOptions & options)
  : Node("crobot_navigation_action_client", options)
  {
    this->client_ptr_ = rclcpp_action::create_client<NavigationGoalPoints>(
      this,
      "crobot_navigation");

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&NavActionClient::send_goal, this));
  }

  void create_goal_points(std::vector<geometry_msgs::msg::Pose2D> &points)
  {
    geometry_msgs::msg::Pose2D p0;
    p0.x = 0;
    p0.y = 0;
    p0.theta = 0;
    points.push_back(p0);

    geometry_msgs::msg::Pose2D p1;
    p1.x = 40;
    p1.y = -40;
    p1.theta = 0;
    points.push_back(p1);

    geometry_msgs::msg::Pose2D p2;
    p2.x = 0;
    p2.y = 0;
    p2.theta = 0;
    points.push_back(p2);

    geometry_msgs::msg::Pose2D p3;
    p3.x = 10;
    p3.y = 10;
    p3.theta = 3.14;
    points.push_back(p3);
  }

  void send_goal()
  {
    using namespace std::placeholders;

    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = NavigationGoalPoints::Goal();
    create_goal_points(goal_msg.points);

    for (auto p : goal_msg.points) {
      RCLCPP_INFO(this->get_logger(), "Got position (%f, %f, %f)", p.x, p.y, p.theta);
    }

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<NavigationGoalPoints>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&NavActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&NavActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&NavActionClient::result_callback, this, _1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<NavigationGoalPoints>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

  void goal_response_callback(const GoalHandleNav::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandleNav::SharedPtr,
    const std::shared_ptr<const NavigationGoalPoints::Feedback> feedback)
  {
    // std::stringstream ss;
    // ss << "Next number in sequence received: ";
    // for (auto number : feedback->partial_sequence) {
    //   ss << number << " ";
    // }
    // RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }

  void result_callback(const GoalHandleNav::WrappedResult & result)
  {
    // switch (result.code) {
    //   case rclcpp_action::ResultCode::SUCCEEDED:
    //     break;
    //   case rclcpp_action::ResultCode::ABORTED:
    //     RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
    //     return;
    //   case rclcpp_action::ResultCode::CANCELED:
    //     RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
    //     return;
    //   default:
    //     RCLCPP_ERROR(this->get_logger(), "Unknown result code");
    //     return;
    // }
    // std::stringstream ss;
    // ss << "Result received: ";
    // for (auto number : result.result->sequence) {
    //   ss << number << " ";
    // }
    // RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    // rclcpp::shutdown();
  }
};  // class FibonacciActionClient

}  // namespace action_tutorials_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(crobot_navigation::NavActionClient)