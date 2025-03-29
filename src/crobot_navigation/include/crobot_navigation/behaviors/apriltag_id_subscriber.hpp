#ifndef APRILTAG_ID_SUBSCRIBER_HPP
#define APRILTAG_ID_SUBSCRIBER_HPP

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include "crobot_navigation/BezierPath.hpp"
#include "crobot_msgs/action/navigation_points.hpp"
#include <geometry_msgs/msg/pose2_d.hpp>

#include <optional>
#include <thread>
#include <vector>
// using namespace std;
using NavPoints = crobot_msgs::action::NavigationPoints;
using NavPointsGoal = crobot_msgs::action::NavigationPoints::Goal;
using Pose2D = geometry_msgs::msg::Pose2D;

class AprilTagSubscriberID : public BT::StatefulActionNode {
public:
    AprilTagSubscriberID(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node_ptr);
    ~AprilTagSubscriberID();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

    static BT::PortsList providedPorts();

private:
    void preparePoints();
    void callback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg);

    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr subscription_;
    std::optional<int> last_detected_id;
    std::vector<NavPointsGoal> positions;
    bool detection = false;
};

#endif // APRILTAG_ID_SUBSCRIBER_NODE_HPP