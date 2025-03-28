#ifndef APRILTAG_ID_SUBSCRIBER_HPP
#define APRILTAG_ID_SUBSCRIBER_HPP

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>

#include <optional>
#include <thread>
#include <vector>
// using namespace std;
using NavPose = nav2_msgs::action::NavigateToPose;
using NavGoal = nav2_msgs::action::NavigateToPose_Goal;

NavGoal MakeNavGoal(float x, float y, float th);

class AprilTagSubscriberID : public BT::StatefulActionNode {
public:
    AprilTagSubscriberID(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node_ptr);
    ~AprilTagSubscriberID();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

    static BT::PortsList providedPorts();

private:
    void callback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg);

    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr subscription_;
    std::optional<int> last_detected_id;

    vector<Pose2D> positions{(215, 10, 0), (215, 33, 0), (215, 56, 0), (215, 79, 0), (215, 102, 0)};
    bool detection = false;
};

#endif // APRILTAG_ID_SUBSCRIBER_NODE_HPP