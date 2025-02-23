#include "crobot_navigation/behavior_node.hpp"
#include <behaviortree_ros2/bt_action_node.hpp>

#include "crobot_navigation/behaviors/go_to_position.hpp"
#include "crobot_navigation/behaviors/start.hpp"
#include "crobot_navigation/behaviors/set_beacon.hpp"
#include "crobot_navigation/behaviors/set_chassis_velocity.hpp"
#include "crobot_navigation/behaviors/apriltag_id_subscriber.hpp"

using namespace std::chrono_literals;

const std::string bt_xml_dir =
    ament_index_cpp::get_package_share_directory("crobot_navigation") + "/tree";

BehaviorNode::BehaviorNode(const std::string &node_name) 
    : Node(node_name)
{
    this->declare_parameter("location_file", "none");

    RCLCPP_INFO(get_logger(), "Init done");
}

void BehaviorNode::setup()
{
    create_behavior_tree();

    const auto timer_period = 500ms;
    timer_ = this->create_wall_timer(
        timer_period,
        std::bind(&BehaviorNode::update_behavior_tree, this)
    );

    rclcpp::spin(shared_from_this());
    rclcpp::shutdown();
}

void BehaviorNode::create_behavior_tree()
{
    BT::BehaviorTreeFactory factory;

    BT::NodeBuilder start_builder = 
        [=](const std::string &name, const BT::NodeConfiguration &config)
        {
            return std::make_unique<StartBehavior>(name, config, shared_from_this());
        };

    BT::NodeBuilder go_to_pose_builder = 
        [=](const std::string &name, const BT::NodeConfiguration &config)
        {
            return std::make_unique<GoToPose>(name, config, shared_from_this());
        };

    BT::NodeBuilder set_beacon_builder = 
        [=](const std::string &name, const BT::NodeConfiguration &config)
        {
            return std::make_unique<SetBeacon>(name, config, shared_from_this());
        };

    BT::NodeBuilder set_chassis_velocity_builder = 
        [=](const std::string &name, const BT::NodeConfiguration &config)
        {
            return std::make_unique<SetChassisVelocity>(name, config, shared_from_this());
        };

    BT::NodeBuilder apriltag_id_subscriber_builder = 
        [=](const std::string &name, const BT::NodeConfiguration &config)
        {
            return std::make_unique<AprilTagSubscriberID>(name, config, shared_from_this());
        };

    factory.registerBuilder<StartBehavior>("Start", start_builder);
    factory.registerBuilder<GoToPose>("GoToPose", go_to_pose_builder);
    factory.registerBuilder<SetBeacon>("SetBeacon", set_beacon_builder);
    factory.registerBuilder<SetChassisVelocity>("SetChassisVelocity", set_chassis_velocity_builder);
    factory.registerBuilder<AprilTagSubscriberID>("AprilTagSubscriberID", apriltag_id_subscriber_builder);

    tree_ = factory.createTreeFromFile(bt_xml_dir + "/bt_default.xml");
}

void BehaviorNode::update_behavior_tree()
{
    BT::NodeStatus tree_status = tree_.tickExactlyOnce();

    if (tree_status == BT::NodeStatus::RUNNING)
    {
        return;
    } else if (tree_status == BT::NodeStatus::SUCCESS)
    {
        RCLCPP_INFO(this->get_logger(), "Finished Navigation");
    } else if (tree_status == BT::NodeStatus::FAILURE)
    {
        RCLCPP_INFO(this->get_logger(), "Navigation Failed");
        // timer_->cancel();
    }

    RCLCPP_INFO(this->get_logger(), "Ticked once");
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BehaviorNode>("autonomy_node");
    node->setup();

    return 0;
}