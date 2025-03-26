#include "crobot_navigation/odom_go_to_position"

GoToPoseWithOdometry::GoToPoseWithOdometry(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node_ptr)
  : BT::StatefulActionNode(name, config), node_ptr_(node_ptr)
{
  odom_sub_ = node_ptr_->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&GoToPoseWithOdometry::odomCallback, this, std::placeholders::_1));
  
  cmd_vel_pub_ = node_ptr_->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
  
  currentPos = {0.0, 0.0, 0.0};
}

void GoToPoseWithOdometry::odom_gtp_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
///// commented out section was taken from crobot_navigation_server.cpp and can be uncommented if the other stuff doesn't work //////
    // currentPos.x = msg.pose.pose.position.x;
    // currentPos.y = msg.pose.pose.position.y;

    // tf2::Quaternion q(
    //     msg.pose.pose.orientation.x,
    //     msg.pose.pose.orientation.y,
    //     msg.pose.pose.orientation.z,
    //     msg.pose.pose.orientation.w);

    // tf2::Matrix3x3 m(q);
    // double r, p, y;
    // m.getRPY(r, p, y);

    // currentPos.theta = y;

    currentPos.x = msg.x;
    currentPos.y = msg.y;
    currentPos.theta = y;
}