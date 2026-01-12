#include <algorithm>
#include <chrono>

#include "control_node.hpp"

ControlNode::ControlNode() : Node("control"), control_(robot::ControlCore(this->get_logger())) {
  const double lookahead_distance = this->declare_parameter<double>("lookahead_distance", 1.0);
  const double goal_tolerance = this->declare_parameter<double>("goal_tolerance", 0.2);
  const double linear_speed = this->declare_parameter<double>("linear_speed", 0.5);
  const double max_angular_speed = this->declare_parameter<double>("max_angular_speed", 1.0);
  const double control_rate_hz = this->declare_parameter<double>("control_rate_hz", 10.0);

  control_.setLookaheadDistance(lookahead_distance);
  control_.setGoalTolerance(goal_tolerance);
  control_.setLinearSpeed(linear_speed);
  control_.setMaxAngularSpeed(max_angular_speed);

  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, control_rate_hz));
  control_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&ControlNode::timerCallback, this));
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  control_.setPath(*msg);
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  control_.setOdometry(*msg);
}

void ControlNode::timerCallback() {
  geometry_msgs::msg::Twist cmd;
  if (control_.computeCommand(cmd)) {
    cmd_vel_pub_->publish(cmd);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
