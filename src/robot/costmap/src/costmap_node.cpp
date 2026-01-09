#include <chrono>
#include <memory>
 
#include "costmap_node.hpp"
 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Subscribe to the lidar topic
  costmap_lidar_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar",
    10,
    std::bind(&CostmapNode::lidar_callback, this, std::placeholders::_1)
  );
  // Initialize the constructs and their parameters
  string_pub_ = this->create_publisher<std_msgs::msg::String>("/test_topic", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishMessage, this));
}

// Define the lidar callback function
void CostmapNode::lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr message) {
//here i will need to update the costmap from the lidar
//i will also need to publish the costmap
}
 
// Define the timer to publish a message every 500ms
void CostmapNode::publishMessage() {
  auto message = std_msgs::msg::String();
  message.data = "Hello, ROS 2!";
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  string_pub_->publish(message);
}
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}