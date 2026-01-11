#include <chrono>
#include <memory>
 
#include "costmap_node.hpp"
 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger(), 0.1, 30, 30, 1)) {
  // Subscribe to the lidar topic
  costmap_lidar_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar",
    10,
    std::bind(&CostmapNode::lidar_callback, this, std::placeholders::_1)
  );
  costmap_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  // Initialize the constructs and their parameters
  string_pub_ = this->create_publisher<std_msgs::msg::String>("/test_topic", 10);
  //timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishMessage, this));
}

void CostmapNode::publish_costmap() {
  auto message = nav_msgs::msg::OccupancyGrid();

    // Fill in the Header
  message.header.stamp = this->now();
  message.header.frame_id = "robot/chassis/lidar"; 

    // Fill in the Meta-Data (Map Info)
  message.info.resolution = 0.1; 
  message.info.width = costmap_.grid_width_cells(); // You need a getter for this in Core
  message.info.height = costmap_.grid_height_cells();

    // Define the Origin 
    // Since we centered the robot, the map origin is actually negative
  message.info.origin.position.x = -(costmap_.grid_width_cells() * 0.1) / 2.0;
  message.info.origin.position.y = -(costmap_.grid_height_cells() * 0.1) / 2.0;
  message.info.origin.position.z = 0.0;
  message.info.origin.orientation.w = 1.0;

    // Fill in the Data
    // We need a getter in CostmapCore that returns the "occupancy_grid_" vector
  message.data = costmap_.get_grid_data(); 

    // Send the message
  costmap_publisher_->publish(message);
}


// Define the lidar callback function
void CostmapNode::lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
//here i will need to update the costmap from the lidar, so run the fuction update_occupancy_grid_
  costmap_.update_occupancy_grid(msg->ranges, msg->angle_min, msg->angle_increment);
//i will also need to publish the costmap
  publish_costmap();
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