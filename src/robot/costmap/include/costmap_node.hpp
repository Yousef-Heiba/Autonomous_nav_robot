#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_
 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
 
#include "costmap_core.hpp"
 
class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode(); //this is the constructor
    
    // Place callback function here
    void publishMessage();
 
  private:
    // callback function for the lidar topic
    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr message);
    robot::CostmapCore costmap_;
    // Place these constructs here
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr costmap_lidar_;
    rclcpp::TimerBase::SharedPtr timer_;
    // Publisher for the costmap
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_publisher_;
    // Function to bundle data and publish
    void publish_costmap();
};
 
#endif 