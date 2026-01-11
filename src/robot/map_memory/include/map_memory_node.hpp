#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "map_memory_core.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/quaternion.hpp"


class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

  private:
    robot::MapMemoryCore map_memory_;
    
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Global map and robot position
    nav_msgs::msg::OccupancyGrid global_map_;
    double last_x = 0.0;
    double last_y = 0.0;
    double last_yaw = 0.0;
    const double distance_threshold = 5.0;
    bool costmap_updated_ = false;
    
    int gridIndex(int x, int y, int width);

    double getYawFromQuaternion(const geometry_msgs::msg::Quaternion &quat);

    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    void updateMap();

    void integrateCostmap();

    nav_msgs::msg::OccupancyGrid latest_costmap_;
    bool should_update_map_ = false;
};

#endif 