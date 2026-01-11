#include "map_memory_node.hpp"
#include <cmath>

MapMemoryNode::MapMemoryNode() 
    : Node("map_memory_node"), 
      map_memory_(robot::MapMemoryCore(this->get_logger())),  // Move this earlier
      last_x(0.0), 
      last_y(0.0), 
      last_yaw(0.0),
      distance_threshold(1.5), 
      costmap_updated_(false), 
      should_update_map_(false),
      first_update_map_(false)
{
    // Set up the global_map_ metadata
    double resolution = 0.1; 
    global_map_.info.resolution = resolution;
    global_map_.info.width = 30.0 / resolution;
    global_map_.info.height = 30.0 / resolution;
    global_map_.info.origin.position.x = -15.0;
    global_map_.info.origin.position.y = -15.0;
    global_map_.info.origin.position.z = 0.0;
    global_map_.info.origin.orientation.w = 1.0;
    global_map_.header.frame_id = "sim_world";
    global_map_.data.resize(300 * 300, 0);
    
    // Initialize subscribers
    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

    // Initialize publisher
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

    // Initialize timer
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1), std::bind(&MapMemoryNode::updateMap, this));
    
    // Publish initial empty map so planner has something to work with
    map_pub_->publish(global_map_);
    
    RCLCPP_INFO(this->get_logger(), "Map Memory Node initialized!");
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    if (costmap_counter_ < 20) {
        costmap_counter_++;
    }
    latest_costmap_ = *msg;
    costmap_updated_ = true;
    RCLCPP_DEBUG(this->get_logger(), "Received costmap");
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double yaw = getYawFromQuaternion(msg->pose.pose.orientation);

    double distance = std::sqrt(std::pow(x - last_x, 2) + std::pow(y - last_y, 2));
    if (distance >= distance_threshold) {
        last_x = x;
        last_y = y;
        last_yaw = yaw;
        should_update_map_ = true;
        RCLCPP_INFO(this->get_logger(), "Robot moved %.2fm, will update map", distance);
    }
}

void MapMemoryNode::updateMap() {
    if (costmap_counter_ < 20) {
        RCLCPP_INFO(this->get_logger(), "Delay for costmap update on intialization");
        return;
    }
    if (first_update_map_ == false) {
        should_update_map_ = true;
        first_update_map_ = true;
    }
    if (should_update_map_ && costmap_updated_) {
        RCLCPP_INFO(this->get_logger(), "Integrating costmap at pose (%.2f, %.2f, %.2f)", 
                    last_x, last_y, last_yaw);
        integrateCostmap();
        global_map_.header.stamp = this->now();
        map_pub_->publish(global_map_);
        should_update_map_ = false;
        RCLCPP_INFO(this->get_logger(), "Published updated global map");
    }
}

int MapMemoryNode::gridIndex(int x, int y, int width) {
    return (y * width) + x;
}

double MapMemoryNode::getYawFromQuaternion(const geometry_msgs::msg::Quaternion &quat) {
    return atan2(2.0 * (quat.w * quat.z + quat.x * quat.y), 
                 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z));
}

void MapMemoryNode::integrateCostmap() {
    int costmap_width = latest_costmap_.info.width;
    int costmap_height = latest_costmap_.info.height;
    double costmap_resolution = latest_costmap_.info.resolution;
    
    int cells_updated = 0;
    int cells_unknown = 0;
    
    for (int cy = 0; cy < costmap_height; cy++) {
        for (int cx = 0; cx < costmap_width; cx++) {
            int costmap_idx = gridIndex(cx, cy, costmap_width);
            int8_t costmap_value = latest_costmap_.data[costmap_idx];
            
            // STEP 1: Grid cell → Meters (costmap local frame)
            double lx = cx * costmap_resolution + latest_costmap_.info.origin.position.x;
            double ly = cy * costmap_resolution + latest_costmap_.info.origin.position.y;
            
            // STEP 2: Costmap local → World frame
            double wx = lx * cos(last_yaw) - ly * sin(last_yaw) + last_x;
            double wy = lx * sin(last_yaw) + ly * cos(last_yaw) + last_y;
            
            // STEP 3: World → Global map grid
            int gx = (wx - global_map_.info.origin.position.x) / global_map_.info.resolution;
            int gy = (wy - global_map_.info.origin.position.y) / global_map_.info.resolution;
            
            // Bounds check and write
            if (gx >= 0 && gx < global_map_.info.width && 
                gy >= 0 && gy < global_map_.info.height) {
                
                int global_idx = gridIndex(gx, gy, global_map_.info.width);
                
                if (costmap_value != -1) {
                    global_map_.data[global_idx] = costmap_value;
                    cells_updated++;
                }
            }
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "Updated %d cells (%d were unknown)", cells_updated, cells_unknown);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapMemoryNode>());
    rclcpp::shutdown();
    return 0;
}