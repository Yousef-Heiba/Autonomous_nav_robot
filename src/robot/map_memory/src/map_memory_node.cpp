class MappingNode : public rclcpp::Node {
public:
    MappingNode() : Node("mapping_node"), last_x(0.0), last_y(0.0), distance_threshold(5.0) {
        // Initialize subscribers
        costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/costmap", 10, std::bind(&MappingNode::costmapCallback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom/filtered", 10, std::bind(&MappingNode::odomCallback, this, std::placeholders::_1));
 
        // Initialize publisher
        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
 
        // Initialize timer
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&MappingNode::updateMap, this));
    }
 
private:
    // Subscribers and Publisher  
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
 
    // Global map and robot position
    nav_msgs::msg::OccupancyGrid global_map_;
    double last_x, last_y;
    const double distance_threshold;
    bool costmap_updated_ = false;
 
    // Callback for costmap updates
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        // Store the latest costmap
        latest_costmap_ = *msg;
        costmap_updated_ = true;
    }
 
    // Callback for odometry updates
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
 
        // Compute distance traveled
        double distance = std::sqrt(std::pow(x - last_x, 2) + std::pow(y - last_y, 2));
        if (distance >= distance_threshold) {
            last_x = x;
            last_y = y;
            should_update_map_ = true;
        }
    }
 
    // Timer-based map update
    void updateMap() {
        if (should_update_map_ && costmap_updated_) {
            integrateCostmap();
            map_pub_->publish(global_map_);
            should_update_map_ = false;
        }
    }
 
    // Integrate the latest costmap into the global map
    void integrateCostmap() {
        // Transform and merge the latest costmap into the global map
        // (Implementation would handle grid alignment and merging logic)
    }
 
    // Flags
    nav_msgs::msg::OccupancyGrid latest_costmap_;
    bool should_update_map_ = false;
}