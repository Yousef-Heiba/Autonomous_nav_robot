#include "planner_node.hpp"

PlannerNode::PlannerNode() 
    : Node("planner_node"), 
      state_(State::WAITING_FOR_GOAL),
      planner_(this->get_logger()),
      goal_received_(false)
{
    // Subscribers
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
    
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
    
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));
    
    // Publisher
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
    
    // Timer (500ms period)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500), 
        std::bind(&PlannerNode::timerCallback, this));
    
    RCLCPP_INFO(this->get_logger(), "Planner Node initialized - waiting for goal");
}

// ------------------- Callbacks ------------------- //

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    // Store the current map
    current_map_ = *msg;
    
    // If we're currently navigating to a goal, replan when map updates
    if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
        RCLCPP_DEBUG(this->get_logger(), "Map updated - replanning path");
        planPath();
    }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    // Store the new goal
    goal_ = *msg;
    goal_received_ = true;
    
    // Transition to navigating state
    state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
    
    RCLCPP_INFO(this->get_logger(), "New goal received at (%.2f, %.2f)", 
                goal_.point.x, goal_.point.y);
    
    // Plan path to the new goal
    planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // Simply store the current robot pose
    robot_pose_ = msg->pose.pose;
}

void PlannerNode::timerCallback()
{
    // Only check if we're currently navigating to a goal
    if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
        if (goalReached()) {
            RCLCPP_INFO(this->get_logger(), "Goal reached!");
            state_ = State::WAITING_FOR_GOAL;
        } else {
            // Add timeout logic here
            // For now, just rely on map updates triggering replanning
            RCLCPP_DEBUG(this->get_logger(), "Replanning due to timeout or progress...");
            planPath();
        }
    }
}

// ------------------- Helper Functions ------------------- //

bool PlannerNode::goalReached()
{
    // Return true if distance < threshold (e.g., 0.5 meters)
    
    double dx = goal_.point.x - robot_pose_.position.x;
    double dy = goal_.point.y - robot_pose_.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);
    
    double threshold = 0.5;  // adjustable
    
    return distance < threshold;
}

void PlannerNode::planPath()
{
    // Check if we have all necessary data
    if (!goal_received_ || current_map_.data.empty()) {
        RCLCPP_WARN(this->get_logger(), "Cannot plan path: Missing map or goal!");
        return;
    }
    
    // TODO: Call the planner core to generate the path
    // Steps:
    // 1. Call planner_.planPath(current_map_, robot_pose_, goal_.point)
    //    This should return a std::vector<geometry_msgs::msg::PoseStamped>
    // 2. Check if the path is empty (planning failed)
    // 3. If successful, create a nav_msgs::msg::Path message
    // 4. Fill in the header (timestamp and frame_id)
    // 5. Set path.poses to the vector returned from planner
    // 6. Publish the path
    
    RCLCPP_INFO(this->get_logger(), "Planning path from (%.2f, %.2f) to (%.2f, %.2f)",
                robot_pose_.position.x, robot_pose_.position.y,
                goal_.point.x, goal_.point.y);
    
    // Call the A* planner
    // Note: You'll need to convert goal_.point to a Pose for the planner
    geometry_msgs::msg::Pose goal_pose;
    goal_pose.position.x = goal_.point.x;
    goal_pose.position.y = goal_.point.y;
    goal_pose.position.z = 0.0;
    goal_pose.orientation.w = 1.0;  // Default orientation
    
    std::vector<geometry_msgs::msg::PoseStamped> path_poses = 
        planner_.planPath(current_map_, robot_pose_, goal_pose);
    
    if (path_poses.empty()) {
        RCLCPP_WARN(this->get_logger(), "Planning failed - no valid path found!");
        return;
    }
    
    // Create and publish the path message
    nav_msgs::msg::Path path;
    path.header.stamp = this->get_clock()->now();
    path.header.frame_id = "sim_world";  // Should match the map frame
    path.poses = path_poses;
    
    path_pub_->publish(path);
    
    RCLCPP_INFO(this->get_logger(), "Published path with %zu waypoints", path_poses.size());
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}