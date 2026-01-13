#include "planner_core.hpp"

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger)
    : logger_(logger),
      resolution_(0.0),
      origin_x_(0.0),
      origin_y_(0.0),
      width_(0),
      height_(0)
{
    RCLCPP_INFO(logger_, "PlannerCore initialized");
}

// ------------------- Main Planning Function ------------------- //

std::vector<geometry_msgs::msg::PoseStamped> PlannerCore::planPath(
    const nav_msgs::msg::OccupancyGrid& map,
    const geometry_msgs::msg::Pose& start,
    const geometry_msgs::msg::Pose& goal)
{
    // 1. Store map information
    resolution_ = map.info.resolution;
    origin_x_ = map.info.origin.position.x;
    origin_y_ = map.info.origin.position.y;
    width_ = map.info.width;
    height_ = map.info.height;
    
    // 2. Convert start and goal to grid coordinates
    CellIndex start_cell = worldToGrid(start.position.x, start.position.y);
    CellIndex goal_cell = worldToGrid(goal.position.x, goal.position.y);
    
    // 3. Validate start and goal cells
    if (!isValidCell(start_cell, map) || !isValidCell(goal_cell, map)) {
        RCLCPP_WARN(logger_, "Start or Goal cell is invalid/occupied!");
        return std::vector<geometry_msgs::msg::PoseStamped>();
    }
    
    // 4. Setup A* Data Structures
    // OPEN SET: Priority Queue (Min-Heap by f_score)
    std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
    
    // G_SCORE: Cost from start to current node (Default infinity)
    std::unordered_map<CellIndex, double, CellIndexHash> g_score;
    
    // CAME_FROM: To reconstruct the path later
    std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
    
    // Initialize Start Node
    g_score[start_cell] = 0.0;
    double start_f = calculateHeuristic(start_cell, goal_cell);
    open_set.push(AStarNode(start_cell, start_f));
    
    // 5. Main A* Loop
    while (!open_set.empty()) {
        // Get node with lowest f_score
        CellIndex current = open_set.top().index;
        open_set.pop();
        
        // Check if goal reached
        if (current == goal_cell) {
            // Reconstruct path
            std::vector<CellIndex> path_indices = reconstructPath(start_cell, goal_cell, came_from);
            
            // Convert to World Coordinates (PoseStamped)
            std::vector<geometry_msgs::msg::PoseStamped> path_output;
            for (const auto& cell : path_indices) {
                geometry_msgs::msg::PoseStamped pose;
                pose.header.frame_id = "map"; // Ensure this matches your map frame
                
                double world_x, world_y;
                gridToWorld(cell, world_x, world_y);
                
                pose.pose.position.x = world_x;
                pose.pose.position.y = world_y;
                pose.pose.position.z = 0.0;
                pose.pose.orientation.w = 1.0;
                
                path_output.push_back(pose);
            }
            RCLCPP_INFO(logger_, "Path found with %zu steps", path_output.size());
            return path_output;
        }
        
        // Expand Neighbors
        std::vector<CellIndex> neighbors = getNeighbors(current);
        for (const auto& neighbor : neighbors) {
            
            // Skip if obstacle
            if (!isValidCell(neighbor, map)) {
                continue;
            }
            
            // Calculate tentative g_score
            double move_cost = getMovementCost(current, neighbor);
            double tentative_g = g_score[current] + move_cost;
            
            // If this path to neighbor is better than any previous one
            if (g_score.find(neighbor) == g_score.end() || tentative_g < g_score[neighbor]) {
                
                // Update tracking info
                came_from[neighbor] = current;
                g_score[neighbor] = tentative_g;
                double f_score = tentative_g + calculateHeuristic(neighbor, goal_cell);
                
                // Add to Open Set
                open_set.push(AStarNode(neighbor, f_score));
            }
        }
    }
    
    RCLCPP_WARN(logger_, "Failed to find a path!");
    return std::vector<geometry_msgs::msg::PoseStamped>();
}
// ------------------- Coordinate Conversion ------------------- //

CellIndex PlannerCore::worldToGrid(double world_x, double world_y) const
{
    // Convert world coordinates to grid indices
    
    int grid_x = static_cast<int>((world_x - origin_x_) / resolution_);
    int grid_y = static_cast<int>((world_y - origin_y_) / resolution_);
    
    return CellIndex(grid_x, grid_y);
}

void PlannerCore::gridToWorld(const CellIndex& cell, double& world_x, double& world_y) const
{
    // Convert grid indices to world coordinates
    // Formula: world_coord = (cell_index * resolution) + origin
    // Add 0.5 to get the center of the cell
    
    world_x = (cell.x + 0.5) * resolution_ + origin_x_;
    world_y = (cell.y + 0.5) * resolution_ + origin_y_;
}

// ------------------- Validation ------------------- //

bool PlannerCore::isValidCell(const CellIndex& cell, const nav_msgs::msg::OccupancyGrid& map) const
{
    // Check if cell is within map bounds
    if (cell.x < 0 || cell.x >= width_ || cell.y < 0 || cell.y >= height_) {
        return false;
    }
    
    // Check if cell is not occupied
    int index = cell.y * width_ + cell.x;
    
    // Occupancy values: -1 = unknown, 0 = free, 100 = occupied
    int8_t occupancy = map.data[index];
    
    if (occupancy < 0 || occupancy > 50) {
        return false;  // Unknown or occupied, also adjustable
    }
    
    return true;  // Free space
}

// ------------------- A* Algorithm Helpers ------------------- //

double PlannerCore::calculateHeuristic(const CellIndex& from, const CellIndex& to) const
{
    // Calculate Euclidean distance between two cells
    
    int dx = to.x - from.x;
    int dy = to.y - from.y;
    
    return std::sqrt(dx * dx + dy * dy);
}

std::vector<CellIndex> PlannerCore::getNeighbors(const CellIndex& cell) const
{
    // Return all 8 neighbors
    // Order: N, NE, E, SE, S, SW, W, NW
    
    std::vector<CellIndex> neighbors;
    
    // All 8 possible directions
    static const int dx[] = {0, 1, 1, 1, 0, -1, -1, -1};
    static const int dy[] = {1, 1, 0, -1, -1, -1, 0, 1};
    
    for (int i = 0; i < 8; ++i) {
        int new_x = cell.x + dx[i];
        int new_y = cell.y + dy[i];
        neighbors.push_back(CellIndex(new_x, new_y));
    }
    
    return neighbors;
}

double PlannerCore::getMovementCost(const CellIndex& from, const CellIndex& to) const
{
    // Calculate the cost to move from one cell to another
    // Straight moves cost 1.0
    // Diagonal moves cost sqrt(2) ≈ 1.414
    
    int dx = std::abs(to.x - from.x);
    int dy = std::abs(to.y - from.y);
    
    // If both dx and dy are 1, it's a diagonal move
    if (dx == 1 && dy == 1) {
        return std::sqrt(2.0);  // Diagonal cost
    }
    
    // Otherwise it's a straight move (dx + dy should be 1)
    return 1.0;  // Straight cost
}

std::vector<CellIndex> PlannerCore::reconstructPath(
    const CellIndex& start,
    const CellIndex& goal,
    const std::unordered_map<CellIndex, CellIndex, CellIndexHash>& came_from) const
{
    // Reconstruct the path by following parent pointers from goal to start
    // Then reverse it to get path from start to goal
    
    std::vector<CellIndex> path;
    CellIndex current = goal;
    
    // Follow the parent chain from goal back to start
    while (current != start) {
        path.push_back(current);
        
        // Look up the parent of current node
        auto it = came_from.find(current);
        if (it == came_from.end()) {
            // This shouldn't happen if A* completed successfully
            RCLCPP_ERROR(logger_, "Error reconstructing path: broken parent chain");
            return std::vector<CellIndex>();
        }
        
        current = it->second;
    }
    
    // Add the start cell
    path.push_back(start);
    
    // Reverse the path so it goes from start to goal
    std::reverse(path.begin(), path.end());
    
    return path;
}

}  // namespace robot