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
    // Store map information
    // resolution_, origin_x_, origin_y_, width_, height_
    
    // Convert start and goal to grid coordinates
    
    // Validate start and goal cells
    
    // OPEN - set of nodes to be evaluated (priority queue)
    // CLOSED - set of nodes already evaluated (unordered_set)
    // add start node to OPEN
    
    // loop
        // current = node in OPEN with lowest f_cost
        // remove current from OPEN
        // add current to CLOSED
        
        // if current is the target node, path has been found
            // return
        
        // foreach neighbour of current node
            // if neighbour is not traversable or neighbour is in CLOSED
                // skip to next neighbour
            
            // if new path to neighbour is shorter OR neighbour is not in OPEN
                // set f_cost of neighbour
                // set parent of neighbour to current
                // if neighbour is not in OPEN
                    // add neighbour to OPEN
    
    // Check if path was found
    
    // Reconstruct path from start to goal
    
    // Convert path cells to world coordinates (PoseStamped)
    
    // Return path
    
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