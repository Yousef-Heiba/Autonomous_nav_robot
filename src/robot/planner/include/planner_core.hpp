#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <algorithm>

// ------------------- Supporting Structures ------------------- //
struct CellIndex {
    int x;
    int y;
    
    CellIndex(int xx, int yy) : x(xx), y(yy) {}
    CellIndex() : x(0), y(0) {}
    
    bool operator==(const CellIndex &other) const {
        return (x == other.x && y == other.y);
    }
    
    bool operator!=(const CellIndex &other) const {
        return (x != other.x || y != other.y);
    }
};

struct CellIndexHash {
    std::size_t operator()(const CellIndex &idx) const {
        return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
    }
};

struct AStarNode {
    CellIndex index;
    double f_score;
    
    AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

struct CompareF {
    bool operator()(const AStarNode &a, const AStarNode &b) {
        return a.f_score > b.f_score;
    }
};

namespace robot
{

class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger& logger);

    // Main A* planning function
    std::vector<geometry_msgs::msg::PoseStamped> planPath(
        const nav_msgs::msg::OccupancyGrid& map,
        const geometry_msgs::msg::Pose& start,
        const geometry_msgs::msg::Pose& goal
    );

  private:
    rclcpp::Logger logger_;
    
    // Map information
    double resolution_;
    double origin_x_;
    double origin_y_;
    int width_;
    int height_;
    
    // Coordinate conversion
    CellIndex worldToGrid(double world_x, double world_y) const;
    void gridToWorld(const CellIndex& cell, double& world_x, double& world_y) const;
    
    // Validation
    bool isValidCell(const CellIndex& cell, const nav_msgs::msg::OccupancyGrid& map) const;
    
    // A* helpers
    double calculateHeuristic(const CellIndex& from, const CellIndex& to) const;
    std::vector<CellIndex> getNeighbors(const CellIndex& cell) const;
    double getMovementCost(const CellIndex& from, const CellIndex& to) const;
    std::vector<CellIndex> reconstructPath(
        const CellIndex& start,
        const CellIndex& goal,
        const std::unordered_map<CellIndex, CellIndex, CellIndexHash>& came_from
    ) const;
};

}  

#endif  // PLANNER_CORE_HPP_ 