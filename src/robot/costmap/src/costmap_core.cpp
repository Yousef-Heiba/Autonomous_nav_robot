#include "costmap_core.hpp"

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {}

void CostmapCore::initialize_occupancy_grid(double resolution, int width, int height) {
    resolution_ = resolution;
    width_ = width;
    height_ = height;
    // Calculate the number of cells (Meters / Resolution)
    int grid_width_cells = static_cast<int>(width / resolution);
    int grid_height_cells = static_cast<int>(height / resolution);

    // Resize the member vector and initialize with 0 (Free Space)
    occupancy_grid_.assign(grid_width_cells * grid_height_cells, 0);
}

void CostmapCore::update_occupancy_grid(const std::vector<float>&ranges, float angle_min, float angle_increment) {
    float angle{};
    float distance{};
    for (std::size_t i{0}; i < ranges.size(); i++) {
        distance = range[i];
        angle = angle_min + (i*angle_increment);
        //testing my push

    }
}

}

