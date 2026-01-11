#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"

namespace robot
{

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger, double resolution, int height, int width, int inflation_radius);
    void initialize_occupancy_grid();
    void inflate_occupancy_grid(int x_grid, int y_grid, int max_cost, int inflation_radius);
    void update_occupancy_grid(const std::vector<float>&ranges, float angle_min, float angle_increment);
    double resolution() const;
    int height() const;
    int width() const;
    int inflation_radius() const;
    int grid_width_cells();
    int grid_height_cells();
    std::vector<int8_t> get_grid_data() const;
    

  private:
    rclcpp::Logger logger_;
    std::vector<int8_t> occupancy_grid_; //this is 1D array since the published msg has to be 1D, we will still use it as a 2D array by using math to calc the index
    double resolution_;
    int height_;
    int width_;
    int inflation_radius_;
    int grid_width_cells_; //= static_cast<int>(width() / resolution());
    int grid_height_cells_; //= static_cast<int>(height() / resolution());
};

}  

#endif 