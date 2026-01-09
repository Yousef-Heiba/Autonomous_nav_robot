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
    explicit CostmapCore(const rclcpp::Logger& logger);
    void initialize_occupancy_grid(double resolution, int width, int height);
    void update_occupancy_grid(const std::vector<float>&ranges, float angle_min, float angle_increment);
    

  private:
    rclcpp::Logger logger_;
    std::vector<int8_t> occupancy_grid_;
    double resolution_;
    int height_;
    int width_;
    int inflation_radius_;
};

}  

#endif 