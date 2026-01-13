#include "costmap_core.hpp"

namespace robot {

CostmapCore::CostmapCore(const rclcpp::Logger& logger, double resolution, int height, int width, double inflation_radius):
 logger_(logger),
 resolution_{resolution},
 height_{height},
 width_{width},
 inflation_radius_{inflation_radius} {
}

    double CostmapCore::resolution() const {
        return resolution_;
    }

    int CostmapCore::height() const {
        return height_;
    }

    int CostmapCore::width() const {
        return width_;
    }

    double CostmapCore::inflation_radius() const {
        return inflation_radius_;
    }

    int CostmapCore::grid_width_cells() {
        grid_width_cells_ = static_cast<int>(width() / resolution());
        return grid_width_cells_;
    }


    int CostmapCore::grid_height_cells() {
        grid_height_cells_ = static_cast<int>(height() / resolution());
        return grid_height_cells_;
    }
    
    std::vector<int8_t> CostmapCore::get_grid_data() const {
        return occupancy_grid_;
    }


void CostmapCore::initialize_occupancy_grid() {
    // Calculate the number of cells (Meters / Resolution)

    // Resize the member vector and initialize with 0 (Free Space)
    occupancy_grid_.assign(grid_width_cells() * grid_height_cells(), 0);
}

void CostmapCore::inflate_occupancy_grid(int x_grid, int y_grid, int max_cost, double inflation_radius) {
    int radius_cells = static_cast<int>(inflation_radius / resolution());
    for (int dy = -radius_cells; dy <= (radius_cells); dy++) {
        for (int dx = -radius_cells; dx <= (radius_cells); dx++) {
            double euc_distance = std::sqrt(dx*dx + dy*dy);
            int nx = x_grid + dx;
            int ny = y_grid + dy;
            if (radius_cells > euc_distance) {
                if ((nx >= 0 && nx <grid_width_cells()) && (ny >= 0 && ny < grid_height_cells())) {
                    int curr_index = (ny * grid_width_cells()) + nx;
                    int curr_cost = occupancy_grid_[curr_index];
                    int calc_cost = max_cost * (1 - (euc_distance / radius_cells));
                    if (calc_cost > curr_cost) {
                    occupancy_grid_[curr_index] = calc_cost;
                    }
                }
            }
        }
    }
}

void CostmapCore::update_occupancy_grid(const std::vector<float>&ranges, float angle_min, float angle_increment) {
    initialize_occupancy_grid();
    double angle{};
    double distance{};
    double x_distance{0};
    double y_distance{0};
    int max_cost{100};
    int x_grid{0};
    int y_grid{0};
    int origin_x = static_cast<int>( grid_width_cells() / 2);
    int origin_y = static_cast<int>( grid_height_cells() / 2);
    for (std::size_t i{0}; i < ranges.size(); i++) {    
        distance = ranges[i];
        angle = angle_min + (i*angle_increment);
        if (std::isinf(distance) != true) {
            //calculate the x and y distance of the oject
            x_distance = distance * std::cos(angle);
            y_distance = distance * std::sin(angle);
            //convert the distance to grid and add it to the occpuancy grid 
            x_grid = origin_x + static_cast<int>((x_distance / resolution())); 
            y_grid = origin_y + static_cast<int>((y_distance / resolution()));
        
            // Added to occupancy grid as occupied
            if ((x_grid >= 0 && x_grid < grid_width_cells()) && y_grid >= 0 && y_grid < grid_height_cells()) { 
                int index = (y_grid * grid_width_cells()) + x_grid;    
                occupancy_grid_[index] = max_cost;
            }   
        
            //now i will need to inflate the object
            inflate_occupancy_grid(x_grid, y_grid, 100, inflation_radius());
        }
    }

}

}

