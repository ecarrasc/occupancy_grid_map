#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "occupancy_grid_map/occupancy_grid_map.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<occupancy_grid_map::OccupancyGridMap>(
        "occupancy_grid_map",   // node name
        100.0,                 // width
        100.0,                 // height
        0.1,                   // resolution
        0.5                    // default value
    );

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}