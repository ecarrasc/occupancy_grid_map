#ifndef OCCUPANCY_GRID_MAP_HPP
#define OCCUPANCY_GRID_MAP_HPP

#include <fstream>
#include <memory>
#include <vector>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
//#include <gazebo_msgs/msg/model_states.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/transform_broadcaster.h>

namespace occupancy_grid_map
{

struct Cell
{
    unsigned int hits = 0;
    unsigned int misses = 0;

    float occupancy() const
    {
        return hits / static_cast<double>(hits + misses);
    }

    unsigned int sum() const
    {
        return hits + misses;
    }
};

struct WorldCoord
{
    Eigen::Vector2f coord;

    WorldCoord(Eigen::Vector2f world_coord)
        : coord(world_coord) {}

    WorldCoord(float x, float y)
        : coord(x, y) {}
};

struct CellCoord
{
    Eigen::Vector2i coord;

    CellCoord(Eigen::Vector2i cell_coord)
        : coord(cell_coord) {}

    CellCoord(int x, int y)
        : coord(x, y) {}
};

class OccupancyGridMap : public rclcpp::Node
{
private:
    using Storage_t = std::vector<Cell>;

public:
    OccupancyGridMap();

    OccupancyGridMap(
        const std::string & node_name,
        float width,
        float height,
        float resolution,
        float default_value
    );

    Eigen::Vector2f get_origin() const;
    float get_resolution() const;
    Eigen::Vector2i cell_size() const;
    Eigen::Vector2f world_size() const;

    float operator()(CellCoord const& p) const;
    float operator()(WorldCoord const& p) const;

    Cell const& get_cell(int x, int y) const;

    Eigen::Vector2f cell_to_world(Eigen::Vector2i const& coord) const;
    Eigen::Vector2i world_to_cell(Eigen::Vector2f const& coord) const;

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    //void poseCallback(const gazebo_msgs::msg::ModelStates::SharedPtr msg);

    void integrate(
        const sensor_msgs::msg::LaserScan & msg,
        const Eigen::Vector3f & pose
    );

    nav_msgs::msg::OccupancyGrid publish_map() const;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr tf_timer_;

private:
    void bresenham(
        const Eigen::Vector2f & start_point,
        const Eigen::Vector2f & end_point,
        bool is_max_range
    );

    Eigen::Vector2i bound_end_cell(Eigen::Vector2i end_cell_unbounded);

    void updatePoseFromTF();

    size_t coord_to_index(const Eigen::Vector2i & coord) const;

    double normalize_angle(double angle);

    Eigen::Vector3d quaternion2rpy(double x, double y, double z, double w);

    void publishMapToOdom();

private:
    // ROS 2 interfaces
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    //rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr pose_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;

    std::string scan_topic_;
    std::string pose_topic_;
    int subscriber_queue_size_;
    std::string map_frame;

    Eigen::Vector3f current_pose;

    Storage_t m_map;
    unsigned int m_cell_width;
    unsigned int m_cell_height;

    Eigen::Affine3f m_world_pose;
    float m_resolution = 0.1;
    float m_default_value = -1.0;

    Eigen::Vector2f m_world_origin = Eigen::Vector2f::Zero();
    Eigen::Vector2i m_cell_origin = Eigen::Vector2i::Zero();

};

// Inline functions remain mostly unchanged

inline size_t OccupancyGridMap::coord_to_index(const Eigen::Vector2i & coord) const
{
    return coord[1] * m_cell_width + coord[0];
}

inline double OccupancyGridMap::normalize_angle(double angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));
}

inline Eigen::Vector3d OccupancyGridMap::quaternion2rpy(double x, double y, double z, double w)
{
    Eigen::Quaterniond q(w, x, y, z);
    Eigen::Vector3d rpy = q.toRotationMatrix().eulerAngles(0, 1, 2);
    return rpy;
}

} // namespace occupancy_grid_map

#endif // OCCUPANCY_GRID_MAP_HPP