#include "occupancy_grid_map/occupancy_grid_map.hpp"

#include <cmath>
#include <iostream>
#include <functional>

namespace occupancy_grid_map
{

OccupancyGridMap::OccupancyGridMap()
: Node("occupancy_grid_map")
{
    // Default constructor if needed
}

OccupancyGridMap::OccupancyGridMap(
    const std::string & node_name,
    float width,
    float height,
    float resolution,
    float default_value
)
: Node(node_name)
, subscriber_queue_size_(10)
, scan_topic_("/scan")
, pose_topic_("/gazebo/model_states")
, map_frame("map")
, m_map((width / resolution) * (height / resolution))
, m_cell_width(width / resolution)
, m_cell_height(height / resolution)
, m_resolution(resolution)
, m_default_value(default_value)
, m_world_origin(Eigen::Vector2f(-width/2, -height/2))
, m_cell_origin(Eigen::Vector2i(0, 0))
{
    // Subscribers
    scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_,
        subscriber_queue_size_,
        std::bind(&OccupancyGridMap::scanCallback, this, std::placeholders::_1)
    );

    // pose_subscriber_ = this->create_subscription<gazebo_msgs::msg::ModelStates>(
    //     pose_topic_,
    //     subscriber_queue_size_,
    //     std::bind(&OccupancyGridMap::poseCallback, this, std::placeholders::_1)
    // );

    // Publisher
    map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/occupancy_grid", 1
    );

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    current_pose = {0, 0, 0};

    RCLCPP_INFO(this->get_logger(), "Initialized OccupancyGridMap");
}

float OccupancyGridMap::get_resolution() const
{
    return m_resolution;
}

Eigen::Vector2i OccupancyGridMap::cell_size() const
{
    return Eigen::Vector2i(m_cell_width, m_cell_height);
}

Eigen::Vector2f OccupancyGridMap::world_size() const
{
    return Eigen::Vector2f(
        m_cell_width * m_resolution,
        m_cell_height * m_resolution
    );
}

float OccupancyGridMap::operator()(CellCoord const& p) const
{
    auto cell = m_map[p.coord[1] * m_cell_width + p.coord[0]];
    return (cell.sum() > 0) ? cell.occupancy() : m_default_value;
}

float OccupancyGridMap::operator()(WorldCoord const& p) const
{
    return operator()(CellCoord(world_to_cell(p.coord)));
}

Cell const& OccupancyGridMap::get_cell(int x, int y) const
{
    return m_map[y * m_cell_width + x];
}

Eigen::Vector2f OccupancyGridMap::cell_to_world(Eigen::Vector2i const& coord) const
{
    return m_world_origin + ((coord - m_cell_origin).cast<float>() * m_resolution);
}

Eigen::Vector2i OccupancyGridMap::world_to_cell(Eigen::Vector2f const& coord) const
{
    return ((coord - m_world_origin) / m_resolution).cast<int>() + m_cell_origin;
}

void OccupancyGridMap::scanCallback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg
)
{
    updatePoseFromTF();

    integrate(*msg, current_pose);

    //auto nav_msg = publish_map();
    //(void)nav_msg; // suppress unused warning if needed

    publish_map();
}

void OccupancyGridMap::updatePoseFromTF()
{
    try
    {
        auto transform = tf_buffer_->lookupTransform(
            "map",        // target frame
            "base_link",  // robot frame (or smb base)
            tf2::TimePointZero
        );

        double x = transform.transform.translation.x;
        double y = transform.transform.translation.y;

        auto q = transform.transform.rotation;

        Eigen::Vector3d rpy = quaternion2rpy(q.x, q.y, q.z, q.w);

        current_pose[0] = x;
        current_pose[1] = y;
        current_pose[2] = rpy[2];
    }
    catch (const tf2::TransformException & ex)
    {
        RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
    }
}

// void OccupancyGridMap::poseCallback(
//     const gazebo_msgs::msg::ModelStates::SharedPtr msg
// )
// {
//     int smb_index = -1;

//     for (size_t i = 0; i < msg->name.size(); i++)
//     {
//         if (msg->name[i] == "smb")
//             smb_index = i;
//     }

//     if (smb_index < 0)
//     {
//         RCLCPP_WARN(this->get_logger(), "SMB model not found in ModelStates");
//         return;
//     }

//     double x = msg->pose[smb_index].position.x;
//     double y = msg->pose[smb_index].position.y;

//     double quat_x = msg->pose[smb_index].orientation.x;
//     double quat_y = msg->pose[smb_index].orientation.y;
//     double quat_z = msg->pose[smb_index].orientation.z;
//     double quat_w = msg->pose[smb_index].orientation.w;

//     Eigen::Vector3d rpy = quaternion2rpy(quat_x, quat_y, quat_z, quat_w);

//     current_pose[0] = x;
//     current_pose[1] = y;
//     current_pose[2] = rpy[2];
// }

void OccupancyGridMap::integrate(
    const sensor_msgs::msg::LaserScan & msg,
    const Eigen::Vector3f & pose
)
{
    for (size_t i = 0; i < msg.ranges.size(); ++i)
    {
        //auto angle = normalize_angle(
            auto angle = pose[2] - msg.angle_min + msg.angle_increment * i;// + M_PI
        //);

        auto range = msg.ranges[i];

        if (!std::isinf(range))
        {
            auto end_point = Eigen::Vector2f(
                pose[0] + range * std::cos(angle),
                pose[1] + range * std::sin(angle)
            );

            bresenham(
                {pose[0], pose[1]},
                end_point,
                std::abs(range - msg.range_max) < 0.1
            );
        }
    }
}

Eigen::Vector2i OccupancyGridMap::bound_end_cell(
    Eigen::Vector2i end_cell_unbounded
)
{
    auto end_cell = end_cell_unbounded;

    if (end_cell[0] < 0) end_cell[0] = 0;
    if (end_cell[1] < 0) end_cell[1] = 0;

    if (end_cell[0] >= m_cell_width) end_cell[0] = m_cell_width - 1;
    if (end_cell[1] >= m_cell_height) end_cell[1] = m_cell_height - 1;

    return end_cell;
}

void OccupancyGridMap::bresenham(
    const Eigen::Vector2f & start_point,
    const Eigen::Vector2f & end_point,
    bool is_max_range
)
{
    auto start_cell = world_to_cell(start_point);
    auto end_cell_unbounded = world_to_cell(end_point);

    auto end_cell = bound_end_cell(end_cell_unbounded);
    if (end_cell != end_cell_unbounded)
        is_max_range = true;

    auto dx = std::abs(end_cell[0] - start_cell[0]);
    auto dy = std::abs(end_cell[1] - start_cell[1]);

    auto x = start_cell[0];
    auto y = start_cell[1];

    auto step_x = (start_cell[0] > end_cell[0]) ? -1 : 1;
    auto step_y = (start_cell[1] > end_cell[1]) ? -1 : 1;

    if (dx > dy)
    {
        auto error = dx / 2.0;
        while (x != end_cell[0])
        {
            m_map[coord_to_index({x, y})].misses++;
            error -= dy;
            if (error < 0)
            {
                y += step_y;
                error += dx;
            }
            x += step_x;
        }
    }
    else
    {
        auto error = dy / 2.0;
        while (y != end_cell[1])
        {
            m_map[coord_to_index({x, y})].misses++;
            error -= dx;
            if (error < 0)
            {
                x += step_x;
                error += dy;
            }
            y += step_y;
        }
    }

    if (!is_max_range)
        m_map[coord_to_index({x, y})].hits++;
    else
        m_map[coord_to_index({x, y})].misses++;
}

nav_msgs::msg::OccupancyGrid OccupancyGridMap::publish_map() const
{
    nav_msgs::msg::OccupancyGrid msg;

    msg.header.frame_id = map_frame;
    msg.header.stamp = this->now();

    msg.info.resolution = m_resolution;
    msg.info.width = m_cell_width;
    msg.info.height = m_cell_height;

    msg.info.origin.position.x = m_world_origin[0];
    msg.info.origin.position.y = m_world_origin[1];
    msg.info.origin.position.z = 0.0;

    for (size_t i = 0; i < m_map.size(); ++i)
    {
        if (m_map[i].sum() == 0)
            msg.data.push_back(-1);
        else
            msg.data.push_back(static_cast<int8_t>(m_map[i].occupancy() * 100.0));
    }

    map_publisher_->publish(msg);

    return msg;
}

} // namespace occupancy_grid_map