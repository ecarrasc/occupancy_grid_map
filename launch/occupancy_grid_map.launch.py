from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('smb_common')

    # Paths
    smb_gazebo_launch = PathJoinSubstitution([
        FindPackageShare("smb_common"),
        "launch",
        "gazebo.launch.py"
    ])

    world_file = PathJoinSubstitution([
        FindPackageShare("smb_gazebo"),
        "worlds",
        "arena3.world"
    ])

    rviz_config = PathJoinSubstitution([
        FindPackageShare("smb_common"),
        "rviz",
        "pflocalization.launch.rviz"
    ])

    return LaunchDescription([

        # ✅ Include Gazebo launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(smb_gazebo_launch),
            launch_arguments={
                "world_file": world_file,
                "laser_enabled": "true",
                "laser_scan_min_height": "-0.2",
                "laser_scan_max_height": "1.0",
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'smb_sim_se.launch.py')
            ),
            launch_arguments={
                'rviz_config': os.path.join(pkg_share, 'rviz', 'smb_localization.rviz')
            }.items()
        ),

        # ✅ Occupancy Grid Map Node
        Node(
            package="occupancy_grid_map",
            executable="occupancy_grid_map",
            name="occupancy_grid_map",
            output="screen"
        ),

        # ✅ Teleop node
        # Node(
        #     package="pflocalizer",
        #     executable="better_teleop.py",
        #     name="better_teleop",
        #     output="screen",
        #     parameters=[{
        #         "forward_rate": 1.5,
        #         "backward_rate": 0.5,
        #         "rotation_rate": 4.5,
        #         "cmd_topic": "/cmd_vel",
        #     }]
        # ),

        # ✅ RViz
        # Node(
        #     package="rviz2",
        #     executable="rviz2",
        #     name="rviz",
        #     arguments=["-d", rviz_config],
        #     output="screen"
        # ),

        # ✅ Static transform (tf2!)
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="odom_map_broadcaster",
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        ),
    ])