import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


# ---------------------------------------------------------------
# Overview of the frames of reference:
#   map: The global frame of reference (often used for mapping). This has ENU axes (aka east=x, north=y, up=z) and is centered on the location where we first receive a GPS lock
#   odom: Frame of reference of all filtered odometry data from the EKF. Equivalent to map frame in this project since we know the orientation relative to true east.
#   base_link: The coordinate frame centered on the physical body of the robot centered at the GPS antenna. This has FLU axes (aka forward=x, left=y, up=z)
#   depth_camera_link: TODO TODO TODO TODO
#
#   Note: In other projects map may be fixed center global orientation whereas odom may be fixed center local orientation if we don't know how to convert between local and global orientation.
#         Thankfully, our compass gives us this information, so we don't have to fret and have more frames of reference.
# ---------------------------------------------------------------


# TODO: We need a transform node to transform to the depth_camera_link aka the depth cameras frame of reference.
# Basically we need to figure out the relative position of the depth camera to the GPS antenna


def generate_launch_description():
    pkg_share = get_package_share_directory("robot_navigation_launch")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")


    ekf_params_file = os.path.join(pkg_share, "config", "ekf.yaml")
    navsat_transform_params_file = os.path.join(pkg_share, "config", "navsat_transform.yaml")
    nav2_params_file = os.path.join(pkg_share, "config", "nav2.yaml")


    microcontroller_node = Node(
        package="microcontroller_communicator",
        executable="microcontroller_communicator",  # Or entry_point name in setup.py
        name="microcontroller_communicator",
        output="screen",
        emulate_tty=True,
        respawn_delay=0.5
    )


    # Literally just says that map frame = odometry frame (aka identity transform)
    map_to_odom_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom_tf",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "map", "odom"],
        respawn_delay=0.5
    )


    # NavSat Transform Node (converts /gps/fix + /imu -> /odometry/gps)
    # /odometry/gps is in the odom frame of reference instead of in longitude/ latitude
    navsat_transform_node = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform",
        output="screen",
        parameters=[navsat_transform_params_file],
        respawn_delay=0.5
    )


    # EKF Node (fuses IMU + GPS Odometry into /odometry/filtered)
    ekf_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_params_file],
        respawn_delay=0.5
    )




    # nav2_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, "launch", "navigation_launch.py")),
    #     launch_arguments={
    #         "use_sim_time": "False",
    #         "params_file": nav2_params_file,
    #         "autostart": "True",
    #     }.items(),
    # )

    return LaunchDescription([
        microcontroller_node,
        map_to_odom_transform,
        navsat_transform_node,
        ekf_localization_node,
        # nav2_launch
    ])
