# FIXME: This launch file is for debugging purpose ONLY. Never use this on the integrated vehicle.
# FIXME: FOR LAUNCHING THE INTEGRATED SOFTWARE STACK, USE THE LAUNCH FILE IN IAC_LAUNCH/LAUNCH.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python import get_package_share_directory
import os

def generate_launch_description():
    # since this is a very basic package, including any launch helper function
    # is not encouraged (as it couples with other higher-level packages).
    # Therefore, we choose to explicitly declare the launch arguments here.

    launch_desc_list = [
        # whether we are running a simulation or on a physical vehicle is controlled
        # through this launch argument.
        DeclareLaunchArgument(
            "use_sim_time", default_value="False", description="Use simulation clock if True"
        )
    ]

    launch_arg_dict = {"use_sim_time": LaunchConfiguration("use_sim_time")}

    # LVMS setup, no path offseting because we do not have a boundry file
    # initial path file to be loaded when the stack starts. Use pit lane here.
    # pathFile = os.path.join(get_package_share_directory("path_server_overhaul"),
    #                     "maps", "LVMS_SVL_Pitlane.csv")
    # the path to be switched into (race line)
    # altPathFile = os.path.join(get_package_share_directory("path_server_overhaul"),
    #                     "maps", "LVMS_SVL_center_line.csv")
    # left boundaries
    # leftBoundaryPath = os.path.join(get_package_share_directory("path_server_overhaul"),
    #                     "maps", "LVMS_SVL_Pitlane.csv")
    # altLeftBoundaryPath = os.path.join(get_package_share_directory("path_server_overhaul"),
    #                     "maps", "LVMS_SVL_center_line.csv")
    # right boundaries
    # rightBoundaryPath = os.path.join(get_package_share_directory("path_server_overhaul"),
    #                     "maps", "LVMS_SVL_Pitlane.csv")
    # altRightBoundaryPath = os.path.join(get_package_share_directory("path_server_overhaul"),
    #                     "maps", "LVMS_SVL_center_line.csv")

    # IMS setup
    # initial path file to be loaded when the stack starts. Use pit lane here.
    pathFile = os.path.join(get_package_share_directory("path_server_overhaul"),
                        "maps", "IMS_SVL_Road_crs.csv")
    # the path to be switched into (race line)
    altPathFile = os.path.join(get_package_share_directory("path_server_overhaul"),
                        "maps", "IMS_SVL_Road_crs.csv")
    # left boundaries
    leftBoundaryPath = os.path.join(get_package_share_directory("path_server_overhaul"),
                        "maps", "IMS_SVL_Road_crs.csv")
    altLeftBoundaryPath = os.path.join(get_package_share_directory("path_server_overhaul"),
                        "maps", "IMS_SVL_Road_crs.csv")
    # right boundaries
    rightBoundaryPath = os.path.join(get_package_share_directory("path_server_overhaul"),
                        "maps", "IMS_SVL_Road_crs.csv")
    altRightBoundaryPath = os.path.join(get_package_share_directory("path_server_overhaul"),
                        "maps", "IMS_SVL_Road_crs.csv")

    pathServerNode = Node(
        package="path_server_overhaul",
        executable="path_server_node",
        name="path_server_overhaul",
        namespace="planning",
        output="screen",
        parameters=[
            {
                "path_file_reference_frame": "earth",
                "target_reference_frame": "base_link",
                "init_path_file_name": pathFile,
                "alt_path_file_name": altPathFile,
                "init_left_bound_file_name": leftBoundaryPath,
                "alt_left_bound_file_name": altLeftBoundaryPath,
                "init_right_bound_file_name": rightBoundaryPath,
                "alt_right_bound_file_name": altRightBoundaryPath,
                "on_receiving_new_path": "replace",
                "look_ahead_distance": 100.0,
                "look_behind_distance": 50.0,
                "is_closed_path": True,
                "interpret_yaw": True,
                "perform_interpolation": True,
                "interpolation_step_length": 1.0,
                "lateral_offset_rate_limit": 0.1,
                "front_path_topic": "front_path",
                "rear_path_topic": "rear_path",
            },
            launch_arg_dict,
        ],
        remappings=[
            ("/planning/errors", "/vehicle/errors"),
        ]
    )

    launch_desc_list.append(pathServerNode)

    return LaunchDescription(launch_desc_list)
