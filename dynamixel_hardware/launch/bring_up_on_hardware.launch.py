import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

import xacro

# User settings
use_gazebo = "false"
single_arm = "false"
controller_name = (
    "position_trajectory_controller"  # Select controller from controller_list
)

# Path setting for xacro, urdf, rviz, and world files
spacediver_ros2_control_pkg_dir = get_package_share_directory(
    "spacediver_ros2_control")
spacediver_urdf_xacro_path = os.path.join(
    spacediver_ros2_control_pkg_dir,
    "description", "urdf", "spacediver.urdf.xacro"
)
rviz_path = os.path.join(
    spacediver_ros2_control_pkg_dir,
    "description", "rviz", "spacediver_config.rviz")

pkg_dir = get_package_share_directory("dynamixel_hardware")
controller_config = os.path.join(
    pkg_dir, "config", "dynamixel_controllers.yaml"
)


def generate_launch_description():
    # Parse xacro file to urdf
    robot_description_config = xacro.process_file(
        spacediver_urdf_xacro_path, mappings={
            "use_gazebo": use_gazebo,
            "single_arm": single_arm,})

    return LaunchDescription(
        [
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[
                    {"robot_description": robot_description_config.toxml()},
                    controller_config,
                ],
                output={
                    "stdout": "screen",
                    "stderr": "screen",
                },
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager",
                    "/controller_manager",
                ],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    controller_name,
                    "--controller-manager",
                    "/controller_manager",
                ],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {"robot_description": robot_description_config.toxml()},
                    {"use_sim_time": True},
                ],
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                output="screen",
            ),
            # RViz2 Node
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_path],
            ),
        ]
    )
