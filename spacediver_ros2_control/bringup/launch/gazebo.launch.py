import os
from re import L

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

import xacro

controller_list = [
    "forward_position_controller",
    "forward_effort_controller",
    "feedback_effort_controller",
    "forward_position_controller_single_arm",
    "forward_effort_controller_single_arm",
    "feedback_effort_controller_single_arm",
]


# User settings
use_sim_time = LaunchConfiguration("use_sim_time", default="true")
controller_name = controller_list[2]

# Path setting for xacro, urdf, rviz, and world files
pkg_dir = get_package_share_directory("spacediver_ros2_control")
spacediver_urdf_xacro_path = os.path.join(
    pkg_dir, "description", "urdf", "spacediver.urdf.xacro"
)
spacediver_urdf_path = os.path.join(pkg_dir, "description", "urdf", "spacediver.urdf")
spacediver_sdf_xacro_path = os.path.join(
    pkg_dir, "description", "gazebo", "models", "spacediver.sdf.xacro"
)
spacediver_sdf_path = os.path.join(
    pkg_dir, "description", "gazebo", "models", "spacediver.sdf"
)
controller_config = os.path.join(
    pkg_dir, "bringup", "config", "spacediver_controllers.yaml"
)
rviz_path = os.path.join(pkg_dir, "description", "rviz", "spacediver_config.rviz")
world_path = os.path.join(
    pkg_dir, "description", "gazebo", "worlds", "underwater.world"
)


def parse_xacro(xacro_path, xml_path):
    doc = xacro.process_file(xacro_path)
    robot_desc = doc.toprettyxml(indent=" ")
    f = open(xml_path, "w")
    f.write(robot_desc)
    f.close()


def generate_launch_description():

    parse_xacro(spacediver_urdf_xacro_path, spacediver_urdf_path)
    parse_xacro(spacediver_sdf_xacro_path, spacediver_sdf_path)

    sdf_model = LaunchConfiguration("sdf_model")

    return LaunchDescription(
        [
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
            DeclareLaunchArgument(
                name="sdf_model",
                default_value=spacediver_sdf_path,
                description="Absolute path to robot sdf file",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(
                            get_package_share_directory("gazebo_ros"), "launch"
                        ),
                        "/gazebo.launch.py",
                    ]
                ),
                launch_arguments={"world": world_path}.items(),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
                arguments=[spacediver_urdf_path],
            ),
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=[
                    "-entity",
                    "spacediver",
                    "-file",
                    sdf_model,
                    "-x",
                    "0",
                    "-y",
                    "0",
                    "-z",
                    "0",
                    "-Y",
                    "0",
                ],
                output="screen",
            ),
        ]
    )
