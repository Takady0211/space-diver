import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration
# from launch.actions import DeclareLaunchArgument
import xacro

# User settings
controller_name = "position_trajectory_controller" # Select controller from controller_list

# Path setting for xacro, urdf, rviz, and world files
pkg_dir = get_package_share_directory("spacediver_ros2_control")
spacediver_urdf_xacro_path = os.path.join(
    pkg_dir, "description", "urdf", "spacediver_dual_arm.urdf.xacro")
spacediver_urdf_path = os.path.join(
    pkg_dir, "description", "urdf", "spacediver_dual_arm.urdf")
controller_config = os.path.join(
    pkg_dir, "bringup", "config", "spacediver_controllers.yaml")
rviz_path = os.path.join(pkg_dir, "description", "rviz", "spacediver_config.rviz")

def parse_xacro(xacro_path, xml_path):
    doc = xacro.process_file(xacro_path)
    robot_desc = doc.toprettyxml(indent=' ')
    f = open(xml_path, 'w')
    f.write(robot_desc)
    f.close()

def generate_launch_description():
    # Parse xacro file to urdf
    robot_description_config = xacro.process_file(spacediver_urdf_xacro_path)
    parse_xacro(spacediver_urdf_xacro_path, spacediver_urdf_path)


    return LaunchDescription([

        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                {"robot_description": robot_description_config.toxml()},
                controller_config
            ],
            output={
                "stdout": "screen",
                "stderr": "screen",
            },
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster",
                       "--controller-manager", "/controller_manager"],
        ),
        Node( 
        package="controller_manager",
        executable="spawner",
        arguments=[controller_name, "--controller-manager",
                     "/controller_manager"]
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description_config.toxml()}, {'use_sim_time': True}],
            # arguments=[spacediver_urdf_path],
        ),
                Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
        ),

        # RViz2 Node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_path]
        )
    ])