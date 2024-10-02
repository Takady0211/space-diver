import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import xacro

# User settings
use_sim_time = LaunchConfiguration('use_sim_time', default='true')
use_gazebo = LaunchConfiguration('use_gazebo', default='false') # TODO: Implement this feature
single_arm_test = False                                         # TODO: Implement this feature

# Path setting for xacro, urdf, rviz, and world files
pkg_dir = get_package_share_directory("spacediver_ros2_control")
spacediver_xacro_path = os.path.join(
    pkg_dir, "description", "urdf", "spacediver.urdf.xacro")
spacediver_urdf_path = os.path.join(
    pkg_dir, "description", "urdf", "spacediver.urdf")
rviz_path = os.path.join(pkg_dir, "description", "rviz", "spacediver_config.rviz")
world_path = os.path.join(pkg_dir, "bringup", "worlds", "underwater.world")


def create_urdf_from_xacro(xacro_path, urdf_path):
    doc = xacro.process_file(xacro_path)
    robot_desc = doc.toprettyxml(indent=' ')
    f = open(urdf_path, 'w')
    f.write(robot_desc)
    f.close()


def generate_launch_description():
    create_urdf_from_xacro(spacediver_xacro_path, spacediver_urdf_path)

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager",
                   "/controller_manager"]
    )

    # Full body controllers
    forward_position_controller = Node( 
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "--controller-manager",
                     "/controller_manager"]
    )


    forward_effort_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_effort_controller", "--controller-manager",
                   "/controller_manager"]
    )

    feedback_effort_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["feedback_effort_controller", "--controller-manager",
                    "/controller_manager"]
    )

    # Single arm controllers
    forward_position_controller_single_arm = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller_single_arm", "--controller-manager",
                    "/controller_manager"]
    )

    forward_effort_controller_single_arm = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_effort_controller_single_arm", "--controller-manager",
                    "/controller_manager"]
    )

    feedback_effort_controller_single_arm = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["feedback_effort_controller_single_arm", "--controller-manager",
                    "/controller_manager"]
    )

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[spacediver_urdf_path]
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_path]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory("gazebo_ros"), "launch"),
            "/gazebo.launch.py"]),
        launch_arguments={'world': world_path}.items()
    )

    robot_spawner = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="urdf_spawner",
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=["-topic", "/robot_description", "-entity", "sar_simple"]
    )

    return LaunchDescription([
        joint_state_broadcaster_spawner,
        # feedback_effort_controller_single_arm,
        forward_effort_controller_single_arm,
        rviz2,
        gazebo,
        robot_state_publisher_node,
        robot_spawner
    ])
