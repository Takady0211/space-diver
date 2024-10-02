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
single_arm_test = True

# Path setting for xacro, urdf, rviz, and world files
pkg_dir = get_package_share_directory("spacediver_ros2_control")
spacediver_xacro_path = os.path.join(
    pkg_dir, "description", "urdf", "spacediver.urdf.xacro")
spacediver_urdf_path = os.path.join(
    pkg_dir, "description", "urdf", "spacediver.urdf")
spacediver_sdf_path = os.path.join(
    pkg_dir, "description", "gazebo", "models", "spacediver.sdf")
rviz_path = os.path.join(pkg_dir, "description", "rviz", "spacediver_config.rviz")
world_path = os.path.join(pkg_dir, "description", "gazebo", "worlds", "underwater.world")

def create_urdf_from_xacro(xacro_path, urdf_path):
    doc = xacro.process_file(xacro_path)
    robot_desc = doc.toprettyxml(indent=' ')
    f = open(urdf_path, 'w')
    f.write(robot_desc)
    f.close()


def generate_launch_description():
    create_urdf_from_xacro(spacediver_xacro_path, spacediver_urdf_path)

    # Launch configurations
    sdf_model = LaunchConfiguration('sdf_model')

    # Declare the launch arguments
    declare_sdf_model_path_cmd = DeclareLaunchArgument(
      name='sdf_model', 
      default_value=spacediver_sdf_path, 
      description='Absolute path to robot sdf file')

    # Controller spawners Node
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager",
                   "/controller_manager"]
    )

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

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory("gazebo_ros"), "launch"),
            "/gazebo.launch.py"]),
        launch_arguments={'world': world_path}.items()
    )

    robot_spawner = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "spacediver", "-file", sdf_model, 
                   "-x", "0", "-y", "0", "-z", "0", "-Y", "0"],
        output="screen"
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add controllers 
    if single_arm_test:
        ld.add_action(joint_state_broadcaster_spawner)
        ld.add_action(forward_position_controller_single_arm)
        ld.add_action(forward_effort_controller_single_arm)
        ld.add_action(feedback_effort_controller_single_arm)

    else:
        ld.add_action(joint_state_broadcaster_spawner)
        ld.add_action(forward_position_controller)
        ld.add_action(forward_effort_controller)
        ld.add_action(feedback_effort_controller)

    # Declare the launch arguments
    ld.add_action(declare_sdf_model_path_cmd)

    # Add actions to launch description
    ld.add_action(rviz2)
    ld.add_action(gazebo)
    ld.add_action(robot_state_publisher_node)
    # ld.add_action(robot_spawner)

    return ld
