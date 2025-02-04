import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import xacro

controller_list = ['forward_position_controller',
                   'forward_effort_controller',
                   'feedback_effort_controller',
                   'forward_position_controller_single_arm',
                   'forward_effort_controller_single_arm',
                   'feedback_effort_controller_single_arm',
                   'position_trajectory_controller',
                   'forward_velocity_controller']

# User settings
use_sim_time = LaunchConfiguration('use_sim_time', default='true')
use_gazebo = LaunchConfiguration('use_gazebo', default='false')  # TODO: Implement this feature
controller_name = controller_list[2]  # Select controller from controller_list
use_floating_base = False
single_arm = False

# Path setting for xacro, urdf, rviz, and world files
# Pay attention to the pkg_dir is
# ".../spacediver_ws/install/spacediver_ros2_control/share/spacediver_ros2_control"
# not under this repository.
pkg_dir = get_package_share_directory("spacediver_ros2_control")
file_name = "spacediver_single_arm" if single_arm else "spacediver"
spacediver_urdf_xacro_path = os.path.join(
    pkg_dir, "description", "urdf", file_name + ".urdf.xacro")
spacediver_urdf_path = os.path.join(
    pkg_dir, "description", "urdf", file_name + ".urdf")
spacediver_sdf_xacro_path = os.path.join(
    pkg_dir, "description", "gazebo", "models", file_name + ".sdf.xacro")
spacediver_sdf_path = os.path.join(
    pkg_dir, "description", "gazebo", "models", file_name + ".sdf")
rviz_path = os.path.join(
    pkg_dir, "description", "rviz", "spacediver_config.rviz")
world_path = os.path.join(
    pkg_dir, "description", "gazebo", "worlds", "underwater.world")


def parse_xacro(xacro_path, xml_path):
    doc = xacro.process_file(xacro_path)
    robot_desc = doc.toprettyxml(indent=' ')
    f = open(xml_path, 'w')
    f.write(robot_desc)
    f.close()


def generate_launch_description():
    # Parse xacro file to urdf
    parse_xacro(spacediver_urdf_xacro_path, spacediver_urdf_path)

    # Parse urdf file to sdf
    parse_xacro(spacediver_sdf_xacro_path, spacediver_sdf_path)

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

    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[controller_name, "--controller-manager",
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

    parallel_to_serial_joint_states = Node(
        package='floating_robot_controller',
        executable='parallel_to_serial_joint_states',
        name='parallel_to_serial_joint_states',
        output='screen',
    )

    if (use_floating_base):
        end_eff_traj_controller_executable = 'end_effector_trajectory_controller'
    else:
        end_effector_trajectory_executable = 'fixed_base_end_eff_traj_controller'

    # End effector trajectory controller
    end_effector_trajectory_controller = Node(
        package='floating_robot_controller',
        executable=end_effector_trajectory_executable,
        name=end_effector_trajectory_executable,
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
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(controller_spawner)
    ld.add_action(parallel_to_serial_joint_states)
    ld.add_action(end_effector_trajectory_controller)

    # Declare the launch arguments
    ld.add_action(declare_sdf_model_path_cmd)

    # Add actions to launch description
    ld.add_action(rviz2)
    ld.add_action(gazebo)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(robot_spawner)

    return ld
