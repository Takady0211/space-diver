import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import xacro

# User settings
use_sim_time = LaunchConfiguration('use_sim_time', default='true')

pkg_dir = get_package_share_directory("floating_robot_controller")
example_urdf_path = os.path.join(
    pkg_dir, "model", "example.urdf")
rviz_path = os.path.join(pkg_dir, "rviz", "simple_config.rviz")


def parse_xacro(xacro_path, xml_path):
    doc = xacro.process_file(xacro_path)
    robot_desc = doc.toprettyxml(indent=' ')
    f = open(xml_path, 'w')
    f.write(robot_desc)
    f.close()


def file_contents(file):
    with open(file) as f:
        return f.read()


def generate_launch_description():

    # parse_xacro(spacediver_urdf_xacro_path, spacediver_urdf_path)

    return LaunchDescription([

        # Robot state publisher node
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': file_contents(example_urdf_path),
                'use_sim_time': use_sim_time
            }],
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
