import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from scripts import GazeboRosPaths


def generate_launch_description():
    model_path, plugin_path, media_path = GazeboRosPaths.get_paths()

    env = {
        "GAZEBO_MODEL_PATH": model_path,
        "GAZEBO_PLUGIN_PATH": plugin_path,
        "GAZEBO_RESOURCE_PATH": media_path,
    }

    urdf_prefix = get_package_share_directory("my_robot")
    urdf_file = os.path.join(urdf_prefix, "urdf", "my_robot.urdf")

    world_prefix = get_package_share_directory("my_robot")
    world_file = os.path.join(world_prefix, "worlds", "my_robot.world")

    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=[
                    "gazebo",
                    "-s",
                    "libgazebo_ros_init.so",
                    "-s",
                    "libgazebo_ros_factory.so",
                    world_file,
                ],
                output="screen",
                additional_env=env,
            ),
            Node(
                package="gazebo_ros",
                node_executable="spawn_entity.py",
                arguments=[
                    "-entity",
                    "my_robot",
                    "-x",
                    "0",
                    "-y",
                    "0",
                    "-z",
                    "0",
                    "-R",
                    # $(arg roll) -P $(arg pitch) -Y $(arg yaw)"
                    "0",
                    "-P",
                    "0",
                    "-Y",
                    "0",
                    "-b",
                    "-file",
                    urdf_file,
                ],
            ),
            Node(
                package="my_robot",
                node_executable="my_robot",
                output="my_robot",
                arguments=[urdf_file],
            ),
        ]
    )
