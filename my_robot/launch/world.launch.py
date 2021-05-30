import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch_ros.actions import Node
from scripts import GazeboRosPaths


def get_package_file(package, file_path):
    """Get the location of a file installed in an ament package"""
    package_path = get_package_share_directory(package)
    absolute_file_path = os.path.join(package_path, file_path)
    return absolute_file_path

def load_file(file_path):
    """Load the contents of a file into a string"""
    try:
        with open(file_path, 'r') as file:
            return file.read()
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def load_yaml(file_path):
    """Load a yaml file into a dictionary"""
    try:
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def run_xacro(xacro_file):
    """Run xacro and output a file in the same directory with the same name, w/o a .xacro suffix"""
    urdf_file, ext = os.path.splitext(xacro_file)
    if ext != '.xacro':
        raise RuntimeError(f'Input file to xacro must have a .xacro extension, got {xacro_file}')
    os.system(f'xacro {xacro_file} -o {urdf_file}')
    return urdf_file


def generate_launch_description():
    model_path, plugin_path, media_path = GazeboRosPaths.get_paths()

    # Find my robot Description
    urdf_prefix = get_package_share_directory("my_robot")
    urdf_file = os.path.join(urdf_prefix, "urdf", "my_robot.urdf")

    # Find World Description
    world_prefix = get_package_share_directory("my_robot")
    world_file = os.path.join(world_prefix, "worlds", "my_robot.world")

    # # Setup Launch Arguments incase needed
    # use_sim_time = DeclareLaunchArgument(
    #             'use_sim_time',
    #             default_value='false',
    #             description='Use simulation (Gazebo) clock if true')
    
    # debug = DeclareLaunchArgument(
    #             'debug',
    #             default_value='false',
    #             description='Enable debug messages for gazebo')

    # gui = DeclareLaunchArgument(
    #             'gui',
    #             default_value='true',
    #             description='gui for gazebo')

    # pkg_name = 'my_robot'
    # xacro_file = get_package_file(pkg_name, 'urdf/my_robot.urdf.xacro')
    # xacro_path = DeclareLaunchArgument(
                # 'xacro_path',
                # default_value=xacro_file,
                # description='path to urdf.xacro file to publish')


    return LaunchDescription(
        [
            # use_sim_time,
            # debug,
            # gui,
            # xacro_path,

            # Execute Gazebo in a separate process indicating the world file
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
                additional_env={ 
                    "GAZEBO_MODEL_PATH": model_path,
                    "GAZEBO_PLUGIN_PATH": plugin_path,
                    "GAZEBO_RESOURCE_PATH": media_path,
                }
            ),

            #  Spawn My Robot 
            Node(
                name='urdf_spawner',
                # type='spawn_model',
                # respawn="false",
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
                    "-R", # Roll
                    "0",
                    "-P", # pitch
                    "0",
                    "-Y", # Yaw
                    "0",
                    "-b",
                    "-file",
                    urdf_file,
                    # "-param",
                    # robot_description
                ],

            ),
            Node(
                package="robot_state_publisher",
                node_executable="robot_state_publisher",
                output="screen",
                arguments=[urdf_file],
            ),
        ]
    )
