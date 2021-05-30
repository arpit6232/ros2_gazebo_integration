#!/usr/bin/env python3

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

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

    xacro_file = get_package_file('my_robot', 'urdf/my_robot.urdf.xacro')
    urdf_file = run_xacro(xacro_file)

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )

    pkg_name = 'my_robot'
    world_file_name = 'my_robot.world'
    model_xacro_file_name = 'my_robot.urdf.xacro'

    pkg_dir = get_package_share_directory(pkg_name)

    world = os.path.join(pkg_dir, 'world', world_file_name)
    xacro_path = os.path.join(pkg_dir, 'urdf', model_xacro_file_name)

    my_robot_node = Node(
        package='my_robot',
        node_executable='my_robot',
        output='screen',
        arguments=[urdf_file],
    )

    print(xacro_file)

    return LaunchDescription([

        DeclareLaunchArgument(
            'world',
            default_value=[os.path.join(pkg_name, 'world', 'my_robot.world'), ''],
            description='SDF world file for the robot'
            ),
        gazebo
        # my_robot_node
        
    ])

    



