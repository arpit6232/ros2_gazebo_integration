# ros2_gazebo_integration

The aim of this repository is to create and manage a launch file to integrate gazebo with ros2 eloquent such that to detect a white ball in the camera frame of a xacro based URDF based robot

## URDF based Xacro Updates
- urdf : resources to describe the robot, specifically as a URDF model. This is used to proide the robot's physical properties, enabling performing forward kinematics by TF2, visualisation in Rviz2, and simulating the robot in Gazebo.
### Major Lookouts and Additions/Changes
- Add a model.config file at the base of the src
- xacro based urdf files should be named as xxxx.urdf.xacro for parsing portability
#### CMakeLists.txt changes 
1. Add the xacro package 
```
find_package(xacro REQUIRED)
```
2. Add the location of the xacro file to be detected by cmake 
Example 
```
xacro_add_files("urdf/my_robot.urdf.xacro" INSTALL DESTINATION urdf/)
```

#### package.xml changes 
- Add a build dependency below buildtool_depened
```
<build_depend>xacro</build_depend>
```

<!-- boldbot_gazebo_plugin - plugin for Gazebo that exposes control and
sensor topics to interact with a simulated version of the BoldBot. -->

## Gazebo Simulation updates
Configuration and launch files to launch Gazebo with the custom robot

### Major Lookouts and Additions/Changes
1. Create launch, worlds and or meshes folders under src of the workspace
2. Fill up the world files with appropriate gazebo worlds. 
#### CMakeLists.txt changes 
1. Add the following, the *Destination line is important to add the data content to be shared amongst other libraries*
```
install(
  DIRECTORY 
  launch 
  worlds
  DESTINATION share/${PROJECT_NAME})

```

#### Package.xml changes 
1. Add the nexessary dependencies , example
```
<exec_depend>robot_state_publisher</exec_depend>
<depend>gazebo_ros</depend>
```
2. Export the plugins for use
```
<export>
    <build_type>ament_cmake</build_type>
    <gazebo_ros plugin_path="${prefix}" />
  </export>
```


## Start Simulation
To run a simulation of the custom robot, build all necessary packages, and then run the following after sourcing the workspace:
- ``` ros2 launch my_robot world.launch.py```