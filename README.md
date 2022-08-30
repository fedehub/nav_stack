# Launching the simulation

For launching the overall simulation

```sh
roslaunch nav_stack nav_stack_gazebo.launch
```

For starting the mapping process

```sh
roslaunch hector_slam_launch tutorial.launch
```

For starting teleoperation throgh keyboard's commands

```sh
roslaunch nav_stack teleop_control.launch
```

https://user-images.githubusercontent.com/61761835/187448562-8ae407e9-1471-48e7-9ec4-fcb57d776b0f.mp4


# pkgs needed 

- mobile_manipulator_body
- amcl
- Hector-SLAM package
- robot_pose_ekf




#  To be installed

For seeing the active coordinate frames, we need to install the tf2 ros tool. If we don't have it yet 

```sh
sudo apt-get install ros-noetic-tf2-tools
```
To install the ROS navigation stack, run

```sh
sudo apt-get install ros-noetic-navigation
```

For granting the correctness of the  installation procedure, run 

```sh
rospack find amcl  
```
> **Remark:** as output we should have a path like `/opt/ros/noetic/share/amcl`

For the graphical user interface, the fourth version of qt has been employed; If not yet installed, please run:

```sh
sudo add-apt-repository ppa:rock-core/qt4 
sudo apt update 
sudo apt-get install qt4-qmake qt4-dev-tools 
```

For the Hector_SLAM package 

```sh
git clone https://github.com/tu-darmstadt-ros-pkg/hector_slam.git
```

For the map_server, needed for both saving and loading maps on Rviz, run:

```sh
sudo apt-get install ros-noetic-map-server
```
For setting up the robot_pose_ekf (extended Kalaman filter), just download

```sh
sudo apt-get install ros-noetic-robot-pose-ekf
```

> **Remark:**  remember to build the package by running `catkin_make` in the ros_workspace

# About the ROS navigation stack

A [ROS Navigation stack][1] is a package that allows us to make a generic robot capable of autonoously moving through the emvironment.

As mentioned by the official ROS documentatiom, a 2D navigation stack takes in information from odometry, sensor streams, and a goal pose and outputs safe velocity commands that are sent to a mobile base.

In our specific case, the robot employed is a mobile base equipped with a mobile manipulator, moving inside a scenario developed by our professors ( :house: [house.world][2])


## Generalities

To store information about obstacles within the environment, the ROS navigation stack uses both:

- Global costmap
- Local costmap 

The **Global costmap** differs from the local one since it is mainly used to generate **long-term** plans rather than local ones, over the entire environment. 

> **Remark** While global costmaps are used for calculating a certain path, i.e. from a *starting point* 0 to an *ending point* 1, local costmaps are employed to avoid obstacles in the nearby 

## Local costmap files 

For hosting the parameters necessary for configfuring the local/global costmaps, three files have been chosen

- Common configuration file: [com_costmap.yaml][]
- Global configuration file: [glo_costmap.yaml][]
- Local configuration  file: [loc_costmap.yaml][]

## Base local planner

It is responsible for sending velocity commands to the robot base controller. It depends on our robot's structure and

## AMCL parameters

The amcl implements the *adaptive* (or KLD-sampling) *Monte Carlo localization* approach (as described by Dieter Fox), which uses a particle filter to track the pose of a robot against a known map. 

It is a well-established probabilistic localization system and in order to *tune* its parameters, it is necessary to enter the **amcl** directory and modify the `amcl_diff.launch` file 

```txt
amcl
    ├── cmake
    │   ├── amclConfig.cmake
    │   └── amclConfig-version.cmake
    ├── examples
    │   ├── amcl_diff.launch
    │   └── amcl_omni.launch
    └── package.xml

    2 directories, 5 files
```

with respect tp the orginal one, it is suggestable to add the `transform_tolerance` parameter, after the definition of the `odom_alpha_5` and `resemple_interval`, with slightly different values. 

> From the ROS wiki [documentatiomn][7], it is referred to *" The time with which to post-date the transform that is published, to indicate that this transform is valid into the future. "* 

```yaml
    <!-- <param name="odom_alpha5" value="0.003"/> -->
    <param name="transform_tolerance" value="0.2" />

    <!-- <param name="resample_interval" value="1.0"/> -->
    <param name="transform_tolerance" value="0.1" />
```
Moreover, further modifications have been applied to the default vakues of the aforementioned parameter server, here below reported 

```yaml 

<param name="odom_model_type" value="diff-corrected"/>
<param name="odom_alpha1" value="0.005"/>
<param name="odom_alpha2" value="0.005"/>
<param name="odom_alpha3" value="0.010"/>
<param name="odom_alpha4" value="0.005"/>
<param name="odom_alpha5" value="0.003"/>

```

## Hector-SLAM Coordinate Frame parameters

Within the **hector-SLAM package** two lines have been changed in order to make the mobile manipulator robot's **frame names** coincide. Starting with the [mapping_default.launch] file. we set (line 7 and 9):

```yaml
<arg name="base_frame" default="base_link"/>
<arg name="odom_frame" default="odom"/>
```

and (line 59)

```yaml
  <node pkg="tf" type="static_transform_publisher" name="base_to_laser_broadcaster" args="0 0 0 0 0 0 base_link laser_link 100"/>
```

For launching the simulation please refer to [this](#launching-the-simulation) section



## About the Map

Inside the [world][9] folder, it is possible to retrieve the mapping records of the arena arranged for the assignment fullfillment. The name of the file where the map is stored can be accessed by following the following steps 

1. Move to the right directory
  ```sh
  roscd nav_stack/maps
  ``` 
  
2. Open a terminal and run the ROSCORE in background
   
   ```sh
   roscore &
   ```

3. Open another termminal and run `map_server` by:
   
   ```sh
   rosrun map_server map_server test.yaml
   ```

4. Open another terminal and launch Rviz by typing: 
   
   ```sh
   rviz
   ```

5. Add manually the topic `/map`
   
6. Select  in the drop down menu, appeared in the left side of the Rviz GUI, aside -> **map**
  

> **Remark:** For saving a new map, after having launched the **hector_slam** pkg and having mapped the overall environment,  run the following instruction (where *"test"* stands by the name of the simulation)

```sh
rosrun map_server map_saver -f test
```

## About the sensors

An IMU sensor has been added to the robot model, being it responsible for a more accurate localisation. All the data gathered by the robot are sent over the `/imu_data` topic, and each time stamp provides i.e., the following information

```txt
---
header: 
  seq: 57
  stamp: 
    secs: 64
    nsecs: 177000000
  frame_id: "imu_link"
orientation: 
  x: 2.0619728275745428e-05
  y: 0.00024832481579281195
  z: -0.0005228758893711238
  w: 0.9999998322551945
orientation_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
angular_velocity: 
  x: 1.6092944895808343e-05
  y: -5.319004252648346e-05
  z: -3.7358866763833176e-05
angular_velocity_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
linear_acceleration: 
  x: -0.004856549931038139
  y: 0.0003927275973314464
  z: 9.799972587559754
linear_acceleration_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
---

```

## about the robot_pose_ekf pkg

This package makes use of an important algorithm, frequently used in the robotics application. It is known as "**Extended Kalaman Filter**" and it uses data gathered through the robot's sensor for then estimating the next state of the robot, in terms of its 
- position 
- orientation 

> **Remark**: For installing it, please refer to the [Installing procedures](#to-be-installed) reported in the linekd section 


### The robot_pose_ekf Subscribers

- `/odom `:  Position and velocity estimate based on the information from the wheel encoder tick counts. The orientation is in quaternion format. (nav_msgs/Odometry)
- `/imu_data` : Data from the Inertial Measurement Unit (IMU) sensor (sensor_msgs/Imu.msg)

### The robot_pose_ekf Publishers

- `/robot_pose_ekf/odom_combined` : The output of the filter or the estimated 3D robot pose (geometry_msgs/PoseWithCovarianceStamped)


### Some notes from the tutorial's author

The data for `/odom `will come from the `/base_truth_odom `topic which is declared inside the URDF file for the robot.

The data for `/imu_data ` will come from the `/imu_data `topic which is also declared inside the URDF file for the robot. 

However, in this simulation, we are using Gazebo ground truth for the odometry, instead of IMU data. 

> **Remark**: to use the IMU data, it is necessary to set that parameter to `true` inside the launch file section for the **robot_pose_ekf** code.

## The final launch file 

In the last version of the launch file, named [nav_stack_v1_gazebo.launch][] we need to remap the data coming from the` /base_truth_odom` topic since the **robot_pose_ekf** node needs the topic name to be `/odom.`

![rosgraph_nodes_topics_all](https://user-images.githubusercontent.com/61761835/185404236-9530af31-9fd0-42d8-94ea-cc32b6949013.png)

<!-- Links & Resources -->
[1]: http://wiki.ros.org/navigation
[2]: gitblabla

[7]: http://library.isr.ist.utl.pt/docs/roswiki/amcl.html
[8]: https://github.com/fedehub/nav_stack/hector_slam_pkgs/hector_mapping/launch/mapping_default.launch
[9]: https://github.com/fedehub/nav_stack/launch/nav_stack_v2_gazebo.launch
