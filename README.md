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

## pkgs needed 

- mobile_manipulator_body
- amcl
- Hector-SLAM package


##  To be installed

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
> **Note** as output we should have a path like `/opt/ros/noetic/share/amcl`

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

# About the ROS navigation stack

A [ROS Navigation stack][1] is a package that allows us to make a generic robot capable of autonoously moving through the emvironment.

As mentioned by the official ROS documentatiom, a 2D navigation stack takes in information from odometry, sensor streams, and a goal pose and outputs safe velocity commands that are sent to a mobile base.

In our specific case, the robot employed is a mobile base equipped with a mobile manipulator, moving inside a scenario developed by our professors ( :house: [house.world][2])


## Generalities

To store information about obstacles within the environment, the ROS navigation stack uses both:

- Global costmap
- Local costmap 

The **Global costmap** differs from the local one since it is mainly used to generate **long-term** plans rather than local ones, over the entire environment. 

> :info: While global costmaps are used for calculating a certain path, i.e. from a *starting point* 0 to an *ending point* 1, local costmaps are employed to avoid obstacles in the nearby 

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
  

> For saving a new map, after having launched the **hector_slam** pkg and having mapped the overall environment,  run the following instruction (where *"test"* stands by the name of the simulation)

```sh

rosrun map_server map_saver -f test


```




<!-- Links & Resources -->
[1]: http://wiki.ros.org/navigation
[2]: gitblabla

[7]: http://library.isr.ist.utl.pt/docs/roswiki/amcl.html
[8]: https://github.com/fedehub/nav_stack/hector_slam_pkgs/hector_mapping/launch/mapping_default.launch