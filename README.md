## Launching the simulation

```sh

roslaunch nav_stack nav_stack_gazebo.launch

```

## pkgs needed 

- mobile_manipulator_body
- amcl
- 


##  To install

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
> as output we should have a path like `/opt/ros/noetic/share/amcl`


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

The amcl implements the adaptive (or KLD-sampling) Monte Carlo localization approach (as described by Dieter Fox), which uses a particle filter to track the pose of a robot against a known map. 

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

<!-- Links & Resources -->
[1]: http://wiki.ros.org/navigation
[2]: gitblabla

[7]: http://library.isr.ist.utl.pt/docs/roswiki/amcl.html