# Gradient Field Based Local Planner for the ROS Navigation Stack

This repository contains the source file for a gradient field based local planner that can be used with the ROS Navigation Stack. It also contains some launch and parameter files.

## gradplanner library

The `gradplanner` library is a cpp library that implements the gradient field based controller. It implements a repulsive and an attractor gradient field which together provide a local minima free gradient field to control the robot based on them. Tests for the library are written with the help of `CXXTest-4.3`.

### Basic algorithm description of the controller

The controller becomes the actual state of the robot, the occupancy grid from ROS and the actual goal. The actual goal is calculated from the Navigation Stack's global plan by finding the point where the global plan enters the local costmap. Then the controller decides, in which mode from the following modes to control the robot:

* __Grad mode__: Combines the repulsive and the attractor gradient fields. Then sums up some neighbouring cell's gradients around the robot to create a desired orientation that the robot should follow. The provides the `cmd_vel` command velocities based on the error from this desired orientation.
* __Direct mode__: If the robot is sufficiently far enough from the obstacles and there is no obstacle between the robot and its goal, the desired orientation simply points to the goal and the command velocities are calculated based on this.
* __End mode__: If the robot is within the position error tolerance to the goal, only its orientation is adjusted.

## GradFieldPlannerROS

This ROS plugin is basically a wrapper for the `gradplanner` library's `GradFieldController` class. It inherits from the `nav_core::BaseLocalPlanner` and interfaces the planner with ROS's Navigation Stack. `GradFieldPlannerROS` is exported as a ROS plugin, hence the navigation stack is able to load it dynamically during runtime.

## Files

* `gflp_plugin.xml`: A required for being able to use `GradFieldPlannerROS` as a plugin. 
* `rviz/turtlebot3_navigation.rviz`: rviz file that includes the visualization of the inflated occupancy grid and the gradient field.
* `launch/move_base.launch`: Launches the `move_base` package of the Navigation Stack with the custom local planner and loads the custom parameter files.
* `launch/turtlebot3_navigation.launch`: Launches all the packages which are needed for navigation. (Including the previously mentioned `launch/move_base.launch`.) It uses the custom rviz file `rviz/turtlebot3_navigation.rviz`.
* `params/costmap_common_params_waffle_pi_stereo.yaml`: Contains the layer definitions for the costmap. This allows to use several sensors such as the RGB-D camera and the cliff sensors for creating the occupancy grid around the robot.
* `params/local_costmap_params.yaml`: This is where the local costmap parameters are defined such as the layers, which build up the local costmap and the size and resolution of the costmap.
* `gf_local_planner_params_waffle_pi_stereo.yaml`: The parameter file that contains the params for the `GradFieldPlannerROS` local planner. Here most of the parameters are self explanatory and also have comments in the param file, but for some of them here is a short descreption:
  * Proportional gains: The `K` params for the different modes are the proportional gains with which the angular velocity command of the robot is calculated: `cmd_omega = - K * orientation_error`. The orientation error is calculated from the desired orientation. The desired orientation is calculated differently in ever mode, this was detailed previously.
  * `boundar_error` and `max_error`: These parameters are used to calculate the translational velocity of the robot. The idea is, that if the absolute value of the orientation error is smaller than the `boundary_error`, then the robot tries to go as fast as it can, respecting the maximal velocity and acceleration constraints. If the absolute value of the orientation error is bigger then `max_error`, then the translational command velocity will be zero, and the robot will just turn around its axis until it is again below `max_error`. If the error is between these two boundaries, the command velocity of the robot will be linearly decreasing with the absolute value of the error from v_max to zero. (The velocity commands are always calculated with respecting the acceleration constraints.)
  * `direct_mode/min_obst_dist`: This parameter is used during deciding whether the robot can operate in `direct_mode` or should operate in `grad_mode`. If the robot is closer to an obstalce than this value, the robot will operate in gradient mode, no matter if the goal is visible or not. (The only exeption is if the goal is already closer to the robot than the radius of the repulsive field.)

## TODO

* For now it only works if the local costmaps global frame (`params/local_costmap_params.yaml`: `local_costmap/global_frame`) is the `map` frame. It's not a big problem actually, but the local costmap usually uses the `odom` frame for smoother state signal. For this, the transformation of the global plan from `map` to `odom` would be needed in the `GradFieldPlannerROS.getGoal()` method.
* Add `Params` to `GradFieldBase` and `set_from_params` method.
* Write tests for attractor field when using 8 search directions.
* Maybe use separate occupancy grids for the attractor and the repuslive fields in `GradFieldPlannerROS`.
* Make `gradplanner` library independent from the ros packages. It is basically independent, ros was only used for some info printing anyways. However in CMakeLists.txt it is still a dependency. (Also in some files the headers are included but not used.)
* get velocity from ROS, not just assume that the low level controller will be able to control the plant to go with the commanded velocities.
