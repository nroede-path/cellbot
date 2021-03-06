# cellbot

ROS package for the Path UR10e robot cell

# Usage

Full robot bringup can be performed with the command:
roslaunch cellbot urbringup.launch

This launches the robot driver, moveit, RViz, and the collision model, meaning the robot will be fully set up to control.

Make sure to stop and restart the external control program from the robot pendant when relaunching using this launch file. Otherwise the planning will succeed, but execution will fail and the following error message will be displayed:

Can't accept new action goals. Controller is not running.

This will persist (even if the ROS nodes are restarted) until the UR pendant program itself is stopped and restarted.

# Programs

## ur10e_to_home

Moves the robot immediately to its home position using specifically defined joint position values. The current home position is straight up facing the computer with wrist 2 turned 90 degrees counterclockwise.

## ur10e_collision_model

Moves the robot to its home position (straight up facing the computer with wrist 2 turned 90 degrees counterclockwise), then establishes a collision model for the robot cell (including the floor, pedestal, sensor table, computer table, and floor clutter).

## ur10e_cartesian_point_move
Initializes, waits for a further input, then plans TCP movement to a defined Cartesian point and orientation and moves the robot along the planned path.
The planner algorithm used is PRMstar, set using

"move_group.setPlannerId("PRMstar");

This algorithm, along with constraining the shoulder pan joint to +-3 resulted in fairly consistent/reasonable paths that could be successfully found every trial.

# Dependencies

Tutorial must be followed at: https://pathrobotics.atlassian.net/wiki/spaces/sensor/pages/1777467407/Setting+up+ROS+connection+with+UR10
in order to have the correct packages setup for the robot to run.
Packages needed:
- Universal_Robots_ROS_Driver
- fmauch_universal_robot
- pylon-ros-camera (https://github.com/basler/pylon-ros-camera)
- gazebo_ros_pkgs (https://classic.gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros)

# Collision Model

Following tutorial at: https://pathrobotics.atlassian.net/wiki/spaces/sensor/pages/1807188074/Collision+modelling+of+UR+robot+Cell
and based on code at: https://github.com/path-robotics/sensor_robotics/blob/robot_specific/ur10_samples/src/move_w_collision_model_clean.cpp#L110

The collision model can be manually launched with:

rosrun cellbot ur10e_collision_model
