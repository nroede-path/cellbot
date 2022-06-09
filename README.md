# cellbot
ROS package for the Path UR10e robot cell

# Usage
Full robot bringup can be performed with the command:
roslaunch cellbot urbringup.launch

This launches the robot driver, moveit, RViz, and the collision model, meaning the robot will be fully set up to control.

Make sure to stop and restart the external control program from the robot pendant when relaunching using this launch file. Otherwise the planning will succeed, but execution will fail and the following error message will be displayed:

Can't accept new action goals. Controller is not running.

This will persist (even if the ROS nodes are restarted) until the UR pendant program itself is stopped and restarted.

# Dependencies
Tutorial must be followed at: https://pathrobotics.atlassian.net/wiki/spaces/sensor/pages/1777467407/Setting+up+ROS+connection+with+UR10
in order to have the correct packages setup for the robot to run.
Packages needed:
- Universal_Robots_ROS_Driver
- fmauch_universal_robot

# Collision Model
Following tutorial at: https://pathrobotics.atlassian.net/wiki/spaces/sensor/pages/1807188074/Collision+modelling+of+UR+robot+Cell
and based on code at: https://github.com/path-robotics/sensor_robotics/blob/robot_specific/ur10_samples/src/move_w_collision_model_clean.cpp#L110

The collision model can be manually launched with:

rosrun cellbot ur10e_collision_model
