/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <iostream>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "direct_move");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // BEGIN_TUTORIAL
    //
    // Setup
    // ^^^^^
    //
    // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
    // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
    // are used interchangably.
    static const std::string PLANNING_GROUP = "manipulator";

    // The :move_group_interface:`MoveGroupInterface` class can be easily
    // setup using just the name of the planning group you would like to control and plan for.
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    // We will use the :planning_scene_interface:`PlanningSceneInterface`
    // class to add and remove collision objects in our "virtual world" scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Raw pointers are frequently used to refer to the planning group for improved performance.
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);


    // Getting Basic Information
    // ^^^^^^^^^^^^^^^^^^^^^^^^^
    //
    // We can print the name of the reference frame for this robot.
    ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());

    // We can also print the name of the end-effector link for this group.
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

    // We can get a list of all the groups in the robot:
    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
              std::ostream_iterator<std::string>(std::cout, ", "));







    // Define a collision object ROS message.
    // ^^^^^^^^^^^^^^^^^^^^^^^^^
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group.getPlanningFrame();

    // The id of the object is used to identify it.
    collision_object.id = "box1";

    // Define a box to add to the world.
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 2.8;
    primitive.dimensions[1] = 2.8;
    primitive.dimensions[2] = 0.1;

    // Define a pose for the box (specified relative to frame_id)
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.0;
    box_pose.position.y = 0.0;
    box_pose.position.z = -0.2;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    // Now, let's add the collision object into the world
    ROS_INFO_NAMED("tutorial", "Add an object into the world");
    planning_scene_interface.addCollisionObjects(collision_objects);

    // Now when we plan a trajectory it will avoid the obstacle


    // Planning with Path Constraints
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    //
    // Path constraints can easily be specified for a link on the robot.
    // Let's specify a path constraint and a pose goal for our group.
    // First define the path constraint.
    // moveit_msgs::OrientationConstraint ocm;
    // ocm.link_name = "wrist_3_link";
    // ocm.header.frame_id = "base_link";
    // ocm.orientation.w = 1.0;
    // ocm.absolute_x_axis_tolerance = 0.1;
    // ocm.absolute_y_axis_tolerance = 0.1;
    // ocm.absolute_z_axis_tolerance = 0.1;
    // ocm.weight = 1.0;

    moveit_msgs::PositionConstraint pc;
    pc.link_name="forearm_link";
    pc.header.frame_id="base_link";
    // pc.target_point_offset.x=0;
    // pc.target_point_offset.y=0;
    pc.target_point_offset.z=0.7;
    pc.weight=1.0;

    // Now, set it as the path constraint for the group.
    moveit_msgs::Constraints test_constraints;
    test_constraints.position_constraints.push_back(pc);
    move_group.setPathConstraints(test_constraints);


    // Command series of motion
    // ^^^^^^^^^^^^^^^^^^^^^^^^^

    move_group.setStartState(*move_group.getCurrentState());


    double y[5]={-0.5,-0.25,0.0,0.25,0.5};

    geometry_msgs::Pose target_pose1;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    //bool success;


    while (ros::ok())
    {
        for (int i = 0; i < 5; i++)
        {
            target_pose1.orientation.w = 1.0;
            target_pose1.position.x = 0.5;
            target_pose1.position.y = y[i];
            target_pose1.position.z = 0.5;
            //move_group.setPoseTarget(target_pose1);
            move_group.setApproximateJointValueTarget(target_pose1, "tool0");

            // Now, we call the planner to compute the plan and visualize it.
            // Note that we are just planning, not asking move_group
            // to actually move the robot.

            int plan_trial=0;
            while (ros::ok() && !move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
            {
                ros::Duration(0.1).sleep();
                ROS_INFO("planning trial %i",plan_trial++);
            }


            ROS_INFO("Moving to pose goal %i",i);


            move_group.move();

            ros::Duration(0.1).sleep();
        }
    }




    ros::shutdown();
    return 0;
}