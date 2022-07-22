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

#include <std_msgs/Bool.h>

// #include <sensor_msgs/JointState.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <iostream>

// pose codes
int W=0;
int X=1;
int Y=2;
int Z=3;

class robotPlanner {
    ros::NodeHandle node_handle;
    ros::Subscriber subfound;

public:
    robotPlanner() {
        subfound = node_handle.subscribe("cam1/board_found", 1, &robotPlanner::boardCb, this);
    }

    void boardCb(const std_msgs::Bool& msg) {
        // BEGIN_TUTORIAL
        //
        // Setup
        // ^^^^^
        //
        // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
        // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
        // are used interchangeably.
        static const std::string PLANNING_GROUP = "manipulator";

        // The :move_group_interface:`MoveGroupInterface` class can be easily
        // setup using just the name of the planning group you would like to control and plan for.
        moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

        // We will use the planning_scene_interface:`PlanningSceneInterface`
        // class to add and remove collision objects in our "virtual world" scene
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

        // Raw pointers are frequently used to refer to the planning group for improved performance.
        const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(
                PLANNING_GROUP);


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



        // Pre posing the robot in an upright position
        // ^^^^^^^^^^^^^^^^^^^^^^^^^
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

//    move_group.setStartState(*move_group.getCurrentState());
//
//
//    sensor_msgs::JointState js;
//
//    // js.header.frame_id.push_back("base_link");
//    // js.header.stamp.now();
//
//
//    js.name.push_back("shoulder_lift_joint");
//    js.name.push_back("shoulder_pan_joint");
//    js.name.push_back("elbow_joint");
//    js.name.push_back("wrist_1_joint");
//    js.name.push_back("wrist_2_joint");
//    js.name.push_back("wrist_3_joint");
//
//    js.position.push_back(-1.5708);
//    js.position.push_back(0.0);
//    js.position.push_back(0.0);
//    js.position.push_back(0.0);
//    js.position.push_back(0.0);
//    js.position.push_back(0.0);
//
//
//
//    move_group.setJointValueTarget(js);
//
//
//    int plan_trial=0;
//    while (ros::ok() && !move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
//    {
//        ros::Duration(0.1).sleep();
//        ROS_INFO("planning trial %i",plan_trial++);
//    }
//
//
//    move_group.move();

        // Establish joint constraints

        moveit_msgs::Constraints joint_constraints;
//
//    // constrains the shoulder lift joint to stay approximately in the upright position
//    moveit_msgs::JointConstraint shoulder_lift_constraint;
//    shoulder_lift_constraint.joint_name="shoulder_lift_joint";
//    shoulder_lift_constraint.position=-1.5708;
//    shoulder_lift_constraint.tolerance_below= 0.872665;
//    shoulder_lift_constraint.tolerance_above=0.872665;
//    shoulder_lift_constraint.weight=1.0;
//

//    joint_constraints.joint_constraints.push_back(shoulder_lift_constraint);
//
//
        // constrains the shoulder pan joint to avoid longer paths
        moveit_msgs::JointConstraint shoulder_pan_constraint;
        shoulder_pan_constraint.joint_name = "shoulder_pan_joint";
        shoulder_pan_constraint.position = 0;
        shoulder_pan_constraint.tolerance_below = 3;
        shoulder_pan_constraint.tolerance_above = 3;
        shoulder_pan_constraint.weight = 1.0;

        joint_constraints.joint_constraints.push_back(shoulder_pan_constraint);
//
//    // constrains the wrist_3_joint joint to avoid longer paths
//    moveit_msgs::JointConstraint wrist_3_constraint;
//    wrist_3_constraint.joint_name="wrist_3_joint";
//    wrist_3_constraint.position=0;
//    wrist_3_constraint.tolerance_below= 0.001;
//    wrist_3_constraint.tolerance_above= 0.001;
//    wrist_3_constraint.weight=1.0;
//
//    joint_constraints.joint_constraints.push_back(wrist_3_constraint);

        move_group.setPathConstraints(joint_constraints);

        // Command series of motion
        // ^^^^^^^^^^^^^^^^^^^^^^^^^
        int next;
        std::cout << "About to start motion, press any button to cont";
        std::cin >> next;

        move_group.setStartState(*move_group.getCurrentState());
        move_group.setPlanningTime(1); //speed up planning from default of 5s to 1s

        // Planning algorithms:
        // - RRTConnect (fast)
        // - RRTstar, PRMstar (optimal)
        // the list of planning algorithm available fmauch_universal_robot/ur10e_moveit_config/config/ompl_planning.yaml

        move_group.setPlannerId("PRMstar");
        move_group.setNumPlanningAttempts(50);
//
//
//    //double y[5]={-0.5,-0.25,0.0,0.25,0.5};
//
//    geometry_msgs::Pose target_pose1;
//    //bool success;
//
//
//    while (ros::ok())
//    {
//        for (int i = 0; i < 5; i++)
//        {
//            target_pose1.orientation.x = -0.707107;
//            target_pose1.orientation.y = 0.000108;
//            target_pose1.orientation.z = 0.000061;
//            target_pose1.orientation.w = 0.707106;
//            target_pose1.position.x = 0.004517;
//            //target_pose1.position.y = y[i];
//            target_pose1.position.y = 0.290701;
//            target_pose1.position.z = 1.31131;
//            move_group.setPoseTarget(target_pose1, "tool0");
//            move_group.setGoalTolerance(0.01);
//
//            // Now, we call the planner to compute the plan and visualize it.
//            // Note that we are just planning, not asking move_group
//            // to actually move the robot.
//
//            int plan_trial=0;
//            while (ros::ok() && !move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
//            {
//                ros::Duration(0.1).sleep();
//                ROS_INFO("planning trial %i",plan_trial++);
//            }
//
//
//            ROS_INFO("Moving to pose goal %i",i);
//
//
//            move_group.move();
//
//            ros::Duration(1.0).sleep();
//        }
//    }

        geometry_msgs::Pose nextPose;
        if(msg.data) {
            nextPose.orientation.w = 1.0;
            nextPose.position.x = 0.28;
            nextPose.position.y = -0.2;
            nextPose.position.z = 1.2;

        }
        else {
            nextPose.orientation.w = 1.0;
            nextPose.position.x = 0.28;
            nextPose.position.y = -0.2;
            nextPose.position.z = 0.8;
        }

        move_group.setPoseTarget(nextPose);

        // Now, we call the planner to compute the plan and visualize it.
        // Note that we are just planning, not asking move_group
        // to actually move the robot.
        moveit::planning_interface::MoveGroupInterface::Plan my_plan2;

        int plan_trial2 = 0;
        while (ros::ok() && !move_group.plan(my_plan2) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            ros::Duration(0.1).sleep();
            ROS_INFO("planning trial %i", plan_trial2++);
        }

        move_group.move();
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "board_move");
    robotPlanner rp;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::waitForShutdown();
    //ros::shutdown();
    return 0;
}