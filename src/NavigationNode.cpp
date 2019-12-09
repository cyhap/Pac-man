/*
 * Copyright (c) 2019, Ari Kupferberg, Ethan Quist, Corbyn Yhap
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 /**
  * @file NavigationNode.cpp
  * @copyright 2019 Ari Kupferberg
  * @author Ari Kupferberg
  * @date 12/3/2019
  * @brief This is the ROS Node that will handle the interaction with the
      Navigation Stack.
  */
#include "pacman/NavPose.h"
#include "ros/ros.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"

// The following lines of code have been pulled from the ROS wiki:
// http://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
                                                          MoveBaseClient;

/**
*  @brief   Server Function for Nav Stack Goal Pose
*  @param	  Service Request Type variable passed by reference
*  @param	  Service Response Type variable passed by reference
*  @return	boolean true
*/
bool nav(pacman::NavPose::Request &req,
          pacman::NavPose::Response &res) {
  // Tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);
  // Wait for the action server to come up
  while (!ac.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  // Set up goal
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();
  // Assign Position values
  goal.target_pose.pose.position.x = req.pose.x;
  goal.target_pose.pose.position.y = req.pose.y;
  goal.target_pose.pose.position.z = req.pose.z;
  // Assign Orientation Values
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal");
  ac.sendGoalAndWait(goal, ros::Duration(20.0));

  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved forward");
  else
    ROS_INFO("The base failed to move forward for some reason");
  res.str = "Arrived at Goal Pose";
  ROS_INFO_STREAM(res.str);

  return true;
}


/**
*  @brief   This is the main function
*  @param	  argc for ROS
*  @param	  argv for ROS
*  @return	0 Exit status
*/
int main(int argc, char** argv) {
  // Initialize the Nav Stack Interaction node.
  ros::init(argc, argv, "navigation");

  // Create ROS node handle
  ros::NodeHandle n;

  // Set up the server
  ros::ServiceServer srv = n.advertiseService("navpose", nav);

  while (ros::ok()) {
    // Loop through the callback
    ros::spinOnce();
  }
  return 0;
}
