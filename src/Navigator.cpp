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
  * @file Navigator.cpp
  * @copyright 2019 Ari Kupferberg
  * @author Ari Kupfeberg
  * @date 12/7/2019
  * @brief This ROS dependent Class is for Navigation Control.
  */

#include <math.h>
#include <stdlib.h>

#include <memory>
#include <vector>
#include <algorithm>

#include "Navigator.hpp"
#include "Movement.hpp"
#include "Object.hpp"

#include "pacman/ObjPose.h"  // Our custom msg data
#include "pacman/VecPoses.h"  // Our custom msg vector
#include "pacman/NavPose.h"  // Our custom srv type
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"

// used for object deletion
#include "gazebo_msgs/DeleteModel.h"
#include "gazebo_msgs/GetModelState.h"
#include "gazebo_msgs/ModelStates.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"

Navigator::Navigator()
  : navStackStatus{ false }, allowImgCallback{ true } , sendGoal{ false } ,
          aclient("move_base", true) {
  // Subsriber to find closest object -
  //THIS IS A CUSTOM THROTTLE TOPIC, DONT FORGET TO ADD IT TO LAUNCH
  subClosestObj_ = n_.subscribe("/my_model_states", 1000,
                                &Navigator::closestCallback, this);

  // Service client for deleting objects
  clientDelObj_ = n_.serviceClient<gazebo_msgs::DeleteModel>
                                              ("/gazebo/delete_model");

  // Service client from get model state
  clientGetPos_ = n_.serviceClient<gazebo_msgs::GetModelState>
                                              ("/gazebo/get_model_state");
}

void Navigator::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  ROS_INFO_STREAM("Received LaserScan");
  std::vector<float> ranges = msg->ranges;
  // Replace all nans with max range to get the actual minimum
  std::replace_if(ranges.begin(), ranges.end(), isNan, msg->range_max);
  float minRange = *std::min_element(ranges.begin(), ranges.end());
  ROS_INFO_STREAM("The closest object is " << minRange << "(m) away.");
  movement->updateMinDist(minRange);
}

void Navigator::imgCallback(const pacman::VecPoses::ConstPtr& vecPoses) {
  if (allowImgCallback) {
    allowImgCallback = false;

    // Service cleint to get Turtlebot orientation
    gazebo_msgs::GetModelState anglesrv;

    //  -- Find the closest pose from the vector input
    double minMag = 100;
    for (auto indpose : vecPoses->poses) {
      // Grab x,y,z coordinates of current pose
      double xpos = indpose.x;
      double ypos = indpose.y;
      double zpos = indpose.z;
      
      // Obtain magnitude from pose to base
      double mag = sqrt(pow(xpos, 2) + pow(ypos, 2) + pow(zpos, 2));
      if (mag < minMag) {
        minMag = mag;
        closestPose = indpose;
        // Call service for orientation of base
        anglesrv.request.model_name = "mobile_base";
        if (clientGetPos_.call(anglesrv))
          closestPose.theta = anglesrv.response.pose.orientation.z;
        else
          ROS_ERROR_STREAM("Failed to get model state service");
      }
    }
    ROS_WARN_STREAM("Sending Pose to Navigation stack");
    sendGoal = true;
  } else {
    ROS_WARN_STREAM("Navigation stack is runnning");
    sendGoal = false;
  }
}

bool Navigator::checkVisuals() {
  // Wait for the action server to come up
  while (!aclient.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  // Check State of Navigation Stack
  if (aclient.getState().isDone())
    navStackStatus = false;
  else
    navStackStatus = true;
  navStackStatus = false;  // set always false, for now.
  ROS_INFO_STREAM("Checked Visuals, output: " << navStackStatus);
  return navStackStatus;
}

void Navigator::goalDelete() {
  // Service call to delete object
  gazebo_msgs::DeleteModel dmsrv;
  dmsrv.request.model_name = closestObject;
  if (clientDelObj_.call(dmsrv)) {
    // calls in background
  }
}

void Navigator::closestCallback(const gazebo_msgs::ModelStates msg) {
  float closest_dist = 1000.0;  // arbitrary large number
  int closest = 0;
  float dist;
  int lent;
  float x, y, x1, y1, x2, y2;

  // Finding turtle position
  gazebo_msgs::GetModelState getsrv;
  getsrv.request.model_name = "mobile_base";
  if (clientGetPos_.call(getsrv)) {
    x1 = getsrv.response.pose.position.x;
    y1 = getsrv.response.pose.position.y;
    ROS_INFO_STREAM("Position of turtlebot: " << x1 << " " << y1);
  } else {
    ROS_ERROR_STREAM("Failed to call get model state service");
    x1 = 8.0;  // some number
    y1 = 8.0;  // some number
  }

  // Find object nearest to turtlebot
  lent = msg.pose.size();
  for (int i = 0; i < lent; i++) {
    x2 = msg.pose[i].position.x;
    y2 = msg.pose[i].position.y;
    x = x1 - x2;
    y = y1 - y2;
    dist = sqrt(x * x + y * y);
    if (dist < closest_dist) {
      if (msg.name[i] == "mobile_base") {
        // Skip mobile_base
      } else {
        if (msg.name[i] == "ground_plane") {
          // Skip ground plane
        } else {
          closest_dist = dist;
          closest = i;
        }
      }
    }
  }

  ROS_INFO_STREAM("node: " << closest << ", name: " << msg.name[closest]);
  closestObject = msg.name[closest];
}

