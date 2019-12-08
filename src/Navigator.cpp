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

Navigator::Navigator() 
  : navStackStatus{ false }, allowImgCallback{ true } , sendGoal{ false } {
}

//void Navigator::setSendGoal(bool status_) {
//  sendGoal = status_;
//}

//void Navigator::setAllowImgCallback(bool status_) {
//  allowImgCallback = status_;
//}

//void Navigator::getPose() {
//  return closestPose;
//}

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
      }
    }
    ROS_ERROR_STREAM("Sending Pose to Navigation stack");
    sendGoal = true;
  } else {
    ROS_ERROR_STREAM("Navigation stack is runnning");
    sendGoal = false;
  }
}

bool Navigator::checkVisuals() {
  // ----------------------------- FILL IN CODE HERE
  //  -- Call Navigation Stack Flag
  //  if (<nav stack is still running>)
  //    navStackStatus = true;
  //  else if (<nav stack is not running>)
  //    navStackStatus = false;
  navStackStatus = false;  // set always false, for now.
  ROS_INFO_STREAM("Checked Visuals, output: " << navStackStatus);
  return navStackStatus;
}

