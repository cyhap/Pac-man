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
  * @file Navigator.hpp
  * @copyright 2019 Ari Kupferberg
  * @author Ari Kupfeberg
  * @date 12/7/2019
  * @brief This ROS dependent Class is for Navigation Control.
  */

#ifndef INCLUDE_NAVIGATOR_HPP_
#define INCLUDE_NAVIGATOR_HPP_

#include <math.h>
#include <stdlib.h>

#include <memory>
#include <vector>
#include <algorithm>

#include "Movement.hpp"
#include "Object.hpp"

#include "pacman/ObjPose.h"  // Our custom msg data
#include "pacman/VecPoses.h"  // Our custom msg vector
#include "pacman/NavPose.h"  // Our custom srv type
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

// used for object deletion
#include "gazebo_msgs/DeleteModel.h"
#include "gazebo_msgs/GetModelState.h"
#include "gazebo_msgs/ModelStates.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"

class Navigator {
 public:
  pacman::ObjPose closestPose;
  bool navStackStatus;
  bool allowImgCallback;
  bool sendGoal;
  std::shared_ptr<Movement> movement;

  // Constructor
  Navigator();

//  // setter
//  void setSendGoal(bool);

//  // setter
//  void setAllowImgCallback(bool);

//  // getter
//  pacman::ObjPose Navigator::getPose();

//  // getter
//  bool Navigator::getNavStatus();

//  // getter
//  bool Navigator::getSendGoal();

  // Laser Scanner Callback
  void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr&);

  // Image Poses Callback
  void imgCallback(const pacman::VecPoses::ConstPtr&);

  // Nan
  static bool isNan(float i) { return std::isnan(i); }

  // Nav Stack Status Checker
  bool checkVisuals();

  // Function called when goal reaached
  void goalDelete();

  // finding closest object to turtlebot
  void closestCallback(const gazebo_msgs::ModelStates);


 private:
  ros::NodeHandle n_;
  ros::Subscriber subClosestObj_;
  ros::ServiceClient clientDelObj_;
  ros::ServiceClient clientGetPos_;
  std::string closestObject;
};

#endif  // INCLUDE_NAVIGATOR_HPP_
