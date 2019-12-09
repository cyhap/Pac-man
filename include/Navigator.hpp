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
#include <string>
#include <vector>
#include <algorithm>

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

class Navigator {
  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
                                                          MoveBaseClient;

 public:
  // Closest object pose to the Turtlebot
  pacman::ObjPose closestPose;
  // Status of the Navigation Stack
  bool navStackStatus;
  // Flag to allow the imgCallback method to implement
  bool allowImgCallback;
  // boolean for sending goal to navigation stack
  bool sendGoal;
  // Smart Pointer to instance of Movement Class
  std::shared_ptr<Movement> movement;

  /**
  *  @brief   This is the constructor for the Navigator Class
  *  @param	  None
  *  @return	None
  */
  Navigator();

  /**
  *  @brief   This is a callback for the laser scanner data
  *  @param	  msg Laser Scanner distance data
  *  @return	None
  */
  void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr&);

  /**
  *  @brief   This is a callback for the object poses from image processing
  *  @param	  vecPoses vector of object poses
  *  @return	None
  */
  void imgCallback(const pacman::VecPoses::ConstPtr&);

  /**
  *  @brief   This is a function that checks for NaNs
  *  @param	  i variable
  *  @return	Boolean
  */
  static bool isNan(float i) { return std::isnan(i); }

  /**
  *  @brief   This is a function that checks the Navigation Stack status
  *  @param	  None
  *  @return	Boolean
  */
  bool checkVisuals();

  /**
  *  @brief   This is a callback to find the closest object to the robot
  *  @param	  msg States of the models
  *  @return	None
  */
  void closestCallback(const gazebo_msgs::ModelStates);

  /**
  *  @brief   Function to delete object off world
  *  @param   None
  *  @return  None
  */
  void deleteObject();

  /**
  *  @brief   Function to set delete bool to true
  *  @param   None
  *  @return  None
  */
  void setDelete();

  /**
  *  @brief   Function to set delete bool to false
  *  @param   None
  *  @return  None
  */
  void resetDelete();

  /**
  *  @brief   Function to navigate the world
  *  @param	  clear_ Boolean for path clear
  *  @param	  turns Count for number of right/left rotations
  *  @return  Twist Velocity paramters
  */
  geometry_msgs::Twist navigate(bool, int&);

 private:
  // Node handler for class
  ros::NodeHandle n_;
  // Publisher for collected object poses
  ros::Publisher colObjPose_;
  // subscriber for model states topic
  ros::Subscriber subClosestObj_;
  // client handle for deleting the object
  ros::ServiceClient clientDelObj_;
  // client handle for getting poses
  ros::ServiceClient clientGetPos_;
  // string name of closest object
  std::string closestObject;
  // flag to allow deletion of object
  bool deleteOkay;
  // variable for publishing velocities
  geometry_msgs::Twist vels;
  // Set up the action client with it set to spin a thread by default
  MoveBaseClient aclient;
};

#endif  // INCLUDE_NAVIGATOR_HPP_
