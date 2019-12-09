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
  * @file main.cpp
  * @copyright 2019 Ethan Quist, Ari Kupferberg
  * @author Ethan Quist, Ari Kupferberg
  * @date 11/24/2019
  * @brief This is the main ROS Node that will run the Turtlebot and interaction.
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
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"



/**
*  @brief   This is the main function
*  @param	  argc for ROS
*  @param	  argv for ROS
*  @return	0 Exit status
*/
int main(int argc, char **argv) {
  // Initialize the Movement node.
  ros::init(argc, argv, "movement");

  // Create a node handle
  ros::NodeHandle nm;

  // Declare Navigator Object for function callbacks and interfacing
  Navigator navigator;

  // Initialize the shared pointer to the movement class that will be used to
  // process the point cloud call backs.
  navigator.movement.reset(new Movement);

  // Publish on the topic required to move turtlebot
  // This will be remmapped in the launch file.
  auto pub = nm.advertise<geometry_msgs::Twist>("/cmd_vel",
    1000);
  auto lsrSub = nm.subscribe("/scan", 1000, &Navigator::laserScanCallback,
    &navigator);
  auto imgSub = nm.subscribe("imgPoses", 1000, &Navigator::imgCallback,
    &navigator);

//  ros::ServiceClient client = nm.serviceClient<pacman::NavPose>("navpose");

  // Publish at 10 Hz.
  ros::Rate loop_rate(10.0);

  // Set center width range of image
  double midImgLeft = 240;
  double midImgright = 400;

  geometry_msgs::Twist velMsg;

  while (ros::ok()) {
    // Check for obstacles from the laser scanner callback.
    bool pathclear = navigator.movement->getClearAhead();
    if (pathclear) {
      ROS_WARN_STREAM("Path is clear!");
      double xVal = navigator.closestPose.x;
      if (xVal > midImgright) {  // Object on the right
        // Turn right
        ROS_WARN_STREAM("Rigth Turn: [0,1]");
        velMsg.linear.x = 0.25;
        velMsg.angular.z = 0.50;
      } else if (xVal < midImgLeft) {  // Object on the left
        // Turn left
        ROS_WARN_STREAM("Left Turn: [0,-1]");
        velMsg.linear.x = 0.25;
        velMsg.angular.z = -0.50;
      } else {  // Object centered
        // Go straight
        ROS_WARN_STREAM("Straight Ahead: [1,0]");
        velMsg.linear.x = 1.00;
        velMsg.angular.z = 0.00;
      }
    } else {  // Object blocking path
      if (navigator.closestPose.collect) {  // Is a good object
        navigator.setDelete();
      } else {
        navigator.resetDelete();
        ROS_WARN_STREAM("Object in path! Turning right");
        velMsg.linear.x = 0.00;
        velMsg.angular.z = 1.00;
      }
    }
    pub.publish(velMsg);

    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}
