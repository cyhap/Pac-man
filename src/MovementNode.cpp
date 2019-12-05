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
  * @file MovementNode.cpp
  * @copyright 2019 Ethan Quist
  * @author Ethan Quist
  * @date 11/24/2019
  * @brief This is the ROS Node that will handle the search movement of the Turtlebot.
  */


#include <math.h>
#include <stdlib.h>

#include <memory>
#include <vector>
#include <algorithm>

#include "Movement.hpp"
#include "Object.hpp"

#include "ros/ros.h"
#include "kobuki_msgs/BumperEvent.h"
#include "gazebo_msgs/DeleteModel.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

std::shared_ptr<Movement> movement;  // SHOULD NOT USE GLOBAL VARIABLES ----


class Mover {
 private:
  bool navStackStatus;
  bool allowImgCallback;

 public:
  Mover() : navStackStatus { false }, allowImgCallback { true } {}
  virtual ~Mover() {}
  bool getAllowImgCallback() { return allowImgCallback; }
  void setAllowImgCallback(bool status_) { allowImgCallback = status_; }
  void imgCallback(const geometry_msgs::Twist::ConstPtr& imgPose) {
    if (allowImgCallback) {
      //  -- Find the closest pose from the input
      Object::Pose closestPose;
      closestPose.x = imgPose->linear.x;
      closestPose.y = imgPose->linear.y;
      closestPose.yaw = imgPose->angular.z;
      //  -- Send closestPose to Navigation Stack
      allowImgCallback = false;
    } else {
      ROS_INFO_STREAM("Navigation stack is runnning");
    }
  }
  bool checkVisuals() {
    //  -- Call Navigation Stack Flag
    //  if (<nav stack is still running>)
    //    navStackStatus = true;
    //  else if (<nav stack is not running>)
    //    navStackStatus = false;
    navStackStatus = false;  // set always false, for now.
    ROS_INFO_STREAM("Checked Visuals, output: " << navStackStatus);
    return navStackStatus;
  }
};


static bool isNan(float i) {
  return std::isnan(i);
}
void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  ROS_INFO_STREAM("Received LaserScan");
  std::vector<float> ranges = msg->ranges;
  // Replace all nans with max range to get the actual minimum
  std::replace_if(ranges.begin(), ranges.end(), isNan, msg->range_max);
  float minRange = *std::min_element(ranges.begin(), ranges.end());
  ROS_INFO_STREAM("The closest object is " << minRange << "(m) away.");
  movement->updateMinDist(minRange);
}

/**
*  @brief   This is the main function
*  @param	  argc for ROS
*  @param	  argv for ROS
*  @return	0 Exit status
*/
int main(int argc, char **argv) {
  // Initialize the walker node.
  ros::init(argc, argv, "movement");

  // Create a node handle
  ros::NodeHandle nm;

  // Declare Mover Object for function callbacks and interfacing
  Mover mover;

  // Initialize the shared pointer to the movement class that will be used to
  // process the point cloud call backs.
  movement.reset(new Movement);

  // Publish on the topic required to move turtlebot
  // This will be remmapped in the launch file.
  auto pub = nm.advertise <geometry_msgs::Twist> ("/cmd_vel_mux/input/navi",
                                                                        1000);
  auto lsrSub = nm.subscribe("/scan", 1000, laserScanCallback);
  auto imgSub = nm.subscribe("imgPoses", 1000, &Mover::imgCallback, &mover);

  // Publish at 10 Hz.
  ros::Rate loop_rate(10.0);

  while (ros::ok()) {
    // Use the Navigation Stack status to decide movement
    if (!mover.checkVisuals()) {  // Returns navigation stack flag
      ROS_INFO_STREAM("Movement Search with Laser Scanner");
      // Allow image callback to look for new objects
      mover.setAllowImgCallback(true);

      // Set the turtlebot velocities from the laser scanner callback.
      geometry_msgs::Twist velMsg;
      std::pair<double, double> output = movement->computeVelocities();
      ROS_WARN_STREAM("Vels: " << output.first << "," << output.second);
      velMsg.linear.x = output.first;
      velMsg.angular.z = output.second;

      pub.publish(velMsg);
    }

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
