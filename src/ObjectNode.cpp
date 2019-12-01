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
  * @file ObjectNode.cpp
  * @copyright 2019 Ari Kupferberg
  * @author Ari Kupfeberg
  * @date 11/25/2019
  * @brief This ROS Node is for wrapping the Object Classes
  */

#include "Object.hpp"
#include "GoodObject.hpp"
#include "BadObject.hpp"
#include "ImageProcessing.hpp"
#include <ros/ros.h>

#include "image_transport/image_transport.h"
#include "sensor_msgs/Image.h"

void depthImgCallback(const sensor_msgs::ImageConstPtr &aImg) {
  // Determine what to do here.
  ROS_INFO_STREAM("Depth Image Call back Successful.");
}

void rgbImgCallback(const sensor_msgs::ImageConstPtr &aImg) {
  // Determine what to do here.
  ROS_INFO_STREAM("RGB Image Call back Successful.");
}

/**
*  @brief   This is the main function
*  @param	  argc for ROS
*  @param	  argv for ROS
*  @return	0 Exit status
*/
int main(int argc, char **argv) {
  // Initialize the ROS node
  ros::init(argc, argv, "object");

  // Create a node handle
  ros::NodeHandle nh;
  image_transport::ImageTransport imTrans(nh);
  image_transport::Subscriber depthImgSub;
  image_transport::Subscriber rgbImgSub;

  // Subscribe to the rectified depth image.
  imTrans.subscribe("/depth_registered/image_rect", 1, &depthImgCallback);
  // Subscribe to the raw rgb image.
  imTrans.subscribe("/camera/rgb/image_raw", 1, &rgbImgCallback);

  // Note the default resolution is 640 x 480 as of Indigo. Further research
  // required

  ros::spin();

  return 0;
}
