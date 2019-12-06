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
  * @author Ari Kupfeberg, Corbyn Yhap
  * @date 11/25/2019
  * @brief This ROS Node is for running the object identification.
  */
#include <memory>

#include "Object.hpp"
#include "GoodObject.hpp"
#include "BadObject.hpp"
#include "ImageProcessing.hpp"

#include "ros/ros.h"
#include "pacman/ObjPose.h"  // Our custom msg type
#include "image_transport/image_transport.h"
#include "sensor_msgs/PointCloud2.h"
#include "cv_bridge/cv_bridge.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

class Identification {
 public:
  ImageProcessing eyes;
  Identification()
      :
      eyes() {
  }
  /**
   *  @brief   This Callback function ... DOES SOMETHING
   *  @param   aImg An image with depth data
   *  @return  None
   */

  void pntCldCallback(const sensor_msgs::PointCloud2 &aPtCloud) {
    // Determine what to do here.
    ROS_INFO_STREAM("Point Cloud Call back Successful.");

    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > tConvPntCld(
        new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(aPtCloud, *tConvPntCld);

    // Pass this to set the pntCloud in the image processing Class
    eyes.setPntCld(tConvPntCld);
  }

  /**
   *  @brief   This Callback function ... DOES SOMETHING
   *  @param   aImg An image with depth data
   *  @return  None
   */
  void rgbImgCallback(const sensor_msgs::ImageConstPtr &aImg) {
    ROS_INFO_STREAM("RGB Image Call back Successful.");

    // Convert ROS Message into a cv::Mat for ImageProcessing Class.

    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(aImg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Convert the image to a shared pointer
    // May need to make a new pointer above to prevent the ros pointer from
    // going out of scope instead...
    //std::shared_ptr<cv::Mat> pic(new cv::Mat(cv_ptr->image));
    // Consider using above than below
    std::shared_ptr<cv::Mat> pic(&cv_ptr->image);
    eyes.setRgbImg(pic);
  }
};


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

  // Create an Instance of Identification.
  Identification identifier;

  // Update the Masks we want for Good and Bad Objects
  cv::Scalar lowGood;
  cv::Scalar highGood;
  cv::Scalar lowBad;
  cv::Scalar highBad;

  identifier.eyes.setGoodObjectMask(lowGood, highGood);
  identifier.eyes.setBadObjectMask(lowBad, highBad);

  // Create the Proper Subscribers.
  image_transport::ImageTransport imTrans(nh);
  ros::Subscriber pntCldSub;
  image_transport::Subscriber rgbImgSub;

  // Subscribe to the Rectified Point Cloud
  pntCldSub = nh.subscribe("/pacman/points", 1, &Identification::pntCldCallback,
                           &identifier);

  // Subscribe to the raw rgb image.
  rgbImgSub = imTrans.subscribe("/camera/rgb/image_raw", 1,
                                &Identification::rgbImgCallback, &identifier);

  // Publisher for Good Object Poses obtained from Image Processing
  ros::Publisher objpub = nh.advertise<pacman::ObjPose>("imgPoses", 1000);

  //#### Process ####//
  // process images
  //// apply good mask
  // create good objects
  //// apply bad mask
  // create bad objects
  // update map with bad objects - Service call
  // Publish good objects

  // Publish at 10 Hz.
  ros::Rate loop_rate(10.0);

  while (ros::ok()) {
    // Process the Images and Extract Objects
    std::vector<std::shared_ptr<Object> > tObjs = identifier.eyes.process();
    std::vector<Object::Pose> toSend;

    // Parse Out Good Objects to Publish on a Topic
    // Update Nav Stack Map with Bad Objects.
    for (const auto &tObj : tObjs) {
      // update if statement to make sure only good poses get published
      if (true) {
        toSend.emplace_back(tObj->getPose());
      } else {
        // Learn how to send this to the Map!!!
      }
    }

    // Publish the list of Good Objects.
    // Discuss with Ari.

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
