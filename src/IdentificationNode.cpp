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
#include "pacman/VecPoses.h"  // Our custom msg type
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
  // Publish The Masked Image Output of Identified Objects
  image_transport::Publisher goodMaskPub;
  image_transport::Publisher badMaskPub;
  Identification()
      :
      eyes(),
      goodMaskPub(),
      badMaskPub() {
  }
  /**
   *  @brief   This Callback function ... DOES SOMETHING
   *  @param   aImg An image with depth data
   *  @return  None
   */

  void pntCldCallback(const sensor_msgs::PointCloud2 &aPtCloud) {
    // Determine what to do here.
    // ROS_INFO_STREAM("Point Cloud Call back Successful.");

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
    // ROS_INFO_STREAM("RGB Image Call back Successful.");

    // Convert ROS Message into a cv::Mat for ImageProcessing Class.

    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(aImg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv_bridge::CvImage output, output2;
    // Allocate Memory for processing
    cv::Mat hsvImg, blurImg, goodThresh, badThresh;

    // Blur the Image
    int kernelSize = 3;
    cv::blur(cv_ptr->image, blurImg, cv::Size(kernelSize, kernelSize));

    output.header = aImg->header;
    output.encoding = sensor_msgs::image_encodings::MONO8;
    output.image = eyes.applyGoodMask(blurImg);

    output2.header = aImg->header;
    output2.encoding = sensor_msgs::image_encodings::MONO8;
    output2.image = eyes.applyBadMask(blurImg);


    goodMaskPub.publish(output.toImageMsg());
    badMaskPub.publish(output2.toImageMsg());

    // Convert the image to a shared pointer
    // May need to make a new pointer above to prevent the ros pointer from
    // going out of scope instead...
    std::shared_ptr<cv::Mat> pic(new cv::Mat(cv_ptr->image));
    // Consider using above than below
    // FIXME [Yhap] Using this ros' boost shared pointer will free
    // and then we would also try to free hence we use above
    // std::shared_ptr<const cv::Mat> pic(&cv_ptr->image);
    if (eyes.setRgbImg(pic)) {
      // ROS_INFO_STREAM("RGB Image set successful.");
    }
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
  // Mask for the Strawberry
  cv::Scalar lowGood(0, 80, 80);
  cv::Scalar highGood(50, 255, 255);
  // Mask for the Ghost
  cv::Scalar lowBad(80, 80, 80);
  cv::Scalar highBad(255, 255, 255);

  identifier.eyes.setGoodObjectMask(lowGood, highGood);
  identifier.eyes.setBadObjectMask(lowBad, highBad);
  // For this application we will use the pixel location instead.
  identifier.eyes.setPixelForPose(true);

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
  ros::Publisher objpub = nh.advertise<pacman::VecPoses>("imgPoses", 1000);

  identifier.goodMaskPub = imTrans.advertise("/goodObjMask/image_raw", 1);
  identifier.badMaskPub = imTrans.advertise("/badObjMask/image_raw", 1);


  // Publish at 10 Hz.
  ros::Rate loop_rate(10.0);

  while (ros::ok()) {
    // Process the Images and Extract Objects
    // ROS_INFO_STREAM("Making the process Call");
    std::vector<std::shared_ptr<Object> > tObjs = identifier.eyes.process();


    // ROS_INFO_STREAM("Call successful.");
    pacman::VecPoses output;

    // Parse Out Good Objects to Publish on a Topic
    // Update Nav Stack Map with Bad Objects.
    for (const auto &tObj : tObjs) {
        // Convert the Pose to the ROS Pose Msg Type
        pacman::ObjPose tPosMsg;
        Object::Pose toSend = tObj->getPose();
        tPosMsg.x = toSend.x;
        tPosMsg.y = toSend.y;
        tPosMsg.z = toSend.z;
        tPosMsg.theta = toSend.yaw;
        tPosMsg.collect = tObj->checkCollect();
        output.poses.emplace_back(tPosMsg);
    }
    // Publish the list of Good Objects.
    // Verify there is something to publish
    if (output.poses.size()) {
      objpub.publish(output);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
