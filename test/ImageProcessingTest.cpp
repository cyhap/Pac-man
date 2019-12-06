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
  * @file ImageProcessingTest.cpp
  * @copyright 2019 Corbyn Yhap
  * @author Corbyn Yhap
  * @date 11/27/2019
  * @brief This TEST file is for testing the methods in the Image Processing
  *        class
  */

#include "ImageProcessing.hpp"
#include "GoodObject.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"

#include "ros/ros.h"
#include "gtest/gtest.h"

TEST(ImageProcessing, setRgbImgFunction) {
  // Create an all White Image
  cv::Scalar tWhite(255, 255, 255);
  int tImgSize = 401;
  std::shared_ptr<cv::Mat> slate;
  slate = std::make_shared<cv::Mat>(
      cv::Mat(tImgSize, tImgSize, CV_8UC3, tWhite));

  // Create a "Depth" Image where everything but the block is 10m away
  float tDefaultDepth = 10;  // Meters
  std::shared_ptr<cv::Mat> depth;
  depth = std::make_shared<cv::Mat>(
      cv::Mat(tImgSize, tImgSize, CV_32FC1, tDefaultDepth));

  // Create an Image Processing instance.
  ImageProcessing imageProc;

  ASSERT_TRUE(imageProc.setRgbImg(slate));
  ASSERT_FALSE(imageProc.setRgbImg(depth));
}

TEST(ImageProcessing, setPntCldFunction) {
  // Create a Point Cloud with fake data.
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > cloud(
      new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data
  int tImgSize = 401;
  cloud->width = tImgSize;
  cloud->height = tImgSize;
  cloud->is_dense = false;
  cloud->points.resize(cloud->width * cloud->height);

  for (std::size_t i = 0; i < cloud->points.size(); ++i) {
    cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
    cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
    cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
  }

  // Create an Image Processing instance.
  ImageProcessing imageProc;

  ASSERT_TRUE(imageProc.setPntCld(cloud));
}

TEST(ImageProcessing, CenteredGreenBlock) {
  // Create the expected values.
  Object::Pose tExpected;
  tExpected.x = 10;
  tExpected.y = 15;
  tExpected.z = 20;

  // Create an all White Image
  cv::Scalar tWhite(255, 255, 255);
  int tImgSize = 401;
  std::shared_ptr<cv::Mat> slate;
  slate = std::make_shared<cv::Mat>(
      cv::Mat(tImgSize, tImgSize, CV_8UC3, tWhite));

  // Center of the Image is 200,200
  // Create a Rectangle Object to put at the center of our blank slate.
  int tHeight = 20;
  int tWidth = 20;
  // Subtract half the height and width from the image center to find the top
  // left corner of the rectangle.
  int tX = tImgSize / 2 - tWidth / 2;
  int tY = tImgSize / 2 - tHeight / 2;

  // Create the rectangle that will be placed in the image.
  cv::Rect2i block(tX, tY, tWidth, tHeight);

  // Place the Rectangle in the image.
  cv::Scalar tGreen(0, 255, 0);
  cv::rectangle(*slate, block, tGreen, CV_FILLED);
  // Slate will now have a centered green rectangle.

  // Create a Point Cloud with fake data.
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > cloud(
      new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data
  cloud->width = tImgSize;
  cloud->height = tImgSize;
  cloud->is_dense = false;
  cloud->points.resize(cloud->width * cloud->height);

  for (std::size_t i = 0; i < cloud->points.size(); ++i) {
    cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
    cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
    cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
  }

  // Update the data corresponding to the block to be the expected Point.
  for (int tXs = tX - tWidth / 2; tXs < tX + tWidth / 2; tXs++) {
    for (int tYs = tY - tHeight / 2; tYs < tY + tHeight / 2; tYs++) {
      pcl::PointXYZ &tPnt = cloud->at(tXs, tYs);
      tPnt.x = tExpected.x;
      tPnt.y = tExpected.y;
      tPnt.z = tExpected.z;
    }
  }

  // Create an Image Processing instance.
  ImageProcessing imageProc;
  // Pass this information to the Image Processing Function.
  imageProc.setPntCld(cloud);
  imageProc.setRgbImg(slate);
  std::vector<std::shared_ptr<Object> > tObjects = imageProc.process();
  ASSERT_EQ(tObjects.size(), 1);
  std::shared_ptr<Object> tObj = *tObjects.begin();

  Object::Pose tCompare = tObj->getPose();
  ASSERT_EQ(tCompare.x, tExpected.x);
  ASSERT_EQ(tCompare.y, tExpected.y);
  ASSERT_EQ(tCompare.z, tExpected.z);
  ASSERT_EQ(tCompare.roll, tExpected.roll);
  ASSERT_EQ(tCompare.pitch, tExpected.pitch);
  ASSERT_EQ(tCompare.yaw, tExpected.yaw);
}
/*
TEST(ImageProcessing, MultipleGreenBlocks) {
  // Create an all White Image
  cv::Scalar tWhite(255, 255, 255);
  int tImgSize = 401;
  cv::Mat slate(tImgSize, tImgSize, CV_8UC3, tWhite);
  // Center of the Image is 200,200
  // Create a Rectangle Object to put at the center of our blank slate.
  int tHeight = 20;
  int tWidth = 20;
  // Subtract half the height and width from the image center to find the top
  // left corner of the rectangle.
  int tX = tImgSize / 2 - tWidth / 2;
  int tY = tImgSize / 2 - tHeight / 2;

  // Create the rectangle that will be placed in the image.
  cv::Rect2i block(tX, tY, tWidth, tHeight);

  // Place the Rectangle in the image.
  cv::Scalar tGreen(0, 255, 0);
  cv::rectangle(slate, block, tGreen, CV_FILLED);
  // Slate will now have a centered green rectangle.

  // Add an additional Rectangle in theimage.
  cv::Rect2i block2(0, 0, tWidth, tHeight);
  cv::rectangle(slate, block2, tGreen, CV_FILLED);

  // Create a "Depth" Image where everything but the block is 10m away
  // Note Depth is less relevant for this test.
  float tDefaultDepth = 10;  // Meters
  cv::Mat depth(tImgSize, tImgSize, CV_32FC1, tDefaultDepth);

  // Create an Image Processing instance.
  ImageProcessing imageProc;
  // Pass this information to the Image Processing Function.
  imageProc.setDptImg(depth);
  imageProc.setRgbImg(slate);
  std::vector<std::shared_ptr<Object> > tObjects = imageProc.process();
  ASSERT_EQ(tObjects.size(), 2);
}

TEST(ImageProcessing, MultipleMultiColoredBlocks) {
  // Create an all White Image
  cv::Scalar tWhite(255, 255, 255);
  int tImgSize = 401;
  cv::Mat slate(tImgSize, tImgSize, CV_8UC3, tWhite);
  // Center of the Image is 200,200
  // Create a Rectangle Object to put at the center of our blank slate.
  int tHeight = 20;
  int tWidth = 20;
  // Subtract half the height and width from the image center to find the top
  // left corner of the rectangle.
  int tX = tImgSize / 2 - tWidth / 2;
  int tY = tImgSize / 2 - tHeight / 2;

  // Create the rectangle that will be placed in the image.
  cv::Rect2i block(tX, tY, tWidth, tHeight);

  // Place the Rectangle in the image.
  cv::Scalar tGreen(0, 255, 0);
  cv::rectangle(slate, block, tGreen, CV_FILLED);
  // Slate will now have a centered green rectangle.

  // Add an additional Rectangle in the image.
  cv::Rect2i block2(0, 0, tWidth, tHeight);
  cv::rectangle(slate, block2, tGreen, CV_FILLED);

  // Add an addition Red rectangle in the image
  cv::Rect2i block3(100, 0, tWidth, tHeight);
  cv::Scalar tRed(0, 0, 255);
  cv::rectangle(slate, block3, tRed, CV_FILLED);

  // Create a "Depth" Image where everything but the block is 10m away
  // Note Depth is less relevant for this test.
  float tDefaultDepth = 10;  // Meters
  cv::Mat depth(tImgSize, tImgSize, CV_32FC1, tDefaultDepth);

  // Create an Image Processing instance.
  ImageProcessing imageProc;
  // Pass this information to the Image Processing Function.
  imageProc.setDptImg(depth);
  imageProc.setRgbImg(slate);
  std::vector<std::shared_ptr<Object> > tObjects = imageProc.process();
  ASSERT_EQ(tObjects.size(), 3);
}
 */
// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "ImageProcessing_test");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
