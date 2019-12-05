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
  * @file ImageProcessing.cpp
  * @copyright 2019 Corbyn Yhap
  * @author Corbyn Yhap
  * @date 11/27/2019
  * @brief This Class defines the functions that process RGB and Depth sensor
  *        data together.
 */

#include "ImageProcessing.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

#include "GoodObject.hpp"
#include "BadObject.hpp"

ImageProcessing::ImageProcessing()
    :
    rgbImg(),
    rectDepthImg() {
  // Fixme [Yhap] Consider what default images should be used if any at all.
}

ImageProcessing::~ImageProcessing() {
}

std::vector<std::shared_ptr<Object> > ImageProcessing::process() {
  // Create the return variable.
  std::vector<std::shared_ptr<Object> > tReturn;

  // Define the  BGR Masks used to detect blocks
  // Green Block
  cv::Scalar lowGreen(0, 200, 0);
  cv::Scalar highGreen(0, 255, 0);
  // Red Block
  cv::Scalar lowRed(0, 0, 200);
  cv::Scalar highRed(0, 0, 255);

  cv::Mat blurImg, greenThresh, redThresh;

  // Blur the Image
  int kernelSize = 3;
  cv::blur(rgbImg, blurImg, cv::Size(kernelSize, kernelSize));

  cv::inRange(blurImg, lowGreen, highGreen, greenThresh);
  cv::inRange(blurImg, lowRed, highRed, redThresh);

  // Retrieve Poses for Good Objects
  std::vector<Object::Pose> goodPoses = processMask(greenThresh);
  // Retrieve Poses for Bad Objects
  std::vector<Object::Pose> badPoses = processMask(redThresh);

  // Create Good Objects with the goodPoses
  for (const auto &tPose : goodPoses) {
    std::shared_ptr<Object> tAdd(new GoodObject(tPose));
    tReturn.push_back(tAdd);
  }
  // Create Bad Objects with the badPoses
  for (const auto &tPose : badPoses) {
    std::shared_ptr<Object> tAdd(new BadObject(tPose));
    tReturn.push_back(tAdd);
  }

  return tReturn;
}

std::vector<Object::Pose> ImageProcessing::processMask(
    const cv::Mat &aImage) {
  // Declare the return type
  std::vector<Object::Pose> tReturn;
  // Find the contours in the image
  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(aImage, contours, hierarchy, cv::RETR_TREE,
                   cv::CHAIN_APPROX_SIMPLE);
  // Extract the moments from the contours
  std::vector<cv::Moments> mu(contours.size());
  auto momentIter = mu.begin();
  for (const auto &tContour : contours) {
    *momentIter = cv::moments(tContour, false);
    momentIter++;
  }

  std::vector<cv::Point2i> pixels(mu.size());
  auto pixelIter = pixels.begin();
  // Extract the Centroid from the Moments
  for (const auto &tMoment : mu) {
    *pixelIter = cv::Point2i(tMoment.m10 / tMoment.m00,
                             tMoment.m01 / tMoment.m00);
    pixelIter++;
  }

  // Obtain the corresponding depth at these points
  for (const auto &tPixel : pixels) {
    Object::Pose tPose;
    tPose.x = rectDepthImg.at<float>(tPixel.x, tPixel.y);
    // Add that Pose to the list of poses.
    tReturn.push_back(tPose);
  }
  return tReturn;
}
#include <iostream>

bool ImageProcessing::setRgbImg(const cv::Mat &aRgbImg) {
  bool validRGB = false;
  if (aRgbImg.type() == CV_8UC3) {
    validRGB = true;
    rgbImg = aRgbImg;
  }
  return validRGB;
}
bool ImageProcessing::setDptImg(const cv::Mat &aDepthImg) {
  bool validDpt = false;
  if (aDepthImg.type() == CV_32FC1) {
    validDpt = true;
    rectDepthImg = aDepthImg;
  }
  return validDpt;
}
