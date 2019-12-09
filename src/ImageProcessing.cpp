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
#include <math.h>

#include <memory>

#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include "ImageProcessing.hpp"
#include "GoodObject.hpp"
#include "BadObject.hpp"

ImageProcessing::ImageProcessing()
    :
    rgbImg(
        new cv::Mat(500, 500, CV_8UC3, cv::Scalar(255, 255, 255))),
    rectPntCld(),
    lowGood(80, 80, 80),
    lowBad(0, 80, 80),
    highGood(135, 255, 0),
    highBad(150, 255, 255),
    pixelForPose(false) {
}

ImageProcessing::~ImageProcessing() {
}

std::vector<std::shared_ptr<Object> > ImageProcessing::process() {
  // Create the return variable.
  std::vector<std::shared_ptr<Object> > tReturn;

  cv::Mat blurImg, goodThresh, badThresh;

  // Blur the Image
  int kernelSize = 3;
  cv::blur(*rgbImg, blurImg, cv::Size(kernelSize, kernelSize));

  goodThresh = applyGoodMask(blurImg);
  badThresh = applyBadMask(blurImg);

  // Only perform processing if there is data to process.
  // The pixel data replaces the pointcloud requriement.
  // Otherwise return an empty list.
  if (rgbImg && (rectPntCld || pixelForPose)) {
    // Retrieve Poses for Good Objects
    std::vector<Object::Pose> goodPoses = processMask(goodThresh);
    // Retrieve Poses for Bad Objects
    std::vector<Object::Pose> badPoses = processMask(badThresh);

    // Create Good Objects with the goodPoses
    for (const auto &tPose : goodPoses) {
      std::shared_ptr<Object> tAdd(new GoodObject(tPose));
      tReturn.emplace_back(tAdd);
    }
    // Create Bad Objects with the badPoses
    for (const auto &tPose : badPoses) {
      std::shared_ptr<Object> tAdd(new BadObject(tPose));
      tReturn.emplace_back(tAdd);
    }
  }

  return tReturn;
}
cv::Mat ImageProcessing::applyGoodMask(const cv::Mat &aImg) {
  // Allocate Memory for processing
  cv::Mat hsvImg, goodThresh;

  // Transform the colors into HSV
  cv::cvtColor(aImg, hsvImg, CV_BGR2HSV);

  cv::inRange(hsvImg, lowGood, highGood, goodThresh);
  return goodThresh;
}

cv::Mat ImageProcessing::applyBadMask(const cv::Mat &aImg) {
  // Allocate Memory for processing
  cv::Mat hsvImg, badThresh;

  // Transform the colors into HSV
  cv::cvtColor(aImg, hsvImg, CV_BGR2HSV);

  cv::inRange(hsvImg, lowBad, highBad, badThresh);
  return badThresh;
}

std::vector<Object::Pose> ImageProcessing::processMask(
    const cv::Mat &aImage) {
  // Declare the return type
  std::vector<Object::Pose> tReturn;

  // Find the contours in the image
  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(aImage, contours, hierarchy, cv::RETR_TREE,
                   cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
  // Extract the moments from the contours
  std::vector<cv::Moments> mu(contours.size());
  auto momentIter = mu.begin();
  for (const auto &tContour : contours) {
    *momentIter = cv::moments(tContour, false);
    momentIter++;
  }

  std::vector<cv::Point2i> pixels;
  // Extract the Centroid from the Moments
  for (const auto &tMoment : mu) {
    // Ignore m00 == 0 Moments (Cant compute center)
    if (tMoment.m00 > 1200) {
      pixels.emplace_back(
          cv::Point2i(tMoment.m10 / tMoment.m00, tMoment.m01 / tMoment.m00));
    }
  }

  // Obtain the corresponding XYZ Point
  for (const auto &tPixel : pixels) {
    Object::Pose tPose;
    // Use the pixel location instead of the point cloud.
    if (pixelForPose) {
      tPose.x = tPixel.x;
      tPose.y = tPixel.y;
      // Add that Pose to the list of poses.
      tReturn.push_back(tPose);
    } else {
      tPose = extractPose(tPixel.x, tPixel.y);
      // Check whether the depth sensor got an accurate (Non-NAN) reading.
      if (!std::isnan(tPose.x)) {
        // Add that Pose to the list of poses.
        tReturn.push_back(tPose);
      }
    }
  }
  return tReturn;
}

bool ImageProcessing::setRgbImg(std::shared_ptr<const cv::Mat> aRgbImg) {
  bool validRGB = false;
  if (aRgbImg && aRgbImg->type() == CV_8UC3) {
    validRGB = true;
    rgbImg = aRgbImg;
  }
  return validRGB;
}
bool ImageProcessing::setPntCld(
    std::shared_ptr<const pcl::PointCloud<pcl::PointXYZ> > aPntCld) {
  bool validDpt = false;
  // Check the Point cloud has no Nans And is Organized
  if (aPntCld && aPntCld->height > 1) {
    validDpt = true;
    rectPntCld = aPntCld;
  }
  return validDpt;
}
void ImageProcessing::setGoodObjectMask(const cv::Scalar &aLow,
                                        const cv::Scalar &aHigh) {
  lowGood = aLow;
  highGood = aHigh;
}

void ImageProcessing::setBadObjectMask(const cv::Scalar &aLow,
                                       const cv::Scalar &aHigh) {
  lowBad = aLow;
  highBad = aHigh;
}
void ImageProcessing::setPixelForPose(bool aUsePixel) {
  pixelForPose = aUsePixel;
}

Object::Pose ImageProcessing::extractPose(int x, int y) {
  Object::Pose tReturn;
  if (rectPntCld) {
    pcl::PointXYZ tPnt = rectPntCld->at(x, y);

    tReturn.x = tPnt.x;
    tReturn.y = tPnt.y;
    tReturn.z = tPnt.z;

  } else {
    // This should never happen.
  }
  return tReturn;
}
