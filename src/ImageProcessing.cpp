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

#include <memory>

#include "ImageProcessing.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include "GoodObject.hpp"
#include "BadObject.hpp"

#include <iostream> //REMOVE

ImageProcessing::ImageProcessing()
    :
    rgbImg(
        new cv::Mat(500, 500, CV_8UC3, cv::Scalar(255, 255, 255))),
    rectPntCld(),
    lowGood(0, 200, 0),
    lowBad(0, 0, 200),
    highGood(0, 255, 0),
    highBad(0, 0, 255) {
}

ImageProcessing::~ImageProcessing() {
}

std::vector<std::shared_ptr<Object> > ImageProcessing::process() {
  // Create the return variable.
  std::vector<std::shared_ptr<Object> > tReturn;

  // Only perform processing if there is data to process.
  // Otherwise return an empty list.
  if (rgbImg && rectPntCld) {

    if (rgbImg) {
      std::cout << "Still Valid Here 1" << std::endl;
    }

    // Allocate Memory for processing
    cv::Mat blurImg(rgbImg->rows, rgbImg->cols, rgbImg->type());
    cv::Mat goodThresh(rgbImg->rows, rgbImg->cols, rgbImg->type());
    cv::Mat badThresh(rgbImg->rows, rgbImg->cols, rgbImg->type());

    if (rgbImg) {
      std::cout << "Still Valid Here 2" << std::endl;
    }
    // Blur the Image
    int kernelSize = 3;
    if (rgbImg) {
      std::cout << "Still Valid Here 3" << std::endl;
    }
    std::cout << "Begin Processing..." << std::endl;
    cv::blur(*rgbImg, blurImg, cv::Size(kernelSize, kernelSize));
    std::cout << "First Pointer Used" << std::endl;

    cv::inRange(blurImg, lowGood, highGood, goodThresh);
    cv::inRange(blurImg, lowBad, highBad, badThresh);

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
  std::cout << "Begin Processing Moments." << std::endl;

  std::vector<cv::Moments> mu(contours.size());
  auto momentIter = mu.begin();
  for (const auto &tContour : contours) {
    *momentIter = cv::moments(tContour, false);
    momentIter++;
    std::cout << "Inc Moment" << std::endl;
  }
  std::cout << "Begin Processing Centroids." << std::endl;

  std::vector<cv::Point2i> pixels;
  // Extract the Centroid from the Moments
  for (const auto &tMoment : mu) {
    std::cout << "Moment 10: " << tMoment.m10 << std::endl;
    std::cout << "Moment 00: " << tMoment.m00 << std::endl;
    std::cout << "Moment 01: " << tMoment.m01 << std::endl;
    // Ignore m00 == 0 Moments (Cant compute center)
    if (tMoment.m00) {
      pixels.emplace_back(
          cv::Point2i(tMoment.m10 / tMoment.m00, tMoment.m01 / tMoment.m00));
      std::cout << "Inc Pixel" << std::endl;
    }
  }
  std::cout << "Begin Extracting Poses." << std::endl;
  // Obtain the corresponding XYZ Point
  for (const auto &tPixel : pixels) {
    Object::Pose tPose = extractPose(tPixel.x, tPixel.y);

    // Add that Pose to the list of poses.
    tReturn.push_back(tPose);
  }
  std::cout << "Complete Mask Processing." << std::endl;
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
  else {

    std::cout << "Setting of Point Cloud was attempted but failed."
              << std::endl;
    if (aPntCld->height <= 1) {
      std::cout << "Unorganized Point Cloud" << std::endl;
    }
    if (!aPntCld->is_dense) {
      std::cout << "Non Dense Point Cloud" << std::endl;
    }
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

Object::Pose ImageProcessing::extractPose(int x, int y) {
  Object::Pose tReturn;
  if (rectPntCld) {
    if (rectPntCld->height <= 1) {
      std::cout << "Unorganized Point Cloud" << std::endl;
    }
    if (!rectPntCld->is_dense) {
      std::cout << "Non Dense Point Cloud" << std::endl;
    }

    std::cout << "The X,Y Chosen: (" << x << ", " << y << ")" << std::endl;
    pcl::PointXYZ tPnt = rectPntCld->at(x, y);

    tReturn.x = tPnt.x;
    tReturn.y = tPnt.y;
    tReturn.z = tPnt.z;

  } else {
    // This should never happen.
  }
  return tReturn;
}
