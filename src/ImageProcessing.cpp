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

#include "GoodObject.hpp"

ImageProcessing::ImageProcessing()
    :
    rgbImg(),
    rectDepthImg() {
  // Fixme [Yhap] Consider what default images should be used if any at all.
}

ImageProcessing::~ImageProcessing() {
}

std::vector<std::shared_ptr<Object> > ImageProcessing::process() {
  // Fixme[Yhap] Fill in this function;
  /*
  // Define the  BGR Masks used to detect blocks
  // Green Block
  cv::Scalar lowGreen(0, 200, 0);
  cv::Scalar highGreen(0, 255, 0);
  // Red Block
  cv::Scalar lowRed(0, 0, 200);
  cv::Scalar highRed(0, 0, 255);

  cv::Mat greenThresh, redThresh;

  cv::inRange(rgbImg, lowGreen, highGreen, greenThresh);
  cv::inRange(rgbImg, lowRed, highRed, redThresh);
   */
  std::vector<std::shared_ptr<Object> > tReturn;
  return tReturn;
}

std::vector<Object::Pose> ImageProcessing::processMask(
    const cv::Mat &aImage) {
  // Image Processing taking two different inputs since images updated separately
  // updating the object creating code portion.

  (void) aImage;
  std::vector<Object::Pose> tReturn;
  return tReturn;
}

bool ImageProcessing::setRgbImg(const cv::Mat &aRgbImg) {
  (void) aRgbImg;
  return false;
  //rgbImg = aRgbImg;
}
bool ImageProcessing::setDptImg(const cv::Mat &aDepthImg) {
  (void) aDepthImg;
  //rectDepthImg = aDepthImg;
  return false;
}
